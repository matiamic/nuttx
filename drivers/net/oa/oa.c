/****************************************************************************
 * drivers/net/oa.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <sys/endian.h>

#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>

#include <nuttx/net/netdev_lowerhalf.h>

#include <nuttx/net/oa.h>

#ifdef CONFIG_NET_OA_NCV7410
#include "oa_ncv7410.h"
#endif

#ifdef CONFIG_NET_OA_NCN26010
#include "oa_ncn26010.h"
#endif

#ifdef CONFIG_NET_OA_LAN8650
#include "oa_lan8650.h"
#endif

#include "oa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OAWORK LPWORK

#define OA_RESET_TRIES 5

/* Maximum frame size = (MTU + LL heaader size) + FCS size */

#define OA_MAX_FRAME_SIZE(p) (p->dev.netdev.d_pktsize + 4)

/* Packet Memory ************************************************************/

/* Maximum number of allocated tx and rx packets */

#define OA_TX_QUOTA        1
#define OA_RX_QUOTA        2

#if CONFIG_IOB_NBUFFERS < (OA_TX_QUOTA + OA_RX_QUOTA)
#  error "CONFIG_IOB_NBUFFERS must be > (OA_TX_QUOTA + OA_RX_QUOTA)"
#endif

#ifndef CONFIG_SCHED_LPWORK
#  error "CONFIG_SCHED_LPWORK is needed by OA driver"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Bit calculations */

static int oa_get_parity(uint32_t word);

/* SPI transfers */

static int oa_poll_footer(FAR struct oa_driver_s *priv,
                          FAR uint32_t *footer);

static int oa_exchange_chunk(FAR struct oa_driver_s *priv,
                             FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                             uint32_t header, uint32_t *footer);

/* Interrupt handling */

static int oa_interrupt(int irq, FAR void *context, FAR void *arg);
static void oa_interrupt_work(FAR void *arg);

/* Data Transaction Protocol logic */

static void oa_io_work(FAR void *arg);
static uint32_t oa_prepare_chunk_exchange(FAR struct oa_driver_s *priv,
                                          FAR uint8_t *txbuf);
static bool oa_can_rx(FAR struct oa_driver_s *priv);
static void oa_try_finish_tx_packet(FAR struct oa_driver_s *priv);
static void oa_handle_rx_chunk(FAR struct oa_driver_s *priv,
                               uint32_t footer, FAR uint8_t *rxbuf);
static void oa_finalize_rx_packet(FAR struct oa_driver_s *priv);
static void oa_release_tx_packet(FAR struct oa_driver_s *priv);
static void oa_release_rx_packet(FAR struct oa_driver_s *priv);

/* SPI inline utility functions */

static inline void oa_select_spi(FAR struct oa_driver_s *priv);
static inline void oa_deselect_spi(FAR struct oa_driver_s *priv);

/* OA reset and configuration */

static int oa_reset(FAR struct oa_driver_s *priv);
static int oa_config(FAR struct oa_driver_s *priv);
static int oa_enable(FAR struct oa_driver_s *priv);
static int oa_disable(FAR struct oa_driver_s *priv);

static int oa_get_device_type(FAR struct oa_driver_s *priv,
                              FAR uint32_t *device_type);

/* Driver buffer manipulation */

static void oa_reset_driver_buffers(FAR struct oa_driver_s *priv);

/* NuttX callback functions */

static int oa_ifup(FAR struct netdev_lowerhalf_s *dev);
static int oa_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int oa_transmit(FAR struct netdev_lowerhalf_s *dev,
                       FAR netpkt_t *pkt);
static FAR netpkt_t *oa_receive(FAR struct netdev_lowerhalf_s *dev);

/* Debug */

#ifdef CONFIG_DEBUG_NET_INFO
static void oa_print_footer(uint32_t footer);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_oa_ops =
{
  .ifup     = oa_ifup,
  .ifdown   = oa_ifdown,
  .transmit = oa_transmit,
  .receive  = oa_receive,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oa_interrupt
 *
 * Description:
 *   Schedule interrupt work when the interrupt signal from MAC-PHY is
 *   received.
 *
 * Input Parameters:
 *   irq     - not used
 *   context - not used
 *   arg     - oa_driver_s priv structure to be passed to the interrupt
 *             worker
 *
 * Returned Value:
 *   OK is always returned.
 *
 ****************************************************************************/

static int oa_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct oa_driver_s *priv = (FAR struct oa_driver_s *)arg;

  ninfo("OA interrupt!\n");

  /* Schedule interrupt work */

  work_queue(OAWORK, &priv->interrupt_work, oa_interrupt_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: oa_interrupt_work
 *
 * Description:
 *   Identify the interrupt source and perform necessary work.
 *
 * Input Parameters:
 *   arg - pointer to driver private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_interrupt_work(FAR void *arg)
{
  FAR struct oa_driver_s *priv = (FAR struct oa_driver_s *)arg;
  uint32_t footer;

  nxmutex_lock(&priv->lock);

  if (priv->ifstate != OA_IFSTATE_INIT_UP)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  ninfo("OA interrupt worker invoked!\n");

  /* Poll the data chunk footer */

  if (oa_poll_footer(priv, &footer))
    {
      nerr("Polling footer unsuccessful\n");

      /* TODO: don't */

      PANIC();
    }

#ifdef CONFIG_DEBUG_NET_INFO
  oa_print_footer(footer);
#endif

  /* If EXST in the footer, check enabled sources
   * STATUS0, link-status in clause 22 phy registers
   * (not yet implemented)
   */

  /* Update MAC-PHY buffer status */

  priv->txc = oa_tx_credits(footer);
  priv->rca = oa_rx_available(footer);

  if ((priv->tx_pkt && priv->txc) || priv->rca)
    {
      /* Schedule IO work */

      work_queue(OAWORK, &priv->io_work, oa_io_work, priv, 0);
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: oa_io_work
 *
 * Description:
 *   Exchange data chunk with the MAC-PHY.
 *
 * Input Parameters:
 *   arg - pointer to driver private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_io_work(FAR void *arg)
{
  FAR struct oa_driver_s *priv = (FAR struct oa_driver_s *)arg;

  uint8_t txbuf[OA_CHUNK_DEFAULT_PAYLOAD_SIZE];
  uint8_t rxbuf[OA_CHUNK_DEFAULT_PAYLOAD_SIZE];

  uint32_t header;
  uint32_t footer;

  nxmutex_lock(&priv->lock);

  if (priv->ifstate != OA_IFSTATE_INIT_UP)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  header = oa_prepare_chunk_exchange(priv, txbuf);

  /* Perform the SPI exchange */

  if (oa_exchange_chunk(priv, txbuf, rxbuf, header, &footer))
    {
      nerr("Error during chunk exchange\n");

      /* TODO: do not panic, the best is probably to report the error
       * and reset MAC to some defined state and reset driver
       */

      PANIC();
    }

  oa_try_finish_tx_packet(priv);

  oa_handle_rx_chunk(priv, footer, rxbuf);

  /* Schedule further work if needed */

  if ((priv->tx_pkt && priv->txc) || priv->rca)
    {
      work_queue(OAWORK, &priv->io_work, oa_io_work, priv, 0);
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: oa_prepare_chunk_exchange
 *
 * Description:
 *   Determine whether there is data to transmit or receive.
 *   Set the appropriate header bitfields and fill the txbuf accordingly.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   txbuf - pointer to the transmit chunk buffer
 *
 * Returned Value:
 *   Returns the prepared chunk header.
 *
 ****************************************************************************/

static uint32_t oa_prepare_chunk_exchange(FAR struct oa_driver_s *priv,
                                          FAR uint8_t *txbuf)
{
  uint32_t header = 0;
  int txlen;

  if (priv->tx_pkt && priv->txc)
    {
      header |= (1 << OA_DV_POS);  /* Data Valid */

      if (priv->tx_pkt_idx == 0)
        {
          header |=   (1 << OA_SV_POS)   /* Start Valid           */
                    | (0 << OA_SWO_POS); /* Start Word Offset = 0 */
        }

      txlen = priv->tx_pkt_len - priv->tx_pkt_idx;

      if (txlen <= OA_CHUNK_DEFAULT_PAYLOAD_SIZE)
        {
          header |=   (1 << OA_EV_POS)             /* End Valid       */
                    | ((txlen - 1) << OA_EBO_POS); /* End Byte Offset */
        }
      else
        {
          txlen = OA_CHUNK_DEFAULT_PAYLOAD_SIZE;
        }

      /* Copy data from network to txbuf */

      netpkt_copyout(&priv->dev, txbuf, priv->tx_pkt,
                     txlen, priv->tx_pkt_idx);
      priv->tx_pkt_idx += txlen;
    }

  if (oa_can_rx(priv) == false)
    {
      header |= (1 << OA_NORX_POS);  /* No RX */
    }

  return header;
}

/****************************************************************************
 * Name: oa_can_rx
 *
 * Description:
 *   Determine whether rx data is available and whether it can be received.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   If it is possible to receive an rx chunk, true is returned,
 *   otherwise false is returned.
 *
 ****************************************************************************/

static bool oa_can_rx(FAR struct oa_driver_s *priv)
{
  if (!priv->rca)
    {
      return false;
    }

  if (priv->rx_pkt_ready)
    {
      return false;
    }

  if (priv->rx_pkt)
    {
      return true;
    }

  /* No RX packet, try to alloc */

  priv->rx_pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (priv->rx_pkt)
    {
      return true;
    }

  ninfo("INFO: Failed to alloc rx netpkt\n");

  /* There is no buffer for RX data */

  return false;
}

/****************************************************************************
 * Name: oa_try_finish_tx_packet
 *
 * Description:
 *   Check whether the entire packet has been transmitted.
 *   If so, free the tx netpkt and notify the upperhalf.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_try_finish_tx_packet(FAR struct oa_driver_s *priv)
{
  if (priv->tx_pkt && (priv->tx_pkt_idx == priv->tx_pkt_len))
    {
      oa_release_tx_packet(priv);
      netdev_lower_txdone(&priv->dev);
    }
}

/****************************************************************************
 * Name: oa_handle_rx_chunk
 *
 * Description:
 *   Parse the received footer, update buffer status and handle data
 *   in the rxbuf.
 *
 * Input Parameters:
 *   priv   - pointer to the driver-specific state structure
 *   footer - the received footer
 *   rxbuf  - pointer to the received data buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_handle_rx_chunk(FAR struct oa_driver_s *priv,
                               uint32_t footer, FAR uint8_t *rxbuf)
{
  int rxlen;
  int newlen;

  /* Update buffer status */

  priv->txc = oa_tx_credits(footer);
  priv->rca = oa_rx_available(footer);

  /* Check rx_pkt && !rx_pkt_ready,
   * oa_data_valid flag might have been set due to an SPI error
   */

  if (oa_data_valid(footer) && priv->rx_pkt && !priv->rx_pkt_ready)
    {
      if (oa_start_valid(footer))
        {
          priv->rx_pkt_idx = 0;
        }

      if (oa_end_valid(footer))
        {
          if (oa_frame_drop(footer))
            {
              oa_release_rx_packet(priv);
              return;
            }

          rxlen = oa_end_byte_offset(footer) + 1;
        }
      else
        {
          rxlen = OA_CHUNK_DEFAULT_PAYLOAD_SIZE;
        }

      newlen = priv->rx_pkt_idx + rxlen;

      if (newlen > OA_MAX_FRAME_SIZE(priv))
        {
          nwarn("Dropping chunk of a packet that is too long");

          /* set index so that a subsequent chunk with
           * smaller payload won't pass
           */

          priv->rx_pkt_idx = OA_MAX_FRAME_SIZE(priv) + 1;
          return;
        }

      netpkt_copyin(&priv->dev, priv->rx_pkt, rxbuf,
                    rxlen, priv->rx_pkt_idx);
      priv->rx_pkt_idx = newlen;

      if (oa_end_valid(footer))
        {
          /* finalize packet and notify the upper */

          oa_finalize_rx_packet(priv);
          netdev_lower_rxready(&priv->dev);
        }
    }
}

/****************************************************************************
 * Name: oa_finalize_rx_packet
 *
 * Description:
 *   Strip down last 4 bytes (FCS) from the rx packet and mark it ready.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_finalize_rx_packet(FAR struct oa_driver_s *priv)
{
  netpkt_setdatalen(&priv->dev, priv->rx_pkt,
                    netpkt_getdatalen(&priv->dev, priv->rx_pkt) - 4);
  priv->rx_pkt_ready = true;
}

/****************************************************************************
 * Name: oa_release_tx_packet
 *
 * Description:
 *   Release the tx packet.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_release_tx_packet(FAR struct oa_driver_s *priv)
{
  netpkt_free(&priv->dev, priv->tx_pkt, NETPKT_TX);
  priv->tx_pkt = NULL;
}

/****************************************************************************
 * Name: oa_release_rx_packet
 *
 * Description:
 *   Release the rx packet.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_release_rx_packet(FAR struct oa_driver_s *priv)
{
  netpkt_free(&priv->dev, priv->rx_pkt, NETPKT_RX);
  priv->rx_pkt = NULL;
}

/****************************************************************************
 * Name: oa_get_parity
 *
 * Description:
 *   Obtain parity of a 32-bit word.
 *
 * Input Parameters:
 *   word - 32-bit word, subject to the parity calculation
 *
 * Returned Value:
 *   If the parity of the word is even, zero is returned.
 *   Otherwise one is returned.
 *
 ****************************************************************************/

static int oa_get_parity(uint32_t word)
{
  /* www-graphics.stanford.edu/~seander/bithacks.html */

  word ^= word >> 1;
  word ^= word >> 2;
  word = (word & 0x11111111u) * 0x11111111u;
  return (word >> 28) & 1;
}

/****************************************************************************
 * Name: oa_(select/deselect)_spi
 *
 * Description:
 *   Helper functions to setup SPI hardware.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void oa_select_spi(FAR struct oa_driver_s *priv)
{
  SPI_LOCK(priv->spi, true);

  SPI_SETMODE(priv->spi, OA_SPI_MODE);
  SPI_SETBITS(priv->spi, OA_SPI_NBITS);
  SPI_HWFEATURES(priv->spi, 0);  /* disable HW features */
  SPI_SETFREQUENCY(priv->spi, priv->config->frequency);

  SPI_SELECT(priv->spi, priv->config->id, true);
}

static inline void oa_deselect_spi(FAR struct oa_driver_s *priv)
{
  SPI_SELECT(priv->spi, priv->config->id, false);

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: oa_exchange_chunk
 *
 * Description:
 *   Send a data chunk to MAC-PHY and simultaneously receive chunk.
 *
 *   Computing header parity, checking footer parity, converting to proper
 *   endianness and setting DNC flag is done by this function.
 *
 * Input Parameters:
 *   priv   - pointer to the driver-specific state structure
 *   txbuf  - buffer with transmit chunk data
 *   rxbuf  - buffer to save the received chunk to
 *   header - header controlling the transaction
 *   footer - pointer to a 32-bit value for the footer
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_exchange_chunk(FAR struct oa_driver_s *priv,
                             FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                             uint32_t header, uint32_t *footer)
{
  header |= (1 << OA_DNC_POS);
  header |= (!oa_get_parity(header) << OA_P_POS);
  header = htobe32(header);

  oa_select_spi(priv);

  /* This depends on SW Chip Select */

  SPI_EXCHANGE(priv->spi, (uint8_t *)&header, rxbuf, 4);
  SPI_EXCHANGE(priv->spi, txbuf,
               &rxbuf[4], OA_CHUNK_DEFAULT_PAYLOAD_SIZE - 4);
  SPI_EXCHANGE(priv->spi, &txbuf[OA_CHUNK_DEFAULT_PAYLOAD_SIZE - 4],
               (uint8_t *)footer, 4);
  oa_deselect_spi(priv);

  *footer = be32toh(*footer);
  if (!oa_get_parity(*footer))
    {
      nerr("Wrong parity in the footer\n");
      return ERROR;
    }

  if (oa_header_bad(*footer))
    {
      nerr("HDRB set in the footer\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_poll_footer
 *
 * Description:
 *   Poll a data transaction chunk footer.
 *
 * Input Parameters:
 *   priv   - pointer to the driver-specific state structure
 *   footer - pointer to a 32-bit footer destination variable
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_poll_footer(FAR struct oa_driver_s *priv,
                           FAR uint32_t *footer)
{
  uint8_t txdata[OA_CHUNK_DEFAULT_PAYLOAD_SIZE];
  uint8_t rxdata[OA_CHUNK_DEFAULT_PAYLOAD_SIZE];
  uint32_t header;

  header =   (1 << OA_DNC_POS)   /* Data Not Control */
           | (1 << OA_NORX_POS); /* No Read */

  if (oa_exchange_chunk(priv, txdata, rxdata, header, footer))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_reset
 *
 * Description:
 *   Perform SW reset of the MAC-PHY.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On a successful reset OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_reset(FAR struct oa_driver_s *priv)
{
  int tries = OA_RESET_TRIES;
  uint32_t regval = (1 << OA_RESET_SWRESET_POS);

  if (oa_write_reg(priv, OA_RESET_REGID, regval))
    {
      return ERROR;
    }

  /* Check whether the RESET bit cleared itself */

  do
    {
      if (oa_read_reg(priv, OA_RESET_REGID, &regval))
        {
          return ERROR;
        }
    }
  while (tries-- && (regval & OA_RESET_SWRESET_MASK));

  if (regval & OA_RESET_SWRESET_MASK)
    {
      return ERROR;
    }

  /* Check whether the reset complete flag is set */

  tries = OA_RESET_TRIES;

  do
    {
      if (oa_read_reg(priv, OA_STATUS0_REGID, &regval))
        {
          return ERROR;
        }
    }
  while (tries-- && !(regval & OA_STATUS0_RESETC_MASK));

  if (!(regval & OA_STATUS0_RESETC_MASK))
    {
      return ERROR;
    }

  /* Clear HDRE in STATUS0 (due to a bug in NCV7410) */
  // move this to device-specific

  if (oa_write_reg(priv, OA_STATUS0_REGID, (1 << OA_STATUS0_HDRE_POS)))
    {
      return ERROR;
    }

  /* Clear reset complete flag */

  if (oa_write_reg(priv, OA_STATUS0_REGID, (1 << OA_STATUS0_RESETC_POS)))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_config
 *
 * Description:
 *   Configure the MAC-PHY into promiscuous mode and set the SYNC flag.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 * Assumptions:
 *   The function is called after the MAC address is initialized.
 *
 ****************************************************************************/

static int oa_config(FAR struct oa_driver_s *priv)
{
  uint32_t regval;

  ninfo("Configuring OA\n");

  /* Enable RX buffer overflow interrupt */

  // questionable
  regval = OA_IMSK0_DEF & ~(1 << OA_IMSK0_RXBOEM_POS);

  if (oa_write_reg(priv, OA_IMSK0_REGID, regval))
    {
      return ERROR;
    }

  /* Setup SPI protocol and set SYNC flag */

  regval =   (1 << OA_CONFIG0_SYNC_POS)
           | (1 << OA_CONFIG0_CSARFE_POS)
           | (1 << OA_CONFIG0_ZARFE_POS)
           | (1 << OA_CONFIG0_RXCTE_POS)  /* A bit lower latency */
           | (3 << OA_CONFIG0_TXCTHRESH_POS)
           | (6 << OA_CONFIG0_CPS_POS);

  if (oa_write_reg(priv, OA_CONFIG0_REGID, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_enable
 *
 * Description:
 *   Enable TX and RX on the MAC-PHY.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_enable(FAR struct oa_driver_s *priv)
{
  /* Enable PHY */

  uint32_t setbits;

  ninfo("Enabling OA\n");

  /* Enable RX and TX in PHY */

  setbits = (1 << OA_PHY_CONTROL_LCTL_POS);

  if (oa_set_clear_bits(priv, OA_PHY_CONTROL_REGID, setbits, 0))
    {
      return ERROR;
    }

  /* Enable PHY interrupt */
  // questionable
  setbits = (1 << OA_IMSK0_PHYINTM_POS);

  if (oa_set_clear_bits(priv, OA_IMSK0_REGID, setbits, 0))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_disable
 *
 * Description:
 *   Disable TX and RX on the MAC-PHY.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_disable(FAR struct oa_driver_s *priv)
{
  /* Disable PHY */

  uint32_t clearbits;

  ninfo("Disabling OA\n");

  /* Disable PHY interrupt */

  clearbits = (1 << OA_IMSK0_PHYINTM_POS);

  if (oa_set_clear_bits(priv, OA_IMSK0_REGID, 0, clearbits))
    {
      return ERROR;
    }

  /* Disable RX and TX in PHY */

  clearbits = (1 << OA_PHY_CONTROL_LCTL_POS);

  if (oa_set_clear_bits(priv, OA_PHY_CONTROL_REGID, 0, clearbits))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_get_device_type
 *
 * Description:
 *   Read the device type from the PHYID register.
 *
 * Input Parameters:
 *   priv        - pointer to the driver-specific state structure
 *   device_type - pointer to the destination of the PHYID value
 *
 * Returned Value:
 *   On success OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

static int oa_get_device_type(FAR struct oa_driver_s *priv,
                              FAR uint32_t *device_type)
{
  return oa_read_reg(priv, OA_PHYID_REGID, device_type);
}

/****************************************************************************
 * Name: oa_reset_driver_buffers
 *
 * Description:
 *   If allocated, release both tx and rx netpackets and reset buffer status
 *   to the default.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void oa_reset_driver_buffers(FAR struct oa_driver_s *priv)
{
  priv->txc = 0;
  priv->rca = 0;

  if (priv->tx_pkt)
    {
      oa_release_tx_packet(priv);
    }

  if (priv->rx_pkt)
    {
      oa_release_rx_packet(priv);
    }

  priv->tx_pkt_idx = 0;
  priv->rx_pkt_idx = 0;
  priv->tx_pkt_len = 0;
  priv->rx_pkt_ready = false;
}

/****************************************************************************
 * Name: oa_print_footer
 *
 * Description:
 *   print individual bitfield of a receive chunk footer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_NET_INFO
static void oa_print_footer(uint32_t footer)
{
  ninfo("Footer:\n");
  ninfo("  EXST: %d\n", oa_ext_status(footer));
  ninfo("  HDRB: %d\n", oa_header_bad(footer));
  ninfo("  SYNC: %d\n", oa_mac_phy_sync(footer));
  ninfo("  RCA:  %d\n", oa_rx_available(footer));
  ninfo("  DV:   %d\n", oa_data_valid(footer));
  ninfo("  SV:   %d\n", oa_start_valid(footer));
  ninfo("  SWO:  %d\n", oa_start_word_offset(footer));
  ninfo("  FD:   %d\n", oa_frame_drop(footer));
  ninfo("  EV:   %d\n", oa_end_valid(footer));
  ninfo("  EBO:  %d\n", oa_end_byte_offset(footer));
  ninfo("  RTSA: %d\n", oa_rx_frame_timestamp_added(footer));
  ninfo("  RTSP: %d\n", oa_rx_frame_timestamp_parity(footer));
  ninfo("  TXC:  %d\n", oa_tx_credits(footer));
}
#endif

/****************************************************************************
 * Netdev upperhalf callbacks
 ****************************************************************************/

/****************************************************************************
 * Name: oa_ifup
 *
 * Description:
 *   NuttX callback: Bring up the Ethernet interface
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct oa_driver_s *priv = (FAR struct oa_driver_s *)dev;

  if (priv->ifstate == OA_IFSTATE_INIT_UP)
    {
      nerr("Tried to bring OA interface up when already up\n");
      return -EINVAL;
    }

  ninfo("Bringing up OA\n");

  if (priv->ifstate == OA_IFSTATE_RESET)
    {
      if (oa_config(priv) == ERROR)
        {
          nerr("Error configuring OA\n");
          return -EIO;
        }

      priv->ifstate = OA_IFSTATE_INIT_DOWN;
    }

  /* Set OA_IFSTATE_INIT_UP prior to enabling to allow oa_interrupt_work right
   * after MAC-PHY enable
   */

  priv->ifstate = OA_IFSTATE_INIT_UP;

  if (oa_enable(priv) == ERROR)
    {
      nerr("Error enabling OA\n");
      priv->ifstate = OA_IFSTATE_INIT_DOWN;
      return -EIO;
    }

  /* Schedule interrupt work to initialize txc and rca */

  work_queue(OAWORK, &priv->interrupt_work, oa_interrupt_work, priv, 0);

  return OK;
}

/****************************************************************************
 * Name: oa_ifdown
 *
 * Description:
 *   NuttX callback: Shut down the Ethernet interface.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct oa_driver_s *priv = (FAR struct oa_driver_s *)dev;

  nxmutex_lock(&priv->lock);

  if (priv->ifstate != OA_IFSTATE_INIT_UP)
    {
      nxmutex_unlock(&priv->lock);
      nerr("Tried to bring the OA interface down but it is not up\n");
      return -EINVAL;
    }

  work_cancel(OAWORK, &priv->interrupt_work);
  work_cancel(OAWORK, &priv->io_work);

  if (oa_disable(priv) == ERROR)
    {
      nxmutex_unlock(&priv->lock);
      nerr("Error disabling OA\n");
      return -EIO;
    }

  oa_reset_driver_buffers(priv);

  priv->ifstate = OA_IFSTATE_INIT_DOWN;

  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: oa_transmit
 *
 * Description:
 *   NuttX callback: Transmit the given packet.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *   pkt - network packet to be transmitted
 *
 * Returned Values:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

static int oa_transmit(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt)
{
  FAR struct oa_driver_s *priv = (FAR struct oa_driver_s *)dev;

  nxmutex_lock(&priv->lock);

  if (priv->tx_pkt || priv->ifstate != OA_IFSTATE_INIT_UP)
    {
      /* Previous TX packet was not yet sent to the network
       * or the interface has been shut down while waiting for the lock
       */

      nxmutex_unlock(&priv->lock);
      return -EAGAIN;
    }

  priv->tx_pkt_idx = 0;
  priv->tx_pkt_len = netpkt_getdatalen(dev, pkt);
  priv->tx_pkt = pkt;

  nxmutex_unlock(&priv->lock);

  work_queue(OAWORK, &priv->io_work, oa_io_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: oa_receive
 *
 * Description:
 *   NuttX callback: Claims an rx packet if available.
 *
 * Input Parameters:
 *   dev - reference to the NuttX driver state structure
 *
 * Returned Values:
 *   If the rx packet is ready, its pointer is returned.
 *   NULL is returned otherwise.
 *
 ****************************************************************************/

static FAR netpkt_t *oa_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct oa_driver_s *priv = (FAR struct oa_driver_s *)dev;

  nxmutex_lock(&priv->lock);

  if (priv->rx_pkt_ready)
    {
      netpkt_t *retval = priv->rx_pkt;
      priv->rx_pkt_ready = false;
      priv->rx_pkt = NULL;
      nxmutex_unlock(&priv->lock);
      return retval;
    }

  nxmutex_unlock(&priv->lock);

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oa_write_reg
 *
 * Description:
 *   Write to a MAC-PHY register.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   regid - Register id encapsulating MMS and ADDR
 *   word  - 32-bit word to be written to the register
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

int oa_write_reg(FAR struct oa_driver_s *priv,
                 oa_regid_t regid, uint32_t word)
{
  uint32_t txdata[3];
  uint32_t rxdata[3];
  uint8_t  mms  = OA_REGID_GET_MMS(regid);
  uint16_t addr = OA_REGID_GET_ADDR(regid);

  /* Prepare header */

  uint32_t header =   (1    << OA_WNR_POS)   /* Write Not Read */
                    | (mms  << OA_MMS_POS)
                    | (addr << OA_ADDR_POS);
  int parity = oa_get_parity(header);
  header |= parity ? 0 : OA_P_MASK;  /* Make header odd parity */

  /* Convert to big endian */

  header = htobe32(header);
  word = htobe32(word);

  /* Prepare exchange */

  txdata[0] = header;
  txdata[1] = word;

  oa_select_spi(priv);
  SPI_EXCHANGE(priv->spi, txdata, rxdata, 12);
  oa_deselect_spi(priv);
  if (rxdata[1] != header)
    {
      nerr("Error writing register\n");
      return ERROR;
    }

  ninfo("Writing register OK\n");
  return OK;
}

/****************************************************************************
 * Name: oa_read_reg
 *
 * Description:
 *   Read a MAC-PHY register.
 *
 * Input Parameters:
 *   priv  - pointer to the driver-specific state structure
 *   regid - register id encapsulating MMS and ADDR
 *   word  - pointer to a 32-bit destination variable
 *
 * Returned Value:
 *   On successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

int oa_read_reg(FAR struct oa_driver_s *priv,
                oa_regid_t regid, FAR uint32_t *word)
{
  uint32_t txdata[3];
  uint32_t rxdata[3];
  uint8_t  mms  = OA_REGID_GET_MMS(regid);
  uint16_t addr = OA_REGID_GET_ADDR(regid);
  int parity;
  uint32_t header;

  /* Prepare header */

  header =   (mms  << OA_MMS_POS)
           | (addr << OA_ADDR_POS);
  parity = oa_get_parity(header);
  header |= parity ? 0 : OA_P_MASK;  /* Make header odd parity */

  /* Convert to big endian */

  header = htobe32(header);

  /* Prepare exchange */

  txdata[0] = header;

  oa_select_spi(priv);
  SPI_EXCHANGE(priv->spi, txdata, rxdata, 12);
  oa_deselect_spi(priv);

  *word = be32toh(rxdata[2]);
  if (rxdata[1] != header)
    {
      nerr("Error reading register\n");
      return ERROR;
    }

  ninfo("Reading register OK\n");
  return OK;
}

/****************************************************************************
 * Name: oa_set_clear_bits
 *
 * Description:
 *   Perform a read-modify-write operation on a given register
 *   while setting bits from the setbits argument and clearing bits from
 *   the clearbits argument.
 *
 * Input Parameters:
 *   priv      - pointer to the driver-specific state structure
 *   regid     - register id of the register to be modified
 *   setbits   - bits set to one will be set in the register
 *   clearbits - bits set to one will be cleared in the register
 *
 * Returned Value:
 *   On a successful transaction OK is returned, otherwise ERROR is returned.
 *
 ****************************************************************************/

int oa_set_clear_bits(FAR struct oa_driver_s *priv,
                      oa_regid_t regid,
                      uint32_t setbits, uint32_t clearbits)
{
  uint32_t regval;

  if (oa_read_reg(priv, regid, &regval))
    {
      return ERROR;
    }

  regval |= setbits;
  regval &= ~clearbits;

  if (oa_write_reg(priv, regid, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: oa_set_clear_bits
 *
 * Description:
 *   Store the given MAC address into the net driver structure.
 *
 * Input Parameters:
 *   priv - pointer to the driver-specific state structure
 *   mac  - pointer to an array containing the MAC address
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void oa_store_mac_address(struct oa_driver_s *priv,
                          uint8_t *mac)
{
  memcpy(&priv->dev.netdev.d_mac.ether, mac, sizeof(struct ether_addr));
}

/****************************************************************************
 * Name: oa_bitrev8
 *
 * Description:
 *   Perform a bit reverse of a byte.
 *
 * Input Parameters:
 *   byte - byte to be reversed
 *
 * Returned Value:
 *   Byte with reversed bits is returned.
 *
 ****************************************************************************/

uint8_t oa_bitrev8(uint8_t byte)
{
  /* https://stackoverflow.com/a/2602885 */

  byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
  byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
  byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
  return byte;
}

/****************************************************************************
 * Name: oa_initialize
 *
 * Description:
 *   Initialize the Ethernet driver.
 *
 * Input Parameters:
 *   spi    - reference to the SPI driver state data
 *   config - reference to the predefined configuration of the driver
 *
 * Returned Value:
 *   On success OK is returned, otherwise negated errno is returned.
 *
 ****************************************************************************/

int oa_initialize(FAR struct spi_dev_s *spi,
                  struct oa_config_s *config)
{
  FAR struct oa_driver_s        *priv   = NULL;
  FAR struct netdev_lowerhalf_s *netdev = NULL;
  uint32_t device_type;
  int retval;

  /* Setup a dummy driver so SPI transfers are possible using the same interface */

  struct oa_driver_s dummy = { 0 };
  dummy.spi = spi;
  dummy.config = config;

  /* Reset MAC-PHY (done only using OA common registers) */

  if (oa_reset(&dummy))
    {
      nerr("Error resetting OA device.\n");
      retval = -EIO;
      goto errout;
    }

  /* Get device type from MAC-PHY OA common registers */

  if (oa_get_device_type(&dummy, &device_type))
    {
      nerr("Error getting the type of the OA device.\n");
      retval = -EIO;
      goto errout;
    }

  /* Call init function based on the MAC-PHY type */

  switch(device_type)
    {
#ifdef CONFIG_NET_OA_NCV7410
      case OA_NCV7410_DEVTYPE:
          priv = oa_ncv7410_initialize(spi, config);
          break;
#endif
#ifdef CONFIG_NET_OA_NCN26010
      case OA_NCN26010_DEVTYPE:
          priv = oa_ncn26010_initialize(spi, config);
          break;
#endif
#ifdef CONFIG_NET_OA_LAN8650
      case OA_LAN8650_DEVTYPE:
          priv = oa_lan8650_initialize(spi, config);
          break;
#endif
      default:
          retval = -EINVAL;
          nerr("Unknown device type, is the support enabled in Kconfig? "
               "Does the revision match?\n");
          goto errout;
    }

  if (priv == NULL)
    {
      nerr("Error initializing OA device\n");
      retval = -ENOMEM;
      goto errout;
    }

  priv->spi = spi;       /* Save the SPI instance                   */
  priv->config = config; /* Save the reference to the configuration */

  priv->ifstate = OA_IFSTATE_RESET;

  /* Init MAC address */

  if (priv->ops->action && priv->ops->action(priv, OA_ACTION_INIT_MAC))
    {
      nerr("Error initializing MAC address\n");
      retval = -EIO;
      goto errout;
    }

  /* Attach ISR */

  if (! priv->config->attach)
    {
      nerr("Error: Attach callback not provided by caller\n");
      retval = -EINVAL;
      goto errout;
    }

  if (! priv->config->enable)
    {
      nerr("Error: Enable callback not provided by caller\n");
      retval = -EINVAL;
      goto errout;
    }

  priv->config->attach(priv->config, oa_interrupt, priv);

  /* Init lock */

  nxmutex_init(&priv->lock);

  /* Register the device with the OS */

  netdev = &priv->dev;
  netdev->quota[NETPKT_TX] = OA_TX_QUOTA;
  netdev->quota[NETPKT_RX] = OA_RX_QUOTA;
  netdev->ops = &g_oa_ops;

  retval = netdev_lower_register(netdev, NET_LL_ETHERNET);
  if (retval == OK)
    {
      ninfo("Successfully registered OA network driver\n");
      return OK;
    }

  nerr("Error registering OA network driver: %d\n", retval);

errout:
  kmm_free(priv);
  return retval;
}
