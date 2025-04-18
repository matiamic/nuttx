/****************************************************************************
 * drivers/net/ncv7410.c
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

#if defined(CONFIG_NET) && defined(CONFIG_NCV7410)

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <sys/endian.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/ncv7410.h>
#include <nuttx/net/netdev_lowerhalf.h>


#include "ncv7410.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* NCV7410 Configuration Settings:
 *
 * CONFIG_NCV7410           - Enabled NCV7410 support
 * CONFIG_NCV7410_FREQUENCY - Define to use a different bus frequency
 * CONFIG_NCV7410_INT_PIN   - Pin used for MAC-PHY interrupt signal
 */

#define NCVWORK LPWORK

/* Packet Memory ************************************************************/
//TODO: make this make sense
#define NCV7410_PKTBUF_SIZE     100 //2048
#define NCV7410_PKTBUF_SIZE     100 //2048

/* Maximum number of allocated tx and rx packets */

#define NCV7410_TX_QUOTA        1
#define NCV7410_RX_QUOTA        1

#if CONFIG_IOB_NBUFFERS < (NCV7410_TX_QUOTA + NCV7410_RX_QUOTA)
#  error CONFIG_IOB_NBUFFERS must be > (NCV7410_TX_QUOTA + NCV7410_RX_QUOTA)
#endif

#if CONFIG_IOB_BUFSIZE < NCV7410_PKTBUF_SIZE
#  error CONFIG_IOB_BUFSIZE must be > NCV7410_PKTBUF_SIZE
#endif

#ifndef CONFIG_SCHED_LPWORK
#  error CONFIG_SCHED_LPWORK is needed by NCV7410 driver
#endif

#define NCV_RESET_TRIES 5

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The ncv7410_driver_s encapsulates all state information for a single hardware
 * interface
 */

enum ncv_ifstate_e
{
  NCV_RESET,
  NCV_INIT_DOWN,
  NCV_INIT_UP
};

/* use this instead of read-modify-write when changing setup
 * e.g. when changing state up/down, can also server as a backup for
 * configuration, when MAC-PHY is reset unexpectedly, now not used */

struct ncv7410_registers_s
{
  uint32_t oa_config0;
  uint32_t dio_config;
  uint32_t mac_control0;
  uint32_t phy_control;
};

struct ncv7410_driver_s
{
  /* This holds the information visible to the NuttX network
   * (must be placed first) */

  struct netdev_lowerhalf_s dev;

  /* This is the contained SPI driver instance */

  FAR struct spi_dev_s *spi;

  /* irq number of the interrupt signal pin */

  int irqnum;

  /* Work instances for work_queue handling */

  struct work_s interrupt_work;
  struct work_s io_work;

  /* Driver state, one of ncv_ifstate_e values */

  uint8_t ifstate;

  /* MAC-PHY internal buffer status */

  int txc;
  int rxa;

  /* Packet buffer management */

  FAR netpkt_t *tx_pkt;
  FAR netpkt_t *rx_pkt;
  int tx_pkt_idx;
  int rx_pkt_idx;
  int tx_pkt_len;
  bool rx_pkt_ready;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Parity calculation */

static int ncv_get_parity(uint32_t w);

/* SPI transfers */

static int ncv_write_reg(FAR struct ncv7410_driver_s *priv,
                         oa_regid_t regid, uint32_t word);

static int ncv_read_reg(FAR struct ncv7410_driver_s *priv,
                        oa_regid_t regid, FAR uint32_t *word);

static int ncv_poll_footer(FAR struct ncv7410_driver_s *priv,
                           FAR uint32_t *footer);

static int ncv_exchange_chunk(FAR struct ncv7410_driver_s *priv,
                              FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                              uint32_t header, uint32_t *footer);

/* Interrupt handling */

static int ncv_interrupt(int irq, FAR void *context, FAR void *arg);
static void ncv_interrupt_work(FAR void *arg);

/* Data Transaction Protocol logic */

static void ncv_io_work(FAR void *arg);

/* SPI inline utility functions */

static inline void ncv_lock_spi(FAR struct ncv7410_driver_s *priv);
static inline void ncv_unlock_spi(FAR struct ncv7410_driver_s *priv);

static inline void ncv_config_spi(FAR struct ncv7410_driver_s *priv);

static inline void ncv_select_spi(FAR struct ncv7410_driver_s *priv);
static inline void ncv_deselect_spi(FAR struct ncv7410_driver_s *priv);

/* ncv7410 reset and configuration */

static int ncv_reset(FAR struct ncv7410_driver_s *priv);
static int ncv_config(FAR struct ncv7410_driver_s *priv);
static int ncv_enable(FAR struct ncv7410_driver_s *priv);
static int ncv_disable(FAR struct ncv7410_driver_s *priv);

/* NuttX callback functions */

static int ncv7410_ifup(FAR struct netdev_lowerhalf_s *dev);
static int ncv7410_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int ncv7410_transmit(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt);
static FAR netpkt_t *ncv7410_receive(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int ncv7410_addmac(FAR struct netdev_lowerhalf_s *dev,
                          FAR const uint8_t *mac);
static int ncv7410_rmmac(FAR struct netdev_lowerhalf_s *dev,
                         FAR const uint8_t *mac);
#endif

/* Debug */

static void ncv_print_footer(uint32_t footer);

/* Initialization */

int ncv7410_initialize(FAR struct spi_dev_s *spi, int irq);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ncv_interrupt
 *
 * Description:
 *   Schedule interrupt work when the interrupt signal from ncv7410 is received
 *
 * Input Parameters:
 *   irq     - not used
 *   context - not used
 *   arg     - ncv7410_driver_s priv structure to be passed to the interupt worker
 *
 * Returned Value:
 *   zero
 *
 * Assumptions:
 *
 ****************************************************************************/

static int ncv_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) arg;

  DEBUGASSERT(priv != NULL);

  ninfo("ncv7410 interrupt!\n");

  /* schedule the work to be done */
  work_queue(NCVWORK, &priv->interrupt_work, ncv_interrupt_work, priv, 0);
  return 0;
}

/****************************************************************************
 * Name: ncv_interrupt_work
 *
 * Description:
 *   checks the origin of the interrupt and carry out necessary work
 *
 * Input Parameters:
 *   arg - pointer to driver private data
 *
 * Returned Value:
 *   none
 *
 * Assumptions:
 *
 ****************************************************************************/

static void ncv_interrupt_work(FAR void *arg)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) arg;
  uint32_t footer;

  DEBUGASSERT(priv != NULL);

  ninfo("ncv7410 interrupt worker invoked!\n");

  /* poll the data chunk footer */

  if (ncv_poll_footer(priv, &footer))
    {
      nerr("polling footer unsuccesful\n");
      PANIC();
    }
  ncv_print_footer(footer);

  /* find out the origin of the interrupt */
  /* if EXST in the footer, check enabled sources */
  /* STATUS0, link-status in clause 22 phy registers */

  /* update MAC-PHY buffer status */

  priv->txc = tx_credits(footer);
  priv->rxa = rx_available(footer);

  if ((priv->tx_pkt && priv->txc) || priv->rxa)
    {
      /* schedule IO work */
      work_queue(NCVWORK, &priv->io_work, ncv_io_work, priv, 0);
    }
}

/****************************************************************************
 * Name: ncv_io_work
 *
 * Description:
 *   exchanges data chunks with the MAC-PHY
 *
 * Input Parameters:
 *   arg - pointer to driver private data
 *
 * Returned Value:
 *   none
 *
 * Assumptions:
 *
 ****************************************************************************/

static void ncv_io_work(FAR void *arg)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) arg;

  uint8_t txbuf[NCV_CHUNK_DEFAULT_SIZE];
  uint8_t rxbuf[NCV_CHUNK_DEFAULT_SIZE];

  uint32_t header = (1 << OA_DNC_POS);
  uint32_t footer;

  int txlen;
  int rxlen;

  if (priv->txc && priv->tx_pkt != NULL)
    {
      header |= (1 << OA_DV_POS);  /* data valid */

      if (priv->tx_pkt_idx == 0)
        {
          header |=   (1 << OA_SV_POS)   /* start valid */
                    | (0 << OA_SWO_POS); /* start word at postion 0 in chunk */
        }

      txlen = priv->tx_pkt_len - priv->tx_pkt_idx;

      if (txlen <= NCV_CHUNK_DEFAULT_PAYLOAD_SIZE)
        {
          header |=   (1 << OA_EV_POS)             /* end valid */
                    | ((txlen - 1) << OA_EBO_POS); /* end byte offset */
        }
      else
        {
          txlen = NCV_CHUNK_DEFAULT_PAYLOAD_SIZE;
        }
      netpkt_copyout(&priv->dev, txbuf, priv->tx_pkt, txlen, priv->tx_pkt_idx);
      priv->tx_pkt_idx += txlen;
    }

  if (priv->rx_pkt == NULL)
    {
      priv->rx_pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
      if (priv->rx_pkt == NULL)
        {
          ninfo("info: Failed to alloc rx netpkt\n");

          /* there is no buffer for potential rx data
           * => rx receiving is disabled */

          header |= (1 << OA_NORX_POS);  /* no rx */
        }
    }

  /* disable receiving if the rx packet is waiting to be received by network */

  if (priv->rx_pkt_ready)
    {
      header |= (1 << OA_NORX_POS);  /* no rx */
    }

  /* do the SPI exchange */

  if (ncv_exchange_chunk(priv, txbuf, rxbuf, header, &footer))
    {
      nerr("Error during chunk exchange\n");
      PANIC();
    }

  /* if finished tx packet, do the housekeeping */

  if (priv->tx_pkt_idx == priv->tx_pkt_len)
    {
      netpkt_free(&priv->dev, priv->tx_pkt, NETPKT_TX);
      priv->tx_pkt = NULL;
      netdev_lower_txdone(&priv->dev);
    }

  /* update buffer status */

  priv->txc = tx_credits(footer);
  priv->rxa = rx_available(footer);

  if (frame_drop(footer))
    {
      if (priv->rx_pkt)
        {
          netpkt_free(&priv->dev, priv->rx_pkt, NETPKT_RX);
          priv->rx_pkt = NULL;
        }
    }

  if (data_valid(footer))  // note: this is OK when errors on SPI bus are NOT expected
                           //       otherwise rx_pkt != NULL and !rx_pkt_ready should be checked
    {
      if (start_valid(footer))
        {
          priv->rx_pkt_idx = 0;
        }
      if (end_valid(footer))
        {
          rxlen = end_byte_offset(footer) + 1;
        }
      else
        {
          rxlen = NCV_CHUNK_DEFAULT_PAYLOAD_SIZE;
        }
      netpkt_copyin(&priv->dev, priv->rx_pkt, rxbuf, rxlen, priv->rx_pkt_idx);
      priv->rx_pkt_idx += rxlen;

      if (end_valid(footer))
        {
          /* strip down last 4 bytes including FCS */
          netpkt_setdatalen(&priv->dev, priv->rx_pkt,
                            netpkt_getdatalen(&priv->dev, priv->rx_pkt) - 4);
          priv->rx_pkt_ready = true;
          netdev_lower_rxready(&priv->dev);
        }
    }

  // logic behind this is not yet fully done
  // should I do a big while loop? or should I plan io_work using work_queue?
  // what exact conditions must be met to plan the invokation?
  if ((priv->tx_pkt && priv->txc) || priv->rxa)
    {
      work_queue(NCVWORK, &priv->io_work, ncv_io_work, priv, 0);
    }
}

/****************************************************************************
 * Name: ncv_get_parity
 *
 * Description:
 *   Obtain parity of 32-bit word.
 *
 * Input Parameters:
 *   w - 32-bit word, subject to the parity calculation
 *
 * Returned Value:
 *   If the parity of the word is even, zero is returned. Otherwise one is returned.
 *
 ****************************************************************************/

static int ncv_get_parity(uint32_t w)
{
  /* www-graphics.stanford.edu/~seander/bithacks.html */
  w ^= w >> 1;
  w ^= w >> 2;
  w = (w & 0x11111111U) * 0x11111111U;
  return (w >> 28) & 1;
}

/****************************************************************************
 * Name: ncv_lock_spi
 *
 * Description:
 *
 * Input Parameters:
 *   priv -
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ncv_lock_spi(FAR struct ncv7410_driver_s *priv)
{
  SPI_LOCK(priv->spi, true);
}

/****************************************************************************
 * Name: ncv_unlock_spi
 *
 * Description:
 *
 * Input Parameters:
 *   priv -
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ncv_unlock_spi(FAR struct ncv7410_driver_s *priv)
{
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: ncv_config_spi
 *
 * Description:
 *
 * Input Parameters:
 *   priv -
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ncv_config_spi(FAR struct ncv7410_driver_s *priv)
{
  SPI_SETMODE(priv->spi, OA_SPI_MODE);
  SPI_SETBITS(priv->spi, OA_SPI_NBITS);
  SPI_HWFEATURES(priv->spi, 0);  // disable HW features
  SPI_SETFREQUENCY(priv->spi, CONFIG_NCV7410_FREQUENCY);
}

/****************************************************************************
 * Name: ncv_select_spi
 *
 * Description:
 *   Assert device's CS pin
 *
 * Input Parameters:
 *   priv -
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ncv_select_spi(FAR struct ncv7410_driver_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), true);
}

/****************************************************************************
 * Name: ncv_deselect_spi
 *
 * Description:
 *   Releases device's CS pin
 *
 * Input Parameters:
 *   priv -
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ncv_deselect_spi(FAR struct ncv7410_driver_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_ETHERNET(0), false);
}

/****************************************************************************
 * Name: ncv_write_reg
 *
 * Description:
 *   Write a word to ncv7410's registers.
 *
 * Input Parameters:
 *   priv  - pointer to the driver specific data structure
 *   regid - Register id encapsulating MMS and ADDR
 *   word  - 32-bit word to be written to the register
 *
 * Returned Value:
 *   on successful transaction 0 is returned, otherwise 1 is returned
 *
 ****************************************************************************/

static int ncv_write_reg(FAR struct ncv7410_driver_s *priv,
                         oa_regid_t regid, uint32_t word)
{
  uint32_t txdata[3] = { 0 };
  uint32_t rxdata[3] = { 0 };
  uint8_t  mms  = OA_REGID_GET_MMS(regid);
  uint16_t addr = OA_REGID_GET_ADDR(regid);

  // prepare header
  uint32_t header =   (1    << OA_WNR_POS)   // write-not-read
                    | (mms  << OA_MMS_POS)   // mms
                    | (addr << OA_ADDR_POS); // address
  int parity = ncv_get_parity(header);
  header |= parity ? 0 : OA_P_MASK;  // make header odd parity

  // convert to big endian
  header = htobe32(header);
  word = htobe32(word);

  // prepare exchange
  txdata[0] = header;
  txdata[1] = word;

  ncv_lock_spi(priv);
  ncv_config_spi(priv);
  ncv_select_spi(priv);
  SPI_EXCHANGE(priv->spi, txdata, rxdata, 12);
  ncv_deselect_spi(priv);
  ncv_unlock_spi(priv);
  if (rxdata[1] != header)
    {
      nerr("Error writing register\n");
      return 1;  // error
    }
  ninfo("Writing register OK\n");
  return 0;
}

/****************************************************************************
 * Name: ncv_read_reg
 *
 * Description:
 *   Read a word from ncv7410's registers.
 *
 * Input Parameters:
 *   priv - pointer to the driver specific data structure
 *   regid - Register id encapsulating MMS and ADDR
 *   word - pointer to 32-bit variable into which the register will be stored
 *
 * Returned Value:
 *   on successful transaction OK is returned, otherwise ERROR is returned
 *
 ****************************************************************************/

static int ncv_read_reg(FAR struct ncv7410_driver_s *priv,
                        oa_regid_t regid, FAR uint32_t *word)
{
  uint32_t txdata[3] = { 0 };
  uint32_t rxdata[3] = { 0 };
  uint8_t  mms  = OA_REGID_GET_MMS(regid);
  uint16_t addr = OA_REGID_GET_ADDR(regid);
  int parity;
  uint32_t header;

  // prepare header
  header =   (mms  << OA_MMS_POS)   // mms
           | (addr << OA_ADDR_POS); // address
  parity = ncv_get_parity(header);
  header |= parity ? 0 : OA_P_MASK;  // make header odd parity

  // convert to big endian
  header = htobe32(header);

  // prepare exchange
  txdata[0] = header;

  ncv_lock_spi(priv);
  ncv_config_spi(priv);
  ncv_select_spi(priv);
  SPI_EXCHANGE(priv->spi, txdata, rxdata, 12);
  ncv_deselect_spi(priv);
  ncv_unlock_spi(priv);

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
 * Name: ncv_poll_footer
 *
 * Description:
 *   poll a data transaction chunk footer without reading or writing frame data
 *
 * Input Parameters:
 *   priv   - pointer to the driver specific data structure
 *   footer - pointer to a 32-bit variable for the footer
 *
 * Returned Value:
 *   on successful transaction OK is returned, otherwise ERROR is returned
 *
 ****************************************************************************/

static int ncv_poll_footer(FAR struct ncv7410_driver_s *priv,
                           FAR uint32_t *footer)
{
  uint32_t txdata[NCV_CHUNK_DEFAULT_SIZE / 4] = { 0 };
  uint32_t rxdata[NCV_CHUNK_DEFAULT_SIZE / 4] = { 0 };
  uint32_t header;
  *footer = 0;

  header =   (1 << OA_DNC_POS)   // Data Not Control
           | (1 << OA_NORX_POS); // No Read

  header |= (!ncv_get_parity(header) << OA_P_POS);
  header = htobe32(header);
  txdata[0] = header;

  ncv_lock_spi(priv);
  ncv_config_spi(priv);
  ncv_select_spi(priv);
  SPI_EXCHANGE(priv->spi, txdata, rxdata, NCV_CHUNK_DEFAULT_SIZE);
  ncv_deselect_spi(priv);
  ncv_unlock_spi(priv);

  *footer = rxdata[NCV_CHUNK_DEFAULT_PAYLOAD_SIZE / 4];
  *footer = be32toh(*footer);
  if (!ncv_get_parity(*footer))
    {
      nerr("Wrong parity in the footer\n");
      *footer = 0;
      return ERROR;
    }
  if (header_bad(*footer))
    {
      nerr("HDRB set in the footer\n");
      *footer = 0;
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_exchange_chunk
 *
 * Description:
 *   send a data chunk to MAC-PHY and simultaneously receive chunk,
 *
 *   computing header parity and checking footer parity as well as
 *   converting to proper endianity is done by this function
 *
 * Input Parameters:
 *   priv   - pointer to the driver specific data structure
 *   txbuf  - buffer with transmit chunk data
 *   rxbuf  - buffer to save the received chunk to
 *   header - header controlling the transaction
 *   footer - pointer to a 32-bit value for the footer
 *
 * Returned Value:
 *   on successful transaction OK is returned, otherwise ERROR is returned
 *
 ****************************************************************************/

static int ncv_exchange_chunk(FAR struct ncv7410_driver_s *priv,
                              FAR uint8_t *txbuf, FAR uint8_t *rxbuf,
                              uint32_t header, uint32_t *footer)
{
  header |= (!ncv_get_parity(header) << OA_P_POS);
  header = htobe32(header);

  ncv_lock_spi(priv);
  ncv_config_spi(priv);
  ncv_select_spi(priv);
  SPI_EXCHANGE(priv->spi, (uint8_t *) &header, rxbuf, 4);
  SPI_EXCHANGE(priv->spi, txbuf, &rxbuf[4], NCV_CHUNK_DEFAULT_PAYLOAD_SIZE - 4);
  SPI_EXCHANGE(priv->spi, &txbuf[NCV_CHUNK_DEFAULT_PAYLOAD_SIZE - 4],
               (uint8_t *) footer, 4);
  ncv_deselect_spi(priv);
  ncv_unlock_spi(priv);

  *footer = be32toh(*footer);
  if (!ncv_get_parity(*footer))
    {
      nerr("Wrong parity in the footer\n");
      return ERROR;
    }
  if (header_bad(*footer))
    {
      nerr("HDRB set in the footer\n");
      return ERROR;
    }
  return OK;
}

/****************************************************************************
 * Name: ncv_reset
 *
 * Description:
 *   Perform SW reset of the ncv7410 MAC-PHY
 *
 * Input Parameters:
 *   priv - pointer to the driver specific data structure
 *
 * Returned Value:
 *   on successful reset OK is returned, otherwise ERROR is returned
 *
 ****************************************************************************/

static int ncv_reset(FAR struct ncv7410_driver_s *priv)
{
  int tries = NCV_RESET_TRIES;
  uint32_t regval = (1 << OA_RESET_SWRESET_POS);

  if (ncv_write_reg(priv, OA_RESET_REGID, regval))
    {
      return ERROR;
    }

  do
    {
      if (ncv_read_reg(priv, OA_RESET_REGID, &regval))
        {
          return ERROR;
        }
    }
  while (tries-- && (regval & 1));
  if (regval & 1)
    {
      return ERROR;
    }

  /* clear HDRE in STATUS0 (due to a bug in ncv7410) */

  if (ncv_write_reg(priv, OA_STATUS0_REGID, (1 << OA_STATUS0_HDRE_POS)))
    {
      return ERROR;
    }

  /* clear reset complete flag */

  if (ncv_write_reg(priv, OA_STATUS0_REGID, (1 << OA_STATUS0_RESETC_POS)))
    {
      return ERROR;
    }

  /* blink with LEDs for debugging purposes */

  for (int i = 0; i < 4; i++)
    {
      regval = 0x0302;
      if (ncv_write_reg(priv, NCV_DIO_CONFIG_REGID, regval))
        {
          return ERROR;
        }
      nxsig_usleep(250000);
      regval = 0x0203;
      if (ncv_write_reg(priv, NCV_DIO_CONFIG_REGID, regval))
        {
          return ERROR;
        }
      nxsig_usleep(250000);
    }

  // set DIOs to default
  regval = 0x6060;
  if (ncv_write_reg(priv, NCV_DIO_CONFIG_REGID, regval))
    {
      return ERROR;
    }
  return OK;
}

/****************************************************************************
 * Name: ncv_config
 *
 * Description:
 *   configure ncv7410 into promiscuous mode and set SYNC flag
 *
 * Input Parameters:
 *   priv - pointer to the driver specific data structure
 *
 * Returned Value:
 *   on success OK is returned, otherwise ERROR is returned
 *
 ****************************************************************************/

static int ncv_config(FAR struct ncv7410_driver_s *priv)
{
  uint32_t regval;

  ninfo("Configuring ncv7410\n");

  /* setup LEDs DIO0: txrx blink                      */
  /*            DIO1: link enabled and link status up */

  regval =   (NCV_DIO_TXRX_FUNC << NCV_DIO0_FUNC_POS)
           | (NCV_DIO_LINK_CTRL_FUNC << NCV_DIO1_FUNC_POS)
           | (1 << NCV_DIO0_OUT_VAL_POS)
           | (1 << NCV_DIO1_OUT_VAL_POS);

  if (ncv_write_reg(priv, NCV_DIO_CONFIG_REGID, regval))
    {
      return ERROR;
    }

  /* enable MAC TX, RX, enable transmit FCS computation on MAC */

  regval =   (1 << NCV_MAC_CONTROL0_FCSA_POS)
           | (1 << NCV_MAC_CONTROL0_TXEN_POS)
           | (1 << NCV_MAC_CONTROL0_RXEN_POS);

  if (ncv_write_reg(priv, NCV_MAC_CONTROL0_REGID, regval))
    {
      return ERROR;
    }

  /* setup SPI protocol and set SYNC flag */

  regval =   (1 << OA_CONFIG0_SYNC_POS)
           | (1 << OA_CONFIG0_CSARFE_POS)
           | (1 << OA_CONFIG0_ZARFE_POS)
           | (3 << OA_CONFIG0_TXCTHRESH_POS)
           | (6 << OA_CONFIG0_CPS_POS);

  if (ncv_write_reg(priv, OA_CONFIG0_REGID, regval))
    {
      return ERROR;
    }

  /* enable rx buffer overflow interrupt */

  regval = 0x1FBF & ~(1 << OA_IMSK0_RXBOEM_POS);

  if (ncv_write_reg(priv, OA_IMSK0_REGID, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_enable
 *
 * Description:
 *   enable TX and RX on the MAC-PHY
 *
 * Input Parameters:
 *   priv - pointer to the driver specific data structure
 *
 * Returned Value:
 *   on success OK is returned, otherwise ERROR is returned
 *
 * Assumptions:
 *   ncv7410 is already configured using ncv_config()
 *
 ****************************************************************************/

static int ncv_enable(FAR struct ncv7410_driver_s *priv)
{
  /* enable PHY */

  uint32_t regval;

  ninfo("Enabling ncv7410\n");

  /* enable RX and TX in PHY */

  if (ncv_read_reg(priv, OA_PHY_CONTROL_REGID, &regval))
    {
      return ERROR;
    }

  regval |= (1 << OA_PHY_CONTROL_LCTL_POS);

  if (ncv_write_reg(priv, OA_PHY_CONTROL_REGID, regval))
    {
      return ERROR;
    }

  /* enable PHY interrupt */

  if (ncv_read_reg(priv, OA_IMSK0_REGID, &regval))
    {
      return ERROR;
    }

  regval |= (1 << OA_IMSK0_PHYINTM_POS);

  if (ncv_write_reg(priv, OA_IMSK0_REGID, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_disable
 *
 * Description:
 *   disable TX and RX on the MAC-PHY
 *
 * Input Parameters:
 *   priv - pointer to the driver specific data structure
 *
 * Returned Value:
 *   on success OK is returned, otherwise ERROR is returned
 *
 ****************************************************************************/

static int ncv_disable(FAR struct ncv7410_driver_s *priv)
{
  /* disable PHY */

  uint32_t regval;

  ninfo("Disabling ncv7410\n");

  /* disable PHY interrupt */

  if (ncv_read_reg(priv, OA_IMSK0_REGID, &regval))
    {
      return ERROR;
    }

  regval &= ~(1 << OA_IMSK0_PHYINTM_POS);

  if (ncv_write_reg(priv, OA_IMSK0_REGID, regval))
    {
      return ERROR;
    }

  /* disable RX and TX in PHY */

  if (ncv_read_reg(priv, OA_PHY_CONTROL_REGID, &regval))
    {
      return ERROR;
    }

  regval &= ~(1 << OA_PHY_CONTROL_LCTL_POS);

  if (ncv_write_reg(priv, OA_PHY_CONTROL_REGID, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_print_footer
 *
 * Description:
 *   print individual bitfield of a receive chunk footer
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void ncv_print_footer(uint32_t footer)
{
  ninfo("Footer:\n");
  ninfo("  EXST: %d\n", ext_status(footer));
  ninfo("  HDRB: %d\n", header_bad(footer));
  ninfo("  SYNC: %d\n", mac_phy_sync(footer));
  ninfo("  RCA:  %d\n", rx_available(footer));
  ninfo("  DV:   %d\n", data_valid(footer));
  ninfo("  SV:   %d\n", start_valid(footer));
  ninfo("  SWO:  %d\n", start_word_offset(footer));
  ninfo("  FD:   %d\n", frame_drop(footer));
  ninfo("  EV:   %d\n", end_valid(footer));
  ninfo("  EBO:  %d\n", end_byte_offset(footer));
  ninfo("  RTSA: %d\n", rx_frame_timestamp_added(footer));
  ninfo("  RTSP: %d\n", rx_frame_timestamp_parity(footer));
  ninfo("  TXC:  %d\n", tx_credits(footer));
}

/*****************************************************************************
 * Netdev upperhalf callbacks
 *****************************************************************************/

/****************************************************************************
 * Name: ncv7410_ifup
 *
 * Description:
 *   NuttX callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Values:
 *   OK (0) is returned on success
 *   negated errno is returned otherwise
 *
 ****************************************************************************/

static int ncv7410_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) dev;

  ninfo("Bringing up ncv7410\n");

  if (priv->ifstate == NCV_RESET)
    {
      if (ncv_config(priv) == ERROR)
        {
          nerr("Error configuring ncv7410\n");
          return -EIO;
        }
      priv->ifstate = NCV_INIT_DOWN;
    }

  if (ncv_enable(priv) == ERROR)
    {
      nerr("Error enabling ncv7410\n");
      return -EIO;
    }

  priv->ifstate = NCV_INIT_UP;

  return OK;
}

static int ncv7410_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) dev;

  if (priv->ifstate == NCV_RESET)
    {
      return OK;
    }

  if (ncv_disable(priv) == ERROR)
    {
      nerr("Error disabling ncv7410\n");
      return -EIO;
    }

  priv->ifstate = NCV_INIT_DOWN;

  return OK;
}

static int ncv7410_transmit(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) dev;

  priv->tx_pkt_idx = 0;
  priv->tx_pkt_len = netpkt_getdatalen(dev, pkt);
  // maybe atomicity of the following operation and barrier here is needed
  // or mutex
  // for sync with work queue tasks
  priv->tx_pkt = pkt;
  work_queue(NCVWORK, &priv->io_work, ncv_io_work, priv, 0);
  return OK;
}

static FAR netpkt_t *ncv7410_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) dev;

  netpkt_t *retval;

  if (priv->rx_pkt_ready)
    {
      priv->rx_pkt_ready = false;
      retval = priv->rx_pkt;
      priv->rx_pkt = NULL;
      return retval;
    }
  /* nerr("Receive called when rx packet is not ready"); */
  return NULL;
}

#ifdef CONFIG_NET_MCASTGROUP
static int ncv7410_addmac(FAR struct netdev_lowerhalf_s *dev,
                          FAR const uint8_t *mac)
{
  return 0;
}

static int ncv7410_rmmac(FAR struct netdev_lowerhalf_s *dev,
                         FAR const uint8_t *mac)
{
  return 0;
}
#endif

/*****************************************************************************
 * Private Data
 *****************************************************************************/

static const struct netdev_ops_s g_ncv7410_ops =
{
  .ifup     = ncv7410_ifup,
  .ifdown   = ncv7410_ifdown,
  .transmit = ncv7410_transmit,
  .receive  = ncv7410_receive,
#ifdef CONFIG_NET_MCASTGROUP
  .addmac   = ncv7410_addmac,
  .rmmac    = ncv7410_rmmac,
#endif
#if CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD > 0
  .reclaim  = ncv7410_reclaim,
#endif
  // TODO: add ioctl
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ncv7410_initialize
 *
 * Description:
 *   Initialize the Ethernet driver. The NCV7410 device is assumed to be
 *   in the post-reset state upon entry to this function.
 *
 * Input Parameters:
 *   spi - A reference to the platform's SPI driver for the NCV7410
 *   irq - interrupt number of the pin connected to NCV7410's interrupt signal
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ncv7410_initialize(FAR struct spi_dev_s *spi, int irq)
{
  FAR struct ncv7410_driver_s   *priv   = NULL;
  FAR struct netdev_lowerhalf_s *netdev = NULL;
  int retval;

  /* Allocate the interface structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      nerr("Could not allocate data for ncv7410 priv\n");
      return -ENOMEM;
    }

  priv->spi = spi;     /* Save the SPI instance */
  priv->irqnum = irq;  /* Save the low-level MCU interface */

  /* reset ncv7410 chip */

  if (ncv_reset(priv))
    {
      nerr("Error resetting ncv7410\n");
      retval = -EIO;
      goto errout;
    }
  priv->ifstate = NCV_RESET;
  ninfo("Resetting ncv7410 OK\n");

  /* attach testing ISR */

  irq_attach(priv->irqnum, ncv_interrupt, priv);

  /* Register the device with the OS */

  netdev = &priv->dev;
  netdev->quota[NETPKT_TX] = NCV7410_TX_QUOTA;
  netdev->quota[NETPKT_RX] = NCV7410_RX_QUOTA;
  netdev->ops = &g_ncv7410_ops;

  retval = netdev_lower_register(netdev, NET_LL_ETHERNET);
  if (retval == OK)
    {
      ninfo("Succesfully registered ncv7410 network driver\n");
      return OK;
    }
  nerr("Error registering ncv7410 network driver: %d\n", retval);

errout:
  kmm_free(priv);
  return retval;
}

#endif /* CONFIG_NET && CONFIG_NCV7410_NET */
