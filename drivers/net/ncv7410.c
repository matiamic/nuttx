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
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <sys/endian.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/net/ncv7410.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "ncv7410.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NCVWORK LPWORK

#ifdef CONFIG_NETDEV_HPWORK_THREAD
#  warn "Synchronization is not implemented for "                   \
        "CONFIG_NETDEV_HPWORK_THREAD, CONFIG_NETDEV_HPWORK_THREAD " \
        "not recommended"
#endif

#ifndef CONFIG_SCHED_LPWORK
#  error "CONFIG_SCHED_LPWORK not defined, NCV7410 driver depends on it"
#endif

#if defined(CONFIG_NETDEV_WORK_THREAD) || CONFIG_SCHED_LPNTHREADS > 1
#  define NCV_MUTEX
#  define ncv_mutex_lock(m)    nxmutex_lock(m)
#  define ncv_mutex_trylock(m) nxmutex_trylock(m)
#  define ncv_mutex_unlock(m)  nxmutex_unlock(m)
#else
#  define ncv_mutex_lock(m)    ncv_return_ok()
#  define ncv_mutex_trylock(m) ncv_return_ok()
#  define ncv_mutex_unlock(m)  ncv_return_ok()
#endif

#define NCV_RESET_TRIES 5

/* Packet Memory ************************************************************/

/* Maximum number of allocated tx and rx packets */

#define NCV7410_TX_QUOTA        1
#define NCV7410_RX_QUOTA        1

#if CONFIG_IOB_NBUFFERS < (NCV7410_TX_QUOTA + NCV7410_RX_QUOTA)
#  error "CONFIG_IOB_NBUFFERS must be > (NCV7410_TX_QUOTA + NCV7410_RX_QUOTA)"
#endif

#ifndef CONFIG_SCHED_LPWORK
#  error "CONFIG_SCHED_LPWORK is needed by NCV7410 driver"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The ncv7410_driver_s encapsulates all state information for a single
 * hardware interface
 */

enum ncv_ifstate_e
{
  NCV_RESET,
  NCV_INIT_DOWN,
  NCV_INIT_UP
};

/* use this instead of read-modify-write when changing setup
 * e.g. when changing state up/down, can also server as a backup for
 * configuration, when MAC-PHY is reset unexpectedly, now not used
 */

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
   * (must be placed first)
   */

  struct netdev_lowerhalf_s dev;

  /* This is the contained SPI driver instance */

#ifdef NCV_MUTEX
  mutex_t mutex;
#endif

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

/* Bit calculations */

static int ncv_get_parity(uint32_t w);
uint8_t ncv_bitrev8(uint8_t b);

/* SPI transfers */

static int ncv_write_reg(FAR struct ncv7410_driver_s *priv,
                         oa_regid_t regid, uint32_t word);

static int ncv_read_reg(FAR struct ncv7410_driver_s *priv,
                        oa_regid_t regid, FAR uint32_t *word);

static int ncv_set_clear_bits(FAR struct ncv7410_driver_s *priv,
                        oa_regid_t regid,
                        uint32_t setbits, uint32_t clearbits);

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
static int ncv_init_mac_addr(FAR struct ncv7410_driver_s *priv);

/* driver buffer manipulation */

static void ncv_reset_driver_buffers(FAR struct ncv7410_driver_s *priv);

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

/* alternative for mutex operations, when mutex not needed */

static inline int ncv_return_ok(void)
{
  return OK;
}


/* Initialization */

int ncv7410_initialize(FAR struct spi_dev_s *spi, int irq);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ncv_interrupt
 *
 * Description:
 *   Schedule interrupt work when the interrupt signal from ncv7410 is
 *   received
 *
 * Input Parameters:
 *   irq     - not used
 *   context - not used
 *   arg     - ncv7410_driver_s priv structure to be passed to the interupt
 *             worker
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

  /* schedule interrupt work */

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

  ncv_mutex_lock(&priv->mutex);

  if (priv->ifstate != NCV_INIT_UP)
    {
      return;
    }

  ninfo("ncv7410 interrupt worker invoked!\n");

  /* poll the data chunk footer */

  if (ncv_poll_footer(priv, &footer))
    {
      nerr("polling footer unsuccesful\n");

      /* TODO: don't */

      PANIC();
    }

  ncv_print_footer(footer);

  /* find out the origin of the interrupt
   * if EXST in the footer, check enabled sources
   * STATUS0, link-status in clause 22 phy registers
   */

  /* update MAC-PHY buffer status */

  priv->txc = oa_tx_credits(footer);
  priv->rxa = oa_rx_available(footer);

  if ((priv->tx_pkt && priv->txc) || priv->rxa)
    {
      /* schedule IO work */

      work_queue(NCVWORK, &priv->io_work, ncv_io_work, priv, 0);
    }

  ncv_mutex_unlock(&priv->mutex);
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

  uint32_t header = (1 << OA_DNC_POS); /* Data Not Control */
  uint32_t footer;

  int txlen; /* how many bytes will be sent in chunk */
  int rxlen; /* how many bytes are received from the chunk */

  ncv_mutex_lock(&priv->mutex);

  if (priv->ifstate != NCV_INIT_UP)
    {
      return;
    }

  if (priv->txc && priv->tx_pkt != NULL)
    {
      header |= (1 << OA_DV_POS);  /* Data Valid */

      if (priv->tx_pkt_idx == 0)
        {
          header |=   (1 << OA_SV_POS)   /* Start Valid */
                    | (0 << OA_SWO_POS); /* start word at postion 0 in chunk */
        }

      txlen = priv->tx_pkt_len - priv->tx_pkt_idx;

      if (txlen <= NCV_CHUNK_DEFAULT_PAYLOAD_SIZE)
        {
          header |=   (1 << OA_EV_POS)             /* End Valid */
                    | ((txlen - 1) << OA_EBO_POS); /* End Byte Offset */
        }
      else
        {
          txlen = NCV_CHUNK_DEFAULT_PAYLOAD_SIZE;
        }

      netpkt_copyout(&priv->dev, txbuf, priv->tx_pkt,
                     txlen, priv->tx_pkt_idx);
      priv->tx_pkt_idx += txlen;
    }

  if (priv->rx_pkt == NULL)
    {
      priv->rx_pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
      if (priv->rx_pkt == NULL)
        {
          ninfo("info: Failed to alloc rx netpkt\n");

          /* there is no buffer for potential rx data
           * => rx receiving is disabled
           */

          header |= (1 << OA_NORX_POS);  /* no rx */
        }
    }

  /* disable receiving if the rx packet is waiting to be claimed by network */

  if (priv->rx_pkt_ready)
    {
      header |= (1 << OA_NORX_POS);  /* no rx */
    }

  /* do the SPI exchange */

  if (ncv_exchange_chunk(priv, txbuf, rxbuf, header, &footer))
    {
      nerr("Error during chunk exchange\n");

      /* TODO: do not panic, the best is probably to report the error
       * and reset MAC to some defined state and reset driver */

      PANIC();
    }

  /* if finished tx packet, do the housekeeping */

  if (priv->tx_pkt && (priv->tx_pkt_idx == priv->tx_pkt_len))
    {
      netpkt_free(&priv->dev, priv->tx_pkt, NETPKT_TX);
      priv->tx_pkt = NULL;
      netdev_lower_txdone(&priv->dev);
    }

  /* update buffer status */

  priv->txc = oa_tx_credits(footer);
  priv->rxa = oa_rx_available(footer);

  if (oa_frame_drop(footer))
    {
      if (priv->rx_pkt)
        {
          netpkt_free(&priv->dev, priv->rx_pkt, NETPKT_RX);
          priv->rx_pkt = NULL;
        }
    }

  /* check rx_pkt && !rx_pkt_ready,
   * oa_data_valid flag migh be set due to an SPI error
   */

  if (oa_data_valid(footer) && priv->rx_pkt && !priv->rx_pkt_ready)
    {
      if (oa_start_valid(footer))
        {
          priv->rx_pkt_idx = 0;
        }

      if (oa_end_valid(footer))
        {
          rxlen = oa_end_byte_offset(footer) + 1;
        }
      else
        {
          rxlen = NCV_CHUNK_DEFAULT_PAYLOAD_SIZE;
        }

      netpkt_copyin(&priv->dev, priv->rx_pkt, rxbuf,
                    rxlen, priv->rx_pkt_idx);
      priv->rx_pkt_idx += rxlen;

      if (oa_end_valid(footer))
        {
          /* strip down last 4 bytes including FCS */

          netpkt_setdatalen(&priv->dev, priv->rx_pkt,
                            netpkt_getdatalen(&priv->dev, priv->rx_pkt) - 4);
          priv->rx_pkt_ready = true;
          netdev_lower_rxready(&priv->dev);
        }
    }

  /* plan further work if needed */

  if ((priv->tx_pkt && priv->txc) || priv->rxa)
    {
      work_queue(NCVWORK, &priv->io_work, ncv_io_work, priv, 0);
    }

  ncv_mutex_unlock(&priv->mutex);
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
 *   If the parity of the word is even, zero is returned.
 *   Otherwise one is returned.
 *
 ****************************************************************************/

static int ncv_get_parity(uint32_t w)
{
  /* www-graphics.stanford.edu/~seander/bithacks.html */

  w ^= w >> 1;
  w ^= w >> 2;
  w = (w & 0x11111111u) * 0x11111111u;
  return (w >> 28) & 1;
}

/****************************************************************************
 * Name: ncv_bitrev8
 *
 * Description:
 *   perform a bit reverse of a byte
 *
 * Input Parameters:
 *   b - byte to be reversed
 *
 * Returned Value:
 *   Byte with reversed bits is returned
 *
 ****************************************************************************/

uint8_t ncv_bitrev8(uint8_t b)
{
  /* https://stackoverflow.com/a/2602885 */

  b = (b & 0xf0) >> 4 | (b & 0x0f) << 4;
  b = (b & 0xcc) >> 2 | (b & 0x33) << 2;
  b = (b & 0xaa) >> 1 | (b & 0x55) << 1;
  return b;
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
  SPI_HWFEATURES(priv->spi, 0);  /* disable HW features */
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
  uint32_t txdata[3];
  uint32_t rxdata[3];
  uint8_t  mms  = OA_REGID_GET_MMS(regid);
  uint16_t addr = OA_REGID_GET_ADDR(regid);

  /* prepare header */

  uint32_t header =   (1    << OA_WNR_POS)   /* Write Not Read */
                    | (mms  << OA_MMS_POS)
                    | (addr << OA_ADDR_POS);
  int parity = ncv_get_parity(header);
  header |= parity ? 0 : OA_P_MASK;  /* make header odd parity */

  /* convert to big endian */

  header = htobe32(header);
  word = htobe32(word);

  /* prepare exchange */

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
      return ERROR;
    }

  ninfo("Writing register OK\n");
  return OK;
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
  uint32_t txdata[3];
  uint32_t rxdata[3];
  uint8_t  mms  = OA_REGID_GET_MMS(regid);
  uint16_t addr = OA_REGID_GET_ADDR(regid);
  int parity;
  uint32_t header;

  /* prepare header */

  header =   (mms  << OA_MMS_POS)
           | (addr << OA_ADDR_POS);
  parity = ncv_get_parity(header);
  header |= parity ? 0 : OA_P_MASK;  /* make header odd parity */

  /* convert to big endian */

  header = htobe32(header);

  /* prepare exchange */

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
 * Name: ncv_set_clear_bits
 *
 * Description:
 *   Perform read-modify-write operation on a given register
 *   while setting bits from the setbits argument and clearing bits from
 *   the clearbits argument
 *
 * Input Parameters:
 *   priv      - pointer to the driver specific data structure
 *   regid     - Register id of a word to be modified
 *   setbits   - bits set to one will be set in the register
 *   clearbits - bits set to one will be cleared in the register
 *
 * Returned Value:
 *   on successful transaction OK is returned, otherwise ERROR is returned
 *
 ****************************************************************************/

static int ncv_set_clear_bits(FAR struct ncv7410_driver_s *priv,
                              oa_regid_t regid,
                              uint32_t setbits, uint32_t clearbits)
{
  uint32_t regval;

  if (ncv_read_reg(priv, regid, &regval))
    {
      return ERROR;
    }

  regval |= setbits;
  regval &= ~clearbits;

  if (ncv_write_reg(priv, regid, regval))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_poll_footer
 *
 * Description:
 *   poll a data transaction chunk footer while not reading nor writing
 *   frame data
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
  uint32_t txdata[NCV_CHUNK_DEFAULT_SIZE / 4];
  uint32_t rxdata[NCV_CHUNK_DEFAULT_SIZE / 4];
  uint32_t header;
  *footer = 0;

  header =   (1 << OA_DNC_POS)   /* Data Not Control */
           | (1 << OA_NORX_POS); /* No Read */

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

  if (oa_header_bad(*footer))
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
  SPI_EXCHANGE(priv->spi, txbuf,
               &rxbuf[4], NCV_CHUNK_DEFAULT_PAYLOAD_SIZE - 4);
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

  if (oa_header_bad(*footer))
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

  /* set DIOs to default */

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

  /* setup LEDs DIO0: txrx blink
   *            DIO1: link enabled and link status up
   */

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

  regval = 0x1fbf & ~(1 << OA_IMSK0_RXBOEM_POS);

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

  uint32_t setbits;

  ninfo("Enabling ncv7410\n");

  /* enable RX and TX in PHY */

  setbits = (1 << OA_PHY_CONTROL_LCTL_POS);

  if (ncv_set_clear_bits(priv, OA_PHY_CONTROL_REGID, setbits, 0))
    {
      return ERROR;
    }

  /* enable PHY interrupt */

  setbits = (1 << OA_IMSK0_PHYINTM_POS);

  if (ncv_set_clear_bits(priv, OA_IMSK0_REGID, setbits, 0))
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

  uint32_t clearbits;

  ninfo("Disabling ncv7410\n");

  /* disable PHY interrupt */

  clearbits = (1 << OA_IMSK0_PHYINTM_POS);

  if (ncv_set_clear_bits(priv, OA_IMSK0_REGID, 0, clearbits))
    {
      return ERROR;
    }

  /* disable RX and TX in PHY */

  clearbits = (1 << OA_PHY_CONTROL_LCTL_POS);

  if (ncv_set_clear_bits(priv, OA_PHY_CONTROL_REGID, 0, clearbits))
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: ncv_init_mac_addr
 *
 * Description:
 *   read the hardware MAC address and copy it into network device struct
 *
 * Input Parameters:
 *   priv - pointer to the driver specific data structure
 *
 * Returned Value:
 *   on success OK is returned, otherwise ERROR is returned
 *
 ****************************************************************************/

static int ncv_init_mac_addr(FAR struct ncv7410_driver_s *priv)
{
  uint32_t regval;
  uint8_t  mac[6];

  if (ncv_read_reg(priv, OA_PHYID_REGID, &regval))
    {
      return ERROR;
    }

  mac[0] = ncv_bitrev8(regval >> 26);
  mac[1] = ncv_bitrev8(regval >> 18);
  mac[2] = ncv_bitrev8(regval >> 10);

  if (ncv_read_reg(priv, NCV_MACID1_REGID, &regval))
    {
      return ERROR;
    }

  mac[3] = regval;

  if (ncv_read_reg(priv, NCV_MACID0_REGID, &regval))
    {
      return ERROR;
    }

  mac[4] = regval >> 8;
  mac[5] = regval;

  memcpy(&priv->dev.netdev.d_mac.ether, &mac, sizeof(struct ether_addr));

  return OK;
}

/****************************************************************************
 * Name: ncv_reset_driver_buffers
 *
 * Description:
 *   set driver's buffers to the initial setup
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   priv - pointer to the driver specific data structure
 *
 * Assumptions:
 *   called with locked mutex
 *
 ****************************************************************************/

static void ncv_reset_driver_buffers(FAR struct ncv7410_driver_s *priv)
{
  priv->txc = 0;
  priv->rxa = 0;

  if (priv->tx_pkt)
    {
      netpkt_free(&priv->dev, priv->tx_pkt, NETPKT_TX);
      priv->tx_pkt = NULL;
    }

  if (priv->rx_pkt)
    {
      netpkt_free(&priv->dev, priv->rx_pkt, NETPKT_RX);
      priv->rx_pkt = NULL;
    }

  priv->tx_pkt_idx = 0;
  priv->rx_pkt_idx = 0;
  priv->tx_pkt_len = 0;
  priv->rx_pkt_ready = false;
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

/****************************************************************************
 * Netdev upperhalf callbacks
 ****************************************************************************/

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

  if (priv->ifstate == NCV_INIT_UP)
    {
      nerr("Tried to bring ncv7410 interface up when already up\n");
      return -EINVAL;
    }

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

  /* set NCV_INIT_UP prior to enabling to allow ncv_interrupt_work right after
   * MAC-PHY enable
   */

  priv->ifstate = NCV_INIT_UP;

  if (ncv_enable(priv) == ERROR)
    {
      nerr("Error enabling ncv7410\n");
      priv->ifstate = NCV_INIT_DOWN;
      return -EIO;
    }

  /* schedule interrupt work to initialize txc and rxa */

  work_queue(NCVWORK, &priv->interrupt_work, ncv_interrupt_work, priv, 0);

  return OK;
}

static int ncv7410_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) dev;

  ncv_mutex_lock(&priv->mutex);

  if (priv->ifstate != NCV_INIT_UP)
    {
      ncv_mutex_unlock(&priv->mutex);
      nerr("Tried to bring ncv7410 interface down when already down\n");
      return -EINVAL;
    }

  work_cancel(NCVWORK, &priv->interrupt_work);
  work_cancel(NCVWORK, &priv->io_work);

  if (ncv_disable(priv) == ERROR)
    {
      ncv_mutex_unlock(&priv->mutex);
      nerr("Error disabling ncv7410\n");
      return -EIO;
    }

  ncv_reset_driver_buffers(priv);

  priv->ifstate = NCV_INIT_DOWN;

  ncv_mutex_unlock(&priv->mutex);

  return OK;
}

static int ncv7410_transmit(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) dev;

  ncv_mutex_lock(&priv->mutex);

  if (priv->tx_pkt != NULL || priv->ifstate != NCV_INIT_UP)
    {
      /* previous tx packet was not yet sent to the network
       * or the interface was shut down while waiting for the mutex
       */

      ncv_mutex_unlock(&priv->mutex);
      return -EAGAIN;
    }

  priv->tx_pkt_idx = 0;
  priv->tx_pkt_len = netpkt_getdatalen(dev, pkt);
  priv->tx_pkt = pkt;

  ncv_mutex_unlock(&priv->mutex);

  work_queue(NCVWORK, &priv->io_work, ncv_io_work, priv, 0);
  return OK;
}

static FAR netpkt_t *ncv7410_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *) dev;

  netpkt_t *retval;

  ncv_mutex_lock(&priv->mutex);

  if (priv->rx_pkt_ready)
    {
      priv->rx_pkt_ready = false;
      retval = priv->rx_pkt;
      priv->rx_pkt = NULL;
      ncv_mutex_unlock(&priv->mutex);
      return retval;
    }

  ncv_mutex_unlock(&priv->mutex);

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  /* TODO: add ioctl */
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
 *   irq - irq number of the pin connected to NCV7410's interrupt signal
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

  if (ncv_init_mac_addr(priv))
    {
      nerr("Error initializing ncv7410 MAC address\n");
      retval = -EIO;
      goto errout;
    }

  ninfo("Initializing MAC address OK\n");

  /* attach ISR */

  irq_attach(priv->irqnum, ncv_interrupt, priv);

  /* init mutex if needed */

#ifdef NCV_MUTEX
  nxmutex_init(&priv->mutex);
#endif

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
