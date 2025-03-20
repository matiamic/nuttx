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
#include <time.h>
/* #include <string.h> */
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <sys/endian.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
/* #include <nuttx/wdog.h> */
/* #include <nuttx/wqueue.h> */
/* #include <nuttx/clock.h> */
#include <nuttx/net/ncv7410.h>
#include <nuttx/net/netdev_lowerhalf.h>

// DEBUG
#include <unistd.h>

#include "ncv7410.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* NCV7410 Configuration Settings:
 *
 * CONFIG_NCV7410 - Enabled NCV7410 support
 * CONFIG_NCV7410_FREQUENCY   - Define to use a different bus frequency
 * CONFIG_NCV7410_DUMP_PACKET - dump packets to the console
 */

#define ENCWORK LPWORK

#ifdef CONFIG_NCV7410_DUMPPACKET
#  define ncv_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define ncv_dumppacket(m,a,n)
#endif

/* Timing *******************************************************************/

/* TX timeout = 1 minute */

#define NCV7410_TXTIMEOUT (60*CLOCKS_PER_SECOND)

/* Packet Memory ************************************************************/
#define NCV7410_PKTBUF_SIZE     100 //2048
#define NCV7410_PKTBUF_SIZE     100 //2048

/* this will probably need some tweaking */

#define NCV7410_TX_QUOTA        1
#define NCV7410_RX_QUOTA        1

#if CONFIG_IOB_NBUFFERS < (NCV7410_TX_QUOTA + NCV7410_RX_QUOTA)
#  error CONFIG_IOB_NBUFFERS must be > (NCV7410_TX_QUOTA + NCV7410_RX_QUOTA)
#endif

#if CONFIG_IOB_BUFSIZE < NCV7410_PKTBUF_SIZE
#  error CONFIG_IOB_BUFSIZE must be > NCV7410_PKTBUF_SIZE
#endif

/* Orher ********************************************************************/

#define NCV_RESET_TRIES 5

/* Debug ********************************************************************/

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

struct ncv7410_driver_s
{
  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s dev;

  /* This is the contained SPI driver instance */

  FAR struct spi_dev_s *spi;

  /* irq number of the interrupt signal pin */

  int irqnum;

  /* Work to be performed on the interrupt activation */

  struct work_s work;

  /* Driver state as in ncv_ifstate */

  uint8_t ifstate;

  /* struct wdog_s         txtimeout;     /1* TX timeout timer *1/ */

  /* Pointers to rx and tx buffers */

  FAR netpkt_t *tx_pkt;
  FAR netpkt_t *rx_pkt;
  bool rx_pkt_ready;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

static int ncv_get_parity(uint32_t w);

/* Low-level SPI helpers */

/* SPI control register access */

/* SPI buffer transfers */

/* PHY register access */

/* Common TX logic */

/* Interrupt handling */

static int ncv_test_isr(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

/* NuttX callback functions */

/* Initialization */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ncv_test_isr
 *
 * Description:
 *   Test whether the ISR is being invoked correctly
 *
 * Input Parameters:
 *   input parameters are not used
 *
 * Returned Value:
 *   zero
 *
 * Assumptions:
 *
 ****************************************************************************/

static int ncv_test_isr(int irq, FAR void *context, FAR void *arg)
{
  ninfo("ncv7410 interrupt service routine called\n");
  return 0;
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
  SPI_SETMODE(priv->spi, OA_TC6_SPI_MODE);
  SPI_SETBITS(priv->spi, OA_TC6_SPI_NBITS);
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
 *   priv - pointer to the driver specific data structure
 *   mms  - Memory Map Selector
 *   addr - Address in the selected MMS region
 *   word - 32-bit word to be written to the register
 *
 * Returned Value:
 *   on successful transaction 0 is returned, otherwise 1 is returned
 *
 ****************************************************************************/

static int ncv_write_reg(FAR struct ncv7410_driver_s *priv,
                     uint8_t mms, uint16_t addr, uint32_t word)
{
  uint32_t txdata[3] = { 0 };
  uint32_t rxdata[3] = { 0 };

  // prepare header
  uint32_t header =   (1    << CTP_WNR_POS)   // write-not-read
                    | (mms  << CTP_MMS_POS)   // mms
                    | (addr << CTP_ADDR_POS); // address
  int parity = ncv_get_parity(header);
  header |= parity ? 0 : CTP_P_MASK;  // make header odd parity

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
 *   mms  - Memory Map Selector
 *   addr - Address in the selected MMS region
 *   word - pointer to 32-bit variable into which the register will be stored
 *
 * Returned Value:
 *   on successful transaction 0 is returned, otherwise 1 is returned
 *
 ****************************************************************************/

static int ncv_read_reg(FAR struct ncv7410_driver_s *priv,
                     uint8_t mms, uint16_t addr, FAR uint32_t *word)
{
  uint32_t txdata[3] = { 0 };
  uint32_t rxdata[3] = { 0 };
  int parity;
  uint32_t header;

  // prepare header
  header =   (mms  << CTP_MMS_POS)   // mms
           | (addr << CTP_ADDR_POS); // address
  parity = ncv_get_parity(header);
  header |= parity ? 0 : CTP_P_MASK;  // make header odd parity

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
      return 1;  // error
    }
  ninfo("Reading register OK\n");
  return 0;
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
 *   on successful reset 0 is returned, otherwise 1 is returned
 *
 ****************************************************************************/

static int ncv_reset(FAR struct ncv7410_driver_s *priv)
{
  int tries = NCV_RESET_TRIES;
  uint32_t regval = (1 << RESET_SWRESET_POS);

  if (ncv_write_reg(priv, RESET_MMS, RESET_ADDR, regval)) return 1;
  do
    {
      if (ncv_read_reg(priv, RESET_MMS, RESET_ADDR, &regval)) return 1;
    }
  while (tries-- && (regval & 1));
  if (regval & 1)
    {
      return 1;
    }

  // clear HDRE in STATUS0 (due to a bug in ncv7410)
  if (ncv_write_reg(priv, STATUS0_MMS, STATUS0_ADDR, (1 << STATUS0_HDRE_POS))) return 1;

  // clear reset complete flag
  if (ncv_write_reg(priv, STATUS0_MMS, STATUS0_ADDR, (1 << STATUS0_RESETC_POS))) return 1;

  // blink with LEDs for debugging purposes
  for (int i = 0; i < 4; i++)
    {
      regval = 0x0302;
      if (ncv_write_reg(priv, DIO_CONFIG_REG_MMS, DIO_CONFIG_REG_ADDR, regval)) return 1;
      usleep(250000);
      regval = 0x0203;
      if (ncv_write_reg(priv, DIO_CONFIG_REG_MMS, DIO_CONFIG_REG_ADDR, regval)) return 1;
      usleep(250000);
    }

  // set DIOs to default
  regval = 0x6060;
  if (ncv_write_reg(priv, DIO_CONFIG_REG_MMS, DIO_CONFIG_REG_ADDR, regval)) return 1;
  return 0;
}

/* Netdev upperhalf callback functions */

static int ncv7410_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  return 0;
}

static int ncv7410_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  return 0;
}

static int ncv7410_transmit(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt)
{
  return 0;
}

static FAR netpkt_t *ncv7410_receive(FAR struct netdev_lowerhalf_s *dev)
{
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

#if CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD > 0
static void ncv7410_reclaim(FAR struct netdev_lowerhalf_s *dev)
{
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
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ncv7410_initialize
 *
 * Description:
 *   Initialize the Ethernet driver.  The NCV7410 device is assumed to be
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

  priv->spi = spi;                     /* Save the SPI instance */
  priv->irqnum = irq;                   /* Save the low-level MCU interface */

  /* reset ncv7410 chip */

  if (ncv_reset(priv))
    {
      nerr("Error resetting ncv7410\n");
      retval = -EIO;
      goto errout;
    }
  priv->ifstate = ENC_RESET;
  ninfo("Resetting ncv7410 OK\n");

  /* attach testing ISR */

  irq_attach(priv->irqnum, ncv_test_isr, NULL);

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
