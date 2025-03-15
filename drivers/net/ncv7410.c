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
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/net/enc28j60.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

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

/* CONFIG_NET_ETH_PKTSIZE must always be defined */

#if !defined(CONFIG_NET_ETH_PKTSIZE) && (CONFIG_NET_ETH_PKTSIZE <= MAX_FRAMELEN)
#  error "CONFIG_NET_ETH_PKTSIZE is not valid for the ENC28J60"
#endif

/* We need to have the work queue to handle SPI interrupts */

#if !defined(CONFIG_SCHED_LPWORK)
#  error "Low priority worker thread support is required (CONFIG_SCHED_LPWORK)"
#endif

#define ENCWORK LPWORK

#ifdef CONFIG_NCV7410_DUMPPACKET
#  define ncv7410_dumppacket(m,a,n) lib_dumpbuffer(m,a,n)
#else
#  define ncv7410_dumppacket(m,a,n)
#endif

/* Timing *******************************************************************/

/* TX timeout = 1 minute */

#define NCV7410_TXTIMEOUT (60*CLK_TCK)

/* Poll timeout */

#define NCV7410_POLLTIMEOUT MSEC2TICK(50)

/* Packet Memory ************************************************************/

/* Packet memory layout */

#define ALIGNED_BUFSIZE ((CONFIG_NET_ETH_PKTSIZE + 255) & ~255)

/* Work around Errata #5 (spurious reset of ERXWRPT to 0) by placing the RX
 * FIFO at the beginning of packet memory.
 */

#define ERRATA5 1
#if ERRATA5
#  define PKTMEM_RX_START 0x0000                            /* RX buffer must be at addr 0 for errata 5 */
#  define PKTMEM_RX_END   (PKTMEM_END-ALIGNED_BUFSIZE)      /* RX buffer length is total SRAM minus TX buffer */
#  define PKTMEM_TX_START (PKTMEM_RX_END+1)                 /* Start TX buffer after */
#  define PKTMEM_TX_ENDP1 (PKTMEM_TX_START+ALIGNED_BUFSIZE) /* Allow TX buffer for one frame */
#else
#  define PKTMEM_TX_START 0x0000                            /* Start TX buffer at 0 */
#  define PKTMEM_TX_ENDP1 ALIGNED_BUFSIZE                   /* Allow TX buffer for one frame */
#  define PKTMEM_RX_START PKTMEM_TX_ENDP1                   /* Followed by RX buffer */
#  define PKTMEM_RX_END   PKTMEM_END                        /* RX buffer goes to the end of SRAM */
#endif

/* Packet buffer size */

#define PKTBUF_SIZE (MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((FAR struct eth_hdr_s *)priv->dev.d_buf)

/* Debug ********************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the interface */

enum enc_state_e
{
  ENCSTATE_UNINIT = 0,                /* The interface is in an uninitialized state */
  ENCSTATE_DOWN,                      /* The interface is down */
  ENCSTATE_UP                         /* The interface is up */
};

/* The enc_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct ncv7410_driver_s
{
  /* Device control */

  uint8_t               ifstate;       /* Interface state:  See ENCSTATE_* */
  uint16_t              nextpkt;       /* Next packet address */

  /* Timing */

  struct wdog_s         txtimeout;     /* TX timeout timer */

  struct work_s         irqwork;       /* Interrupt continuation work queue support */
  struct work_s         towork;        /* Tx timeout work queue support */
  struct work_s         pollwork;      /* Poll timeout work queue support */

  /* This is the contained SPI driver instance */

  FAR struct spi_dev_s *spi;

  /* irq number of the interrupt signal pin */

  int irqnum;

  /* This holds the information visible to the NuttX network */

  struct net_driver_s   dev;          /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A single packet buffer is used */

static uint16_t g_pktbuf[(PKTBUF_SIZE + 1) / 2];

/* Driver status structure */

static struct ncv7410_driver_s g_ncv7410;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpers */

/* SPI control register access */

/* SPI buffer transfers */

/* PHY register access */

/* Common TX logic */

/* Interrupt handling */

/* Watchdog timer expirations */

/* NuttX callback functions */

/* Initialization */

/****************************************************************************
 * Private Functions
 ****************************************************************************/
int test_isr(int irq, FAR void *context, FAR void *arg)
{
  ninfo("ncv7410 interrupt service routine called\n");
  return 0;
}

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
  FAR struct ncv7410_driver_s *priv;

  priv = &g_ncv7410;

  /* Initialize the driver structure */

  memset(&g_ncv7410, 0, sizeof(struct ncv7410_driver_s));

  priv->dev.d_buf     = (FAR uint8_t *)g_pktbuf; /* Single packet buffer */
  /* priv->dev.d_ifup    = enc_ifup;                /1* I/F down callback *1/ */
  /* priv->dev.d_ifdown  = enc_ifdown;              /1* I/F up (new IP address) callback *1/ */
  /* priv->dev.d_txavail = enc_txavail;             /1* New TX data callback *1/ */
/* #ifdef CONFIG_NET_MCASTGROUP */
  /* priv->dev.d_addmac  = enc_addmac;              /1* Add multicast MAC address *1/ */
  /* priv->dev.d_rmmac   = enc_rmmac;               /1* Remove multicast MAC address *1/ */
/* #endif */
  priv->dev.d_private = priv;                    /* Used to recover private state from dev */
  priv->spi           = spi;                     /* Save the SPI instance */
  priv->irqnum        = irq;                   /* Save the low-level MCU interface */

  /* priv->ifstate = ENCSTATE_UNINIT; */

  /* TODO: inintialize ncv7410 chip */

  /* Register the device with the OS so that socket IOCTLs can be performed */
  irq_attach(priv->irqnum, test_isr, NULL);

  return netdev_register(&priv->dev, NET_LL_ETHERNET);
}

#endif /* CONFIG_NET && CONFIG_NCV7410_NET */

