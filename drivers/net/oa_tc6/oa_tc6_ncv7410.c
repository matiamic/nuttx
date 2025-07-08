/****************************************************************************
 * drivers/net/oa/oa_tc6_ncv7410.h
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

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>

#include <nuttx/kmalloc.h>

#include "oa_tc6.h"
#include "oa_tc6_ncv7410.h"

/*****************************************************************************
 * Private Types
 ****************************************************************************/

struct oa_tc6_ncv7410_driver_s
{
  struct oa_tc6_driver_s oa_tc6_dev;

  int somethingmore;
};

/*****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static int oa_tc6_ncv7410_init_mac(struct oa_tc6_ncv7410_driver_s *priv);
static int oa_tc6_ncv7410_config(struct oa_tc6_ncv7410_driver_s *priv);

/* OA-TC6 lower callbacks */

static int oa_tc6_ncv7410_action(struct oa_tc6_driver_s *dev,
                                 enum oa_tc6_action_e action);
static int oa_tc6_ncv7410_rm_mac(struct oa_tc6_driver_s *dev, uint8_t *mac);
static int oa_tc6_ncv7410_add_mac(struct oa_tc6_driver_s *dev, uint8_t *mac);
static int oa_tc6_ncv7410_rm_mac(struct oa_tc6_driver_s *dev, uint8_t *mac);
static int oa_tc6_ncv7410_ioctl(struct oa_tc6_driver_s *dev, int cmd,
                                unsigned long arg);

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

static int oa_tc6_ncv7410_init_mac(struct oa_tc6_ncv7410_driver_s *priv)
{
  struct oa_tc6_driver_s *dev = (oa_tc6_driver_s *)priv;

  uint32_t regval;
  uint8_t  mac[6];

  if (oa_tc6_read_reg(dev, OA_PHYID_REGID, &regval))
    {
      return ERROR;
    }

  mac[0] = oa_tc6_bitrev8(regval >> 26);
  mac[1] = oa_tc6_bitrev8(regval >> 18);
  mac[2] = oa_tc6_bitrev8(regval >> 10);

  if (oa_tc6_read_reg(dev, NCV_MACID1_REGID, &regval))
    {
      return ERROR;
    }

  mac[3] = regval;

  if (oa_tc6_read_reg(dev, NCV_MACID0_REGID, &regval))
    {
      return ERROR;
    }

  mac[4] = regval >> 8;
  mac[5] = regval;

  oa_tc6_store_mac(dev, mac);

  return OK;
}

static int oa_tc6_ncv7410_config(struct oa_tc6_ncv7410_driver_s *priv)
{
  struct oa_tc6_driver_s *dev = (oa_tc6_driver_s *)priv;

  uint32_t regval;

  ninfo("Configuring NCV7410\n");

  /* setup LEDs DIO0: txrx blink
   *            DIO1: link enabled and link status up
   */

  regval =   (NCV_DIO_TXRX_FUNC << NCV_DIO0_FUNC_POS)
           | (NCV_DIO_LINK_CTRL_FUNC << NCV_DIO1_FUNC_POS)
           | (1 << NCV_DIO0_OUT_VAL_POS)
           | (1 << NCV_DIO1_OUT_VAL_POS);

  if (ncv_write_reg(dev, NCV_DIO_CONFIG_REGID, regval))
    {
      return ERROR;
    }

  /* enable MAC TX, RX, enable transmit FCS computation on MAC,
   * enable MAC address filtering
   */

  regval =   (1 << NCV_MAC_CONTROL0_FCSA_POS)
           | (1 << NCV_MAC_CONTROL0_TXEN_POS)
           | (1 << NCV_MAC_CONTROL0_RXEN_POS)
           | (1 << NCV_MAC_CONTROL0_ADRF_POS);

  if (ncv_write_reg(dev, NCV_MAC_CONTROL0_REGID, regval))
    {
      return ERROR;
    }

  return OK;
}

static int oa_tc6_ncv7410_action(struct oa_tc6_driver_s *dev,
                                 enum oa_tc6_action_e action)
{
  struct oa_tc6_ncv7410_driver_s *priv = (struct oa_tc6_ncv7410_driver_s *)dev;

  switch (action)
    {
      case OA_TC6_ACTION_CONFIG:
          return oa_tc6_ncv7410_config(priv);
      case OA_TC6_ACTION_INIT_MAC:
          return oa_tc6_ncv7410_init_mac(priv);
      case OA_TC6_ACTION_IFUP:
      case OA_TC6_ACTION_IFDOWN:
      case OA_TC6_ACTION_EXST:
          break;
      default:
          nerr("Unknown OA-TC6 lower action number\n");
    }

  return OK;
}

static int oa_tc6_ncv7410_add_mac(struct oa_tc6_driver_s *dev, uint8_t *mac)
{
  struct oa_tc6_ncv7410_driver_s *priv = (struct oa_tc6_ncv7410_driver_s *)dev;

  /* do something */
  return OK;
}

static int oa_tc6_ncv7410_rm_mac(struct oa_tc6_driver_s *dev, uint8_t *mac)
{
  struct oa_tc6_ncv7410_driver_s *priv = (struct oa_tc6_ncv7410_driver_s *)dev;

  /* do something */
  return OK;
}

static int oa_tc6_ncv7410_ioctl(struct oa_tc6_driver_s *dev, int cmd,
                                unsigned long arg)
{
  struct oa_tc6_ncv7410_driver_s *priv = (struct oa_tc6_ncv7410_driver_s *)dev;

  /* do something */
  return OK;
}

/*****************************************************************************
 * Private Data
 ****************************************************************************/

static struct oa_tc6_ops_s g_oa_tc6_ncv7410_ops =
{
  oa_tc6_ncv7410_action,
  oa_tc6_ncv7410_add_mac,
  oa_tc6_ncv7410_rm_mac,
  oa_tc6_ncv7410_ioctl
};

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

struct oa_tc6_driver_s *oa_tc6_ncv7410_initialize(struct spi_dev_s *spi,
                                                  struct oa_tc6_config_s *config)
{
  FAR struct oa_tc6_ncv7410_driver_s *priv = NULL;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      nerr("Could not allocate data for oa_tc6_ncv7410_driver_s priv\n");
      return NULL;
    }

  /* Assign spi and config only if needed by the lan8650 init code, in any case it will be reassigned later in oa_tc6_initialize */

  priv->oa_tc6_dev.spi = spi;
  priv->oa_tc6_dev.config = config;

  /* Save the ops pointer */

  priv->oa_tc6_dev.ops = &g_oa_tc6_ncv7410_ops;

  /* Do something with additional structure fields or with the device */

  /* Return */

  return &priv->oa_tc6_dev;
}
