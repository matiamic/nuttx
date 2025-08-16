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

#include <string.h>

#include <debug.h>

#include <nuttx/kmalloc.h>

#include "oa_tc6.h"
#include "oa_tc6_ncv7410.h"

/*****************************************************************************
 * Preprocessor Macros
 ****************************************************************************/

#define NCV_ADDR_FILTER_SLOTS 4
#define NCV_ADDR_FILTER_FULL  0xf

/*****************************************************************************
 * Private Types
 ****************************************************************************/

struct ncv7410_addrfilter
{
  uint8_t addrs[NCV_ADDR_FILTER_SLOTS][6]; /* Addrs that pass the filter    */
  uint8_t active;                          /* On/Off status of the slots
                                              LSB represents the first slot
                                              1 pass if match, 0 inactive   */
};

struct ncv7410_driver_s
{
  struct oa_tc6_driver_s oa_tc6_dev;

  struct ncv7410_addrfilter filter;
};

/*****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static int ncv7410_init_mac_addr(FAR struct ncv7410_driver_s *priv);
static int ncv7410_refresh_mac_filter(FAR struct ncv7410_driver_s *priv);
static int ncv7410_set_filter_slot(FAR struct ncv7410_driver_s *priv,
                                   FAR const uint8_t *mac,
                                   int slot);
static int ncv7410_config(FAR struct ncv7410_driver_s *priv);
static int ncv7410_enable(FAR struct ncv7410_driver_s *priv);
static int ncv7410_disable(FAR struct ncv7410_driver_s *priv);

/* OA-TC6 lower callbacks */

static int ncv7410_action(FAR struct oa_tc6_driver_s *dev,
                          enum oa_tc6_action_e action);
static int ncv7410_addmac(FAR struct oa_tc6_driver_s *dev,
                          FAR const uint8_t *mac);
#ifdef CONFIG_NET_MCASTGROUP
static int ncv7410_rmmac(FAR struct oa_tc6_driver_s *dev,
                         FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int ncv7410_ioctl(FAR struct oa_tc6_driver_s *dev, int cmd,
                         unsigned long arg);
#endif

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ncv7410_init_mac_addr(FAR struct ncv7410_driver_s *priv)
{
  FAR struct oa_tc6_driver_s *dev = &priv->oa_tc6_dev;

  uint32_t regval;
  uint8_t  mac[6];

  if (oa_tc6_read_reg(dev, OA_TC6_PHYID_REGID, &regval))
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

  oa_tc6_store_mac_addr(dev, mac);

  return OK;
}

static int ncv7410_refresh_mac_filter(FAR struct ncv7410_driver_s *priv)
{
  /* Write all filter slots marked as active into the MAC-PHY */

  uint8_t active = priv->filter.active;
  int i;

  for (i = 0; i < NCV_ADDR_FILTER_SLOTS; i++)
    {
      if (active & (1 << i))
        {
          FAR uint8_t *mac = priv->filter.addrs[i];
          if (ncv7410_set_filter_slot(priv, mac, i))
            {
              return ERROR;
            }
        }
    }

  return OK;
}

static int ncv7410_set_filter_slot(FAR struct ncv7410_driver_s *priv,
                                   FAR const uint8_t *mac,
                                   int slot)
{
  FAR struct oa_tc6_driver_s *dev = &priv->oa_tc6_dev;
  uint32_t regval;

  /* Write to the MAC-PHY */

  regval =   (mac[2] << 24)
           | (mac[3] << 16)
           | (mac[4] << 8)
           | (mac[5]);

  if (oa_tc6_write_reg(dev, NCV_ADDRFILTL_REGID(slot), regval))
    {
      nerr("Error: Error during SPI transmission\n");
      return ERROR;
    }

  regval =   (1 << 31)  /* Enable filter */
           | (mac[0] << 8)
           | (mac[1]);

  if (oa_tc6_write_reg(dev, NCV_ADDRFILTH_REGID(slot), regval))
    {
      nerr("Error: Error during SPI transmission\n");
      return ERROR;
    }

  /* All fields are significant */

  regval = 0xffffffff;

  if (oa_tc6_write_reg(dev, NCV_ADDRMASKL_REGID(slot), regval))
    {
      nerr("Error: Error during SPI transmission\n");
      return ERROR;
    }

  regval = 0x0000ffff;

  if (oa_tc6_write_reg(dev, NCV_ADDRMASKH_REGID(slot), regval))
    {
      nerr("Error: Error during SPI transmission\n");
      return ERROR;
    }

  return OK;
}

static int ncv7410_config(FAR struct ncv7410_driver_s *priv)
{
  FAR struct oa_tc6_driver_s *dev = &priv->oa_tc6_dev;

  uint32_t regval;

  ninfo("Configuring NCV7410\n");

  /* Refresh the MAC address filter in case the configuration is called
   * due to the MAC-PHY losing its configuration */

  if (ncv7410_refresh_mac_filter(priv))
    {
      return ERROR;
    }

  /* setup LEDs DIO0: txrx blink
   *            DIO1: link enabled and link status up
   */

  regval =   (NCV_DIO_TXRX_FUNC << NCV_DIO0_FUNC_POS)
           | (NCV_DIO_LINK_CTRL_FUNC << NCV_DIO1_FUNC_POS)
           | (1 << NCV_DIO0_OUT_VAL_POS)
           | (1 << NCV_DIO1_OUT_VAL_POS);

  if (oa_tc6_write_reg(dev, NCV_DIO_CONFIG_REGID, regval))
    {
      return ERROR;
    }

  /* Enable MAC TX, RX, enable transmit FCS computation on MAC,
   * Enable MAC address filtering if not promiscuous
   */

  regval =   (1 << NCV_MAC_CONTROL0_FCSA_POS)
           | (1 << NCV_MAC_CONTROL0_TXEN_POS)
           | (1 << NCV_MAC_CONTROL0_RXEN_POS);

#ifndef CONFIG_NET_PROMISCUOUS
  regval |= 1 << NCV_MAC_CONTROL0_ADRF_POS;
#endif

  if (oa_tc6_write_reg(dev, NCV_MAC_CONTROL0_REGID, regval))
    {
      return ERROR;
    }

  return OK;
}

static int ncv7410_enable(FAR struct ncv7410_driver_s *priv)
{
  FAR struct oa_tc6_driver_s *dev = &priv->oa_tc6_dev;

  uint32_t setbits;

  ninfo("Info: Enabling NCV7410 PHY's TX and RX\n");

  setbits = (1 << NCV_PHY_CONTROL_LCTL_POS);

  if (oa_tc6_set_clear_bits(dev, OA_TC6_PHY_CONTROL_REGID, setbits, 0))
    {
      return ERROR;
    }

  return OK;
}

static int ncv7410_disable(FAR struct ncv7410_driver_s *priv)
{
  FAR struct oa_tc6_driver_s *dev = &priv->oa_tc6_dev;

  uint32_t clearbits;

  ninfo("Info: Disabling NCV7410 PHY's TX and RX\n");

  clearbits = (1 << NCV_PHY_CONTROL_LCTL_POS);

  if (oa_tc6_set_clear_bits(dev, OA_TC6_PHY_CONTROL_REGID, 0, clearbits))
    {
      return ERROR;
    }

  return OK;
}

static int ncv7410_action(FAR struct oa_tc6_driver_s *dev,
                          enum oa_tc6_action_e action)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)dev;

  switch (action)
    {
      case OA_TC6_ACTION_CONFIG:
          return ncv7410_config(priv);

      case OA_TC6_ACTION_ENABLE:
          return ncv7410_enable(priv);

      case OA_TC6_ACTION_DISABLE:
          return ncv7410_disable(priv);

      case OA_TC6_ACTION_EXST:
          break;

      default:
          nerr("Error: Unknown OA-TC6 lower action code\n");
          return ERROR;
    }

  return OK;
}

static int ncv7410_addmac(FAR struct oa_tc6_driver_s *dev,
                          FAR const uint8_t *mac)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)dev;
  uint8_t active = priv->filter.active;
  int i;

  /* Check if there is a free slot in the filter */

  if (active == NCV_ADDR_FILTER_FULL)
    {
      nerr("Error: The address filter is already full\n");
      return -EINVAL;
    }

  /* Check if the addr is already included */

  for (i = 0; i < NCV_ADDR_FILTER_SLOTS; i++)
    {
      if (((active >> i) & 1) && memcmp(priv->filter.addrs[i], mac, 6) == 0)
        {
          nwarn("Warning: The provided address is already in the slot %d "
                "of the filter\n", i);
          return OK;
        }
    }

  /* Find the first free slot */

  for (i = 0; i < NCV_ADDR_FILTER_SLOTS; i++)
    {
      if (((active >> i) & 1) == 0)
        {
          break;
        }
    }

  if (ncv7410_set_filter_slot(priv, mac, i))
    {
      nerr("Error setting filter slot\n");
      return -EIO;
    }

  /* Update the filter structure */

  memcpy(priv->filter.addrs[i], mac, 6);
  active |= 1 << i;
  priv->filter.active = active;

  ninfo("Info: Adding new MAC address to the filter slot %d OK\n", i);

  return OK;
}

#ifdef CONFIG_NET_MCASTGROUP
static int ncv7410_rmmac(FAR struct oa_tc6_driver_s *dev,
                         FAR const uint8_t *mac)
{
  FAR struct ncv7410_driver_s *priv = (FAR struct ncv7410_driver_s *)dev;
  uint8_t active = priv->filter.active;
  uint32_t regval;
  int i;

  for (i = 0; i < NCV_ADDR_FILTER_SLOTS; i++)
    {
      if (((active >> i) & 1) && memcmp(priv->filter.addrs[i], mac, 6) == 0)
        {
          break;
        }

      if (i == NCV_ADDR_FILTER_SLOTS - 1)
        {
          nwarn("Warning: The address is not present in the filter\n");
          return OK;
        }
    }

  /* Clear the ADDRFILT0H, where enable flag is located */

  regval = 0;

  if (oa_tc6_write_reg(dev, NCV_ADDRFILTH_REGID(i), regval))
    {
      nerr("Error: Error during SPI transmission\n");
      return -EIO;
    }

  /* Update the filter structure */
  active &= ~(1 << i);
  priv->filter.active = active;

  ninfo("Info: Removing the MAC address from the filter slot %d OK\n", i);

  return OK;
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int ncv7410_ioctl(FAR struct oa_tc6_driver_s *dev, int cmd,
                         unsigned long arg)
{
  return OA_TC6_IOCTL_CMD_NOT_IMPLEMENTED;
}
#endif

/*****************************************************************************
 * Private Data
 ****************************************************************************/

static struct oa_tc6_ops_s g_ncv7410_ops =
{
  ncv7410_action,
  ncv7410_addmac,
#ifdef CONFIG_NET_MCASTGROUP
  ncv7410_rmmac,
#endif
#ifdef CONFIG_NETDEV_IOCTL
  ncv7410_ioctl
#endif
};

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

int ncv7410_initialize(FAR struct spi_dev_s *spi,
                       FAR struct oa_tc6_config_s *config)
{
  FAR struct ncv7410_driver_s *priv = NULL;
  FAR struct oa_tc6_driver_s *dev;
  int retval;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      nerr("Error: Could not allocate memory for ncv7410_driver_s priv\n");
      return -ENOMEM;
    }

  dev = &priv->oa_tc6_dev;

  /* Save the ops pointer */

  dev->ops = &g_ncv7410_ops;

  retval = oa_tc6_common_init(dev, spi, config);
  if (retval)
    {
      nerr("Error: OA-TC6 common initialization failed\n");
      goto errout;
    }

  /* Clear HDRE in STATUS0 (due to a bug in NCV7410) */

  if (oa_tc6_write_reg(dev, OA_TC6_STATUS0_REGID,
                       1 << OA_TC6_STATUS0_HDRE_POS))
    {
      nerr("Error: Clearing HDRE bit in STATUS0 failed\n");
      retval = -EIO;
      goto errout;
    }

  /* Init MAC address */

  if (ncv7410_init_mac_addr(priv))
    {
      nerr("Error: Initialization of the MAC address failed\n");
      retval = -EIO;
      goto errout;
    }

  /* Do something with additional structure fields or with the device
   * if needed
   */

  retval = oa_tc6_register(dev);
  if (retval == OK)
    {
      ninfo("Successfully registered OA-TC6 network driver\n");
      return OK;
    }

  nerr("Error: Registration of the OA-TC6 driver failed: %d\n", retval);

errout:
  kmm_free(priv);
  return retval;
}
