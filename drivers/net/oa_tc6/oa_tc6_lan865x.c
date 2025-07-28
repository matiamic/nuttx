/****************************************************************************
 * drivers/net/oa/oa_tc6_lan865x.h
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
#include "oa_tc6_lan865x.h"

/*****************************************************************************
 * Preprocessor Macros
 ****************************************************************************/

#define LAN865x_ADDR_FILTER_SLOTS 4
#define LAN865x_ADDR_FILTER_FULL  0xf

/*****************************************************************************
 * Private Types
 ****************************************************************************/

struct lan865x_addrfilter
{
  uint8_t addrs[LAN865x_ADDR_FILTER_SLOTS][6]; /* Addrs that pass       */
  uint8_t active;                              /* LSB is the first slot
                                                  1 active, 0 inactive  */
};

struct lan865x_driver_s
{
  struct oa_tc6_driver_s oa_tc6_dev;

  struct lan865x_addrfilter filter;
};

/*****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static int lan865x_init_mac_addr(FAR struct lan865x_driver_s *priv);
static int lan865x_config(FAR struct lan865x_driver_s *priv);
/* static int lan865x_indirect_read(FAR struct lan865x_driver_s *priv, */
/*                                  uint8_t addr, uint8_t mask, */
/*                                  FAR uint8_t *regval); */

/* OA-TC6 lower callbacks */

static int lan865x_action(FAR struct oa_tc6_driver_s *dev,
                          enum oa_tc6_action_e action);
static int lan865x_addmac(FAR struct oa_tc6_driver_s *dev,
                          FAR const uint8_t *mac);
#ifdef CONFIG_NET_MCASTGROUP
static int lan865x_rmmac(FAR struct oa_tc6_driver_s *dev,
                         FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int lan865x_ioctl(FAR struct oa_tc6_driver_s *dev, int cmd,
                         unsigned long arg);
#endif

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lan865x_init_mac_addr(FAR struct lan865x_driver_s *priv)
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

  /* TODO: Replace this with some smart MAC address generating mechanism */

  mac[3] = (uint8_t)(LAN865x_MAC_ADDRESS_LOW >> 16);
  mac[4] = (uint8_t)(LAN865x_MAC_ADDRESS_LOW >> 8);
  mac[5] = (uint8_t)LAN865x_MAC_ADDRESS_LOW;

  oa_tc6_store_mac_addr(dev, mac);

  return OK;
}

static int lan865x_config(FAR struct lan865x_driver_s *priv)
{
  FAR struct oa_tc6_driver_s *dev = &priv->oa_tc6_dev;

  uint32_t regval;

  /* AN1760 appnote variables */

  /* int err; */
  /* uint8_t value1; */
  /* uint8_t value2; */
  /* int8_t offset1; */
  /* int8_t offset2; */
  /* uint16_t cfgparam1; */
  /* uint16_t cfgparam2; */

  ninfo("Configuring LAN865x\n");

  /* Perform the configuration procedure as outlined in the AN1760 appnote */

  /* err = lan865x_indirect_read(priv, 0x04, 0x1f, &value1); */
  /* if ((value1 & 0x10) != 0) */
  /*   { */
  /*     offset1 = (int8_t) ((uint8_t)value1 - 0x20); */
  /*   } */
  /* else */
  /*   { */
  /*     offset1 = (int8_t) value1; */
  /*   } */

  /* err |= lan865x_indirect_read(priv, 0x08, 0x1f, &value2); */
  /* if ((value2 & 0x10) != 0) */
  /*   { */
  /*     offset2 = (int8_t) ((uint8_t)value2 - 0x20); */
  /*   } */
  /* else */
  /*   { */
  /*     offset2 = (int8_t) value2; */
  /*   } */

  /* cfgparam1 =   (uint16_t) (((9 + offset1) & 0x3f) << 10) */
  /*             | (uint16_t) (((14 + offset1) & 0x3f) << 4) | 0x03; */
  /* cfgparam2 = (uint16_t) (((40 + offset2) & 0x3f) << 10); */

  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00d0), 0x3f31); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00e0), 0xc000); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0084), cfgparam1); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x008a), cfgparam2); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00e9), 0x9e50); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00f5), 0x1cf8); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00f4), 0xc020); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00f8), 0xb900); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00f9), 0x4e53); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0081), 0x0080); */

  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0091), 0x9660); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x1, 0x0077), 0x0028); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0043), 0x00ff); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0044), 0xffff); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0045), 0x0000); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0053), 0x00ff); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0054), 0xffff); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0055), 0x0000); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0040), 0x0002); */
  /* err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x0050), 0x0002); */

  /* if (err) */
  /*   { */
  /*     return ERROR; */
  /*   } */

  /* Enable MAC TX, RX, disable MAC address filtering if promiscuous */

  regval =   (1 << LAN865x_MAC_NCR_TXEN_POS)
           | (1 << LAN865x_MAC_NCR_RXEN_POS);

  if (oa_tc6_write_reg(dev, LAN865x_MAC_NCR_REGID, regval))
    {
      return ERROR;
    }

#ifdef CONFIG_NET_PROMISCUOUS
  regval = 1 << LAN865x_MAC_NCFGR_CAF_POS;

  /* Use read-modify-write so reserved bits are not overridden */

  if (oa_tc6_set_clear_bits(dev, LAN865x_MAC_NCFGR_REGID, regval, 0))
    {
      return ERROR;
    }
#endif

  return OK;
}

/* static int lan865x_indirect_read(FAR struct lan865x_driver_s *priv, */
/*                                  uint8_t addr, uint8_t mask, */
/*                                  FAR uint8_t *regval) */
/* { */
/*   FAR struct oa_tc6_driver_s *dev = &priv->oa_tc6_dev; */
/*   uint32_t regval32; */
/*   int err; */

/*   err = oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00D8), addr); */
/*   err |= oa_tc6_write_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00DA), 0x2); */
/*   err |= oa_tc6_read_reg(dev, OA_TC6_MAKE_REGID(0x4, 0x00D9), &regval32); */
/*   *regval = (uint8_t)regval32 & mask; */
/*   if (err) */
/*     { */
/*       return ERROR; */
/*     } */

/*   return OK; */
/* } */

static int lan865x_action(FAR struct oa_tc6_driver_s *dev,
                          enum oa_tc6_action_e action)
{
  FAR struct lan865x_driver_s *priv = (FAR struct lan865x_driver_s *)dev;

  switch (action)
    {
      case OA_TC6_ACTION_CONFIG:
          return lan865x_config(priv);
      case OA_TC6_ACTION_IFUP:
      case OA_TC6_ACTION_IFDOWN:
      case OA_TC6_ACTION_EXST:
          break;
      default:
          nerr("Unknown OA-TC6 lower action number\n");
    }

  return OK;
}

static int lan865x_addmac(FAR struct oa_tc6_driver_s *dev,
                          FAR const uint8_t *mac)
{
  FAR struct lan865x_driver_s *priv = (FAR struct lan865x_driver_s *)dev;
  uint8_t active = priv->filter.active;
  uint32_t regval;
  int i;

  /* Check if there is a free slot in the filter */

  if (active == LAN865x_ADDR_FILTER_FULL)
    {
      nerr("Error: The address filter is already full\n");
      return -EINVAL;
    }

  /* Check if the addr is already included */

  for (i = 0; i < LAN865x_ADDR_FILTER_SLOTS; i++)
    {
      if (((active >> i) & 1) && memcmp(priv->filter.addrs[i], mac, 6) == 0)
        {
          nerr("Error: The provided address is already in the slot %d "
               "of the filter\n", i + 1);
          return -EINVAL;
        }
    }

  /* Find the first free slot */

  for (i = 0; i < LAN865x_ADDR_FILTER_SLOTS; i++)
    {
      if (((active >> i) & 1) == 0)
        {
          break;
        }
    }

  /* Write to the MAC-PHY */

  regval =   (mac[3] << 24)
           | (mac[2] << 16)
           | (mac[1] << 8)
           | (mac[0]);

  /* Must write address register (i + 1) as in the case of LAN865x
   * the MAC address filter registers are numbered from 1
   */

  if (oa_tc6_write_reg(dev, LAN865x_MAC_SAB_REGID(i + 1), regval))
    {
      nerr("Error: Error during SPI transmission\n");
      return -EIO;
    }

  regval =   (mac[5] << 8)
           | (mac[4]);

  if (oa_tc6_write_reg(dev, LAN865x_MAC_SAT_REGID(i + 1), regval))
    {
      nerr("Error: Error during SPI transmission\n");
      return -EIO;
    }

  /* Update the filter structure */

  memcpy(priv->filter.addrs[i], mac, 6);
  active |= 1 << i;
  priv->filter.active = active;

  ninfo("Info: Adding new MAC address to the filter slot %d OK\n", i + 1);

  return OK;
}

#ifdef CONFIG_NET_MCASTGROUP
static int lan865x_rmmac(FAR struct oa_tc6_driver_s *dev,
                         FAR const uint8_t *mac)
{
  FAR struct lan865x_driver_s *priv = (FAR struct lan865x_driver_s *)dev;
  uint8_t active = priv->filter.active;
  int i;

  for (i = 0; i < LAN865x_ADDR_FILTER_SLOTS; i++)
    {
      if (((active >> i) & 1) && memcmp(priv->filter.addrs[i], mac, 6) == 0)
        {
          break;
        }

      if (i == LAN865x_ADDR_FILTER_SLOTS - 1)
        {
          nerr("Error: The address is not present in the filter\n");
          return -EINVAL;
        }
    }

  /* Filter is disabled by writing the appropriate MAC_SAB register */

  if (oa_tc6_write_reg(dev, LAN865x_MAC_SAB_REGID(i + 1), 0))
    {
      nerr("Error: Error during SPI transmission\n");
      return -EIO;
    }

  /* Update the filter structure */
  active &= ~(1 << i);
  priv->filter.active = active;

  ninfo("Info: Removing the MAC address from the filter slot %d OK\n", i + 1);

  return OK;
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int lan865x_ioctl(FAR struct oa_tc6_driver_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct lan865x_driver_s *priv = (FAR struct lan865x_driver_s *)dev;

  /* do something */
  return OA_TC6_IOCTL_CMD_NOT_IMPLEMENTED;
}
#endif

/*****************************************************************************
 * Private Data
 ****************************************************************************/

static struct oa_tc6_ops_s g_lan865x_ops =
{
  lan865x_action,
  lan865x_addmac,
#ifdef CONFIG_NET_MCASTGROUP
  lan865x_rmmac,
#endif
#ifdef CONFIG_NETDEV_IOCTL
  lan865x_ioctl
#endif
};

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

int lan865x_initialize(FAR struct spi_dev_s *spi,
                       FAR struct oa_tc6_config_s *config)
{
  FAR struct lan865x_driver_s *priv = NULL;
  FAR struct oa_tc6_driver_s *dev;
  int retval;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      nerr("Error: Could not allocate memory for lan865x_driver_s priv\n");
      return -ENOMEM;
    }

  dev = &priv->oa_tc6_dev;

  /* Save the ops pointer */

  dev->ops = &g_lan865x_ops;

  retval = oa_tc6_common_init(dev, spi, config);
  if (retval)
    {
      nerr("Error: OA-TC6 common initialization failed\n");
      goto errout;
    }

  /* Init MAC address */

  if (lan865x_init_mac_addr(priv))
    {
      nerr("Error: Initialization of the MAC address failed\n");
      retval = -EIO;
      goto errout;
    }

  /* Do something with additional structure fields or with the device
   * if needed
   */

  /* Register */

  retval = oa_tc6_register(dev);
  if (retval == OK)
    {
      ninfo("Successfully registered OA-TC6 LAN865x network driver\n");
      return OK;
    }

  nerr("Error: Registration of the OA-TC6 LAN865x driver failed: %d\n",
       retval);

errout:
  kmm_free(priv);
  return retval;
}
