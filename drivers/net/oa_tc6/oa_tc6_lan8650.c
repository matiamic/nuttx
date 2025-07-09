/****************************************************************************
 * drivers/net/oa_tc6/oa_tc6_lan8650.h
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

/*****************************************************************************
 * Private Types
 ****************************************************************************/

struct oa_tc6_lan8650_driver_s
{
  struct oa_tc6_driver_s oa_tc6_dev;

  int somethingmore;
};

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

static int oa_tc6_lan8650_action(struct oa_tc6_driver_s *dev,
                                 enum oa_tc6_action_e action)
{
  struct oa_tc6_lan8650_driver_s *priv = (struct oa_tc6_lan8650_driver_s *)dev;

  /* do something */
  return OK;
}

static int oa_tc6_lan8650_addmac(struct oa_tc6_driver_s *dev, uint8_t *mac)
{
  struct oa_tc6_lan8650_driver_s *priv = (struct oa_tc6_lan8650_driver_s *)dev;

  /* do something */
  return OK;
}

static int oa_tc6_lan8650_rmmac(struct oa_tc6_driver_s *dev, uint8_t *mac)
{
  struct oa_tc6_lan8650_driver_s *priv = (struct oa_tc6_lan8650_driver_s *)dev;

  /* do something */
  return OK;
}

static int oa_tc6_lan8650_ioctl(struct oa_tc6_driver_s *dev, int cmd,
                                unsigned long arg)
{
  struct oa_tc6_lan8650_driver_s *priv = (struct oa_tc6_lan8650_driver_s *)dev;

  /* do something */
  return OK;
}

/*****************************************************************************
 * Private Data
 ****************************************************************************/

static struct oa_tc6_ops_s g_oa_tc6_lan8650_ops =
{
  oa_tc6_lan8650_action,
  oa_tc6_lan8650_addmac,
  oa_tc6_lan8650_rmmac,
  oa_tc6_lan8650_ioctl
};

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

struct oa_tc6_driver_s *oa_tc6_lan8650_initialize(struct spi_dev_s *spi,
                                                  struct oa_tc6_config_s *config)
{
  FAR struct oa_tc6_lan8650_driver_s *priv = NULL;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      nerr("Could not allocate data for oa_tc6_lan8650_driver_s priv\n");
      return NULL;
    }

  /* Assign spi and config only if needed by the lan8650 init code, in any case it will be reassigned later in oa_tc6_initialize */

  priv->oa_tc6_dev.spi = spi;
  priv->oa_tc6_dev.config = config;

  /* Save the ops pointer */

  priv->oa_tc6_dev.ops = &g_oa_tc6_lan8650_ops;

  /* Do something with additional structure fields or with the device */

  /* Return */

  return &priv->oa_tc6_dev;
}
