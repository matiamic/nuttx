/****************************************************************************
 * drivers/net/oa/oa_ncn26010.h
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

#include "oa.h"

/*****************************************************************************
 * Private Types
 ****************************************************************************/

struct oa_ncn26010_driver_s
{
  struct oa_driver_s oa_dev;

  int somethingmore;
}

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

static int oa_ncn26010_config(struct oa_driver_s *dev)
{
  struct oa_ncn26010_driver_s *priv = (oa_ncn26010_driver_s *)dev;

  /* do something */
}

static int oa_ncn26010_init_mac(struct oa_driver_s *dev)
{
  struct oa_ncn26010_driver_s *priv = (oa_ncn26010_driver_s *)dev;

  /* do something */
}

static int oa_ncn26010_add_mac(struct oa_driver_s *dev, uint8_t *mac)
{
  struct oa_ncn26010_driver_s *priv = (oa_ncn26010_driver_s *)dev;

  /* do something */
}

static int oa_ncn26010_rm_mac(struct oa_driver_s *dev, uint8_t *mac)
{
  struct oa_ncn26010_driver_s *priv = (oa_ncn26010_driver_s *)dev;

  /* do something */
}

static int oa_ncn26010_ioctl(struct oa_driver_s *dev, int cmd,
                            unsigned long arg)
{
  struct oa_ncn26010_driver_s *priv = (oa_ncn26010_driver_s *)dev;

  /* do something */
}

/*****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oa_ops_s g_oa_ncn26010_ops =
{
  oa_ncn26010_init_mac,
  oa_ncn26010_config,
  oa_ncn26010_add_mac,
  oa_ncn26010_rm_mac,
  oa_ncn26010_ioctl
};

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

struct oa_driver_s *oa_ncn26010_initialize(struct spi_dev_s *spi,
                                          struct oa_config_s *config)
{
  FAR struct oa_ncn26010_driver_s *priv = NULL;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      nerr("Could not allocate data for oa_ncn26010_driver_s priv\n");
      return NULL;
    }

  /* Assign spi and config only if needed by the lan8650 init code, in any case it will be reassigned later in oa_initialize */

  priv->oa_dev.spi = spi;
  priv->oa_dev.config = config;

  /* Save the ops pointer */

  priv->oa_dev.ops = &g_oa_ncn26010_ops;

  /* Do something with additional structure fields or with the device */

  /* Return */

  return priv->oa_dev;
}
