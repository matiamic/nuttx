/****************************************************************************
 * boards/risc-v/esp32c6/common/src/esp_board_oa_tc6.c
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
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/net/oa_tc6.h>

#include <arch/board/board.h>

#include "espressif/esp_spi.h"
#include "espressif/esp_gpio.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_board_oa_tc6_attach(FAR struct oa_tc6_config_s *config,
                                   xcpt_t handler,
                                   FAR void *arg);

static int esp_board_oa_tc6_enable(FAR struct oa_tc6_config_s *config,
                                   bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct oa_tc6_config_s g_esp_oa_tc6_config =
{
  .id            = SPIDEV_ETHERNET(0),
  .frequency     = 20000000,
  .chunk_size    = 64,
  .interrupt_pin = 5,
  .attach        = esp_board_oa_tc6_attach,
  .enable        = esp_board_oa_tc6_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int esp_board_oa_tc6_attach(FAR struct oa_tc6_config_s *config,
                                   xcpt_t handler, FAR void *arg)
{
  esp_configgpio(config->interrupt_pin, INPUT_FUNCTION_2 | PULLUP);
  irq_attach(ESP_PIN2IRQ(config->interrupt_pin), handler, arg);

  return OK;
}

static int esp_board_oa_tc6_enable(FAR struct oa_tc6_config_s *config, bool enable)
{
  if (enable)
    {
      esp_gpioirqenable(ESP_PIN2IRQ(config->interrupt_pin), FALLING);
    }
  else
    {
      esp_gpioirqdisable(ESP_PIN2IRQ(config->interrupt_pin));
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_oa_tc6_initialize
 *
 * Description:
 *   Initialize and register the OA-TC6 10BASE-T1S network driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

void board_oa_tc6_initialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  spi = esp_spibus_initialize(ESPRESSIF_SPI2);
  if (!spi)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize SPI port %d\n", ESPRESSIF_SPI2);
      return;
    }

  /* Bind the SPI port and config to the OA driver */

  ret = oa_tc6_initialize(spi, &g_esp_oa_tc6_config);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SPI port and config to the OA"
             " network driver: %d\n", ret);
      return;
    }

  syslog(LOG_INFO,
         "Bound SPI and config to the OA network driver\n");
}
