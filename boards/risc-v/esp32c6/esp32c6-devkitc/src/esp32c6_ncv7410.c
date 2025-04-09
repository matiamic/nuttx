/****************************************************************************
 * boards/risc-v/esp32c6/esp32c6-devkitc/src/esp32c6_ncv7410.c
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
#include <nuttx/net/ncv7410.h>

#include <arch/board/board.h>

#include "espressif/esp_spi.h"
#include "espressif/esp_gpio.h"

#ifdef CONFIG_NCV7410

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ncv7410_initialize
 ****************************************************************************/

void board_ncv7410_initialize(void)
{
  struct spi_dev_s *spi;
  int irq;
  int ret;

  spi = esp_spibus_initialize(ESPRESSIF_SPI2);
  if (!spi)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n", ESPRESSIF_SPI2);
      return;
    }

  /* initialize the interrupt GPIO pin as input with PULLUP */
  esp_configgpio(CONFIG_NCV7410_INT_PIN, INPUT_FUNCTION_2 | PULLUP);
  irq = ESP_PIN2IRQ(CONFIG_NCV7410_INT_PIN);

  /* Bind the SPI port and interrupt to the NCV7410 driver */

  ret = ncv7410_initialize(spi, irq);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind interrupt and SPI port to the NCV7410 network driver: %d\n", ret);
      return;
    }

  /* driver attaches function to the interrupt, now it can be enabled */

  esp_gpioirqenable(irq, FALLING);

  ninfo("Bound interrupt and SPI port to the NCV7410 network driver\n");
}

#endif /* CONFIG_NCV7410 */
