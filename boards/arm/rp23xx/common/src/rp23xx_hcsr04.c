/****************************************************************************
 * boards/arm/rp23xx/common/src/rp23xx_hcsr04.c
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

#include <stdbool.h>

#include <nuttx/sensors/hc_sr04.h>

#include <arch/board/board.h>

#include "rp23xx_gpio.h"
#include "rp23xx_timer.h"

#include <nuttx/clock.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_hcsr04_config_s
{
  /* Configuration structure as seen by the HC-SR04 driver */

  struct hcsr04_config_s config;

  /* Additional private definitions only known to this driver */

  xcpt_t isr;   /* ISR Handler */
  void *arg;    /* Argument to pass to the interrupt handler */
  int int_mode; /* Falling or rising edge */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  hcsr04_irq_attach(struct hcsr04_config_s *state, xcpt_t isr,
                              void *arg);
static void hcsr04_irq_enable(const struct hcsr04_config_s *state,
                              bool enable);
static void hcsr04_irq_clear(const struct hcsr04_config_s *state);
static void hcsr04_irq_setmode(struct hcsr04_config_s *state,
                               bool rise_mode);
static void hcsr04_set_trigger(const struct hcsr04_config_s *state,
                               bool on);
static int64_t hcsr04_get_clock(const struct hcsr04_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the HC-SR04
 * driver.  This structure provides information about the configuration
 * of the HC-SR04 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

static struct rp23xx_hcsr04_config_s g_hcsr04config =
{
  .config =
  {
    .irq_attach  = hcsr04_irq_attach,
    .irq_enable  = hcsr04_irq_enable,
    .irq_clear   = hcsr04_irq_clear,
    .irq_setmode = hcsr04_irq_setmode,
    .set_trigger = hcsr04_set_trigger,
    .get_clock   = hcsr04_get_clock,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Attach the HC-SR04 interrupt handler to the GPIO interrupt */

static int hcsr04_irq_attach(struct hcsr04_config_s *state, xcpt_t isr,
                             void *arg)
{
  struct rp23xx_hcsr04_config_s *priv =
             (struct rp23xx_hcsr04_config_s *)state;
  irqstate_t flags;

  sinfo("hcsr04_irq_attach\n");

  flags = enter_critical_section();

  priv->isr = isr;
  priv->arg = arg;

  leave_critical_section(flags);

  return OK;
}

/* Setup the interruption mode: Rising or Falling */

static void hcsr04_irq_setmode(struct hcsr04_config_s *state,
                               bool rise_mode)
{
  struct rp23xx_hcsr04_config_s *priv =
             (struct rp23xx_hcsr04_config_s *)state;

  irqstate_t flags;
  flags = enter_critical_section();

  if (rise_mode)
    {
      priv->int_mode = RP23XX_GPIO_INTR_EDGE_HIGH;
    }
  else
    {
      priv->int_mode = RP23XX_GPIO_INTR_EDGE_LOW;
    }

  leave_critical_section(flags);
}

/* Enable or disable the GPIO interrupt */

static void hcsr04_irq_enable(const struct hcsr04_config_s *state,
                              bool enable)
{
  struct rp23xx_hcsr04_config_s *priv =
             (struct rp23xx_hcsr04_config_s *)state;

  iinfo("%d\n", enable);

  if (enable)
    {
      rp23xx_gpio_irq_attach(BOARD_HCSR04_GPIO_ECHO,
                             priv->int_mode,
                             priv->isr, priv->arg);
      rp23xx_gpio_enable_irq(BOARD_HCSR04_GPIO_ECHO);
    }
  else
    {
      rp23xx_gpio_disable_irq(BOARD_HCSR04_GPIO_ECHO);
    }
}

/* Acknowledge/clear any pending GPIO interrupt */

static void hcsr04_irq_clear(const struct hcsr04_config_s *state)
{
  rp23xx_gpio_clear_interrupt(BOARD_HCSR04_GPIO_ECHO, true, true);
}

/* Set the Trigger pin state */

static void hcsr04_set_trigger(const struct hcsr04_config_s *state,
                               bool on)
{
  rp23xx_gpio_put(BOARD_HCSR04_GPIO_TRIG, on);
}

/* Return the current Free Running clock tick */

static int64_t hcsr04_get_clock(const struct hcsr04_config_s *state)
{
  /* Get the time from the microsecond system timer TIMER0 */

  return (int64_t)rp23xx_get_us_time(0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_hcsr04_initialize
 *
 * Description:
 *   This function is called by application-specific, setup logic to
 *   configure the HC-SR04 sensor.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/distN
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_hcsr04_initialize(int devno)
{
  char devpath[12];

  /* Configure the PIO interrupt */

  rp23xx_gpio_init(BOARD_HCSR04_GPIO_ECHO);

  /* Configure the Trigger pin */

  rp23xx_gpio_init(BOARD_HCSR04_GPIO_TRIG);
  rp23xx_gpio_setdir(BOARD_HCSR04_GPIO_TRIG, true);
  rp23xx_gpio_put(BOARD_HCSR04_GPIO_TRIG, false);

  snprintf(devpath, sizeof(devpath), "/dev/dist%d", devno);
  return hcsr04_register(devpath, &g_hcsr04config.config);
}
