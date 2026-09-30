/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_timer.c
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
#include <nuttx/debug.h>

#include <stdbool.h>

#include "hardware/rp23xx_timer.h"
#include "hardware/rp23xx_ticks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: rp23xx_get_time
 *
 * Description:
 *   Get the time from the microsecond timer TIMER0 or TIMER1
 *
 * Input Parameters:
 *   timer - 0 for TIMER0, 1 for TIMER1
 *
 * Returned Value:
 *   64-bit value of the requested timer. This is time in microseconds.
 *
 ****************************************************************************/

uint64_t rp23xx_get_us_time(int timer)
{
  DEBUGASSERT(timer == 0 || timer == 1);
  uint32_t base = RP23XX_TIMER_BASE(timer);
  // TODO: investigate why not working
  // uint32_t timerh_addr = base + 0x24;
  // uint32_t timerl_addr = base + 0x28;
  // uint32_t hi = getreg32(timerh_addr);
  // uint32_t lo;
  // do
  //   {
  //     lo = getreg32(timerl_addr);
  //     uint32_t next_hi = getreg32(timerh_addr);
  //     if (hi == next_hi) break;
  //     hi = next_hi;
  //   } while (true);

  uint32_t timerh_addr = base + RP23XX_TIMER_TIMEHR_OFFSET;
  uint32_t timerl_addr = base + RP23XX_TIMER_TIMELR_OFFSET;
  uint32_t lo = getreg32(timerl_addr);
  uint32_t hi = getreg32(timerh_addr);

  return ((uint64_t)hi << 32u) | lo;
}
