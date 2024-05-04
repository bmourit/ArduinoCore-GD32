/*
  Copyright (c) 2018 Frederic Pillon <frederic.pillon@st.com> for
  STMicroelectronics. All right reserved.
  Copyright (c) 2018 Venelin Efremov <ghent360@iqury.us>

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "FWatchdogTimer.h"

#if defined(GD32F30x)
#include "gd32f30x_fwdgt.h"
#include "gd32f30x_rcu.h"
#elif defined(GD32F10x)
#include "gd32f10x_fwdgt.h"
#include "gd32f10x_rcu.h"
#endif

// Initialize static variable
bool FWatchdogTimerClass::_enabled = false;

/**
  * @brief  Enable FWDGT, must be called once
  * @param  timeout: value in microseconds
  * @param  window: optional value in microseconds
  *         Default: FWDGT_TIMEOUT_MAX
  * @retval None
  */
void FWatchdogTimerClass::begin(uint32_t timeout)
{
  if (!IS_FWDGT_TIMEOUT(timeout)) {
    return;
  }

  // Enable the IRC40K peripheral clock FWDGT
  rcu_osci_on(RCU_IRC40K);
  rcu_osci_stab_wait(RCU_IRC40K);
  // Enable the FWDGT by writing 0xCCCC in the FWDGT_CTL register
  fwdgt_enable();
  _enabled = true;

  set(timeout);
}

/**
  * @brief  Set the timeout value
  * @param  timeout: value in microseconds
  *         Default: FWDGT_TIMEOUT_MAX
  * @retval None
  */
void FWatchdogTimerClass::set(uint32_t timeout)
{
  if ((isEnabled()) && (!IS_FWDGT_TIMEOUT(timeout))) {
    return;
  }

  // Compute the prescaler value
  uint16_t div = 0;
  uint8_t prescaler = 0;
  uint16_t reload = 0;

  // Convert timeout to seconds
  float t_sec = (float)timeout / 1000000 * IRC40K_VAL;

  do {
    div = 4 << prescaler;
    prescaler++;
  } while ((t_sec / div) > FWDGT_RLD_RLD);

  // 'prescaler' value is one of the FWDGT_PSC_DIVXX define
  if (--prescaler > FWDGT_PSC_DIV256) {
    return;
  }
  reload = (uint16_t)(t_sec / div) - 1;

  // Enable register access by writing 0x5555 in the FWDGT_CTL register
  // Write the FWDGT prescaler by programming IWDG_PR from 0 to 6
  // FWDGT_PSC_DIV4 (0) is lowest divider
  // This is all handled in the below function
  fwdgt_prescaler_value_config((uint16_t)prescaler);
  // Write the reload register (FWDGT_RLD)
  fwdgt_reload_value_config(reload);

  // Wait for the registers to be updated (FWDGT_STAT = 0x00000000)
  while ((fwdgt_flag_get(FWDGT_FLAG_PUD) | fwdgt_flag_get(FWDGT_FLAG_RUD)) != 0) {
  }
  // Refresh the counter value with FWDGT_KEY_RELOAD (FWDGT_CTL = 0xAAAA)
  fwdgt_counter_reload();
}

/**
  * @brief  Get the current timeout and window values
  * @param  timeout: pointer to the get the value in microseconds
  * @retval None
  */
void FWatchdogTimerClass::get(uint32_t *timeout)
{
  if (timeout != NULL) {
    uint32_t prescaler = 0;
    uint32_t reload = 0;
    float base = (1000000.0 / IRC40K_VAL);

    while (fwdgt_flag_get(FWDGT_FLAG_RUD));
    reload = (FWDGT_RLD & FWDGT_RLD_RLD);

    while (fwdgt_flag_get(FWDGT_FLAG_PUD));
    prescaler = (FWDGT_PSC & FWDGT_PSC_PSC);

    // Timeout given in microseconds
    *timeout = (uint32_t)((4 << prescaler) * (reload + 1) * base);
  }
}

/**
  * @brief  Reload the counter value with FWDGT_KEY_RELOAD (FWDGT_CTL = 0xAAAA)
  * @retval None
  */
void FWatchdogTimerClass::reload(void)
{
  if (isEnabled()) {
    fwdgt_counter_reload();
  }
}

/**
  * @brief  Check if the system has resumed from FWDGT reset
  * @param  clear: if true clear FWDGT reset flag. Default false
  * @retval return reset flag status
  */
bool FWatchdogTimerClass::isReset(bool clear)
{
  bool status = false;

  if (rcu_flag_get(RCU_FLAG_FWDGTRST) != 0) {
    status = true;
  }

  if (status && clear) {
    clearReset();
  }

  return status;
}

/**
  * @brief  Clear IWDG reset flag
  * @retval None
  */
void FWatchdogTimerClass::clearReset(void)
{
  rcu_all_reset_flag_clear();
}

// Preinstantiate Object
FWatchdogTimerClass FWatchdogTimer = FWatchdogTimerClass();
