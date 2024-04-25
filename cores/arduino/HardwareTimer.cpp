/*
  Copyright (c) 2020, GigaDevice Semiconductor Inc.

  Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of its contributors
     may be used to endorse or promote products derived from this software without
     specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "HardwareTimer.h"
#include "pins_arduino.h"

//#define TIMERNUMS   13
#define TIMER_NUM_CHANNELS 4

HardwareTimer *hardwaretimerObj[TIMER_NUM] = { NULL };

/*!
  \brief      HardwareTimer object construct
  \param[in]  instance: TIMERx(x=0..13)
  \param[out] none
  \retval     none
*/
HardwareTimer::HardwareTimer(uint32_t instance)
{
  uint32_t index = getTimerIndex(instance);
  hardwaretimerObj[index] = this;
  this->timerDevice = instance;
  this->updateCallback = NULL;
  this->isTimerActive = false;
  this->timerPeriod.time = 1;
  this->timerPeriod.format = FORMAT_MS;
  timerHandle.init(timerDevice, &timerPeriod);
}

/*!
  \brief      start timer
  \param[in]  none
  \param[out] none
  \retval     none
*/
void HardwareTimer::start(void)
{
  timerHandle.start(timerDevice);
  this->isTimerActive = true;
}

/*!
  \brief      stop timer
  \param[in]  none
  \param[out] none
  \retval     none
*/
void HardwareTimer::stop(void)
{
  timerHandle.stop(timerDevice);
  this->isTimerActive = false;
}

/*!
  \brief      set prescaler
  \param[in]  prescaler: prescaler value
  \param[out] none
  \retval     none
*/
void HardwareTimer::setPrescaler(uint16_t prescaler)
{
  timer_prescaler_config(timerDevice, prescaler - 1, TIMER_PSC_RELOAD_NOW);
}

/*!
  \brief      set counter
  \param[in]  count: counter value
  \param[out] none
  \retval     none
*/
void HardwareTimer::setCounter(uint16_t count)
{
  timer_counter_value_config(timerDevice, count - 1);
}

/*!
  \brief      retrieve timer counter value
  \param[in]  none
  \param[out] none
  \retval     returns counter value
*/
uint32_t HardwareTimer::getCounter(void)
{
  return timer_counter_read(timerDevice);
}

/*!
  \brief      set repetition value
  \param[in]  repetition: repetition value
  \param[out] none
  \retval     none
*/
void HardwareTimer::setRepetitionValue(uint16_t repetition)
{
  timer_repetition_value_config(timerDevice, repetition - 1);
}

/*!
  \brief      set timer period
  \param[in]  time: period time
  \param[in]  format: time format
  \param[out] none
  \retval     none
*/
void HardwareTimer::setPeriodTime(uint32_t time, enum timeFormat format)
{
  this->timerPeriod.time = time;
  this->timerPeriod.format = format;
  timerHandle.setPeriodTime(timerDevice, &timerPeriod);
}

/*!
  \brief      set timer period with the inital format
  \param[in]  value: period time
  \param[out] none
  \retval     none
*/
void HardwareTimer::setReloadValue(uint32_t value, enum timeFormat format)
{
  uint32_t tval;
  uint32_t prescaler;
  uint32_t pcycle;
  uint32_t arv;

  switch (format) {
  case FORMAT_US:
    pcycle = value * (getTimerClkFreq() / 1000000);
    prescaler = (pcycle / 0x10000) + 1;
    TIMER_PSC(timerDevice) = prescaler - 1;
    tval = pcycle / prescaler;
    break;
  case FORMAT_HZ:
    pcycle = getTimerClkFreq() / value;
    prescaler = (pcycle / 0x10000) + 1;
    TIMER_PSC(timerDevice) = prescaler - 1;
    tval = pcycle / prescaler;
    break;
  case FORMAT_TICK:
  default:
    tval = value;
    break;
  }

  if (tval > 0) {
    arv = tval - 1;
  } else {
    arv = 0;
  }

  timer_autoreload_value_config(timerDevice, arv);
}

/*!
  \brief      update some registers to restart counters
  \param[in]  none
  \param[out] none
  \retval     none
*/
void HardwareTimer::refresh(void)
{
  timerHandle.refresh(timerDevice);
}

/*!
  \brief      Set the priority of the interrupt
  \note       Must be call before start()
  \param[in]  preemptPriority: the pre-emption priority for the IRQn channel
  \param[in]  subPriority: the subpriority level for the IRQ channel.
  \param[out] none
  \retval     none
*/
void HardwareTimer::setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority)
{
  timerHandle.setIntPriority(timerDevice, preemptPriority, subPriority);
}

/*!
  \brief      attach callback for period interrupt
  \param[in]  callback: callback function
  \param[in]  channel: timer channel
  \param[out] none
  \retval     none
*/
void HardwareTimer::attachInterrupt(timerCallback_t callback, uint8_t channel)
{
  if (channel < TIMER_NUM_CHANNELS + 1) {
    this->captureCallbacks[channel] = callback;
    timerHandle.enableCaptureIT(timerDevice, channel);
  } else if (0xFF == channel) {
    this->updateCallback = callback;
    timerHandle.enableUpdateIT(timerDevice);
  }
}

/*!
  \brief      detach callback for period interrupt
  \param[in]  channel: timer channel
  \param[out] none
  \retval     none
*/
void HardwareTimer::detachInterrupt(uint8_t channel)
{
  if (channel < TIMER_NUM_CHANNELS + 1) {
    this->captureCallbacks[channel] = NULL;
    timerHandle.disableCaptureIT(timerDevice, channel);
  } else if (0xFF == channel) {
    this->updateCallback = NULL;
    timerHandle.disableUpdateIT(timerDevice);
  }
}

/*!
  \brief      checks if there's an interrupt callback attached on update (rollover) event
  \param[in]  none
  \param[out] none
  \retval     returns true if a timer update (rollover) interrupt has already been set
*/
bool HardwareTimer::hasInterrupt()
{
  return updateCallback != NULL;
}

/*!
  \brief      checks if there's an interrupt callback attached on Capture/Compare event
  \param[in]  channel
  \param[out] none
  \retval     returns true if a channel compare match interrupt has already been set
*/
bool HardwareTimer::hasInterrupt(uint8_t channel)
{
  return captureCallbacks[channel] != NULL;
}

/*!
  \brief      set channel mode
  \param[in]  pin: pin number
  \param[in]  channel: timer channel
  \param[in]  mode: mode configuration for the channel (see captureMode)
  \param[out] none
  \retval     none
*/
void HardwareTimer::setCaptureMode(uint32_t ulpin, uint8_t channel, captureMode mode)
{
  timer_ic_parameter_struct timer_icinitpara;

  PinName pinname = DIGITAL_TO_PINNAME(ulpin);
  uint32_t function = pinmap_find_function(pinname, PinMap_PWM);
  uint32_t remap  = GD_PIN_REMAP_GET(function);
  uint32_t port = gpio_port[GD_PORT_GET(pinname)];
  uint32_t pin = gpio_pin[GD_PIN_GET(pinname)];
  gpio_clock_enable(port);
#if defined(GD32F30x)
  rcu_periph_clock_enable(RCU_AF);
  gpio_init(port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, pin);
  if (0 != remap) {
    gpio_pin_remap_config(GD_GPIO_REMAP[remap], ENABLE);
  }
#elif defined(GD32F3x0) || defined(GD32F1x0)
  /* !!TODO!! */
#endif

  switch (mode) {
    case RISING_EDGE:
      timer_icinitpara.icpolarity = TIMER_IC_POLARITY_RISING;
      break;
    case FALLING_EDGE:
      timer_icinitpara.icpolarity = TIMER_IC_POLARITY_FALLING;
      break;
    case BOTH_EDGE:
      timer_icinitpara.icpolarity = TIMER_IC_POLARITY_BOTH_EDGE;
      break;
    default:
      timer_icinitpara.icpolarity = TIMER_IC_POLARITY_RISING;
      break;
  }
  timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
  timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
  timer_icinitpara.icfilter = 0x0;
  timer_input_capture_config(timerDevice, channel, &timer_icinitpara);
  /* save the selected channel mode to object attribute */
  _ChannelMode[channel - 1] = mode;
}

/*!
  \brief      retrieves the configured channel mode
  \param[in]  channel
  \param[out] none
  \retval     returns configured mode
*/
captureMode HardwareTimer::getCaptureMode(uint8_t channel)
{
  if ((0 <= channel) && (channel <= TIMER_NUM_CHANNELS)) {
    return _ChannelMode[channel - 1];
  } else {
    return DISABLED;
  }
}

/*!
  /brief      enable or disable preloading (auto reload shadow enable)
              for overflow value
              when disabled, changes to the overflow value take effect
              immediately. When enabled (the default), the value takes
              effect only at the next update event (typically the next
              overflow).
  \param[in]  value: true to enable preloading (ARSE bit), false to disable
  \param[out] none
  \retval     none
*/
void HardwareTimer::setPreloadARSEnable(bool val)
{
  if (val) {
    timer_auto_reload_shadow_enable(timerDevice);
  } else {
    timer_auto_reload_shadow_disable(timerDevice);
  }
}

/*!
  \brief      retrieve the capture/compare value
  \param[in]  channel: timer channel
  \param[out] none
  \retval     returns the capture/compare value
*/
uint32_t HardwareTimer::getCaptureValue(uint8_t channel)
{
  return timer_channel_capture_value_register_read(timerDevice, channel);
}

/*!
  \brief      get timer clock frequency
  \param[in]  none
  \param[out] none
  \retval     returns the timer clock frequency
*/
uint32_t HardwareTimer::getTimerClkFreq(void)
{
  return getTimerClkFrequency(timerDevice);
}

/*!
  \brief      period callback handler
  \param[in]  none
  \param[out] none
  \retval     none
*/
void HardwareTimer::periodCallback(void)
{
  if (NULL != this->updateCallback) {
    this->updateCallback();
  }
}

/*!
  \brief      capture/compare callback handler
  \param[in]  channel: timer channel
  \param[out] none
  \retval     none
*/
void HardwareTimer::captureCallback(uint8_t channel)
{
  if (NULL != this->captureCallbacks[channel]) {
  this->captureCallbacks[channel]();
  }
}

/*!
  \brief      get timer index from timer instance
  \param[in]  instance: timer instancew
  \param[out] none
  \retval     returns the timer index
*/
timer_index_t get_timer_index(uint32_t instance)
{
  timer_index_t index = UNKNOWN_TIMER;
#if defined(TIMER0)
  if (instance == TIMER0) {
    index = TIMER0_INDEX;
  }
#endif
#if defined(TIMER1)
  if (instance == TIMER1) {
    index = TIMER1_INDEX;
  }
#endif
#if defined(TIMER2)
  if (instance == TIMER2) {
    index = TIMER2_INDEX;
  }
#endif
#if defined(TIMER3)
  if (instance == TIMER3) {
    index = TIMER3_INDEX;
  }
#endif
#if defined(TIMER4)
  if (instance == TIMER4) {
    index = TIMER4_INDEX;
  }
#endif
#if defined(TIMER5)
  if (instance == TIMER5) {
    index = TIMER5_INDEX;
  }
#endif
#if defined(TIMER6)
  if (instance == TIMER6) {
    index = TIMER6_INDEX;
  }
#endif
#if defined(TIMER7)
  if (instance == TIMER7) {
    index = TIMER7_INDEX;
  }
#endif
#if defined(TIMER8)
  if (instance == TIMER8) {
    index = TIMER8_INDEX;
  }
#endif
#if defined(TIMER9)
  if (instance == TIMER9) {
    index = TIMER9_INDEX;
  }
#endif
#if defined(TIMER10)
  if (instance == TIMER10) {
    index = TIMER10_INDEX;
  }
#endif
#if defined(TIMER11)
  if (instance == TIMER11) {
    index = TIMER11_INDEX;
  }
#endif
#if defined(TIMER12)
  if (instance == TIMER12) {
    index = TIMER12_INDEX;
  }
#endif
#if defined(TIMER13)
  if (instance == TIMER12) {
    index = TIMER13_INDEX;
  }
#endif

  return index;
}

extern "C"
{
  /*!
    \brief      timer update interrupt handler
    \param[in]  timer: TIMERx(x=0..13)
    \param[out] none
    \retval     none
  */
  void Timer_updateHandle(uint32_t timer)
  {
    uint32_t index = getTimerIndex(timer);
    if (hardwaretimerObj[index]) {
      hardwaretimerObj[index]->periodCallback();
    }
  }
  /*!
    \brief      timer capture interrupt handler
    \param[in]  timer: TIMERx(x=0..13)
    \param[in]  channel: TIMERx(x=0..3)
    \param[out] none
    \retval     none
  */
  void Timer_captureHandle(uint32_t timer, uint8_t channel)
  {
    uint32_t index =  getTimerIndex(timer);
    if (hardwaretimerObj[index]) {
      hardwaretimerObj[index]->captureCallback(channel);
    }
  }
}
