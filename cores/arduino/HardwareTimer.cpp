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

#include "Arduino.h"
#include "HardwareTimer.h"
#include "pins_arduino.h"

//#define TIMERNUMS   13

#define PERIOD_MAX  ((1 << 16) - 1)

//HardwareTimer *hardwaretimerObj[TIMER_NUM] = { NULL };
timerDevice_t *HWTimer_Handle[TIMER_NUM] = {NULL};

/*!
  \brief      HardwareTimer object construct
  \param[in]  instance: TIMERx(x=0..13)
  \retval     none
*/
HardwareTimer::HardwareTimer(uint32_t instance)
{
  _timerObj._timer_instance = (void *)this;
  _timerObj.handle.timer_instance = instance;
  _timerObj.handle.Channel = CHAN_ALL_CLEARED;
  _timerObj.handle.isTimerActive = false;

  _timerObj.timerPreemptPriority = TIMER_IRQ_PRIORITY;
  _timerObj.timerSubPriority = TIMER_IRQ_SUBPRIORITY;
  _timerObj.handle.init_params.prescaler = 0;
  _timerObj.handle.init_params.period = PERIOD_MAX;
  _timerObj.handle.init_params.counterdirection = TIMER_COUNTER_UP;
  _timerObj.handle.init_params.clockdivision = TIMER_CKDIV_DIV1;
  _timerObj.handle.init_params.alignedmode = TIMER_COUNTER_EDGE;
  _timerObj.handle.init_params.repetitioncounter = 0;
    
    uint32_t index = get_timer_index(instance);
    if (index == UNKNOWN_TIMER) {
        Error_Handler();
    }

    HWTimer_Handle[index] = &_timerObj;
    Timer_clock_enable(&(_timerObj.handle));

    /* null callback initialization */
    for (int i = 0; i < TIMER_NUM_CHANNELS + 1; i++) {
        timerCallbacks[i] = NULL;
    }

    /* channel mode and complementary initialization */
    for (int i = 0; i < TIMER_NUM_CHANNELS; i++) {
#if defined(TIMER_CHCTL2_CH0NEN)
        isLinkedChannel[i] = false;
#endif
        _ChannelMode[i] = TIMER_DISABLED;
    }

    Timer_init(&(_timerObj.handle));
}


/*!
  \brief  stop or pause a timer
  \retval None
*/
void HardwareTimer::timerStop(void)
{
  Timer_disableUpdateIT(_timerObj.handle.timer_instance);
  Timer_disableCaptureIT(_timerObj.handle.timer_instance, TIMER_INT_CH0);
  Timer_disableCaptureIT(_timerObj.handle.timer_instance, TIMER_INT_CH1);
  Timer_disableCaptureIT(_timerObj.handle.timer_instance, TIMER_INT_CH2);
  Timer_disableCaptureIT(_timerObj.handle.timer_instance, TIMER_INT_CH3);

  timer_disable(_timerObj.handle.timer_instance);
  _timerObj.handle.isTimerActive = false;
}

/*!
  \brief  stop/pause a channel
  \       timer is still running but channel is disabled (output and interrupt)
  \param  arduino channel [1..4]
  \retval none
*/
void HardwareTimer::stopChannel(uint8_t channel)
{
  int linkedChan;
  uint32_t CCChan;

  int chan = getChannel((uint32_t)channel);
  if (chan == -1) {
    Error_Handler();
  }

  int intChan = getInterruptChannel((uint32_t)channel);
  if (intChan == -1) {
    Error_Handler();
  }

  Timer_disableCaptureIT(_timerObj.handle.timer_instance, intChan);
  switch (channel) {
    case 1:
      CCChan = TIMER_CHCTL2_CH0EN;
      break;
    case 2:
      CCChan = TIMER_CHCTL2_CH1EN;
      break;
    case 3:
      CCChan = TIMER_CHCTL2_CH2EN;
      break;
    case 4:
      CCChan = TIMER_CHCTL2_CH3EN;
      break;
    default:
      CCChan = -1;
  }
  TIMER_CHCTL2(_timerObj.handle.timer_instance) &= (~(uint32_t)CCChan);

  if (_ChannelMode[channel - 1] == IC_MEASUREMENT) {
    linkedChan = getLinkedChannel((uint32_t)channel);
    Timer_disableCaptureIT(_timerObj.handle.timer_instance, getInterruptChannel(linkedChan));
  }
}

/*!
  \brief      start/resume timer
  \retval     none
*/
void HardwareTimer::timerStart(void)
{
  if (timerCallbacks[0]) {
    Timer_enableUpdateIT(_timerObj.handle.timer_instance);
    timer_enable(_timerObj.handle.timer_instance);
  }
  _timerObj.handle.isTimerActive = true;
  startChannel(1);
  startChannel(2);
  startChannel(3);
  startChannel(4);
}

/*!
  \brief  aonverts arduino channel into SPL channel
  \param  arduino channel [1..4]
  \retval SPL channel. return -1 if arduino channel is invalid
*/
int HardwareTimer::getChannel(uint32_t channel)
{
  uint32_t chan;

  switch (channel) {
  case 1:
    chan = TIMER_CH_0;
    break;
  case 2:
    chan = TIMER_CH_1;
    break;
  case 3:
    chan = TIMER_CH_2;
    break;
  case 4:
    chan = TIMER_CH_3;
    break;
  default:
    chan = -1;
  }

  return chan;
}

/*!
  \brief  convert arduino channel into SPL Interrupt ID
  \param  arduino channel [1..4]
  \retval SPL channel. return -1 if arduino channel is invalid
*/
int HardwareTimer::getInterruptChannel(uint32_t channel)
{
  uint32_t chan;

  switch (channel) {
    case 1:
      chan = TIMER_INT_CH0;
      break;
    case 2:
      chan = TIMER_INT_CH1;
      break;
    case 3:
      chan = TIMER_INT_CH2;
      break;
    case 4:
      chan = TIMER_INT_CH3;
      break;
    default:
      chan = -1;
  }

  return chan;
}

/*!
  \brief  gets input linked/complimentary channel
  \         channel 1 and 2 are complimentary; channel 3 and 4 are complimentary
  \param  arduino channel [1..4]
  \retval SPL channel. return -1 if arduino channel is invalid
*/
int HardwareTimer::getLinkedChannel(uint32_t channel)
{
  int linked;

  switch (channel) {
  case 1:
    linked = 2;
    break;
  case 2:
    linked = 1;
    break;
  case 3:
    linked = 4;
    break;
  case 4:
    linked = 3;
    break;
  default:
    linked = -1;
  }
  return linked;
}

/*!
  /brief  configure specified channel and start/resume timer
  /param  arduino channel [1..4]
  /retval None
*/
void HardwareTimer::startChannel(uint8_t channel)
{
  int timerChan = getChannel((uint32_t)channel);
  int timerLink;

  if (timerChan == -1) {
    Error_Handler();
  }

  int interruptChan = getInterruptChannel((uint32_t)channel);
  if (interruptChan == -1) {
    Error_Handler();
  }

  /* clear flag and enable interrupt */
  if (timerCallbacks[channel]) {
    Timer_enableCaptureIT(_timerObj.handle.timer_instance, interruptChan);
  }

  switch (_ChannelMode[channel - 1]) {
    case OC_PWM0:
    case OC_PWM1: {
#if defined(TIMER_CHCTL2_CH0NEN)
        if (isLinkedChannel[channel - 1]) {
          timer_channel_complementary_output_state_config(_timerObj.handle.timer_instance, (int)timerChan, TIMER_CCXN_ENABLE);
          timer_primary_output_config(_timerObj.handle.timer_instance, ENABLE);
          timer_enable(_timerObj.handle.timer_instance);
        } else
#endif
        {
          timer_channel_output_state_config(_timerObj.handle.timer_instance, (int)timerChan, TIMER_CCX_ENABLE);
          timer_primary_output_config(_timerObj.handle.timer_instance, ENABLE);
          timer_enable(_timerObj.handle.timer_instance);
        }
      }
      break;
    case OC_TIMING:
    case OC_ACTIVE:
    case OC_INACTIVE:
    case OC_TOGGLE:
    case OC_LOW:
    case OC_HIGH: {
#if defined(TIMER_CHCTL2_CH0NEN)
        if (isLinkedChannel[channel - 1]) {
          timer_channel_complementary_output_state_config(_timerObj.handle.timer_instance, (int)timerChan, TIMER_CCXN_ENABLE);
          timer_primary_output_config(_timerObj.handle.timer_instance, ENABLE);
          timer_enable(_timerObj.handle.timer_instance);
        } else
#endif
        {
          timer_channel_output_state_config(_timerObj.handle.timer_instance, (int)timerChan, TIMER_CCX_ENABLE);
          timer_primary_output_config(_timerObj.handle.timer_instance, ENABLE);
          timer_enable(_timerObj.handle.timer_instance);
        }
      }
      break;
    case IC_MEASUREMENT: {
        /**
         * NOTE: this SPL function name is misleading as
         * it enables the Capture/Compare functionality on 
         * the specified channel for both input and output
         */
        timer_channel_output_state_config(_timerObj.handle.timer_instance, (int)timerChan, TIMER_CCX_ENABLE);
        timer_enable(_timerObj.handle.timer_instance);

        /* enable complimentary/linked channel */
        timerLink = getLinkedChannel((uint32_t)channel);
        timer_channel_complementary_output_state_config(_timerObj.handle.timer_instance, (int)timerLink, TIMER_CCXN_ENABLE);
        if (timerCallbacks[channel]) {
          Timer_enableCaptureIT(_timerObj.handle.timer_instance, getInterruptChannel(timerLink));
        }
      }
      break;
    case IC_RISING_EDGE:
    case IC_FALLING_EDGE:
    case IC_BOTH_EDGE: {
        timer_channel_output_state_config(_timerObj.handle.timer_instance, (int)timerChan, TIMER_CCX_ENABLE);
        timer_enable(_timerObj.handle.timer_instance);
      }
      break;
    case TIMER_UNUSED:
    default:
      break;
  }
}

/*!
  /brief  gets the prescaler
  /retval returns the prescaler
*/
uint32_t HardwareTimer::getPrescaler()
{
  return (uint32_t)(timer_prescaler_read(_timerObj.handle.timer_instance) + 1);
}

/*!
  \brief      set prescaler
  \param[in]  prescaler: prescaler value
  \retval     none
*/
void HardwareTimer::setPrescaler(uint16_t prescaler)
{
  timer_prescaler_config(_timerObj.handle.timer_instance, prescaler - 1, TIMER_PSC_RELOAD_NOW);
}

/*!
  /brief  gets overflow (rollover) autoreload value
  /param  format: value format (default: FORMAT_TICK)
  /retval autoreload depending on the format:
  /           FORMAT_TICK:  returns value in number of ticks
  /           FORMAT_US:    returns value in number of microsecondes
  /           FORMAT_HZ:    returns frequency in hertz as value
*/
uint32_t HardwareTimer::getAutoReloadValue(enum timeFormat format)
{
  uint32_t CAR_RegValue = (uint32_t)TIMER_CAR(_timerObj.handle.timer_instance);
  uint32_t PrescalerValue = (uint32_t)timer_prescaler_read(_timerObj.handle.timer_instance) + 1;
  uint32_t value = 0;

  switch (format) {
    /* microseconds */
    case FORMAT_US:
      value = (uint32_t)(((CAR_RegValue + 1) * PrescalerValue * 1000000.0) / getTimerClkFreq());
      break;
    /* Hz */
    case FORMAT_HZ:
      value = (uint32_t)(getTimerClkFreq() / ((CAR_RegValue + 1) * PrescalerValue));
      break;
    /* tick */
    case FORMAT_TICK:
      value = CAR_RegValue + 1;
      break;
    case FORMAT_MS:
    case FORMAT_S:
      break;
  }
  return value;
}

/*!
  \brief      set timer CAR overflow (rollover) autoreload
  \param[in]  value: value based on the format
  \param[in]  format: the format of the value parameter. One of the following:
  \                   FORMAT_TICK:  value in number of ticks
  \                   FORMAT_US:    value in number of microseconds
  \                   FORMAT_HZ:    value in hertz frequency
  \retval     none
*/
void HardwareTimer::setAutoReloadValue(uint32_t value, enum timeFormat format)
{
  uint32_t CAR_RegValue;
  uint32_t TickValue;
  uint32_t PrescalerValue;
  uint32_t pcycle;

  switch (format) {
  case FORMAT_US:
    pcycle = value * (getTimerClkFreq() / 1000000);
    PrescalerValue = (pcycle / 0x10000) + 1;
    TIMER_PSC(_timerObj.handle.timer_instance) = PrescalerValue - 1;
    TickValue = pcycle / PrescalerValue;
    break;
  case FORMAT_HZ:
    pcycle = getTimerClkFreq() / value;
    PrescalerValue = (pcycle / 0x10000) + 1;
    TIMER_PSC(_timerObj.handle.timer_instance) = PrescalerValue - 1;
    TickValue = pcycle / PrescalerValue;
    break;
  case FORMAT_TICK:
  default:
    TickValue = value;
    break;
  }

  if (TickValue > 0) {
    CAR_RegValue = TickValue - 1;
  } else {
    CAR_RegValue = 0;
  }

  timer_autoreload_value_config(_timerObj.handle.timer_instance, CAR_RegValue);
}

/*!
  \brief      retrieve timer counter value
  \param[in]  format: the format of the value (default: FORMAT_TICK)
  \                   FORMAT_TICK:  counter in number of ticks
  \                   FORMAT_US:    counter in number of microseconds
  \                   FORMAT_HZ:    freq in hertz for counter
  \retval     returns counter value
*/
uint32_t HardwareTimer::getCounter(enum timeFormat format)
{
  uint32_t CNT_RegValue = timer_counter_read(_timerObj.handle.timer_instance);
  uint32_t PrescalerValue = (uint32_t)timer_prescaler_read(_timerObj.handle.timer_instance) + 1;
  uint32_t value;

  switch (format) {
    case FORMAT_US:
      value = (uint32_t)((CNT_RegValue * PrescalerValue * 1000000.0) / getTimerClkFreq());
      break;
    case FORMAT_HZ:
      value = (uint32_t)(getTimerClkFreq() / (CNT_RegValue * PrescalerValue));
      break;
    case FORMAT_TICK:
    default:
      value = CNT_RegValue;
      break;
  }

  return value;
}

/*!
  \brief      set counter value
  \param[in]  count: counter value depending on format
  \param[in]  format: the value format
  \retval     none
*/
void HardwareTimer::setCounter(uint16_t count, enum timeFormat format)
{
  uint32_t CNT_RegValue;
  uint32_t PerscalerValue = (uint32_t)timer_prescaler_read(_timerObj.handle.timer_instance) + 1;

  switch (format) {
    case FORMAT_US:
      CNT_RegValue = ((count * (getTimerClkFreq() / 1000000)) / PerscalerValue);
      break;
    case FORMAT_HZ:
      CNT_RegValue = (uint32_t)(getTimerClkFreq() / (count * PerscalerValue));
      break;
    case FORMAT_TICK:
    default:
      CNT_RegValue = (uint32_t)count;
      break;
  }
  timer_counter_value_config(_timerObj.handle.timer_instance, (uint32_t)CNT_RegValue);
}

/*!
  /brief  set capture/compare channel mode
  /param  pin: pin
  /param  channel: arduino timer channel [1..4]
  /param  mode: mode configuration for the channel (see captureMode)
  /retval none
*/
void HardwareTimer::setChannelMode(uint8_t channel, captureMode mode, uint32_t pin)
{
  setChannelMode(channel, mode, DIGITAL_TO_PINNAME(pin));
}

/*!
  \brief      set capture/compare channel mode
  \param[in]  pin: pin name
  \param[in]  channel: arduino timer channel [1..4]
  \param[in]  mode: mode configuration for the channel (see captureMode)
  \retval     none
*/
void HardwareTimer::setChannelMode(uint8_t channel, captureMode mode, PinName pin)
{
  int timerChan = getChannel((uint32_t)channel);
  int linkedChan;
  timer_oc_parameter_struct chanParamsOC;
  timer_ic_parameter_struct chanParamsIC;

  if (timerChan == -1) {
    Error_Handler();
  }

  _ChannelMode[channel - 1] = mode;

  /* here we configure some default values that may be overwritten later */
  chanParamsOC.outputstate = TIMER_CCX_DISABLE;
  chanParamsOC.ocpolarity = TIMER_OC_POLARITY_HIGH;
#if defined(TIMER_CHCTL2_CH0NEN)
  chanParamsOC.outputnstate = TIMER_CCXN_DISABLE;
  chanParamsOC.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
#endif
#if defined(TIMER_CTL1_ISO0N)
  chanParamsOC.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
  chanParamsOC.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
#endif
  chanParamsIC.icpolarity = TIMER_IC_POLARITY_RISING;
  chanParamsIC.icselection = TIMER_IC_SELECTION_DIRECTTI;
  chanParamsIC.icprescaler = TIMER_IC_PSC_DIV1;
  chanParamsIC.icfilter = 0U;

  switch (mode) {
    case TIMER_DISABLED:
      timer_channel_output_mode_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_MODE_TIMING);
      timer_channel_output_config(_timerObj.handle.timer_instance, timerChan, &chanParamsOC);
      break;
    case OC_TIMING:
      timer_channel_output_mode_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_MODE_TIMING);
      timer_channel_output_config(_timerObj.handle.timer_instance, timerChan, &chanParamsOC);
      break;
    case OC_ACTIVE:
      timer_channel_output_mode_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_MODE_ACTIVE);
      timer_channel_output_config(_timerObj.handle.timer_instance, timerChan, &chanParamsOC);
      break;
    case OC_INACTIVE:
      timer_channel_output_mode_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_MODE_INACTIVE);
      timer_channel_output_config(_timerObj.handle.timer_instance, timerChan, &chanParamsOC);
      break;
    case OC_TOGGLE:
      timer_channel_output_mode_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_MODE_TOGGLE);
      timer_channel_output_config(_timerObj.handle.timer_instance, timerChan, &chanParamsOC);
      break;
    case OC_LOW:
      timer_channel_output_mode_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_MODE_LOW);
      timer_channel_output_config(_timerObj.handle.timer_instance, timerChan, &chanParamsOC);
      break;
    case OC_HIGH:
      timer_channel_output_mode_config(_timerObj.handle.timer_instance, timerChan,TIMER_OC_MODE_HIGH);
      timer_channel_output_config(_timerObj.handle.timer_instance, timerChan, &chanParamsOC);
      break;
    case OC_PWM0:
      timer_channel_output_mode_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_MODE_PWM0);
      timer_channel_output_config(_timerObj.handle.timer_instance, timerChan, &chanParamsOC);
      timer_channel_output_shadow_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_SHADOW_ENABLE);
      timer_channel_output_fast_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_FAST_ENABLE);
      break;
    case OC_PWM1:
      timer_channel_output_mode_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_MODE_PWM1);
      timer_channel_output_config(_timerObj.handle.timer_instance, timerChan, &chanParamsOC);
      timer_channel_output_shadow_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_SHADOW_ENABLE);
      timer_channel_output_fast_config(_timerObj.handle.timer_instance, timerChan, TIMER_OC_FAST_ENABLE);
      break;
    case IC_RISING_EDGE:
      chanParamsIC.icpolarity = TIMER_IC_POLARITY_RISING;
      timer_input_capture_config(_timerObj.handle.timer_instance, timerChan, &chanParamsIC);
      break;
    case IC_FALLING_EDGE:
      chanParamsIC.icpolarity = TIMER_IC_POLARITY_FALLING;
      timer_input_capture_config(_timerObj.handle.timer_instance, timerChan, &chanParamsIC);
      break;
    case IC_BOTH_EDGE:
      chanParamsIC.icpolarity = TIMER_IC_POLARITY_BOTH_EDGE;
      timer_input_capture_config(_timerObj.handle.timer_instance, timerChan, &chanParamsIC);
      break;
    case IC_MEASUREMENT:
      chanParamsIC.icpolarity = TIMER_IC_POLARITY_RISING;
      chanParamsIC.icselection = TIMER_IC_SELECTION_DIRECTTI;
      timer_input_capture_config(_timerObj.handle.timer_instance, timerChan, &chanParamsIC);

      linkedChan = getLinkedChannel((uint32_t)channel);
      _ChannelMode[linkedChan - 1] = mode;
      chanParamsIC.icpolarity = TIMER_IC_POLARITY_FALLING;
      chanParamsIC.icselection = TIMER_IC_SELECTION_INDIRECTTI;
      timer_input_capture_config(_timerObj.handle.timer_instance, getChannel(linkedChan), &chanParamsIC);
      break;
    default:
      break;
  }

  if (pin != NC) {
    if ((int)get_pwm_channel(pin) == timerChan) {
      pinmap_pinout(pin, PinMap_PWM);
#if defined(GD32F30x)
      if ((mode == IC_RISING_EDGE) || (mode == IC_FALLING_EDGE) \
          || (mode == IC_BOTH_EDGE) || (mode == IC_MEASUREMENT)) {
        pinMode(PinName_to_digital(pin), INPUT);
      }
#endif
    } else {
      Error_Handler();
    }

#if defined(TIMER_CHCTL2_CH0NEN)
    isLinkedChannel[channel - 1] = GD_PIN_CHON_GET(pinmap_function(pin, PinMap_PWM));
#endif
  }
}

/*!
  \brief      retrieves the configured channel capture/compare mode
  \param[in]  channel
  \retval     returns configured mode
*/
captureMode HardwareTimer::getChannelMode(uint8_t channel)
{
  if ((1 <= channel) && (channel <= TIMER_NUM_CHANNELS)) {
    return _ChannelMode[channel - 1];
  } else {
    return TIMER_DISABLED;
  }
}

/*!
  /brief      enable or disable preloading (auto reload shadow enable)
              for overflow value. When disabled, changes to the overflow
              value take effect immediately. When enabled (default),
              the value takes effect at the next update event
  \param[in]  value: true to enable preloading (ARSE bit), false to disable
  \retval     none
*/
void HardwareTimer::setPreloadARSEnable(bool value)
{
  if (value) {
    timer_auto_reload_shadow_enable(_timerObj.handle.timer_instance);
  } else {
    timer_auto_reload_shadow_disable(_timerObj.handle.timer_instance);
  }
}

/*!
  \brief  set channel capture/compare register 
  \param  channel: arduino timer channel [1..4]
  \param  compare: compare value depending on format
  \param  format of compare parameter
  \retval None
*/
void HardwareTimer::setCaptureCompare(uint8_t channel, uint32_t cvalue, enum captureCompareFormat format)
{
  int timerChan = getChannel((uint32_t)channel);
  uint32_t PrescalerValue = (uint32_t)timer_prescaler_read(_timerObj.handle.timer_instance) + 1;
  uint32_t CHXCV_RegValue;

  if (timerChan == -1) {
    Error_Handler();
  }

  switch (format) {
   case CC_FORMAT_US:
    CHXCV_RegValue = ((cvalue * (getTimerClkFreq() / 1000000)) / PrescalerValue);
    break;
   case CC_FORMAT_HZ:
    CHXCV_RegValue = getTimerClkFreq() / (cvalue * PrescalerValue);
    break;
   case CC_FORMAT_PERCENT:
    CHXCV_RegValue = ((TIMER_CAR(_timerObj.handle.timer_instance) + 1) * cvalue) / 100;
    break;
   case CC_FORMAT_1B:
   case CC_FORMAT_2B:
   case CC_FORMAT_3B:
   case CC_FORMAT_4B:
   case CC_FORMAT_5B:
   case CC_FORMAT_6B:
   case CC_FORMAT_7B:
   case CC_FORMAT_8B:
   case CC_FORMAT_9B:
   case CC_FORMAT_10B:
   case CC_FORMAT_11B:
   case CC_FORMAT_12B:
   case CC_FORMAT_13B:
   case CC_FORMAT_14B:
   case CC_FORMAT_15B:
   case CC_FORMAT_16B:
    CHXCV_RegValue = ((TIMER_CAR(_timerObj.handle.timer_instance) + 1) * cvalue) / ((1 << format) - 1);
    break;
   case CC_FORMAT_TICK:
   default:
    CHXCV_RegValue = cvalue;
    break;
  }

  if ((TIMER_CAR(_timerObj.handle.timer_instance) == PERIOD_MAX) && (CHXCV_RegValue == PERIOD_MAX + 1)) {
    CHXCV_RegValue = PERIOD_MAX;
  }

  timer_channel_output_pulse_value_config(_timerObj.handle.timer_instance, timerChan, CHXCV_RegValue);
}

/*!
  \brief  get the channel's capture/compare value
  \param  channel: arduino timer channel [1..4]
  \param  format of value
  \retval None
*/
uint32_t HardwareTimer::getCaptureCompare(uint8_t channel, enum captureCompareFormat format)
{
  int timerChan = getChannel((uint32_t)channel);
  uint32_t PrescalerValue = (uint32_t)timer_prescaler_read(_timerObj.handle.timer_instance) + 1;
  uint32_t CHXCV_RegValue = timer_channel_capture_value_register_read(_timerObj.handle.timer_instance, timerChan);
  uint32_t value = 0;

  if (timerChan == -1) {
    Error_Handler();
  }

  switch (format) {
    case CC_FORMAT_US:
      value = (uint32_t)((CHXCV_RegValue * PrescalerValue * 1000000.0) / getTimerClkFreq());
      break;
    case CC_FORMAT_HZ:
      value = (uint32_t)(getTimerClkFreq() / (CHXCV_RegValue * PrescalerValue));
      break;
    case CC_FORMAT_PERCENT:
      value = (CHXCV_RegValue * 100) / TIMER_CAR(_timerObj.handle.timer_instance);
      break;
    case CC_FORMAT_1B:
    case CC_FORMAT_2B:
    case CC_FORMAT_3B:
    case CC_FORMAT_4B:
    case CC_FORMAT_5B:
    case CC_FORMAT_6B:
    case CC_FORMAT_7B:
    case CC_FORMAT_8B:
    case CC_FORMAT_9B:
    case CC_FORMAT_10B:
    case CC_FORMAT_11B:
    case CC_FORMAT_12B:
    case CC_FORMAT_13B:
    case CC_FORMAT_14B:
    case CC_FORMAT_15B:
    case CC_FORMAT_16B:
      value = (CHXCV_RegValue * ((1 << format) - 1)) / TIMER_CAR(_timerObj.handle.timer_instance);
      break;
    case CC_FORMAT_TICK:
      value = CHXCV_RegValue;
      break;
  }

  return value;
}

void HardwareTimer::setPWM(uint8_t channel, uint32_t pin, uint32_t freq, uint32_t dutycycle, timerCallback_t PeriodCallback, timerCallback_t CompareCallback)
{
  setPWM(channel, DIGITAL_TO_PINNAME(pin), freq, dutycycle, PeriodCallback, CompareCallback);
}

void HardwareTimer::setPWM(uint8_t channel, PinName pin, uint32_t freq, uint32_t dutycycle, timerCallback_t PeriodCallback, timerCallback_t CompareCallback)
{
  setChannelMode(channel, OC_PWM0, pin);
  setAutoReloadValue(freq, FORMAT_HZ);
  setCaptureCompare(channel, dutycycle, CC_FORMAT_PERCENT);
  if (PeriodCallback) {
    attachInterrupt(PeriodCallback);
  }
  if (CompareCallback) {
    attachInterrupt(CompareCallback, channel);
  }
  timerStart();
}

/*!
  \brief      Set the priority of the interrupt
  \note       Must be call before timerStart()
  \param[in]  preemptPriority: the pre-emption priority for the IRQn channel
  \param[in]  subPriority: the subpriority level for the IRQ channel.
  \param[out] none
  \retval     none
*/
void HardwareTimer::setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority)
{
#if defined(GD32E23x)
  nvic_irq_enable(getTimerUpIrq(_timerObj.handle.timer_instance), preemptPriority);
#else
  nvic_irq_enable(getTimerUpIrq(_timerObj.handle.timer_instance), preemptPriority, subPriority);
#endif

  if (getTimerCCIrq(_timerObj.handle.timer_instance) != getTimerUpIrq(_timerObj.handle.timer_instance)) {
#if defined(GD32E23x)
    nvic_irq_enable(getTimerCCIrq(_timerObj.handle.timer_instance), preemptPriority);
#else
    nvic_irq_enable(getTimerCCIrq(_timerObj.handle.timer_instance), preemptPriority, subPriority);
#endif
  }

  _timerObj.timerPreemptPriority = preemptPriority;
  _timerObj.timerSubPriority = subPriority;
}

/*!
  \brief      attach an interrupt callback on update event
  \param[in]  callback: callback function
  \retval     none
*/
void HardwareTimer::attachInterrupt(timerCallback_t callback)
{
  if (timerCallbacks[0]) {
    timerCallbacks[0] = callback;
  } else {
    timerCallbacks[0] = callback;
    if (callback) {
      Timer_enableUpdateIT(_timerObj.handle.timer_instance);
    }
  }
}

/*!
  \brief  detach callback for update interrupt
  \retval none
*/
void HardwareTimer::detachInterrupt()
{
  Timer_disableUpdateIT(_timerObj.handle.timer_instance);
  timerCallbacks[0] = NULL;
}

/*!
  \brief  attach interrupt callback on capture/compare event
  \param  channel: arduino channel [1..4]
  \param  callback: interrupt callback
  \retval none
*/
void HardwareTimer::attachInterrupt(timerCallback_t callback, uint8_t channel)
{
  int interrupt = getInterruptChannel((uint32_t)channel);
  if (interrupt == -1) {
    Error_Handler();
  }

  if ((channel == 0) || (channel > (TIMER_NUM_CHANNELS + 1))) {
    Error_Handler();
  }

  if (timerCallbacks[channel]) {
    timerCallbacks[channel] = callback;
  } else {
    timerCallbacks[channel] = callback;
    if (callback) {
      Timer_enableCaptureIT(_timerObj.handle.timer_instance, interrupt);
    }
  }
}

/*!
  \brief  dettach interrupt callback on capture/compare event
  \param  channel: Arduino channel [1..4]
  \retval none
*/
void HardwareTimer::detachInterrupt(uint8_t channel)
{
  int interrupt = getInterruptChannel((uint32_t)channel);

  if (interrupt == -1) {
    Error_Handler();
  }

  if ((channel == 0) || (channel > (TIMER_NUM_CHANNELS + 1))) {
    Error_Handler();
  }

  Timer_disableCaptureIT(_timerObj.handle.timer_instance, interrupt);
  timerCallbacks[channel] = NULL;
}

/*!
  \brief  checks if there's an interrupt callback attached on update event
  \retval returns true if a timer update interrupt has already been set
*/
bool HardwareTimer::hasInterrupt()
{
  return timerCallbacks[0] != NULL;
}

/*!
  \brief      checks if there's an interrupt callback attached on capture/compare event
  \param[in]  channel: arduino channel [1..4]
  \retval     returns true if a channel compare match interrupt has already been set
*/
bool HardwareTimer::hasInterrupt(uint8_t channel)
{
  if ((channel == 0) || (channel > (TIMER_NUM_CHANNELS + 1))) {
    Error_Handler();
  }
  return timerCallbacks[channel] != NULL;
}

/*!
  \brief  generate an update event to force all registers (autoreload, prescaler, compare) to be taken into account
  \note   refresh() can only be called after a 1st call to timerStart() to be sure timer is initialised.
  \retval none
*/
void HardwareTimer::refresh(void)
{
  timer_event_software_generate(_timerObj.handle.timer_instance, TIMER_EVENT_SRC_UPG);
}

SPL_TimerHandle_t *HardwareTimer::getHandle()
{
  return &_timerObj.handle;
}

/*!
  \brief  generic update callback which will call user callback
  \param  timer_handle: SPL timer handle
  \retval none
*/
void HardwareTimer::updateCallback(SPL_TimerHandle_t *timer_handle)
{
  if (!timer_handle) {
    Error_Handler();
  }

  timerDevice_t *timerObj = get_timer_object(timer_handle);
  HardwareTimer *HT = (HardwareTimer *)(timerObj->_timer_instance);

  if (HT->timerCallbacks[0]) {
    HT->timerCallbacks[0]();
  }
}

/*!
  \brief  generic capture/compare callback which will call user callback
  \param  timer_handle: SPL timer handle
  \retval none
*/
void HardwareTimer::captureCompareCallback(SPL_TimerHandle_t *timer_handle)
{
  if (!timer_handle) {
    Error_Handler();
  }
  uint32_t channel = timer_handle->Channel;

  switch (timer_handle->Channel) {
    case CHAN0_ACTIVE: {
        channel = 1;
        break;
      }
    case CHAN1_ACTIVE: {
        channel = 2;
        break;
      }
    case CHAN2_ACTIVE: {
        channel = 3;
        break;
      }
    case CHAN3_ACTIVE: {
        channel = 4;
        break;
      }
    default:
      return;
  }

  timerDevice_t *obj = get_timer_object(timer_handle);
  HardwareTimer *HT = (HardwareTimer *)(obj->_timer_instance);

  if (HT->timerCallbacks[channel]) {
    HT->timerCallbacks[channel]();
  }
}

/*!
  \brief      gets the timer clock frequency
  \retval     returns frequency in Hz
*/
uint32_t HardwareTimer::getTimerClkFreq(void)
{
  return getTimerClkFrequency(_timerObj.handle.timer_instance);;
}

/*!
  \brief      get timer index from timer instance
  \param[in]  instance: timer instance
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
  if (instance == TIMER13) {
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
  void timer_updateHandle(SPL_TimerHandle_t *timer_handle)
  {
    HardwareTimer::updateCallback(timer_handle);
  }

  /*!
    \brief      timer capture interrupt handler
    \param[in]  timer: TIMERx(x=0..13)
    \param[in]  channel: TIMERx(x=0..3)
    \param[out] none
    \retval     none
  */
  void timer_captureHandle(SPL_TimerHandle_t *timer_handle)
  {
    HardwareTimer::captureCompareCallback(timer_handle);
  }

#if defined(TIMER0)
  void TIMER0_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER0_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER0_INDEX]->handle);
    }
#if defined(TIMER9)
    if (HWTimer_Handle[TIMER9_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER9_INDEX]->handle);
    }
#endif
  }

  void TIMER0_Channel_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER0_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER0_INDEX]->handle);
    }
  }

  /* some devices have this. */
  void TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER0_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER0_INDEX]->handle);
    }
  }

  void TIMER0_UP_TIMER9_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER0_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER0_INDEX]->handle);
    }
#if defined(TIMER9)
      if (HWTimer_Handle[TIMER9_INDEX]) {
        timerInterruptHandler(&HWTimer_Handle[TIMER9_INDEX]->handle);
      }
#endif
  }

  void TIMER0_UP_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER0_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER0_INDEX]->handle);
    }
  }

#endif /* TIMER0/TIMER9 handler */

#if defined(TIMER1)
  void TIMER1_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER1_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER1_INDEX]->handle);
    }
  }
#endif /* TIMER1 handler */

#if defined(TIMER2)
  void TIMER2_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER2_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER2_INDEX]->handle);
    }
  }
#endif /* TIMER2 handler */

#if defined(TIMER3)
  void TIMER3_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER3_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER3_INDEX]->handle);
    }
  }
#endif /* TIMER3 handler */

#if defined(TIMER4)
  void TIMER4_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER4_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER4_INDEX]->handle);
    }
  }
#endif /* TIMER4 handler */

#if defined(TIMER5) && !defined(NO_TIMER_5)
  void TIMER5_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER5_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER5_INDEX]->handle);
    }
  }

  /* interrupt handler name for multiple F1x0, E50, F3x0, F4xx,.. chips */
  void TIMER5_DAC_IRQHandler(void) {
    if (HWTimer_Handle[TIMER5_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER5_INDEX]->handle);
    }
  }
#endif /* TMER5 handler */

#if defined(TIMER6)
  void TIMER6_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER6_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER6_INDEX]->handle);
    }
  }
#endif /* TIMER6 handler */

#if defined(TIMER7)
  void TIMER7_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER7_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER7_INDEX]->handle);
    }
#if defined(TIMER12)
    if (HWTimer_Handle[TIMER12_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER12_INDEX]->handle);
    }
#endif
  }

  void TIMER7_Channel_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER7_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER7_INDEX]->handle);
    }
  }
#endif /* TIMER7/TIMER12 handler */

#if defined(TIMER8)
  void TIMER8_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER8_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER8_INDEX]->handle);
    }
  }
#endif /* TIMER8 handler */

#if defined(TIMER9) && !defined(TIMER0)
  void TIMER9_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER9_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER9_INDEX]->handle);
    }
  }
#endif /* TIMER9 handler */

#if defined(TIMER10)
  void TIMER10_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER10_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER10_INDEX]->handle);
    }
  }
#endif /* TIMER10 handler */

#if defined(TIMER11)
  void TIMER11_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER11_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER11_INDEX]->handle);
    }
  }
#endif /* TIMER11 handler */

#if defined(TIMER12) && !defined (TIMER7)
  void TIMER12_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER12_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER12_INDEX]->handle);
    }
  }
#endif /* TIMER12 handler */

#if defined(TIMER13)
  void TIMER13_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER13_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER13_INDEX]->handle);
    }
  }
#endif /* TIMER13 handler */

#if defined(TIMER14)
  void TIMER14_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER14_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER14_INDEX]->handle);
    }
  }
#endif

#if defined(TIMER15)
  void TIMER15_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER15_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER15_INDEX]->handle);
    }
  }
#endif

#if defined(TIMER16)
  void TIMER16_IRQHandler(void)
  {
    if (HWTimer_Handle[TIMER16_INDEX]) {
      timerInterruptHandler(&HWTimer_Handle[TIMER16_INDEX]->handle);
    }
  }
#endif
}
