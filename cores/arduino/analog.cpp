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

  Based on mbed-os/targets/TARGET_GigaDevice/TARGET_GD32F30X/analogin_api.c
  Based on mbed-os/targets/TARGET_GigaDevice/TARGET_GD32F30X/analogout_api.c
*/

#include "Arduino.h"
#include "analog.h"
#include "gd32xxyy.h"
#include "PinNames.h"
#include "PeripheralPins.h"
#include "HardwareTimer.h"
#include "gd_debug.h"
#include "gd32f30x_remap.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(DAC0) && defined(DAC1)
#define DAC_NUMS  2
#elif defined(DAC) || defined(DAC0)
#define DAC_NUMS  1
#else
#define DAC_NUMS  0
#endif

#define PWM_NUMS  40

#if defined(GD32F30x)
#if defined(GD32F30X_HD) || defined(GD32F30X_XD)
#define ADC_NUMS  3
#else
#define ADC_NUMS  2
#endif
#elif defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
#define ADC_NUMS  1
#elif defined(GD32F10x)
#define ADC_NUMS  3
#elif defined(GD32E50X)
#ifdef ADC2
#define ADC_NUMS  3
#elif defined(ADC1)
#define ADC_NUMS  2
#elif defined(ADC0)
#define ADC_NUMS  1
#endif
#endif

analog_t ADC_[ADC_NUMS] = { 0 };

/**
 * Allow configurable sampling times for both regular and internal channels
 * If these are not set, we set reasonable values here
 * define for precisely by defining:
 * ADC_SAMPLETIME to the desired ADC sample time
 */
#ifndef ADC_SAMPLETIME
#if defined(ADC_SAMPLETIME_7POINT5)
#define ADC_SAMPLETIME      ADC_SAMPLETIME_7POINT5
#elif defined(ADC_SAMPLETIME_13POINT5)
#define ADC_SAMPLETIME      ADC_SAMPLETIME_13POINT5
#endif
#endif /* !ADC_SAMPLETIME */

/**
 * Internal channels (TEMP/VREF) require a minimum sampling time
 * when reading, so set it to max possible value
 * It can be defined more precisely by defining:
 * ADC_SAMPLETIME_INTERNAL to the desired ADC sample time
 */
#ifndef ADC_SAMPLETIME_INTERNAL
#if defined(ADC_SAMPLETIME_239POINT5)
#define ADC_SAMPLETIME_INTERNAL     ADC_SAMPLETIME_239POINT5
#elif defined(ADC_SAMPLETIME_71POINT5)
#define ADC_SAMPLETIME_INTERNAL     ADC_SAMPLETIME_71POINT5
#elif defined(ADC_SAMPLETIME_55POINT5)
#define ADC_SAMPLETIME_INTERNAL     ADC_SAMPLETIME_55POINT5
#else
#error "ADC sampling time could not be defined for internal channels!"
#endif
#endif /* !ADC_SAMPLETIME_INTERNAL */

#ifndef ADC_PRESCALE_DIV
#if defined(RCU_CKADC_CKAPB2_DIV6)
#define ADC_PRESCALE_DIV      RCU_CKADC_CKAPB2_DIV6
#elif defined(RCU_CKADC_CKAPB2_DIV4)
#define ADC_PRESCALE_DIV      RCU_CKADC_CKAPB2_DIV4
#endif
#endif

/**
  * @brief  This function will set the DAC to the required value
  * @param  pin : the gpio pin to use
  * @param  value : the value to push on the dac output
  * @param  needs_init : if set to 1 the initialization of the dac is done
  * @retval None
 */
void set_dac_value(PinName pn, uint16_t value, uint8_t needs_init)
{
#if DAC_NUMS != 0
  uint32_t dac_periph = pinmap_peripheral(pn, PinMap_DAC);

  if (needs_init == 1) {
    dac_deinit();
#if (defined(GD32F1x0) && defined(GD32F170_190)) || defined(GD32F30x) || defined(GD32E50X)
    dac_trigger_disable(dac_periph);
#if defined(GD32F30x) || defined(GD32E50X)
    dac_wave_mode_config(dac_periph, DAC_WAVE_DISABLE);
#endif
    dac_output_buffer_enable(dac_periph);
    dac_enable(dac_periph);
    dac_data_set(dac_periph, DAC_ALIGN_12B_R, value);
#elif defined(GD32F1x0) && !defined(GD32F170_190)
    dac0_trigger_disable();
    dac0_output_buffer_enable();
    dac0_enable();
    dac0_data_set(DAC_ALIGN_12B_R, value);
#elif defined(GD32F3x0)
    /* only has 1 DAC at maximum, no need for a parameter... */
    dac_trigger_disable();
    dac_wave_mode_config(DAC_WAVE_DISABLE);
    dac_output_buffer_enable();
    dac_enable();
    dac_data_set(DAC_ALIGN_12B_R, value);
#endif
    /* DAC0 and DAC1 use same clock */
    rcu_periph_clock_enable(RCU_DAC);
    pinmap_pinout(pn, PinMap_DAC);
  } else {
    /* set dac value */
#if defined(GD32F30x) || (defined(GD32F1x0) && defined(GD32F170_190)) || defined(GD32E50X) || defined(GD32F10x)
    dac_data_set(dac_periph, DAC_ALIGN_12B_R, value);
#elif defined(GD32F1x0) && !defined(GD32F170_190)
    dac0_data_set(DAC_ALIGN_12B_R, value);
#elif defined(GD32F3x0)
    dac_data_set(DAC_ALIGN_12B_R, value);
#endif
  }
#endif
}

void dac_stop(PinName pn)
{
  uint32_t dac_periph = pinmap_peripheral(pn, PinMap_DAC);
  if (dac_periph == NP) {
    return;
  }
  dac_disable(dac_periph);
  dac_deinit();
}

uint32_t get_pwm_channel(PinName pin)
{
  uint32_t function = pinmap_function(pin, PinMap_PWM);
  uint32_t channel = -1;

  switch (GD_PIN_CHANNEL_GET(function)) {
    case 0:
      channel = TIMER_CH_0;
      break;
    case 1:
      channel = TIMER_CH_1;
      break;
    case 2:
      channel = TIMER_CH_2;
      break;
    case 3:
      channel = TIMER_CH_3;
      break;
    default:
      channel = -1;
      break;
  }
  return channel;
}

/* pwm start */
void pwm_start(PinName pin, uint32_t PWM_freq, uint32_t value, enum captureCompareFormat resolution)
{
  uint32_t instance = (uint32_t)pinmap_peripheral(pin, PinMap_PWM);
  HardwareTimer *HWT;
  captureMode previous;
  uint32_t index = get_timer_index(instance);
  if (HWTimer_Handle[index] == NULL) {
    HWTimer_Handle[index]->_timer_instance = new HardwareTimer((uint32_t)pinmap_peripheral(pin, PinMap_PWM));
  }
  HWT = (HardwareTimer *)(HWTimer_Handle[index]->_timer_instance);
  uint32_t channel = GD_PIN_CHANNEL_GET(pinmap_function(pin, PinMap_PWM));

  previous = HWT->getChannelMode((uint32_t)channel);
  if (previous != OC_PWM0) {
    HWT->setChannelMode((uint32_t)channel, OC_PWM0, pin);
  }
  HWT->setAutoReloadValue(PWM_freq, FORMAT_HZ);
  HWT->setCaptureCompare((uint32_t)channel, value, resolution);
  if (previous != OC_PWM0) {
    HWT->timerStart();
  }
}

/* pwm stop */
void pwm_stop(PinName pin)
{
  uint32_t instance = (uint32_t)pinmap_peripheral(pin, PinMap_PWM);
  HardwareTimer *HWT;
  uint32_t index = get_timer_index(instance);
  if (HWTimer_Handle[index] == NULL) {
    HWTimer_Handle[index]->_timer_instance = new HardwareTimer((uint32_t)pinmap_peripheral(pin, PinMap_PWM));
  }
  HWT = (HardwareTimer *)(HWTimer_Handle[index]->_timer_instance);
  if (HWT != NULL) {
    delete (HWT);
    HWT = NULL;
  }
}

/* get adc value */
uint16_t get_adc_value(PinName pn, uint32_t resolution)
{
  uint16_t value;
  uint32_t adc_periph = NP;
  uint8_t index = 0;
  uint32_t sampling_time = ADC_SAMPLETIME;
  uint32_t channel = 0;

  if ((pn & ADC_PINS_BASE) && (pn < ADC_START_ANALOG)) {
    adc_periph = ADC0;
    switch (pn) {
    case ADC_TEMP:
      channel = ADC_CHANNEL_16;
      break;
    case ADC_VREF:
      channel = ADC_CHANNEL_17;
      break;
    default:
      channel = 0;
      break;
    }
    sampling_time = ADC_SAMPLETIME_INTERNAL;
  } else {
    adc_periph = pinmap_peripheral(pn, PinMap_ADC);
    channel = get_adc_channel(pn);
  }
  if (adc_periph == NP) {
    return 0;
  }

  uint32_t res = 0;
  switch (resolution) {
#ifdef ADC_RESOLUTION_6B
    case 6:
      res = ADC_RESOLUTION_6B;
      break;
#endif
    case 8:
      res = ADC_RESOLUTION_8B;
      break;
    case 10:
      res = ADC_RESOLUTION_10B;
      break;
    case 12:
      res = ADC_RESOLUTION_12B;
      break;
    default:
      res = ADC_RESOLUTION_10B;
      break;
  }

  /**
   * configure clocks before starting them
   * to prevent startup errors
   */
  rcu_adc_clock_config(ADC_PRESCALE_DIV);
  adc_clock_enable((ADCName)adc_periph);

  index = get_adc_index(adc_periph);
  if (!(ADC_[index].isactive & ADC_PINS_BASE)) {
    pinmap_pinout(pn, PinMap_ADC);

#if defined(GD32F30x)|| defined(GD32E50X)
    //rcu_adc_clock_config(ADC_PRESCALE_DIV);
    adc_mode_config(ADC_MODE_FREE);
    adc_resolution_config(adc_periph, res);
    adc_data_alignment_config(adc_periph, ADC_DATAALIGN_RIGHT);
    adc_channel_length_config(adc_periph, ADC_REGULAR_CHANNEL, 1U);
    adc_regular_channel_config(adc_periph, 0U, channel, sampling_time);
#elif defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
    //rcu_adc_clock_config(ADC_PRESCALE_DIV);
    adc_special_function_config(ADC_CONTINUOUS_MODE, ENABLE);
#if defined(GD32F3x0) || defined(GD32F170_190) || defined(GD32E23x)
    adc_resolution_config(res);
#endif
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 1U);
#endif
#if defined(GD32F30x) || defined(GD32E50X)
    adc_external_trigger_source_config(adc_periph, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
#elif defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
#endif
#if defined(GD32F30x) || defined(GD32E50X)
    adc_external_trigger_config(adc_periph, ADC_REGULAR_CHANNEL, ENABLE);
    adc_enable(adc_periph);
    delay(1U);
    adc_calibration_enable(adc_periph);
#elif defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
    adc_enable();
    delay(1U);
    adc_calibration_enable();
#endif
    ADC_[index].isactive = true;
  }
#if defined(GD32F30x) || defined(GD32E50X)
  if ((pn == ADC_TEMP) | (pn == ADC_VREF)) {
      adc_tempsensor_vrefint_enable();
      delay(1U);
  }

  adc_software_trigger_enable(adc_periph, ADC_REGULAR_CHANNEL);
  while (!adc_flag_get(adc_periph, ADC_FLAG_EOC));
  adc_flag_clear(adc_periph, ADC_FLAG_EOC);
  value = adc_regular_data_read(adc_periph);
#elif defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
  adc_regular_channel_config(0U, channel, sampling_time);
  if (pn == ADC_TEMP || pn == ADC_VREF) {
    adc_tempsensor_vrefint_enable();
    delay(1U);
  }
  adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
  while (!adc_flag_get(ADC_FLAG_EOC));
  adc_flag_clear(ADC_FLAG_EOC);
  value = adc_regular_data_read();
#endif
  return value;
}

/* get adc index value */
uint8_t get_adc_index(uint32_t instance)
{
  uint8_t index;
  switch (instance) {
#ifdef ADC
  case ADC:
    index = 0;
    break;
#endif
#ifdef ADC0
  case ADC0:
    index = 0;
    break;
#endif
#ifdef ADC1
  case ADC1:
    index = 1;
    break;
#endif
#if (defined(GD32F30X_HD) || defined(GD32F30X_XD))
#ifdef ADC2
  case ADC2:
    index = 2;
    break;
#endif
#endif
  default:
    index = 0;
    break;
  }
  return index;
}

/* get dac index value */
uint8_t get_dac_index(uint32_t instance)
{
  uint8_t index;
  switch (instance) {
#if defined(DAC) && !defined(DAC0)  /* GD32F350 series had DAC. GD32F30x series has both DAC and DAC0 defined */
  case DAC:
    index = 0;
    break;
#endif
#ifdef DAC0
  case DAC0:
    index = 0;
    break;
#endif
#ifdef DAC1
  case DAC1:
    index = 1;
    break;
#endif
  default:
    index = 0;
    break;
  }
  return index;
}

/* get adc channel */
uint32_t get_adc_channel(PinName pinname)
{
  uint32_t function = pinmap_function(pinname, PinMap_ADC);
  uint32_t channel = 0xFF;
  switch (GD_PIN_CHANNEL_GET(function)) {
  #ifdef ADC_CHANNEL_0
    case 0:
      channel = ADC_CHANNEL_0;
      break;
  #endif
  #ifdef ADC_CHANNEL_1
    case 1:
      channel = ADC_CHANNEL_1;
      break;
  #endif
  #ifdef ADC_CHANNEL_2
    case 2:
      channel = ADC_CHANNEL_2;
      break;
  #endif
  #ifdef ADC_CHANNEL_3
    case 3:
      channel = ADC_CHANNEL_3;
      break;
  #endif
  #ifdef ADC_CHANNEL_4
    case 4:
      channel = ADC_CHANNEL_4;
      break;
  #endif
  #ifdef ADC_CHANNEL_5
    case 5:
      channel = ADC_CHANNEL_5;
      break;
  #endif
  #ifdef ADC_CHANNEL_6
    case 6:
      channel = ADC_CHANNEL_6;
      break;
  #endif
  #ifdef ADC_CHANNEL_7
    case 7:
      channel = ADC_CHANNEL_7;
      break;
  #endif
  #ifdef ADC_CHANNEL_8
    case 8:
      channel = ADC_CHANNEL_8;
      break;
  #endif
  #ifdef ADC_CHANNEL_9
    case 9:
      channel = ADC_CHANNEL_9;
      break;
  #endif
  #ifdef ADC_CHANNEL_10
    case 10:
      channel = ADC_CHANNEL_10;
      break;
  #endif
  #ifdef ADC_CHANNEL_11
    case 11:
      channel = ADC_CHANNEL_11;
      break;
  #endif
  #ifdef ADC_CHANNEL_12
    case 12:
      channel = ADC_CHANNEL_12;
      break;
  #endif
  #ifdef ADC_CHANNEL_13
    case 13:
      channel = ADC_CHANNEL_13;
      break;
  #endif
  #ifdef ADC_CHANNEL_14
    case 14:
      channel = ADC_CHANNEL_14;
      break;
  #endif
  #ifdef ADC_CHANNEL_15
    case 15:
      channel = ADC_CHANNEL_15;
      break;
  #endif
  default:
    channel = 0xFF;
    break;
  }
  return channel;
}

/* adc clock enable */
void adc_clock_enable(ADCName instance)
{
  rcu_periph_enum temp;
  switch (instance) {
#if defined(GD32F30x) || defined(GD32F10x) || defined(GD32E50X) //todo: other series
  case ADC_0:
    temp = RCU_ADC0;
    break;
  case ADC_1:
    temp = RCU_ADC1;
    break;
#if defined(GD32F30X_HD) || defined(GD32F30X_XD) || defined(GD32F10X_HD)
  case ADC_2:
    temp = RCU_ADC2;
    break;
#endif
#elif defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
  case ADC:
    temp = RCU_ADC;
    break;
#endif
  default:
    gd_debug("cannot enable ADC clock for unknown instance 0x%p", instance);
    return;
  }
  rcu_periph_clock_enable(temp);
}

#ifdef __cplusplus
}
#endif
