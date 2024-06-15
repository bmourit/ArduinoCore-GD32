#ifndef _GD32F30X_REMAP_H
#define _GD32F30X_REMAP_H

#ifdef GD32F30X_HD

#include "gpio_pull_helper.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
  REMAP_NONE,
  SPI0_REMAP,
  DISABLE_SPI0_REMAP,
  I2C0_REMAP,
  DISABLE_I2C0_REMAP,
  USART0_REMAP,
  DISABLE_USART0_REMAP,
  USART1_REMAP,
  DISABLE_USART1_REMAP,
  USART2_PARTIAL_REMAP,
  USART2_FULL_REMAP,
  DISABLE_USART2_REMAP,
  TIMER0_PARTIAL_REMAP,
  TIMER0_FULL_REMAP,
  DISABLE_TIMER0_REMAP,
  TIMER1_PARTIAL_REMAP0,
  TIMER1_PARTIAL_REMAP1,
  TIMER1_FULL_REMAP,
  DISABLE_TIMER1_REMAP,
  TIMER2_PARTIAL_REMAP,
  TIMER2_FULL_REMAP,
  DISABLE_TIMER2_REMAP,
  TIMER3_REMAP,
  DISABLE_TIMER3_REMAP,
  PD01_REMAP,
  DISABLE_PD01_REMAP,
  TIMER4CH3_IREMAP,
  DISABLE_TIMER4CH3_IREMAP,
#if defined(GD32F30X_HD) || defined(GD32F30X_XD)
  CAN_PARTIAL_REMAP,
  CAN_FULL_REMAP,
  DISABLE_CAN_REMAP,
  ADC0_ETRGINS_REMAP,
  DISABLE_ADC0_ETRGINS_REMAP,
  ADC0_ETRGREG_REMAP,
  DISABLE_ADC0_ETRGREG_REMAP,
  ADC1_ETRGINS_REMAP,
  DISABLE_ADC1_ETRGINS_REMAP,
  ADC1_ETRGREG_REMAP,
  DISABLE_ADC1_ETRGREG_REMAP,
#endif /* (GD32F30X_HD) || (GD32F30X_XD)) */
  SWJ_NONJTRST_REMAP,
  SWJ_SWDPENABLE_REMAP,
  SWJ_DISABLE_REMAP,
  SPI2_REMAP,
  DISABLE_SPI2_REMAP,
#if defined(GD32F30X_CL)
  CAN0_PARTIAL_REMAP,
  CAN0_FULL_REMAP,
  DISABLE_CAN0_REMAP,
  ENET_REMAP,
  DISABLE_ENET_REMAP,
  CAN1_REMAP,
  DISABLE_CAN1_REMAP,
  TIMER1ITR0_REMAP,
  DISABLE_TIMER1ITR0_REMAP,
  PTP_PPS_REMAP,
  DISABLE_PTP_PPS_REMAP,
#endif /* GD32F30X_CL */
  TIMER8_REMAP,
  DISABLE_TIMER8_REMAP,
  TIMER9_REMAP,
  DISABLE_TIMER9_REMAP,
  TIMER10_REMAP,
  DISABLE_TIMER10_REMAP,
  TIMER12_REMAP,
  DISABLE_TIMER12_REMAP,
  TIMER13_REMAP,
  DISABLE_TIMER13_REMAP,
  EXMC_NADV_REMAP,
  DISABLE_EXMC_NADV_REMAP,
  CTC_REMAP0,
  DISABLE_CTC_REMAP0,
  CTC_REMAP1,
  DISABLE_CTC_REMAP1
};

static inline void f3_debug_disconnect(PinName pin)
{
#ifndef LOCK_LOWLEVEL_DEBUG
  rcu_periph_clock_enable(RCU_AF);

  /* JTAG-DP disabled and SW-DP disabled */
  if ((pin == PORTA_13) || (pin == PORTA_14)) {
    gpio_pin_remap_config(GPIO_SWJ_DISABLE_REMAP, DISABLE);
    gpio_pin_remap_config(GPIO_SWJ_DISABLE_REMAP, ENABLE);      
  }
  /* JTAG-DP disabled and SW-DP enabled */
  if ((pin == PORTA_15) || (pin == PORTB_3) || (pin == PORTB_4)) {
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, DISABLE);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
  }
#endif
}

static inline void f3_pin_pull_config(uint32_t gpio, uint32_t pin, uint32_t pull)
{
  uint32_t current_mode = gd_gpio_get_mode(gpio, pin);

  switch (pull) {
  case GPIO_PULLUP:
    if (current_mode == SPL_GPIO_MODE_INPUT_FLOATING) {
      gd_gpio_set_mode(gpio, pin, SPL_GPIO_MODE_INPUT);
    }
    gd_gpio_set_pull(gpio, pin, GPIO_MODE_IPU);
    break;
  case GPIO_PULLDOWN:
    if (current_mode == SPL_GPIO_MODE_INPUT_FLOATING) {
      gd_gpio_set_mode(gpio, pin, SPL_GPIO_MODE_INPUT);
    }
    gd_gpio_set_pull(gpio, pin, GPIO_MODE_IPD);
    break;
  default:
    if (current_mode == SPL_GPIO_MODE_INPUT) {
      gd_gpio_set_mode(gpio, pin, SPL_GPIO_MODE_INPUT_FLOATING);
    }
    gd_gpio_set_pull(gpio, pin, GPIO_MODE_IPD);
    break;
  }
}

static inline void f3_afpin_set(uint32_t afn)
{
    rcu_periph_clock_enable(RCU_AF);

    switch (afn) {
      case SPI0_REMAP:
        gpio_pin_remap_config(GPIO_SPI0_REMAP, ENABLE);
        break;
      case DISABLE_SPI0_REMAP:
        gpio_pin_remap_config(GPIO_SPI0_REMAP, DISABLE);
        break;
      case I2C0_REMAP:
        gpio_pin_remap_config(GPIO_I2C0_REMAP, ENABLE);
        break;
      case DISABLE_I2C0_REMAP:
        gpio_pin_remap_config(GPIO_I2C0_REMAP, DISABLE);
        break;
      case USART0_REMAP:
        gpio_pin_remap_config(GPIO_USART0_REMAP, ENABLE);
        break;
      case DISABLE_USART0_REMAP:
        gpio_pin_remap_config(GPIO_USART0_REMAP, DISABLE);
        break;
      case USART1_REMAP:
        gpio_pin_remap_config(GPIO_USART1_REMAP, ENABLE);
        break;
      case DISABLE_USART1_REMAP:
        gpio_pin_remap_config(GPIO_USART1_REMAP, DISABLE);
        break;
      case USART2_PARTIAL_REMAP:
        gpio_pin_remap_config(GPIO_USART2_PARTIAL_REMAP, ENABLE);
        break;
      case USART2_FULL_REMAP:
        gpio_pin_remap_config(GPIO_USART2_FULL_REMAP, ENABLE);
        break;
      case DISABLE_USART2_REMAP:
        gpio_pin_remap_config(GPIO_USART2_PARTIAL_REMAP, DISABLE);
        gpio_pin_remap_config(GPIO_USART2_FULL_REMAP, DISABLE);
        break;
      case TIMER0_PARTIAL_REMAP:
        gpio_pin_remap_config(GPIO_TIMER0_PARTIAL_REMAP, ENABLE);
        break;
      case TIMER0_FULL_REMAP:
        gpio_pin_remap_config(GPIO_TIMER0_FULL_REMAP, ENABLE);
        break;
      case DISABLE_TIMER0_REMAP:
        gpio_pin_remap_config(GPIO_TIMER0_PARTIAL_REMAP, DISABLE);
        gpio_pin_remap_config(GPIO_TIMER0_FULL_REMAP, DISABLE);
        break;
      case TIMER1_PARTIAL_REMAP0:
        gpio_pin_remap_config(GPIO_TIMER1_PARTIAL_REMAP0, ENABLE);
        break;
      case TIMER1_PARTIAL_REMAP1:
        gpio_pin_remap_config(GPIO_TIMER1_PARTIAL_REMAP1, ENABLE);
        break;
      case TIMER1_FULL_REMAP:
        gpio_pin_remap_config(GPIO_TIMER1_FULL_REMAP, ENABLE);
        break;
      case DISABLE_TIMER1_REMAP:
        gpio_pin_remap_config(GPIO_TIMER1_PARTIAL_REMAP0, DISABLE);
        gpio_pin_remap_config(GPIO_TIMER1_PARTIAL_REMAP1, DISABLE);
        gpio_pin_remap_config(GPIO_TIMER1_FULL_REMAP, DISABLE);
        break;
      case TIMER2_PARTIAL_REMAP:
        gpio_pin_remap_config(GPIO_TIMER2_PARTIAL_REMAP, ENABLE);
        break;
      case TIMER2_FULL_REMAP:
        gpio_pin_remap_config(GPIO_TIMER2_FULL_REMAP, ENABLE);
        break;
      case DISABLE_TIMER2_REMAP:
        gpio_pin_remap_config(GPIO_TIMER2_PARTIAL_REMAP, DISABLE);
        gpio_pin_remap_config(GPIO_TIMER2_FULL_REMAP, DISABLE);
        break;
      case TIMER3_REMAP:
        gpio_pin_remap_config(GPIO_TIMER3_REMAP, ENABLE);
        break;
      case DISABLE_TIMER3_REMAP:
        gpio_pin_remap_config(GPIO_TIMER3_REMAP, DISABLE);
        break;
#if defined(GD32F30X_HD) || defined(GD32F30X_XD)
      case CAN_PARTIAL_REMAP:
        gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, ENABLE);
        break;
      case CAN_FULL_REMAP:
        gpio_pin_remap_config(GPIO_CAN_FULL_REMAP, ENABLE);
        break;
      case DISABLE_CAN_REMAP:
        gpio_pin_remap_config(GPIO_CAN_PARTIAL_REMAP, DISABLE);
        gpio_pin_remap_config(GPIO_CAN_FULL_REMAP, DISABLE);
        break;
#endif
#ifdef GD32F30X_CL
      case CAN0_PARTIAL_REMAP:
        gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP, ENABLE);
        break;
      case CAN0_FULL_REMAP:
        gpio_pin_remap_config(GPIO_CAN0_FULL_REMAP, ENABLE);
        break;
      case DISABLE_CAN0_REMAP:
        gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP, DISABLE);
        gpio_pin_remap_config(GPIO_CAN0_FULL_REMAP, DISABLE);
        break;
#endif
      case PD01_REMAP:
        gpio_pin_remap_config(GPIO_PD01_REMAP, ENABLE);
        break;
      case DISABLE_PD01_REMAP:
        gpio_pin_remap_config(GPIO_PD01_REMAP, DISABLE);
        break;
      case TIMER4CH3_IREMAP:
        gpio_pin_remap_config(GPIO_TIMER4CH3_IREMAP, ENABLE);
        break;
      case DISABLE_TIMER4CH3_IREMAP:
        gpio_pin_remap_config(GPIO_TIMER4CH3_IREMAP, DISABLE);
        break;
      case SPI2_REMAP:
        gpio_pin_remap_config(GPIO_SPI2_REMAP, ENABLE);
        break;
      case DISABLE_SPI2_REMAP:
        gpio_pin_remap_config(GPIO_SPI2_REMAP, DISABLE);
        break;
#if defined(GD32F30X_HD) || defined(GD32F30X_XD)
      case ADC0_ETRGINS_REMAP:
        gpio_pin_remap_config(GPIO_ADC0_ETRGINS_REMAP, ENABLE);
        break;
      case DISABLE_ADC0_ETRGINS_REMAP:
        gpio_pin_remap_config(GPIO_ADC0_ETRGINS_REMAP, DISABLE);
        break;
      case ADC0_ETRGREG_REMAP:
        gpio_pin_remap_config(GPIO_ADC0_ETRGREG_REMAP, ENABLE);
        break;
      case DISABLE_ADC0_ETRGREG_REMAP:
        gpio_pin_remap_config(GPIO_ADC0_ETRGREG_REMAP, DISABLE);
        break;
      case ADC1_ETRGINS_REMAP:
        gpio_pin_remap_config(GPIO_ADC1_ETRGINS_REMAP, ENABLE);
        break;
      case DISABLE_ADC1_ETRGINS_REMAP:
        gpio_pin_remap_config(GPIO_ADC1_ETRGINS_REMAP, DISABLE);
        break;
      case ADC1_ETRGREG_REMAP:
        gpio_pin_remap_config(GPIO_ADC1_ETRGREG_REMAP, ENABLE);
        break;
      case DISABLE_ADC1_ETRGREG_REMAP:
        gpio_pin_remap_config(GPIO_ADC1_ETRGREG_REMAP, DISABLE);
        break;
#endif
#ifdef GD32F30X_CL
      case ENET_REMAP:
        gpio_pin_remap_config(GPIO_ENET_REMAP, ENABLE);
        break;
      case DISABLE_ENET_REMAP:
        gpio_pin_remap_config(GPIO_ENET_REMAP, DISABLE);
        break;
      case CAN1_REMAP:
        gpio_pin_remap_config(GPIO_CAN1_REMAP, ENABLE);
        break;
      case DISABLE_CAN1_REMAP:
        gpio_pin_remap_config(GPIO_CAN1_REMAP, DISABLE);
        break;
      case TIMER1ITR0_REMAP:
        gpio_pin_remap_config(GPIO_TIMER1ITR0_REMAP, ENABLE);
        break;
      case DISABLE_TIMER1ITR0_REMAP:
        gpio_pin_remap_config(GPIO_TIMER1ITR0_REMAP, DISABLE);
        break;
      case PTP_PPS_REMAP:
        gpio_pin_remap_config(GPIO_PTP_PPS_REMAP, ENABLE);
        break;
      case DISABLE_PTP_PPS_REMAP:
        gpio_pin_remap_config(GPIO_PTP_PPS_REMAP, DISABLE);
        break;
#endif
      case TIMER8_REMAP:
        gpio_pin_remap_config(GPIO_TIMER8_REMAP, ENABLE);
        break;
      case DISABLE_TIMER8_REMAP:
        gpio_pin_remap_config(GPIO_TIMER8_REMAP, DISABLE);
        break;
      case TIMER9_REMAP:
        gpio_pin_remap_config(GPIO_TIMER9_REMAP, ENABLE);
        break;
      case DISABLE_TIMER9_REMAP:
        gpio_pin_remap_config(GPIO_TIMER9_REMAP, DISABLE);
        break;
      case TIMER10_REMAP:
        gpio_pin_remap_config(GPIO_TIMER10_REMAP, ENABLE);
        break;
      case DISABLE_TIMER10_REMAP:
        gpio_pin_remap_config(GPIO_TIMER10_REMAP, DISABLE);
        break;
      case TIMER12_REMAP:
        gpio_pin_remap_config(GPIO_TIMER12_REMAP, ENABLE);
        break;
      case DISABLE_TIMER12_REMAP:
        gpio_pin_remap_config(GPIO_TIMER12_REMAP, DISABLE);
        break;
      case TIMER13_REMAP:
        gpio_pin_remap_config(GPIO_TIMER13_REMAP, ENABLE);
        break;
      case DISABLE_TIMER13_REMAP:
        gpio_pin_remap_config(GPIO_TIMER13_REMAP, DISABLE);
        break;
      case EXMC_NADV_REMAP:
        gpio_pin_remap_config(GPIO_EXMC_NADV_REMAP, ENABLE);
        break;
      case DISABLE_EXMC_NADV_REMAP:
        gpio_pin_remap_config(GPIO_EXMC_NADV_REMAP, DISABLE);
        break;
      case CTC_REMAP0:
        gpio_pin_remap_config(GPIO_CTC_REMAP0, ENABLE);
        break;
      case DISABLE_CTC_REMAP0:
        gpio_pin_remap_config(GPIO_CTC_REMAP0, DISABLE);
        break;
      case CTC_REMAP1:
        gpio_pin_remap_config(GPIO_CTC_REMAP1, ENABLE);
        break;
      case DISABLE_CTC_REMAP1:
        gpio_pin_remap_config(GPIO_CTC_REMAP1, DISABLE);
        break;
      default:
      case REMAP_NONE:
        break;
    }
}

#ifdef __cplusplus
}
#endif

#endif  /* GD32F30X_HD */

#endif /* _GD32F30X_REMAP_H */
