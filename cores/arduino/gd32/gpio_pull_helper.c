
#include "Arduino.h"
#include "gd32xxyy_gpio.h"
#include "gpio_pull_helper.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t gd_gpio_get_mode(uint32_t gpio, uint32_t pin)
{
  uint16_t i;
  uint32_t reg = 0U;
  uint32_t mode = 1U;

  for (i = 0U; i < 8; i++) {
    if ((1U << i) & pin) {
      reg = GPIO_CTL0(gpio);
      mode = ((i) * 4) >> (reg & GPIO_MODE_MASK(i));
    }
  }
  for (i = 8U; i < 16; i++) {
    if ((1U << i) & pin) {
      reg = GPIO_CTL1(gpio);
      mode = ((i - 8U) * 4) >> (reg & GPIO_MODE_MASK(i - 8U));
    }
  }
  return mode;
}

void gd_gpio_set_mode(uint32_t gpio, uint32_t pin, uint32_t mode)
{
  uint16_t i;
  uint32_t reg = 0U;

  for (i = 0U; i < 8U; i++) {
    if ((1U << i) & pin) {
      reg = GPIO_CTL0(gpio);
      reg &= ~GPIO_MODE_MASK(i);
      reg |= GPIO_MODE_SET(i, mode);
      GPIO_CTL0(gpio) = reg;
    }
  }
  for (i = 8U; i < 16U; i++) {
    if ((1U << i) & pin) {
      reg = GPIO_CTL1(gpio);
      reg &= ~GPIO_MODE_MASK(i - 8U);
      reg |= GPIO_MODE_SET(i - 8U, mode);
      GPIO_CTL1(gpio) = reg;
    }
  }
}

void gd_gpio_set_pull(uint32_t gpio, uint32_t pin, uint32_t pull)
{
  uint16_t i;

  for (i = 0U; i < 16U; i++) {
    if ((1U << i) & pin) {
      if (pull == GPIO_MODE_IPD) {
        GPIO_BC(gpio) = (uint32_t)((1U << i) & pin);
      } else {
        if (pull == GPIO_MODE_IPU) {
          GPIO_BOP(gpio) = (uint32_t)((1U << i) & pin);
        }
      }
    }
  }
}

#ifdef __cplusplus
}
#endif
