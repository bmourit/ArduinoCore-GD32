
#include "Arduino.h"
#include "gd32xxyy_spl_gpio.h"
#include "gpio_pull_helper.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t gd_gpio_get_mode(uint32_t gpio, uint32_t pin)
{
  uint32_t reg0 = GPIO_CTL0(gpio);
  uint32_t reg1 = GPIO_CTL1(gpio);
  uint32_t mode = 1U;

  for (uint16_t i = 0U; pin; i++, pin >>= 1) {
    if (pin & 1U) {
      mode = (i < 8U) ? (((i) * 4) >> (reg0 & GPIO_MODE_MASK(i))) :
                        (((i - 8U) * 4) >> (reg1 & GPIO_MODE_MASK(i - 8U)));
    }
  }
  return mode;
}

void gd_gpio_set_mode(uint32_t gpio, uint32_t pin, uint32_t mode)
{
  uint32_t pin_mask = 0x000000ffU;
  uint32_t reg_offset = (pin & pin_mask) >> 3;
  uint32_t bit_offset = (pin & pin_mask) & 0x07;
  uint32_t reg_val = GPIO_CTL0(gpio) + reg_offset;

  *(volatile uint32_t *)reg_val = (*(volatile uint32_t *)reg_val & ~(GPIO_MODE_MASK(bit_offset))) | GPIO_MODE_SET(bit_offset, mode);

  pin >>= 8;
  reg_offset = (pin & pin_mask) >> 3;
  bit_offset = (pin & pin_mask) & 0x07;
  reg_val = GPIO_CTL1(gpio) + reg_offset;

  *(volatile uint32_t *)reg_val = (*(volatile uint32_t *)reg_val & ~(GPIO_MODE_MASK(bit_offset))) | GPIO_MODE_SET(bit_offset, mode);
}

void gd_gpio_set_pull(uint32_t gpio, uint32_t pin, uint32_t pull)
{
  uint32_t port_bit_mask[16] = {0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
                                0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000};
  uint32_t port_bit_shift[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  uint32_t port_reg_addr[16] = {GPIO_BC(gpio), GPIO_BC(gpio), GPIO_BC(gpio), GPIO_BC(gpio),
                                 GPIO_BC(gpio), GPIO_BC(gpio), GPIO_BC(gpio), GPIO_BC(gpio),
                                 GPIO_BOP(gpio), GPIO_BOP(gpio), GPIO_BOP(gpio), GPIO_BOP(gpio),
                                 GPIO_BOP(gpio), GPIO_BOP(gpio), GPIO_BOP(gpio), GPIO_BOP(gpio)};

  while (pin) {
    uint32_t bit_pos = __CLZ(__RBIT(pin));
    if (pull == GPIO_MODE_IPD) {
      *(volatile uint32_t *)port_reg_addr[bit_pos] |= port_bit_mask[bit_pos];
    } else if (pull == GPIO_MODE_IPU) {
      *(volatile uint32_t *)port_reg_addr[bit_pos + 8] |= port_bit_mask[bit_pos];
    }
    pin &= ~(1U << bit_pos);
  }
}

#ifdef __cplusplus
}
#endif
