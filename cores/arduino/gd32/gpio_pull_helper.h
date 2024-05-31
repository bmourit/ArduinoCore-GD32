#ifndef _GPIO_PULL_HELPER_H
#define _GPIO_PULL_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_NOPULL     0U
#define GPIO_PULLUP     1U
#define GPIO_PULLDOWN   2U

#define SPL_GPIO_MODE_ANALOG            0U
#define SPL_GPIO_MODE_INPUT_FLOATING    4U
#define SPL_GPIO_MODE_INPUT             8U

uint32_t gd_gpio_get_mode(uint32_t gpio, uint32_t pin);
void gd_gpio_set_mode(uint32_t gpio, uint32_t pin, uint32_t mode);
void gd_gpio_set_pull(uint32_t gpio, uint32_t pin, uint32_t pull);

#ifdef __cplusplus
}
#endif

#endif /* _GPIO_PULL_HELPER_H */
