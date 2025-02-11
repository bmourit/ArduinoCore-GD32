#ifndef _GPIO_INTERRUPT_
#define _GPIO_INTERRUPT_

#include "gd32_def.h"

#if defined(SPL_EXTI_ENABLE)

#ifndef EXTI_IRQ_PRIO
#if (__CORTEX_M == 0x00U)
#define EXTI_IRQ_PRIO     3
#else
#define EXTI_IRQ_PRIO     6
#endif
#endif
#ifndef EXTI_IRQ_SUBPRIO
#define EXTI_IRQ_SUBPRIO  0
#endif

void gpio_interrupt_enable(uint32_t gpioPort, uint16_t pin, void (*callback)(void), uint32_t mode);
void gpio_interrupt_disable(uint32_t gpioPort, uint16_t pin);

#endif /* SPL_EXTI_ENABLE */

#endif /* _GPIO_INTERRUPT_ */
