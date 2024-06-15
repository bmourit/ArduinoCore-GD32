#include "Arduino.h"
#include "gpio_interrupt.h"
#include <cstddef>

#if defined(SPL_EXTI_ENABLE)

#define EXTI_NUMS   (16)

typedef struct {
  IRQn_Type irqNum;
  void (*callback)(void);
} extiConf_t;

static extiConf_t gpio_exti_info[EXTI_NUMS] = {
#if defined(GD32F30x) || defined(GD32E50X)
  {.irqNum = EXTI0_IRQn, .callback = NULL },
  {.irqNum = EXTI1_IRQn, .callback = NULL },
  {.irqNum = EXTI2_IRQn, .callback = NULL },
  {.irqNum = EXTI3_IRQn, .callback = NULL },
  {.irqNum = EXTI4_IRQn, .callback = NULL },
  {.irqNum = EXTI5_9_IRQn, .callback = NULL },
  {.irqNum = EXTI5_9_IRQn, .callback = NULL },
  {.irqNum = EXTI5_9_IRQn, .callback = NULL },
  {.irqNum = EXTI5_9_IRQn, .callback = NULL },
  {.irqNum = EXTI5_9_IRQn, .callback = NULL },
  {.irqNum = EXTI10_15_IRQn, .callback = NULL },
  {.irqNum = EXTI10_15_IRQn, .callback = NULL },
  {.irqNum = EXTI10_15_IRQn, .callback = NULL },
  {.irqNum = EXTI10_15_IRQn, .callback = NULL },
  {.irqNum = EXTI10_15_IRQn, .callback = NULL },
  {.irqNum = EXTI10_15_IRQn, .callback = NULL }
#elif defined(GD32F3x0) || defined(GD32F1x0)
  {.irqNum = EXTI0_1_IRQn, .callback = NULL },
  {.irqNum = EXTI0_1_IRQn, .callback = NULL },
  {.irqNum = EXTI2_3_IRQn, .callback = NULL },
  {.irqNum = EXTI2_3_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL }
#elif defined(GD32E23x)
  {.irqNum = EXTI0_1_IRQn, .callback = NULL },
  {.irqNum = EXTI0_1_IRQn, .callback = NULL },
  {.irqNum = EXTI2_3_IRQn, .callback = NULL },
  {.irqNum = EXTI2_3_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL },
  {.irqNum = EXTI4_15_IRQn, .callback = NULL }
#endif
};

static uint8_t get_exti_pin_id(uint16_t pin)
{
  uint8_t id = 0;

  while (pin != 0x0001) {
    pin = pin >> 1;
    id++;
  }

  return id;
}

void gpio_interrupt_enable(uint32_t gpioPort, uint16_t pin, void (*callback)(void), uint32_t mode)
{
  uint8_t id = get_exti_pin_id(pin);
  exti_line_enum exti_line = (exti_line_enum)BIT(id);
  exti_mode_enum exti_mode = (exti_mode_enum)EXTI_INTERRUPT;
  exti_trig_type_enum trig_type = (exti_trig_type_enum)mode;
  gpio_exti_info[id].callback = callback;
#if defined(GD32F30x) || defined(GD32E50X)
  uint8_t pos;
  uint32_t ctlReg;
  uint32_t ctlRegOffset = 0;
  uint32_t octlRegOffset = 0;
  const uint32_t configMask = 0x00000008;
#endif
  uint32_t pupd;

/**
 * get the currently configured mode and use it
 * in the gpio_init function to avoid misconfiguration
 */
#if defined(GD32F30x) || defined(GD32E50X)
  ctlReg = (pin < GPIO_PIN_8) ? GPIO_CTL0(gpioPort) : GPIO_CTL1(gpioPort);

  for (pos = 0; pos < 16; pos++) {
    if (pin == (0x0001 << pos)) {
      ctlRegOffset = (pin < GPIO_PIN_8) ? (pos << 2) : ((pos - 8) << 2);
      octlRegOffset = pos;
    }
  }

  if ((ctlReg & ((GPIO_CTL0_MD0 | GPIO_CTL0_CTL0) << ctlRegOffset)) == (configMask << ctlRegOffset)) {
    if ((GPIO_OCTL(gpioPort) & (GPIO_OCTL_OCTL0 << octlRegOffset)) == (GPIO_OCTL_OCTL0 << octlRegOffset)) {
      pupd = GPIO_MODE_IPU;
    } else {
      pupd = GPIO_MODE_IPD;
    }
  } else {
    pupd = GPIO_MODE_IN_FLOATING;
  }
  rcu_periph_clock_enable(RCU_AF);
  /**
   * use the currently set mode and configure the port
   * NOTE: only pins set as input will be attaching interrupts.
   */
  gpio_init(gpioPort, pupd, GPIO_OSPEED_50MHZ, pin);
#elif defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
  rcu_periph_clock_enable(RCU_CFGCMP);
  //gpio_mode_set(portNum, GPIO_MODE_INPUT, GPIO_PUPD_NONE, pinNum);
#endif

  /* some NVIC controllers do not have subprio? */
#if defined(GD32E23x)
  nvic_irq_enable(gpio_exti_info[id].irqNum, EXTI_IRQ_PRIO);
#else
  nvic_irq_enable(gpio_exti_info[id].irqNum, EXTI_IRQ_PRIO, EXTI_IRQ_SUBPRIO);
#endif
#if defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
  syscfg_exti_line_config((uint8_t)gpioPort, (uint8_t)id);
#elif defined(GD32F30x) || defined(GD32E50X)
  uint8_t port_source = (uint8_t)NC;
  switch (gpioPort) {
    case GPIOA:
      port_source = GPIO_PORT_SOURCE_GPIOA;
      break;
    case GPIOB:
      port_source = GPIO_PORT_SOURCE_GPIOB;
      break;
#ifdef GPIOC
    case GPIOC:
      port_source = GPIO_PORT_SOURCE_GPIOC;
      break;
#endif
#ifdef GPIOD
    case GPIOD:
      port_source = GPIO_PORT_SOURCE_GPIOD;
      break;
#endif
    default:
      break;
  }

  uint8_t pin_source = (uint8_t)NC;
  switch (pin) {
    case GPIO_PIN_0:
      pin_source = GPIO_PIN_SOURCE_0;
      break;
    case GPIO_PIN_1:
      pin_source = GPIO_PIN_SOURCE_1;
      break;
    case GPIO_PIN_2:
      pin_source = GPIO_PIN_SOURCE_2;
      break;
    case GPIO_PIN_3:
      pin_source = GPIO_PIN_SOURCE_3;
      break;
    case GPIO_PIN_4:
      pin_source = GPIO_PIN_SOURCE_4;
      break;
    case GPIO_PIN_5:
      pin_source = GPIO_PIN_SOURCE_5;
      break;
    case GPIO_PIN_6:
      pin_source = GPIO_PIN_SOURCE_6;
      break;
    case GPIO_PIN_7:
      pin_source = GPIO_PIN_SOURCE_7;
      break;
    case GPIO_PIN_8:
      pin_source = GPIO_PIN_SOURCE_8;
      break;
    case GPIO_PIN_9:
      pin_source = GPIO_PIN_SOURCE_9;
      break;
    case GPIO_PIN_10:
      pin_source = GPIO_PIN_SOURCE_10;
      break;
    case GPIO_PIN_11:
      pin_source = GPIO_PIN_SOURCE_11;
      break;
    case GPIO_PIN_12:
      pin_source = GPIO_PIN_SOURCE_12;
      break;
    case GPIO_PIN_13:
      pin_source = GPIO_PIN_SOURCE_13;
      break;
    case GPIO_PIN_14:
      pin_source = GPIO_PIN_SOURCE_14;
      break;
    case GPIO_PIN_15:
      pin_source = GPIO_PIN_SOURCE_15;
      break;
    default:
      break;
  }

  gpio_exti_source_select(port_source, pin_source);
#endif

  exti_init(exti_line, exti_mode, trig_type);
  exti_interrupt_flag_clear(exti_line);
}

void gpio_interrupt_disable(uint32_t gpioPort, uint16_t pin)
{
  UNUSED(gpioPort);
  uint8_t id = get_exti_pin_id(pin);
  gpio_exti_info[id].callback = NULL;

  for (int i = 0; i < EXTI_NUMS; i++) {
    if (gpio_exti_info[id].irqNum == gpio_exti_info[i].irqNum
        && gpio_exti_info[i].callback != NULL) {
      return;
    }
  }
  nvic_irq_disable(gpio_exti_info[id].irqNum);
}

void exti_callbackHandler(uint16_t pin)
{
  uint8_t irq_id = get_exti_pin_id(pin);

  exti_line_enum linex = (exti_line_enum)BIT(irq_id);
  if (exti_interrupt_flag_get(linex) != RESET) {
    exti_interrupt_flag_clear(linex);
    if (gpio_exti_info[irq_id].callback != NULL) {
      gpio_exti_info[irq_id].callback();
    }
  }
}

#ifdef __cplusplus
extern "C" {
#endif

#if defined(GD32F30x) || defined(GD32E50X)
void EXTI0_IRQHandler(void)
{
  exti_callbackHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)
{
  exti_callbackHandler(GPIO_PIN_1);
}

void EXTI2_IRQHandler(void)
{
  exti_callbackHandler(GPIO_PIN_2);
}

void EXTI3_IRQHandler(void)
{
  exti_callbackHandler(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
  exti_callbackHandler(GPIO_PIN_4);
}

void EXTI5_9_IRQHandler(void)
{
  uint32_t i;
  for (i = GPIO_PIN_5; i <= GPIO_PIN_9; i = i << 1) {
    exti_callbackHandler(i);
  }
}

void EXTI10_15_IRQHandler(void)
{
  uint32_t i;
  for (i = GPIO_PIN_10; i <= GPIO_PIN_15; i = i << 1) {
    exti_callbackHandler(i);
  }
}
#elif defined(GD32F3x0) || defined(GD32F1x0)
void EXTI0_1_IRQHandler(void)
{
  uint32_t i;
  for (i = GPIO_PIN_0; i <= GPIO_PIN_1; i = i << 1) {
    exti_callbackHandler(i);
  }
}

void EXTI2_3_IRQHandler(void)
{
  uint32_t i;
  for (i = GPIO_PIN_2; i <= GPIO_PIN_3; i = i << 1) {
    exti_callbackHandler(i);
  }
}

void EXTI4_15_IRQHandler(void)
{
  uint32_t i;
  for (i = GPIO_PIN_4; i <= GPIO_PIN_15; i = i << 1) {
    exti_callbackHandler(i);
  }
}
#elif defined(GD32E23x)
void EXTI0_1_IRQHandler(void)
{
  uint32_t i;
  for (i = GPIO_PIN_0; i <= GPIO_PIN_1; i = i << 1) {
    exti_callbackHandler(i);
  }
}
void EXTI2_3_IRQHandler(void)
{
  uint32_t i;
  for (i = GPIO_PIN_2; i <= GPIO_PIN_3; i = i << 1) {
    exti_callbackHandler(i);
  }
}
void EXTI4_15_IRQHandler(void)
{
  uint32_t i;
  for (i = GPIO_PIN_4; i <= GPIO_PIN_15; i = i << 1) {
    exti_callbackHandler(i);
  }
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* SPL_EXTI_ENABLE */
