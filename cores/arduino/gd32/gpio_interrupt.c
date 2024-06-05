#include "gpio_interrupt.h"
#include "pinmap.h"
#include "pins_arduino.h"
#include "gd32xxyy_gpio.h"

#define EXTI_NUMS   (16)

#ifdef __cplusplus
extern "C" {
#endif

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

void gpio_interrupt_enable(uint32_t portNum, uint32_t pinNum, void (*callback)(void), uint32_t mode)
{
  exti_line_enum exti_line = BIT(pinNum);
  exti_mode_enum exti_mode = EXTI_INTERRUPT;
  exti_trig_type_enum trig_type = mode;
  gpio_exti_info[pinNum].callback = callback;

  gpio_clock_enable(portNum);
#if defined(GD32F30x) || defined(GD32E50X)
  rcu_periph_clock_enable(RCU_AF);
  // do not re-init to IN_FLOATING, this might destroy previously set INPUT_PULLUP or INPUT_PULLDOWN modes!
  // only messes up things if pin was in OUTPUT / OUTPUT_OPENDRAIN mode previously, but, I don't think
  // anyone expects interrupt to work when they explicitly initialize a pin in OUTPUT mode...
  //gpio_init(portNum, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, pinNum);
#elif defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
  rcu_periph_clock_enable(RCU_CFGCMP);
  //gpio_mode_set(portNum, GPIO_MODE_INPUT, GPIO_PUPD_NONE, pinNum);
#endif

  /* some NVIC controllers do not have subprio? */
#if defined(GD32E23x)
  nvic_irq_enable(gpio_exti_info[pinNum].irqNum, EXTI_IRQ_PRIO);
#else
  nvic_irq_enable(gpio_exti_info[pinNum].irqNum, EXTI_IRQ_PRIO, EXTI_IRQ_SUBPRIO);
#endif
#if defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32E23x)
  syscfg_exti_line_config((uint8_t)portNum, (uint8_t)pinNum);
#elif defined(GD32F30x) || defined(GD32E50X)
  uint8_t port_source = (uint8_t)NC;
  switch (portNum) {
    case PORTA:
      port_source = GPIO_PORT_SOURCE_GPIOA;
      break;
    case PORTB:
      port_source = GPIO_PORT_SOURCE_GPIOB;
      break;
#ifdef GPIOC
    case PORTC:
      port_source = GPIO_PORT_SOURCE_GPIOC;
      break;
#endif
#ifdef GPIOD
    case PORTD:
      port_source = GPIO_PORT_SOURCE_GPIOD;
      break;
#endif
#ifdef GPIOE
    case PORTE:
      port_source = GPIO_PORT_SOURCE_GPIOE;
      break;
#endif
#ifdef GPIOF
    case PORTF:
      port_source = GPIO_PORT_SOURCE_GPIOF;
      break;
#endif
#ifdef GPIOG
    case PORTG:
      port_source = GPIO_PORT_SOURCE_GPIOG;
      break;
#endif
    default:
      break;
  }

  uint8_t pin_source = (uint8_t)NC;
  switch (gpio_pin[pinNum]) {
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

void gpio_interrupt_disable(uint32_t pinNum)
{
  gpio_exti_info[pinNum].callback = NULL;

  int i = 0;
  for (i = 0; i < EXTI_NUMS; i++) {
    if (gpio_exti_info[pinNum].irqNum == gpio_exti_info[i].irqNum  && \
        NULL != gpio_exti_info[i].callback) {
      return;
    }
  }
  nvic_irq_disable(gpio_exti_info[pinNum].irqNum);
}

void exti_callbackHandler(uint32_t pinNum)
{
  exti_line_enum linex = (exti_line_enum)BIT(pinNum);
  if (RESET != exti_interrupt_flag_get(linex)) {
    exti_interrupt_flag_clear(linex);
    if (NULL != gpio_exti_info[pinNum].callback) {
      gpio_exti_info[pinNum].callback();
    }
  }
}

#if defined(GD32F30x) || defined(GD32E50X)
void EXTI0_IRQHandler(void)
{
  exti_callbackHandler(0);
}

void EXTI1_IRQHandler(void)
{
  exti_callbackHandler(1);
}

void EXTI2_IRQHandler(void)
{
  exti_callbackHandler(2);
}

void EXTI3_IRQHandler(void)
{
  exti_callbackHandler(3);
}

void EXTI4_IRQHandler(void)
{
  exti_callbackHandler(4);
}

void EXTI5_9_IRQHandler(void)
{
  uint32_t i;
  for (i = 5; i < 10; i++) {
    exti_callbackHandler(i);
  }
}

void EXTI10_15_IRQHandler(void)
{
  uint32_t i;
  for (i = 10; i < 16; i++) {
    exti_callbackHandler(i);
  }
}
#elif defined(GD32F3x0) || defined(GD32F1x0)
void EXTI0_1_IRQHandler(void)
{
  uint32_t i;
  for (i = 0; i <= 1; i++) {
    exti_callbackHandler(i);
  }
}

void EXTI2_3_IRQHandler(void)
{
  uint32_t i;
  for (i = 2; i <= 3; i++) {
    exti_callbackHandler(i);
  }
}

void EXTI4_15_IRQHandler(void)
{
  uint32_t i;
  for (i = 4; i <= 15; i++) {
    exti_callbackHandler(i);
  }
}
#elif defined(GD32E23x)
void EXTI0_1_IRQHandler(void)
{
  uint32_t i;
  for ( i = 0; i < 2; i++) {
    exti_callbackHandler(i);
  }
}
void EXTI2_3_IRQHandler(void)
{
  uint32_t i;
  for (i = 2; i < 4; i++) {
    exti_callbackHandler(i);
  }
}
void EXTI4_15_IRQHandler(void)
{
  uint32_t i;
  for (i = 4; i < 16; i++) {
    exti_callbackHandler(i);
  }
}
#endif

#ifdef __cplusplus
}
#endif
