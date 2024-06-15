/* mbed Microcontroller Library
 * Copyright (c) 2018 GigaDevice Semiconductor Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PERIPHERALNAMES_H
#define PERIPHERALNAMES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32_def"

typedef enum {
    ADC_0 = (int)ADC0,
    ADC_1 = (int)ADC1,
    ADC_2 = (int)ADC2
} ADCName;

typedef enum {
    DAC_0 = (int)DAC0,
    DAC_1 = (int)DAC1
} DACName;

typedef enum {
    UART_0 = (int)USART0,
    UART_1 = (int)USART1,
    UART_2 = (int)USART2,
    UART_3 = (int)UART3,
    UART_4 = (int)UART4
} UARTName;

typedef enum {
    SPI_0 = (int)SPI0,
    SPI_1 = (int)SPI1,
    SPI_2 = (int)SPI2
} SPIName;

typedef enum {
    I2C_0 = (int)I2C0,
    I2C_1 = (int)I2C1
} I2CName;

typedef enum {
    TIMER_0 = (int)TIMER0,
    TIMER_1 = (int)TIMER1,
    TIMER_2 = (int)TIMER2,
    TIMER_3 = (int)TIMER3,
    TIMER_4 = (int)TIMER4,
    TIMER_5 = (int)TIMER5,
    TIMER_6 = (int)TIMER6,
    TIMER_7 = (int)TIMER7,
    TIMER_8 = (int)TIMER8,
    TIMER_9 = (int)TIMER9,
    TIMER_10 = (int)TIMER10,
    TIMER_11 = (int)TIMER11,
    TIMER_12 = (int)TIMER12,
    TIMER_13 = (int)TIMER13
} TIMERName;

#ifdef __cplusplus
}
#endif

#endif
