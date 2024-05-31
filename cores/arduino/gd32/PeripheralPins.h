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

#ifndef _PERIPHERALPINS_H
#define _PERIPHERALPINS_H

#include "pinmap.h"

#ifdef GD32F30x
#include "gd32f30x_remap.h"
#endif

extern const int GD_GPIO_REMAP[];
extern const int GD_GPIO_MODE[];
extern const int GD_GPIO_SPEED[];

/* ADC */
extern const PinMap PinMap_ADC[];

/* DAC */
extern const PinMap PinMap_DAC[];

/* I2C */
extern const PinMap PinMap_I2C_SDA[];
extern const PinMap PinMap_I2C_SCL[];

/* PWM */
extern const PinMap PinMap_PWM[];

/* SERIAL */
extern const PinMap PinMap_UART_TX[];
extern const PinMap PinMap_UART_RX[];
extern const PinMap PinMap_UART_RTS[];
extern const PinMap PinMap_UART_CTS[];

/* SPI */
extern const PinMap PinMap_SPI_MOSI[];
extern const PinMap PinMap_SPI_MISO[];
extern const PinMap PinMap_SPI_SCLK[];
extern const PinMap PinMap_SPI_SSEL[];

/* CAN */
extern const PinMap PinMap_CAN_RD[];
extern const PinMap PinMap_CAN_TD[];

/* USB */
extern const PinMap PinMap_USB[];

/* SDIO */
extern const PinMap PinMap_SD_CMD[];
extern const PinMap PinMap_SD_CK[];
extern const PinMap PinMap_SD_DATA0[];
extern const PinMap PinMap_SD_DATA1[];
extern const PinMap PinMap_SD_DATA2[];
extern const PinMap PinMap_SD_DATA3[];
extern const PinMap PinMap_SD_DATA4[];
extern const PinMap PinMap_SD_DATA5[];
extern const PinMap PinMap_SD_DATA6[];
extern const PinMap PinMap_SD_DATA7[];

#endif
