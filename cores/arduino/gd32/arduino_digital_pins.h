/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _ARDUINO_DIGITAL_PINS_H_
#define _ARDUINO_DIGITAL_PINS_H_

#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Arduino digital pins alias */
enum {
  D0,   D1,   D2,   D3,   D4,   D5,   D6,   D7,   D8,   D9,
  D10,  D11,  D12,  D13,  D14,  D15,  D16,  D17,  D18,  D19,
  D20,  D21,  D22,  D23,  D24,  D25,  D26,  D27,  D28,  D29,
  D30,  D31,  D32,  D33,  D34,  D35,  D36,  D37,  D38,  D39,
  D40,  D41,  D42,  D43,  D44,  D45,  D46,  D47,  D48,  D49,
  D50,  D51,  D52,  D53,  D54,  D55,  D56,  D57,  D58,  D59,
  D60,  D61,  D62,  D63,  D64,  D65,  D66,  D67,  D68,  D69,
  D70,  D71,  D72,  D73,  D74,  D75,  D76,  D77,  D78,  D79,
  D80,  D81,  D82,  D83,  D84,  D85,  D86,  D87,  D88,  D89,
  D90,  D91,  D92,  D93,  D94,  D95,  D96,  D97,  D98,  D99,
  D100, D101, D102, D103, D104, D105, D106, D107, D108, D109,
  D110, D111, D112, D113, D114, D115, D116, D117, D118, D119,
  D120, D121, D122, D123, D124, D125, D126, D127, D128, D129,
  D130, D131, D132, D133, D134, D135, D136, D137, D138, D139,
  D140, D141, D142, D143, D144, D145, D146, D147, D148, D149,
  D150, D151, D152, D153, D154, D155, D156, D157, D158, D159,
  D160, D161, D162, D163, D164, D165, D166, D167, D168, D169,
  D170, D171, D172, D173, D174, D175, D176, D177, D178, D179,
  D180, D181, D182, D183, D184, D185, D186, D187, D188, D189,
  D190, D191, DMAX
};

/**
 * Each GD32 pin number defined in the variant.h
 * is also the alternate pin number
 */
#ifdef PA0
  #define PA0_ALT0       (PA0  | ALT0)
#endif
#ifdef PA1
  #define PA1_ALT0       (PA1  | ALT0)
#endif
#ifdef PA2
  #define PA2_ALT0       (PA2  | ALT0)
#endif
#ifdef PA3
  #define PA3_ALT0       (PA3  | ALT0)
#endif
#ifdef PA4
  #define PA4_ALT0       (PA4  | ALT0)
#endif
#ifdef PA5
  #define PA5_ALT0       (PA5  | ALT0)
#endif
#ifdef PA6
  #define PA6_ALT0       (PA6  | ALT0)
#endif
#ifdef PA7
  #define PA7_ALT0       (PA7  | ALT0)
#endif
#ifdef PA8
  #define PA8_ALT0       (PA8  | ALT0)
#endif
#ifdef PA9
  #define PA9_ALT0       (PA9  | ALT0)
#endif
#ifdef PA10
  #define PA10_ALT0      (PA10 | ALT0)
#endif
#ifdef PA11
  #define PA11_ALT0      (PA11 | ALT0)
#endif
#ifdef PA12
  #define PA12_ALT0      (PA12 | ALT0)
#endif
#ifdef PA13
  #define PA13_ALT0      (PA13 | ALT0)
#endif
#ifdef PA14
  #define PA14_ALT0      (PA14 | ALT0)
#endif
#ifdef PA15
  #define PA15_ALT0      (PA15 | ALT0)
#endif
#ifdef PB0
  #define PB0_ALT0       (PB0  | ALT0)
#endif
#ifdef PB1
  #define PB1_ALT0       (PB1  | ALT0)
#endif
#ifdef PB2
  #define PB2_ALT0       (PB2  | ALT0)
#endif
#ifdef PB3
  #define PB3_ALT0       (PB3  | ALT0)
#endif
#ifdef PB4
  #define PB4_ALT0       (PB4  | ALT0)
#endif
#ifdef PB5
  #define PB5_ALT0       (PB5  | ALT0)
#endif
#ifdef PB6
  #define PB6_ALT0       (PB6  | ALT0)
#endif
#ifdef PB7
  #define PB7_ALT0       (PB7  | ALT0)
#endif
#ifdef PB8
  #define PB8_ALT0       (PB8  | ALT0)
#endif
#ifdef PB9
  #define PB9_ALT0       (PB9  | ALT0)
#endif
#ifdef PB10
  #define PB10_ALT0      (PB10 | ALT0)
#endif
#ifdef PB11
  #define PB11_ALT0      (PB11 | ALT0)
#endif
#ifdef PB12
  #define PB12_ALT0      (PB12 | ALT0)
#endif
#ifdef PB13
  #define PB13_ALT0      (PB13 | ALT0)
#endif
#ifdef PB14
  #define PB14_ALT0      (PB14 | ALT0)
#endif
#ifdef PB15
  #define PB15_ALT0      (PB15 | ALT0)
#endif
#ifdef PC0
  #define PC0_ALT0       (PC0  | ALT0)
#endif
#ifdef PC1
  #define PC1_ALT0       (PC1  | ALT0)
#endif
#ifdef PC2
  #define PC2_ALT0       (PC2  | ALT0)
#endif
#ifdef PC3
  #define PC3_ALT0       (PC3  | ALT0)
#endif
#ifdef PC4
  #define PC4_ALT0       (PC4  | ALT0)
#endif
#ifdef PC5
  #define PC5_ALT0       (PC5  | ALT0)
#endif
#ifdef PC6
  #define PC6_ALT0       (PC6  | ALT0)
#endif
#ifdef PC7
  #define PC7_ALT0       (PC7  | ALT0)
#endif
#ifdef PC8
  #define PC8_ALT0       (PC8  | ALT0)
#endif
#ifdef PC9
  #define PC9_ALT0       (PC9  | ALT0)
#endif
#ifdef PC10
  #define PC10_ALT0      (PC10 | ALT0)
#endif
#ifdef PC11
  #define PC11_ALT0      (PC11 | ALT0)
#endif
#ifdef PC12
  #define PC12_ALT0      (PC12 | ALT0)
#endif
#ifdef PC13
  #define PC13_ALT0      (PC13 | ALT0)
#endif
#ifdef PC14
  #define PC14_ALT0      (PC14 | ALT0)
#endif
#ifdef PC15
  #define PC15_ALT0      (PC15 | ALT0)
#endif
#ifdef PD0
  #define PD0_ALT0       (PD0  | ALT0)
#endif
#ifdef PD1
  #define PD1_ALT0       (PD1  | ALT0)
#endif
#ifdef PD2
  #define PD2_ALT0       (PD2  | ALT0)
#endif
#ifdef PD3
  #define PD3_ALT0       (PD3  | ALT0)
#endif
#ifdef PD4
  #define PD4_ALT0       (PD4  | ALT0)
#endif
#ifdef PD5
  #define PD5_ALT0       (PD5  | ALT0)
#endif
#ifdef PD6
  #define PD6_ALT0       (PD6  | ALT0)
#endif
#ifdef PD7
  #define PD7_ALT0       (PD7  | ALT0)
#endif
#ifdef PD8
  #define PD8_ALT0       (PD8  | ALT0)
#endif
#ifdef PD9
  #define PD9_ALT0       (PD9  | ALT0)
#endif
#ifdef PD10
  #define PD10_ALT0      (PD10 | ALT0)
#endif
#ifdef PD11
  #define PD11_ALT0      (PD11 | ALT0)
#endif
#ifdef PD12
  #define PD12_ALT0      (PD12 | ALT0)
#endif
#ifdef PD13
  #define PD13_ALT0      (PD13 | ALT0)
#endif
#ifdef PD14
  #define PD14_ALT0      (PD14 | ALT0)
#endif
#ifdef PD15
  #define PD15_ALT0      (PD15 | ALT0)
#endif
#ifdef PE0
  #define PE0_ALT0       (PE0  | ALT0)
#endif
#ifdef PE1
  #define PE1_ALT0       (PE1  | ALT0)
#endif
#ifdef PE2
  #define PE2_ALT0       (PE2  | ALT0)
#endif
#ifdef PE3
  #define PE3_ALT0       (PE3  | ALT0)
#endif
#ifdef PE4
  #define PE4_ALT0       (PE4  | ALT0)
#endif
#ifdef PE5
  #define PE5_ALT0       (PE5  | ALT0)
#endif
#ifdef PE6
  #define PE6_ALT0       (PE6  | ALT0)
#endif
#ifdef PE7
  #define PE7_ALT0       (PE7  | ALT0)
#endif
#ifdef PE8
  #define PE8_ALT0       (PE8  | ALT0)
#endif
#ifdef PE9
  #define PE9_ALT0       (PE9  | ALT0)
#endif
#ifdef PE10
  #define PE10_ALT0      (PE10 | ALT0)
#endif
#ifdef PE11
  #define PE11_ALT0      (PE11 | ALT0)
#endif
#ifdef PE12
  #define PE12_ALT0      (PE12 | ALT0)
#endif
#ifdef PE13
  #define PE13_ALT0      (PE13 | ALT0)
#endif
#ifdef PE14
  #define PE14_ALT0      (PE14 | ALT0)
#endif
#ifdef PE15
  #define PE15_ALT0      (PE15 | ALT0)
#endif
#ifdef PF0
  #define PF0_ALT0       (PF0  | ALT0)
#endif
#ifdef PF1
  #define PF1_ALT0       (PF1  | ALT0)
#endif
#ifdef PF2
  #define PF2_ALT0       (PF2  | ALT0)
#endif
#ifdef PF3
  #define PF3_ALT0       (PF3  | ALT0)
#endif
#ifdef PF4
  #define PF4_ALT0       (PF4  | ALT0)
#endif
#ifdef PF5
  #define PF5_ALT0       (PF5  | ALT0)
#endif
#ifdef PF6
  #define PF6_ALT0       (PF6  | ALT0)
#endif
#ifdef PF7
  #define PF7_ALT0       (PF7  | ALT0)
#endif
#ifdef PF8
  #define PF8_ALT0       (PF8  | ALT0)
#endif
#ifdef PF9
  #define PF9_ALT0       (PF9  | ALT0)
#endif
#ifdef PF10
  #define PF10_ALT0      (PF10 | ALT0)
#endif
#ifdef PF11
  #define PF11_ALT0      (PF11 | ALT0)
#endif
#ifdef PF12
  #define PF12_ALT0      (PF12 | ALT0)
#endif
#ifdef PF13
  #define PF13_ALT0      (PF13 | ALT0)
#endif
#ifdef PF14
  #define PF14_ALT0      (PF14 | ALT0)
#endif
#ifdef PF15
  #define PF15_ALT0      (PF15 | ALT0)
#endif
#ifdef PG0
  #define PG0_ALT0       (PG0  | ALT0)
#endif
#ifdef PG1
  #define PG1_ALT0       (PG1  | ALT0)
#endif
#ifdef PG2
  #define PG2_ALT0       (PG2  | ALT0)
#endif
#ifdef PG3
  #define PG3_ALT0       (PG3  | ALT0)
#endif
#ifdef PG4
  #define PG4_ALT0       (PG4  | ALT0)
#endif
#ifdef PG5
  #define PG5_ALT0       (PG5  | ALT0)
#endif
#ifdef PG6
  #define PG6_ALT0       (PG6  | ALT0)
#endif
#ifdef PG7
  #define PG7_ALT0       (PG7  | ALT0)
#endif
#ifdef PG8
  #define PG8_ALT0       (PG8  | ALT0)
#endif
#ifdef PG9
  #define PG9_ALT0       (PG9  | ALT0)
#endif
#ifdef PG10
  #define PG10_ALT0      (PG10 | ALT0)
#endif
#ifdef PG11
  #define PG11_ALT0      (PG11 | ALT0)
#endif
#ifdef PG12
  #define PG12_ALT0      (PG12 | ALT0)
#endif
#ifdef PG13
  #define PG13_ALT0      (PG13 | ALT0)
#endif
#ifdef PG14
  #define PG14_ALT0      (PG14 | ALT0)
#endif
#ifdef PG15
  #define PG15_ALT0      (PG15 | ALT0)
#endif
#ifdef PH0
  #define PH0_ALT0       (PH0  | ALT0)
#endif
#ifdef PH1
  #define PH1_ALT0       (PH1  | ALT0)
#endif
#ifdef PH2
  #define PH2_ALT0       (PH2  | ALT0)
#endif
#ifdef PH3
  #define PH3_ALT0       (PH3  | ALT0)
#endif
#ifdef PH4
  #define PH4_ALT0       (PH4  | ALT0)
#endif
#ifdef PH5
  #define PH5_ALT0       (PH5  | ALT0)
#endif
#ifdef PH6
  #define PH6_ALT0       (PH6  | ALT0)
#endif
#ifdef PH7
  #define PH7_ALT0       (PH7  | ALT0)
#endif
#ifdef PH8
  #define PH8_ALT0       (PH8  | ALT0)
#endif
#ifdef PH9
  #define PH9_ALT0       (PH9  | ALT0)
#endif
#ifdef PH10
  #define PH10_ALT0      (PH10 | ALT0)
#endif
#ifdef PH11
  #define PH11_ALT0      (PH11 | ALT0)
#endif
#ifdef PH12
  #define PH12_ALT0      (PH12 | ALT0)
#endif
#ifdef PH13
  #define PH13_ALT0      (PH13 | ALT0)
#endif
#ifdef PH14
  #define PH14_ALT0      (PH14 | ALT0)
#endif
#ifdef PH15
  #define PH15_ALT0      (PH15 | ALT0)
#endif
#ifdef PI0
  #define PI0_ALT0       (PI0  | ALT0)
#endif
#ifdef PI1
  #define PI1_ALT0       (PI1  | ALT0)
#endif
#ifdef PI2
  #define PI2_ALT0       (PI2  | ALT0)
#endif
#ifdef PI3
  #define PI3_ALT0       (PI3  | ALT0)
#endif
#ifdef PI4
  #define PI4_ALT0       (PI4  | ALT0)
#endif
#ifdef PI5
  #define PI5_ALT0       (PI5  | ALT0)
#endif
#ifdef PI6
  #define PI6_ALT0       (PI6  | ALT0)
#endif
#ifdef PI7
  #define PI7_ALT0       (PI7  | ALT0)
#endif
#ifdef PI8
  #define PI8_ALT0       (PI8  | ALT0)
#endif
#ifdef PI9
  #define PI9_ALT0       (PI9  | ALT0)
#endif
#ifdef PI10
  #define PI10_ALT0      (PI10 | ALT0)
#endif
#ifdef PI11
  #define PI11_ALT0      (PI11 | ALT0)
#endif
#ifdef PI12
  #define PI12_ALT0      (PI12 | ALT0)
#endif
#ifdef PI13
  #define PI13_ALT0      (PI13 | ALT0)
#endif
#ifdef PI14
  #define PI14_ALT0      (PI14 | ALT0)
#endif
#ifdef PI15
  #define PI15_ALT0      (PI15 | ALT0)
#endif

#ifdef __cplusplus
}
#endif

#endif  /* _ARDUINO_DIGITAL_PINS_H_ */
