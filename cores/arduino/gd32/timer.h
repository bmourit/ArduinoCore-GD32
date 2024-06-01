/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef _TIMER_H_
#define _TIMER_H_

#include "gd32xxyy.h"
#include "PeripheralPins.h"
#include "PeripheralNames.h"

#ifdef __cplusplus
extern "C" {
#endif

/* timer interrupts */
#ifndef TIMER_IRQ_PRIORITY
	#define TIMER_IRQ_PRIORITY	14
#endif
#ifndef TIMER_IRQ_SUBPRIORITY
	#define TIMER_IRQ_SUBPRIORITY	0
#endif

#if defined(TIMER0) && !defined(TIMER0_IRQn)
#if defined(GD32F30x)
#if defined(GD32F30X_CL) || defined(GD32F30X_XD)
//#define TIMER0_IRQn			TIMER0_UP_TIMER9_IRQn
//#define TIMER0_IRQHandler	TIMER0_UP_TIMER9_IRQHandler
#else
//#define TIMER0_IRQn			TIMER0_UP_IRQn
//#define TIMER0_IRQHandler	TIMER0_UP_IRQHandler
#endif
#endif
#endif

#if defined(TIMER7) && !defined(TIMER7_IRQn)
#if defined(GD32F30x)
#if defined(GD32F30X_CL) || defined(GD32F30X_XD)
#define TIMER7_IRQn		TIMER7_UP_TIMER12_IRQn
#define TIMER7_IRQHandler	TIMER7_UP_TIMER12_IRQHandler
#else
#define TIMER7_IRQn		TIMER7_UP_IRQn
#define TIMER7_IRQHandler	TIMER7_UP_IRQHandler
#endif
#endif
#endif

#if defined(TIMER8) && !defined(TIMER8_IRQn)
#if defined(GD32F30x)
#if defined(GD32F30X_CL) || defined(GD32F30X_XD)
#define TIMER8_IRQn		TIMER0_BRK_TIMER8_IRQn
#define TIMER8_IRQHandler	TIMER0_BRK_TIMER8_IRQHandler
#endif
#endif
#endif

#if defined(TIMER9) && !defined(TIMER9_IRQn)
#if defined(GD32F30x)
#if defined(GD32F30X_CL) || defined(GD32F30X_XD)
#define TIMER9_IRQn		TIMER0_UP_TIMER9_IRQn
//TIMER9_IRQHandler is mapped on TIMER0_IRQHandler when TIMER9_IRQn is not defined
#endif
#endif
#endif

#if defined(TIMER10) && !defined(TIMER10_IRQn)
#if defined(GD32F30x)
#if defined(GD32F30X_CL) || defined(GD32F30X_XD)
#define TIMER10_IRQn		TIMER0_TRG_CMT_TIMER10_IRQn
#define TIMER10_IRQHandler	TIMER0_TRG_CMT_TIMER10_IRQHandler
#endif
#endif
#endif

#if defined(TIMER11) && !defined(TIMER11_IRQn)
#if defined(GD32F30x)
#if defined(GD32F30X_CL) || defined(GD32F30X_XD)
#define TIMER11_IRQn		TIMER7_BRK_TIMER11_IRQn
#define TIMER11_IRQHandler	TIMER7_BRK_TIMER11_IRQHandler
#endif
#endif
#endif

#if defined(TIMER12) && !defined(TIMER12_IRQn)
#if defined(GD32F30x)
#if defined(GD32F30X_CL) || defined(GD32F30X_XD)
#define TIMER12_IRQn		TIMER7_UP_TIMER12_IRQn
#endif
#endif
#endif

#if defined(TIMER13) && !defined(TIMER13_IRQn)
#if defined(GD32F30x)
#if defined(GD32F30X_CL) || defined(GD32F30X_XD)
#define TIMER13_IRQn		TIMER7_TRG_CMT_TIMER13_IRQn
#define TIMER13_IRQHandler	TIMER7_TRG_CMT_TIMER13_IRQHandler
#endif
#endif
#endif

typedef enum {
#if defined(TIMER0)
	TIMER0_INDEX,
#endif
#if defined(TIMER1)
	TIMER1_INDEX,
#endif
#if defined(TIMER2)
	TIMER2_INDEX,
#endif
#if defined(TIMER3)
	TIMER3_INDEX,
#endif
#if defined(TIMER4)
	TIMER4_INDEX,
#endif
#if defined(TIMER5)
	TIMER5_INDEX,
#endif
#if defined(TIMER6)
	TIMER6_INDEX,
#endif
#if defined(TIMER7)
	TIMER7_INDEX,
#endif
#if defined(TIMER8)
	TIMER8_INDEX,
#endif
#if defined(TIMER9)
	TIMER9_INDEX,
#endif
#if defined(TIMER10)
	TIMER10_INDEX,
#endif
#if defined(TIMER11)
	TIMER11_INDEX,
#endif
#if defined(TIMER12)
	TIMER12_INDEX,
#endif
#if defined(TIMER13)
	TIMER13_INDEX,
#endif
	TIMER_NUM,
	UNKNOWN_TIMER = 0xFFFF
} timer_index_t;

enum captureCompareFormat {
	CC_FORMAT_1B = 1,
	CC_FORMAT_2B,
	CC_FORMAT_3B,
	CC_FORMAT_4B,
	CC_FORMAT_5B,
	CC_FORMAT_6B,
	CC_FORMAT_7B,
	CC_FORMAT_8B,
	CC_FORMAT_9B,
	CC_FORMAT_10B,
	CC_FORMAT_11B,
	CC_FORMAT_12B,
	CC_FORMAT_13B,
	CC_FORMAT_14B,
	CC_FORMAT_15B,
	CC_FORMAT_16B,
	CC_FORMAT_TICK = 0x80,
	CC_FORMAT_US,
	CC_FORMAT_HZ,
	CC_FORMAT_PERCENT
};

enum captureMode {
	TIMER_DISABLED,
	IC_RISING_EDGE,
	IC_FALLING_EDGE,
	IC_BOTH_EDGE,
	IC_MEASUREMENT,
	OC_TIMING,
	OC_ACTIVE,
	OC_INACTIVE,
	OC_TOGGLE,
	OC_LOW,
	OC_HIGH,
	OC_PWM0,
	OC_PWM1,
	TIMER_UNUSED = 0xffff
};

enum timeFormat {
	FORMAT_TICK,
	FORMAT_US,
	FORMAT_MS,
	FORMAT_S,
	FORMAT_HZ
};

typedef enum {
	CHAN0_ACTIVE = 0x1U,
	CHAN1_ACTIVE = 0x2U,
	CHAN2_ACTIVE = 0x4U,
	CHAN3_ACTIVE = 0x8U,
	CHAN_ALL_CLEARED = 0x00U
} timer_active_channel_t;

typedef struct {
	TIMERName timer_instance;
	uint32_t Channel;
	bool isTimerActive;
	timer_parameter_struct init_params;
} SPL_TimerHandle_t;

typedef struct {
	void *_timer_instance;
	SPL_TimerHandle_t handle;
	uint32_t timerPreemptPriority;
	uint32_t timerSubPriority;
} timerDevice_t;

timerDevice_t *get_timer_object(SPL_TimerHandle_t *timerHandle);	// get the timer object (timerDevice_t) belonging to the timer

void Timer_clock_enable(SPL_TimerHandle_t *timerHandle);				// enable timer clock
void Timer_clock_disable(SPL_TimerHandle_t *timerHandle);			// disable timer clock

void Timer_init(SPL_TimerHandle_t *timerHandle);						// initialize timer

void Timer_enableUpdateIT(TIMERName instance);								// enable timer update interrupt
void Timer_disableUpdateIT(TIMERName instance);								// disable timer update interrupt
void Timer_enableCaptureIT(TIMERName instance, uint32_t interrupt);	// enable timer channel capture interrupt
void Timer_disableCaptureIT(TIMERName instance, uint32_t interrupt);	// disable timer channel capture interrupt

uint32_t  getTimerClkFrequency(TIMERName instance);		// get timer clock frequency
IRQn_Type getTimerUpIrq(TIMERName timer);					// get timer update IRQn
IRQn_Type getTimerCCIrq(TIMERName timer);					// get timer capture/compare IRQn

void timerInterruptHandler(SPL_TimerHandle_t *timer_handle);
void timer_captureHandle(SPL_TimerHandle_t *timer_handle);
void timer_updateHandle(SPL_TimerHandle_t *timer_handle);

#ifdef __cplusplus
}
#endif

#endif /* _TIMER_H_ */
