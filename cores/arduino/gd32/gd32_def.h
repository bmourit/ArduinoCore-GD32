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

#ifndef _GD32_DEF_
#define _GD32_DEF_

/**
 * @brief GD32 core version number
 */
#define GD32_CORE_VERSION_MAJOR    (0x01U) /*!< [23:16] major version */
#define GD32_CORE_VERSION_MINOR    (0x00U) /*!< [15:8] minor version */
#define GD32_CORE_VERSION_PATCH    (0x00U) /*!< [7:0]  patch version */
#define GD32_CORE_VERSION          ((GD32_CORE_VERSION_MAJOR << 16U) \
                                    | (GD32_CORE_VERSION_MINOR << 8U) \
                                    | (GD32_CORE_VERSION_PATCH))


#if defined(GD32F10x)
#include "gd32f10x.h"
#elif defined(GD32F1x0)
#include "gd32f1x0.h"
#elif defined(GD32F20x)
#include "gd32f20x.h"
#elif defined(GD32F3x0)
#include "gd32f3x0.h"
#elif defined(GD32F30x)
#include "gd32f30x.h"
#elif defined(GD32F4xx)
#include "gd32f4xx.h"
#elif defined(GD32F403)
#include "gd32f403.h"
#elif defined(GD32E10X)
#include "gd32e10x.h"
#elif defined(GD32E23x)
#include "gd32e23x.h"
#elif defined(GD32E50X)
#include "gd32e50x.h"
#else
#error "Unknown chip series"
#endif

#ifndef F_CPU
  #define F_CPU SystemCoreClock
#endif

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

/**
 * Libc porting layers
 */
#if defined(  __GNUC__  ) /* GCC CS3 */
#define WEAK __attribute__((weak))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* weaked */
void SystemClock_Config(void);

void _Error_Handler(const char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#ifdef __cplusplus
}
#endif

#endif /* _GD32_DEF_ */
