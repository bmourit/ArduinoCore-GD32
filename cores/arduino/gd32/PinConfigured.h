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

#ifndef _PINCONFIGURED_H
#define _PINCONFIGURED_H

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PINCFG_INDEX(X) (GD_PORT_GET(X) - FirstPort)

#define PINCFG_MASK     0x01
#define PINCFG_OFFSET(x)   (GD_PIN_GET(x))
#define PINCFG_BIT(x)      (PINCFG_MASK << PINCFG_OFFSET(x))

#define PIN_STATE_VAL(x, y)   ((y >> PINCFG_OFFSET(x)) & PINCFG_MASK)

#define CHECK_PIN_STATE(pin, port) (PIN_STATE_VAL(pin, port[PINCFG_INDEX(pin)]))
#define SET_PIN_STATE(pin, port)   (port[PINCFG_INDEX(pin)] = port[PINCFG_INDEX(pin)] | PINCFG_BIT(pin))
#define RESET_PIN_STATE(pin, port) (port[PINCFG_INDEX(pin)] = port[PINCFG_INDEX(pin)] & (~PINCFG_BIT(pin)))

#ifdef __cplusplus
}
#endif

#endif
