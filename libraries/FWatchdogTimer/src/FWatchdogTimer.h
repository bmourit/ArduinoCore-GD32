/*
  Copyright (c) 2018 Frederic Pillon <frederic.pillon@st.com> for
  STMicroelectronics. All right reserved.
  Copyright (c) 2018 Venelin Efremov <ghent360@iqury.us>

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
#ifndef __FWATCHDOGTIMER_H__
#define __FWATCHDOGTIMER_H__

#include "Arduino.h"

#define IRC40K_VAL  40000U
// Minimal timeout in microseconds
#define FWDGT_TIMEOUT_MIN    ((4 * 1000000) / IRC40K_VAL)
// Maximal timeout in microseconds
#define FWDGT_TIMEOUT_MAX    (((256 * 1000000) / IRC40K_VAL) * FWDGT_RLD_RLD)

#define IS_FWDGT_TIMEOUT(X) (((X) >= FWDGT_TIMEOUT_MIN) &&\
                             ((X) <= FWDGT_TIMEOUT_MAX))

class FWatchdogTimerClass {

  public:
    void begin(uint32_t timeout);
    void set(uint32_t timeout);
    void get(uint32_t *timeout);
    void reload(void);
    bool isEnabled(void)
    {
      return _enabled;
    };
    bool isReset(bool clear = false);
    void clearReset(void);

  private:
    static bool _enabled;
};

extern FWatchdogTimerClass FWatchdogTimer;
#endif /* __IWATCHDOG_H__ */
