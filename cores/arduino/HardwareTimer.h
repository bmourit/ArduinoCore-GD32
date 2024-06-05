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

#ifndef HARDWARETIMER_H
#define HARDWARETIMER_H

extern "C" {
#include "timer.h"
}

// Copied from 1.0.x firmware library for compatibility, but this is
// unspecified in the data sheet. Being able to use both edges is not
// described as supported behavior, and bit 4 is in a reserved section
// of the TIMERx_CHCTL2 register. -bjc (2021-Aug-20)
#ifndef TIMER_IC_POLARITY_BOTH_EDGE
#define TIMER_IC_POLARITY_BOTH_EDGE         ((uint16_t)0x000AU)                     /*!< input capture both edge(not for timer1..6) */
#endif

#define TIMER_NUM_CHANNELS 4

#ifdef __cplusplus

typedef void(*timerCallback_t)(void);

class HardwareTimer
{
  public:
    HardwareTimer(void) {};                                                         //default construct
    HardwareTimer(TIMERName instance);                                              //HardwareTimer construct

    void timerStart(void);                                                          //start or resume timer
    void timerStop(void);                                                           //stop timer
    void startChannel(uint8_t channel);                                             //start or resume a timer channel
    void stopChannel(uint8_t channel);                                              //stop or pause a timer channel (leaves timer running but channel output/interrupt is disabled

    void setPrescaler(uint16_t prescaler);                                          //set prescaler
    uint32_t getPrescaler(void);                                                    //get prescaler

    void setAutoReloadValue(uint32_t value, enum timeFormat format = FORMAT_TICK);  //set reload value (overflow)
    uint32_t getAutoReloadValue(enum timeFormat format = FORMAT_TICK);              //get reload value (overflow)

    void setPWM(uint8_t channel, PinName pin, uint32_t freq,
            uint32_t dutycycle, timerCallback_t PeriodCallback = nullptr,
            timerCallback_t CompareCallback = nullptr);                             //set freq in Hz, dutycycle in percentage including both interrups in one function
    void setPWM(uint8_t channel, uint32_t pin, uint32_t freq,
            uint32_t dutycycle, timerCallback_t PeriodCallback = nullptr,
            timerCallback_t CompareCallback = nullptr);                             //set freq in Hz, dutycycle in percentage including both interrups in one function

    void setCounter(uint16_t count, enum timeFormat format = FORMAT_TICK);          //set counter
    uint32_t getCounter(enum timeFormat format = FORMAT_TICK);                      //get counter

    void setChannelMode(uint8_t channel, captureMode mode, PinName pin = NC);
    void setChannelMode(uint8_t channel, captureMode mode, uint32_t pin);

    captureMode getChannelMode(uint8_t channel);

    void setPreloadARSEnable(bool val);                                             //set preload enable (ARSE) 

    uint32_t getCaptureCompare(uint8_t channel,
                        enum captureCompareFormat format = CC_FORMAT_TICK);         //get cc mode
    void setCaptureCompare(uint8_t channel, uint32_t cvalue,
                        enum captureCompareFormat format = CC_FORMAT_TICK);         //set cc mode

    void setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority);      //set timer priority

    void attachInterrupt(timerCallback_t callback);                                 //attach callback for update interrupt
    void detachInterrupt(void);                                                     //detach callback for update interrupt
    bool hasInterrupt(void);                                                        //returns true if a timer interrupt has been set for update

    void attachInterrupt(timerCallback_t callback, uint8_t channel);                //attach callback for capture/compare interrupt
    void detachInterrupt(uint8_t channel);                                          //detach callback for capture/compare interrupt
    bool hasInterrupt(uint8_t channel);                                             //returns true if a timer interrupt has already been set on capture/compare

    void refresh(void);                                                             //update some registers to restart counters

    uint32_t getTimerClkFreq(void);                                                 //gets timer clock frequency in hertz

    static void captureCompareCallback(SPL_TimerHandle_t *timer_handle);            //capture callback
    static void updateCallback(SPL_TimerHandle_t *timer_handle);                    //update callback

    SPL_TimerHandle_t *getHandle();                                                 //returns the handle address for SPL
    int getChannel(uint32_t channel);
    int getInterruptChannel(uint32_t channel);
    int getLinkedChannel(uint32_t channel);

#if defined(TIMER_CHCTL2_CH0NEN)
    bool isLinkedChannel[TIMER_NUM_CHANNELS];
#endif

  private:
    timerCallback_t timerCallbacks[1 + TIMER_NUM_CHANNELS];
    captureMode _ChannelMode[TIMER_NUM_CHANNELS];
    timerDevice_t _timerObj;
};

extern timerDevice_t *HWTimer_Handle[TIMER_NUM];

extern timer_index_t get_timer_index(TIMERName htimer);

#endif /* __cplusplus */

#endif /* HARDWARETIMER_H */