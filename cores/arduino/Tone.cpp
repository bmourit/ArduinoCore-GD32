
#include "Arduino.h"
//#include "HardwareTimer.h"

#include "gpio_extra.h"

#define MAX_FREQ  65535

typedef struct {
  PinName pin;
  int32_t count;
} timerPinInfo_t;

static void timerTonePinInit(PinName p, uint32_t frequency, uint32_t duration);
static void tonePeriodElapsedCallback();

static timerPinInfo_t TimerTone_pinInfo = { NC, 0 };
static HardwareTimer *TimerTone = NULL;

// Tone Period elapsed callback in non-blocking mode
static void tonePeriodElapsedCallback()
{
  uint32_t port = APORT_TO_GPORT(GD_PORT_GET(TimerTone_pinInfo.pin));
  uint32_t pin =  APORT_TO_GPORT(GD_PIN_GET(TimerTone_pinInfo.pin));

  if (port != NP) {
    if (TimerTone_pinInfo.count != 0) {
      if (TimerTone_pinInfo.count > 0) {
        TimerTone_pinInfo.count--;
      }
      gpio_digital_toggle(port, pin);
    } else {
      gpio_digital_write(port, pin, 0);
    }
  }
}

static void timerTonePinDeinit()
{
  if (TimerTone != NULL) {
    TimerTone->timerStop();
  }
  if (TimerTone_pinInfo.pin != NC) {
    pin_function(TimerTone_pinInfo.pin, GD_PIN_DATA(GD_MODE_INPUT, PIN_PUPD_NONE, 0));
    TimerTone_pinInfo.pin = NC;
  }
}

static void timerTonePinInit(PinName p, uint32_t frequency, uint32_t duration)
{
  uint32_t timFreq = 2 * frequency;

  if (frequency <= MAX_FREQ) {
    if (frequency == 0) {
      if (TimerTone != NULL) {
        TimerTone->timerStop();
      }
    } else {
      TimerTone_pinInfo.pin = p;

      // Calculate the toggle count
      if (duration > 0) {
        TimerTone_pinInfo.count = ((timFreq * duration) / 1000);
      } else {
        TimerTone_pinInfo.count = -1;
      }

      pin_function(TimerTone_pinInfo.pin, GD_PIN_DATA(GD_MODE_OUT_PP, PIN_PUPD_NONE, 0));

      TimerTone->setChannelMode(1, OC_TIMING, NC);
      TimerTone->setAutoReloadValue(timFreq, FORMAT_HZ);
      TimerTone->attachInterrupt(tonePeriodElapsedCallback);
      TimerTone->timerStart();
    }
  }
}

// frequency (in hertz) and duration (in milliseconds).
void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
  PinName p = DIGITAL_TO_PINNAME(_pin);

  if (TimerTone == NULL) {
    TimerTone = new HardwareTimer((TIMERName)TIMER_TONE);
  }

  if (p != NC) {
    if ((TimerTone_pinInfo.pin == NC) || (TimerTone_pinInfo.pin == p)) {
      timerTonePinInit(p, frequency, duration);
    }
  }
}

void noTone(uint8_t _pin, bool destruct)
{
  PinName p = DIGITAL_TO_PINNAME(_pin);
  if ((p != NC) && (TimerTone_pinInfo.pin == p) && (TimerTone != NULL)) {
    if (destruct) {
      timerTonePinDeinit();
      delete (TimerTone);
      TimerTone = NULL;
    } else {
      TimerTone->timerStop();
    }
  }
}
