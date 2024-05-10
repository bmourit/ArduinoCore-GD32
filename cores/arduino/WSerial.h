#ifndef WIRING_SERIAL_H
#define WIRING_SERIAL_H

#include "variant.h"
#include "HardwareSerial.h"

#if defined(USBCON) && defined(USBD_USE_CDC)
  #ifndef DISABLE_GENERIC_SERIALUSB
    #define ENABLE_SERIALUSB
    #if !defined(Serial)
      #define Serial SerialUSB
      #define serialEvent serialEventUSB
    #endif
  #endif

  #if defined(ENABLE_SERIALUSB)
    #define HAVE_SERIALUSB
  #endif

  extern void serialEventUSB(void) __attribute__((weak));
#endif /* USBCON && USBD_USE_CDC */

#ifndef DEFAULT_HWSERIAL_INSTANCE 
  #define DEFAULT_HWSERIAL_INSTANCE 1
#endif

#if defined(DEFAULT_HWSERIAL_INSTANCE)
  #if DEFAULT_HWSERIAL_INSTANCE == 1
    #define ENABLE_HWSERIAL1
    #if !defined(Serial)
      #define Serial Serial1
      #define serialEvent serialEvent1
    #endif
  #elif DEFAULT_HWSERIAL_INSTANCE == 2
    #define ENABLE_HWSERIAL2
    #if !defined(Serial)
      #define Serial Serial2
      #define serialEvent serialEvent2
    #endif
  #elif DEFAULT_HWSERIAL_INSTANCE == 3
    #define ENABLE_HWSERIAL3
    #if !defined(Serial)
      #define Serial Serial3
      #define serialEvent serialEvent3
    #endif
  #elif DEFAULT_HWSERIAL_INSTANCE == 4
    #define ENABLE_HWSERIAL4
    #if !defined(Serial)
      #define Serial Serial4
      #define serialEvent serialEvent4
    #endif
  #elif DEFAULT_HWSERIAL_INSTANCE == 5
    #define ENABLE_HWSERIAL5
    #if !defined(Serial)
      #define Serial Serial5
      #define serialEvent serialEvent5
    #endif
    #if !defined(Serial)
      #warning "No generic 'Serial' defined!"
    #endif
  #endif /* DEFAULT_HWSERIAL_INSTANCE == x */
#endif /* DEFAULT_HWSERIAL_INSTANCE */

#if defined(ENABLE_HWSERIAL1)
  #if defined(USART0)
    #define HAVE_HWSERIAL1
  #endif
#endif
#if defined(ENABLE_HWSERIAL2)
  #if defined(USART1)
    #define HAVE_HWSERIAL2
  #endif
#endif
#if defined(ENABLE_HWSERIAL3)
  #if defined(USART2)
    #define HAVE_HWSERIAL3
  #endif
#endif
#if defined(ENABLE_HWSERIAL4)
  #if defined(USART3) || defined(UART3)
    #define HAVE_HWSERIAL4
  #endif
#endif
#if defined(ENABLE_HWSERIAL5)
  #if defined(USART4) || defined(UART4)
    #define HAVE_HWSERIAL5
  #endif
#endif

extern void serialEvent1(void) __attribute__((weak));
extern void serialEvent2(void) __attribute__((weak));
extern void serialEvent3(void) __attribute__((weak));
extern void serialEvent4(void) __attribute__((weak));
extern void serialEvent5(void) __attribute__((weak));

extern void serialEventRun(void);

#endif /* WIRING_SERIAL_H */
