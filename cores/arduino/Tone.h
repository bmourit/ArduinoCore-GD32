
#ifndef _WIRING_TONE
#define _WIRING_TONE

#ifdef __cplusplus

extern void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);

extern void noTone(uint8_t _pin, bool destruct = false);

#endif

#endif /* _WIRING_TONE */
