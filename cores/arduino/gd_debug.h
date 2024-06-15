#ifndef GD_DEBUG_H
#define GD_DEBUG_H

/* standard setting: swallow fatal erros for firmware size reasons */
//#ifndef GD_DEBUG
//#define GD_DEBUG  0
//#endif

#ifdef GD_DEBUG
#include <stdio.h>
#include <stdarg.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

static inline void gd_debug(const char *format, ...)
{
#ifdef GD_DEBUG
	va_list arglist;
	va_start(arglist, format);
	vprintf(format, arglist);
	va_end(arglist);
#else
  (void)(format);
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* GD_DEBUG_H */
