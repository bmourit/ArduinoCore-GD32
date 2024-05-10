#ifndef GD_DEBUG_H_
#define GD_DEBUG_H_

/* standard setting: swallow fatal erros for firmware size reasons */
#ifndef GD_DEBUG
#define GD_DEBUG  0
#endif

#ifdef GD_DEBUG
#include <stdio.h>
#include <stdarg.h>
#endif

#include <variant.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline void gd_debug(const char *format, ...)
{
#if GD_DEBUG != 0
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

#endif /* GD_DEBUG_H_ */
