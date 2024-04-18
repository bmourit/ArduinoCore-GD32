
#include <stdarg.h>
#include <stdio.h>
#include <variant.h>
#include <gd_debug.h>

/* standard setting: swallow fatal erros for firmware size reasons */
#ifndef GD_DEBUG
#define GD_DEBUG  0
#endif

void gd_debug(const char *format, ...)
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
