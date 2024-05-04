#ifndef _GD32YYXX_EXTI_H_
#define _GD32YYXX_EXTI_H_
/* SPL raised several warnings, ignore them */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#ifdef GD32F30x
  #include "gd32f30x_exti.h"
#endif
#ifdef GD32F1x0
  #include "gd32f1x0_exti.h"
#endif
#ifdef GD32F3x0
  #include "gd32f3x0_exti.h"
#endif
#ifdef GD32E50x
  #include "gd32e50x_exti.h"
#endif
#ifdef GD32E23x
  #include "gd32e23x_exti.h"
#endif

#pragma GCC diagnostic pop
#endif /* _GD32YYXX_EXTI_H_ */