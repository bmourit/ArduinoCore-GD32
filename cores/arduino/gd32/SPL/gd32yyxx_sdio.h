#ifndef _GD32YYXX_SDIO_H_
#define _GD32YYXX_SDIO_H_
/* SPL raised several warnings, ignore them */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#ifdef GD32F30x
  #include "gd32f30x_sdio.h"
#endif
#ifdef GD32E50x
  #include "gd32e50x_sdio.h"
#endif

#pragma GCC diagnostic pop
#endif /* _GD32YYXX_SDIO_H_ */