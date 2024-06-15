#ifndef _GD32XXYY_SPL_SDIO_H_
#define _GD32XXYY_SPL_SDIO_H_

#if defined(GD32F30x)
#include "gd32f30x_sdio.h"
#endif
#if defined(GD32F10x)
#include "gd32f10x_sdio.h"
#endif
#if defined(GD32E50X)
#if !defined(GD32EPRT) && !defined(GD32E50X_CL) && !defined(GD32E508)
#include "gd32e50x_sdio.h"
#endif
#endif

#endif /* _GD32XXYY_SPL_SDIO_H_ */
