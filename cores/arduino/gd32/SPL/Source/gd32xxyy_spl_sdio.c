#include "gd32xxyy_spl_config.h"

#if defined(SPL_SDIO_ENABLE)
#if defined(GD32F30x)
#include "gd32f30x_sdio.c"
#endif
#if defined(GD32F10x)
#include "gd32f10x_sdio.c"
#endif
#if defined(GD32E50X)
#if !defined(GD32EPRT) && !defined(GD32E50X_CL) && !defined(GD32E508)
#include "gd32e50x_sdio.c"
#endif
#endif
#endif