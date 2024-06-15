#include "gd32xxyy_spl_config.h"

#if defined(SPL_TSI_ENABLE)
#if defined(GD32F3x0)
#include "gd32f3x0_tsi.c"
#endif
#if defined(GD32F1x0)
#include "gd32f1x0_tsi.c"
#endif
#endif
