#include "gd32xxyy_spl_config.h"

#if defined(SPL_SYSCFG_ENABLE)
#if defined(GD32F3x0)
#include "gd32f3x0_syscfg.c"
#endif
#if defined(GD32E23x)
#include "gd32e23x_syscfg.c"
#endif
#endif
