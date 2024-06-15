#include "gd32xxyy_spl_config.h"

#if defined(SPL_CMP_ENABLE)
#if defined(GD32F350)
#include "gd32f3x0_cmp.c"
#endif
#if defined(GD32F1x0)
#include "gd32f1x0_cmp.c"
#endif
#if defined(GD32E23x)
#include "gd32e23x_cmp.c"
#endif
#if defined(GD32E50X_CL) || defined(GD32E508)
#include "gd32e50x_cmp.c"
#endif
#endif
