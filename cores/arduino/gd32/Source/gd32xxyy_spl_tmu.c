#include "gd32xxyy_spl_config.h"

#if defined(SPL_TMU_ENABLE)
#if defined(GD32E50X_CL) || defined(GD32E508)
#include "gd32e50x_tmu.c"
#endif
#endif
