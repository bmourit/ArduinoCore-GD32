#include "gd32xxyy_spl_config.h"

#if defined(SPL_RCU_ENABLE)
#if defined(GD32F30x)
#include "gd32f30x_rcu.c"
#endif
#if defined(GD32F3x0)
#include "gd32f3x0_rcu.c"
#endif
#if defined(GD32F1x0)
#include "gd32f1x0_rcu.c"
#endif
#if defined(GD32F10x)
#include "gd32f10x_rcu.c"
#endif
#if defined(GD32E23x)
#include "gd32e23x_rcu.c"
#endif
#if defined(GD32E50X)
#include "gd32e50x_rcu.c"
#endif
#endif
