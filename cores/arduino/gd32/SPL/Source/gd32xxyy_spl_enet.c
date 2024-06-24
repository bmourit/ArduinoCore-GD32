#include "gd32xxyy_spl_config.h"

#if defined(SPL_ENET_ENABLE)
#if defined(GD32F30x)
#include "gd32f30x_enet.c"
#endif
#if defined(GD32F10x)
#include "gd32f10x_enet.c"
#endif
#if defined(GD32E50X_CL) || defined(GD32E508) || defined(GD32EPRT)
#include "gd32e50x_enet.c"
#endif
#endif
