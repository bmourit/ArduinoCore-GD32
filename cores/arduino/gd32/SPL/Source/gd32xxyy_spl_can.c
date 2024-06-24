#include "gd32xxyy_spl_config.h"

#if defined(SPL_CAN_ENABLE)
	#if defined(GD32F30x)
		#include "gd32f30x_can.c"
	#endif
	#if defined(GD32F10x)
		#include "gd32f10x_can.c"
	#endif
	#if defined(GD32F1x0)
	#if defined(GD32F170_190)
		#include "gd32f1x0_can.c"
	#endif
	#endif
	#if defined(GD32E50X)
	#if !defined(GD32EPRT)
		#include "gd32e50x_can.c"
	#endif
	#endif
#endif