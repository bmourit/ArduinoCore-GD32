#include "gd32xxyy_spl_config.h"

#if defined(SPL_CEC_ENABLE)
	#if defined(GD32F350)
		#include "gd32f3x0_cec.c"
	#endif
	#if defined(GD32F1x0)
		#include "gd32f1x0_cec.c"
	#endif
#endif
