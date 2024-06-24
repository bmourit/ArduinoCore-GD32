#include "gd32xxyy_spl_config.h"

#if defined(SPL_DAC_ENABLE)
	#if defined(GD32F30x)
		#include "gd32f30x_dac.c"
	#endif
	#if defined(GD32F350)
		#include "gd32f3x0_dac.c"
	#endif
	#if defined(GD32F1x0)
		#include "gd32f1x0_dac.c"
	#endif
	#if defined(GD32F10x)
		#include "gd32f10x_dac.c"
	#endif
	#if defined(GD32E50X)
		#include "gd32e50x_dac.c"
	#endif
#endif
