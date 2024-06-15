#include "gd32xxyy_spl_config.h"

#if defined(SPL_BKP_ENABLE)
	#if defined(GD32F30x)
		#include "gd32f30x_bkp.c"
	#endif
	#if defined(GD32F10x)
		#include "gd32f10x_bkp.c"
	#endif
	#if defined(GD32E50X)
		#include "gd32e50x_bkp.c"
	#endif
#endif
