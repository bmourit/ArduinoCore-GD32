#include "gd32xxyy_spl_config.h"

#if defined(SPL_CTC_ENABLE)
	#if defined(GD32F30x)
		#include "gd32f30x_ctc.c"
	#endif
	#if defined(GD32F3x0)
		#include "gd32f3x0_ctc.c"
	#endif
	#if defined(GD32E50X)
		#include "gd32e50x_ctc.c"
	#endif
#endif
