#include "gd32xxyy_spl_config.h"

#if defined(SPL_WWDGT_ENABLE)
#if defined(GD32F30x)
#include "gd32f30x_wwdgt.c"
#endif
#if defined(GD32F1x0)
#include "gd32f1x0_wwdgt.c"
#endif
#if defined(GD32F10x)
#include "gd32f10x_wwdgt.c"
#endif
#if defined(GD32E23x)
#include "gd32e23x_wwdgt.c"
#endif
#if defined(GD32E50X)
#include "gd32e50x_wwdgt.c"
#endif
#endif
