#ifndef _GD32_BUILD_DEF_
#define _GD32_BUILD_DEF_

#if !defined(CMSIS_STARTUP_FILE) && !defined(CUSTOM_STARTUP_FILE)
#if defined(GD32F30X_HD)
#define CMSIS_STARTUP_FILE "startup_gd32f30x_hd.S"
#elif defined(GD32F30X_XD)
#define CMSIS_STARTUP_FILE "startup_gd32f30x_xd.S"
#elif defined(GD32F30X_CL)
#define CMSIS_STARTUP_FILE "startup_gd32f30x_cl.S"
#elif defined(GD32F3x0)
#define CMSIS_STARTUP_FILE "startup_gd32f3x0.S"
#elif defined(GD32F1x0)
#define CMSIS_STARTUP_FILE "startup_gd32f1x0.S"
#elif defined(GD32F10X_CL)
#define CMSIS_STARTUP_FILE "startup_gd32f10x_cl.S"
#elif defined(GD32F10X_HD)
#define CMSIS_STARTUP_FILE "startup_gd32f10x_hd.S"
#elif defined(GD32F10X_MD)
#define CMSIS_STARTUP_FILE "startup_gd32f10x_md.S"
#elif defined(GD32F10X_XD)
#define CMSIS_STARTUP_FILE "startup_gd32f10x_xd.S"
#elif defined(GD32E23x)
#define CMSIS_STARTUP_FILE "startup_gd32e23x.S"
#elif defined(GD32E50X_HD)
#define CMSIS_STARTUP_FILE "startup_gd32e50x_hd.S"
#elif defined(GD32E50X_XD)
#define CMSIS_STARTUP_FILE "startup_gd32e50x_xd.S"
#elif defined(GD32E50X_CL)
#define CMSIS_STARTUP_FILE "startup_gd32e50x_cl.S"
#elif defined(GD32E508)
#define CMSIS_STARTUP_FILE "startup_gd32e508.S"
#elif defined(GD32EPRT)
#define CMSIS_STARTUP_FILE "startup_gd32eprt.S"
#else
#error "Unknown chip series!"
#endif
#else
  #warning "No CMSIS startup file defined, custom one should be used"
#endif /* !CMSIS_STARTUP_FILE && !CUSTOM_STARTUP_FILE */

#endif /* _GD32_BUILD_DEF_ */
