#ifndef _GD32XXYY_SPL_CAN_H_
#define _GD32XXYY_SPL_CAN_H_

#if defined(GD32F30x)
	#include "gd32f30x_can.h"
#endif
#if defined(GD32F10x)
	#include "gd32f10x_can.h"
#endif
#if defined(GD32F1x0)
#if defined(GD32F170_190)
	#include "gd32f1x0_can.h"
#endif
#endif
#if defined(GD32E50X)
#if !defined(GD32EPRT)
	#include "gd32e50x_can.h"
#endif
#endif

#endif /* _GD32XXYY_SPL_CAN_H_ */
