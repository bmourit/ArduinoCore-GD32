#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Core and peripherals registers definitions
*/
#include "api/Interrupts.h"
#include "analog.h"
#include "backup_domain.h"
#include "systick.h"
#include "hw_config.h"
#include "rtc.h"
#include "timer.h"
#include "uart.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
