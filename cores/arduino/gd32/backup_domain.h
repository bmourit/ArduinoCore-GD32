/**
  ******************************************************************************
  * @file    backup.h
  * @author  bmourit
  * @brief   Header for backup domain driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 GD32CommunityCores
  * All rights reserved.
  *
  * This software component is licensed under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#ifndef _BACKUP_H_
#define _BACKUP_H_

#include "gd32_def.h"
#include "gd32xxyy.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(PMU_CTL_BKPWEN) || defined(PMU_CTL0_BKPWEN)
#define HAVE_PMU_BACKUP_DOMAIN
#endif

#if !defined(RTC_BACKUP_INDEX) && defined(HAVE_PMU_BACKUP_DOMAIN)
#define RTC_BACKUP_INDEX	BKP_DATA_0
#else
#define RTC_BACKUP_INDEX	0
#endif
#ifndef RTC_BACKUP_VALUE
#define RTC_BACKUP_VALUE	0x32F2
#endif

#if defined(BL_HID)
#if !defined(HID_MAGIC_BACKUP_INDEX)
#define HID_MAGIC_BACKUP_INDEX		BKP_DATA_3
#endif
#if !defined(HID_MAGIC_BACKUP_INDEX) && defined(BKP_DATA_9)
#define HID_OLD_MAGIC_BACKUP_INDEX	BKP_DATA_9
#endif
#ifndef HID_MAGIC_BACKUP_VALUE
#define HID_MAGIC_BACKUP_VALUE		0x424C
#endif
#endif

#if defined(KILL_RTC_BACKUP_DOMAIN_ON_RESTART)
static inline void backup_domain_kill(void)
{
#if defined(GD32F30x) || defined(GD32E50X)
  bkp_deinit();
#elif defined(GD32F3x0) || defined(GD32F1x0)
  rcu_bkp_reset_enable();
  rcu_bkp_reset_disable();
#endif
}
#endif

static inline void backup_domain_reset(void)
{
#if defined(HAVE_PMU_BACKUP_DOMAIN)
	pmu_backup_write_enable();
	/**
   *  Write twice the value to flush the APB-AHB bridge
   *  This bit must be written in the register before writing the next one
   */
  pmu_backup_write_enable();
#endif
  rcu_bkp_reset_enable();
  rcu_bkp_reset_disable();
}

static inline void backup_domain_enable(void)
{
#if defined(HAVE_PMU_BACKUP_DOMAIN)
	rcu_periph_clock_enable(RCU_PMU);
	pmu_backup_write_enable();
#endif
#if defined(RCU_BKPI)
	rcu_periph_clock_enable(RCU_BKPI);
#endif
}

static inline void backup_domain_disable(void)
{
#if defined(HAVE_PMU_BACKUP_DOMAIN)
	pmu_backup_write_disable();
#endif
#if defined(RCU_BKPI)
	rcu_periph_clock_disable(RCU_BKPI);
#endif
}

static inline void backup_register_set(bkp_data_register_enum index, uint16_t value)
{
#if defined(HAVE_PMU_BACKUP_DOMAIN)
	bkp_write_data(index, value);
#endif
}

static inline uint16_t backup_rergister_get(bkp_data_register_enum index)
{
#if defined(HAVE_PMU_BACKUP_DOMAIN)
	return bkp_read_data(index);
#else
	return 0;
#endif
}

#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif /* _BACKUP_H_ */
