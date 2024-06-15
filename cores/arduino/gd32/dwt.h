
#ifndef DWT_H_
#define DWT_H_

#include <stdbool.h>

#include "gd32_def.h"

#ifdef DWT_BASE

#ifdef __cplusplus
extern "C" {
#endif

uint32_t ARM_DWT_init(void);
void ARM_DWT_enable(bool enable);

static inline uint32_t ARM_DWT_s_max(void)
{
  return (UINT32_MAX / SystemCoreClock);
};

static inline uint32_t ARM_DWT_ms_max(void)
{
  return (UINT32_MAX / (SystemCoreClock / 1000));
};

static inline uint32_t ATM_DWT_us_max(void)
{
  return (UINT32_MAX / (SystemCoreClock / 1000000));
};

static inline uint32_t ARM_DWT_cycle_count_get(void)
{
  return (DWT->CYCCNT);
};

#ifdef __cplusplus
}
#endif

#endif /* DWT_BASE */
#endif /* _DWT_H_ */
