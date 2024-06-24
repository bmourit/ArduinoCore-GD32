#include "gd32_def.h"
#include "systick.h"

void SPL_init(void)
{
  nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
  tickInit(0x00U);
}
