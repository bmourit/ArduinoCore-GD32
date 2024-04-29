#include "gd32_def.h"
#include "gd_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
WEAK void _Error_Handler(const char *msg, int val)
{
  /**
   * User can add their own implementation to report the SPL error return state
   * by replacing the default gd_debug
   */
  gd_debug("Error: %s (%i)\n", msg, val);
  while (1) {
  }
}

#ifdef __cplusplus
}
#endif
