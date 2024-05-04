
#include "bootloader.h"
#include "gd32_def.h"
#include "backup_domain.h"

#ifdef BL_LEGACY_LEAF
void toggle_dtr_hook(uint8_t *buf, uint32_t *length)
{
	/**
	 * Four byte is the magic pack "1EAF" that puts the MCU into bootloader.
	 * Check if the incoming contains the string "1EAF".
	 * If yes, put the MCU into the bootloader mode.
	 */
  if ((buf[0] == '1') && (buf[1] == 'E') && (buf[2] == 'A') && (buf[3] == 'F') && (*length >= 4)) {
    NVIC_SystemReset();
  }
}
#endif /* BL_LEGACY_LEAF */

#ifdef BL_HID
void toggle_dtr_hook(uint8_t *buf, uint32_t *length)
{
  /**
   * Four byte is the magic pack "1EAF" that puts the MCU into bootloader.
   * Check if the incoming contains the string "1EAF".
   * If yes, put the MCU into the bootloader mode.
   */
  if ((buf[0] == '1') && (buf[1] == 'E') && (buf[2] == 'A') && (buf[3] == 'F') && (*length >= 4)) {
    backup_domain_enable();
    /* New HID Bootloader (ver 2.2+) */
    backup_register_set(HID_MAGIC_BACKUP_INDEX, HID_MAGIC_BACKUP_VALUE);
#ifdef HID_OLD_MAGIC_BACKUP_INDEX
    /* Compatibility to the old HID Bootloader (ver <= 2.1) */
    backup_register_set(HID_OLD_MAGIC_BACKUP_INDEX, HID_MAGIC_BACKUP_VALUE);
#endif
    NVIC_SystemReset();
  }
}
#endif /* BL_HID */