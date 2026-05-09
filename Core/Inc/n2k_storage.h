/**
 * n2k_storage.h
 *
 * Persistence of NMEA 2000 negotiated source address (and a spare instance
 * slot, kept layout-compatible with the SDolve Water/Fuel-Level node) in
 * the last flash page of the STM32F103 (0x0801FC00..0x0801FFFF, 1 KB).
 *
 * The linker script reserves this page by limiting FLASH to 127 KB.
 *
 * Storage format (32-bit words, little-endian):
 *   word[0] = magic  (0x4E324B53 = "N2KS")
 *   word[1] = packed source address  - low byte = addr, next byte = ~addr
 *   word[2] = packed instance        - low byte = inst, next byte = ~inst
 *
 * If the page is blank (0xFFFFFFFF) or the magic mismatches, Load functions
 * return false and callers fall back to their defaults.
 */
#ifndef N2K_STORAGE_H
#define N2K_STORAGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Read the persisted N2K source address.
 * @param  out_addr  receives the stored address on success
 * @return true if a valid stored address was found
 */
bool N2kStorage_LoadSource(uint8_t *out_addr);

/**
 * Persist the given source address.
 * Erases and rewrites the flash page only when the value changes.
 * @return true on success (or no-op when value already matches)
 */
bool N2kStorage_SaveSource(uint8_t addr);

/**
 * Read the persisted instance number (reserved for future use on this node).
 * @param  out_instance  receives the stored instance on success
 * @return true if a valid stored instance was found
 */
bool N2kStorage_LoadInstance(uint8_t *out_instance);

/**
 * Persist the given instance number (reserved for future use on this node).
 * Erases and rewrites the flash page only when the value changes.
 * @return true on success (or no-op when value already matches)
 */
bool N2kStorage_SaveInstance(uint8_t instance);

#ifdef __cplusplus
}
#endif

#endif /* N2K_STORAGE_H */
