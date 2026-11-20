/**
 * @file cache.h
 * @brief ROM device cache (filled by the 1-Wire search)
 */

#ifndef BR_CACHE_H
#define BR_CACHE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The ROM cache and the library's device table are sized by the same build
   option (CMake OW_MAX_DEVICES, passed to both). 32 entries is the practical
   ceiling: the bridge holds 2 temperature bytes per DS18B20 and AT+DEVS
   prints up to 20 ROMs per response batch. */
#ifndef OW_MAX_DEVICES
#define OW_MAX_DEVICES 16u
#endif
#define BR_CACHE_MAX OW_MAX_DEVICES

#if (BR_CACHE_MAX < 1) || (BR_CACHE_MAX > 32)
#error "BR_CACHE_MAX (OW_MAX_DEVICES) must be between 1 and 32"
#endif

/** @brief Clear the cache */
void cache_clear(void);

/** @brief Number of cached devices */
uint8_t cache_count(void);

/** @brief Get a cached ROM (8 bytes, LSB first), or NULL when out of range */
const uint8_t *cache_get(uint8_t index);

/**
 * @brief Start of the dense ROM array (8 bytes per entry)
 *
 * Entries are contiguous, so a range of ROMs can be copied with a single
 * fast_copy() instead of one cache_get() call per device.
 */
const uint8_t *cache_roms(void);

/** @brief Add a ROM (silently ignored when the cache is full) */
void cache_add(const uint8_t *rom);

/** @brief Whether the ROM is already cached */
uint8_t cache_has(const uint8_t *rom);

/** @brief Find a cached ROM index by family code (0xFF when not found) */
uint8_t cache_find_family(uint8_t family);

/** @brief Whether the last search found more devices than the cache holds */
uint8_t cache_truncated(void);

/**
 * @brief Mark the last search as truncated
 *
 * The search probes one device past the requested limit, so a set flag means
 * at least one device was seen but not stored. cache_clear() resets it.
 */
void cache_mark_truncated(void);

#ifdef __cplusplus
}
#endif

#endif /* BR_CACHE_H */
