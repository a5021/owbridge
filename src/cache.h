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

#define BR_CACHE_MAX 16u

/** @brief Clear the cache */
void cache_clear(void);

/** @brief Number of cached devices */
uint8_t cache_count(void);

/** @brief Get a cached ROM (8 bytes, LSB first), or NULL when out of range */
const uint8_t *cache_get(uint8_t index);

/** @brief Add a ROM (silently ignored when the cache is full) */
void cache_add(const uint8_t *rom);

/** @brief Whether the ROM is already cached */
uint8_t cache_has(const uint8_t *rom);

/** @brief Find a cached ROM index by family code (0xFF when not found) */
uint8_t cache_find_family(uint8_t family);

#ifdef __cplusplus
}
#endif

#endif /* BR_CACHE_H */
