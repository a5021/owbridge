/**
 * @file cache.c
 * @brief ROM device cache - see cache.h
 */

#include "cache.h"

#include <string.h>

static uint8_t cache_rom[BR_CACHE_MAX][8];
static uint8_t cache_count_;

void cache_clear(void)
{
    cache_count_ = 0u;
}

uint8_t cache_count(void)
{
    return cache_count_;
}

const uint8_t *cache_get(uint8_t index)
{
    if (index >= cache_count_) {
        return 0;
    }
    return cache_rom[index];
}

void cache_add(const uint8_t *rom)
{
    if (rom == 0 || cache_count_ >= BR_CACHE_MAX) {
        return;
    }
    if (cache_has(rom)) {
        return;
    }
    memcpy(cache_rom[cache_count_], rom, 8u);
    cache_count_++;
}

uint8_t cache_has(const uint8_t *rom)
{
    if (rom == 0) {
        return 0u;
    }
    for (uint8_t i = 0u; i < cache_count_; i++) {
        if (memcmp(cache_rom[i], rom, 8u) == 0) {
            return 1u;
        }
    }
    return 0u;
}

uint8_t cache_find_family(uint8_t family)
{
    for (uint8_t i = 0u; i < cache_count_; i++) {
        if (cache_rom[i][0] == family) {
            return i;
        }
    }
    return 0xFFu;
}
