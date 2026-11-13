/**
 * @file cache.c
 * @brief ROM device cache - see cache.h
 */

#include "cache.h"

#include "fast_copy.h"


static uint8_t cache_rom[BR_CACHE_MAX][8] __attribute__((aligned(4)));
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

const uint8_t *cache_roms(void)
{
    return &cache_rom[0][0];
}

void cache_add(const uint8_t *rom)
{
    if (rom == 0 || cache_count_ >= BR_CACHE_MAX) {
        return;
    }
    if (cache_has(rom)) {
        return;
    }
    fast_copy(cache_rom[cache_count_], rom, 8u);
    cache_count_++;
}

uint8_t cache_has(const uint8_t *rom)
{
    if (rom == 0) {
        return 0u;
    }
    /* The search sink runs inside a poll that must stay short: an 8-byte
       memcmp() call per cached entry cost about 10 us on the F030 with nine
       devices. The cached rows are word-aligned, so compare two words; the
       incoming pointer is not assumed to be aligned and is packed once. */
    uint32_t key0 = (uint32_t)rom[0] | ((uint32_t)rom[1] << 8) |
                    ((uint32_t)rom[2] << 16) | ((uint32_t)rom[3] << 24);
    uint32_t key1 = (uint32_t)rom[4] | ((uint32_t)rom[5] << 8) |
                    ((uint32_t)rom[6] << 16) | ((uint32_t)rom[7] << 24);
    for (uint8_t i = 0u; i < cache_count_; i++) {
        const uint32_t *have = (const uint32_t *)(const void *)cache_rom[i];
        if (have[0] == key0 && have[1] == key1) {
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
