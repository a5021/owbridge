/**
 * @file stats.c
 * @brief Bridge diagnostics counters - see stats.h
 */

#include "stats.h"
#include "board.h"

static uint32_t stats[BR_STAT_COUNT];

void stats_init(void)
{
    stats_reset();
}

void stats_count(uint8_t kind)
{
    if (kind < BR_STAT_COUNT) {
        stats[kind]++;
    }
}

void stats_reset(void)
{
    for (uint8_t i = 0u; i < BR_STAT_COUNT; i++) {
        stats[i] = 0u;
    }
}

uint32_t stats_get(uint8_t kind)
{
    return (kind < BR_STAT_COUNT) ? stats[kind] : 0u;
}

uint32_t stats_uptime(void)
{
    return board_millis() / 1000u;
}
