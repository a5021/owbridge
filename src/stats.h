/**
 * @file stats.h
 * @brief Bridge diagnostics counters
 */

#ifndef BR_STATS_H
#define BR_STATS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    BR_STAT_RESETS = 0,
    BR_STAT_PRESENCE_FAIL,
    BR_STAT_CRC_ERROR,
    BR_STAT_BUS_SHORT,
    BR_STAT_TIMEOUT,
    BR_STAT_COMMANDS,
    BR_STAT_ABORTS,
    BR_STAT_COUNT
};

void stats_init(void);
void stats_count(uint8_t kind);
void stats_reset(void);
uint32_t stats_get(uint8_t kind);
uint32_t stats_uptime(void);

#ifdef __cplusplus
}
#endif

#endif /* BR_STATS_H */
