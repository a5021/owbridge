/**
 * @file profile.h
 * @brief One-off I2C polling profiler (TIM3, 1 MHz), results in RAM for SWD
 *
 * Enabled with -DOWBRIDGE_PROFILE=1. When disabled every call compiles away,
 * so production builds pay nothing. The experiment measures:
 *   - the full main-loop period (prof_loop_*),
 *   - the host_i2c_soft_poll() duration (prof_i2c_*),
 *   - the bridge_poll() duration (prof_bridge_*),
 *   - the wall-clock time of four commands (prof_cmd_*_ms).
 *
 * The busy-loop period (priority scheduling, bridge poll skipped) is
 * approximately prof_loop_* minus prof_bridge_*. The measured results are
 * documented in docs/i2c-timing.md.
 */

#ifndef PROFILE_H
#define PROFILE_H

#include <stdint.h>

#if defined(OWBRIDGE_PROFILE)

extern volatile uint32_t prof_loop_min;
extern volatile uint32_t prof_loop_max;
extern volatile uint32_t prof_loop_sum;
extern volatile uint32_t prof_loop_count;

extern volatile uint32_t prof_i2c_min;
extern volatile uint32_t prof_i2c_max;
extern volatile uint32_t prof_i2c_sum;
extern volatile uint32_t prof_i2c_count;

extern volatile uint32_t prof_bridge_min;
extern volatile uint32_t prof_bridge_max;
extern volatile uint32_t prof_bridge_sum;
extern volatile uint32_t prof_bridge_count;

extern volatile uint32_t prof_cmd_reset_ms;
extern volatile uint32_t prof_cmd_search_ms;
extern volatile uint32_t prof_cmd_conv_ms;
extern volatile uint32_t prof_cmd_read_ms;
extern volatile uint32_t prof_cmd_done;

void profile_init(void);
void profile_loop_begin(void);
void profile_loop_end(void);
void profile_i2c_begin(void);
void profile_i2c_end(void);
void profile_bridge_begin(void);
void profile_bridge_end(void);
void profile_experiment(void);

#else

static inline void profile_init(void) {}
static inline void profile_loop_begin(void) {}
static inline void profile_loop_end(void) {}
static inline void profile_i2c_begin(void) {}
static inline void profile_i2c_end(void) {}
static inline void profile_bridge_begin(void) {}
static inline void profile_bridge_end(void) {}
static inline void profile_experiment(void) {}

#endif

#endif /* PROFILE_H */
