/**
 * @file profile.c
 * @brief One-off I2C polling profiler - see profile.h
 */

#include "profile.h"

#if defined(OWBRIDGE_PROFILE)

#include "board.h"
#include "bridge.h"
#include "stm32f0xx.h"

volatile uint32_t prof_loop_min = 0xFFFFFFFFu;
volatile uint32_t prof_loop_max = 0u;
volatile uint32_t prof_loop_sum = 0u;
volatile uint32_t prof_loop_count = 0u;

volatile uint32_t prof_i2c_min = 0xFFFFFFFFu;
volatile uint32_t prof_i2c_max = 0u;
volatile uint32_t prof_i2c_sum = 0u;
volatile uint32_t prof_i2c_count = 0u;

volatile uint32_t prof_bridge_min = 0xFFFFFFFFu;
volatile uint32_t prof_bridge_max = 0u;
volatile uint32_t prof_bridge_sum = 0u;
volatile uint32_t prof_bridge_count = 0u;
volatile uint32_t prof_bridge_last = 0u;
volatile uint32_t prof_bridge_hist[8];

/* Longest interval between two consecutive bridge_poll() calls, in us. It
   bounds how long the 1-Wire state machine may go unserviced; values up to
   ~131 ms are exact (the 16-bit timer may wrap once). */
volatile uint32_t prof_gap_max = 0u;

/* A "missed start" is an I2C START that arrived while bridge_poll() was
   running: the bus went from idle to busy across one poll. */
volatile uint32_t prof_start_missed = 0u;
volatile uint32_t prof_start_missed_max = 0u;

volatile uint32_t prof_submit_max = 0u;
volatile uint32_t prof_work_max = 0u;
volatile uint32_t prof_sink_max = 0u;
volatile uint32_t prof_lib_max = 0u;
volatile uint32_t prof_config_set_us = 0u;

volatile uint32_t prof_cmd_reset_ms = 0u;
volatile uint32_t prof_cmd_search_ms = 0u;
volatile uint32_t prof_cmd_conv_ms = 0u;
volatile uint32_t prof_cmd_read_ms = 0u;
volatile uint32_t prof_cmd_done = 0u;

static uint32_t t_loop;
static uint32_t t_i2c;
static uint32_t t_bridge;
static uint32_t t_submit;
static uint32_t t_work;
static uint8_t t_phase;
static uint32_t t_sink;
static uint32_t t_lib;
static uint32_t t_config;

/* Gap tracking between consecutive bridge_poll() calls. */
static uint32_t t_prev_bridge;
static uint8_t have_prev_bridge;

static inline uint32_t now_us(void)
{
    return TIM3->CNT;
}

/* 16-bit timer: handle the wrap between the start and end of a measurement. */
static inline uint32_t dt_us(uint32_t start)
{
    uint32_t now = now_us();
    return (now >= start) ? (now - start) : (0x10000u - start + now);
}

void profile_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = (uint32_t)(OW_SYSCLK_MHZ) - 1u; /* 1 MHz */
    TIM3->ARR = 0xFFFFu;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0u;
    TIM3->CR1 = TIM_CR1_CEN;
}

void profile_loop_begin(void)
{
    t_loop = now_us();
}

void profile_loop_end(void)
{
    uint32_t dt = dt_us(t_loop);
    if (dt < prof_loop_min) {
        prof_loop_min = dt;
    }
    if (dt > prof_loop_max) {
        prof_loop_max = dt;
    }
    prof_loop_sum += dt;
    prof_loop_count++;
}

void profile_i2c_begin(void)
{
    t_i2c = now_us();
}

void profile_i2c_end(void)
{
    uint32_t dt = dt_us(t_i2c);
    if (dt < prof_i2c_min) {
        prof_i2c_min = dt;
    }
    if (dt > prof_i2c_max) {
        prof_i2c_max = dt;
    }
    prof_i2c_sum += dt;
    prof_i2c_count++;
}

void profile_bridge_begin(void)
{
    uint32_t now = now_us();
    if (have_prev_bridge) {
        uint32_t gap = dt_us(t_prev_bridge);
        if (gap > prof_gap_max) {
            prof_gap_max = gap;
        }
    }
    t_bridge = now;
}

void profile_bridge_end(void)
{
    uint32_t dt = dt_us(t_bridge);
    if (dt < prof_bridge_min) {
        prof_bridge_min = dt;
    }
    if (dt > prof_bridge_max) {
        prof_bridge_max = dt;
    }
    prof_bridge_sum += dt;
    prof_bridge_count++;
    prof_bridge_last = dt;

    /* Log2 histogram: bucket 0 is 0-1 us, then 2-3, 4-7, ... 64-127, >=128. */
    uint32_t idx = 0u;
    uint32_t v = dt;
    while (v > 1u && idx < 7u) {
        v >>= 1;
        idx++;
    }
    prof_bridge_hist[idx]++;

    t_prev_bridge = now_us();
    have_prev_bridge = 1u;
}

void profile_start_check(uint8_t idle_before, uint8_t busy_after)
{
    if (idle_before && busy_after) {
        prof_start_missed++;
        if (prof_bridge_last > prof_start_missed_max) {
            prof_start_missed_max = prof_bridge_last;
        }
    }
}

void profile_submit_begin(void)
{
    t_submit = now_us();
}

void profile_submit_end(void)
{
    uint32_t dt = dt_us(t_submit);
    if (dt > prof_submit_max) {
        prof_submit_max = dt;
    }
}

void profile_config_set_begin(void)
{
    t_config = now_us();
}

void profile_config_set_end(void)
{
    prof_config_set_us = dt_us(t_config);
}

static void run_cmd(const uint8_t *frame, uint8_t len, volatile uint32_t *out_ms)
{
    uint32_t t0 = board_millis();
    bridge_submit(frame, len);
    while (bridge_response_ptr()[0] == BR_BUSY && (board_millis() - t0) < 5000u) {
        board_watchdog_refresh();
        bridge_poll();
    }
    *out_ms = board_millis() - t0;
}

/* Library work-event hooks (OW_PROFILE): the driver calls these around the
   branch that actually advances its state machine, excluding the application
   sink. The firmware's search sink is timed separately, so a long poll can be
   attributed to the driver or to the callback. */
void ow_profile_work_begin(void)
{
    t_work = now_us();
}

void ow_profile_phase(uint8_t phase)
{
    t_phase = phase;
}

void ow_profile_work_end(void)
{
    uint32_t d = dt_us(t_work);
    if (d > prof_work_max) {
        prof_work_max = d;
    }
}

void profile_lib_begin(void)
{
    t_lib = now_us();
}

void profile_lib_end(void)
{
    uint32_t d = dt_us(t_lib);
    if (d > prof_lib_max) {
        prof_lib_max = d;
    }
}

void profile_sink_begin(void)
{
    t_sink = now_us();
}

void profile_sink_end(void)
{
    uint32_t d = dt_us(t_sink);
    if (d > prof_sink_max) {
        prof_sink_max = d;
    }
}

void profile_experiment(void)
{
    static const uint8_t f_reset[] = { BR_CMD_OW_RESET, 0u };
    static const uint8_t f_search[] = { BR_CMD_OW_SEARCH, 3u, 0xF0u, 0x00u, 16u };
    static const uint8_t f_conv[] = { BR_CMD_TEMP_CONVERT, 0u };
    static const uint8_t f_read[] = { BR_CMD_TEMP_READ_ALL, 0u };

    prof_cmd_done = 1u;
    run_cmd(f_reset, (uint8_t)sizeof(f_reset), &prof_cmd_reset_ms);
    run_cmd(f_search, (uint8_t)sizeof(f_search), &prof_cmd_search_ms);
    run_cmd(f_conv, (uint8_t)sizeof(f_conv), &prof_cmd_conv_ms);
    run_cmd(f_read, (uint8_t)sizeof(f_read), &prof_cmd_read_ms);
}

#endif /* OWBRIDGE_PROFILE */
