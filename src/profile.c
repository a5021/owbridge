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

volatile uint32_t prof_cmd_reset_ms = 0u;
volatile uint32_t prof_cmd_search_ms = 0u;
volatile uint32_t prof_cmd_conv_ms = 0u;
volatile uint32_t prof_cmd_read_ms = 0u;
volatile uint32_t prof_cmd_done = 0u;

static uint32_t t_loop;
static uint32_t t_i2c;
static uint32_t t_bridge;

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
    t_bridge = now_us();
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
