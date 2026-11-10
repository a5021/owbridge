/**
 * @file host_i2c_soft.c
 * @brief Software I2C slave host interface, fully polled (no interrupts)
 *
 * Pins (configured in board.c):
 *   PA4 = SCL, input with pull-up (the master drives the clock)
 *   PA5 = SDA, open-drain output with pull-up
 *
 * Protocol (same frame as UART/SPI):
 *   Master writes: [cmd][len][data...]
 *   Master reads:  [status][cmd][len][data...]
 *
 * The main loop polls the two lines, detects START/STOP and the SCL edges,
 * and runs a small state machine. No EXTI, no DMA (GPIO edges cannot trigger
 * DMA). The achievable clock is set by the main-loop latency; at ~2 us per
 * iteration 100 kHz has a comfortable margin.
 */

#include "host_i2c_soft.h"
#include "bridge.h"
#include "config.h"

#include "stm32f0xx.h"

#include <string.h>

#define SCL_PIN 4u
#define SDA_PIN 5u

#define SCL_READ() (((GPIOA->IDR) >> SCL_PIN) & 1u)
#define SDA_READ() (((GPIOA->IDR) >> SDA_PIN) & 1u)
#define SDA_LOW() (GPIOA->BSRR = (1u << (SDA_PIN + 16u)))
#define SDA_RELEASE() (GPIOA->BSRR = (1u << SDA_PIN))

#define I2C_RX_BUF 80u
#define I2C_TX_BUF (3u + BR_MAX_RESP)

typedef enum {
    I2C_IDLE = 0,
    I2C_ADDR,
    I2C_RX,
    I2C_TX
} i2c_state_t;

static i2c_state_t state;
static uint8_t my_addr;
static uint8_t bit;
static uint8_t shift;
static uint8_t ack;
static uint8_t ack_phase;
static uint8_t rw;

static uint8_t rx_buf[I2C_RX_BUF];
static uint8_t rx_idx;

static uint8_t tx_buf[I2C_TX_BUF];
static uint8_t tx_len;
static uint8_t tx_idx;
static uint8_t tx_byte;

static uint8_t last_scl;
static uint8_t last_sda;

/* --- Helpers -------------------------------------------------------------- */

static void i2c_start(void)
{
    state = I2C_ADDR;
    bit = 0u;
    shift = 0u;
    ack = 0u;
    ack_phase = 0u;
    rx_idx = 0u;
}

static void i2c_stop(void)
{
    if (state == I2C_RX && rx_idx >= 2u) {
        bridge_submit(rx_buf, rx_idx);
    }
    state = I2C_IDLE;
    SDA_RELEASE();
    bit = 0u;
    shift = 0u;
    ack_phase = 0u;
}

static void i2c_scl_rising(uint8_t sda)
{
    switch (state) {
    case I2C_ADDR:
    case I2C_RX:
        if (bit < 8u) {
            shift = (uint8_t)((shift << 1) | sda);
            bit++;
            if (bit == 8u) {
                if (state == I2C_ADDR) {
                    uint8_t addr = (uint8_t)(shift >> 1);
                    rw = (uint8_t)(shift & 1u);
                    ack = (addr == my_addr) ? 1u : 0u;
                } else {
                    if (rx_idx < I2C_RX_BUF) {
                        rx_buf[rx_idx++] = shift;
                    }
                    ack = 1u;
                }
            }
        }
        break;
    case I2C_TX:
        if (bit < 8u) {
            bit++;
        } else {
            /* ACK clock: sample the master's ACK (low = ACK). */
            if (sda) {
                state = I2C_IDLE; /* NACK */
            } else {
                if (tx_idx < tx_len) {
                    tx_byte = tx_buf[tx_idx++];
                } else {
                    tx_byte = 0xFFu;
                }
                bit = 0u;
            }
        }
        break;
    default:
        break;
    }
}

static void i2c_scl_falling(void)
{
    switch (state) {
    case I2C_ADDR:
    case I2C_RX:
        if (bit < 8u) {
            /* Receiving: nothing to drive. */
        } else if (!ack_phase) {
            /* Start of the ACK bit: drive ACK or NACK. */
            if (ack) {
                SDA_LOW();
            } else {
                SDA_RELEASE();
            }
            ack_phase = 1u;
        } else {
            /* End of the ACK bit: release and move on. */
            SDA_RELEASE();
            ack_phase = 0u;
            if (state == I2C_ADDR) {
                if (!ack) {
                    state = I2C_IDLE;
                } else if (rw) {
                    tx_len = bridge_get_response(tx_buf);
                    if (tx_len == 0u) {
                        tx_len = 3u;
                    }
                    tx_idx = 0u;
                    tx_byte = tx_buf[tx_idx++];
                    state = I2C_TX;
                } else {
                    state = I2C_RX;
                }
            } else {
                state = I2C_RX;
            }
            bit = 0u;
        }
        break;
    case I2C_TX:
        if (bit < 8u) {
            if ((tx_byte >> (7u - bit)) & 1u) {
                SDA_RELEASE();
            } else {
                SDA_LOW();
            }
        } else {
            /* ACK bit: release SDA for the master. */
            SDA_RELEASE();
        }
        break;
    default:
        break;
    }
}

/* --- Public --------------------------------------------------------------- */

void host_i2c_soft_init(void)
{
    my_addr = config_get()->i2c_addr;
    if (my_addr < 0x08u || my_addr > 0x77u) {
        my_addr = BR_DEFAULT_I2C_ADDR;
    }

    SDA_RELEASE();
    state = I2C_IDLE;
    bit = 0u;
    shift = 0u;
    ack = 0u;
    ack_phase = 0u;
    rx_idx = 0u;
    tx_len = 0u;
    tx_idx = 0u;

    last_scl = SCL_READ();
    last_sda = SDA_READ();
}

void host_i2c_soft_poll(void)
{
    uint8_t scl = SCL_READ();
    uint8_t sda = SDA_READ();

    /* START/STOP: SDA edge while SCL is high. */
    if (sda != last_sda && scl) {
        if (!sda) {
            i2c_start();
        } else {
            i2c_stop();
        }
    }

    /* SCL edge. */
    if (scl != last_scl) {
        if (scl) {
            i2c_scl_rising(sda);
        } else {
            i2c_scl_falling();
        }
    }

    last_scl = scl;
    last_sda = sda;
}
