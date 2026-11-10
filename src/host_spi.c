/**
 * @file host_spi.c
 * @brief SPI1 slave host interface, fully polled (no interrupts)
 *
 * CS-framed, two phases per transaction:
 *   1. CS low  -> master sends [cmd][len][data...], slave receives.
 *   CS high -> the bridge submits the command.
 *   2. CS low  -> master clocks dummy bytes, slave sends
 *                 [status][cmd][len][data...].
 *
 * CS is polled in host_spi_poll(); the SPI peripheral runs in slave mode with
 * software slave select (SSM=1, SSI=1) and the frame boundaries come from CS.
 * CPOL=0/CPHA=0 (mode 0) is assumed. DMA is not used: SPI1_RX is DMA1 CH2 and
 * SPI1_TX is DMA1 CH3, which the 1-Wire engine owns.
 */

#include "host_spi.h"
#include "bridge.h"

#include "stm32f0xx.h"

#define SPI_RX_BUF 80u
#define SPI_TX_BUF (3u + BR_MAX_RESP)

static uint8_t rx_buf[SPI_RX_BUF];
static uint8_t rx_idx;
static uint8_t tx_buf[SPI_TX_BUF];
static uint8_t tx_len;
static uint8_t tx_idx;

static uint8_t cs_prev;
static uint8_t phase;

enum {
    SPI_PHASE_CMD = 0u,
    SPI_PHASE_RESP
};

static void spi_cs_edge(uint8_t cs_low)
{
    if (cs_low) {
        if (phase == SPI_PHASE_CMD) {
            rx_idx = 0u;
        } else {
            /* Response phase: load the first byte immediately. */
            tx_len = bridge_get_response(tx_buf);
            if (tx_len == 0u) {
                tx_len = 3u;
            }
            tx_idx = 0u;
            if (SPI1->SR & SPI_SR_TXE) {
                SPI1->DR = tx_buf[tx_idx++];
            }
        }
    } else {
        if (phase == SPI_PHASE_CMD) {
            if (rx_idx >= 2u) {
                bridge_submit(rx_buf, rx_idx);
            }
            phase = SPI_PHASE_RESP;
        } else {
            phase = SPI_PHASE_CMD;
        }
    }
}

void host_spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* Slave, software slave select, 8-bit data, mode 0. */
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;
    SPI1->CR2 = SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;

    phase = SPI_PHASE_CMD;
    cs_prev = (GPIOA->IDR & (1u << 4)) ? 1u : 0u;
    rx_idx = 0u;
    tx_len = 0u;
    tx_idx = 0u;
}

void host_spi_poll(void)
{
    /* CS edge. */
    uint8_t cs = (GPIOA->IDR & (1u << 4)) ? 1u : 0u;
    if (cs != cs_prev) {
        cs_prev = cs;
        spi_cs_edge(cs ? 0u : 1u);
    }

    /* RX: read one byte per call. */
    if (SPI1->SR & SPI_SR_RXNE) {
        uint8_t b = (uint8_t)SPI1->DR;
        if (phase == SPI_PHASE_CMD && rx_idx < SPI_RX_BUF) {
            rx_buf[rx_idx++] = b;
        }
    }

    /* TX: feed one byte per call while CS is low. */
    if (cs == 0u && (SPI1->SR & SPI_SR_TXE)) {
        if (phase == SPI_PHASE_RESP && tx_idx < tx_len) {
            SPI1->DR = tx_buf[tx_idx++];
        } else {
            SPI1->DR = 0x00u;
        }
    }

    /* Overrun: clear it. */
    if (SPI1->SR & SPI_SR_OVR) {
        (void)SPI1->DR;
        (void)SPI1->SR;
    }
}
