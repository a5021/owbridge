/**
 * @file host_spi.c
 * @brief SPI1 slave host interface, fully polled (no interrupts)
 *
 * CS-framed, two phases per transaction:
 *   1. CS low  -> master sends [cmd][len][data...], slave receives.
 *   CS high -> the bridge submits the command and the response is loaded
 *              into the TX FIFO, so the first clock already returns data.
 *   2. CS low  -> master clocks dummy bytes, slave sends
 *                 [status][cmd][len][data...].
 *
 * CS is polled in host_spi_poll(); the SPI peripheral runs in slave mode with
 * software slave select (SSM=1, SSI=1) and the frame boundaries come from CS.
 * CPOL=0/CPHA=0 (mode 0) is assumed. DMA is not used: SPI1_RX is DMA1 CH2 and
 * SPI1_TX is DMA1 CH3, which the 1-Wire engine owns.
 *
 * DR is accessed as a byte. The register is 16 bits wide on the F0; a 32-bit
 * store can be split by the AHB/APB bridge and clock an extra frame out.
 * The peripheral has a 4-byte FIFO, so each poll drains/fills several bytes
 * instead of one, which leaves the main loop room at faster SPI clocks.
 */

#include "host_spi.h"
#include "bridge.h"
#include "board.h"

#include "stm32f0xx.h"

#define SPI_RX_BUF 80u
#define SPI_TX_BUF (3u + BR_MAX_RESP)
#define SPI_FIFO_DEPTH 4u
#define SPI_IDLE_MS 100u

static uint8_t rx_buf[SPI_RX_BUF];
static uint8_t rx_idx;
static uint8_t tx_buf[SPI_TX_BUF];
static uint8_t tx_len;
static uint8_t tx_idx;

static uint8_t cs_prev;
static uint8_t phase;
static uint32_t cs_edge_ms;

enum {
    SPI_PHASE_CMD = 0u,
    SPI_PHASE_RESP
};

static inline volatile uint8_t *spi_dr(void)
{
    return (volatile uint8_t *)&SPI1->DR;
}

/* Clearing SPE empties both FIFOs and resets the shift register without
   touching the configured mode. It is only done while CS is high, so no
   clock can be lost. */
static void spi_fifo_reset(void)
{
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_SPE;
}

static void spi_fill_tx(void)
{
    uint8_t budget = SPI_FIFO_DEPTH;
    while (budget-- != 0u) {
        if ((SPI1->SR & SPI_SR_FTLVL) == SPI_SR_FTLVL) {
            break; /* TX FIFO full */
        }
        uint8_t b = 0u;
        if (tx_idx < tx_len) {
            b = tx_buf[tx_idx++];
        }
        *spi_dr() = b;
    }
}

/* Read the RX FIFO without changing the phase. Called before a rising CS so
   the last bytes of the command are collected before bridge_submit(). */
static void spi_drain_rx(void)
{
    uint8_t budget = SPI_FIFO_DEPTH;
    while ((SPI1->SR & SPI_SR_RXNE) && budget-- != 0u) {
        uint8_t b = *spi_dr();
        if (phase == SPI_PHASE_CMD && rx_idx < SPI_RX_BUF) {
            rx_buf[rx_idx++] = b;
        }
    }
    if (SPI1->SR & SPI_SR_OVR) {
        (void)*spi_dr();
        (void)SPI1->SR;
        if (phase == SPI_PHASE_CMD) {
            rx_idx = 0u; /* overflowed command: drop the frame */
        }
    }
}

static void spi_prepare_response(void)
{
    uint8_t n = bridge_get_response(tx_buf);
    if (n == 0u) {
        tx_buf[0] = 0u;
        tx_buf[1] = 0u;
        tx_buf[2] = 0u;
        n = 3u;
    }
    tx_len = n;
    tx_idx = 0u;
    /* Old response bytes may still sit in the FIFO if the master read fewer
       bytes than were queued. */
    spi_fifo_reset();
    spi_fill_tx();
}

static void spi_cs_edge(uint8_t cs_low)
{
    if (cs_low) {
        if (phase == SPI_PHASE_CMD) {
            /* Leftover TX bytes were cleared when the previous response
               ended; resetting the FIFO here could swallow a byte the master
               has already clocked. */
            rx_idx = 0u;
        }
        /* Response phase: bytes were loaded at the rising edge. */
    } else {
        if (phase == SPI_PHASE_CMD) {
            if (rx_idx >= 2u) {
                bridge_submit(rx_buf, rx_idx);
                phase = SPI_PHASE_RESP;
                spi_prepare_response();
            } else {
                /* Short or noise-only write: stay ready for a command. */
                rx_idx = 0u;
                spi_fifo_reset();
            }
        } else {
            phase = SPI_PHASE_CMD;
            tx_idx = 0u;
            tx_len = 0u;
            spi_fifo_reset();
        }
    }
    cs_edge_ms = board_millis();
}

void host_spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* Slave, software slave select, 8-bit data, mode 0. With SSM=1 the SSI
       bit is the internal NSS level, and a slave is selected when it is low;
       SSI=1 would leave the peripheral deselected and it would never shift a
       byte. Configure the data size before enabling the peripheral. */
    SPI1->CR2 = SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SPE;

    phase = SPI_PHASE_CMD;
    cs_prev = (GPIOA->IDR & (1u << 4)) ? 1u : 0u;
    cs_edge_ms = board_millis();
    rx_idx = 0u;
    tx_len = 0u;
    tx_idx = 0u;
    spi_fifo_reset();
}

void host_spi_poll(void)
{
    uint32_t now = board_millis();

    /* CS edge. Drain the RX FIFO before a rising edge so the last command
       bytes are in rx_buf before bridge_submit() runs. A falling edge resets
       the frame state first, then the normal drain below picks up any byte
       that arrived with it. */
    uint8_t cs = (GPIOA->IDR & (1u << 4)) ? 1u : 0u;
    if (cs != cs_prev) {
        if (cs != 0u) {
            spi_drain_rx();
        }
        cs_prev = cs;
        spi_cs_edge(cs ? 0u : 1u);
    }

    /* A master that goes away between the command and the read must not
       leave the slave stuck in response phase forever. */
    if (cs != 0u && phase == SPI_PHASE_RESP &&
        (uint32_t)(now - cs_edge_ms) >= SPI_IDLE_MS) {
        phase = SPI_PHASE_CMD;
        tx_idx = 0u;
        tx_len = 0u;
        rx_idx = 0u;
        spi_fifo_reset();
    }

    /* RX: drain the FIFO, a few bytes per call. */
    spi_drain_rx();

    /* Overrun: the frame is unusable; drop it instead of submitting a
       truncated command. */
    if (SPI1->SR & SPI_SR_OVR) {
        (void)*spi_dr();
        (void)SPI1->SR;
        if (phase == SPI_PHASE_CMD) {
            rx_idx = 0u;
        } else {
            phase = SPI_PHASE_CMD;
            tx_idx = 0u;
            tx_len = 0u;
        }
        spi_fifo_reset();
    }

    /* TX: keep the FIFO full while the master clocks the response. */
    if (cs == 0u && phase == SPI_PHASE_RESP) {
        spi_fill_tx();
    }
}
