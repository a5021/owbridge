/**
 * @file host_uart.c
 * @brief USART1 host interface, fully polled (no interrupts)
 *
 * Two protocols share the line:
 *
 *   Binary:  AA 55 [cmd][len][data...] [crc8]
 *   Response: 5A A5 [status][cmd][len][data...] [crc8]
 *
 *   AT (text, CR/LF terminated):
 *     AT, AT+VER, AT+OWRST, AT+OWSRCH, AT+OWCNT, AT+TEMP, AT+STAT, AT+DEVS
 *
 * TX uses DMA1 channel 2; RX uses DMA1 channel 5 in circular mode, remapped
 * from the default channel 3 (which the 1-Wire library owns) through
 * SYSCFG_CFGR1.USART1RX_DMA_RMP. The main loop polls the DMA state and
 * processes at most a few RX bytes per call so it stays short.
 */

#include "host_uart.h"
#include "bridge.h"
#include "config.h"
#include "protocol.h"

#include "stm32f0xx.h"

#include <string.h>

#define UART_RX_BUF 128u
#define UART_TX_BUF 200u
#define AT_LINE_BUF 48u

/* --- RX DMA ring ---------------------------------------------------------- */

static uint8_t rx_dma_buf[UART_RX_BUF];
static uint8_t rx_head;

/* --- TX DMA --------------------------------------------------------------- */

static uint8_t tx_dma_buf[UART_TX_BUF];
static uint8_t tx_pos;
static uint8_t tx_busy;

/* --- Binary RX decoder ---------------------------------------------------- */

static uint8_t rx_frame[80];
static uint8_t rx_idx;
static uint8_t rx_state;
static uint8_t rx_len;

/* --- AT state ------------------------------------------------------------- */

static char at_line[AT_LINE_BUF];
static uint8_t at_idx;
static uint8_t at_active;
static uint8_t at_cmd;
static uint8_t at_temp_step;

/* --- Response tracking ---------------------------------------------------- */

static uint8_t last_seq;

enum {
    AT_NONE = 0u,
    AT_TEST,
    AT_VER,
    AT_OWRST,
    AT_OWSRCH,
    AT_OWCNT,
    AT_TEMP,
    AT_STAT,
    AT_DEVS
};

/* --- TX into the DMA buffer ---------------------------------------------- */

static void tx_putc(char c)
{
    if (tx_pos < UART_TX_BUF) {
        tx_dma_buf[tx_pos++] = (uint8_t)c;
    }
}

static void tx_puts(const char *s)
{
    while (*s) {
        tx_putc(*s++);
    }
}

static void tx_put_int(int32_t v)
{
    char buf[12];
    char *p = buf + sizeof(buf) - 1u;
    *p = '\0';
    if (v == 0) {
        *(--p) = '0';
    } else {
        uint32_t uvalue;
        uint8_t neg = 0u;
        if (v < 0) {
            neg = 1u;
            uvalue = (uint32_t)-(v + 1) + 1u;
        } else {
            uvalue = (uint32_t)v;
        }
        do {
            *(--p) = (char)('0' + (uvalue % 10u));
            uvalue /= 10u;
        } while (uvalue);
        if (neg) {
            *(--p) = '-';
        }
    }
    tx_puts(p);
}

static void tx_put_hex(uint8_t b)
{
    static const char hex[] = "0123456789ABCDEF";
    tx_putc(hex[b >> 4]);
    tx_putc(hex[b & 0x0Fu]);
}

static void uart_tx_start(uint8_t len)
{
    if (len == 0u) {
        return;
    }
    DMA1_Channel2->CCR = 0u;
    DMA1_Channel2->CMAR = (uint32_t)tx_dma_buf;
    DMA1_Channel2->CNDTR = len;
    DMA1_Channel2->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_EN;
    tx_busy = 1u;
}

/* --- Response push -------------------------------------------------------- */

static void push_binary_response(void)
{
    const uint8_t *resp = bridge_response_ptr();
    uint8_t n = bridge_response_len();
    tx_pos = 0u;
    tx_putc((char)0x5A);
    tx_putc((char)0xA5);
    for (uint8_t i = 0u; i < n; i++) {
        tx_putc((char)resp[i]);
    }
    tx_putc((char)protocol_crc8(resp, n));
    uart_tx_start(tx_pos);
}

static void push_at_response(void)
{
    const uint8_t *resp = bridge_response_ptr();
    uint8_t n = bridge_response_len();
    uint8_t status = (n >= 3u) ? resp[0] : BR_BUS_ERROR;

    if (status == BR_BUSY) {
        return; /* wait for the final response */
    }

    /* AT+TEMP is two commands: Convert T, then read every scratchpad. */
    if (at_cmd == AT_TEMP && at_temp_step == 0u) {
        uint8_t frame[2] = { BR_CMD_TEMP_READ_ALL, 0u };
        at_temp_step = 1u;
        bridge_submit(frame, 2u);
        return;
    }

    tx_pos = 0u;

    if (status != BR_OK) {
        tx_puts(status == BR_NO_DEVICE ? "NO DEVICE\r\n" : "ERROR\r\n");
        uart_tx_start(tx_pos);
        at_active = 0u;
        return;
    }

    switch (at_cmd) {
    case AT_TEST:
        tx_puts("OK\r\n");
        break;
    case AT_VER:
        tx_puts("owbridge 1.0.0 (lib 3.1.2)\r\n");
        break;
    case AT_OWRST:
        tx_puts("OK\r\n");
        break;
    case AT_OWSRCH:
    case AT_OWCNT:
        tx_puts("COUNT ");
        tx_put_int(resp[3]);
        tx_puts("\r\n");
        break;
    case AT_TEMP:
        tx_puts("TEMP");
        for (uint8_t i = 0u; i < resp[2]; i += 2u) {
            int16_t t = (int16_t)((resp[3u + i] << 8) | resp[3u + i + 1u]);
            tx_puts(" ");
            tx_put_int(t / 10);
            tx_puts(".");
            tx_put_int((t % 10) < 0 ? -(t % 10) : (t % 10));
        }
        tx_puts("\r\n");
        break;
    case AT_STAT: {
        uint32_t r = ((uint32_t)resp[3] << 24) | ((uint32_t)resp[4] << 16) |
                     ((uint32_t)resp[5] << 8) | (uint32_t)resp[6];
        uint32_t pf = ((uint32_t)resp[7] << 24) | ((uint32_t)resp[8] << 16) |
                      ((uint32_t)resp[9] << 8) | (uint32_t)resp[10];
        tx_puts("STAT resets=");
        tx_put_int((int32_t)r);
        tx_puts(" no_presence=");
        tx_put_int((int32_t)pf);
        tx_puts("\r\n");
        break;
    }
    case AT_DEVS:
        tx_puts("DEVS ");
        for (uint8_t i = 0u; i < resp[2]; i += 8u) {
            for (uint8_t j = 0u; j < 8u; j++) {
                tx_put_hex(resp[3u + i + j]);
            }
            tx_puts(" ");
        }
        tx_puts("\r\n");
        break;
    default:
        tx_puts("OK\r\n");
        break;
    }
    uart_tx_start(tx_pos);
    at_active = 0u;
}

/* --- AT parsing ----------------------------------------------------------- */

static uint8_t at_parse(const char *line)
{
    if (strcmp(line, "AT") == 0) {
        return AT_TEST;
    }
    if (strcmp(line, "AT+VER") == 0) {
        return AT_VER;
    }
    if (strcmp(line, "AT+OWRST") == 0) {
        return AT_OWRST;
    }
    if (strcmp(line, "AT+OWSRCH") == 0) {
        return AT_OWSRCH;
    }
    if (strcmp(line, "AT+OWCNT") == 0) {
        return AT_OWCNT;
    }
    if (strcmp(line, "AT+TEMP") == 0) {
        return AT_TEMP;
    }
    if (strcmp(line, "AT+STAT") == 0) {
        return AT_STAT;
    }
    if (strcmp(line, "AT+DEVS") == 0) {
        return AT_DEVS;
    }
    return AT_NONE;
}

static void at_submit(uint8_t cmd)
{
    uint8_t frame[12];
    uint8_t len = 0u;

    if (cmd == AT_TEST) {
        tx_pos = 0u;
        tx_puts("OK\r\n");
        uart_tx_start(tx_pos);
        return;
    }
    if (cmd == AT_VER) {
        tx_pos = 0u;
        tx_puts("owbridge 1.0.0 (lib 3.1.2)\r\n");
        uart_tx_start(tx_pos);
        return;
    }

    switch (cmd) {
    case AT_OWRST:
        frame[0] = BR_CMD_OW_RESET;
        frame[1] = 0u;
        len = 2u;
        break;
    case AT_OWSRCH:
        frame[0] = BR_CMD_OW_SEARCH;
        frame[1] = 3u;
        frame[2] = 0xF0u;
        frame[3] = 0x00u;
        frame[4] = 16u;
        len = 5u;
        break;
    case AT_OWCNT:
        frame[0] = BR_CMD_OW_GET_COUNT;
        frame[1] = 0u;
        len = 2u;
        break;
    case AT_TEMP:
        frame[0] = BR_CMD_TEMP_CONVERT;
        frame[1] = 0u;
        len = 2u;
        at_temp_step = 0u;
        break;
    case AT_STAT:
        frame[0] = BR_CMD_STATS_GET;
        frame[1] = 0u;
        len = 2u;
        break;
    case AT_DEVS:
        frame[0] = BR_CMD_OW_GET_DEVICES;
        frame[1] = 2u;
        frame[2] = 0u;
        frame[3] = 12u;
        len = 4u;
        break;
    default:
        return;
    }
    bridge_submit(frame, len);
    at_cmd = cmd;
    at_active = 1u;
}

/* --- Binary RX decoder ---------------------------------------------------- */

static void uart_decode(uint8_t b)
{
    switch (rx_state) {
    case 0u:
        if (b == 0xAAu) {
            rx_state = 1u;
        }
        break;
    case 1u:
        rx_state = (b == 0x55u) ? 2u : 0u;
        break;
    case 2u:
        rx_frame[0] = b;
        rx_state = 3u;
        break;
    case 3u:
        rx_frame[1] = b;
        rx_len = b;
        rx_idx = 2u;
        rx_state = (rx_len > 0u) ? 4u : 5u;
        break;
    case 4u:
        if (rx_idx < sizeof(rx_frame)) {
            rx_frame[rx_idx++] = b;
        }
        if (rx_idx >= (uint8_t)(2u + rx_len)) {
            rx_state = 5u;
        }
        break;
    case 5u:
        if (protocol_crc8(rx_frame, (uint8_t)(2u + rx_len)) == b) {
            bridge_submit(rx_frame, (uint8_t)(2u + rx_len));
        }
        rx_state = 0u;
        break;
    default:
        rx_state = 0u;
        break;
    }
}

/* --- One received byte ---------------------------------------------------- */

static void uart_rx_byte(uint8_t b)
{
    if (rx_state != 0u || b == 0xAAu) {
        uart_decode(b);
    } else if (b == '\r' || b == '\n') {
        if (at_idx > 0u) {
            at_line[at_idx] = '\0';
            uint8_t cmd = at_parse(at_line);
            at_idx = 0u;
            if (cmd == AT_NONE) {
                tx_pos = 0u;
                tx_puts("ERROR\r\n");
                uart_tx_start(tx_pos);
            } else if (at_active) {
                tx_pos = 0u;
                tx_puts("BUSY\r\n");
                uart_tx_start(tx_pos);
            } else {
                at_submit(cmd);
            }
        }
    } else if (at_idx < (AT_LINE_BUF - 1u)) {
        at_line[at_idx++] = (char)b;
    }
}

/* --- Public --------------------------------------------------------------- */

void host_uart_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN;

    /* USART1_RX DMA: default is channel 3 (owned by the 1-Wire feed). */
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1RX_DMA_RMP;

    uint32_t baud = config_baud(config_get()->uart_baud);
    USART1->BRR = (uint32_t)(((uint32_t)OW_SYSCLK_MHZ * 1000000u) + (baud / 2u)) / baud;
    USART1->CR2 = 0u;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART1->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;

    /* RX: DMA1 channel 5, circular. */
    DMA1_Channel5->CCR = 0u;
    DMA1_Channel5->CPAR = (uint32_t)&USART1->RDR;
    DMA1_Channel5->CMAR = (uint32_t)rx_dma_buf;
    DMA1_Channel5->CNDTR = UART_RX_BUF;
    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;

    /* TX: DMA1 channel 2, configured per transfer. */
    DMA1_Channel2->CCR = 0u;
    DMA1_Channel2->CPAR = (uint32_t)&USART1->TDR;

    rx_head = 0u;
    tx_busy = 0u;
    last_seq = 0u;
    rx_state = 0u;
    at_idx = 0u;
    at_active = 0u;
}

void host_uart_poll(void)
{
    /* 1. Consume up to a few RX bytes from the circular DMA buffer. */
    uint16_t cndtr = DMA1_Channel5->CNDTR;
    uint8_t pos = (uint8_t)(UART_RX_BUF - cndtr);
    uint8_t budget = 8u;
    while (rx_head != pos && budget > 0u) {
        uint8_t b = rx_dma_buf[rx_head];
        rx_head = (uint8_t)((rx_head + 1u) % UART_RX_BUF);
        uart_rx_byte(b);
        budget--;
    }

    /* 2. TX completion. */
    if (tx_busy && (DMA1->ISR & DMA_ISR_TCIF2)) {
        DMA1->IFCR = DMA_IFCR_CTCIF2;
        DMA1_Channel2->CCR = 0u;
        tx_busy = 0u;
    }

    /* 3. Push a new response when the sequence changed. */
    uint8_t seq = bridge_response_seq();
    if (seq != last_seq && !tx_busy) {
        last_seq = seq;
        if (at_active) {
            push_at_response();
        } else {
            push_binary_response();
        }
    }
}
