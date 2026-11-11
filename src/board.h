/**
 * @file board.h
 * @brief Board bring-up for the owbridge on STM32F030F4P6 (TSSOP-20)
 *
 * Provides what the 1-Wire library does not: the 48 MHz system clock, the
 * host-interface GPIOs, a polled millisecond tick and the independent
 * watchdog. Nothing here installs an interrupt, and every host interface is
 * polled as well; the 1-Wire engine stays interrupt-free.
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Bring up the clock, GPIOs, tick and watchdog */
void board_init(void);

/** @brief Reset the MCU; does not return */
void board_reset(void);

/** @brief Milliseconds since board_init() (wraps after ~49 days) */
uint32_t board_millis(void);

#if defined(HOST_IF_UART)
/* --- Debug output helpers (UART build only) ------------------------------- */
/* USART1 is owned by host_uart.c (PA2 = TX, PA3 = RX). These helpers are not
   a separate console; call them only after host_uart_init(). */

void board_putc(char c);
void board_puts(const char *s);
void board_put_int(int32_t v);
void board_put_hex(uint8_t b);
#endif

/* --- Indicators and actuators -------------------------------------------- */

/** @brief Status LED (PA0): 1 = on */
void board_led_status(uint8_t on);

/** @brief Error LED (PA1): 1 = on */
void board_led_error(uint8_t on);

/** @brief External strong pull-up switch (PA9): 1 = pull-up active */
void board_gate_pullup(uint8_t on);

/* --- Independent watchdog ------------------------------------------------ */

void board_watchdog_init(void);
void board_watchdog_refresh(void);

/* --- Configuration jumpers (PB1/PF0, active-low, internal pull-up) ------- */

/** @brief Read the jumper inputs as a 2-bit value (bit0 = PB1, bit1 = PF0) */
uint8_t board_read_jumpers(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
