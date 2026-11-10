/**
 * @file board.h
 * @brief Board bring-up for the owbridge on STM32F030F4P6 (TSSOP-20)
 *
 * Provides what the 1-Wire library does not: the 48 MHz system clock, the
 * host-interface GPIOs, a polled millisecond tick and the independent
 * watchdog. Nothing here installs an interrupt; the host interfaces bring
 * their own (I2C/USART/SPI), the 1-Wire engine stays interrupt-free.
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Bring up the clock, GPIOs, tick and watchdog */
void board_init(void);

/** @brief Milliseconds since board_init() (wraps after ~49 days) */
uint32_t board_millis(void);

/* --- Debug console (USART1 TX, PA9, 115200 8N1) -------------------------- */

void board_putc(char c);
void board_puts(const char *s);
void board_put_int(int32_t v);
void board_put_hex(uint8_t b);

/* --- Indicators and actuators -------------------------------------------- */

/** @brief Status LED (PA11, active-low) */
void board_led_status(uint8_t on);

/** @brief Error LED (PA12, active-low) */
void board_led_error(uint8_t on);

/** @brief External strong pull-up gate (PA8): 1 = P-MOSFET conducts */
void board_gate_pullup(uint8_t on);

/* --- Independent watchdog ------------------------------------------------ */

void board_watchdog_init(void);
void board_watchdog_refresh(void);

/* --- Configuration jumpers (PA0/PA1, active-low, internal pull-up) ------- */

/** @brief Read the two jumper inputs as a 2-bit value (bit0 = PA0, bit1 = PA1) */
uint8_t board_read_jumpers(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
