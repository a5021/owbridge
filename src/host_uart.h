/**
 * @file host_uart.h
 * @brief USART2 host interface
 */

#ifndef HOST_UART_H
#define HOST_UART_H

#ifdef __cplusplus
extern "C" {
#endif

void host_uart_init(void);
void host_uart_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* HOST_UART_H */
