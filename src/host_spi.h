/**
 * @file host_spi.h
 * @brief SPI1 slave host interface
 */

#ifndef HOST_SPI_H
#define HOST_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

void host_spi_init(void);
void host_spi_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* HOST_SPI_H */
