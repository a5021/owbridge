/**
 * @file host_i2c_soft.h
 * @brief Software I2C slave host interface (PA4 = SCL, PA5 = SDA)
 */

#ifndef HOST_I2C_SOFT_H
#define HOST_I2C_SOFT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void host_i2c_soft_init(void);
void host_i2c_soft_poll(void);

/** @brief 1 while an I2C transaction is active (START..STOP), 0 when idle */
uint8_t host_i2c_soft_busy(void);

/** @brief Monotonic count of observed SCL/SDA edges (wraps at 2^32) */
uint32_t host_i2c_soft_edge_count(void);

#ifdef __cplusplus
}
#endif

#endif /* HOST_I2C_SOFT_H */
