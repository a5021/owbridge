/**
 * @file host_i2c_soft.h
 * @brief Software I2C slave host interface (PA4 = SCL, PA5 = SDA)
 */

#ifndef HOST_I2C_SOFT_H
#define HOST_I2C_SOFT_H

#ifdef __cplusplus
extern "C" {
#endif

void host_i2c_soft_init(void);
void host_i2c_soft_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* HOST_I2C_SOFT_H */
