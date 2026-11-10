/**
 * @file config.h
 * @brief Bridge configuration, stored in the last flash page
 *
 * The config page is at 0x08003C00 (1 KB, reserved by the linker script).
 * The structure is 40 bytes; the last field is a CRC-32 over the preceding
 * 36 bytes. A page that fails the magic or the CRC is treated as erased and
 * the defaults are written.
 */

#ifndef BR_CONFIG_H
#define BR_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BR_CONFIG_MAGIC   0x4F574252u /* "OWBR" */
#define BR_CONFIG_VERSION 0x0100u
#define BR_CONFIG_SIZE    40u

struct br_config {
    uint32_t magic;
    uint16_t version;
    uint8_t  i2c_addr;     /* I2C slave address (7-bit) */
    uint8_t  uart_baud;    /* baud rate index, see config_baud() */
    uint8_t  timing;       /* timing preset index (informational) */
    uint8_t  parasite;     /* default parasite power mode */
    uint8_t  strong_pullup;/* external strong pull-up gate present */
    uint8_t  auto_search;  /* search devices at startup */
    uint8_t  max_devices;  /* ROM cache size */
    uint8_t  led_enable;   /* status/error LEDs */
    uint8_t  reserved[22];
    uint32_t crc32;
};

/** @brief Load the config from flash, writing defaults when invalid */
void config_init(void);

/** @brief Pointer to the in-RAM copy of the config */
const struct br_config *config_get(void);

/** @brief Validate, write to flash and update the in-RAM copy */
uint8_t config_set(const struct br_config *cfg);

/** @brief Baud rate for a config uart_baud index */
uint32_t config_baud(uint8_t index);

/** @brief Default I2C address (used when no valid config is present) */
#define BR_DEFAULT_I2C_ADDR 0x30u

#ifdef __cplusplus
}
#endif

#endif /* BR_CONFIG_H */
