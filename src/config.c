/**
 * @file config.c
 * @brief Bridge configuration in the last flash page - see config.h
 */

#include "config.h"

#include "stm32f0xx.h"

#include <string.h>

/* Linker symbol: the reserved config page (linker/STM32F030X6_FLASH.ld). */
extern const uint8_t _config_page[];

#define CONFIG_ADDR ((uint32_t)(uintptr_t)_config_page)

static struct br_config config_ram;

static const uint32_t baud_table[] = {
    9600u, 19200u, 38400u, 57600u, 115200u, 230400u
};
#define BAUD_TABLE_LEN (sizeof(baud_table) / sizeof(baud_table[0]))

uint32_t config_baud(uint8_t index)
{
    return (index < BAUD_TABLE_LEN) ? baud_table[index] : 115200u;
}

/* --- CRC-32 (IEEE 802.3, reflected) --------------------------------------- */

static uint32_t config_crc32(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint16_t i = 0u; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0u; b < 8u; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

/* --- Flash page access ---------------------------------------------------- */

static void flash_unlock(void)
{
    FLASH->KEYR = 0x45670123u;
    FLASH->KEYR = 0xCDEF89ABu;
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_wait(void)
{
    while (FLASH->SR & FLASH_SR_BSY) {
    }
}

static void flash_erase_page(uint32_t addr)
{
    flash_unlock();
    flash_wait();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = addr;
    FLASH->CR |= FLASH_CR_STRT;
    flash_wait();
    FLASH->CR &= ~FLASH_CR_PER;
    flash_lock();
}

static void flash_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
    flash_unlock();
    flash_wait();
    FLASH->CR |= FLASH_CR_PG;
    for (uint16_t i = 0u; i < len; i += 2u) {
        uint16_t half = (uint16_t)data[i] | ((uint16_t)data[i + 1u] << 8);
        *(__IO uint16_t *)(addr + i) = half;
        flash_wait();
    }
    FLASH->CR &= ~FLASH_CR_PG;
    flash_lock();
}

/* --- Public API ----------------------------------------------------------- */

static void config_defaults(struct br_config *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = BR_CONFIG_MAGIC;
    cfg->version = BR_CONFIG_VERSION;
    cfg->i2c_addr = BR_DEFAULT_I2C_ADDR;
    cfg->uart_baud = 4u;     /* 115200 */
    cfg->timing = 0u;        /* STANDARD */
    cfg->parasite = 1u;      /* safe for both parasite and externally powered buses */
    cfg->strong_pullup = 1u;
    cfg->auto_search = 1u;
    cfg->max_devices = 16u;
    cfg->led_enable = 1u;
}

static uint8_t config_valid(const struct br_config *cfg)
{
    if (cfg->magic != BR_CONFIG_MAGIC) {
        return 0u;
    }
    if (cfg->version != BR_CONFIG_VERSION) {
        return 0u;
    }
    uint32_t crc = config_crc32((const uint8_t *)cfg, BR_CONFIG_SIZE - 4u);
    return (crc == cfg->crc32) ? 1u : 0u;
}

void config_init(void)
{
    const struct br_config *flash_cfg = (const struct br_config *)CONFIG_ADDR;

    if (config_valid(flash_cfg)) {
        memcpy(&config_ram, flash_cfg, sizeof(config_ram));
    } else {
        struct br_config def;
        config_defaults(&def);
        def.crc32 = config_crc32((const uint8_t *)&def, BR_CONFIG_SIZE - 4u);
        memcpy(&config_ram, &def, sizeof(config_ram));
        config_set(&config_ram);
    }
}

const struct br_config *config_get(void)
{
    return &config_ram;
}

uint8_t config_set(const struct br_config *cfg)
{
    if (cfg == 0) {
        return 0u;
    }
    struct br_config tmp = *cfg;
    tmp.magic = BR_CONFIG_MAGIC;
    tmp.version = BR_CONFIG_VERSION;
    tmp.crc32 = config_crc32((const uint8_t *)&tmp, BR_CONFIG_SIZE - 4u);

    flash_erase_page(CONFIG_ADDR);
    flash_program(CONFIG_ADDR, (const uint8_t *)&tmp, BR_CONFIG_SIZE);
    memcpy(&config_ram, &tmp, sizeof(tmp));
    return 1u;
}
