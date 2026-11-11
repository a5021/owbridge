/**
 * @file bridge.h
 * @brief I2C/UART/SPI <-> 1-Wire bridge core
 *
 * Frame protocol (identical on all three host interfaces):
 *
 *   Command:  [cmd][len][data...]
 *   Response: [status][cmd][len][data...]
 *
 * `len` is the number of data bytes (0..BR_MAX_DATA, currently 40). The host
 * reads the 3-byte header first, then `len` data bytes.
 *
 * A command is executed asynchronously: while the 1-Wire bus is busy the
 * response status is BR_BUSY and the host polls. Commands submitted while the
 * bus is busy are queued (up to BR_QUEUE_SIZE); the response status is
 * BR_QUEUE_FULL when the queue is full.
 */

#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Sizes ---------------------------------------------------------------- */

#define BR_MAX_DATA 40u   /**< Maximum command data bytes */
#define BR_MAX_RESP 160u  /**< Maximum response data bytes */
#define BR_QUEUE_SIZE 2u  /**< Command queue depth */

/* --- Status codes --------------------------------------------------------- */

enum {
    BR_OK            = 0x00u,
    BR_BUSY          = 0x01u,
    BR_NO_DEVICE     = 0x02u,
    BR_CRC_ERROR     = 0x03u,
    BR_BUS_SHORT     = 0x04u,
    BR_TIMEOUT       = 0x05u,
    BR_INVALID_CMD   = 0x06u,
    BR_PARAM_ERROR   = 0x07u,
    BR_BUS_ERROR     = 0x08u,
    BR_NOT_SUPPORTED = 0x09u,
    BR_QUEUE_FULL    = 0x0Au,
    BR_ABORTED       = 0x0Bu
};

/* --- Commands ------------------------------------------------------------- */

enum {
    /* Raw 1-Wire */
    BR_CMD_OW_RESET       = 0x01u,
    BR_CMD_OW_WRITE_BYTE  = 0x02u,
    BR_CMD_OW_READ_BYTE   = 0x03u,
    BR_CMD_OW_WRITE_BYTES = 0x04u,
    BR_CMD_OW_READ_BYTES  = 0x05u,
    BR_CMD_OW_MATCH_ROM   = 0x06u,  /* data: rom[8] + payload; reset+match+write */
    BR_CMD_OW_SKIP_ROM    = 0x07u,  /* data: payload; reset+skip+write */
    BR_CMD_OW_READ_ROM    = 0x08u,  /* reset+0x33+read 8 bytes */
    BR_CMD_OW_SEARCH      = 0x09u,  /* data: cmd, family, max */
    BR_CMD_OW_SEARCH_ALM  = 0x0Au,  /* data: max */
    BR_CMD_OW_SET_PULLUP  = 0x0Bu,  /* data: on/off */
    BR_CMD_OW_SET_POWER   = 0x0Cu,  /* data: parasite/ext */
    BR_CMD_OW_GET_STATUS  = 0x0Du,
    BR_CMD_OW_GET_DEVICES = 0x0Eu,  /* data: offset, count */
    BR_CMD_OW_GET_COUNT   = 0x0Fu,
    BR_CMD_OW_CLEAR_CACHE = 0x10u,
    BR_CMD_OW_ABORT       = 0x11u,

    /* DS18B20 temperature */
    BR_CMD_TEMP_CONVERT   = 0x20u,  /* data: rom[8] or 0 = all */
    BR_CMD_TEMP_READ      = 0x21u,  /* data: rom[8] */
    BR_CMD_TEMP_READ_ALL  = 0x22u,
    BR_CMD_TEMP_SET_RES   = 0x23u,  /* data: rom[8], bits */
    BR_CMD_TEMP_GET_RES   = 0x24u,  /* data: rom[8] */
    BR_CMD_TEMP_SET_ALARM = 0x25u,  /* data: rom[8], th, tl */
    BR_CMD_TEMP_GET_ALARM = 0x26u,  /* data: rom[8] */
    BR_CMD_TEMP_COPY      = 0x27u,  /* data: rom[8] */
    BR_CMD_TEMP_RECALL    = 0x28u,  /* data: rom[8] */
    BR_CMD_TEMP_DETECT_P  = 0x29u,  /* data: rom[8] */

    /* DS2431 EEPROM */
    BR_CMD_EEPROM_READ    = 0x30u,  /* data: rom[8], addr, len */
    BR_CMD_EEPROM_WRITE   = 0x31u,  /* data: rom[8], addr, data.. */
    BR_CMD_EEPROM_READ_S  = 0x32u,  /* data: rom[8] */

    /* DS2413 PIO */
    BR_CMD_IO_READ        = 0x40u,  /* data: rom[8] */
    BR_CMD_IO_WRITE       = 0x41u,  /* data: rom[8], states */

    /* Service */
    BR_CMD_CONFIG_GET     = 0x50u,
    BR_CMD_CONFIG_SET     = 0x51u,
    BR_CMD_FIRMWARE_VER   = 0x52u,
    BR_CMD_RESET          = 0x53u,
    BR_CMD_STATS_GET      = 0x54u,
    BR_CMD_STATS_RESET    = 0x55u
};

/* --- Types ---------------------------------------------------------------- */

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t pad[2];   /**< keeps data[] word-aligned for fast_copy() */
    uint8_t data[BR_MAX_DATA];
} __attribute__((aligned(4))) br_command_t;

typedef struct {
    uint8_t status;
    uint8_t cmd;
    uint8_t len;
    uint8_t *data;    /**< points at the response data area in bridge.c */
} br_response_t;

/* --- API ------------------------------------------------------------------ */

/** @brief Initialise the bridge (library, cache, config, stats) */
void bridge_init(void);

/** @brief Advance the bus state machine; call from the main loop */
void bridge_poll(void);

/**
 * @brief Submit a command frame
 * @param frame Pointer to [cmd][len][data...]
 * @param len   Total frame length (2 + data_len)
 * @return 1 if accepted (started or queued), 0 if the queue is full
 */
uint8_t bridge_submit(const uint8_t *frame, uint8_t len);

/**
 * @brief Copy the current response to `out`
 * @param out Destination buffer (at least 3 + BR_MAX_RESP bytes)
 * @return Total response length (3 + data_len)
 */
uint8_t bridge_get_response(uint8_t *out);

/** @brief Abort the current bus operation */
void bridge_abort(void);

/** @brief Whether the bus is busy */
uint8_t bridge_busy(void);

/** @brief Monotonic response counter (changes on every response update) */
uint8_t bridge_response_seq(void);

/** @brief Pointer to the current response frame [status][cmd][len][data...] */
const uint8_t *bridge_response_ptr(void);

/** @brief Length of the current response frame */
uint8_t bridge_response_len(void);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_H */
