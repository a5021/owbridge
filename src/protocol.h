/**
 * @file protocol.h
 * @brief Shared frame helpers (CRC-8)
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Dallas/Maxim CRC-8 (poly 0x8C) */
uint8_t protocol_crc8(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
