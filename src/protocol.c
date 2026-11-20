/**
 * @file protocol.c
 * @brief Shared frame helpers - see protocol.h
 *
 * Bitwise Dallas/Maxim CRC-8 (polynomial 0x8C, reflected). The table-driven
 * version cost 256 bytes of flash; the bitwise loop keeps the image small.
 * CRC is not in the I2C sampling path.
 */

#include "protocol.h"

uint8_t protocol_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    for (uint8_t i = 0u; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            crc = (uint8_t)((crc & 0x80u) ? ((crc << 1) ^ 0x8Cu) : (crc << 1));
        }
    }
    return crc;
}
