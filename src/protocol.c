/**
 * @file protocol.c
 * @brief Shared frame helpers - see protocol.h
 */

#include "protocol.h"

uint8_t protocol_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    for (uint8_t i = 0u; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0u; j < 8u; j++) {
            uint8_t mix = (uint8_t)((crc ^ b) & 1u);
            crc >>= 1;
            if (mix) {
                crc ^= 0x8Cu;
            }
            b >>= 1;
        }
    }
    return crc;
}
