/**
 * @file fast_copy.h
 * @brief Small inline copy used in the hot paths instead of libc memcpy
 *
 * The nano libc memcpy is a byte-wise loop. On the 48 MHz Cortex-M0 the hot
 * paths copy at most a few dozen bytes, so a word copy when both pointers are
 * aligned (and a four-byte unrolled byte copy otherwise) is considerably
 * faster. Cortex-M0 has no unaligned load/store support, hence the check.
 */

#ifndef FAST_COPY_H
#define FAST_COPY_H

#include <stdint.h>

static inline __attribute__((always_inline))
void fast_copy(void *dst, const void *src, uint16_t n)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;

    if ((((uintptr_t)d | (uintptr_t)s) & 3u) == 0u) {
        while (n >= 4u) {
            *(uint32_t *)(void *)d = *(const uint32_t *)(const void *)s;
            d += 4u;
            s += 4u;
            n = (uint16_t)(n - 4u);
        }
    } else {
        while (n >= 4u) {
            d[0] = s[0];
            d[1] = s[1];
            d[2] = s[2];
            d[3] = s[3];
            d += 4u;
            s += 4u;
            n = (uint16_t)(n - 4u);
        }
    }
    while (n-- > 0u) {
        *d++ = *s++;
    }
}

#endif /* FAST_COPY_H */
