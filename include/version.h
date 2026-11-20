/**
 * @file version.h
 * @brief Single source of truth for the firmware version string
 *
 * The same text is returned by FIRMWARE_VER (all transports) and by AT+VER
 * and the AT "version" line (UART build). Hosts use the version to tell a
 * pre-1.1.0 TEMP_READ_ALL, which compacted failed readings, from the current
 * positional-slot format.
 */

#ifndef OWBRIDGE_VERSION_H
#define OWBRIDGE_VERSION_H

#define OWBRIDGE_VERSION "owbridge 1.4.0 (lib 3.1.3)"

#endif /* OWBRIDGE_VERSION_H */
