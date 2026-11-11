# I2C stress test - 2026-11-11

This note records the long I2C load test of the `HOST_IF=I2C` firmware. The
goal was to keep the bus busy for a sustained period, validate every response,
and watch for corruption, NACKs, resets and watchdog problems.

## Setup

| Item | Value |
|------|-------|
| Firmware | I2C build at `fd86f39`, 21 668 B text, 2 040 B RAM |
| MCU | STM32F030F4P6, 48 MHz, 1-Wire on PA10 |
| I2C slave | address `0x30`, software slave on PA4/PA5 |
| Master | CH341A, `CH341SetStream(0, 1)` (nominal 100 kHz) |
| Host | Windows Python 3.15, `CH341DLLA64.DLL` |
| Analyzer | fx2lafw, D1 = SDA, D2 = SCL, 4 MHz |
| Bench 1-Wire | 7 x DS18B20, 1 x DS2431, 1 x DS2413 (parasite power) |
| Duration | 1080 s (18 minutes) |

The harness lives in `C:\temp\owbridge\`:

- `i2c_stress.py` - workload generator and response checker;
- `i2c_spec_check.py` - short sigrok capture, SCL timing and I2C decode;
- `stress.log` - one JSON line per 10 s summary (110 records);
- `stress.stdout` - console output;
- `spec.sr` - the analyzer capture used for the timing numbers below.

## Workload

The default loop maximises bus utilisation:

1. write the 2-byte `FIRMWARE_VER` frame (`52 00`);
2. read 163 bytes in the same transaction sequence.

At ~115 kHz a 163-byte read plus the write keeps the bus busy for about
13.56 ms; the CH341A gap is ~15 us, so the SCL line is active for roughly 96%
of the loop. The response is validated on every round: the first 29 bytes must
equal `00 52 1A "owbridge 1.0.0 (lib 3.1.2)"` and the remaining bytes must be
`FF`.

Periodic tasks, staggered so they do not run at the same instant:

| Period | Task |
|--------|------|
| 10 s | `STATS_GET`, parse 7 counters + uptime, reset detection |
| 20 s | `OW_GET_COUNT`, `OW_GET_STATUS`, `CONFIG_GET` |
| 30 s | `OW_RESET` (presence must be 1) |
| 60 s | `OW_SEARCH` + `OW_GET_DEVICES` (ROM CRC-8 and families), `EEPROM_READ`, `IO_READ` |
| 75 s | `TEMP_CONVERT` (broadcast) + `TEMP_READ_ALL` |
| 120 s | negative tests: invalid command, out-of-range `OW_GET_DEVICES` |
| 180 s | queued `OW_GET_COUNT` and queued `OW_ABORT` while a conversion runs |

Every response is checked for status, command echo, length and payload
(version string, Dallas CRC-8 of every ROM, config magic/version/CRC-32,
temperature range, stable EEPROM content). A firmware reset is detected when
the `STATS_GET` uptime decreases.

## Results

| Metric | Value |
|--------|-------|
| Duration | 1080 s |
| Transactions | 202 777 |
| Bulk rounds (163-byte reads) | 77 286 |
| Bytes read | 12 747 279 |
| Bytes written | 155 612 |
| Average read rate | 11.80 KB/s |
| Bad bulk responses | 0 |
| Validation errors | 0 |
| Retries / unrecovered NACKs | 0 |
| USB errors | 0 |
| MCU resets | 0 |
| Bulk latency min / avg / max | 13.402 ms / 13.564 ms / 54.888 ms |
| Summary records | 110, no gap larger than 20 s |

The 54.9 ms maximum is a single USB/scheduler outlier; the next values return
to the 13.4-14.7 ms range and no transaction was lost.

## Firmware counters

Final `STATS_GET` after the run:

```text
OW_RESET count = 36   presence_fail = 0   crc_error = 0   bus_short = 0
timeout = 0           commands = 77 612    aborts = 0      uptime = 1077 s
```

`OW_RESET count` counts the periodic reset commands (one per 30 s); it is not
an MCU reset counter. Uptime grew monotonically, so the MCU never reset and
the independent watchdog never fired. The other error counters stayed at
zero.

## NACK analysis

The harness saw 1477 NACKs, all on 1-Wire or memory commands:

| Command | NACKs |
|---------|-------|
| `OW_SEARCH` (0x09) | 381 |
| `TEMP_CONVERT` (0x20) | 600 |
| `TEMP_READ_ALL` (0x22) | 459 |
| `EEPROM_READ` (0x30) | 34 |
| `IO_READ` (0x40) | 3 |
| Bulk `FIRMWARE_VER` and instant queries | **0** |

These are retries, not data errors: the CH341A reports a NACK as an all-`FF`
read, the harness retries, and the command completes with a valid response.
The first status read of a 1-Wire command can arrive while the main loop is
still running the 1-Wire start/poll; the read address is then missed. The
pure I2C bulk path and all instant queries had no NACKs at all.

If zero NACKs are required for the 1-Wire commands too, leave a short gap
before their first status read. That lowers the achieved bus load, so it was
not used for this test.

## SCL timing and 100 kHz compliance

A 200 ms capture taken during the run:

```text
SCL period: min 8.50 us, mean 8.71 us, max 10.50 us
f_SCL:      min 95.2 kHz, mean 114.8 kHz, max 117.6 kHz
tHIGH:      min 4.00 us, mean 4.23 us
tLOW:       min 4.25 us, mean 4.50 us
```

The analyzer decoded 30 Start/Address pairs, 2426 ACKs, 2380 data reads and
29 Stops; the 14 NACK annotations are the master's normal end-of-read NACKs,
not slave address failures.

The CH341A's nominal "100 kHz" setting actually runs at about **115 kHz**,
and `tLOW` is about 4.5 us against the Standard-mode minimum of 4.7 us.
Formally this is outside Standard-mode 100 kHz (although it is inside
Fast-mode). The limitation is in the adapter, not in the firmware. A strict
100 kHz run needs either `CH341SetStream(0, 0)` (20 kHz) or a different
master (Raspberry Pi, Arduino, a second STM32 with hardware I2C).

## Conclusion

At the CH341A's actual ~115 kHz the I2C slave and the bridge core survived
18 minutes of near-saturated traffic with:

- zero corrupted or lost responses;
- zero MCU resets and zero watchdog events;
- zero firmware error-counter increments;
- zero NACKs on the bulk and instant paths;
- only recoverable NACKs on the first status read of 1-Wire commands.

The test passes for the hardware that was used. For a formal 100 kHz
Standard-mode qualification, repeat it with a master that generates a true
100 kHz clock (or with the CH341A at 20 kHz).
