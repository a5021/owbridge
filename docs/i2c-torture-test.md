# I2C torture test - 2026-11-12

This note records the 60-minute load test of the `HOST_IF=I2C` firmware with
per-command payload validation. It extends the earlier 18-minute saturation
run ([`i2c-stress-test.md`](i2c-stress-test.md)) with invalid-frame bursts,
stale-buffer replay, queue/abort churn and parser fuzz while the bus stays
close to saturation.

## Setup

| Item | Value |
|------|-------|
| Firmware | I2C build at `e764fdb`, 21 812 B text, 2 044 B RAM (bss + data) |
| MCU | STM32F030F4P6, 48 MHz, 1-Wire on PA10 |
| I2C slave | address `0x30`, software slave on PA4/PA5 |
| Master | CH341A, `CH341SetStream(0, 1)` (nominal 100 kHz, measured ~115 kHz) |
| Host | Windows Python 3.15, `CH341DLLA64.DLL` |
| Bench 1-Wire | 7 x DS18B20, 1 x DS2431, 1 x DS2413 (parasite power) |
| Duration | 3601 s (60 minutes) |

The harness lives in `C:\temp\owbridge\`:

- `i2c_torture.py` - workload generator, protocol checker and chaos injector;
- `torture_report.py` - per-phase summary of the JSON log;
- `torture2.log` - one JSON summary per 10 s plus error records;
- `torture2.stdout` - console output.

## Workload

Every phase spends the remaining time in the bulk loop:

1. write the 2-byte `FIRMWARE_VER` frame (`52 00`);
2. read 163 bytes in the same transaction sequence.

At ~115 kHz one round keeps SCL busy for about 13.6 ms; the bulk loop occupied
roughly 95% of wall time. The response is validated on every round: the first
29 bytes must equal `00 52 1A "owbridge 1.0.0 (lib 3.1.2)"` and the remaining
bytes must be `FF`.

| Phase | Duration | Additional workload |
|-------|----------|---------------------|
| 1 | 20 min | pure saturation: statistics every 10 s, instant queries every 20 s, reset every 30 s, search+EEPROM+PIO every 60 s, temperature every 75 s |
| 2 | 20 min | phase 1 plus, every 20 s, a 12-frame invalid burst (unknown codes, `len=0`, short `MATCH_ROM`/`TEMP_READ`, truncated and padded frames, `dlen` 41/255, zero ROM); every 30 s a stale-buffer replay; every 45 s a queue/abort scenario |
| 3 | 20 min | phase 2 plus, every 25 s, a valid-command sweep (`TEMP_READ`, `GET_RES`, `GET_ALARM`, `READ_BYTES 32`, `WRITE_BYTES 32`, `MATCH_ROM`, `SKIP_ROM`, clear cache + search, abort) and, every 40 s, 100 random parser frames |

The queue/abort scenario is the interesting one: a broadcast Convert T runs
for 750 ms, then `OW_GET_COUNT` and `OW_GET_STATUS` fill both queue slots, a
third command must answer `QUEUE_FULL`, an invalid `OW_WRITE_BYTE` must answer
`PARAM_ERROR` immediately, and `OW_ABORT` must clear the queue and finish with
`ABORTED`.

Every response is checked for status, command echo, length and payload
(version string, Dallas CRC-8 of every ROM, config magic/version/CRC-32,
stable EEPROM content, temperature range). A firmware reset is detected when
the `STATS_GET` uptime decreases.

## Results

| Metric | Value |
|--------|-------|
| Duration | 3601 s |
| Transactions | 776 677 |
| Bulk rounds (163-byte reads) | 252 646 |
| Bad bulk responses | 0 |
| Bytes read | 42 002 978 |
| Bytes written | 551 368 |
| Average read rate | 11.66 KB/s |
| Bulk rounds per second | 70.15 |
| Bulk latency min / avg / max | 13.377 / 13.607 / 52.242 ms |
| Validation errors | 0 |
| MCU resets | 0 |
| USB errors | 0 |
| Host write retries | 18 |

Per phase:

| Phase | Seconds | Bulk rounds | Read B/s | Tx/s | Errors | NACKs |
|-------|---------|-------------|----------|------|--------|-------|
| 1 | 1192 | 85 101 | 11 780.7 | 189.5 | 0 | 3 713 |
| 2 | 1201 | 84 689 | 11 679.4 | 201.5 | 0 | 4 555 |
| 3 | 1209 | 82 856 | 11 530.0 | 255.5 | 0 | 6 800 |

The 52.2 ms maximum is a single USB/scheduler outlier; the next values return
to the 13.4-14.7 ms range and no transaction was lost.

## Firmware counters

Final `STATS_GET`:

```text
OW_RESET count = 451   presence_fail = 0   crc_error = 0   bus_short = 0
timeout = 0            commands = 524 260  aborts = 0      uptime = 8295 s
```

`OW_RESET count` is the number of reset commands the harness issued, not an
MCU reset counter. Uptime grew from 4674 s to 8273 s across the 3601 s run, so
the MCU never reset and the independent watchdog never fired. All error
counters stayed at zero.

The device cache stayed stable: every search returned 7 x `0x28`, 1 x `0x2D`
and 1 x `0x3A`, all with valid ROM CRCs. Final temperatures were 26.5-26.9 C.

## NACK analysis

The harness saw 15 068 NACKs, all on 1-Wire or memory commands:

| Command | NACKs |
|---------|-------|
| `TEMP_CONVERT` (0x20) | 6 157 |
| `OW_SEARCH` (0x09) | 4 961 |
| `TEMP_READ_ALL` (0x22) | 1 755 |
| `EEPROM_READ` (0x30) | 311 |
| `EEPROM_READ_S` (0x32) | 190 |
| `OW_WRITE_BYTES` (0x04) | 128 |
| `TEMP_READ`/`GET_RES`/`GET_ALARM` | 162 |
| other classified 1-Wire commands | 30 |
| uncategorised chaos/read retries | 1 374 |
| Bulk `FIRMWARE_VER` and instant queries | **0** |

The 13 694 classified NACKs are the per-command counts collected by
`transact()`; the remaining 1 374 come from the chaos paths (rejected frames,
queue reads and one-byte-frame probes), which count the retry without
attributing it to a command.

These are retries, not data errors: the CH341A reports a NACK as an all-`FF`
read, the harness retries, and the command completes with a valid response.
The pure I2C bulk path and all instant queries had no NACKs at all.

The NACKs on long commands have a firmware-side mechanism, not just an adapter
quirk: while a command is `BUSY`, the main loop skips `bridge_poll()` between
START and STOP, then runs one library step after STOP (or builds the next
command's slots inside the STOP handler). Those windows are 45-93 us on this
bench (see `docs/i2c-timing.md`), so a START chained ~15 us after STOP is
missed and the read address is NACKed. A focused rerun on the final firmware
measured 3 827 retries over 120 long commands at 100 kHz and **0** at 20 kHz;
every command still completed. The earlier explanation that the adapter alone
caused these NACKs was not proven.

## Host write losses

The CH341A also reports success when the slave NACKs the *write* address
phase. During the queue churn the next frame is written immediately after a
header read, and about 3% of those back-to-back writes were lost that way: the
latched response then belongs to the previous command. A focused probe saw
6 losses in 180 writes; over the whole run the harness counted 18 lost writes
and resent each one, and every command completed.

This is the same class of transient as the read NACKs: a write that arrives
while the firmware is still completing a poll step is not sampled from its
first START, so the slave does not see the frame. `Bridge.command()` already
handles it in the demo: if the response does not echo the command code, the
write was not accepted and is sent again (up to three times). The lost write
never reached `bridge_submit()`, so no queue slot, counter or response was
consumed.

The first version of the torture harness did not resend and reported 17
queue-state mismatches in the 60-minute run; adding the echo check and a
resend removed all of them.

## Comparison with the 18-minute run

| Metric | 18 min (`fd86f39`) | 60 min torture (`e764fdb`) |
|--------|--------------------|----------------------------|
| Bulk rounds | 77 286 | 252 646 |
| Transactions | 202 777 | 776 677 |
| Bytes read | 12 747 279 | 42 002 978 |
| Read rate | 11.80 KB/s | 11.66 KB/s |
| Bad bulk responses | 0 | 0 |
| Validation errors | 0 | 0 |
| MCU resets | 0 | 0 |
| Bulk latency max | 54.9 ms | 52.2 ms |

The lower read rate comes from the extra command families and chaos tasks; the
bulk loop itself is unchanged.

## Conclusion

The I2C build with per-command validation survived 60 minutes of
near-saturated traffic and aggressive malformed-frame injection with:

- zero corrupted or lost responses, including 42.0 MB of bulk reads;
- zero MCU resets and zero watchdog events;
- zero firmware error-counter increments;
- zero NACKs on the bulk and instant paths;
- correct `QUEUE_FULL`, `PARAM_ERROR` and `ABORTED` behaviour under queue
  churn while a conversion was running;
- stable device cache, ROM CRCs and EEPROM contents.

The only recoverable transport effect was the write-address NACK on
back-to-back writes (a START that arrived while the firmware was completing a
long-command poll step), which the host handles by resending the frame.
