# Optimization report: the 100 kHz service window

This page records one optimization pass over the bridge firmware and the
`stm32-async-1wire` library: what was measured, what was changed, what was
rejected and why, and what remains. Every number comes from the 9-device
bench (STM32F030F4 at 48 MHz, CH341A host adapter, 7 x DS18B20, one DS2413,
one DS2431). Nothing here is an estimate unless it says so.

## Method

- **TIM3 profiler** (`-DOWBRIDGE_PROFILE=ON`, results read over SWD). The
  instrumented loop cannot service 100 kHz, so the CPU timings are taken at
  20 kHz; CPU cost is independent of the SCL rate.
- **`OW_PROFILE` hooks**: the profiled build defines `OW_PROFILE=1` for the
  library, and `src/profile.c` provides the work-event hooks plus a search
  sink counter, so a long poll can be attributed to the driver or to the
  application callback.
- **Logic analyzer**: Saleae Logic (fx2lafw, 4 MHz) on SCL (PA4), SDA (PA5)
  and DQ (PA10). The `.sr`/binary captures and the decode scripts live in the
  bench directory (`C:\temp\owbridge`), not in this repository.
- **`nack_probe.py`**: 30 searches, 40 `TEMP_READ_ALL`, 40 32-byte writes and
  10 EEPROM writes (120 long commands), counting the retries the host needs.
- **Protocol suite**: `protocol_test.py --speed 100k --fuzz 2000` and
  `--destructive`, plus the same at 20 kHz.

## Baseline (library v3.1.2, firmware before this work)

| Metric | Value |
|---|---|
| idle `bridge_poll()` | 1-3 us |
| `bridge_poll()` during search / read-all | 45 us |
| `bridge_poll()` during EEPROM read-back | 89 us |
| `bridge_submit()` for a 32-byte write | 93 us |
| `config_set()` inside a poll | 23.3 ms |
| retries at 100 kHz, 120 long commands | 3 827 (0 failed) |
| retries at 20 kHz | 0 |
| address NACK rate (LA, one search) | 4.2% |
| SCL half-period | 4.25 us (about 118 kHz) |
| STOP to chained START | 15 us |
| STOP to next poll START while BUSY | median 108 us |

## Library: PR #121 (table-driven codec)

Four small functions in `src/onewire.c`, no API change, output identical
(the host suite compares each against a bit-wise reference for every byte
value):

- `onewire_crc8()`, `onewire_crc16()`: 16-entry reflected nibble tables, two
  lookups per byte instead of eight shift/XOR rounds.
- `onewire_encode_byte()`: a 16x4 table of pre-encoded slots, two row copies
  per byte.
- `onewire_decode_pulses()`: four captured slots per step with one packed
  compare, exact for every 8-bit capture value.

Measured on the release profile (`-Os`, Cortex-M0, Arm GNU Toolchain 15.2):

| | before | after |
|---|---:|---:|
| CRC-8 instructions per byte | ~96 | ~20 (4.8x) |
| CRC-16 instructions per byte | ~96 | ~22 (4.4x) |
| encoder instructions per byte | ~88 | 26 (3.4x) |
| decoder instructions per byte | ~112 | ~55 (2.0x) |
| whole `src/onewire.c` object | - | 444 B smaller |

F030 whole-image budget: `0_dma_pressure` +100 B, `6_statistics` +148 B,
`8_ds2431` -456 B, `9_multi` -460 B, H5 consumer template +180 B; no `.bss`
movement.

Effect on the bench:

| window | before | after |
|---|---:|---:|
| search poll | 45 us | 27 us |
| read-all poll | 45 us | 25 us |
| EEPROM poll | 89 us | 50 us |
| 32-byte write submit | 93 us | 30 us |
| retries at 100 kHz | 3 827 | about 2 900 |

PR status: <https://github.com/a5021/stm32-async-1wire/pull/121>. All checks
pass except `cmake (h7)`, which fails in the dependency download (the pinned
`cmsis_device_h7` tree is not readable from the runner; a local shallow fetch
of the same pin works). That failure is independent of the diff.

## Firmware

### Packed sampler input

PA4 and PA5 are adjacent, so `host_i2c_soft_poll()` keeps the two lines in
one 2-bit field: SCL is bit 0, SDA bit 1. The no-change fast path is one
shift, one mask and one compare instead of two of each, and the edge handlers
use one XOR.

| Metric | before | after |
|---|---:|---:|
| retries at 100 kHz (120 commands) | ~2 930 | 2 227 / 2 345 |
| address NACK rate (LA, one search) | 4.2% | 1.9% |
| retries at 20 kHz | 0 | 0 |

This was tried earlier mixed with a poll API change and looked like a
regression. The isolated version is a clear win and is what shipped.

### Search cache comparison

`cache_add()` called `memcmp()` once per cached ROM. With nine devices the
search sink took 10 us. Two 32-bit word compares (the cached rows are
word-aligned; the incoming ROM is packed once) bring it to 4 us. The
device-found step is the longest single poll, but it runs once per device
while the per-bit steps run 64 times per device, so this is real but does not
move the end-to-end count.

### Bus state after a master NACK

When the master NACKs the last read byte, the slave used to go idle on the
ACK clock, a few microseconds before the STOP arrives. The `I2C_TX_END` state
keeps the interface busy until STOP or a repeated START. It closes the window
in which the main loop could run a 1-Wire step and miss the STOP edge. No
interrupt, no protocol change; measured neutral on retries (2 934).

### Profiler hooks

The profiled build now enables the library's `OW_PROFILE` work-event hooks and
adds a search-sink counter and a search-call counter to `src/profile.c`. The
hooks compile to nothing in production. They were what showed that the
27-30 us "search poll" was partly boot-experiment pollution, that the sink was
10 us, and that the per-bit arming dominates the total.

## Host tooling

- **`--short-poll`** (`owbridge_demo.py`, also in `protocol_test.py`): while a
  command is BUSY, read only the status byte instead of the 3-byte header;
  re-read the full header (and validate the command echo) once the status
  leaves BUSY. The first poll of every command is still a full header, so
  instant commands are unchanged.

  | command / rate | 3-byte polls | status-byte polls |
  |---|---:|---:|
  | search, 20 kHz | 1 187 ms | 635-672 ms (-45%), 0 retries |
  | search, 100 kHz | 404 ms | 325 ms (-20%), +13% retries |
  | read-all, 100 kHz | 190 ms | 188 ms (1-Wire bound) |

  On by default at 20 kHz; off at 100 kHz (use `--short-poll on` there).
- **`--poll-gap`** (default 0): before the packed sampler it cut the retry
  count from 2 746 to 2 081 over 120 long commands at 100 kHz. With the
  sampler at about 2 286 back-to-back, 250 us measures 2 205 (inside the
  run-to-run spread) and costs about 7% of the search time, so the default is
  back-to-back; the knob remains for hosts that prefer it.

## Rejected experiments

| Experiment | Result | Verdict |
|---|---|---|
| Split the capture decode across polls (4 bytes each) | log windows shorter (32-byte read 70 -> 13 us) but retries 2 746 -> 3 899 | total time inside `bridge_poll()`, not the max step, sets the loss rate |
| Defer long-command start out of the STOP handler | selftest fine, LA address NACKs 7.7% versus 4.2% | made it worse |
| Quiet window after STOP (20 us / capped) | 16.4% address NACKs | made it worse |
| Move the search code to RAM (`.data`, `-mlong-calls`) | per-bit arm 7 -> 8 us, +1.1 KB RAM | no gain; flash fetch is not the cost |
| Cache constant timer/DMA registers in `ow_port_write_then_read` | 8 cacheable writes out of ~25, about 0.33 us of a ~5 us arm; 576 arms = about 0.19 ms per 380 ms search (0.05%) | below noise; hidden state and cross-family DMA differences |
| CRC-8 with a 256-byte table | saves about 2 us per CRC, about 18 us per search (0.005%), +240 B flash | not worth the budget |
| Replace the search leaf `memcmp`/`memcpy` with byte loops | about 36 us per search (0.01%); GCC may turn the copy back into `memcpy` | not a hot path |
| Skip `strong_pullup(0)`/`set_pin_mode(0)` when already open-drain | about 0.1 us per command | the library calls it internally; not safely trackable |
| EXTI/interrupt-driven slave | would remove the sampling margin | excluded by the firmware's interrupt-free requirement |

## Where the time still goes

The 1-Wire search is host-bound, not bus-bound: the pure bus time for nine
devices is roughly 130 ms, while the command takes about 380 ms at 100 kHz.
The bridge skips `bridge_poll()` for the whole I2C transaction (the slave must
sample SCL every 4.3 us), so the state machine advances mostly between host
transactions. The remaining levers are therefore the host's polling shape
(done: status-byte polls at 20 kHz, optional at 100 kHz) or a different slave
architecture (hardware I2C peripheral), which the interrupt-free requirement
excludes.

The host contract is unchanged: while a long command is BUSY, either leave at
least 100 us after STOP (250 us is comfortable) or treat a NACK / all-`FF` read
as "not ready" and retry. `CONFIG_SET` blocks for about 23 ms and always needs
the retry rule.

## Commits

Bridge (`a5021/owbridge`): `514a164` LA measurements, `cd8d7bc` slicing
rejected, `599a70b` cache comparison and profiler hooks, `358b86e` poll gap,
`e2120af` gap by bus rate, `7c0a051` NACK->STOP bus state, `a0e1198` short
status polls, `296700f` packed sampler, `e5f2620` gap default 0.

Library (`a5021/stm32-async-1wire`, PR #121): `d88bb50` packed decode,
`8228e1a` CRC/encoder tables, `b6c4cc0` F030 budgets, `353bde8` H5 consumer
budget.
