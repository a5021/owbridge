# Host demo

`tools/owbridge_demo.py` is a self-contained demo for the I2C build of the
firmware. It speaks the bridge protocol over a CH341A USB-I2C adapter and walks
through every command family: raw 1-Wire, device search, DS18B20 temperature,
DS2431 EEPROM, DS2413 PIO, configuration, statistics, queueing, abort and
latency/load measurement.

The script is deliberately stdlib-only: it loads `CH341DLLA64.DLL` through
`ctypes`, so it runs with any Python 3.8+ on Windows without installing
packages. Every scenario prints a human-readable report and returns a
dictionary; `--json` prints only the dictionary, which makes the same commands
usable from scripts.

`tools/protocol_test.py` reuses that transport for protocol conformance: it
sends every command at its minimum, maximum and out-of-range payload lengths,
checks the semantic argument rules, replays stale buffer contents, and can
fuzz the parser. It is described under
[protocol tests](#protocol_testpy) below.

```text
python owbridge_demo.py info
python owbridge_demo.py scan
python owbridge_demo.py temp --json
python owbridge_demo.py watch --interval 1
python owbridge_demo.py eeprom --dump
python owbridge_demo.py eeprom --read 0x10:8
python owbridge_demo.py eeprom --write 0x20:DE,AD,BE,EF --yes
python owbridge_demo.py io --set A=0,B=1
python owbridge_demo.py raw 0x09 --data F0,00,10
python owbridge_demo.py queue
python owbridge_demo.py abort
python owbridge_demo.py timing --all
python owbridge_demo.py load --seconds 10
python owbridge_demo.py selftest
python protocol_test.py
python protocol_test.py --fuzz 10000
python protocol_test.py --destructive
```

Global options are accepted before or after the scenario name: `--addr`
(default `0x30`), `--speed` (`20k` or `100k`), `--dll`, `--timeout`, `--json`,
`--verbose` (print every I2C frame) and `--no-color`.

## Transport behaviour

Each command is a write of `cmd, len, data...` followed by reads of the
three-byte response header (`status, cmd, len`). While the status is `BUSY` the
header is polled again; when it is not, the data bytes are read in a second
transaction. Two adapter quirks shape the transport:

* `CH341StreamI2C()` returns success even when the slave NACKs the address
  phase. An unacknowledged read therefore appears as `FF FF ...`, which the
  demo treats as "not ready" and polls again.
* The bridge echoes the command byte in every response, including `BUSY` and
  `QUEUE_FULL`. If the echoed byte does not match the request, the write was
  lost; the demo re-sends it (up to three times) instead of returning a stale
  frame.
* While a long command is `BUSY`, a START can land inside a `bridge_poll()`
  step (10-50 us) and be missed. A logic-analyzer capture at 100 kHz shows
  these misses at the poll cadence (median 96 us after the previous STOP),
  not at the 15 us chained read that follows STOP. The demo's retry rules
  above absorb them. A host that needs every frame accepted on the first try
  must leave at least 100 us after STOP (250 us is comfortable), or use UART,
  whose frames carry a CRC-8. `CONFIG_SET` blocks for ~23 ms and always needs
  the retry rule.

The I2C frame has no checksum; see the transport integrity table in the
README. Commands with self-checking payloads (ROM CRC-8, DS2431 CRC-16,
configuration CRC-32) still catch corruption end to end.

The adapter is used at the rate it actually produces. `CH341SetStream(0, 1)` is
nominal 100 kHz and measures about 115 kHz on the bench; the polled software
slave is validated at that rate. 400 kHz and 750 kHz are not offered. The 20 kHz
option remains usable for very long frames: the safety valve now waits for
50 ms *without bus edges*, so a 163-byte frame taking ~74 ms is not interrupted.

## Scenarios

| Scenario | What it shows |
|----------|---------------|
| `info` | firmware version, the 40-byte configuration block with CRC-32, bus status and the 32-byte statistics block |
| `scan` | reset, full search, cached ROM list with family names, CRC-8 check and Dallas labels |
| `readrom` | `0x33` read ROM, including the warning that it is only defined for a single-device bus |
| `temp` | broadcast Convert T, `TEMP_READ_ALL`, then per-sensor resolution, TH/TL and parasite flag |
| `watch` | repeated conversions in a live table until Ctrl+C (or `--count`) |
| `eeprom` | 128-byte dump in 8-byte passes, range read, and read-modify-write of one 8-byte block |
| `io` | DS2413 pin/latch read and optional latch write |
| `raw` | any command byte with an arbitrary payload, decoded header and hex dump |
| `queue` | a long command with an instant command queued behind it, then a deliberate `QUEUE_FULL` |
| `abort` | `OW_ABORT` dropping the queued command and reporting `ABORTED` |
| `timing` | latency distribution and poll counts for instant and long commands |
| `load` | the stress-harness workload: write + 163-byte response reads for N seconds |
| `selftest` | version, config CRC, presence, search, ROM CRCs, temperature, EEPROM and PIO in one run |

## Captured session

All output below is from the 9-device bench (7 x DS18B20, 1 x DS2431,
1 x DS2413) at address `0x30`, 100 kHz, with colours disabled.

### `info`

```text
owbridge demo  addr 0x30  100k  C:\Windows\System32\CH341DLLA64.DLL

Firmware
--------
  version            owbridge 1.0.0 (lib 3.1.2)

Configuration (flash page 0x08007C00)
-------------------------------------
  magic              0x4F574252 (valid)
  version            0x0100
  i2c address        0x30
  uart baud index    4
  timing preset      0
  parasite default   yes
  strong pull-up gate yes
  LEDs               enabled
  crc32              0x98FD0B7C (valid)

Bus status
----------
  busy               yes
  power mode         parasite
  cached devices     9
  flags              0x09
  BUSY is set because OW_GET_STATUS itself is still the command in flight.

Statistics
----------
  commands           1465
  resets             22
  presence_fail      0
  crc_error          0
  bus_short          0
  timeout            0
  aborts             0
  uptime             604 s (10 min 4 s)
```

`OW_GET_STATUS` is an instant command, so it runs while its own command is in
flight and always reports `BUSY = 1`. The cached-device count is the useful
field; the flag is explained in the output.

### `scan`

```text
1-Wire bus
----------
  presence           yes
  search found       9
  cache count        9

Devices
-------
  #  ROM (bus order)          family        crc  Dallas label
  -  -----------------------  ------------  ---  -----------------
  0  28 20 78 92 07 00 00 67  0x28 DS18B20  ok   28-00000792782067
  1  28 78 B8 AC 0B 00 00 2C  0x28 DS18B20  ok   28-00000BACB8782C
  2  28 64 69 AB 0B 00 00 1F  0x28 DS18B20  ok   28-00000BAB69641F
  3  28 FC AE AA 0B 00 00 F3  0x28 DS18B20  ok   28-00000BAAAEFCF3
  4  28 C6 BB 4E 04 00 00 67  0x28 DS18B20  ok   28-0000044EBBC667
  5  28 7E 63 AD 0B 00 00 3F  0x28 DS18B20  ok   28-00000BAD637E3F
  6  28 F1 39 AD 0B 00 00 D9  0x28 DS18B20  ok   28-00000BAD39F1D9
  7  3A 9F 47 03 00 00 00 3A  0x3A DS2413   ok   3A-00000003479F3A
  8  2D F8 47 60 26 00 00 69  0x2D DS2431   ok   2D-0000266047F869
```

### `temp`

```text
Convert T (broadcast)
---------------------
  status             OK
  busy polls         1691
  elapsed            0.771 s

Read all scratchpads
--------------------
  values             7

Temperatures
------------
  ROM                C        resolution                     TH/TL      parasite
  -----------------  -------  -----------------------------  ---------  --------
  28-00000792782067  22.9000  12 bit (0.1 C/bit, 750 ms)  +25 / +15  yes
  28-00000BACB8782C  23.0000  12 bit (0.1 C/bit, 750 ms)  +75 / +70  yes
  28-00000BAB69641F  23.0000  12 bit (0.1 C/bit, 750 ms)  +25 / +15  yes
  28-00000BAAAEFCF3  23.1000  12 bit (0.1 C/bit, 750 ms)  +75 / +70  yes
  28-0000044EBBC667  22.8000  12 bit (0.1 C/bit, 750 ms)  +75 / +70  yes
  28-00000BAD637E3F  23.0000  12 bit (0.1 C/bit, 750 ms)  +75 / +70  yes
  28-00000BAD39F1D9  22.9000  12 bit (0.1 C/bit, 750 ms)  +75 / +70  yes
```

`TEMP_READ` and `TEMP_READ_ALL` return the library value unchanged:
signed 16-bit temperature in **0.1 C units** (`0x00E5` = 229 = 22.9 C), not
the raw 1/16 C scratchpad word. The demo decodes it with `value / 10`; the
raw scratchpad register (e.g. `0x016E` = 366) is `366 / 16 = 22.875 C`, which
matches.

A broadcast conversion is followed by a scratchpad read from every sensor. On a
multi-drop bus those reads collide, so the driver reports a CRC failure and the
firmware maps the broadcast case to `OK`: the conversion itself did complete.
The `TH/TL` values are whatever the devices currently hold; they differ because
they were configured at different times.

### `eeprom`

```text
EEPROM dump (128 bytes)
-----------------------
  ROM                2D-0000266047F869
  read in            0.491 s (128 bytes in 8-byte passes)
  0000  F4 C3 D4 05 17 18 19 1A 00 00 00 00 00 00 00 00  ................
  0010  54 02 03 13 1B 07 05 82 4C 02 03 02 19 07 00 4F  T.......L......O
  0020  28 7E 63 AD 0B 00 00 3F 28 20 78 92 07 00 00 67  (~c....?( x....g
  0030  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  ................
  ...

Scratchpad
----------
  frame              78 00 07 00 00 00 00 00 00 00 00
```

`EEPROM_READ` returns at most 8 bytes per command, so the dump is 16 passes.
The DS2431 programs one 8-byte block at a time and refuses a short block (the
device sets PF and Copy Scratchpad declines it). `--write` therefore reads the
current block, merges the requested bytes and writes all eight:

```text
Write EEPROM
------------
  ROM                2D-0000266047F869
  block              0x70..0x77
  was                00 00 00 00 00 00 00 00
  write              00 00 AB CD 00 00 00 00
  read back          00 00 AB CD 00 00 00 00
  status             verified
```

### `io`

```text
PIO state
---------
  pins               A=0 B=0
  latches            A=1 B=1
  Pins A/B differ from the latch: driven externally.
```

`--set A=0,B=1` writes both latches; the read afterwards shows the actual pin
levels, which differ when something external holds a pin.

### `queue`

```text
Command queue (depth 2)
-----------------------
  Submit a 750 ms broadcast Convert T, then OW_GET_COUNT while it runs.
  A queued command reports BUSY with its own command code in the header.
  t+ms   status  cmd   name          data
  -----  ------  ----  ------------  ----
  0.7    BUSY    0x0F  OW_GET_COUNT  -
  761.1  OK      0x0F  OW_GET_COUNT  09

Queue full
----------
  rejected           0x0F OW_GET_COUNT (depth 2 is already occupied)
```

The bridge has one response slot. Convert T's own response is overwritten by
the queued command as soon as that command completes, before the host can read
it; the host identifies responses by the echoed command byte. Two commands fit
in the queue, the third is rejected with `QUEUE_FULL`.

### `timing`

```text
Command latency (20 samples each)
---------------------------------
  command                  status  min ms  avg ms   p95 ms   max ms   polls
  -----------------------  ------  ------  -------  -------  -------  -----
  FIRMWARE_VER (instant)   OK      3.221   3.266    3.322    3.322    1
  OW_GET_COUNT (instant)   OK      1.187   1.265    1.351    1.351    1
  OW_GET_STATUS (instant)  OK      1.243   1.292    1.43     1.43     1
  CONFIG_GET (instant)     OK      4.359   4.416    4.462    4.462    1
  OW_RESET                 OK      2.349   2.651    2.836    2.836    3.9
  OW_READ_BYTE             OK      2.514   2.852    3.051    3.051    4.7
  TEMP_READ_ALL            OK      188.14  189.111  190.382  190.382  417.5
```

These are host-side numbers and are dominated by USB and the CH341A's
transaction turnaround; the firmware's own STOP-to-response path is much
shorter (see `docs/i2c-timing.md`). Instant commands need one poll; long ones
keep `BUSY` and are polled.

### `load`

```text
Bus load (5 s)
--------------
  rounds             371
  throughput         74.0 rounds/s, 12.1 KB/s
  bad responses      0
  firmware commands  2037
  firmware errors    0
```

### `selftest`

```text
Self test
---------
  PASS firmware version       owbridge 1.0.0 (lib 3.1.2)
  PASS config crc             ok
  PASS bus presence           present
  PASS search                 9 devices
  PASS rom crc                9 valid
  PASS temperature            22.90 C, 23.00 C, 23.10 C, 23.10 C, 22.80 C, 23.10 C, 23.00 C
  PASS eeprom                 8 bytes
  PASS pio                    pins=0x00 latches=0x03

  RESULT: PASS
```

### `protocol_test.py`

Boundary matrix, argument checks, stale-buffer replay and a 10 000-frame fuzz
run on the 9-device bench. Only the first and last lines are shown; the full
log is kept outside the repository.

```text
owbridge protocol test  addr 0x30  100k  fuzz=10000
baseline: 9 devices, 7 DS18B20
PASS  unknown    cmd=0x00 len=0  [INVALID_CMD cmd=0x00 len=0]
PASS  matrix     OW_WRITE_BYTE len=0  [PARAM_ERROR cmd=0x02 len=0]
PASS  matrix     OW_WRITE_BYTE len=1  [OK cmd=0x02 len=0]
PASS  matrix     OW_MATCH_ROM len=7  [PARAM_ERROR cmd=0x06 len=0]
PASS  matrix     TEMP_CONVERT len=7  [PARAM_ERROR cmd=0x20 len=0]
PASS  args       TEMP_READ rom[0]=0  [PARAM_ERROR cmd=0x21 len=0]
PASS  structure  1-byte frame is unanswered  [PARAM_ERROR/0x20 -> PARAM_ERROR/0x20]
PASS  structure  padded frame (dlen=0, 4 extra)  [PARAM_ERROR cmd=0x0F len=0]
PASS  stale      rejected frames never touch the bus  [commands 100 -> 101 (delta 1)]
PASS  stale      bus alive after rejects  [reset present=True]
PASS  fuzz       10000 frames accepted without bus errors
PASS  fuzz       uptime kept running  [63 -> 76 s]
PASS  verify     device count unchanged  [search=9 cached=9 baseline=9]
PASS  verify     temperature array intact  [7 values, baseline 7]

RESULT: PASS  (157 passed, 0 failed, 10 skipped)
```

The ten skips are valid payloads the tool deliberately does not execute:
`RESET`, `STATS_RESET`, `CONFIG_SET`, `EEPROM_WRITE`, `TEMP_COPY`,
`TEMP_RECALL`, `IO_WRITE`, and the two power/pull-up commands. Their
out-of-range lengths are still checked; the destructive write paths run only
with `--destructive`.

## Quirks the demo exposes

* `OW_READ_ROM` (`0x33`) only makes sense with one device on the bus. On the
  nine-device bench every sensor answers at once; the demo says so and marks
  the result as a collision pattern rather than a ROM.
* `OW_GET_STATUS` reports `BUSY = 1` because the status command itself is the
  operation in flight.
* `OW_ABORT` cannot cancel an in-flight hardware operation. It drops the queued
  commands and reports `ABORTED` after the running one finishes.
* `OW_READ_BYTES` / `OW_WRITE_BYTES` are limited to 32 bytes by the engine, and
  `EEPROM_READ` to 8 bytes by the DS2431 pass size.
* The DS18B20 driver parks in parasite mode with the strong pull-up engaged.
  On this port that means PA10 is push-pull, which breaks any following search
  until it is released. The bridge now restores the open-drain line before
  every command unless `OW_SET_PULLUP` asked to hold it.
* `stats_uptime()` only advanced while the I2C bus was busy for more than
  50 ms. The main loop now calls `board_millis()` unconditionally, so the
  counter follows real time even when the bus is idle.
* Payload lengths are validated per command before queueing. An all-zero ROM
  on an addressed command is `PARAM_ERROR` rather than a silent broadcast, and
  `OW_MATCH_ROM` no longer underflows on a payload shorter than eight bytes.
