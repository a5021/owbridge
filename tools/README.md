# Host-side tools

Host software for the bridge. The first three are the main tools; the
`test_*.py` scripts at the bottom are the older UART-only harnesses.

## Requirements

- `owbridge_demo.py`, `owbridge_scan.py` and `protocol_test.py`: Windows with
  the CH341A driver (`CH341DLLA64.DLL`), Python 3.8+, no third-party packages.
- The older `test_*.py` scripts: Python 3.8+ and `pyserial`, a UART build and a
  serial port (edit `PORT`/`BAUD` at the top of each script; the bench used
  `COM5` at 115200).
- `test_host_helpers.py`: Python 3.8+ only, no hardware.

All scripts live in this directory; run them from here.

## Main tools

| Script | What it does |
|--------|--------------|
| `owbridge_demo.py` | Walks every firmware command over the CH341A (I2C at address `0x30`). `info`, `scan`, `temp`, `watch`, `eeprom`, `pio`, `raw`, `config`, `stats`, `timing`, `queue`, `selftest`. `--json` for machine-readable output. |
| `owbridge_scan.py` | Scan-mode host: one broadcast resolution write, then Convert T + `TEMP_READ_ALL` on a fixed cadence. Reports BUSY-poll NACKs and slots without a valid reading. Supports `--speed 20k`, `--poll-gap`, `--cycles`, `--json`. |
| `protocol_test.py` | Boundary, argument, fuzz and final-health tests for the command descriptor table. `python protocol_test.py --fuzz 10000`, `--speed 20k`, `--destructive`. |

`TEMP_READ_ALL` returns one 2-byte slot per cached DS18B20, in cache order,
with `0x8000` for a sensor that has no valid reading. All three tools use
`owbridge_demo.pair_temperatures()` to map words to ROMs and refuse an array
whose length does not match the cache (pre-1.1.0 firmware compacted failed
reads, which made positions unreliable).

From 1.2.0 the bridge also supports broadcast `TEMP_SET_RES` (one `bits` byte)
and `TEMP_SET_ALARM` (`th`, `tl`) — `owbridge_scan.py` uses the former instead
of hand-building a Skip ROM frame. `OW_MATCH_ROM`, `OW_SKIP_ROM` and
`OW_READ_ROM` append a presence byte; `owbridge_demo.py` prints it for
`readrom`, and the command still returns `OK` when no device answered.

## Host-only unit tests

`python3 -m unittest -v test_host_helpers` checks the positional mapping,
the `0x8000` handling and the scan summary without a board. This is the fast
regression gate for the temperature-slot contract.

## Legacy UART scripts

| Script | What it does |
|--------|--------------|
| `test_uart.py` | Sends the AT command set and prints the responses. |
| `test_binary.py` | Binary protocol: reset, search, get devices, per-device convert/read. |
| `test_all.py` | Broadcast convert + read all temperatures, raw scratchpad of the first DS18B20. |
| `test_power.py` | `TEMP_DETECT_P` then convert/read with parasite on and off. |
| `test_devices.py` | DS2431 memory/scratchpad read, DS2413 PIO write/read, config/stats. |
| `test_timing.py` | Measures the AT command response time. |

The binary UART frame is `AA 55 [cmd][len][data...] [crc8]` and the response is
`5A A5 [status][cmd][len][data...] [crc8]` (Dallas/Maxim CRC-8, poly 0x8C);
the scripts skip `BUSY` responses and wait for the final one. `AT+TEMP` takes
about one second: broadcast Convert T (750 ms at 12 bit) followed by
`TEMP_READ_ALL`, with `N/A` for a sensor without a reading. These scripts only
cover the UART transport; use the binary frame directly for SPI or I2C.
