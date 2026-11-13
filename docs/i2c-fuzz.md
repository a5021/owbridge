# Host-link fuzzing - 2026-11-14

The deterministic suite (`tools/protocol_test.py`) pins the parser boundary and
the transport corner cases. This note covers the randomized complement: a
layered fuzzer that attacks the host <-> bridge link itself - foreign I2C
addresses, truncated and over-long transfers, stale responses, queue churn and
the destructive configuration/EEPROM paths.

The harness and its logs live outside the repository, in
`C:\temp\owbridge\fuzz_host_bridge.py`. The cases that turned out to be worth
keeping are in `tools/protocol_test.py` as the `transport` phase, so they run
on every regression pass without the fuzzer.

## Layers

| Layer | Input | Oracle |
|-------|-------|--------|
| `L0` wire | foreign addresses, zero/short/long writes, reads of 0/1/2/3/8/40/64/200 bytes, random 0-3 ms gaps | every transaction completes; a valid command answers afterwards; foreign reads are all `FF` |
| `L1` frames | every code, declared/actual length mismatches, boundary lengths, semantic arguments; long and destructive codes only with an invalid length | status is one of the defined values; read-only commands never report `BUS_SHORT`, `TIMEOUT` or `BUS_ERROR` |
| `L2` response | 1-2 byte partial reads, over-reads, aborts, reads with no preceding write | the next full read returns the same frame; the tail of an over-read is `FF`; no TX pointer sticks |
| `L3` sequence | broadcast `TEMP_CONVERT` (750 ms) followed by 0-3 queued commands and `OW_ABORT`, mixed with instant commands | every response matches a known command; the queue drains; the bridge stays readable |
| `L4` destructive | random, corrupted, bad-field and bad-header `CONFIG_SET`; random `EEPROM_WRITE` plus read-back; controlled `RESET` | bad blocks are rejected with the documented status and never reach the flash page; an accepted EEPROM write reads back exactly; the bridge boots after `RESET` |

Shared oracles after each non-destructive layer: `uptime_s` never goes
backwards, the reset counter does not move, and the `bus_short`, `timeout` and
`crc_error` counters do not move. The configuration block and the DS2431 block
are snapshotted before `L4` and restored in a `finally` block, so a failed run
still leaves the bench usable.

## Determinism and replay

`--seed` fixes the action sequence. `--jsonl` records every raw transaction
(address, payload, read length, response), and `--replay` re-runs a log and
compares the responses. `--selftest` injects a failing check and verifies that
the harness reports it, so a green run means the oracle ran, not that it was
silently skipped.

## Deterministic coverage added to the repository

`tools/protocol_test.py` gained a `transport` phase with twelve cases:

1. read without a preceding write returns the latched frame;
2. one-byte header read, then the full frame - the pointer resets;
3. over-read: the bytes past the frame are released as `FF`;
4. a read aborted after two bytes does not disturb the next one;
5. a read address with zero clocks is recovered by the next command;
6. a zero-length write is ignored;
7. a one-byte write does not latch a response;
8. a declared length without its data is `PARAM_ERROR`;
9. a declared length past the command maximum is `PARAM_ERROR`;
10. the maximum valid frame still executes after the rejects;
11. a foreign address is NACKed and does not disturb our frame;
12. a one-byte read (`I2C_TX_END` on the slave side) then a full read.

The phase ends by latching the version response, so the following phases start
from a known frame.

Result: 169 passed, 0 failed, 10 skipped at both 100 kHz and 20 kHz.

## Session results

Each row ran all five layers with destructive mode enabled.

| Speed | Seed | Cases per layer | Raw actions | Result |
|-------|------|-----------------|-------------|--------|
| 100 kHz | 3 | 30 | 136 | pass |
| 100 kHz | 12 | 100 | 448 | pass |
| 20 kHz | 6 | 100 | 446 | pass |

The three runs together logged 1 030 raw transactions. Uptime never went
backwards in a non-destructive layer, the reset counter moved only for the
deliberate `RESET`, and `bus_short`, `timeout` and `crc_error` stayed at zero.

## Findings

The firmware itself produced no failures. The defects the session found were
in the harness, which is the point of `--selftest`:

* `L2` first used a zero-length read to model a partial read. A read address
  with no clocks is host misuse (the slave starts driving SDA on the ACK edge),
  not a firmware defect; the layer now uses 1-2 byte reads, and the recovery
  path is kept as deterministic case 5.
* `L3` first started `TEMP_READ_ALL`, which is short enough to finish before the
  queued commands arrive. It now starts a broadcast `TEMP_CONVERT` (750 ms), so
  the queue is really exercised.
* `L4` compared `CRC_ERROR` against the wrong status value; the status codes now
  come from the same table as the deterministic suite. The CRC case mutates a
  reserved byte, so the field ranges still pass and only the CRC can reject it.

## Limits

* The CH341A DLL reports completion, not the ACK bit, so an address NACK on our
  own address is invisible to `L0`. The retry path is covered by the demo and
  by `tools/protocol_test.py`, not by a fuzz assertion.
* `L4` mutates the header, the field block and the reserved bytes, but never a
  random valid `i2c_addr`: a valid block would be accepted and would move the
  bridge to another address. Only the documented reject paths are fuzzed.
* No soak run: the destructive layer costs one configuration-page erase and a
  few DS2431 writes per run, so it is a correctness tool, not a wear test.
* The harness needs Windows Python and the CH341A adapter and is not shipped in
  the repository.
