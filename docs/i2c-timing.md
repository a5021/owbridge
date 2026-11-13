# Software I2C timing margin

This note records the one-off measurement of the software I2C slave on the
STM32F030F4P6. It answers one question: can the polled main loop observe SCL
and SDA often enough for a reliable 100 kHz bus?

## Method

- Build: `-DHOST_IF=I2C -DOW_OPT=-O2 -DOWBRIDGE_PROFILE=ON`.
- Timer: TIM3 at 1 MHz (16-bit, wrap handled in software).
- The profiler measures three intervals on every main-loop iteration:
  `host_i2c_soft_poll()`, `bridge_poll()`, and the full loop.
- The extended profiler also records a log2 histogram of every
  `bridge_poll()` duration, the longest gap between two polls, the duration of
  `bridge_submit()`, the time spent in `config_set()`, and how often the I2C
  bus went from idle to busy across one poll (a START that arrived while the
  firmware was inside `bridge_poll()`).
- The active-path numbers below come from a real host workload (searches,
  read-alls, 32-byte writes, EEPROM and config writes) at 20 kHz; the
  instrumented loop is too slow for 100 kHz. CPU cost is independent of SCL
  rate.
- After one second the firmware runs a one-off command experiment (reset,
  search, broadcast convert, read-all) and stores the wall-clock durations.
- The values are read from RAM over SWD; nothing is printed. No I2C master and
  no jumper wires are required for this measurement.

The profiler itself adds two TIM3 reads plus volatile bookkeeping on every
iteration (roughly 1.5-2 us), so the absolute loop figures are pessimistic.

## Results

Bench: 9 x 1-Wire devices (7 x DS18B20, 1 x DS2431, 1 x DS2413), I2C build,
`-O2`. The results are from one run of 355,052 loop iterations.

| Metric | min | max | average |
|--------|-----|-----|---------|
| `host_i2c_soft_poll()` | 0 us | 1 us | 0.90 us |
| `bridge_poll()` | 1 us | 2 us | 1.17 us |
| main loop | 4 us | 5 us | 4.09 us |

Command wall-clock times:

| Command | Time |
|---------|------|
| `OW_RESET` | < 1 ms |
| `OW_SEARCH` (9 devices) | 312 ms |
| `TEMP_CONVERT` (broadcast) | 770 ms |
| `TEMP_READ_ALL` | 184 ms |

The command times are 1-Wire protocol and DS18B20 conversion time, not CPU
time.

## Interpretation

- At 100 kHz the SCL half-period is 5 us. The production loop without the
  profiler is about 2-3 us, so the slave samples each half-period at least
  once. The CH341A actually clocks at about 115 kHz (4.3 us), so the margin
  is small but real; adding a few microseconds to the busy loop breaks it.
- While an I2C transaction is active, `bridge_poll()` is skipped, so the busy
  loop is shorter than the figures above. The safety valve releases the poll
  only after 50 ms with no SCL/SDA edge, so it fires on a wedged bus, never in
  the middle of an active frame (a 163-byte frame at 20 kHz lasts ~74 ms but
  keeps clocking).
- 400 kHz would need a 1.25 us half-period and is not supported.

## Idle versus active bridge_poll

The `bridge_poll()` numbers above are the **idle** case: the poll returns in
1-2 us while no command is running. They do not bound the cost while a long
command is in flight.

The I2C main loop deliberately skips `bridge_poll()` between START and STOP,
so a command advances only after STOP. The first poll after STOP, or the STOP
handler itself (`bridge_submit()`), can therefore block the sampler. Measured
on the 9-device bench with the extended profiler, final firmware at `-O2`:

| Path | Min | Max |
|------|-----|-----|
| idle `bridge_poll()` | 1 us | 3 us |
| `bridge_poll()` during `OW_SEARCH` / `TEMP_READ_ALL` | 1 us | 45 us |
| `bridge_poll()` during `EEPROM_WRITE` (DS2431 read-back) | - | 89 us |
| `bridge_submit()` inside STOP, `OW_WRITE_BYTES` 32 B | - | 93 us |
| `bridge_submit()` other long commands | 15 us | 19 us |
| `config_set()` inside `bridge_poll()` (`CONFIG_SET`) | - | 23.3 ms |

Method note: the profiled build itself cannot keep up with 100 kHz once the
extended counters are compiled in, so the host traffic ran at 20 kHz. The CPU
cost above is independent of the SCL rate; only the number of polls per command
changes.

The library CPU profile's public-poll figures (7 217 cycles for a measurement
phase, 3 356 for search) include that example's UART completion callback. The
bridge callback is lighter, which is why the measured maximum is 45 us rather
than the 150 us extrapolated from the library figure.

## Host consequence: missed STARTs

While a long command is `BUSY`, a START that arrives while the bridge is
inside a `bridge_poll()` step is missed: the slave does not sample SCL during
that step, so the address phase is lost, the read address is NACKed and the
host sees `0xFF`. A focused probe (30 searches, 40 `TEMP_READ_ALL`, 40 32-byte
writes, 10 EEPROM writes) measured:

| Speed | Commands | Retries (NACK / all-`FF`) | Failed commands |
|-------|----------|---------------------------|-----------------|
| 20 kHz | 120 | 0 | 0 |
| 100 kHz | 120 | 3 827 | 0 |

At 20 kHz the adapter's transaction turnaround is longer than the blocking
window, so nothing is missed. At 100 kHz the windows are much shorter than
the turnaround, so retries are expected.

### Logic-analyzer measurement at 100 kHz

A Saleae Logic (fx2lafw, 4 MHz) on SCL/SDA decoded one search command and
settles where the STARTs are actually lost:

| Measurement | Value |
|---|---|
| SCL half-period | 4.00-4.50 us (median 4.25 us, about 118 kHz) |
| STOP to chained START after a write | 15 us |
| STOP to next poll START while `BUSY` | median about 108 us |
| Address NACKs | 31 of 742 frames (4.2%) |
| Gap since the previous STOP at a NACK | median 96 us |

All NACKs were on the read address (`0x61`). Only one of the sampled misses
sat at the 15 us chained gap: the STOP handler finishes in time and the
chained read is served. The misses cluster at the **poll cadence** (about
108 us), where the host's next START lands inside a `bridge_poll()` step that
is still 10-50 us long. Deferring the command start or adding a quiet window
after STOP does not help and was measured to make it worse (7.7% and 16.4%
NACKs), because those changes move polls into the gap instead of shortening
them. What matters is the total time the CPU spends inside `bridge_poll()`,
not the length of the longest single step. That was measured the hard way:
splitting the decode into four-byte chunks across polls made the individual
steps shorter (the 32-byte read dropped from 70 to 13 us) but raised the
end-to-end retry count from 2 746 to 3 899, because more calls with the same
work cost more overhead in total. Reducing the work itself (the table-driven
CRC and codec) is the lever that helps.

### Where the search time goes

The library's `OW_PROFILE` hooks are wired into the profiled build
(`-DOWBRIDGE_PROFILE=ON` defines `OW_PROFILE=1` for the library), and the
search sink is timed separately, so a long poll can be attributed to the
driver or to the application callback:

| Part of the search | Time |
|---|---|
| per-bit arming (`onewire_write_then_read`) | about 6 us, once per bit slot |
| device found: CRC, leaf bookkeeping, next reset | about 13 us |
| device found: application sink (`cache_add`) | 10 us before, 4 us after the word compare |

The device-found step is the longest single poll, but it runs once per device
(nine times per search) while the per-bit steps run 64 times per device. The
retry budget tracks the *sum* of all poll time, so the per-bit arming
dominates; speeding up the sink (two word compares instead of a libc
`memcmp()` call per cached ROM) is real but does not move the end-to-end
count outside run-to-run noise.

Host rule for long operations: either leave at least **100 us** after STOP
(250 us is comfortable), or treat a NACK / all-`FF` read as "not ready" and
retry, as `Bridge.command()` already does. The demo applies the first half
directly: `--poll-gap` (250 us at 100 kHz, 0 at 20 kHz) spaces its BUSY
polls, and over 120 long commands at 100 kHz the retry count fell from 2 746
back-to-back to 2 081 at 250 us and 1 599 at 500 us, at the cost of about 8%
more search latency. `CONFIG_SET` blocks for ~23 ms and always needs the
retry rule. The CH341A
`CH341StreamI2C(wlen, rlen)` combined call is the worst case because its
STOP-to-START turnaround is only about 15 us.

## Command turnaround after STOP

The first implementation copied the response twice (poll into `bus.resp.data`,
then into `last_resp`), used the byte-wise libc `memcpy`, and searched a
38-entry table to dispatch the command. The whole STOP-to-response path took
about **20 us**, while a CH341A chains a write and a read with a 15 us
STOP-to-START gap. The read address therefore arrived before the bridge was
ready and was NACKed.

The current path removes each of those costs:

- dispatch is a 256-entry table indexed by the command byte (no linear scan);
- the command frame is copied directly into the bus slot, without the old
  42-byte struct copy;
- the poll functions write straight into `last_resp + 3`, so `resp_commit()`
  only updates the three header bytes;
- `fast_copy()` uses 32-bit moves when both sides are aligned and a four-byte
  unrolled byte loop otherwise. `-fno-tree-loop-distribute-patterns` keeps GCC
  from turning those loops back into libc `memcpy` calls;
- instant commands complete inside `bridge_submit()` before the STOP handler
  returns; long commands start their engine at STOP, latch `BUSY`, and are
  completed by the main-loop poll.

Measured worst case for an instant command (`OW_GET_DEVICES` with nine cached
ROMs, 72 response bytes) is about **10 us** from STOP detection to the latched
response (489 SysTick counts at 48 MHz). `FIRMWARE_VER` is about 5 us. The
CH341A 15 us turnaround is therefore serviced without a host-side gap; only
long 1-Wire operations still use the `BUSY`/poll flow.

Host rule: no mandatory gap for instant commands. For long operations, poll
the 3-byte header until the status is not `BUSY`; a NACK/`0xFF` still means
"not ready" and can be retried. If the host instead chains transactions with a
few-microsecond turnaround, see "Idle versus active bridge_poll" above for the
gap it must allow.

## Validation

The I2C variant was tested with a CH341A master (`CH341SetStream(0, 1)`,
100 kHz) and a Saleae-compatible logic analyzer (D1 = SDA, D2 = SCL). The
analyzer decodes the write frame `30 / 52 / 00`, the read frame `30`, and
every ACK. The full command set listed in the README returns well-formed
responses. `gap = 0` (the CH341A's 15 us turnaround) was exercised for
`FIRMWARE_VER`, `OW_GET_COUNT`, `OW_GET_STATUS`, `OW_GET_DEVICES` with a
populated cache, `STATS_GET` and `CONFIG_GET`. Back-to-back BUSY polling with
no host delay also completed `OW_RESET` in 3 ms, `OW_SEARCH` in 371 ms and
`TEMP_READ_ALL` in 188 ms, so starting the 1-Wire engine at STOP does not
starve it when the host reads continuously.

A 400 kHz run NACKed even the write address: the polled slave needs the 5 us
half-period of 100 kHz, and 1.25 us at 400 kHz is too short for this software
edge sampler. 100 kHz is the validated rate. Clock stretching and repeated
START are not used by this protocol and were not exercised.
