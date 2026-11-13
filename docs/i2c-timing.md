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
  once.
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
so a command advances only after STOP. The first poll after STOP executes one
library poll step, which can cover more than one work event. The library's CPU
profile (`stm32-async-1wire` v3.1.2, `docs/cpu-profile.md`, F030 at 8 MHz)
gives two bounds:

| Metric | Cycles | Time at 48 MHz | Time with flash wait states |
|--------|--------|----------------|-----------------------------|
| worst driver work event (measurement) | 2 857 | 59.5 us | ~65-85 us |
| worst public poll, measurement phase | 7 217 | 150 us | ~160-210 us |
| worst driver work event (search) | 1 117 | 23.3 us | ~25-35 us |
| worst public poll, search phase | 3 356 | 70 us | ~70-100 us |

The public-poll maxima include the CPU-profile example's completion callback
(UART formatting); the bridge's callback is lighter, so the real firmware
should sit between the two rows. The conservative host bound is therefore the
public-poll row: **allow >= 250 us after STOP** until the profiler replaces it
with a measurement. The extended profiler reports the actual value in
`prof_bridge_max`, `prof_bridge_last` and `prof_bridge_hist[8]`;
`prof_start_missed` counts transactions whose START arrived during that window.
Bench results will replace these estimates when the ST-LINK is available again.

Consequence for the host: while a long command is `BUSY`, a START that arrives
less than one worst-case poll after STOP can be missed. At 100 kHz a byte takes
90 us, so losing the first START loses the address phase; the bridge NACKs the
read address (I2C) and the host sees `0xFF`. This is the most likely explanation
for the NACK counts on `TEMP_READ_ALL` and `OW_SEARCH` in the torture test; the
earlier "adapter-only" explanation was not demonstrated. The profiler's
`prof_start_missed` counter settles it on the bench.

Host rule for long operations: either space transactions by more than the
worst-case poll (measured value pending; use >= 250 us conservatively), or
treat a NACK / all-`FF` read as "not ready" and retry, as `Bridge.command()`
already does. The CH341A `CH341StreamI2C(wlen, rlen)` combined call is the
worst case because its STOP-to-START turnaround is only about 15 us.

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
