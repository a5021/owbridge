# Software I2C timing margin

This note records the one-off measurement of the software I2C slave on the
STM32F030F4P6. It answers one question: can the polled main loop observe SCL
and SDA often enough for a reliable 100 kHz bus?

## Method

- Build: `-DHOST_IF=I2C -DOW_OPT=-O2 -DOWBRIDGE_PROFILE=ON`.
- Timer: TIM3 at 1 MHz (16-bit, wrap handled in software).
- The profiler measures three intervals on every main-loop iteration:
  `host_i2c_soft_poll()`, `bridge_poll()`, and the full loop.
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
- While an I2C transaction is active, `bridge_poll()` is skipped (with a
  50 ms safety valve), so the busy loop is shorter than the figures above.
- 400 kHz would need a 1.25 us half-period and is not supported.

## Command turnaround after STOP

The table above measures polling latency while the bus is idle or transferring
bytes. Completing a command is a separate interval: after the STOP of a write,
the main loop runs `bridge_poll()`, which latches the response. Measured on
the same bench with a SysTick counter around that call, the longest
`bridge_poll()` during a `FIRMWARE_VER` command was about **20 us** (979
counts at 48 MHz).

This matters for a master that chains a write and a read. A CH341A at 100 kHz
issues the read START only **15 us** after the write STOP when the two calls
are queued back to back. That is shorter than the 20 us command turnaround, so
the read address can arrive while the bridge is still running the command and
the slave will NACK it. With a host-side gap the bus STOP-to-START interval
grows (200 us host delay produced a 78 us bus gap on this adapter) and the
read is acknowledged.

The firmware-side fix for the data path was to stop copying the response into
a local TX buffer inside the ACK falling-edge handler. `bridge_get_response()`
performed a `memcpy` of up to 163 bytes there, which outlasts the SCL low
phase and made the master sample the first data bit as released. The slave now
transfers directly from the bridge's latched buffer (`bridge_response_ptr()`),
which is stable for the whole transaction because `bridge_poll()` is deferred
while the bus is busy.

Host rule: after a command write, wait at least ~200 us, or poll the 3-byte
header and treat an unacknowledged/`0xFF` read as "not ready" and retry. The
`BUSY` status remains the intended mechanism for long 1-Wire operations.

## Validation

The I2C variant was tested with a CH341A master (`CH341SetStream(0, 1)`,
100 kHz) and a Saleae-compatible logic analyzer (D1 = SDA, D2 = SCL). The
analyzer decodes the write frame `30 / 52 / 00`, the read frame `30`, and
every ACK. The full command set listed in the README returns well-formed
responses. Clock stretching and repeated START are not used by this protocol
and were not exercised.
