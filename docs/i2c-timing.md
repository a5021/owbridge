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
- This experiment measures polling latency only. ACK generation, repeated
  START, clock stretching and the state machine still need a real I2C master
  for validation.
