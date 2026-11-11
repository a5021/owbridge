# Notes on `stm32-async-1wire` from an integration effort

Library version: **v3.1.2** (commit `248d2662`).
Target MCU: **STM32F030F4P6**, TSSOP-20, 16 KB flash (documented) / 4 KB RAM.
Date: 2026-11-10.

The observations below were collected while building an
`I2C/UART/SPI <-> 1-Wire` bridge on this part. Each item follows the same
shape: **observation -> consequence -> suggestion**.

---

## 1. Critical: the target MCU cannot hold three drivers plus a bridge

**Observation.** With `OW_MCU=STM32F030x6`, `DS18B20_MAX_DEVICES=16`,
`OW_PARASITE_POWER=1`, `-Os` and LTO, an image that links `onewire`,
`ds18b20`, `ds2431` and `ds2413` together with the bridge application
measures:

```
text = 23588 B, data = 12 B, bss = 2776 B
```

against a documented limit of 16 KB flash / 4 KB RAM.

**Consequence.** With the library and the application left at the helper's
default (`ow_add_library` sets no optimisation flag, so the library compiles
at `-O0`), the image does not link into the documented 16 KB:
`region FLASH overflowed by 8312 bytes`. Compiling both at `-Os` brings the
same UART image down to 15.3 KB, so it does fit; at `-O2` it is 20.3 KB. The
overflow we first hit was therefore mostly the helper's implicit `-O0`, not
an inherent size floor. Even without the bridge, the library's own `9_multi`
reference occupies 12792 B of text at `-O0`.

**Resolution adopted here.** The F030F4P6 die actually carries **32 KB of
flash** (the same as the F6P6); the upper 16 KB are undocumented but
reachable. The linker script was moved to 31 KB of code plus a 1 KB
configuration page. With `-O2` the UART image is 20.3 KB and fits
comfortably; with `-Os` it is 15.3 KB and would fit the documented 16 KB.
This is a vendor-specific trick and should be called out in the library's
`hardware.md` as a known fact about the F030F4P6.

**Suggestion.**
- State explicitly in `docs/hardware.md` that three drivers plus an
  application do not fit in the documented 16 KB of the F030F4, and that
  production designs should use the F030K6T6 (32 KB) or F030C8T6 (64 KB).
- Add a `9_multi` row to the footprint table in `docs/api-reference.md`
  noting that it leaves roughly 3.4 KB free, which is not enough for a host
  protocol stack.
- Make `-flto` and `--gc-sections` the documented default for F0 rather than
  an optional optimisation.

---

## 2. Critical: the STM32F030F4P6 has no USART2

**Observation.** `stm32f030x6.h` defines only `USART1`; there is no
`USART2`, no `USART2_IRQn` and no `RCC_APB1ENR_USART2EN`. The library's
example uses `USART1 TX` on PA9 as a transmit-only console.

**Consequence.** A full-duplex host UART needs both TX and RX. USART1 TX can
be routed to PA2, PA9 or PA14; RX can be routed to PA3 or PA10. PA10 is
occupied by the 1-Wire bus, so the only usable pair is **PA2 (TX) / PA3
(RX)**. A separate debug console on the same USART1 is not possible.

**Suggestion.** Add an explicit table to `docs/hardware.md` for the F030
family: "USART1: TX = PA2/PA9/PA14, RX = PA3/PA10; PA10 is the bus pin; use
PA2/PA3 for full duplex." The current example silently leaves only TX on
PA9.

---

## 3. Critical: I2C1 collides with the 1-Wire bus

**Observation.** According to the STM32CubeMX database for the F030F4P,
`I2C1_SCL = PA9 (AF4)` and `I2C1_SDA = PA10 (AF4)`. The library's 1-Wire bus
is `PA10 (AF2, TIM1_CH3)`.

**Consequence.** The hardware I2C1 and the 1-Wire bus cannot coexist on PA10.
There is no alternative SDA pin on the F030F4P. The available workarounds are:
- bit-bang a software I2C on free pins (the approach used here);
- time-share PA10 between AF2 and AF4 (fragile; it breaks the input capture);
- drop hardware I2C and use the software slave;
- move to a part with separate pins (F030K6T6 / C8T6).

**Suggestion.** Add a "Pin conflicts on the TSSOP-20" section to
`docs/hardware.md`. It is not obvious that the F030F4P has no PB6/PB7 (where
other F0 parts place I2C1) and that its I2C1 lives on PA9/PA10.

---

## 4. No hook for an external strong pull-up (P-MOSFET gate)

**Observation.** `onewire_strong_pullup()` calls `ow_port_strong_pullup()`,
which calls `ow_port_set_pin_mode()` and switches **only the bus pin itself**
(PA10) between open-drain and push-pull. An external P-MOSFET needs a
separate GPIO driven low to conduct.

**Consequence.** For an industrial bus (long cable, many DS18B20s, parasite
power) the internal pull-up on PA10 may not be enough, and an external switch
cannot be added without patching the port. `docs/api-reference.md` is honest
about this: "The current API cannot drive a separate gate... a gate-pin
option would be a port/API extension."

**Suggestion.** Add a weak hook to `src/ow_port.h`, for example:

```c
__attribute__((weak)) void ow_port_strong_pullup_gate(uint8_t on) { (void)on; }
```

and call it from `ow_port_strong_pullup()` next to
`ow_port_set_pin_mode(on)`. A consumer overrides it with its own GPIO. This
is about three lines and removes a real limitation for industrial boards.

---

## 5. `ow_add_library.cmake` does not pick up the LTO archiver

**Observation.** The helper tries to switch `CMAKE_AR`/`CMAKE_RANLIB` to
`arm-none-eabi-gcc-ar`:

```cmake
set(CMAKE_AR "${_ow_gcc_ar}" PARENT_SCOPE)
```

However, `CMAKE_AR` is a cache variable; `PARENT_SCOPE` does not override it.
As a result `CMAKE_AR` stays `arm-none-eabi-ar`, which does not index LTO
objects, and `-flto` on the archive is effectively a no-op.

**Consequence.** An LTO build through the helper silently degrades to a
regular archive link. In our project the image size did not change until we
set `CMAKE_AR` ourselves with `CACHE ... FORCE`.

**Suggestion.** In `ow_add_library.cmake`, replace `PARENT_SCOPE` with
`set(... CACHE FILEPATH "" FORCE)`, or document that the consumer must set
`CMAKE_AR` explicitly.

---

## 6. CMake: the template requires 3.20 while the helper requires 3.14

**Observation.** `templates/standalone-consumer/CMakeLists.txt` declares
`cmake_minimum_required(VERSION 3.20)`, while `ow_add_library.cmake` declares
`3.14`. On a machine with CMake 3.16 the template does not configure.

**Consequence.** The minimum has to be lowered by hand.

**Suggestion.** Either lower the template to 3.16 (nothing in it is
3.20-specific) or state in the README that CMake >= 3.20 is required.

---

## 7. Timing and overdrive are compile-time only

**Observation.** `inc/ow_config.h` fixes `ONEWIRE_ONE_PULSE`,
`ONEWIRE_ZERO_PULSE`, `ONEWIRE_GUARD_BAND` and `ONEWIRE_SHORT_PULSE_MAX` at
compile time. The API has no overdrive support.

**Consequence.** A bridge cannot switch the 1-Wire speed at runtime; the
preset is chosen when the firmware is built (`TIMING=STANDARD/SLOW/ROBUST/...`).
Overdrive-only devices are not supported.

**Suggestion.** This is a deliberate design choice, but it should be stated
in a "Limitations" section of the README: "no runtime timing profile, no
overdrive". The bridge specification here accounts for it (the `OW_SET_SPEED`
command is intentionally not implemented).

---

## 8. Minor: `ow_stats.c` is an empty translation unit when disabled

**Observation.** With statistics disabled, `ow_stats.c.obj` has `text=0,
bss=0`. This is expected (every inline body compiles away), but
`ow_add_library.cmake` still lists the file in `_ow_srcs`.

**Suggestion.** Not a problem, just a note: the file could be dropped from
the list when `OW_STATS` is off, to avoid empty translation units.

---

## 9. Positive observations worth recording

To keep this review balanced:

- **Genuinely non-blocking** API: `start`/`poll` pairs with no busy-waits;
  `onewire_bus_done()` is a single register read.
- **Honest documentation** on timing, DMA and footprint - rare and useful.
- **`--gc-sections` works**: unused drivers (DS2431/DS2413) are dropped if
  they are never called. The difference between `1_basic` and `9_multi`
  confirms it.
- **`onewire_search_start(sink, max, cmd, family)`** with `family=0` is
  exactly what a bridge needs for full enumeration.
- **Weak callbacks** `ds18b20_busy`/`ds18b20_complete` are easy to override
  in the bridge.

---

## 10. `ow_add_library.cmake` leaves the optimisation level unset

**Observation.** The helper's `target_compile_options` adds `-mcpu`,
`-mthumb`, `-Wall -Wextra` and `-ffunction-sections -fdata-sections`, but no
`-O` flag. GCC therefore compiles the library sources at `-O0`, while the
consumer's own application is usually built at `-Os`/`-O2`. In our image the
unoptimised library accounted for most of the size: the UART build was
23.7 KB at the helper's default and 15.3 KB once both library and application
used `-Os`.

**Consequence.** A downstream project that follows the helper's defaults gets
an oversized and slow 1-Wire library without any warning.

**Suggestion.** Either add a cache option (`OW_OPT`, default `-Os`) to the
helper and apply it to `${tgt}`, or document that the consumer must pass
`-O2`/`-Os` explicitly. Note that `-O3` does not fit a 31 KB image with the
three drivers, so `-O2` is the practical ceiling.

---

## Blocker summary for the F030F4P6

| # | Blocker | Impact | Workaround |
|---|---------|--------|------------|
| 1 | Flash (large at `-O0`) | does not link into 16 KB at the helper default | `-O2` (20.3 KB) or `-Os` (15.3 KB) plus the undocumented 32 KB |
| 2 | No USART2 | only USART1 | PA2/PA3 (full duplex) |
| 3 | I2C1 on PA9/PA10 | collides with the bus | software I2C on PA4/PA5 |
| 4 | No gate hook | no external strong pull-up | patch the port (weak hook) |
| 5 | LTO archiver | LTO silently disabled | `CMAKE_AR` CACHE FORCE |
| 6 | CMake 3.20 | does not configure on 3.16 | lower the minimum |
| 7 | Compile-time timing | no runtime speed change | document in Limitations |
