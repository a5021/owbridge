#!/usr/bin/env python3
"""Protocol boundary, stale-buffer and fuzz tests for the owbridge firmware.

The tool talks to the I2C build through the same CH341A transport as
``owbridge_demo.py``. It checks the central payload validation:

* every command's minimum, maximum and out-of-range lengths;
* semantic argument checks (zero ROM, read count, resolution bits, ...);
* truncated and padded frames, unknown command codes;
* stale bytes left in the command buffer by a previous command;
* an optional fuzz run that must leave the bus and the statistics healthy.

Usage:
    python protocol_test.py
    python protocol_test.py --fuzz 10000
    python protocol_test.py --destructive
    python protocol_test.py --json
"""

from __future__ import annotations

import argparse
import json
import random
import sys
from dataclasses import dataclass

import owbridge_demo as demo
from owbridge_demo import (
    Bridge,
    BridgeError,
    BridgeStatus,
    BridgeTimeout,
    Ch341I2C,
    BR_INVALID_CMD,
    BR_MAX_DATA,
    BR_PARAM_ERROR,
    CMD_NAMES,
    DEFAULT_DLL,
    SPEEDS,
    STATUS_NAMES,
)

# --- Firmware validation table (keep in sync with src/bridge.c) ----------- #

ARG_ROM = 0x01
ARG_ALL_OR_ROM = 0x02
ARG_READ_N = 0x04
ARG_RES_BITS = 0x08
ARG_BOOL = 0x10
ARG_EEPROM_N = 0x20

#: Commands whose valid payload is only exercised with --destructive or is
#: deliberately never executed (it reboots, clears statistics, or changes the
#: bus power state).
NEVER_EXECUTE = {
    0x0B,  # OW_SET_PULLUP: leaves the strong pull-up held
    0x0C,  # OW_SET_POWER: changes parasite power on a live bus
    0x27,  # TEMP_COPY: writes the DS18B20 EEPROM
    0x28,  # TEMP_RECALL: loads the DS18B20 EEPROM
    0x31,  # EEPROM_WRITE: writes the DS2431
    0x41,  # IO_WRITE: drives the DS2413 outputs
    0x51,  # CONFIG_SET: writes the flash configuration page
    0x53,  # RESET: reboots the MCU
    0x55,  # STATS_RESET: clears the counters this test compares
}


@dataclass
class Spec:
    code: int
    name: str
    min_len: int
    max_len: int
    flags: int = 0
    rom: bytes = b""             # addressed commands: the 8-byte ROM
    sample: bytes = b""          # trailing payload; padded with 0xFF
    special: tuple = ()          # explicit valid lengths (TEMP_CONVERT)

    @property
    def valid_lengths(self) -> tuple:
        if self.special:
            return self.special
        return tuple(sorted(set([self.min_len, self.max_len])))

    def accepts(self, n: int) -> bool:
        return self.min_len <= n <= self.max_len


def build_specs(ctx: dict) -> list:
    """Return the firmware command table with device-specific sample data."""
    rom28 = ctx["rom28"]
    rom2d = ctx["rom2d"]
    rom3a = ctx["rom3a"]
    bits = ctx["res_bits"]
    th, tl = ctx["alarm"]
    specs = [
        Spec(0x01, "OW_RESET", 0, 0),
        Spec(0x02, "OW_WRITE_BYTE", 1, 1, sample=b"\xBE"),
        Spec(0x03, "OW_READ_BYTE", 0, 0),
        Spec(0x04, "OW_WRITE_BYTES", 1, 32, sample=b"\xBE"),
        Spec(0x05, "OW_READ_BYTES", 1, 1, flags=ARG_READ_N, sample=b"\x09"),
        Spec(0x06, "OW_MATCH_ROM", 8, 40, flags=ARG_ROM, rom=rom28),
        Spec(0x07, "OW_SKIP_ROM", 1, 32, sample=b"\xBE"),
        Spec(0x08, "OW_READ_ROM", 0, 0),
        Spec(0x09, "OW_SEARCH", 0, 3, sample=b"\xF0\x00\x10"),
        Spec(0x0A, "OW_SEARCH_ALM", 0, 1, sample=b"\x10"),
        Spec(0x0B, "OW_SET_PULLUP", 1, 1, flags=ARG_BOOL, sample=b"\x00"),
        Spec(0x0C, "OW_SET_POWER", 1, 1, flags=ARG_BOOL, sample=b"\x01"),
        Spec(0x0D, "OW_GET_STATUS", 0, 0),
        Spec(0x0E, "OW_GET_DEVICES", 2, 2, sample=b"\x00\x10"),
        Spec(0x0F, "OW_GET_COUNT", 0, 0),
        Spec(0x10, "OW_CLEAR_CACHE", 0, 0),
        Spec(0x11, "OW_ABORT", 0, 0),

        Spec(0x20, "TEMP_CONVERT", 0, 8, flags=ARG_ALL_OR_ROM,
             rom=rom28, special=(0, 8)),
        Spec(0x21, "TEMP_READ", 8, 8, flags=ARG_ROM, rom=rom28),
        Spec(0x22, "TEMP_READ_ALL", 0, 0),
        Spec(0x23, "TEMP_SET_RES", 9, 9, flags=ARG_ROM | ARG_RES_BITS,
             rom=rom28, sample=bytes([bits])),
        Spec(0x24, "TEMP_GET_RES", 8, 8, flags=ARG_ROM, rom=rom28),
        Spec(0x25, "TEMP_SET_ALARM", 10, 10, flags=ARG_ROM,
             rom=rom28, sample=bytes([th & 0xFF, tl & 0xFF])),
        Spec(0x26, "TEMP_GET_ALARM", 8, 8, flags=ARG_ROM, rom=rom28),
        Spec(0x27, "TEMP_COPY", 8, 8, flags=ARG_ROM, rom=rom28),
        Spec(0x28, "TEMP_RECALL", 8, 8, flags=ARG_ROM, rom=rom28),
        Spec(0x29, "TEMP_DETECT_P", 8, 8, flags=ARG_ROM, rom=rom28),

        Spec(0x30, "EEPROM_READ", 10, 10, flags=ARG_ROM | ARG_EEPROM_N,
             rom=rom2d, sample=bytes([0x70, 0x08])),
        Spec(0x31, "EEPROM_WRITE", 10, 17, flags=ARG_ROM,
             rom=rom2d, sample=bytes([0x70]) + b"\x00" * 8),
        Spec(0x32, "EEPROM_READ_S", 8, 8, flags=ARG_ROM, rom=rom2d),

        Spec(0x40, "IO_READ", 8, 8, flags=ARG_ROM, rom=rom3a),
        Spec(0x41, "IO_WRITE", 9, 9, flags=ARG_ROM,
             rom=rom3a, sample=bytes([ctx["io_latch"]])),

        Spec(0x50, "CONFIG_GET", 0, 0),
        Spec(0x51, "CONFIG_SET", 40, 40, sample=ctx["config"]),
        Spec(0x52, "FIRMWARE_VER", 0, 0),
        Spec(0x53, "RESET", 0, 0),
        Spec(0x54, "STATS_GET", 0, 0),
        Spec(0x55, "STATS_RESET", 0, 0),
    ]
    return specs


def payload_for(spec: Spec, n: int) -> bytes:
    """Build a syntactically and semantically valid payload of length n."""
    base = spec.rom + spec.sample
    if n <= len(base):
        return base[:n]
    return base + b"\xFF" * (n - len(base))


# --- Reporting ------------------------------------------------------------ #

class Report:
    def __init__(self) -> None:
        self.cases = []
        self.skips = []
        self.failed = 0

    def check(self, group: str, label: str, ok: bool, detail: str = "") -> None:
        if not ok:
            self.failed += 1
        self.cases.append({"group": group, "case": label,
                           "ok": bool(ok), "detail": detail})
        print("%s  %-10s %s%s" % ("PASS" if ok else "FAIL", group, label,
                                  ("  [" + detail + "]") if detail else ""))

    def skip(self, group: str, label: str, reason: str) -> None:
        self.skips.append({"group": group, "case": label, "reason": reason})
        print("SKIP  %-10s %s  [%s]" % (group, label, reason))

    def summary(self) -> dict:
        total = len(self.cases)
        passed = total - self.failed
        return {
            "passed": passed,
            "failed": self.failed,
            "skipped": len(self.skips),
            "total": total,
            "result": "PASS" if self.failed == 0 else "FAIL",
        }


def status_name(code: int) -> str:
    return STATUS_NAMES.get(code, "0x%02X" % code)


def cmd_name(code: int) -> str:
    return CMD_NAMES.get(code, "0x%02X" % code)


# --- Frame helpers -------------------------------------------------------- #

def send(bridge: Bridge, frame: bytes, timeout: float = 3.0):
    return bridge.raw_frame(frame, timeout=timeout)


def timeout_for(code: int) -> float:
    if code in (0x20, 0x22):       # Convert T, read every scratchpad
        return 10.0
    if code in (0x31,):            # EEPROM write
        return 15.0
    return 5.0


def expect_reject(report: Report, bridge: Bridge, group: str, label: str,
                  frame: bytes, status: int = BR_PARAM_ERROR) -> None:
    try:
        resp = send(bridge, frame)
    except (BridgeError, BridgeTimeout) as exc:
        report.check(group, label, False, "transport: %s" % exc)
        return
    ok = resp.status == status and resp.cmd == frame[0] and resp.data == b""
    report.check(group, label, ok,
                 "%s cmd=0x%02X len=%d" % (resp.status_name, resp.cmd, len(resp.data)))


def expect_valid(report: Report, bridge: Bridge, group: str, label: str,
                 frame: bytes, timeout: float = 5.0) -> None:
    try:
        resp = send(bridge, frame, timeout=timeout)
    except (BridgeError, BridgeTimeout) as exc:
        report.check(group, label, False, "transport: %s" % exc)
        return
    ok = resp.status not in (BR_PARAM_ERROR, BR_INVALID_CMD)
    report.check(group, label, ok,
                 "%s cmd=0x%02X len=%d" % (resp.status_name, resp.cmd, len(resp.data)))


# --- Test phases ---------------------------------------------------------- #

def phase_unknown(report: Report, bridge: Bridge) -> None:
    for code in (0x00, 0x12, 0xFF):
        expect_reject(report, bridge, "unknown",
                      "cmd=0x%02X len=0" % code,
                      bytes([code, 0]), BR_INVALID_CMD)


def phase_matrix(report: Report, bridge: Bridge, specs: list) -> None:
    for spec in specs:
        lengths = set()

        def add(n):
            if 0 <= n <= BR_MAX_DATA:
                lengths.add(n)

        add(0)
        if spec.min_len > 0:
            add(spec.min_len - 1)
        for n in spec.valid_lengths:
            add(n)
        if spec.max_len < BR_MAX_DATA:
            add(spec.max_len + 1)
        add(41)

        for n in sorted(lengths):
            label = "%s len=%d" % (spec.name, n)
            declared = bytes([spec.code, n])
            if n == 41:
                expect_reject(report, bridge, "matrix", label,
                              declared + payload_for(spec, 41))
                continue
            valid = spec.accepts(n)
            if not valid:
                expect_reject(report, bridge, "matrix", label,
                              declared + payload_for(spec, n))
                continue
            if n not in spec.valid_lengths:
                expect_reject(report, bridge, "matrix", label,
                              declared + payload_for(spec, n))
                continue
            if spec.code in NEVER_EXECUTE:
                report.skip("matrix", label, "valid payload not executed")
                continue
            if n != 0 and (spec.flags & (ARG_ROM | ARG_ALL_OR_ROM)) and not spec.rom:
                report.skip("matrix", label, "no ROM available")
                continue
            expect_valid(report, bridge, "matrix", label,
                         declared + payload_for(spec, n), timeout_for(spec.code))


def phase_arguments(report: Report, bridge: Bridge, ctx: dict) -> None:
    rom28 = ctx["rom28"]

    # A zero first byte must not silently fall back to broadcast.
    expect_reject(report, bridge, "args", "TEMP_READ rom[0]=0",
                  bytes([0x21, 8]) + b"\x00" * 8)
    expect_reject(report, bridge, "args", "OW_MATCH_ROM rom[0]=0",
                  bytes([0x06, 8]) + b"\x00" * 8)
    expect_reject(report, bridge, "args", "EEPROM_READ rom[0]=0",
                  bytes([0x30, 10]) + b"\x00" * 8 + bytes([0x70, 8]))
    expect_reject(report, bridge, "args", "IO_READ rom[0]=0",
                  bytes([0x40, 8]) + b"\x00" * 8)

    # OW_READ_BYTES count.
    expect_reject(report, bridge, "args", "OW_READ_BYTES n=0",
                  bytes([0x05, 1, 0]))
    expect_reject(report, bridge, "args", "OW_READ_BYTES n=33",
                  bytes([0x05, 1, 33]))
    expect_valid(report, bridge, "args", "OW_READ_BYTES n=32",
                 bytes([0x05, 1, 32]), timeout=10.0)

    # TEMP_SET_RES resolution bits.
    expect_reject(report, bridge, "args", "TEMP_SET_RES bits=8",
                  bytes([0x23, 9]) + rom28 + bytes([8]))
    expect_reject(report, bridge, "args", "TEMP_SET_RES bits=13",
                  bytes([0x23, 9]) + rom28 + bytes([13]))

    # Boolean flags.
    expect_reject(report, bridge, "args", "OW_SET_PULLUP value=2",
                  bytes([0x0B, 1, 2]))
    expect_reject(report, bridge, "args", "OW_SET_POWER value=2",
                  bytes([0x0C, 1, 2]))

    # EEPROM_READ length byte.
    expect_reject(report, bridge, "args", "EEPROM_READ len=0",
                  bytes([0x30, 10]) + ctx["rom2d"] + bytes([0x70, 0]))
    expect_reject(report, bridge, "args", "EEPROM_READ len=9",
                  bytes([0x30, 10]) + ctx["rom2d"] + bytes([0x70, 9]))

    # TEMP_CONVERT accepts 0 or exactly 8 bytes.
    for n in (1, 4, 7, 9):
        expect_reject(report, bridge, "args", "TEMP_CONVERT len=%d" % n,
                      bytes([0x20, n]) + payload_for(
                          Spec(0x20, "TEMP_CONVERT", 0, 8), n))


def phase_structure(report: Report, bridge: Bridge, ctx: dict) -> None:
    # A frame with only a command byte never reaches bridge_submit().
    before = bridge.read_response()
    bridge._tx(bytes([0x0F]))       # OW_GET_COUNT, no length byte
    after = bridge.read_response()
    same = (before.status == after.status and before.cmd == after.cmd
            and before.data == after.data)
    report.check("structure", "1-byte frame is unanswered", same,
                 "%s/0x%02X -> %s/0x%02X"
                 % (status_name(before.status), before.cmd,
                    status_name(after.status), after.cmd))

    expect_reject(report, bridge, "structure", "OW_WRITE_BYTES declared 40, sent 5",
                  bytes([0x04, 40]) + b"\x01" * 5)
    expect_reject(report, bridge, "structure", "OW_MATCH_ROM declared 8, sent 4",
                  bytes([0x06, 8]) + ctx["rom28"][:4])
    expect_reject(report, bridge, "structure", "padded frame (dlen=0, 4 extra)",
                  bytes([0x0F, 0, 0xAA, 0xBB, 0xCC, 0xDD]))
    expect_reject(report, bridge, "structure", "declared 41 with 41 bytes",
                  bytes([0x04, 41]) + b"\xFF" * 41)
    expect_reject(report, bridge, "structure", "declared 255, no data",
                  bytes([0x04, 255]))
    # RESET with a payload must be rejected before it can reboot the MCU.
    expect_reject(report, bridge, "structure", "RESET len=1",
                  bytes([0x53, 1, 0]))


def phase_stale(report: Report, bridge: Bridge, ctx: dict) -> None:
    # Fill every command buffer with a known pattern.
    expect_valid(report, bridge, "stale", "fill OW_WRITE_BYTES 32 x FF",
                 bytes([0x04, 32]) + b"\xFF" * 32, timeout=10.0)
    stats_before = bridge.stats_get()

    expect_reject(report, bridge, "stale", "OW_WRITE_BYTE len=0",
                  bytes([0x02, 0]))
    expect_reject(report, bridge, "stale", "OW_MATCH_ROM len=7",
                  bytes([0x06, 7]) + ctx["rom28"][:7])
    expect_reject(report, bridge, "stale", "TEMP_READ len=7",
                  bytes([0x21, 7]) + ctx["rom28"][:7])
    expect_reject(report, bridge, "stale", "OW_READ_BYTES len=0",
                  bytes([0x05, 0]))
    expect_reject(report, bridge, "stale", "OW_SKIP_ROM len=0",
                  bytes([0x07, 0]))

    stats_after = bridge.stats_get()
    # STATS_GET is an instant command and its own COMMANDS counter is
    # accounted by bridge_poll after the response is latched, so the baseline
    # read may add one count before the second read. Rejected frames must add
    # nothing beyond that.
    delta = stats_after["commands"] - stats_before["commands"]
    same = (0 <= delta <= 1
            and stats_before["presence_fail"] == stats_after["presence_fail"]
            and stats_before["crc_error"] == stats_after["crc_error"])
    report.check("stale", "rejected frames never touch the bus", same,
                 "commands %d -> %d (delta %d)"
                 % (stats_before["commands"], stats_after["commands"], delta))

    # The bus must still be alive and the cache intact after the rejects.
    try:
        present = bridge.ow_reset()
        count = bridge.ow_search(0xF0, 0, 16)
        bridge.ow_get_count()
        devices = bridge.ow_get_devices(0, 16)
    except (BridgeError, BridgeStatus, BridgeTimeout) as exc:
        report.check("stale", "bus alive after rejects", False, str(exc))
        return
    report.check("stale", "bus alive after rejects", bool(present),
                 "reset present=%s" % present)
    report.check("stale", "cache intact after rejects",
                 count == ctx["device_count"] and len(devices) == count,
                 "search=%d cached=%d baseline=%d"
                 % (count, len(devices), ctx["device_count"]))


def fuzz_frames(rng: random.Random, specs: list, count: int) -> list:
    by_code = {spec.code: spec for spec in specs}
    # Only quick, read-only commands: a fuzz run must not spend minutes in
    # conversions and must not write scratchpads, EEPROM or configuration.
    safe_codes = (0x03, 0x0D, 0x0E, 0x0F, 0x50, 0x52, 0x54)
    safe = [by_code[code] for code in safe_codes]
    unknown = [c for c in range(256) if c not in by_code and c != 0x53]
    frames = []
    for _ in range(count):
        roll = rng.random()
        if roll < 0.60:
            code = rng.choice(unknown)
            frames.append(bytes([code, rng.choice([0, 1, 2, 8, 40, 41, 255])]))
        elif roll < 0.90:
            spec = rng.choice(safe)
            n = rng.choice([0, 1, 2, spec.min_len, spec.max_len, 41])
            n = max(0, min(n, 41))
            data = bytes(rng.randrange(256) for _ in range(n))
            frames.append(bytes([spec.code, n]) + data)
        else:
            code = rng.choice(unknown + list(safe_codes))
            declared = rng.choice([0, 1, 8, 40, 41, 255])
            actual = rng.choice([0, 1, 3, 8, 40])
            data = bytes(rng.randrange(256) for _ in range(actual))
            frames.append(bytes([code, declared]) + data)
    return frames


def phase_fuzz(report: Report, bridge: Bridge, specs: list, ctx: dict,
               count: int, seed: int) -> None:
    rng = random.Random(seed)
    before = bridge.stats_get()
    errors = []
    for i, frame in enumerate(fuzz_frames(rng, specs, count)):
        try:
            resp = send(bridge, frame, timeout=2.0)
        except (BridgeError, BridgeTimeout) as exc:
            errors.append("frame %d (%s): %s" % (i, frame.hex(" "), exc))
            if len(errors) >= 5:
                break
            continue
        if resp.status in (0x04, 0x05, 0x08):   # BUS_SHORT, TIMEOUT, BUS_ERROR
            errors.append("frame %d (%s): %s" % (i, frame.hex(" "), resp.status_name))
        if (i + 1) % 100 == 0:
            try:
                bridge.stats_get()
            except (BridgeError, BridgeTimeout) as exc:
                errors.append("keepalive after frame %d: %s" % (i, exc))
                break
    report.check("fuzz", "%d frames accepted without bus errors" % count,
                 not errors, "; ".join(errors[:3]))

    after = bridge.stats_get()
    report.check("fuzz", "uptime kept running",
                 after["uptime_s"] >= before["uptime_s"],
                 "%d -> %d s" % (before["uptime_s"], after["uptime_s"]))
    for key in ("presence_fail", "crc_error", "bus_short", "timeout", "aborts"):
        report.check("fuzz", "%s unchanged" % key,
                     after[key] == before[key],
                     "%d -> %d" % (before[key], after[key]))


def phase_verify(report: Report, bridge: Bridge, ctx: dict) -> None:
    try:
        version = bridge.firmware_ver()
        present = bridge.ow_reset()
        count = bridge.ow_search(0xF0, 0, 16)
        devices = bridge.ow_get_devices(0, 16)
        temps = bridge.temp_read_all()
        stats = bridge.stats_get()
    except (BridgeError, BridgeStatus, BridgeTimeout) as exc:
        report.check("verify", "final health check", False, str(exc))
        return
    report.check("verify", "firmware answers", bool(version),
                 version if isinstance(version, str) else repr(version))
    report.check("verify", "bus has presence", bool(present))
    report.check("verify", "device count unchanged",
                 count == ctx["device_count"] and len(devices) == count,
                 "search=%d cached=%d baseline=%d"
                 % (count, len(devices), ctx["device_count"]))
    report.check("verify", "temperature array intact",
                 len(temps) == ctx["temp_count"],
                 "%d values, baseline %d" % (len(temps), ctx["temp_count"]))
    report.check("verify", "statistics readable", "uptime_s" in stats,
                 "uptime %d s" % stats["uptime_s"])


def phase_destructive(report: Report, bridge: Bridge, ctx: dict) -> None:
    # DS2431: read, rewrite the same block, verify.
    old = bridge.eeprom_read(ctx["rom2d"], 0x70, 8)
    try:
        bridge.eeprom_write(ctx["rom2d"], 0x70, old)
        new = bridge.eeprom_read(ctx["rom2d"], 0x70, 8)
        report.check("destructive", "EEPROM_WRITE round-trip", new == old,
                     "%s -> %s" % (old.hex(), new.hex()))
    except (BridgeError, BridgeStatus, BridgeTimeout) as exc:
        report.check("destructive", "EEPROM_WRITE round-trip", False, str(exc))

    # Configuration: read the raw 40 bytes, write them back unchanged.
    old_cfg = bridge.expect(0x50)
    try:
        bridge.expect(0x51, old_cfg, timeout=10.0)
        new_cfg = bridge.expect(0x50)
        report.check("destructive", "CONFIG_SET round-trip", new_cfg == old_cfg,
                     "%d bytes" % len(new_cfg))
    except (BridgeError, BridgeStatus, BridgeTimeout) as exc:
        report.check("destructive", "CONFIG_SET round-trip", False, str(exc))


# --- Setup ---------------------------------------------------------------- #

def scan(bridge: Bridge) -> tuple:
    bridge.ow_search(0xF0, 0, 16)
    count = bridge.ow_get_count()
    devices = bridge.ow_get_devices(0, 16)
    return count, devices


def first_family(devices: list, family: int):
    for rom in devices:
        if rom and rom[0] == family:
            return rom
    return None


def build_context(bridge: Bridge, devices: list) -> dict:
    temp_devices = [rom for rom in devices if rom and rom[0] == 0x28]
    rom28 = temp_devices[0] if temp_devices else b""
    rom2d = first_family(devices, 0x2D) or b""
    rom3a = first_family(devices, 0x3A) or b""

    if rom28:
        bits = bridge.temp_get_res(rom28)
        th, tl = bridge.temp_get_alarm(rom28)
    else:
        bits = 12
        th, tl = 25, 15
    io_latch = bridge.io_read(rom3a)[1] if rom3a else 0
    return {
        "rom28": rom28,
        "rom2d": rom2d,
        "rom3a": rom3a,
        "res_bits": bits,
        "alarm": (th, tl),
        "io_latch": io_latch,
        "config": bridge.expect(0x50),
        "device_count": len(devices),
        "temp_count": len(temp_devices),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Boundary and fuzz tests for per-command payload validation.")
    parser.add_argument("--addr", default="0x30", help="I2C slave address (default 0x30)")
    parser.add_argument("--speed", default="100k", choices=demo.SPEED_CHOICES,
                        help="CH341A I2C clock (default 100k)")
    parser.add_argument("--dll", default=DEFAULT_DLL, help="path to CH341DLLA64.DLL")
    parser.add_argument("--fuzz", type=int, default=0,
                        help="number of random frames (default 0)")
    parser.add_argument("--seed", type=int, default=1, help="fuzz seed (default 1)")
    parser.add_argument("--destructive", action="store_true",
                        help="also run the EEPROM and config write round-trips")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    parser.add_argument("--verbose", action="store_true", help="print every I2C frame")
    parser.add_argument("--no-color", action="store_true", help=argparse.SUPPRESS)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        addr = int(args.addr, 0)
    except ValueError:
        print("error: --addr must be a number, e.g. 0x30", file=sys.stderr)
        return 2
    if not 0x08 <= addr <= 0x77:
        print("error: --addr must be a 7-bit address (0x08..0x77)", file=sys.stderr)
        return 2

    trace = None
    if args.verbose:
        def trace(wbytes, rdata, direction=None, frame=None):
            if direction == "TX":
                print("  TX %02X %s" % ((addr << 1), frame.hex(" ")))
            elif direction == "RX":
                print("  RX %s" % rdata.hex(" "))

    report = Report()
    try:
        with Ch341I2C(args.dll, speed=SPEEDS[args.speed]) as io:
            bridge = Bridge(io, addr=addr, trace=trace)
            if not args.json:
                print("owbridge protocol test  addr 0x%02X  %s  fuzz=%d"
                      % (addr, args.speed, args.fuzz))
            count, devices = scan(bridge)
            ctx = build_context(bridge, devices)
            specs = build_specs(ctx)
            if not args.json:
                print("baseline: %d devices, %d DS18B20"
                      % (count, ctx["temp_count"]))

            phase_unknown(report, bridge)
            phase_matrix(report, bridge, specs)
            phase_arguments(report, bridge, ctx)
            phase_structure(report, bridge, ctx)
            phase_stale(report, bridge, ctx)
            if args.fuzz > 0:
                phase_fuzz(report, bridge, specs, ctx, args.fuzz, args.seed)
            phase_verify(report, bridge, ctx)
            if args.destructive:
                phase_destructive(report, bridge, ctx)

            summary = report.summary()
    except (BridgeError, BridgeStatus, BridgeTimeout, demo.Ch341Error) as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps({"summary": summary, "cases": report.cases,
                          "skips": report.skips}, indent=2, sort_keys=True))
    else:
        print()
        print("RESULT: %s  (%d passed, %d failed, %d skipped)"
              % (summary["result"], summary["passed"], summary["failed"],
                 summary["skipped"]))
    return 0 if report.failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
