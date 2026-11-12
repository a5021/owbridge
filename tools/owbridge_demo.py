#!/usr/bin/env python3
"""Interactive demo for the owbridge 1-Wire bridge over a CH341A adapter.

The script speaks the bridge protocol (write a command frame, poll the status
header until it is no longer BUSY, read the response) and walks through every
capability of the firmware: raw 1-Wire, device search, DS18B20 temperature,
DS2431 EEPROM, DS2413 PIO, configuration, statistics, queueing and abort.

It is a demo, not a test suite: the output is meant to be read by a human.
Every scenario also returns a dictionary, so ``--json`` produces
machine-readable output for scripting.

Requirements
------------
* Windows with the CH341A driver (``CH341DLLA64.DLL``) installed.
* Python 3.8+; no third-party packages.

Examples
--------
    python owbridge_demo.py info
    python owbridge_demo.py scan
    python owbridge_demo.py temp --json
    python owbridge_demo.py watch --interval 1
    python owbridge_demo.py eeprom --dump
    python owbridge_demo.py raw 0x09 --data F0,00,10
    python owbridge_demo.py timing --all
    python owbridge_demo.py selftest

Protocol note
-------------
The CH341A reports ``ok=1`` even when a slave NACKs the address phase; an
unacknowledged read shows up as all ``FF`` bytes. The bridge is polled, so the
first status read after a command write can arrive before the slave has
finished the previous transaction. ``Bridge.command()`` therefore treats an
``FF FF`` header as "not ready yet" and keeps polling until the command
timeout, exactly like the host is expected to do.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import statistics
import sys
import time
from dataclasses import dataclass

# --------------------------------------------------------------------------- #
# Protocol constants (mirror of include/bridge.h)
# --------------------------------------------------------------------------- #

BR_OK = 0x00
BR_BUSY = 0x01
BR_NO_DEVICE = 0x02
BR_CRC_ERROR = 0x03
BR_BUS_SHORT = 0x04
BR_TIMEOUT = 0x05
BR_INVALID_CMD = 0x06
BR_PARAM_ERROR = 0x07
BR_BUS_ERROR = 0x08
BR_NOT_SUPPORTED = 0x09
BR_QUEUE_FULL = 0x0A
BR_ABORTED = 0x0B

STATUS_NAMES = {
    BR_OK: "OK",
    BR_BUSY: "BUSY",
    BR_NO_DEVICE: "NO_DEVICE",
    BR_CRC_ERROR: "CRC_ERROR",
    BR_BUS_SHORT: "BUS_SHORT",
    BR_TIMEOUT: "TIMEOUT",
    BR_INVALID_CMD: "INVALID_CMD",
    BR_PARAM_ERROR: "PARAM_ERROR",
    BR_BUS_ERROR: "BUS_ERROR",
    BR_NOT_SUPPORTED: "NOT_SUPPORTED",
    BR_QUEUE_FULL: "QUEUE_FULL",
    BR_ABORTED: "ABORTED",
}

CMD_NAMES = {
    0x01: "OW_RESET",
    0x02: "OW_WRITE_BYTE",
    0x03: "OW_READ_BYTE",
    0x04: "OW_WRITE_BYTES",
    0x05: "OW_READ_BYTES",
    0x06: "OW_MATCH_ROM",
    0x07: "OW_SKIP_ROM",
    0x08: "OW_READ_ROM",
    0x09: "OW_SEARCH",
    0x0A: "OW_SEARCH_ALM",
    0x0B: "OW_SET_PULLUP",
    0x0C: "OW_SET_POWER",
    0x0D: "OW_GET_STATUS",
    0x0E: "OW_GET_DEVICES",
    0x0F: "OW_GET_COUNT",
    0x10: "OW_CLEAR_CACHE",
    0x11: "OW_ABORT",
    0x20: "TEMP_CONVERT",
    0x21: "TEMP_READ",
    0x22: "TEMP_READ_ALL",
    0x23: "TEMP_SET_RES",
    0x24: "TEMP_GET_RES",
    0x25: "TEMP_SET_ALARM",
    0x26: "TEMP_GET_ALARM",
    0x27: "TEMP_COPY",
    0x28: "TEMP_RECALL",
    0x29: "TEMP_DETECT_P",
    0x30: "EEPROM_READ",
    0x31: "EEPROM_WRITE",
    0x32: "EEPROM_READ_S",
    0x40: "IO_READ",
    0x41: "IO_WRITE",
    0x50: "CONFIG_GET",
    0x51: "CONFIG_SET",
    0x52: "FIRMWARE_VER",
    0x53: "RESET",
    0x54: "STATS_GET",
    0x55: "STATS_RESET",
}

FAMILY_NAMES = {
    0x01: "DS1990A/DS2401",
    0x10: "DS18S20",
    0x22: "DS1822",
    0x26: "DS2438",
    0x28: "DS18B20",
    0x2D: "DS2431",
    0x3A: "DS2413",
    0x3B: "DS1825",
    0x42: "DS28EA00",
}

BR_MAX_DATA = 40
BR_MAX_RESP = 160

# CH341SetStream() argument: 0=20 kHz, 1=100 kHz, 2=400 kHz, 3=750 kHz.
SPEEDS = {"20k": 0, "100k": 1, "400k": 2, "750k": 3}

#: The software I2C slave is validated at 100 kHz; higher rates NACK.
SPEED_CHOICES = ("20k", "100k")

DEFAULT_DLL = r"C:\Windows\System32\CH341DLLA64.DLL"
DEFAULT_ADDR = 0x30

DS2431_EEPROM_SIZE = 128
DS2431_PAGE_SIZE = 8

#: DS18B20 conversion time in ms per resolution bit.
CONVERT_MS = {9: 94, 10: 188, 11: 375, 12: 750}

#: Resolution reported in the bridge response, in degrees C per step. The
#: library converts the 1/16 C scratchpad word to 0.1 C units, so the
#: smallest unit the host ever sees is 0.1 C.
RES_STEP = {9: 0.5, 10: 0.25, 11: 0.125, 12: 0.1}

# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #


def crc8_dallas(data: bytes) -> int:
    """Dallas/Maxim 1-Wire CRC-8 (reflected polynomial 0x8C, init 0)."""
    crc = 0
    for byte in data:
        inb = byte
        for _ in range(8):
            mix = (crc ^ inb) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            inb >>= 1
    return crc


def rom_crc_ok(rom: bytes) -> bool:
    """A 1-Wire ROM is valid when the last byte is the CRC over the first 7."""
    return len(rom) == 8 and crc8_dallas(rom[:7]) == rom[7]


def fmt_rom(rom: bytes) -> str:
    """Space-separated ROM bytes in bus order (family first, CRC last)."""
    return " ".join("%02X" % b for b in rom)


def fmt_rom_dallas(rom: bytes) -> str:
    """Dallas label form: family-serial-reversed-crc, e.g. 28-00000792782067."""
    if len(rom) != 8:
        return ""
    serial = "".join("%02X" % b for b in reversed(rom[1:7]))
    return "%02X-%s%02X" % (rom[0], serial, rom[7])


def family_name(family: int) -> str:
    return FAMILY_NAMES.get(family, "unknown")


def decode_temp(word: int) -> float:
    """Signed 16-bit temperature in 0.1 C units to degrees C.

    The bridge forwards the library value unchanged: ds18b20_temp is in
    0.1 C units (e.g. 0x00FB = 251 = 25.1 C), not the raw 1/16 C
    scratchpad word.
    """
    if word & 0x8000:
        word -= 0x10000
    return word / 10.0


def bcd_hex(data: bytes) -> str:
    return " ".join("%02X" % b for b in data)


class Color:
    """Minimal ANSI palette that disables itself when not on a terminal."""

    def __init__(self, enabled: bool):
        self.enabled = enabled

    def _wrap(self, code: str, text: str) -> str:
        if not self.enabled:
            return text
        return "\033[%sm%s\033[0m" % (code, text)

    def green(self, t):
        return self._wrap("32", t)

    def red(self, t):
        return self._wrap("31", t)

    def yellow(self, t):
        return self._wrap("33", t)

    def cyan(self, t):
        return self._wrap("36", t)

    def bold(self, t):
        return self._wrap("1", t)

    def dim(self, t):
        return self._wrap("2", t)


class Out:
    """Human-readable output with a JSON mode that suppresses prose."""

    def __init__(self, color: bool, json_mode: bool):
        self.c = Color(color and not json_mode)
        self.json_mode = json_mode

    def __call__(self, text: str = "") -> None:
        if not self.json_mode:
            print(text)

    def title(self, text: str) -> None:
        if self.json_mode:
            return
        print()
        print(self.c.bold(text))
        print(self.c.dim("-" * len(text)))

    def kv(self, key: str, value: str, width: int = 18) -> None:
        if self.json_mode:
            return
        print("  %s %s" % (self.c.dim(key.ljust(width)), value))

    def note(self, text: str) -> None:
        if self.json_mode:
            return
        print(self.c.dim("  " + text))

    def warn(self, text: str) -> None:
        if self.json_mode:
            return
        print(self.c.yellow("  ! " + text))

    def error(self, text: str) -> None:
        print(self.c.red("error: " + text), file=sys.stderr)

    def table(self, headers, rows) -> None:
        if self.json_mode:
            return
        srows = [[str(cell) for cell in row] for row in rows]
        widths = [len(h) for h in headers]
        for row in srows:
            for i, cell in enumerate(row):
                widths[i] = max(widths[i], len(cell))
        print("  " + "  ".join(h.ljust(widths[i]) for i, h in enumerate(headers)).rstrip())
        print("  " + "  ".join("-" * widths[i] for i in range(len(headers))))
        for row in srows:
            print("  " + "  ".join(cell.ljust(widths[i]) for i, cell in enumerate(row)).rstrip())

    def hexdump(self, data: bytes, base: int = 0) -> None:
        if self.json_mode:
            return
        for off in range(0, len(data), 16):
            chunk = data[off:off + 16]
            hexpart = " ".join("%02X" % b for b in chunk)
            asciipart = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
            print("  %04X  %-47s  %s" % (base + off, hexpart, asciipart))


# --------------------------------------------------------------------------- #
# CH341A I2C transport
# --------------------------------------------------------------------------- #


class Ch341Error(RuntimeError):
    """The adapter or its driver refused the operation."""


class Ch341I2C:
    """Thin ctypes wrapper around the CH341A stream-I2C API."""

    def __init__(self, dll_path: str, index: int = 0, speed: int = 1, trace=None):
        self.index = index
        self.speed = speed
        self.trace = trace
        try:
            self.lib = ctypes.WinDLL(dll_path)
        except OSError as exc:
            raise Ch341Error("cannot load %s: %s" % (dll_path, exc))
        self.lib.CH341OpenDevice.argtypes = [ctypes.c_ulong]
        self.lib.CH341OpenDevice.restype = ctypes.c_void_p
        self.lib.CH341CloseDevice.argtypes = [ctypes.c_ulong]
        self.lib.CH341CloseDevice.restype = None
        self.lib.CH341SetStream.argtypes = [ctypes.c_ulong, ctypes.c_ulong]
        self.lib.CH341SetStream.restype = ctypes.c_int
        self.lib.CH341StreamI2C.argtypes = [
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_void_p,
        ]
        self.lib.CH341StreamI2C.restype = ctypes.c_int
        self.wbuf = (ctypes.c_ubyte * 256)()
        self.rbuf = (ctypes.c_ubyte * 256)()
        self.opened = False
        self.open()

    def open(self) -> None:
        if not self.lib.CH341OpenDevice(self.index):
            raise Ch341Error("CH341OpenDevice failed (is the adapter plugged in?)")
        if not self.lib.CH341SetStream(self.index, self.speed):
            self.lib.CH341CloseDevice(self.index)
            raise Ch341Error("CH341SetStream failed")
        self.opened = True

    def close(self) -> None:
        if self.opened:
            self.lib.CH341CloseDevice(self.index)
            self.opened = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _stream(self, wbytes: bytes, rlen: int) -> bytes:
        for i, byte in enumerate(wbytes):
            self.wbuf[i] = byte
        ok = self.lib.CH341StreamI2C(
            self.index, len(wbytes), self.wbuf, rlen, self.rbuf if rlen else None
        )
        if not ok:
            raise Ch341Error("CH341StreamI2C failed")
        data = bytes(self.rbuf[:rlen]) if rlen else b""
        if self.trace:
            self.trace(wbytes, data)
        return data

    def write(self, payload: bytes) -> None:
        self._stream(payload, 0)

    def read(self, length: int) -> bytes:
        return self._stream(b"", length)


# --------------------------------------------------------------------------- #
# Bridge protocol
# --------------------------------------------------------------------------- #


class BridgeError(RuntimeError):
    """Base class for bridge level failures."""


class BridgeTimeout(BridgeError):
    """The command did not leave BUSY before the deadline."""


class BridgeStatus(BridgeError):
    """The bridge answered, but with a non-OK status."""

    def __init__(self, command: int, status: int, detail: str = ""):
        self.command = command
        self.status = status
        name = STATUS_NAMES.get(status, "0x%02X" % status)
        what = CMD_NAMES.get(command, "0x%02X" % command)
        super().__init__("%s returned %s%s" % (what, name, (": " + detail) if detail else ""))


@dataclass
class Response:
    status: int
    cmd: int
    data: bytes
    polls: int
    elapsed: float
    nacks: int = 0

    @property
    def ok(self) -> bool:
        return self.status == BR_OK

    @property
    def status_name(self) -> str:
        return STATUS_NAMES.get(self.status, "0x%02X" % self.status)


class Bridge:
    """Command level access to the bridge, one method per command."""

    def __init__(self, io: Ch341I2C, addr: int = DEFAULT_ADDR, trace=None):
        self.io = io
        self.addr = addr
        self.trace = trace

    # -- transport ---------------------------------------------------------

    def _tx(self, frame: bytes) -> None:
        payload = bytes([(self.addr << 1) | 0]) + frame
        if self.trace:
            self.trace(payload, b"", "TX", frame)
        self.io.write(payload)

    def _rx(self, length: int) -> bytes:
        payload = bytes([(self.addr << 1) | 1])
        data = self.io._stream(payload, length)
        if self.trace:
            self.trace(payload, data, "RX", data)
        return data

    def command(self, cmd: int, data: bytes = b"", timeout: float = 3.0) -> Response:
        """Send a command, poll BUSY, then read the full response."""
        if len(data) > BR_MAX_DATA:
            raise ValueError("command data is %d bytes, the limit is %d" % (len(data), BR_MAX_DATA))
        frame = bytes([cmd, len(data)]) + bytes(data)
        t0 = time.perf_counter()
        self._tx(frame)
        deadline = time.monotonic() + timeout
        polls = 0
        nacks = 0
        sends = 1
        while True:
            hdr = self._rx(3)
            polls += 1
            if hdr[0] == 0xFF and hdr[1] == 0xFF:
                # No ACK: the slave has not seen (or has not finished) the
                # previous transaction yet. Keep polling until the deadline.
                nacks += 1
                if time.monotonic() >= deadline:
                    raise BridgeTimeout(
                        "no ACK from 0x%02X after %d polls (%d NACKs); check the "
                        "address and that the firmware is running" % (self.addr, polls, nacks)
                    )
                continue
            if hdr[1] != cmd:
                # A response for another command: the write was NACKed and
                # never reached the bridge, so the latched frame is stale.
                # A command that was accepted always echoes its own code,
                # including while it is BUSY or when the queue is full.
                nacks += 1
                if sends >= 3 or time.monotonic() >= deadline:
                    raise BridgeError(
                        "expected a response for %s, got %s; the command write "
                        "was not acknowledged"
                        % (CMD_NAMES.get(cmd, "0x%02X" % cmd),
                           CMD_NAMES.get(hdr[1], "0x%02X" % hdr[1]))
                    )
                time.sleep(0.001)
                self._tx(frame)
                sends += 1
                continue

            if hdr[0] == BR_BUSY:
                if time.monotonic() >= deadline:
                    raise BridgeTimeout(
                        "%s stayed BUSY for %.2f s (%d polls)"
                        % (CMD_NAMES.get(cmd, "0x%02X" % cmd), timeout, polls)
                    )
                continue

            length = min(hdr[2], BR_MAX_RESP)
            full = self._rx(3 + length) if length else hdr
            if full[:3] != hdr:
                # The data read was NACKed (all FF) or the bridge rewrote
                # the frame before we finished. Both are transient: go
                # back to polling the header, like the stress harness does.
                nacks += 1
                if time.monotonic() >= deadline:
                    raise BridgeError(
                        "response frame kept changing between reads (%d polls)" % polls
                    )
                continue
            return Response(
                status=hdr[0],
                cmd=hdr[1],
                data=full[3:3 + length],
                polls=polls,
                elapsed=time.perf_counter() - t0,
                nacks=nacks,
            )

    def expect(self, cmd: int, data: bytes = b"", timeout: float = 3.0) -> bytes:
        """Send a command and require BR_OK, returning the data bytes."""
        resp = self.command(cmd, data, timeout)
        if resp.status != BR_OK:
            raise BridgeStatus(cmd, resp.status, bcd_hex(resp.data))
        return resp.data

    # -- service -----------------------------------------------------------

    def firmware_ver(self) -> str:
        return self.expect(0x52).decode("ascii", "replace")

    def config_get(self) -> dict:
        raw = self.expect(0x50)
        if len(raw) < 40:
            raise BridgeError("CONFIG_GET returned %d bytes, expected 40" % len(raw))
        magic, version = int.from_bytes(raw[0:4], "little"), int.from_bytes(raw[4:6], "little")
        crc = int.from_bytes(raw[36:40], "little")
        return {
            "magic": "0x%08X" % magic,
            "magic_ok": magic == 0x4F574252,
            "version": "0x%04X" % version,
            "i2c_addr": raw[6],
            "uart_baud_index": raw[7],
            "timing": raw[8],
            "parasite": raw[9],
            "strong_pullup": raw[10],
            "auto_search": raw[11],
            "max_devices": raw[12],
            "led_enable": raw[13],
            "crc32": "0x%08X" % crc,
            "crc_ok": crc == crc32(raw[:36]),
            "raw": raw.hex(),
        }

    def stats_get(self) -> dict:
        raw = self.expect(0x54)
        if len(raw) < 32:
            raise BridgeError("STATS_GET returned %d bytes, expected 32" % len(raw))
        words = [int.from_bytes(raw[i * 4:i * 4 + 4], "big") for i in range(8)]
        return {
            "resets": words[0],
            "presence_fail": words[1],
            "crc_error": words[2],
            "bus_short": words[3],
            "timeout": words[4],
            "commands": words[5],
            "aborts": words[6],
            "uptime_s": words[7],
        }

    def stats_reset(self) -> None:
        self.expect(0x55, timeout=2.0)

    def ow_status(self) -> dict:
        raw = self.expect(0x0D)
        return {
            "busy": bool(raw[0] & 0x01),
            "parasite": bool(raw[0] & 0x08),
            "flags": raw[0],
            "devices": raw[1] if len(raw) > 1 else 0,
        }

    # -- raw 1-Wire --------------------------------------------------------

    def ow_reset(self) -> bool:
        resp = self.command(0x01, timeout=3.0)
        if resp.status == BR_NO_DEVICE:
            return False
        if resp.status != BR_OK:
            raise BridgeStatus(0x01, resp.status)
        return bool(resp.data[0]) if resp.data else False

    def ow_read_rom(self) -> bytes:
        return self.expect(0x08, timeout=5.0)

    def ow_write_byte(self, value: int) -> None:
        self.expect(0x02, bytes([value]), timeout=3.0)

    def ow_read_byte(self) -> int:
        return self.expect(0x03, timeout=3.0)[0]

    def ow_write_bytes(self, payload: bytes) -> None:
        if len(payload) > 32:
            raise ValueError("the 1-Wire engine writes at most 32 bytes per command")
        self.expect(0x04, payload, timeout=10.0)

    def ow_read_bytes(self, count: int) -> bytes:
        if not 1 <= count <= 32:
            raise ValueError("the 1-Wire engine reads 1..32 bytes per command")
        return self.expect(0x05, bytes([count]), timeout=10.0)

    def ow_match_rom(self, rom: bytes, payload: bytes = b"") -> None:
        if len(rom) != 8:
            raise ValueError("MATCH ROM needs the full 8-byte ROM")
        self.expect(0x06, rom + payload, timeout=10.0)

    def ow_skip_rom(self, payload: bytes = b"") -> None:
        self.expect(0x07, payload, timeout=10.0)

    def ow_set_pullup(self, on: bool) -> None:
        self.expect(0x0B, bytes([1 if on else 0]), timeout=2.0)

    def ow_set_power(self, parasite: bool) -> None:
        self.expect(0x0C, bytes([1 if parasite else 0]), timeout=2.0)

    def ow_clear_cache(self) -> None:
        self.expect(0x10, timeout=2.0)

    # -- search ------------------------------------------------------------

    def ow_search(self, mode: int = 0xF0, family: int = 0, max_devices: int = 16) -> int:
        data = bytes([mode, family, max_devices])
        return self.expect(0x09, data, timeout=15.0)[0]

    def ow_search_alarm(self, max_devices: int = 16) -> int:
        return self.expect(0x0A, bytes([max_devices]), timeout=15.0)[0]

    def ow_get_count(self) -> int:
        return self.expect(0x0F, timeout=2.0)[0]

    def ow_get_devices(self, offset: int = 0, count: int = 16) -> list:
        raw = self.expect(0x0E, bytes([offset, count]), timeout=2.0)
        return [raw[i:i + 8] for i in range(0, len(raw) - len(raw) % 8, 8)]

    # -- DS18B20 -----------------------------------------------------------

    def temp_convert(self, rom: bytes = b"") -> Response:
        """Broadcast (empty rom) or single-device Convert T."""
        if rom and len(rom) != 8:
            raise ValueError("TEMP_CONVERT takes an 8-byte ROM or nothing")
        # 750 ms at 12-bit resolution plus the 1-Wire transaction itself.
        return self.command(0x20, rom, timeout=10.0)

    def temp_read(self, rom: bytes) -> int:
        raw = self.expect(0x21, rom, timeout=5.0)
        return int.from_bytes(raw[:2], "big")

    def temp_read_all(self) -> list:
        raw = self.expect(0x22, timeout=10.0)
        return [int.from_bytes(raw[i:i + 2], "big") for i in range(0, len(raw) - len(raw) % 2, 2)]

    def temp_get_res(self, rom: bytes) -> int:
        return self.expect(0x24, rom, timeout=5.0)[0]

    def temp_set_res(self, rom: bytes, bits: int) -> None:
        self.expect(0x23, rom + bytes([bits]), timeout=10.0)

    def temp_get_alarm(self, rom: bytes):
        raw = self.expect(0x26, rom, timeout=5.0)
        return signed_byte(raw[0]), signed_byte(raw[1])

    def temp_set_alarm(self, rom: bytes, th: int, tl: int) -> None:
        self.expect(0x25, rom + bytes([th & 0xFF, tl & 0xFF]), timeout=10.0)

    def temp_copy(self, rom: bytes) -> None:
        self.expect(0x27, rom, timeout=10.0)

    def temp_recall(self, rom: bytes) -> None:
        self.expect(0x28, rom, timeout=10.0)

    def temp_detect_parasite(self, rom: bytes) -> bool:
        return bool(self.expect(0x29, rom, timeout=5.0)[0])

    # -- DS2431 ------------------------------------------------------------

    def eeprom_read(self, rom: bytes, addr: int, length: int) -> bytes:
        if not 1 <= length <= 8:
            raise ValueError("EEPROM_READ returns 1..8 bytes per command")
        return self.expect(0x30, rom + bytes([addr & 0xFF, length]), timeout=5.0)

    def eeprom_write(self, rom: bytes, addr: int, payload: bytes) -> None:
        """Write one full 8-byte block; the DS2431 refuses partial blocks."""
        if len(payload) != 8:
            raise ValueError("EEPROM_WRITE must cover a full 8-byte block")
        if addr & 7:
            raise ValueError("EEPROM_WRITE addresses must be 8-byte aligned")
        self.expect(0x31, rom + bytes([addr & 0xFF]) + payload, timeout=15.0)

    def eeprom_read_scratchpad(self, rom: bytes) -> bytes:
        return self.expect(0x32, rom, timeout=5.0)

    # -- DS2413 ------------------------------------------------------------

    def io_read(self, rom: bytes):
        raw = self.expect(0x40, rom, timeout=5.0)
        return raw[0], raw[1]

    def io_write(self, rom: bytes, states: int) -> None:
        self.expect(0x41, rom + bytes([states]), timeout=10.0)


def signed_byte(value: int) -> int:
    return value - 256 if value > 127 else value


def crc32(data: bytes) -> int:
    """zlib is not guaranteed to be present in every embedded Python build."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


# --------------------------------------------------------------------------- #
# Scenarios
# --------------------------------------------------------------------------- #


def find_devices(bridge: Bridge, out: Out) -> list:
    """Search the bus and return the ROM list from the cache."""
    bridge.ow_search(0xF0, 0, 16)
    count = bridge.ow_get_count()
    devices = bridge.ow_get_devices(0, 16)
    if len(devices) != count:
        out.warn("search reported %d devices, the cache returned %d" % (count, len(devices)))
    return devices


def sc_info(bridge: Bridge, out: Out, args) -> dict:
    out.title("Firmware")
    version = bridge.firmware_ver()
    out.kv("version", version)

    out.title("Configuration (flash page 0x08007C00)")
    cfg = bridge.config_get()
    out.kv("magic", "%s %s" % (cfg["magic"], "(valid)" if cfg["magic_ok"] else "(BAD)"))
    out.kv("version", cfg["version"])
    out.kv("i2c address", "0x%02X" % cfg["i2c_addr"])
    out.kv("uart baud index", str(cfg["uart_baud_index"]))
    out.kv("timing preset", str(cfg["timing"]))
    out.kv("parasite default", "yes" if cfg["parasite"] else "no")
    out.kv("strong pull-up gate", "yes" if cfg["strong_pullup"] else "no")
    out.kv("LEDs", "enabled" if cfg["led_enable"] else "disabled")
    out.kv("crc32", "%s %s" % (cfg["crc32"], "(valid)" if cfg["crc_ok"] else "(BAD)"))

    out.title("Bus status")
    status = bridge.ow_status()
    out.kv("busy", "yes" if status["busy"] else "no")
    out.kv("power mode", "parasite" if status["parasite"] else "external")
    out.kv("cached devices", str(status["devices"]))
    out.kv("flags", "0x%02X" % status["flags"])
    if status["busy"]:
        out.note("BUSY is set because OW_GET_STATUS itself is still the command in flight.")

    out.title("Statistics")
    stats = bridge.stats_get()
    for key in ("commands", "resets", "presence_fail", "crc_error", "bus_short", "timeout", "aborts"):
        out.kv(key, str(stats[key]))
    out.kv("uptime", "%d s (%d min %d s)" % (stats["uptime_s"], stats["uptime_s"] // 60, stats["uptime_s"] % 60))

    return {"version": version, "config": cfg, "status": status, "stats": stats}


def sc_scan(bridge: Bridge, out: Out, args) -> dict:
    out.title("1-Wire bus")
    present = bridge.ow_reset()
    out.kv("presence", "yes" if present else out.c.yellow("no"))
    if not present:
        out.warn("no device answered the reset pulse")
        return {"presence": False, "count": 0, "devices": []}

    count = bridge.ow_search(0xF0, 0, 16)
    cached = bridge.ow_get_count()
    devices = bridge.ow_get_devices(0, 16)
    out.kv("search found", str(count))
    out.kv("cache count", str(cached))

    rows = []
    for index, rom in enumerate(devices):
        rows.append([
            index,
            fmt_rom(rom),
            "0x%02X %s" % (rom[0], family_name(rom[0])),
            out.c.green("ok") if rom_crc_ok(rom) else out.c.red("BAD"),
            fmt_rom_dallas(rom),
        ])
    out.title("Devices")
    out.table(["#", "ROM (bus order)", "family", "crc", "Dallas label"], rows)

    out.note("The cache is filled by OW_SEARCH; OW_GET_DEVICES returns it without touching the bus.")
    return {
        "presence": True,
        "count": count,
        "devices": [
            {
                "rom": rom.hex(),
                "family": rom[0],
                "family_name": family_name(rom[0]),
                "crc_ok": rom_crc_ok(rom),
                "label": fmt_rom_dallas(rom),
            }
            for rom in devices
        ],
    }


def sc_readrom(bridge: Bridge, out: Out, args) -> dict:
    """Read ROM works only when exactly one device is on the bus."""
    out.title("Read ROM (0x33)")
    cached = find_devices(bridge, out)
    if len(cached) > 1:
        out.warn("this command is defined only for a single-device bus: %d devices "
                 "answer 0x33 at the same time, so the bytes below are a collision "
                 "pattern, not a ROM" % len(cached))

    rom = bridge.ow_read_rom()
    ok = rom_crc_ok(rom)
    out.kv("ROM", fmt_rom(rom))
    out.kv("family", "0x%02X %s" % (rom[0], family_name(rom[0])))
    out.kv("CRC-8", out.c.green("ok") if ok else out.c.red("bad"))
    out.kv("Dallas label", fmt_rom_dallas(rom))
    if len(cached) == 1 and not ok:
        out.warn("the only cached device answered, but the READ ROM frame has a bad CRC")
    return {
        "rom": rom.hex(),
        "crc_ok": ok,
        "label": fmt_rom_dallas(rom),
        "cached_devices": len(cached),
    }


def sc_temp(bridge: Bridge, out: Out, args) -> dict:
    devices = [rom for rom in find_devices(bridge, out) if rom[0] == 0x28]
    if not devices:
        out.error("no DS18B20 in the cache; run `scan` first")
        return {"devices": [], "error": "no DS18B20"}

    out.title("Convert T (broadcast)")
    t0 = time.perf_counter()
    resp = bridge.temp_convert()
    dt = time.perf_counter() - t0
    out.kv("status", resp.status_name)
    out.kv("busy polls", str(resp.polls))
    out.kv("elapsed", "%.3f s" % dt)
    out.note("A broadcast conversion returns OK even though the scratchpad read collides on a multi-drop bus.")

    out.title("Read all scratchpads")
    readings = bridge.temp_read_all()
    out.kv("values", str(len(readings)))
    if len(readings) != len(devices):
        out.warn("read back %d values for %d sensors" % (len(readings), len(devices)))

    rows = []
    detail = []
    for rom, word in zip(devices, readings):
        celsius = decode_temp(word)
        entry = {
            "rom": rom.hex(),
            "label": fmt_rom_dallas(rom),
            "word": "0x%04X" % word,
            "celsius": round(celsius, 4),
        }
        if not args.fast:
            try:
                bits = bridge.temp_get_res(rom)
                th, tl = bridge.temp_get_alarm(rom)
                parasite = bridge.temp_detect_parasite(rom)
                entry.update({
                    "resolution": bits,
                    "th": th,
                    "tl": tl,
                    "parasite": parasite,
                })
                res = "%d bit (%.4g C/bit, %d ms)" % (bits, RES_STEP.get(bits, 0), CONVERT_MS.get(bits, 0))
            except BridgeError as exc:
                res = "? (%s)" % exc
                entry["resolution"] = None
            rows.append([fmt_rom_dallas(rom), "%.4f" % celsius, res,
                         "%+d / %+d" % (entry.get("th", 0), entry.get("tl", 0)),
                         "yes" if entry.get("parasite") else "no"])
        else:
            rows.append([fmt_rom_dallas(rom), "%.4f" % celsius])
        detail.append(entry)

    out.title("Temperatures")
    if args.fast:
        out.table(["ROM", "C"], rows)
    else:
        out.table(["ROM", "C", "resolution", "TH/TL", "parasite"], rows)

    values = [entry["celsius"] for entry in detail]
    return {
        "convert": {"status": resp.status_name, "polls": resp.polls, "elapsed_s": round(dt, 4)},
        "devices": detail,
        "min_c": min(values) if values else None,
        "max_c": max(values) if values else None,
    }


def sc_watch(bridge: Bridge, out: Out, args) -> dict:
    devices = [rom for rom in find_devices(bridge, out) if rom[0] == 0x28]
    if not devices:
        out.error("no DS18B20 in the cache")
        return {"devices": [], "error": "no DS18B20"}

    if not out.json_mode:
        print()
        print(out.c.bold("Live temperatures (Ctrl+C to stop)"))
        print(out.c.dim("  " + "  ".join("%17s" % fmt_rom_dallas(rom) for rom in devices)))

    history = []
    cycles = 0
    try:
        while args.count == 0 or cycles < args.count:
            cycles += 1
            stats = bridge.stats_get()
            bridge.temp_convert()
            readings = bridge.temp_read_all()
            values = [round(decode_temp(w), 4) for w in readings]
            stamp = time.strftime("%H:%M:%S")
            if not out.json_mode:
                cells = ["%.2f" % value for value in values]
                line = ("  %s  uptime %-5d  %s"
                        % (stamp, stats["uptime_s"], "  ".join(cell.center(17) for cell in cells)))
                print(line.rstrip(), flush=True)
            history.append({"t": stamp, "values": values, "uptime_s": stats["uptime_s"]})
            if args.count and cycles >= args.count:
                break
            time.sleep(max(0.0, args.interval))
    except KeyboardInterrupt:
        if not out.json_mode:
            print(out.c.dim("\n  stopped"))
    return {"devices": [fmt_rom_dallas(rom) for rom in devices], "samples": history, "cycles": cycles}


def sc_eeprom(bridge: Bridge, out: Out, args) -> dict:
    roms = [rom for rom in find_devices(bridge, out) if rom[0] == 0x2D]
    if not roms:
        out.error("no DS2431 in the cache")
        return {"error": "no DS2431"}
    rom = roms[0]

    if args.write:
        if not args.yes:
            out.error("--write changes device memory; pass --yes to confirm")
            return {"error": "confirmation required"}
        addr, payload = parse_write(args.write)
        block = addr & ~7
        if addr + len(payload) > block + 8:
            out.error("the DS2431 programs one 8-byte block at a time; "
                      "keep addr:len inside 0x%02X..0x%02X" % (block, block + 7))
            return {"error": "write crosses an 8-byte block boundary"}
        # A short Write Scratchpad leaves the rest of the block stale, sets the
        # device PF flag and makes Copy Scratchpad refuse the block, so merge
        # the requested bytes into the current block and write all 8.
        current = bridge.eeprom_read(rom, block, 8)
        merged = bytearray(current)
        offset = addr - block
        merged[offset:offset + len(payload)] = payload
        out.title("Write EEPROM")
        out.kv("ROM", fmt_rom_dallas(rom))
        out.kv("block", "0x%02X..0x%02X" % (block, block + 7))
        out.kv("was", bcd_hex(current))
        out.kv("write", bcd_hex(bytes(merged)))
        bridge.eeprom_write(rom, block, bytes(merged))
        readback = bridge.eeprom_read(rom, block, 8)
        out.kv("read back", bcd_hex(readback))
        out.kv("status", "verified" if readback == bytes(merged) else out.c.red("MISMATCH"))
        return {
            "rom": rom.hex(),
            "block": block,
            "written": bytes(merged).hex(),
            "read_back": readback.hex(),
            "verified": readback == bytes(merged),
        }

    if args.read:
        addr, length = parse_read(args.read)
        addr = max(0, min(addr, DS2431_EEPROM_SIZE - 1))
        length = max(1, min(length, DS2431_EEPROM_SIZE - addr))
        out.title("Read EEPROM")
        out.kv("ROM", fmt_rom_dallas(rom))
        data = read_eeprom_range(bridge, rom, addr, length)
        out.hexdump(data, addr)
        return {"rom": rom.hex(), "address": addr, "data": data.hex()}

    out.title("EEPROM dump (128 bytes)")
    out.kv("ROM", fmt_rom_dallas(rom))
    t0 = time.perf_counter()
    data = read_eeprom_range(bridge, rom, 0, DS2431_EEPROM_SIZE)
    dt = time.perf_counter() - t0
    out.kv("read in", "%.3f s (%d bytes in 8-byte passes)" % (dt, len(data)))
    out.hexdump(data)
    out.title("Scratchpad")
    scratch = bridge.eeprom_read_scratchpad(rom)
    out.kv("frame", bcd_hex(scratch))
    out.note("The frame is TA1, TA2, ES and the last written page; the driver verifies its CRC-16.")
    return {
        "rom": rom.hex(),
        "eeprom": data.hex(),
        "scratchpad": scratch.hex(),
        "elapsed_s": round(dt, 4),
    }


def parse_write(text: str):
    addr_text, _, hex_text = text.partition(":")
    addr = int(addr_text, 0)
    payload = bytes(int(part, 16) for part in hex_text.replace(",", " ").split())
    if not 1 <= len(payload) <= 8:
        raise ValueError("the DS2431 writes 1..8 bytes per page")
    if not 0 <= addr < DS2431_EEPROM_SIZE:
        raise ValueError("address must be 0..127")
    return addr, payload


def parse_read(text: str):
    addr_text, _, len_text = text.partition(":")
    addr = int(addr_text, 0)
    length = int(len_text, 0) if len_text else DS2431_PAGE_SIZE
    return addr, length


def read_eeprom_range(bridge: Bridge, rom: bytes, addr: int, length: int) -> bytes:
    out = bytearray()
    while len(out) < length:
        chunk = min(8, length - len(out))
        out += bridge.eeprom_read(rom, addr + len(out), chunk)
    return bytes(out)


def sc_io(bridge: Bridge, out: Out, args) -> dict:
    roms = [rom for rom in find_devices(bridge, out) if rom[0] == 0x3A]
    if not roms:
        out.error("no DS2413 in the cache")
        return {"error": "no DS2413"}
    rom = roms[0]

    if args.set:
        states = parse_pio(args.set)
        out.title("Write PIO latches")
        out.kv("ROM", fmt_rom_dallas(rom))
        out.kv("states", "0x%02X (A=%d B=%d)" % (states, states & 1, (states >> 1) & 1))
        bridge.io_write(rom, states)
        out.kv("status", "written")

    pio, latches = bridge.io_read(rom)
    out.title("PIO state")
    out.kv("pins", "A=%d B=%d" % (pio & 1, (pio >> 1) & 1))
    out.kv("latches", "A=%d B=%d" % (latches & 1, (latches >> 1) & 1))
    if pio != latches:
        driven = []
        for name, mask in (("A", 0x01), ("B", 0x02)):
            if (pio & mask) != (latches & mask):
                driven.append(name)
        out.note("Pin%s %s differ%s from the latch: driven externally."
                 % ("" if len(driven) == 1 else "s", "/".join(driven),
                    "s" if len(driven) == 1 else ""))
    return {
        "rom": rom.hex(),
        "pio": pio,
        "latches": latches,
        "pins": {"A": pio & 1, "B": (pio >> 1) & 1},
        "latches_value": {"A": latches & 1, "B": (latches >> 1) & 1},
    }


def parse_pio(text: str) -> int:
    states = 0
    for part in text.replace(",", " ").split():
        name, _, value = part.partition("=")
        bit = {"a": 0, "b": 1}.get(name.strip().lower())
        if bit is None or value.strip() not in ("0", "1"):
            raise ValueError("expected A=0|1 or B=0|1, got %r" % part)
        if value.strip() == "1":
            states |= 1 << bit
    return states


def sc_raw(bridge: Bridge, out: Out, args) -> dict:
    cmd = int(args.command, 0)
    payload = parse_bytes(args.data)
    out.title("Raw command")
    out.kv("command", "0x%02X %s" % (cmd, CMD_NAMES.get(cmd, "(unknown)")))
    out.kv("data", bcd_hex(payload) if payload else "-")
    resp = bridge.command(cmd, payload, timeout=args.timeout)
    out.kv("status", "%s (0x%02X)" % (resp.status_name, resp.status))
    out.kv("echo cmd", "0x%02X %s" % (resp.cmd, CMD_NAMES.get(resp.cmd, "(unknown)")))
    out.kv("length", str(len(resp.data)))
    out.kv("polls", str(resp.polls))
    out.kv("nacks", str(resp.nacks))
    out.kv("elapsed", "%.6f s" % resp.elapsed)
    if resp.data:
        out.title("Response data")
        out.hexdump(resp.data)
    return {
        "command": cmd,
        "status": resp.status,
        "status_name": resp.status_name,
        "echo_cmd": resp.cmd,
        "data": resp.data.hex(),
        "polls": resp.polls,
        "nacks": resp.nacks,
        "elapsed_s": round(resp.elapsed, 6),
    }


def sc_queue(bridge: Bridge, out: Out, args) -> dict:
    out.title("Command queue (depth 2)")
    out.note("Submit a 750 ms broadcast Convert T, then OW_GET_COUNT while it runs.")
    out.note("A queued command reports BUSY with its own command code in the header.")

    bridge._tx(bytes([0x20, 0x00]))   # long: broadcast Convert T, starts now
    time.sleep(0.01)
    bridge._tx(bytes([0x0F, 0x00]))   # queued: OW_GET_COUNT

    t0 = time.monotonic()
    seen = []
    last = None
    deadline = t0 + 10.0
    while time.monotonic() < deadline:
        hdr = bridge._rx(3)
        if hdr[0] == 0xFF and hdr[1] == 0xFF:
            continue
        if hdr[0] == BR_BUSY:
            frame = hdr
        else:
            length = min(hdr[2], BR_MAX_RESP)
            frame = bridge._rx(3 + length) if length else hdr
            if frame[:3] != hdr:
                continue
        key = bytes(frame[:3]) + bytes(frame[3:3 + hdr[2]])
        if key != last:
            last = key
            seen.append({
                "t_ms": round((time.monotonic() - t0) * 1000.0, 1),
                "status": STATUS_NAMES.get(frame[0], "0x%02X" % frame[0]),
                "cmd": "0x%02X" % frame[1],
                "name": CMD_NAMES.get(frame[1], "?"),
                "data": frame[3:3 + hdr[2]].hex(),
            })
        if frame[0] != BR_BUSY and frame[1] == 0x0F:
            break
        time.sleep(0.001)

    out.table(["t+ms", "status", "cmd", "name", "data"], [
        [item["t_ms"], item["status"], item["cmd"], item["name"], item["data"] or "-"]
        for item in seen
    ])
    out.note("Convert T's own response was replaced by the queued command before")
    out.note("the host could read it: the bridge keeps only the latest response.")

    # Queue full: two entries fit behind the running command, the third is
    # rejected immediately.
    out.title("Queue full")
    bridge._tx(bytes([0x20, 0x00]))
    time.sleep(0.01)
    bridge._tx(bytes([0x0F, 0x00]))
    bridge._tx(bytes([0x0F, 0x00]))
    bridge._tx(bytes([0x0F, 0x00]))
    full = None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        hdr = bridge._rx(3)
        if hdr[0] == 0xFF and hdr[1] == 0xFF:
            continue
        if hdr[0] == BR_QUEUE_FULL:
            full = {
                "status": "QUEUE_FULL",
                "cmd": "0x%02X" % hdr[1],
                "name": CMD_NAMES.get(hdr[1], "?"),
            }
            break
        time.sleep(0.001)
    if full is None:
        out.warn("QUEUE_FULL was not observed")
    else:
        out.kv("rejected", "%s %s (depth 2 is already occupied)" % (full["cmd"], full["name"]))

    # Let the two queued OW_GET_COUNT commands drain so the next process sees
    # an idle bus (the running Convert T takes 750 ms).
    time.sleep(1.2)
    return {"responses": seen, "queue_full": full}


def sc_abort(bridge: Bridge, out: Out, args) -> dict:
    out.title("Queue drop via OW_ABORT")
    out.note("The library cannot cancel an in-flight hardware operation. OW_ABORT")
    out.note("discards what was queued behind it and then reports ABORTED.")
    bridge._tx(bytes([0x20, 0x00]))   # long: broadcast Convert T
    bridge._tx(bytes([0x0F, 0x00]))   # queued: OW_GET_COUNT
    bridge._tx(bytes([0x11, 0x00]))   # queued: OW_ABORT (drops the queue)

    seen = []
    deadline = time.monotonic() + 10.0
    last = None
    while time.monotonic() < deadline:
        hdr = bridge._rx(3)
        if hdr[0] == 0xFF and hdr[1] == 0xFF:
            continue
        if hdr[0] != BR_BUSY:
            length = min(hdr[2], BR_MAX_RESP)
            frame = bridge._rx(3 + length)
            key = frame.hex()
            if key != last:
                last = key
                seen.append({
                    "status": STATUS_NAMES.get(frame[0], "0x%02X" % frame[0]),
                    "cmd": "0x%02X" % frame[1],
                    "name": CMD_NAMES.get(frame[1], "?"),
                })
                if frame[0] == BR_ABORTED:
                    break
        time.sleep(0.001)

    out.table(["#", "status", "cmd", "name"], [
        [i, item["status"], item["cmd"], item["name"]] for i, item in enumerate(seen)
    ])
    if not any(item["status"] == "ABORTED" for item in seen):
        out.warn("ABORTED was not observed")
    return {"responses": seen}


def sc_timing(bridge: Bridge, out: Out, args) -> dict:
    plan = []
    if args.all:
        plan = [
            ("FIRMWARE_VER (instant)", 0x52, b"", 2.0),
            ("OW_GET_COUNT (instant)", 0x0F, b"", 2.0),
            ("OW_GET_STATUS (instant)", 0x0D, b"", 2.0),
            ("CONFIG_GET (instant)", 0x50, b"", 2.0),
            ("OW_RESET", 0x01, b"", 3.0),
            ("OW_READ_BYTE", 0x03, b"", 3.0),
            ("TEMP_READ_ALL", 0x22, b"", 10.0),
        ]
    else:
        plan = [("0x%02X" % int(args.command, 0), int(args.command, 0), parse_bytes(args.data), args.timeout)]

    results = []
    for label, cmd, payload, timeout in plan:
        times = []
        polls = []
        statuses = []
        for _ in range(args.count):
            resp = bridge.command(cmd, payload, timeout)
            if resp.cmd != cmd:
                raise BridgeError("0x%02X answered for 0x%02X" % (cmd, resp.cmd))
            times.append(resp.elapsed)
            polls.append(resp.polls)
            statuses.append(resp.status_name)
        times.sort()
        entry = {
            "command": label,
            "count": len(times),
            "status": "/".join(sorted(set(statuses))),
            "min_ms": round(times[0] * 1000, 3),
            "avg_ms": round(statistics.mean(times) * 1000, 3),
            "p95_ms": round(times[min(len(times) - 1, int(0.95 * len(times)))] * 1000, 3),
            "max_ms": round(times[-1] * 1000, 3),
            "polls_avg": round(statistics.mean(polls), 1),
        }
        results.append(entry)

    out.title("Command latency (%d samples each)" % args.count)
    out.table(
        ["command", "status", "min ms", "avg ms", "p95 ms", "max ms", "polls"],
        [[r["command"], r["status"], r["min_ms"], r["avg_ms"], r["p95_ms"], r["max_ms"], r["polls_avg"]]
         for r in results],
    )
    out.note("Instant commands complete inside bridge_submit(); the poll count stays at 1.")
    out.note("Long commands keep BUSY and are polled by the main loop.")
    return {"samples": results}


def sc_load(bridge: Bridge, out: Out, args) -> dict:
    """Saturate the bus with write + 163-byte reads, like the stress harness."""
    out.title("Bus load (%d s)" % args.seconds)
    version = bridge.firmware_ver().encode("ascii", "replace")
    ver = bytes([0x00, 0x52, len(version)]) + version
    read_len = 163
    rounds = 0
    bad = 0
    t0 = time.perf_counter()
    deadline = t0 + args.seconds
    while time.perf_counter() < deadline:
        # The adapter can lose the address phase of the bulk read and return
        # all FF. That is a transport NACK, not a firmware error: re-send the
        # command once instead of recording a bad response.
        raw = b""
        for _attempt in range(2):
            bridge._tx(bytes([0x52, 0x00]))
            raw = bridge._rx(read_len)
            if raw[:len(ver)] == ver:
                break
        if raw[:len(ver)] != ver or raw[len(ver):] != b"\xff" * (read_len - len(ver)):
            bad += 1
        rounds += 1
    dt = time.perf_counter() - t0
    stats = bridge.stats_get()
    out.kv("rounds", str(rounds))
    out.kv("throughput", "%.1f rounds/s, %.1f KB/s" % (rounds / dt, rounds * read_len / dt / 1000.0))
    out.kv("bad responses", out.c.green("0") if bad == 0 else out.c.red(str(bad)))
    out.kv("firmware commands", str(stats["commands"]))
    out.kv("firmware errors", str(stats["crc_error"] + stats["bus_short"] + stats["timeout"]))
    return {
        "seconds": round(dt, 3),
        "rounds": rounds,
        "bytes_per_s": round(rounds * read_len / dt, 1),
        "bad": bad,
        "stats": stats,
    }


def sc_selftest(bridge: Bridge, out: Out, args) -> dict:
    checks = []

    def check(name, fn):
        try:
            detail = fn()
            checks.append({"check": name, "ok": True, "detail": detail})
            if not out.json_mode:
                print("  %s %-22s %s" % (out.c.green("PASS"), name, detail))
        except Exception as exc:  # noqa: BLE001 - the report needs every failure
            checks.append({"check": name, "ok": False, "detail": str(exc)})
            if not out.json_mode:
                print("  %s %-22s %s" % (out.c.red("FAIL"), name, exc))

    out.title("Self test")
    check("firmware version", lambda: bridge.firmware_ver())
    check("config crc", lambda: "ok" if bridge.config_get()["crc_ok"] else "bad")
    check("bus presence", lambda: "present" if bridge.ow_reset() else "no device")

    def search():
        count = bridge.ow_search(0xF0, 0, 16)
        return "%d devices" % count
    check("search", search)

    def roms():
        roms = bridge.ow_get_devices(0, 16)
        bad = [fmt_rom(r) for r in roms if not rom_crc_ok(r)]
        if bad:
            raise BridgeError("bad ROM CRC: " + ", ".join(bad))
        return "%d valid" % len(roms)
    check("rom crc", roms)

    def temp():
        bridge.temp_convert()
        values = bridge.temp_read_all()
        if not values:
            raise BridgeError("no readings")
        return ", ".join("%.2f C" % decode_temp(v) for v in values)
    check("temperature", temp)

    def eeprom():
        rom = next((r for r in bridge.ow_get_devices(0, 16) if r[0] == 0x2D), None)
        if rom is None:
            return "skipped (no DS2431)"
        data = read_eeprom_range(bridge, rom, 0, 8)
        return "%d bytes" % len(data)
    check("eeprom", eeprom)

    def pio():
        rom = next((r for r in bridge.ow_get_devices(0, 16) if r[0] == 0x3A), None)
        if rom is None:
            return "skipped (no DS2413)"
        pio_value, latches = bridge.io_read(rom)
        return "pins=0x%02X latches=0x%02X" % (pio_value, latches)
    check("pio", pio)

    ok = all(item["ok"] for item in checks)
    if not out.json_mode:
        print()
        print("  " + (out.c.green("RESULT: PASS") if ok else out.c.red("RESULT: FAIL")))
    return {"result": "PASS" if ok else "FAIL", "checks": checks}


# --------------------------------------------------------------------------- #
# Command line
# --------------------------------------------------------------------------- #


def parse_bytes(text: str) -> bytes:
    if not text:
        return b""
    cleaned = text.replace("0x", "").replace(",", " ").replace(":", " ")
    try:
        return bytes(int(part, 16) for part in cleaned.split())
    except ValueError:
        raise argparse.ArgumentTypeError("expected hex bytes, e.g. F0,00,10")


def add_common(parser, suppress: bool):
    def default(value):
        return argparse.SUPPRESS if suppress else value

    parser.add_argument("--addr", default=default("0x30"), help="I2C slave address (default 0x30)")
    parser.add_argument("--speed", default=default("100k"), choices=SPEED_CHOICES,
                        help="CH341A I2C clock (default 100k); 400k+ is not supported by the slave")
    parser.add_argument("--dll", default=default(DEFAULT_DLL), help="path to CH341DLLA64.DLL")
    parser.add_argument("--timeout", type=float, default=default(3.0), help="command BUSY timeout in seconds")
    parser.add_argument("--json", action="store_true", default=default(False), help="machine-readable output")
    parser.add_argument("--verbose", action="store_true", default=default(False), help="print every I2C frame")
    parser.add_argument("--no-color", action="store_true", default=default(False), help="disable ANSI colours")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Demo of the owbridge 1-Wire bridge over a CH341A adapter.",
        epilog="Every scenario also accepts the global options (--addr, --speed, --json, ...).",
    )
    add_common(parser, suppress=False)
    sub = parser.add_subparsers(dest="scenario", metavar="SCENARIO")
    sub.required = True

    def add(name, func, help_text):
        p = sub.add_parser(name, help=help_text, parents=[])
        add_common(p, suppress=True)
        p.set_defaults(func=func)
        return p

    add("info", sc_info, "firmware, configuration, status and statistics")
    add("scan", sc_scan, "search the bus and list every device")
    add("readrom", sc_readrom, "read the ROM of a single-device bus")
    p = add("temp", sc_temp, "convert and read all DS18B20 sensors")
    p.add_argument("--fast", action="store_true", help="skip the per-device resolution/alarm queries")
    p = add("watch", sc_watch, "live temperature table until Ctrl+C")
    p.add_argument("--interval", type=float, default=1.0, help="seconds between samples (default 1.0)")
    p.add_argument("--count", type=int, default=0, help="stop after N samples (0 = forever)")
    p = add("eeprom", sc_eeprom, "dump, read or write the DS2431 EEPROM")
    p.add_argument("--dump", action="store_true", help="dump all 128 bytes (default)")
    p.add_argument("--read", metavar="ADDR:LEN", help="read a range, e.g. 0x10:8")
    p.add_argument("--write", metavar="ADDR:HEX", help="write up to 8 bytes, e.g. 0x10:DE,AD,BE,EF")
    p.add_argument("--yes", action="store_true", help="confirm a write")
    p = add("io", sc_io, "read or drive the DS2413 PIO pins")
    p.add_argument("--set", metavar="A=0,B=1", help="write the output latches")
    p = add("raw", sc_raw, "send any command and dump the response")
    p.add_argument("command", help="command byte, e.g. 0x09")
    p.add_argument("--data", default="", help="hex payload, e.g. F0,00,10")
    add("queue", sc_queue, "show the command queue in action")
    add("abort", sc_abort, "show what OW_ABORT does to the queue")
    p = add("timing", sc_timing, "measure command latency")
    p.add_argument("--count", type=int, default=20, help="samples per command (default 20)")
    p.add_argument("--all", action="store_true", help="measure a fixed set of commands")
    p.add_argument("--command", default="0x52", help="command byte to measure (default 0x52)")
    p.add_argument("--data", default="", help="hex payload")
    p = add("load", sc_load, "saturate the bus with write + 163-byte reads")
    p.add_argument("--seconds", type=float, default=10.0, help="duration (default 10 s)")
    add("selftest", sc_selftest, "run a quick end-to-end check")
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

    color = sys.stdout.isatty() and not args.no_color and not args.json
    out = Out(color=color, json_mode=args.json)

    trace = None
    if args.verbose and not args.json:
        def trace(wbytes, rdata, direction=None, frame=None):
            if direction == "TX":
                print("  " + out.c.dim("TX %02X %s" % ((addr << 1), frame.hex(" "))))
            elif direction == "RX":
                print("  " + out.c.dim("RX %s" % rdata.hex(" ")))

    try:
        with Ch341I2C(args.dll, speed=SPEEDS[args.speed]) as io:
            bridge = Bridge(io, addr=addr, trace=trace)
            if not args.json:
                print(out.c.bold("owbridge demo") + out.c.dim("  addr 0x%02X  %s  %s"
                                                               % (addr, args.speed, args.dll)))
            result = args.func(bridge, out, args)
            if args.json:
                print(json.dumps(result, indent=2, sort_keys=True))
            return 0
    except Ch341Error as exc:
        out.error(str(exc))
        return 1
    except BridgeStatus as exc:
        out.error(str(exc))
        return 1
    except BridgeTimeout as exc:
        out.error(str(exc))
        return 1
    except BridgeError as exc:
        out.error(str(exc))
        return 1
    except ValueError as exc:
        out.error(str(exc))
        return 2


if __name__ == "__main__":
    sys.exit(main())
