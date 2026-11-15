#!/usr/bin/env python3
"""Scan-mode host example: measure every DS18B20 on a fixed cadence.

This is the host-side counterpart of ``examples/4_scan_mode`` in
``stm32-async-1wire``: program one resolution on all sensors with a broadcast
Write Scratchpad, then once per interval issue one broadcast Convert T and
read every scratchpad back. The point of the script is not the temperatures
but the transport cost of that cycle: each command reports how many BUSY polls
it took and how many of those polls were lost (an all-``FF`` read, the
software slave was inside a ``bridge_poll()`` step when the START arrived).
See ``docs/i2c-timing.md`` for the mechanism.

The tool reuses ``owbridge_demo.py`` for the CH341A transport and the command
level API, so it measures exactly what a normal host sees.

Examples
--------
    python owbridge_scan.py                          # 100 kHz, gap 0, 20 cycles
    python owbridge_scan.py --poll-gap 250           # 250 us between BUSY polls
    python owbridge_scan.py --speed 20k --cycles 50
    python owbridge_scan.py --interval 2 --json

Requirements: Windows with the CH341A driver, Python 3.8+, no third-party
packages.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time

import owbridge_demo as demo

# --------------------------------------------------------------------------- #
# DS18B20 scan-mode constants
# --------------------------------------------------------------------------- #

#: Function commands used by the broadcast resolution write (Skip ROM + Write
#: Scratchpad), matching ds18b20_resolution.c in the library.
DS18B20_WRITE_SCRATCHPAD = 0x4E
DS18B20_TH = 0x00
DS18B20_TL = 0x00

CONVERT_CMD = 0x20
READ_ALL_CMD = 0x22
SKIP_ROM_CMD = 0x07

#: Plausible DS18B20 range in 0.1 C units (-55 .. +125 C).
TEMP_MIN = -550
TEMP_MAX = 1250


def res_config(bits: int) -> int:
    """Configuration register byte for a resolution: 0x1F | R1R0 << 5."""
    return 0x1F | ((bits - 9) << 5)


# --------------------------------------------------------------------------- #
# Scan setup
# --------------------------------------------------------------------------- #


def program_resolution(bridge, bits: int, out) -> dict:
    """Broadcast Write Scratchpad so every sensor uses the same resolution.

    Skip ROM reaches all devices, which is what scan mode needs: the single
    conversion wait is only correct when the fleet is uniform.
    """
    payload = bytes([DS18B20_WRITE_SCRATCHPAD, DS18B20_TH, DS18B20_TL,
                     res_config(bits)])
    resp = bridge.command(SKIP_ROM_CMD, payload, timeout=10.0)
    if resp.status != demo.BR_OK:
        raise demo.BridgeStatus(SKIP_ROM_CMD, resp.status, "broadcast Write Scratchpad")
    out.kv("config write", "OK  %d B  %d polls  %d NACKs  %.1f ms"
           % (len(payload), resp.polls, resp.nacks, resp.elapsed * 1000.0))
    return {
        "status": resp.status,
        "polls": resp.polls,
        "nacks": resp.nacks,
        "elapsed_s": round(resp.elapsed, 6),
        "payload": payload.hex(),
    }


def verify_resolution(bridge, sensors, bits: int, out) -> dict:
    """Read the config register back through Match ROM, one sensor at a time."""
    seen = {}
    for rom in sensors:
        value = bridge.temp_get_res(rom)
        seen[value] = seen.get(value, 0) + 1
    uniform = set(seen) == {bits}
    out.kv("resolution", "%d bit on %d/%d sensor(s)%s"
           % (bits, seen.get(bits, 0), len(sensors),
              "" if uniform else "  [%s]" % ", ".join(
                  "%d bit x%d" % (k, v) for k, v in sorted(seen.items()))))
    return {"bits": bits, "seen": {str(k): v for k, v in sorted(seen.items())},
            "uniform": uniform}


# --------------------------------------------------------------------------- #
# One measurement cycle
# --------------------------------------------------------------------------- #


def run_cycle(bridge, sensors, timeout: float) -> dict:
    """Broadcast Convert T, wait for BUSY to clear, then read every scratchpad.

    ``sensors`` is the cached DS18B20 list in cache order. The response has one
    fixed 2-byte slot per sensor, so a sensor without a reading shows up as
    ``None`` at its own position instead of shifting its neighbours.
    """
    conv = bridge.command(CONVERT_CMD, b"", timeout=timeout)
    if conv.status != demo.BR_OK:
        raise demo.BridgeStatus(CONVERT_CMD, conv.status, "broadcast Convert T")

    read = bridge.command(READ_ALL_CMD, b"", timeout=timeout)
    if read.status != demo.BR_OK:
        raise demo.BridgeStatus(READ_ALL_CMD, read.status, "TEMP_READ_ALL")

    if len(read.data) != 2 * len(sensors):
        raise demo.BridgeError(
            "TEMP_READ_ALL returned %d byte(s) for %d sensor(s); firmware "
            "older than 1.1.0 drops failed readings instead of keeping "
            "their slot" % (len(read.data), len(sensors)))

    words = [int.from_bytes(read.data[i:i + 2], "big")
             for i in range(0, len(read.data), 2)]
    measured = [word for word in words if word != demo.BR_TEMP_INVALID]
    if any(word < TEMP_MIN or word > TEMP_MAX for word in measured):
        raise demo.BridgeError("temperature outside -55..+125 C: %s"
                               % ", ".join(str(word) for word in measured))
    missing = [demo.fmt_rom_dallas(rom)
               for rom, word in zip(sensors, words)
               if word == demo.BR_TEMP_INVALID]

    return {
        "convert": {"nacks": conv.nacks, "polls": conv.polls,
                    "elapsed_s": round(conv.elapsed, 6)},
        "read": {"nacks": read.nacks, "polls": read.polls,
                 "elapsed_s": round(read.elapsed, 6)},
        "words": words,
        "values": [None if word == demo.BR_TEMP_INVALID
                   else round(demo.decode_temp(word), 2) for word in words],
        "missing": missing,
        "nacks": conv.nacks + read.nacks,
        "polls": conv.polls + read.polls,
    }


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #


def pct(part: int, whole: int) -> float:
    return 100.0 * part / whole if whole else 0.0


def summarise(records, elapsed_s: float) -> dict:
    def total(metric: str) -> int:
        return sum(rec["nacks"] if metric == "nacks" else rec["polls"] for rec in records)

    conv_nacks = sum(rec["convert"]["nacks"] for rec in records)
    conv_polls = sum(rec["convert"]["polls"] for rec in records)
    read_nacks = sum(rec["read"]["nacks"] for rec in records)
    read_polls = sum(rec["read"]["polls"] for rec in records)
    latencies = [rec["convert"]["elapsed_s"] * 1000.0 for rec in records]
    read_lat = [rec["read"]["elapsed_s"] * 1000.0 for rec in records]
    missing = sum(len(rec["missing"]) for rec in records)
    cycles_missing = sum(1 for rec in records if rec["missing"])

    return {
        "cycles": len(records),
        "elapsed_s": round(elapsed_s, 3),
        "nacks": total("nacks"),
        "polls": total("polls"),
        "nack_pct": round(pct(total("nacks"), total("polls")), 3),
        "nacks_per_s": round(total("nacks") / elapsed_s, 2) if elapsed_s else 0.0,
        "nacks_per_cycle": round(total("nacks") / len(records), 1) if records else 0.0,
        "convert": {
            "nacks": conv_nacks,
            "polls": conv_polls,
            "nack_pct": round(pct(conv_nacks, conv_polls), 3),
            "nacks_per_cycle": round(conv_nacks / len(records), 1) if records else 0.0,
            "latency_ms": [round(min(latencies), 1), round(statistics.median(latencies), 1),
                           round(max(latencies), 1)] if latencies else [],
        },
        "read": {
            "nacks": read_nacks,
            "polls": read_polls,
            "nack_pct": round(pct(read_nacks, read_polls), 3),
            "nacks_per_cycle": round(read_nacks / len(records), 1) if records else 0.0,
            "latency_ms": [round(min(read_lat), 1), round(statistics.median(read_lat), 1),
                           round(max(read_lat), 1)] if read_lat else [],
        },
        "per_cycle_nacks": [rec["nacks"] for rec in records],
        "missing_readings": missing,
        "cycles_with_missing": cycles_missing,
    }


def print_header(args, addr: int, firmware: str, out) -> None:
    out.title("Scan mode")
    out.kv("firmware", firmware)
    out.kv("address", "0x%02X" % addr)
    out.kv("speed", "%s  poll gap %.0f us  short poll %s"
           % (args.speed, args.poll_gap or 0.0, args.short_poll))
    out.kv("cadence", "%.1f s x %s cycles"
           % (args.interval, args.cycles if args.cycles else "unlimited"))


def print_cycle(index: int, rec: dict, out) -> None:
    cells = ["%6.2f" % value if value is not None else "    --"
             for value in rec["values"]]
    temperatures = "  ".join(cells)
    print("  %3d  %s  %s  nack %3d+%3d=%-4d  polls %6d  %5.2f%%  %6.1f/%5.1f ms"
          % (index, time.strftime("%H:%M:%S"), temperatures,
             rec["convert"]["nacks"], rec["read"]["nacks"], rec["nacks"],
             rec["polls"], pct(rec["nacks"], rec["polls"]),
             rec["convert"]["elapsed_s"] * 1000.0,
             rec["read"]["elapsed_s"] * 1000.0),
          flush=True)


def print_summary(summary: dict, out) -> None:
    out.title("Summary")
    out.kv("cycles", "%d in %.1f s" % (summary["cycles"], summary["elapsed_s"]))
    out.kv("NACKs", "%d of %d polls (%.2f%%), %.1f per cycle, %.2f per s"
           % (summary["nacks"], summary["polls"], summary["nack_pct"],
              summary["nacks_per_cycle"], summary["nacks_per_s"]))
    for name in ("convert", "read"):
        part = summary[name]
        out.kv("  " + name, "%d NACKs / %d polls (%.2f%%), %.1f per cycle, "
                            "latency %.1f / %.1f / %.1f ms"
               % (part["nacks"], part["polls"], part["nack_pct"],
                  part["nacks_per_cycle"], *part["latency_ms"]))
    spread = summary["per_cycle_nacks"]
    if spread:
        out.kv("per cycle", "min %d  median %.1f  max %d"
               % (min(spread), statistics.median(spread), max(spread)))
    out.kv("readings", "%d slot(s) without a valid reading in %d cycle(s)"
           % (summary["missing_readings"], summary["cycles_with_missing"]))


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Poll every DS18B20 on a fixed cadence and count BUSY-poll "
                    "NACKs (host side of examples/4_scan_mode).",
        epilog="Expected at 100 kHz without a poll gap: about 150 NACKs per "
               "cycle, roughly 7 percent of the BUSY polls; zero at 20 kHz.",
    )
    parser.add_argument("--addr", default="0x30", help="I2C slave address (default 0x30)")
    parser.add_argument("--speed", default="100k", choices=demo.SPEED_CHOICES,
                        help="CH341A I2C clock (default 100k)")
    parser.add_argument("--dll", default=demo.DEFAULT_DLL, help="path to CH341DLLA64.DLL")
    parser.add_argument("--interval", type=float, default=5.0,
                        help="seconds between measurement cycles (default 5)")
    parser.add_argument("--cycles", type=int, default=20,
                        help="stop after N cycles, 0 = until Ctrl+C (default 20)")
    parser.add_argument("--resolution", default="12", choices=("9", "10", "11", "12", "off"),
                        help="broadcast resolution to program before the loop (default 12)")
    parser.add_argument("--poll-gap", type=float, default=None,
                        help="pause between BUSY polls, microseconds (default 0)")
    parser.add_argument("--short-poll", choices=("auto", "on", "off"), default="auto",
                        help="poll one status byte while BUSY (default: on at 20k, off at 100k)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="per-command BUSY timeout in seconds (default 10)")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    parser.add_argument("--no-color", action="store_true", help="disable ANSI colours")
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
    if args.interval <= 0.0:
        print("error: --interval must be positive", file=sys.stderr)
        return 2

    color = sys.stdout.isatty() and not args.no_color and not args.json
    out = demo.Out(color=color, json_mode=args.json)

    gap_us = 0.0 if args.poll_gap is None else args.poll_gap
    short_poll = (args.short_poll == "on") or (
        args.short_poll == "auto" and args.speed == "20k")
    bits = None if args.resolution == "off" else int(args.resolution)

    result = {
        "config": {
            "addr": "0x%02X" % addr,
            "speed": args.speed,
            "interval_s": args.interval,
            "cycles": args.cycles,
            "poll_gap_us": gap_us,
            "short_poll": short_poll,
            "resolution": args.resolution,
        },
        "firmware": None,
        "sensors": [],
        "resolution_write": None,
        "cycles": [],
        "summary": {},
        "checks": {},
    }

    try:
        with demo.Ch341I2C(args.dll, speed=demo.SPEEDS[args.speed]) as io:
            bridge = demo.Bridge(io, addr=addr, poll_gap=gap_us / 1e6,
                                 short_poll=short_poll)
            result["firmware"] = bridge.firmware_ver()
            if not args.json:
                print(out.c.bold("owbridge scan") + out.c.dim(
                    "  addr 0x%02X  %s  %s" % (addr, args.speed, args.dll)))
                print()
                print_header(args, addr, result["firmware"], out)

            devices = demo.find_devices(bridge, out)
            sensors = [rom for rom in devices if rom[0] == 0x28]
            bad_crc = [rom for rom in devices if not demo.rom_crc_ok(rom)]
            result["sensors"] = [demo.fmt_rom_dallas(rom) for rom in sensors]
            result["checks"]["devices"] = len(devices)
            result["checks"]["rom_crc"] = not bad_crc
            if not args.json:
                out.kv("devices", "%d total, %d DS18B20, %d bad CRC"
                       % (len(devices), len(sensors), len(bad_crc)))
                for rom in sensors:
                    print("    %s  %s" % (demo.fmt_rom_dallas(rom), demo.family_name(rom[0])))
            if not sensors:
                raise demo.BridgeError("no DS18B20 on the bus")
            if bad_crc:
                raise demo.BridgeError("%d ROM(s) with a bad CRC" % len(bad_crc))

            resolution = None
            if bits is not None:
                if not args.json:
                    print()
                write = program_resolution(bridge, bits, out)
                result["resolution_write"] = write
                resolution = verify_resolution(bridge, sensors, bits, out)
                result["checks"]["resolution_uniform"] = resolution["uniform"]

            if not args.json:
                print()
                print("  cycle  time      temperatures (C)             "
                      "NACK c+r=total   polls    rate   lat ms")
                print("  " + "-" * 106)

            records = []
            started = time.monotonic()
            started_str = time.strftime("%H:%M:%S")
            index = 0
            try:
                while args.cycles == 0 or index < args.cycles:
                    index += 1
                    record = run_cycle(bridge, sensors, args.timeout)
                    record["cycle"] = index
                    record["wall"] = time.strftime("%H:%M:%S")
                    records.append(record)
                    if not args.json:
                        print_cycle(index, record, out)
                    if args.cycles == 0 or index < args.cycles:
                        deadline = started + index * args.interval
                        delay = deadline - time.monotonic()
                        if delay > 0.0:
                            time.sleep(delay)
            except KeyboardInterrupt:
                if not args.json:
                    print(out.c.dim("\n  stopped after %d cycle(s)" % index))

            elapsed = time.monotonic() - started
            summary = summarise(records, elapsed)
            summary["started"] = started_str
            summary["finished"] = time.strftime("%H:%M:%S")
            result["cycles"] = records
            result["summary"] = summary

            # The device cache must not have changed while the loop ran.
            cached = bridge.ow_get_devices(0, 16)
            result["checks"]["cache_unchanged"] = cached == devices
            result["checks"]["all_cycles_ok"] = len(records) > 0
            result["checks"]["all_sensors_read"] = summary["missing_readings"] == 0

            if not args.json:
                print()
                print_summary(summary, out)
                if cached != devices:
                    out.warn("device cache changed during the run")
                if summary["missing_readings"]:
                    out.warn("%d reading(s) missing; the sensor kept its slot, "
                             "see the sensors above for the affected ROMs"
                             % summary["missing_readings"])
                if len(records) != index:
                    out.warn("only %d of %d cycles completed" % (len(records), index))

            ok = (result["checks"]["rom_crc"]
                  and result["checks"]["cache_unchanged"]
                  and result["checks"]["all_sensors_read"]
                  and result["checks"].get("resolution_uniform", True)
                  and result["checks"]["all_cycles_ok"])
            result["checks"]["pass"] = bool(ok)
            if not args.json:
                print()
                print("  RESULT: " + (out.c.green("PASS") if ok else out.c.red("FAIL")))
            if args.json:
                print(json.dumps(result, indent=2, sort_keys=True))
            return 0 if ok else 1

    except demo.Ch341Error as exc:
        out.error(str(exc))
        return 1
    except (demo.BridgeStatus, demo.BridgeTimeout, demo.BridgeError) as exc:
        out.error(str(exc))
        return 1


if __name__ == "__main__":
    sys.exit(main())
