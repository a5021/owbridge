#!/usr/bin/env python3
"""Strict soak/regression harness for owbridge over the F103 adapter.

Unlike the smoke harness it validates *semantics* on every cycle, runs
negative controls that prove it can actually detect failures, and records
per-cycle deltas of both the slave and the adapter statistics (including the
raw NACK counters and the firmware retry counter).

Exit code 0 only when every check passed.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time

F103_TOOLS = os.environ.get("F103_TOOLS", r"C:/Users/mr.smith/F103_CDC_I2C/tools")
sys.path.insert(0, F103_TOOLS)
from f103_adapter import (F103I2C, F103Error, build_frame, CMD_PING,  # noqa: E402
                          CMD_WRITE, parse_frame)

KEYS = ("resets", "presence_fail", "crc_error", "bus_short", "timeout",
        "commands", "aborts", "read_fail", "uptime_s")


class Runner:
    def __init__(self, args):
        self.args = args
        self.base = [sys.executable, "tools/owbridge_demo.py",
                     "--adapter", "f103", "--port", args.port,
                     "--addr", hex(args.addr), "--speed", "100k",
                     "--timeout", "15", "--json", "--no-color"]
        self.failures = []

    def demo(self, cmd, timeout=180):
        r = subprocess.run(self.base + cmd, capture_output=True, text=True,
                           timeout=timeout)
        if r.returncode != 0:
            raise RuntimeError("%s rc=%d: %s" % (cmd, r.returncode,
                                                 (r.stderr or r.stdout)[-200:]))
        return json.loads(r.stdout)

    def adapter(self):
        a = F103I2C(self.args.port, self.args.addr, 100, timeout=5)
        return a


def check(cond, name, detail="", failures=None):
    if cond:
        print("  PASS %-42s %s" % (name, detail))
    else:
        print("  FAIL %-42s %s" % (name, detail))
        if failures is not None:
            failures.append("%s: %s" % (name, detail))
    return cond


def scan_ok(data, golden):
    roms = [d["rom"] for d in data["devices"]]
    return (data["count"] == len(golden) and all(d["crc_ok"] for d in data["devices"])
            and sorted(roms) == sorted(golden))


def temp_ok(data):
    vals = [d["celsius"] for d in data["devices"] if d.get("celsius") is not None]
    if len(vals) != len(data["devices"]) or not vals:
        return False, "missing readings"
    return True, "spread %.2f C" % (max(vals) - min(vals))


def selftest_ok(data):
    return all(c["ok"] for c in data["checks"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto")
    ap.add_argument("--addr", default="0x30")
    ap.add_argument("--seconds", type=int, default=1800)
    ap.add_argument("--load-seconds", type=int, default=20)
    ap.add_argument("--csv", default="stress_strict.csv")
    args = ap.parse_args()
    args.addr = int(args.addr, 0)
    run = Runner(args)
    fails = run.failures
    t0 = time.time()

    print("== preflight ==")
    a = run.adapter()
    try:
        print("  firmware:", a.ping())
        a.bus_clear()
        print("  adapter stats:", a.stats())
    finally:
        a.close()
    info = run.demo(["info"])
    check(info["config"]["crc_ok"] and info["config"]["magic_ok"], "config valid")
    golden = sorted(d["rom"] for d in run.demo(["scan"])["devices"])
    check(len(golden) == 9, "baseline device count", str(golden))

    print("== negative controls ==")
    a = run.adapter()
    a.bus_clear()
    try:
        try:
            a._exchange(CMD_WRITE, bytes([0x31, 0x52, 0x00]))
            check(False, "wrong address is rejected", "no error raised", fails)
        except F103Error as exc:
            check(exc.status == 0x01, "wrong address is NACKed", "status=%s" % exc.status, fails)

        # corrupt CRC must be dropped without executing anything
        seq = a.seq
        bad = bytearray(build_frame(seq, CMD_PING))
        bad[-1] ^= 0xFF
        a.ser.write(bytes(bad))
        dropped = False
        try:
            a._read_exact(5)
        except F103Error:
            dropped = True
        check(dropped, "corrupt CRC is dropped", "", fails)
        check(a.ping()[0] == 256, "adapter alive after corrupt frame", "", fails)

        # duplicate frame must execute once
        before = a.stats()["txn"]
        f = build_frame(seq, CMD_WRITE, bytes([args.addr, 0x52, 0x00]))
        for _ in range(2):
            a.ser.write(f)
            raw = a._read_exact(5)
            n = int.from_bytes(raw[3:5], "little")
            a._read_exact(n + 2)
        delta = a.stats()["txn"] - before
        check(delta == 1, "duplicate frame executes once", "txn_delta=%d" % delta, fails)

        # a payload past PAYLOAD_MAX is dropped by the frame parser
        try:
            a._exchange(CMD_WRITE, bytes([args.addr]) + bytes(600))
            check(False, "oversized payload rejected", "accepted", fails)
        except F103Error as exc:
            check(exc.status in (None, 0x06, 0x08, 0x05), "oversized payload rejected",
                  "status=%s" % exc.status, fails)

        # abort during a long command, bus must stay usable
        rom = bytes.fromhex(golden[0])
        a._exchange(CMD_WRITE, bytes([args.addr, 0x20, len(rom)]) + rom)  # TEMP_CONVERT
        try:
            a._exchange(CMD_WRITE, bytes([args.addr, 0x11, 0x00]))  # OW_ABORT
        except F103Error:
            pass
        a.bus_clear()
        check(a.ping()[0] == 256, "bus usable after abort", "", fails)
    finally:
        a.close()
    check(scan_ok(run.demo(["scan"]), [d for d in golden]),
          "scan recovers after controls", "", fails)

    print("== soak %d s ==" % args.seconds)

    def adapter_stats():
        a = run.adapter()
        try:
            return a.stats()
        finally:
            a.close()

    base_slave = run.demo(["info"])["stats"]
    base_adapt = adapter_stats()
    rows = []
    cycle = 0
    with open(args.csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["cycle", "t_s", "scan", "roms", "temp", "spread", "io",
                    "selftest", "info", "uptime_s", "commands", "nack_a",
                    "nack_d", "retries", "timeout", "crc", "read_fail",
                    "aborts"])
        while time.time() - t0 < args.seconds:
            cycle += 1
            try:
                run.demo(["load", "--seconds", str(args.load_seconds)], timeout=300)
                s = run.demo(["scan"])
                scan = scan_ok(s, golden)
                check(scan, "cycle %d scan" % cycle, "count=%d" % s["count"], fails)

                t = run.demo(["temp"])
                tok, td = temp_ok(t)
                check(tok, "cycle %d temp" % cycle, td, fails)

                io = run.demo(["io"])
                check(io["latches"] == 3, "cycle %d io" % cycle,
                      "latches=%s" % io["latches"], fails)

                st = run.demo(["selftest"])
                check(selftest_ok(st), "cycle %d selftest" % cycle, "", fails)

                inf = run.demo(["info"])
                check(inf["config"]["crc_ok"], "cycle %d config" % cycle, "", fails)
                check(inf["status"]["devices"] == 9, "cycle %d device count" % cycle,
                      str(inf["status"]["devices"]), fails)

                cur_slave = inf["stats"]
                cur_adapt = adapter_stats()
                d_slave = {k: cur_slave[k] - base_slave[k] for k in KEYS}
                d_adapt = {k: cur_adapt[k] - base_adapt[k] for k in cur_adapt}
                check(d_slave["crc_error"] == 0, "cycle %d crc_error" % cycle,
                      str(d_slave["crc_error"]), fails)
                check(d_slave["read_fail"] == 0, "cycle %d read_fail" % cycle,
                      str(d_slave["read_fail"]), fails)
                check(d_slave["aborts"] == 0, "cycle %d aborts" % cycle,
                      str(d_slave["aborts"]), fails)
                check(cur_slave["uptime_s"] > base_slave["uptime_s"],
                      "cycle %d uptime" % cycle, str(cur_slave["uptime_s"]), fails)

                spread = td.split()[-2] if "spread" in td else ""
                w.writerow([cycle, "%.0f" % (time.time() - t0), int(scan),
                            s["count"], int(tok), spread, io["latches"],
                            int(selftest_ok(st)), inf["config"]["crc_ok"],
                            cur_slave["uptime_s"], d_slave["commands"],
                            d_adapt["nack_addr"], d_adapt["nack_data"],
                            d_adapt["retries"], d_slave["timeout"],
                            d_slave["crc_error"], d_slave["read_fail"],
                            d_slave["aborts"]])
            except Exception as exc:  # noqa: BLE001 - a cycle failure is data
                fails.append("cycle %d exception: %s" % (cycle, exc))
                print("  FAIL cycle %d exception: %s" % (cycle, exc))
                try:
                    a2 = run.adapter()
                    a2.bus_clear()
                    a2.close()
                except Exception:
                    pass

    print("== summary ==")
    print("cycles:", cycle, "failures:", len(fails))
    for f in fails[:20]:
        print(" -", f)
    print("csv:", args.csv)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
