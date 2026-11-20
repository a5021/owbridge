#!/usr/bin/env python3
"""Host-side unit tests for the pure protocol helpers.

These tests need neither a bridge nor a CH341A adapter: they cover the mapping
and summary logic that turns a TEMP_READ_ALL response into per-sensor values.
Run from the tools directory with ``python3 -m unittest -v test_host_helpers``.
"""

import unittest

import owbridge_demo as demo
import owbridge_scan as scan
from protocol_test import Spec, payload_for


def make_rom(index: int) -> bytes:
    return bytes([0x28, index, 0x11, 0x22, 0x33, 0x44, 0x55, index])


class TemperatureMappingTests(unittest.TestCase):
    def setUp(self):
        self.devices = [make_rom(i) for i in range(3)]

    def test_positional_mapping(self):
        pairs = demo.pair_temperatures(self.devices, [0x00E9, 0x8000, 0x0040])
        self.assertEqual([rom for rom, _ in pairs], self.devices)
        self.assertEqual([word for _, word in pairs], [0x00E9, None, 0x0040])

    def test_empty_cache(self):
        self.assertEqual(demo.pair_temperatures([], []), [])

    def test_length_mismatch_is_reported(self):
        # Pre-1.1.0 firmware compacts failed reads; positions cannot be trusted.
        for words in ([0x00E9, 0x8000], [0x00E9, 0x8000, 0x0040, 0x0050]):
            with self.assertRaises(demo.BridgeError):
                demo.pair_temperatures(self.devices, words)

    def test_invalid_slot_is_not_a_temperature(self):
        pairs = demo.pair_temperatures(self.devices[:1], [demo.BR_TEMP_INVALID])
        self.assertIsNone(pairs[0][1])

    def test_signed_decode(self):
        self.assertAlmostEqual(demo.decode_temp(0x00E9), 23.3)
        self.assertAlmostEqual(demo.decode_temp(0xFF38), -20.0)


class CommandSpecTests(unittest.TestCase):
    def test_broadcast_payload_at_data_zero(self):
        rom = make_rom(1)
        res = Spec(0x23, "TEMP_SET_RES", 1, 9, rom=rom,
                   sample=bytes([12]), special=(1, 9))
        self.assertEqual(payload_for(res, 1), bytes([12]))
        self.assertEqual(payload_for(res, 9), rom + bytes([12]))
        alarm = Spec(0x25, "TEMP_SET_ALARM", 2, 10, rom=rom,
                     sample=bytes([25, 15]), special=(2, 10))
        self.assertEqual(payload_for(alarm, 2), bytes([25, 15]))
        self.assertEqual(payload_for(alarm, 10), rom + bytes([25, 15]))

    def test_convert_broadcast_has_no_payload(self):
        spec = Spec(0x20, "TEMP_CONVERT", 0, 8, special=(0, 8))
        self.assertEqual(payload_for(spec, 0), b"")


def make_record(missing):
    return {
        "convert": {"nacks": 0, "polls": 10, "elapsed_s": 0.75},
        "read": {"nacks": 1, "polls": 20, "elapsed_s": 0.19},
        "words": [0x00E9] * (3 - len(missing)) + [0x8000] * len(missing),
        "values": [23.3] * (3 - len(missing)) + [None] * len(missing),
        "missing": missing,
        "nacks": 1,
        "polls": 30,
    }


class CachePagingTests(unittest.TestCase):
    class FakeBridge:
        def __init__(self, devices):
            self.devices = devices
            self.calls = []

        def ow_get_count(self):
            return len(self.devices)

        def ow_get_devices(self, offset, count):
            self.calls.append((offset, count))
            return self.devices[offset:offset + count]

    def test_single_batch(self):
        devices = [make_rom(i) for i in range(9)]
        bridge = self.FakeBridge(devices)
        self.assertEqual(demo.read_all_devices(bridge), devices)
        self.assertEqual(bridge.calls, [(0, 9)])

    def test_pages_above_the_response_limit(self):
        devices = [make_rom(i) for i in range(25)]
        bridge = self.FakeBridge(devices)
        self.assertEqual(demo.read_all_devices(bridge), devices)
        self.assertEqual(bridge.calls, [(0, demo.CACHE_PAGE), (20, 5)])

    def test_empty_cache(self):
        bridge = self.FakeBridge([])
        self.assertEqual(demo.read_all_devices(bridge), [])
        self.assertEqual(bridge.calls, [])


class ScanSummaryTests(unittest.TestCase):
    def test_missing_readings_are_counted(self):
        summary = scan.summarise([make_record([]), make_record(["28-1"])], 2.0)
        self.assertEqual(summary["missing_readings"], 1)
        self.assertEqual(summary["cycles_with_missing"], 1)

    def test_no_missing_readings(self):
        summary = scan.summarise([make_record([])], 1.0)
        self.assertEqual(summary["missing_readings"], 0)
        self.assertEqual(summary["cycles_with_missing"], 0)


if __name__ == "__main__":
    unittest.main()
