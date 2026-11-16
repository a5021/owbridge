#!/usr/bin/env python3
"""Host-side unit tests for the pure protocol helpers.

These tests need neither a bridge nor a CH341A adapter: they cover the mapping
and summary logic that turns a TEMP_READ_ALL response into per-sensor values.
Run from the tools directory with ``python3 -m unittest -v test_host_helpers``.
"""

import unittest

import owbridge_demo as demo
import owbridge_scan as scan


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
