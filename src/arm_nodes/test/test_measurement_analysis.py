#!/usr/bin/env python3

import pathlib
import sys
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from measurement_analysis import (  # noqa: E402
    FAIL,
    INCONCLUSIVE,
    PASS,
    analyze_pwm_rows,
    analyze_rail_dump,
    parse_pwm_csv_lines,
    parse_rail_dump_lines,
)


def pwm_rows(widths_us, periods_us, sample_rate=1000000):
    length = sum(periods_us) + max(widths_us) + 2
    values = [0] * length
    rising = 1
    for index, width in enumerate(widths_us):
        for sample in range(rising, rising + width):
            values[sample] = 1
        if index < len(periods_us):
            rising += periods_us[index]
    return [[value] for value in values]


def rail_text(
    millivolts, intervals=None, threshold=5000, trigger=1, ended=True
):
    if intervals is None:
        intervals = [50] * (len(millivolts) - 1)
    times = [-50]
    for interval in intervals:
        times.append(times[-1] + interval)
    offset = times[trigger]
    times = [value - offset for value in times]
    lines = [
        "RAILDUMP,BEGIN,N=%d,TRIGGER=%d,THRESHOLD=%d"
        % (len(millivolts), trigger, threshold)
    ]
    for index, (relative_us, mv) in enumerate(zip(times, millivolts)):
        lines.append("RD,%d,%d,%d,%d" % (index, relative_us, 2000, mv))
    if ended:
        lines.append("RAILDUMP,END")
    return "\n".join(lines)


class PwmAnalysisTest(unittest.TestCase):
    def test_nominal_q0_channel_passes(self):
        rows = pwm_rows([1500, 1500, 1501, 1499], [20000, 20000, 20000])
        result = analyze_pwm_rows(
            rows,
            1000000,
            names=["GPIO13"],
            expected_widths_us={"GPIO13": 1500},
        )
        self.assertEqual(PASS, result["status"])
        self.assertAlmostEqual(50.0, result["channels"][0]["frequency_hz"])

    def test_wrong_q0_width_fails(self):
        rows = pwm_rows([1373, 1373, 1373], [20000, 20000])
        result = analyze_pwm_rows(
            rows,
            1000000,
            names=["GPIO26"],
            expected_widths_us={"GPIO26": 1500},
        )
        self.assertEqual(FAIL, result["status"])

    def test_missing_pulses_fail_period_gate(self):
        rows = pwm_rows([1500, 1500, 1500], [20000, 40000])
        result = analyze_pwm_rows(rows, 1000000, names=["GPIO13"])
        self.assertEqual(FAIL, result["status"])

    def test_flat_capture_is_inconclusive(self):
        result = analyze_pwm_rows([[0]] * 100000, 1000000, names=["GPIO13"])
        self.assertEqual(INCONCLUSIVE, result["status"])

    def test_parser_rejects_non_binary_and_ragged_rows(self):
        with self.assertRaises(ValueError):
            parse_pwm_csv_lines(["0,1\n", "0,2\n"])
        with self.assertRaises(ValueError):
            parse_pwm_csv_lines(["0,1\n", "0\n"])

    def test_misspelled_expected_channel_is_rejected(self):
        rows = pwm_rows([1500, 1500, 1500], [20000, 20000])
        with self.assertRaises(ValueError):
            analyze_pwm_rows(
                rows,
                1000000,
                names=["GPIO13"],
                expected_widths_us={"GPIO31": 1500},
            )


class RailAnalysisTest(unittest.TestCase):
    def test_nominal_complete_capture_passes(self):
        parsed = parse_rail_dump_lines(
            rail_text([5500, 4900, 5400]).splitlines()
        )
        result = analyze_rail_dump(parsed)
        self.assertEqual(PASS, result["status"])
        self.assertEqual(20000.0, result["effective_sample_rate_hz"])

    def test_voltage_below_gate_fails(self):
        parsed = parse_rail_dump_lines(
            rail_text([5500, 4700, 5400]).splitlines()
        )
        result = analyze_rail_dump(parsed)
        self.assertEqual(FAIL, result["status"])
        self.assertEqual(1, result["below_minimum_samples"])

    def test_bad_timing_is_inconclusive_not_a_rail_failure(self):
        parsed = parse_rail_dump_lines(
            rail_text([5500, 4900, 5400], intervals=[50, 100]).splitlines()
        )
        result = analyze_rail_dump(parsed)
        self.assertEqual(INCONCLUSIVE, result["status"])
        self.assertTrue(result["timing_errors"])

    def test_truncated_dump_is_inconclusive(self):
        parsed = parse_rail_dump_lines(
            rail_text([5500, 4900, 5400], ended=False).splitlines()
        )
        result = analyze_rail_dump(parsed)
        self.assertEqual(INCONCLUSIVE, result["status"])

    def test_anchor_scale_is_applied_before_voltage_gate(self):
        parsed = parse_rail_dump_lines(
            rail_text([5000, 4500, 5000]).splitlines()
        )
        unscaled = analyze_rail_dump(parsed)
        scaled = analyze_rail_dump(parsed, voltage_scale=1.1)
        self.assertEqual(FAIL, unscaled["status"])
        self.assertEqual(PASS, scaled["status"])

    def test_trigger_sample_must_match_firmware_threshold(self):
        parsed = parse_rail_dump_lines(
            rail_text([5500, 5100, 5400], threshold=5000).splitlines()
        )
        result = analyze_rail_dump(parsed)
        self.assertEqual(INCONCLUSIVE, result["status"])
        self.assertIn(
            "trigger sample is not below firmware threshold",
            result["integrity_errors"],
        )


if __name__ == "__main__":
    unittest.main()
