#!/usr/bin/env python3
"""Validate and classify an ESP32 ``RAILDUMP`` UART transcript."""

import argparse
import json
import sys

from measurement_analysis import analyze_rail_dump, parse_rail_dump_lines


def main():
    parser = argparse.ArgumentParser(
        description="Robot Arm servo-local rail kaydını PASS/FAIL sınıflandır"
    )
    parser.add_argument("path")
    parser.add_argument("--minimum-mv", type=float, default=4800.0)
    parser.add_argument("--expected-period-us", type=float, default=50.0)
    parser.add_argument("--period-tolerance-us", type=float, default=10.0)
    parser.add_argument(
        "--anchor-measured-mv",
        type=float,
        help="aynı sabit anda multimetrede okunan servo-local değer",
    )
    parser.add_argument(
        "--anchor-reported-mv",
        type=float,
        help="aynı anda RAIL komutunun bildirdiği değer",
    )
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args()

    if (args.anchor_measured_mv is None) != (args.anchor_reported_mv is None):
        print(
            "INCONCLUSIVE: iki anchor değeri birlikte verilmelidir",
            file=sys.stderr,
        )
        return 2
    scale = 1.0
    if args.anchor_measured_mv is not None:
        if args.anchor_measured_mv <= 0 or args.anchor_reported_mv <= 0:
            print(
                "INCONCLUSIVE: anchor değerleri pozitif olmalıdır",
                file=sys.stderr,
            )
            return 2
        scale = args.anchor_measured_mv / args.anchor_reported_mv

    try:
        with open(args.path, encoding="utf-8") as handle:
            parsed = parse_rail_dump_lines(handle)
        result = analyze_rail_dump(
            parsed,
            minimum_mv=args.minimum_mv,
            expected_period_us=args.expected_period_us,
            period_tolerance_us=args.period_tolerance_us,
            voltage_scale=scale,
        )
    except (OSError, ValueError) as error:
        print("INCONCLUSIVE: %s" % error, file=sys.stderr)
        return 2

    if args.as_json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print("OVERALL: %s" % result["status"])
        print(
            "samples=%d min=%s mV mean=%s mV rate=%s Hz below=%s"
            % (
                result["record_count"],
                "%.1f" % result["minimum_mv"]
                if "minimum_mv" in result
                else "n/a",
                "%.1f" % result["mean_mv"]
                if "mean_mv" in result
                else "n/a",
                "%.1f" % result["effective_sample_rate_hz"]
                if "effective_sample_rate_hz" in result
                else "n/a",
                result.get("below_minimum_samples", "n/a"),
            )
        )
        for reason in result["integrity_errors"] + result["timing_errors"]:
            print("  - %s" % reason)
    return {"PASS": 0, "FAIL": 1, "INCONCLUSIVE": 2}[result["status"]]


if __name__ == "__main__":
    sys.exit(main())
