#!/usr/bin/env python3
"""Classify a sigrok CSV capture of one or more 50 Hz servo PWM channels."""

import argparse
import json
import sys

from measurement_analysis import analyze_pwm_rows, parse_pwm_csv_lines


def _expected_widths(text):
    result = {}
    if not text:
        return result
    for item in text.split(","):
        if "=" not in item:
            raise ValueError("expected pulse must be NAME=MICROSECONDS")
        name, value = item.split("=", 1)
        result[name.strip()] = float(value)
    return result


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Robot Arm servo PWM logic-analyzer kaydını PASS/FAIL sınıflandır"
        ),
        epilog=(
            "q=0 örneği: %(prog)s cap.csv --names GPIO13,GPIO26 "
            "--expected-us GPIO13=1500,GPIO26=1373"
        ),
    )
    parser.add_argument("path")
    parser.add_argument("--sample-rate", type=int, default=1000000)
    parser.add_argument("--names", default="")
    parser.add_argument("--expected-us", default="")
    parser.add_argument("--pulse-tolerance-us", type=float, default=20.0)
    parser.add_argument("--max-jitter-us", type=float, default=10.0)
    parser.add_argument("--period-tolerance-ms", type=float, default=0.5)
    parser.add_argument("--min-pulses", type=int, default=3)
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args()

    try:
        names = [
            item.strip() for item in args.names.split(",") if item.strip()
        ]
        with open(args.path, encoding="utf-8") as handle:
            rows = parse_pwm_csv_lines(handle)
        if not names:
            names = ["D%d" % index for index in range(len(rows[0]))]
        result = analyze_pwm_rows(
            rows,
            args.sample_rate,
            names=names,
            expected_widths_us=_expected_widths(args.expected_us),
            pulse_tolerance_us=args.pulse_tolerance_us,
            max_jitter_us=args.max_jitter_us,
            period_tolerance_ms=args.period_tolerance_ms,
            min_pulses=args.min_pulses,
        )
    except (OSError, ValueError) as error:
        print("INCONCLUSIVE: %s" % error, file=sys.stderr)
        return 2

    if args.as_json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print("OVERALL: %s" % result["status"])
        for channel in result["channels"]:
            print(
                "%s: %s, pulses=%d, width=%s us, period=%s ms, freq=%s Hz"
                % (
                    channel["name"],
                    channel["status"],
                    channel["complete_pulses"],
                    "%.1f" % channel["width_mean_us"]
                    if "width_mean_us" in channel
                    else "n/a",
                    "%.3f" % channel["period_mean_ms"]
                    if "period_mean_ms" in channel
                    else "n/a",
                    "%.2f" % channel["frequency_hz"]
                    if "frequency_hz" in channel
                    else "n/a",
                )
            )
            for reason in channel["reasons"]:
                print("  - %s" % reason)
    return {"PASS": 0, "FAIL": 1, "INCONCLUSIVE": 2}[result["status"]]


if __name__ == "__main__":
    sys.exit(main())
