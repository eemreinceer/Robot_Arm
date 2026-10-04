#!/usr/bin/env python3
"""Pure, dependency-free analysis for Robot Arm electrical measurement captures.

The functions in this module never access GPIO, UART, ROS, or a robot.  They
only classify already-recorded logic-analyzer and ESP32 rail-monitor data.
"""

from __future__ import division

import statistics


PASS = "PASS"
FAIL = "FAIL"
INCONCLUSIVE = "INCONCLUSIVE"


def _overall_status(statuses):
    if FAIL in statuses:
        return FAIL
    if INCONCLUSIVE in statuses:
        return INCONCLUSIVE
    return PASS


def parse_pwm_csv_lines(lines):
    """Parse the binary-column CSV emitted by ``sigrok-cli -O csv``."""
    rows = []
    column_count = None
    for line_number, source_line in enumerate(lines, 1):
        line = source_line.strip()
        is_comment = line.startswith(";")
        is_logic_header = line.lower().startswith("logic")
        if not line or is_comment or is_logic_header:
            continue
        fields = [field.strip() for field in line.split(",")]
        try:
            row = [int(field) for field in fields]
        except ValueError:
            raise ValueError(
                "line %d is not a binary sample row" % line_number
            )
        if not row or any(value not in (0, 1) for value in row):
            raise ValueError(
                "line %d contains a non-binary value" % line_number
            )
        if column_count is None:
            column_count = len(row)
        elif len(row) != column_count:
            raise ValueError(
                "line %d has %d columns; expected %d"
                % (line_number, len(row), column_count)
            )
        rows.append(row)
    if not rows:
        raise ValueError("no PWM samples parsed")
    return rows


def analyze_pwm_rows(
    rows,
    sample_rate_hz,
    names=None,
    expected_widths_us=None,
    target_period_ms=20.0,
    period_tolerance_ms=0.5,
    pulse_tolerance_us=20.0,
    max_jitter_us=10.0,
    min_pulses=3,
    servo_min_us=400.0,
    servo_max_us=2600.0,
):
    """Measure PWM widths/periods and return a machine-readable verdict."""
    if sample_rate_hz <= 0:
        raise ValueError("sample rate must be positive")
    if min_pulses < 1:
        raise ValueError("min_pulses must be at least one")
    channel_count = len(rows[0])
    if any(len(row) != channel_count for row in rows):
        raise ValueError("PWM rows do not have a consistent column count")
    if names is None:
        names = ["D%d" % index for index in range(channel_count)]
    if len(names) != channel_count:
        raise ValueError(
            "got %d channel names for %d columns" % (len(names), channel_count)
        )
    expected_widths_us = expected_widths_us or {}
    unknown_expectations = sorted(set(expected_widths_us) - set(names))
    if unknown_expectations:
        raise ValueError(
            "expected pulse name(s) not present in capture: %s"
            % ",".join(unknown_expectations)
        )

    channels = []
    for channel_index, name in enumerate(names):
        values = [row[channel_index] for row in rows]
        rising = []
        widths = []
        open_rise = None
        for sample_index in range(1, len(values)):
            previous = values[sample_index - 1]
            current = values[sample_index]
            if previous == 0 and current == 1:
                rising.append(sample_index)
                open_rise = sample_index
            elif previous == 1 and current == 0 and open_rise is not None:
                widths.append(
                    (sample_index - open_rise) * 1000000.0 / sample_rate_hz
                )
                open_rise = None

        periods = [
            (rising[index + 1] - rising[index]) * 1000.0 / sample_rate_hz
            for index in range(len(rising) - 1)
        ]
        result = {
            "name": name,
            "sample_count": len(values),
            "high_fraction": sum(values) / float(len(values)),
            "rising_edges": len(rising),
            "complete_pulses": len(widths),
            "status": INCONCLUSIVE,
            "reasons": [],
        }
        if widths:
            result.update(
                {
                    "width_mean_us": statistics.mean(widths),
                    "width_min_us": min(widths),
                    "width_max_us": max(widths),
                    "width_jitter_us": max(widths) - min(widths),
                }
            )
        if periods:
            result.update(
                {
                    "period_mean_ms": statistics.mean(periods),
                    "period_min_ms": min(periods),
                    "period_max_ms": max(periods),
                    "frequency_hz": 1000.0 / statistics.mean(periods),
                }
            )

        if len(widths) < min_pulses or len(periods) < max(1, min_pulses - 1):
            result["reasons"].append(
                "need at least %d complete pulses and %d periods"
                % (min_pulses, max(1, min_pulses - 1))
            )
            channels.append(result)
            continue

        failures = []
        if min(widths) < servo_min_us or max(widths) > servo_max_us:
            failures.append(
                "pulse outside %.1f..%.1f us servo envelope"
                % (servo_min_us, servo_max_us)
            )
        if max(widths) - min(widths) > max_jitter_us:
            failures.append(
                "pulse jitter %.1f us exceeds %.1f us"
                % (max(widths) - min(widths), max_jitter_us)
            )
        bad_periods = [
            period
            for period in periods
            if abs(period - target_period_ms) > period_tolerance_ms
        ]
        if bad_periods:
            failures.append(
                "%d period(s) outside %.3f +/- %.3f ms"
                % (len(bad_periods), target_period_ms, period_tolerance_ms)
            )

        expected = expected_widths_us.get(name)
        if expected is not None:
            result["expected_width_us"] = float(expected)
            result["width_error_us"] = (
                statistics.mean(widths) - float(expected)
            )
            if abs(result["width_error_us"]) > pulse_tolerance_us:
                failures.append(
                    "mean pulse error %.1f us exceeds +/- %.1f us"
                    % (result["width_error_us"], pulse_tolerance_us)
                )

        result["reasons"] = failures
        result["status"] = FAIL if failures else PASS
        channels.append(result)

    return {
        "kind": "servo_pwm_capture",
        "sample_rate_hz": sample_rate_hz,
        "sample_count": len(rows),
        "status": _overall_status([channel["status"] for channel in channels]),
        "channels": channels,
    }


def parse_rail_dump_lines(lines):
    """Parse one complete ``RAILDUMP`` frame from a mixed UART transcript."""
    metadata = None
    records = []
    ended = False
    for line_number, source_line in enumerate(lines, 1):
        line = source_line.strip()
        if not line:
            continue
        if line.startswith("RAILDUMP,BEGIN,"):
            if metadata is not None:
                raise ValueError("multiple RAILDUMP BEGIN records")
            metadata = {}
            for field in line.split(",")[2:]:
                if "=" not in field:
                    raise ValueError(
                        "invalid BEGIN field on line %d" % line_number
                    )
                key, value = field.split("=", 1)
                metadata[key] = int(value)
            continue
        if line == "RAILDUMP,END":
            if metadata is None:
                raise ValueError("RAILDUMP END appears before BEGIN")
            ended = True
            continue
        if line.startswith("RD,"):
            if metadata is None or ended:
                raise ValueError("RD record outside RAILDUMP frame")
            fields = line.split(",")
            if len(fields) != 5:
                raise ValueError(
                    "invalid RD field count on line %d" % line_number
                )
            try:
                index, relative_us, raw, millivolts = [
                    int(field) for field in fields[1:]
                ]
            except ValueError:
                raise ValueError(
                    "non-integer RD field on line %d" % line_number
                )
            records.append(
                {
                    "index": index,
                    "relative_us": relative_us,
                    "raw": raw,
                    "millivolts": millivolts,
                }
            )
    if metadata is None:
        raise ValueError("RAILDUMP BEGIN not found")
    return {"metadata": metadata, "records": records, "ended": ended}


def _longest_below_span_us(records, minimum_mv):
    longest = 0
    start = None
    previous = None
    for record in records:
        if record["calibrated_mv"] < minimum_mv:
            if start is None:
                start = record["relative_us"]
            previous = record["relative_us"]
        elif start is not None:
            longest = max(longest, previous - start)
            start = None
            previous = None
    if start is not None:
        longest = max(longest, previous - start)
    return longest


def analyze_rail_dump(
    parsed,
    minimum_mv=4800.0,
    expected_period_us=50.0,
    period_tolerance_us=10.0,
    voltage_scale=1.0,
):
    """Validate capture integrity, timing, and the calibrated rail minimum."""
    if minimum_mv <= 0 or expected_period_us <= 0 or period_tolerance_us < 0:
        raise ValueError("rail-analysis thresholds must be positive")
    if voltage_scale <= 0:
        raise ValueError("voltage_scale must be positive")

    metadata = parsed["metadata"]
    source_records = parsed["records"]
    records = []
    for record in source_records:
        copied = dict(record)
        copied["calibrated_mv"] = record["millivolts"] * voltage_scale
        records.append(copied)

    integrity_errors = []
    required = ("N", "TRIGGER", "THRESHOLD")
    missing = [key for key in required if key not in metadata]
    if missing:
        integrity_errors.append(
            "missing BEGIN field(s): %s" % ",".join(missing)
        )
    if not parsed["ended"]:
        integrity_errors.append("RAILDUMP END not found")
    if "N" in metadata and metadata["N"] != len(records):
        integrity_errors.append(
            "BEGIN N=%d but parsed %d RD records"
            % (metadata["N"], len(records))
        )
    expected_indices = list(range(len(records)))
    actual_indices = [record["index"] for record in records]
    if actual_indices != expected_indices:
        integrity_errors.append("RD indices are not contiguous from zero")
    if any(record["raw"] < 0 or record["raw"] > 4095 for record in records):
        integrity_errors.append("raw ADC value outside 0..4095")
    relative_us = [record["relative_us"] for record in records]
    if any(
        relative_us[index] <= relative_us[index - 1]
        for index in range(1, len(relative_us))
    ):
        integrity_errors.append(
            "relative timestamps are not strictly increasing"
        )

    trigger_index = metadata.get("TRIGGER")
    if trigger_index is not None:
        if trigger_index < 0 or trigger_index >= len(records):
            integrity_errors.append("trigger index is outside the capture")
        elif records[trigger_index]["relative_us"] != 0:
            integrity_errors.append(
                "trigger record does not have relative_us=0"
            )
        elif "THRESHOLD" in metadata:
            trigger_mv = records[trigger_index]["millivolts"]
            if trigger_mv >= metadata["THRESHOLD"]:
                integrity_errors.append(
                    "trigger sample is not below firmware threshold"
                )

    intervals = [
        relative_us[index] - relative_us[index - 1]
        for index in range(1, len(relative_us))
    ]
    timing_errors = []
    bad_intervals = [
        interval
        for interval in intervals
        if abs(interval - expected_period_us) > period_tolerance_us
    ]
    if not intervals:
        timing_errors.append("capture has fewer than two samples")
    elif bad_intervals:
        timing_errors.append(
            "%d sample interval(s) outside %.1f +/- %.1f us"
            % (len(bad_intervals), expected_period_us, period_tolerance_us)
        )

    result = {
        "kind": "servo_rail_dump",
        "status": INCONCLUSIVE,
        "record_count": len(records),
        "complete": not integrity_errors,
        "integrity_errors": integrity_errors,
        "timing_errors": timing_errors,
        "minimum_required_mv": float(minimum_mv),
        "voltage_scale": float(voltage_scale),
    }
    if records:
        calibrated = [record["calibrated_mv"] for record in records]
        below = [value for value in calibrated if value < minimum_mv]
        result.update(
            {
                "minimum_mv": min(calibrated),
                "maximum_mv": max(calibrated),
                "mean_mv": statistics.mean(calibrated),
                "below_minimum_samples": len(below),
                "longest_below_minimum_us": _longest_below_span_us(
                    records, minimum_mv
                ),
            }
        )
    if intervals:
        result.update(
            {
                "period_mean_us": statistics.mean(intervals),
                "period_min_us": min(intervals),
                "period_max_us": max(intervals),
                "effective_sample_rate_hz": (
                    1000000.0 / statistics.mean(intervals)
                ),
            }
        )

    if integrity_errors or timing_errors or not records:
        result["status"] = INCONCLUSIVE
    elif result["minimum_mv"] < minimum_mv:
        result["status"] = FAIL
    else:
        result["status"] = PASS
    return result
