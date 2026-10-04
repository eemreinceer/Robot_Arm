#!/usr/bin/env python3
"""Generate ESP32 servo calibration from the canonical ROS calibration YAML."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict, List, Set, Tuple

import yaml


SERVO_COUNT = 6
RAD_TO_DEG = 180.0 / math.pi

# Board wiring and motion policy are firmware-local. Calibration, limits,
# polarity and normal speed come exclusively from servo_calibration.yaml.
FIRMWARE_POLICY = {
    1: {"pin": 13, "ledc": 0, "startup_deg_s": 15.0, "deadband_deg": 0.75},
    2: {"pin": 14, "ledc": 1, "startup_deg_s": 12.0, "deadband_deg": 0.75},
    3: {"pin": 25, "ledc": 2, "startup_deg_s": 12.0, "deadband_deg": 0.75},
    4: {"pin": 26, "ledc": 3, "startup_deg_s": 20.0, "deadband_deg": 1.25},
    5: {"pin": 27, "ledc": 4, "startup_deg_s": 20.0, "deadband_deg": 1.25},
    6: {"pin": 33, "ledc": 5, "startup_deg_s": 15.0, "deadband_deg": 1.25},
}


def _finite_number(entry: Dict[str, Any], field: str, channel: int) -> float:
    value = entry.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"channel {channel}: {field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"channel {channel}: {field} must be finite")
    return result


def load_calibration(path: Path) -> List[Dict[str, Any]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("calibration root must be a mapping")
    safety = data.get("activation_safety")
    if not isinstance(safety, dict) or safety.get("calibration_complete") is not True:
        raise ValueError("activation_safety.calibration_complete must be true")
    channels = data.get("channels")
    if not isinstance(channels, list) or len(channels) != SERVO_COUNT:
        raise ValueError(f"channels must contain exactly {SERVO_COUNT} entries")

    normalized: List[Dict[str, Any]] = []
    seen: Set[int] = set()
    for raw in channels:
        if not isinstance(raw, dict):
            raise ValueError("every channel entry must be a mapping")
        channel = raw.get("channel")
        if isinstance(channel, bool) or not isinstance(channel, int):
            raise ValueError("channel must be an integer")
        if channel not in FIRMWARE_POLICY or channel in seen:
            raise ValueError(f"invalid or duplicate channel: {channel}")
        seen.add(channel)
        expected_joint = f"joint_{channel}"
        if raw.get("joint") != expected_joint:
            raise ValueError(
                f"channel {channel}: expected joint {expected_joint}, "
                f"got {raw.get('joint')!r}"
            )

        min_rad = _finite_number(raw, "min_rad", channel)
        max_rad = _finite_number(raw, "max_rad", channel)
        min_us = _finite_number(raw, "min_us", channel)
        max_us = _finite_number(raw, "max_us", channel)
        zero_rad = _finite_number(raw, "zero_offset_rad", channel)
        max_velocity = _finite_number(raw, "max_velocity_rad_s", channel)
        limit_min_rad = float(raw.get("limit_min_rad", min_rad))
        limit_max_rad = float(raw.get("limit_max_rad", max_rad))
        if not all(math.isfinite(value) for value in (limit_min_rad, limit_max_rad)):
            raise ValueError(f"channel {channel}: limits must be finite")
        if not min_rad < max_rad:
            raise ValueError(f"channel {channel}: min_rad must be below max_rad")
        if not limit_min_rad < limit_max_rad:
            raise ValueError(
                f"channel {channel}: limit_min_rad must be below limit_max_rad"
            )
        if limit_min_rad < min_rad or limit_max_rad > max_rad:
            raise ValueError(
                f"channel {channel}: safety limits exceed calibration anchors"
            )
        if not 0 <= min_us < max_us <= 65535:
            raise ValueError(f"channel {channel}: invalid pulse range")
        if max_velocity <= 0.0:
            raise ValueError(f"channel {channel}: max_velocity_rad_s must be positive")
        if not limit_min_rad <= zero_rad <= limit_max_rad:
            raise ValueError(
                f"channel {channel}: ROS zero lies outside the safety limits"
            )
        invert = raw.get("invert")
        if not isinstance(invert, bool):
            raise ValueError(f"channel {channel}: invert must be boolean")

        policy = FIRMWARE_POLICY[channel]
        normalized.append(
            {
                "channel": channel,
                "joint": expected_joint,
                "pin": policy["pin"],
                "ledc": policy["ledc"],
                "calibration_min_servo_deg": min_rad * RAD_TO_DEG,
                "calibration_max_servo_deg": max_rad * RAD_TO_DEG,
                "joint_limit_min_deg": (limit_min_rad - zero_rad) * RAD_TO_DEG,
                "joint_limit_max_deg": (limit_max_rad - zero_rad) * RAD_TO_DEG,
                "zero_offset_deg": zero_rad * RAD_TO_DEG,
                "min_pulse_us": int(min_us),
                "max_pulse_us": int(max_us),
                "max_speed_deg_s": max_velocity * RAD_TO_DEG,
                "startup_speed_deg_s": min(
                    policy["startup_deg_s"], max_velocity * RAD_TO_DEG
                ),
                "deadband_deg": policy["deadband_deg"],
                "reversed": invert,
            }
        )
    normalized.sort(key=lambda item: item["channel"])
    return normalized


def calibration_fingerprint(path: Path) -> str:
    """Fingerprint of the calibration semantics the HOST also derives.

    Deliberately not calibration_sha256(): that digest covers the normalised
    dicts, which carry pin/ledc/deadband from FIRMWARE_POLICY -- fields that
    never reach the ROS side, so arm_hardware could not reproduce it without
    copying this table into C++. What the two sides actually have to agree on
    is the channel map, the scale anchors, the safety limits, the zero offset,
    the pulse range and the direction. This hashes exactly those, straight from
    the YAML, with the same defaulting rules load_calibration() applies in
    stm32_system_interface.cpp.

    Must stay byte-identical to STM32SystemInterface::calibration_fingerprint.
    Radians are quantised to micro-radians so the value cannot hinge on the
    last bit of a double.
    """
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    rows: List[Tuple[int, str]] = []
    for raw in data["channels"]:
        channel = int(raw["channel"])
        min_rad = float(raw["min_rad"])
        max_rad = float(raw["max_rad"])

        def micro(value: float) -> int:
            # C++ side uses std::llround, which rounds half away from zero.
            # Python's round() is banker's rounding, so it cannot be used here.
            scaled = value * 1.0e6
            return int(math.floor(scaled + 0.5)) if scaled >= 0 else -int(
                math.floor(-scaled + 0.5))

        rows.append(
            (
                channel,
                "{}:{}:{}:{}:{}:{}:{}:{}:{}:{};".format(
                    channel,
                    raw["joint"],
                    micro(min_rad),
                    micro(max_rad),
                    micro(float(raw.get("limit_min_rad", min_rad))),
                    micro(float(raw.get("limit_max_rad", max_rad))),
                    micro(float(raw.get("zero_offset_rad", 0.0))),
                    int(raw["min_us"]),
                    int(raw["max_us"]),
                    1 if raw.get("invert", False) else 0,
                ),
            )
        )
    rows.sort(key=lambda item: item[0])
    canonical = "".join(row for _, row in rows).encode("ascii")

    # FNV-1a 64: guards against an accidental mismatch, not an adversary, and
    # the ESP32 must be able to compute the same thing without a crypto library.
    digest = 0xCBF29CE484222325
    for byte in canonical:
        digest ^= byte
        digest = (digest * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"{digest:016x}"


def calibration_sha256(channels: List[Dict[str, Any]]) -> str:
    payload = json.dumps(
        channels, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("ascii")
    return hashlib.sha256(payload).hexdigest()


def _mapped_pulse_us(
    channel: Dict[str, Any], joint_angle_deg: float, quantized: bool
) -> int:
    def value(field: str) -> float:
        raw = float(channel[field])
        return float(f"{raw:.6f}") if quantized else raw

    lower = value("joint_limit_min_deg")
    upper = value("joint_limit_max_deg")
    joint = min(max(joint_angle_deg, lower), upper)
    servo = joint + value("zero_offset_deg")
    ratio = (
        servo - value("calibration_min_servo_deg")
    ) / (
        value("calibration_max_servo_deg")
        - value("calibration_min_servo_deg")
    )
    if channel["reversed"]:
        ratio = 1.0 - ratio
    pulse = channel["min_pulse_us"] + ratio * (
        channel["max_pulse_us"] - channel["min_pulse_us"]
    )
    return int(math.floor(pulse + 0.5))


def validate_generated_pwm_precision(channels: List[Dict[str, Any]]) -> int:
    maximum_error_us = 0
    for channel in channels:
        lower = float(channel["joint_limit_min_deg"])
        upper = float(channel["joint_limit_max_deg"])
        span = upper - lower
        for joint_angle_deg in (
            lower,
            lower + 0.25 * span,
            0.0,
            lower + 0.75 * span,
            upper,
        ):
            expected = _mapped_pulse_us(channel, joint_angle_deg, quantized=False)
            generated = _mapped_pulse_us(channel, joint_angle_deg, quantized=True)
            maximum_error_us = max(maximum_error_us, abs(expected - generated))
    if maximum_error_us > 1:
        raise ValueError(
            "generated calibration exceeds host PWM parity tolerance: "
            f"{maximum_error_us} us"
        )
    return maximum_error_us


def render_header(
    source: Path, channels: List[Dict[str, Any]], fingerprint: str
) -> str:
    digest = calibration_sha256(channels)
    rows = []
    for item in channels:
        rows.append(
            "  {"
            f"{item['pin']}U, {item['ledc']}U, "
            f"{item['calibration_min_servo_deg']:.6f}f, "
            f"{item['calibration_max_servo_deg']:.6f}f, "
            f"{item['joint_limit_min_deg']:.6f}f, "
            f"{item['joint_limit_max_deg']:.6f}f, "
            f"{item['zero_offset_deg']:.6f}f, "
            f"{item['min_pulse_us']}U, {item['max_pulse_us']}U, "
            f"{item['max_speed_deg_s']:.6f}f, "
            f"{item['startup_speed_deg_s']:.6f}f, "
            f"{item['deadband_deg']:.6f}f, "
            f"{str(item['reversed']).lower()}"
            "},"
        )
    source_label = source.as_posix()
    return (
        "// Generated by tools/generate_robot_config.py; do not edit manually.\n"
        f"// Source: {source_label}\n"
        "#pragma once\n\n"
        "namespace robot_arm {\n\n"
        f'constexpr char kCalibrationSha256[] = "{digest}";\n'
        f'constexpr char kCalibrationSha256Short[] = "{digest[:12]}";\n'
        "// Answered verbatim to the host's 'K?' request. arm_hardware derives\n"
        "// the same value from the same YAML at load time and refuses to\n"
        "// activate when the two disagree, so a YAML edited without reflashing\n"
        "// cannot silently drive against limits this firmware is not enforcing.\n"
        f'constexpr char kCalibrationFingerprint[] = "{fingerprint}";\n\n'
        "constexpr ServoCalibration kServoConfig[kServoCount] = {\n"
        + "\n".join(rows)
        + "\n};\n\n"
        "}  // namespace robot_arm\n"
    )


def default_paths() -> Tuple[Path, Path]:
    repository = Path(__file__).resolve().parents[3]
    source = repository / "src/robot_arm_description/config/servo_calibration.yaml"
    output = (
        repository
        / "firmware/esp32_servo_ctrl/include/robot_config_generated.hpp"
    )
    return source, output


def main() -> int:
    default_source, default_output = default_paths()
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=default_source)
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if the checked-in generated header is stale",
    )
    args = parser.parse_args()

    try:
        channels = load_calibration(args.source)
        parity_error_us = validate_generated_pwm_precision(channels)
        rendered = render_header(
            args.source.resolve().relative_to(default_source.parents[3]),
            channels,
            calibration_fingerprint(args.source),
        )
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"calibration generation failed: {error}", file=sys.stderr)
        return 2

    if args.check:
        try:
            current = args.output.read_text(encoding="utf-8")
        except OSError as error:
            print(f"generated calibration missing: {error}", file=sys.stderr)
            return 1
        if current != rendered:
            print(
                "generated calibration is stale; run "
                "tools/generate_robot_config.py",
                file=sys.stderr,
            )
            return 1
        print(
            f"calibration header current: {calibration_sha256(channels)[:12]}; "
            f"YAML/generated PWM error <= {parity_error_us} us"
        )
        return 0

    args.output.write_text(rendered, encoding="utf-8")
    print(
        f"wrote {args.output} ({calibration_sha256(channels)[:12]}); "
        f"YAML/generated PWM error <= {parity_error_us} us"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
