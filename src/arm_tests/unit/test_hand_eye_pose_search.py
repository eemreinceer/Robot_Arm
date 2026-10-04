"""Hardware-free regressions for the Issue #8 hand-eye pose-set gate."""

import importlib.util
import json
import math
import re
import subprocess
import xml.etree.ElementTree as ET
from decimal import Decimal
from pathlib import Path

import numpy as np

import pytest


REPO = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO / "scripts" / "hand_eye_pose_search.py"
DOC_PATH = REPO / "docs" / "hand_eye_pose_set_requirement.md"
XACRO_PATH = REPO / "src" / "robot_arm_description" / "urdf" / "robot_arm.urdf.xacro"
FIXTURE_PATH = REPO / "src" / "arm_tests" / "test_data" / "hand_eye_bad_subset.json"


def load_pose_search():
    spec = importlib.util.spec_from_file_location("hand_eye_pose_search_under_test", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


POSE_SEARCH = load_pose_search()


def matrix_from_pose(pose):
    return POSE_SEARCH.rt_to_matrix(
        POSE_SEARCH.quat_to_rotation(*pose["quat_xyzw"]),
        pose["xyz_m"],
    )


def rpy_matrix(values):
    roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_matrix(axis, angle):
    unit = np.asarray(axis, dtype=float)
    unit /= np.linalg.norm(unit)
    x, y, z = unit
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def canonical_joints_from_xacro():
    result = subprocess.run(
        ["xacro", str(XACRO_PATH), "gripper_sim:=false"],
        capture_output=True,
        check=False,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, f"xacro failed:\n{result.stdout}\n{result.stderr}"
    root = ET.fromstring(result.stdout)
    by_name = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    joints = []
    for index in range(1, 6):
        name = f"joint_{index}"
        assert name in by_name, f"canonical xacro is missing {name}"
        joint = by_name[name]
        origin = joint.find("origin")
        axis = joint.find("axis")
        assert origin is not None and axis is not None, f"{name} lacks origin or axis"
        joints.append({
            "xyz": [float(value) for value in origin.attrib["xyz"].split()],
            "rpy": [float(value) for value in origin.attrib["rpy"].split()],
            "axis": [float(value) for value in axis.attrib["xyz"].split()],
        })
    return joints


def urdf_fk(joints, q):
    result = np.eye(4)
    for joint, angle in zip(joints, q):
        result = (
            result
            @ transform(rpy_matrix(joint["rpy"]), joint["xyz"])
            @ transform(axis_angle_matrix(joint["axis"], angle), [0.0, 0.0, 0.0])
        )
    return result


def documented_gate_constants():
    text = DOC_PATH.read_text(encoding="utf-8")
    constants = {}
    for name in ("TARGET_MM", "NOISE_FLOOR_MM", "K_GATE"):
        matches = re.findall(
            rf"^\| `{name}` \| `([^`]+)` \|$",
            text,
            flags=re.MULTILINE,
        )
        assert len(matches) == 1, f"expected exactly one parseable {name}, found {matches}"
        constants[name] = Decimal(matches[0])
    return constants


def test_b1_k_samples_are_seeded_and_gate_verdict_uses_p90():
    fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    matrices = [matrix_from_pose(pose["base_to_wrist"]) for pose in fixture["poses"]]
    first = POSE_SEARCH.k_samples(matrices, trials=24, seed=37)
    second = POSE_SEARCH.k_samples(matrices, trials=24, seed=37)
    assert np.array_equal(first, second)

    misleading = {"mean": POSE_SEARCH.K_GATE - 0.1, "p90": POSE_SEARCH.K_GATE + 0.1}
    assert POSE_SEARCH.gate_verdict(misleading) == "KALDI"


def test_b2_chain_matches_canonical_xacro_origins_axes_and_fk():
    joints = canonical_joints_from_xacro()
    assert len(POSE_SEARCH.CHAIN) == len(joints) == 5
    for script_joint, urdf_joint in zip(POSE_SEARCH.CHAIN, joints):
        xyz, rpy = script_joint
        np.testing.assert_allclose(xyz, urdf_joint["xyz"], rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(rpy, urdf_joint["rpy"], rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(urdf_joint["axis"], [0.0, 0.0, -1.0], rtol=0.0, atol=1e-12)

    poses = [
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.31, -0.22, 0.47, -0.63, 0.18],
        [-0.71, 0.55, -0.19, 0.82, -0.44],
        [0.12, 0.34, -0.58, -0.27, 0.73],
    ]
    for q in poses:
        np.testing.assert_allclose(POSE_SEARCH.fk(q), urdf_fk(joints, q), rtol=0.0, atol=1e-10)


def test_b3_code_gate_constants_match_requirement_document():
    documented = documented_gate_constants()
    assert Decimal(str(POSE_SEARCH.TARGET_MM)) == documented["TARGET_MM"]
    assert Decimal(str(POSE_SEARCH.NOISE_FLOOR_MM)) == documented["NOISE_FLOOR_MM"]
    assert Decimal(str(POSE_SEARCH.K_GATE)) == documented["K_GATE"]
    assert math.isclose(
        float(documented["K_GATE"]),
        float(documented["TARGET_MM"] / documented["NOISE_FLOOR_MM"]),
        rel_tol=0.0,
        abs_tol=1e-15,
    )


def test_b4_canonical_bad_ten_pose_subset_is_rejected_by_p90():
    fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    assert fixture["source"]["generator_commit"] == "dd90e85afb1e2c92cec845161b5fef45a9b04ab5"
    assert fixture["source"]["candidate_pool_count"] == 399
    assert fixture["source"]["candidate_pool_sha256"] == (
        "30d858a7d544050d3575025c924b7fe4f9927fdbffd15ba483ad7ad64786af38"
    )
    assert fixture["source"]["selected_subset_sha256"] == (
        "69ead5468bae34f0599c59333addf4a6a397f4252aa637c790fa578a78ee46fd"
    )
    assert fixture["frames"] == {"reference": "base_link", "target": "link_5"}
    assert fixture["units"] == {
        "joints": "rad",
        "translation": "m",
        "expected_error": "mm",
    }
    assert [pose["pose_index"] for pose in fixture["poses"]] == [
        131, 19, 157, 176, 161, 128, 139, 117, 110, 181,
    ]

    matrices = []
    for pose in fixture["poses"]:
        measured = matrix_from_pose(pose["base_to_wrist"])
        np.testing.assert_allclose(
            POSE_SEARCH.fk(pose["joints_rad"]),
            measured,
            rtol=0.0,
            atol=1e-6,
        )
        matrices.append(measured)

    gate = fixture["gate"]
    assert gate["trials"] == POSE_SEARCH.GATE_TRIALS
    assert gate["seed"] == 101
    assert gate["quantile"] == POSE_SEARCH.GATE_QUANTILE
    assert gate["noise_floor_mm"] == POSE_SEARCH.NOISE_FLOOR_MM
    assert gate["k_gate"] == POSE_SEARCH.K_GATE

    stats = POSE_SEARCH.gate_k(matrices)
    tolerance = gate["absolute_tolerance"]
    assert stats["mean"] == pytest.approx(gate["expected_mean"], abs=tolerance)
    assert stats["p90"] == pytest.approx(gate["expected_p90"], abs=tolerance)
    expected_error = stats["p90"] * POSE_SEARCH.NOISE_FLOOR_MM
    assert expected_error == pytest.approx(gate["expected_error_p90_mm"], abs=tolerance)
    assert stats["mean"] < POSE_SEARCH.K_GATE < stats["p90"]
    assert expected_error > POSE_SEARCH.TARGET_MM
    assert POSE_SEARCH.gate_verdict(stats) == gate["verdict"] == "KALDI"
