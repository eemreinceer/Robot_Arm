#!/usr/bin/env python3
"""Drives the arm through hand-eye poses and captures a sample at each.

COMMANDS REAL MOTION. Run only with the operator present and the cutoff in
hand. Every move is small, slow and clamped; the script refuses to leave the
configured envelope.

Design notes that matter for the result:

* Hand-eye is DEGENERATE under pure translation, and near-degenerate when every
  rotation shares an axis. The pose list therefore varies wrist orientation
  (joint_4/joint_5) as much as base yaw, rather than sweeping one joint.
* The board must stay fully visible. A pose whose board is missing or whose
  reprojection is over the gate is SKIPPED, not captured with bad data -- a
  corrupted sample cannot be told from a calibration error later.
* Each pose is held still before sampling. The measured observation noise floor
  is ~0.06 mm std, but only when nothing moves; sampling during settling would
  fold servo motion into the number.

THE BOARD-MOVED GATE
--------------------
Hand-eye assumes the board is FIXED in the world: `base -> board` must come out
identical for every sample. On 2026-07-29 it did not -- the board was nudged
mid-session, the tool happily kept sampling, and 76 good-looking samples were
worthless. The 175 mm "board scatter" in that run was not solver error, it was
a measurement of the board's real motion.

So the session re-measures the board from the ZERO POSE at intervals:

  1. A zero-pose reference is taken before the first move.
  2. Every `--check-every` poses, and once more at the end, the arm returns to
     zero and the board is measured again.
  3. Deviation over the gate ABORTS the session.
  4. A check that cannot see the board also ABORTS. "Cannot verify" is not
     "fine" -- in the failed run the board was out of frame at the end, and
     that was the signal, read as arm droop instead.
  5. Samples are tagged with the check span they fall in. Samples in a span
     that no passing check ever closed are written to `suspect_samples`, NEVER
     to `samples`. `solve_hand_eye.py` reads `samples`, so unverified data
     cannot silently reach the solver.

THE GATE THRESHOLD MUST BE MEASURED, AND THIS TOOL ENFORCES THAT
The arm has NO encoders, so returning to zero has its own repeatability error,
and the check sees `board motion + arm return error` summed. These cannot be
separated here, so a guessed threshold either aborts good sessions or misses
real drift. A capture session therefore REFUSES TO START without either:

    --gate-from <artifact>     derived from a real measurement (preferred), or
    --board-drift-mm X --board-drift-deg Y    both, supplied deliberately

The artifact comes from the calibration mode, run with the board CLAMPED:

    python3 scripts/hand_eye_session.py --measure-return 8

which reports the deviation distribution with a stationary board -- the gate's
own noise floor -- and writes a versioned JSON recording the commit, units,
frames, probe pose and residuals. There are deliberately NO default thresholds:
an unmeasured placeholder that still lets a session run is how the last one
produced 76 worthless samples.

Limitation, stated plainly: a board that moves and returns between two checks
passes both. The gate bounds the damage, it does not prove the board never
moved. Clamping the board is still the primary control.

ACCEPTANCE RUNS ON ONE ATOMIC OBSERVATION
`/vision_encoder/observation` (arm_interfaces/BoardObservation) carries the
pose, the detection verdict, the reprojection error, the corner counts and the
board geometry under a single header whose stamp is the SOURCE IMAGE's stamp.
Everything a sample is accepted on therefore comes from the same frame, by
construction.

This replaced a diagnostic-window stopgap that read three separate topics, two
of them unstamped. That stopgap was not sound: when a pose arrived before its
own diagnostics were published, the window held only the PREVIOUS frames'
values, so a fresh frame could be accepted on the strength of older ones. The
window is gone rather than tuned -- there is no correct window size for a
missing timestamp.

The old `board_pose` / `detected` / `reprojection_px` topics still exist for
RViz and debugging. They are NOT acceptance data and this tool ignores them.

`detected` on the observation means the producer's own gates passed, not merely
that corners were found. This tool still applies its own reprojection gate on
top, because the producer's threshold is set for a different purpose.

The arm has NO encoders: `base->wrist` comes from commanded angles echoed back.
That is the dominant error source (measured: 1 degree of joint error gives
~6 mm of board scatter), so this session's board scatter is a measurement of
the MECHANISM, not of the camera.

WHAT AN INTERRUPT DOES
----------------------
SIGINT and SIGTERM mean ONE thing here: after the first signal, the only
motion this tool will still command is the zero return, and it waits for
evidence that the return was delivered.

That is enforced in `move_to()` -- at the last line before the servos, where
every commanded pose passes -- not by a flag each loop is trusted to read. An
earlier version only set a flag that the capture loop never looked at, so a
SIGINT during a 15-pose session drove all 15 poses anyway and still exited 0.
A guard that depends on every caller remembering to check it is not a guard.

The delivery evidence is `/joint_states` sampled AFTER the zero command:
values already in hand when the command went out prove nothing, because the
arm may have been sitting at a stale zero reading the whole time. It is still
an OPEN LOOP echo of commanded angles -- it shows the controller consumed the
trajectory, not that the arm physically moved -- and this is the only feedback
the hardware has. If the echo does not arrive, the run exits nonzero and says
to cut the servo rail.
"""
import argparse
import datetime
import json
import math
import os
import signal
import subprocess
import sys
import time

try:
    import rclpy
    from arm_interfaces.msg import BoardObservation
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer, TransformListener
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ROS_IMPORT_ERROR = None
except ImportError as error:
    # Argument validation, --dry-run and the unit tests need no ROS. Anything
    # that actually moves the arm re-raises this before rclpy.init(), so a
    # missing ROS environment fails loudly instead of degrading quietly.
    ROS_IMPORT_ERROR = error
    Node = object

ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
JOINT_STATE_OTHER = 'other'
JOINT_STATE_COMPLETE = 'complete-arm'
JOINT_STATE_MALFORMED = 'malformed-arm'

# Hard ceiling on the commanded envelope. The measured working range for this
# arm is +-0.95 rad; anything past 1.0 rad is a typo or a different experiment,
# and either way it should not reach the servos because a flag was mistyped.
# Raising it is a deliberate edit, not a command-line option.
ENVELOPE_CEILING_RAD = 1.0

# Floor on move duration. Sessions run at 5 s/move; this only exists so a
# mistyped --move-seconds cannot turn a calibration into a slam.
MIN_MOVE_SECONDS = 1.0

# A return-repeatability artifact with fewer cycles than this is not a
# distribution, it is an anecdote, and must not set a safety threshold.
MIN_RETURN_CYCLES = 3

# Canonical board, from arm_perception/config/vision_encoder.yaml. The observation
# carries the producer's geometry so the two can be compared instead of assumed:
# stale 6x8/25 mm defaults failed silently on 2026-07-29, reporting only "board
# not detected". Do not copy the 25 mm from imx219_640x480.yaml -- that is the
# intrinsics calibration board and is a different, still-valid measurement.
BOARD_COLS = 6
BOARD_ROWS = 9
BOARD_SQUARE_MM = 27.5

RETURN_ARTIFACT_KIND = 'hand_eye_return_repeatability'
# v4 added purpose/usable_for_gate, per-cycle raw poses and the completion
# flag; incomplete or diagnostic files can no longer set a gate.
# v2 added executed_probe_pose_rad and made the envelope checkable. v3 changed
# what the numbers MEAN: every zero measurement, the reference included, now
# passes through the same approach excursion, so v2 residuals are contaminated
# by approach direction and are not comparable. Older artifacts are refused
# rather than read leniently.
RETURN_ARTIFACT_VERSION = 4

# A run must SAY what it is for. The default is diagnostic, so a short
# hypothesis-testing run cannot quietly become the calibration that sets the
# gate: MIN_RETURN_CYCLES is 3, so three good cycles would otherwise produce a
# gate-valid file. Declaring a calibration is a deliberate act.
PURPOSE_CALIBRATION = 'calibration'
PURPOSE_DIAGNOSTIC = 'diagnostic'

# How close the commanded-angle echo must get to zero before the tool will call
# the arm returned. This is /joint_states, which is an OPEN LOOP echo of what
# was commanded -- it proves the controller consumed the command, not that the
# arm physically moved. It is the only feedback this hardware has.
ZERO_RETURN_TOLERANCE_RAD = 0.02
ZERO_RETURN_TIMEOUT_S = 20.0

# How many joint states published AFTER the zero command must read zero. One
# would be enough if freshness were the only concern, but /joint_states is
# republished continuously and a single message can straddle the command; three
# consecutive fresh ones cannot. Values held from before the command are never
# counted -- an arm parked at a stale zero reading would otherwise "confirm" a
# return that was never delivered.
ZERO_RETURN_CONFIRM_SAMPLES = 3

# Ceiling on joint states taken in one pass while the barrier is established.
# Messages are taken straight off the subscription, so "no more messages" is
# the transport's answer about that subscription and nothing else -- see
# create_joint_state_listener() for the two executor-based versions this
# replaced, and why. The cap bounds the interrupt path, where every added
# millisecond delays the return. Reaching it means the queue was still
# draining, so the fence is NOT proven and the confirmation is refused rather
# than granted on a barrier that may sit behind pre-command messages.
ZERO_RETURN_DRAIN_MESSAGES = 200

# Sleep between polls while waiting for the echo. Nothing spins the listener
# node, so the wait is a poll rather than a blocking spin; this is short enough
# to keep the 50 Hz echo effectively immediate and long enough not to spin the
# CPU while the arm takes seconds to move.
ZERO_RETURN_POLL_S = 0.01

# Deltas in radians from the zero pose. Wrist-heavy on purpose: joint_4/5 swing
# the camera's orientation without walking it away from the board.
DEFAULT_POSES = [
    [0.00, 0.00, 0.00, 0.00, 0.00],
    [0.12, 0.00, 0.00, 0.00, 0.00],
    [-0.12, 0.00, 0.00, 0.00, 0.00],
    [0.00, 0.00, 0.00, 0.20, 0.00],
    [0.00, 0.00, 0.00, -0.20, 0.00],
    [0.00, 0.00, 0.00, 0.00, 0.20],
    [0.00, 0.00, 0.00, 0.00, -0.20],
    [0.10, 0.00, 0.00, 0.18, 0.15],
    [-0.10, 0.00, 0.00, -0.18, 0.15],
    [0.10, 0.00, 0.00, -0.18, -0.15],
    [-0.10, 0.00, 0.00, 0.18, -0.15],
    [0.00, 0.10, -0.10, 0.15, 0.00],
    [0.00, -0.10, 0.10, -0.15, 0.00],
    [0.15, 0.08, -0.08, 0.20, 0.18],
    [-0.15, -0.08, 0.08, -0.20, -0.18],
]

# The approach excursion every zero measurement passes through. It swings every
# joint so the arm returns from a real excursion, and -- more importantly -- it
# is the SAME for the reference and for every later measurement, which is what
# makes them comparable at all on an arm with no encoders.
DEFAULT_RETURN_PROBE = [0.15, 0.08, -0.08, 0.20, 0.18]


class BoardMoved(Exception):
    """The zero-pose check failed: the board is no longer where it was."""


class UnsafeCommand(Exception):
    """A value that would reach the servos failed its last-line check."""


class SessionInterrupted(Exception):
    """A stop was requested; only the verified zero return may still move."""


# --------------------------------------------------------------------------
# Pure helpers. No ROS, no I/O -- these are what the unit tests exercise.
# --------------------------------------------------------------------------

def is_finite_number(value):
    """True for a real, finite int/float.

    bool is excluded on purpose: it is a subclass of int, so without this
    `True` would sail through as a 1.0 radian joint command.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(value)


JOINT_STATE_LISTENER_NAME = 'hand_eye_joint_state_listener'


def create_joint_state_listener():
    """A node that holds the /joint_states subscription and is never spun.

    Two attempts at this were wrong in the same way, and the second failure is
    why the design changed rather than being patched again. Round 6 counted
    callbacks, which conflated "delivered" with "produced". Round 7 spun a
    dedicated node and called a spin that ran no callback "quiet" -- but review
    then MEASURED this machine's Jazzy: there is no
    `enable_type_description_service` keyword, so the constructor fell back and
    left a live `get_type_description` service, plus two QoS EventHandler
    waitables that cannot be switched off at all. On a real Jazzy wait set a
    spin can service that request instead of the subscription, so "no callback
    ran" did not mean "the queue is empty". Regression reproduced a false
    confirmation through it.

    So the queue is no longer read through an executor. Nothing spins this
    node; messages are taken straight off the subscription, where "no more
    messages" is the transport's own answer about THIS subscription and cannot
    be confused with other work. Verified on both platforms: Jazzy on the PC
    and Humble
    3.6.9 in the Jetson's `robot_arm_hw` container, five queued messages taken in
    order with no executor in the process.

    What is still stripped, and why it is no longer load-bearing: parameter
    services and rosout are switched off, and every destroyable entity is
    destroyed, so that if anything ever does spin this node the wait set is as
    close to bare as the API allows. The invariant the drain actually relies on
    is checked at every drain instead -- that no executor is attached.
    """
    node = Node(JOINT_STATE_LISTENER_NAME,
                start_parameter_services=False, enable_rosout=False)
    # Jazzy attaches get_type_description to every node. It is destroyable even
    # though the constructor keyword to prevent it does not exist here.
    for service in list(node.services):
        node.destroy_service(service)
    for timer in list(node.timers):
        node.destroy_timer(timer)
    for client in list(node.clients):
        node.destroy_client(client)
    for guard in list(node.guards):
        node.destroy_guard_condition(guard)
    return node


def stamp_key(msg):
    """(sec, nanosec) of a message's own header stamp, or None if unusable.

    None is returned for a missing header and for the all-zero stamp that an
    unstamped publisher writes. Both mean the same thing to a freshness gate:
    this message cannot say when it was produced, so it must not be counted.
    Keys are only ever compared with other keys from the SAME publisher.
    """
    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
    sec = getattr(stamp, 'sec', None)
    nanosec = getattr(stamp, 'nanosec', None)
    if not isinstance(sec, int) or not isinstance(nanosec, int):
        return None
    if isinstance(sec, bool) or isinstance(nanosec, bool):
        return None
    if sec == 0 and nanosec == 0:
        return None
    return (sec, nanosec)


def quat_angle_rad(quat_a, quat_b):
    """Angle of the rotation taking quat_a to quat_b.

    abs() on the dot product handles the double cover: q and -q are the same
    rotation, and without it a sign flip reads as a 180 degree jump.
    """
    dot = abs(sum(a * b for a, b in zip(quat_a, quat_b)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def vector_distance(xyz_a, xyz_b):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(xyz_a, xyz_b)))


def clamp_pose(positions, max_abs_rad):
    """Clamp a pose into the envelope, refusing anything non-finite.

    The envelope must be positive: with a negative one this expression
    collapses to `max(+e, min(-e, p))`, which returns +e for EVERY input --
    a commanded zero pose becomes a full-envelope move on all five joints.
    That is checked here rather than trusted from the parser, because this is
    the last code between a number and the servos.
    """
    if not is_finite_number(max_abs_rad) or max_abs_rad <= 0:
        raise UnsafeCommand('envelope must be a positive finite number, got %r'
                            % (max_abs_rad,))
    if max_abs_rad > ENVELOPE_CEILING_RAD:
        raise UnsafeCommand('envelope %r exceeds the %.2f rad ceiling'
                            % (max_abs_rad, ENVELOPE_CEILING_RAD))
    clamped = []
    for index, value in enumerate(positions):
        if not is_finite_number(value):
            raise UnsafeCommand('joint %d command is not a finite number: %r'
                                % (index, value))
        clamped.append(float(max(-max_abs_rad, min(max_abs_rad, value))))
    return clamped


def summarize_observations(observations):
    """Collapse repeated board observations into one reference pose.

    Translation is averaged. Rotation uses the MEDOID -- the observed
    quaternion closest to all the others -- rather than an average, because
    averaging quaternions componentwise is only valid for tiny spreads and
    silently degrades otherwise. The medoid is always a real observation.
    """
    count = len(observations)
    mean_xyz = [sum(obs['xyz'][axis] for obs in observations) / count
                for axis in range(3)]

    best_index, best_cost = 0, None
    for index, obs in enumerate(observations):
        cost = sum(quat_angle_rad(obs['quat_xyzw'], other['quat_xyzw'])
                   for other in observations)
        if best_cost is None or cost < best_cost:
            best_index, best_cost = index, cost

    spread_mm = max(vector_distance(obs['xyz'], mean_xyz)
                    for obs in observations) * 1000.0
    spread_deg = math.degrees(max(
        quat_angle_rad(obs['quat_xyzw'],
                       observations[best_index]['quat_xyzw'])
        for obs in observations))
    return {
        'xyz': mean_xyz,
        'quat_xyzw': list(observations[best_index]['quat_xyzw']),
        'sample_count': count,
        'internal_spread_mm': spread_mm,
        'internal_spread_deg': spread_deg,
    }


def summarize_cycles(good):
    """Canonical summary of measured (drift_mm, drift_deg) pairs.

    One implementation for the writer and the reader, so a stored summary can
    be checked against a recomputation that is guaranteed to use the same
    rule rather than a second copy of it.
    """
    mm_values = sorted(pair[0] for pair in good)
    deg_values = sorted(pair[1] for pair in good)
    gate_mm, gate_deg = suggested_gate(mm_values[-1], deg_values[-1])
    return {
        'max_drift_mm': mm_values[-1],
        'max_drift_deg': deg_values[-1],
        'median_drift_mm': mm_values[len(mm_values) // 2],
        'median_drift_deg': deg_values[len(deg_values) // 2],
        'suggested_gate_mm': gate_mm,
        'suggested_gate_deg': gate_deg,
    }


def suggested_gate(max_drift_mm, max_drift_deg):
    """Turn a measured return-repeatability maximum into a gate.

    1.5x the worst observed return error, with a floor so an unusually clean
    run cannot produce a gate tighter than the observation noise (measured at
    0.59 mm) and abort every future session on nothing.
    """
    return (max(1.5 * max_drift_mm, 2.0), max(1.5 * max_drift_deg, 0.2))


def split_samples(samples, span):
    """Verified samples vs. samples no passing check ever closed."""
    verified = [s for s in samples if s['span'] < span]
    suspect = [s for s in samples if s['span'] >= span]
    return verified, suspect


def build_return_artifact(args, cycles, executed_probe, reference=None,
                          complete=False):
    """Assemble the versioned return-repeatability artifact.

    Writer and reader (`load_gate_artifact`) are deliberately paired here so
    the schema has one owner. A gate artifact whose producer and consumer drift
    apart is the same class of defect as a stale default: it fails by handing
    over a number that means something other than what it says.

    Summary fields are None when too few cycles were measured -- the reader
    refuses those, so a calibration that measured nothing cannot set a gate.
    """
    good = [c for c in cycles if c.get('measured')]
    artifact = {
        'kind': RETURN_ARTIFACT_KIND,
        'version': RETURN_ARTIFACT_VERSION,
        'created_utc': utc_now_iso(),
        'git_commit': git_commit(),
        'units': {'translation': 'mm', 'rotation': 'deg', 'joints': 'rad'},
        'frames': {'base': args.base_frame, 'wrist': args.wrist_frame,
                   'measured_quantity': 'camera->board pose at the zero pose'},
        'arm_joints': ARM_JOINTS,
        'probe_pose_rad': list(args.zero_approach),
        # What actually went on the wire. Validation now refuses poses outside
        # the envelope, so these agree -- but the artifact states the executed
        # motion rather than inferring it, because a calibration record must
        # never claim a movement that did not happen.
        'executed_probe_pose_rad': list(executed_probe),
        'max_abs_rad': args.max_abs_rad,
        'move_seconds': args.move_seconds,
        'settle_seconds': args.settle_seconds,
        'zero_samples': args.zero_samples,
        'purpose': args.purpose,
        # Fail-closed on both axes: a run is not a calibration unless it says
        # so, and not usable until it finished. An interrupted diagnostic
        # cannot become the file that sets the drift gate.
        'complete': bool(complete),
        'usable_for_gate': bool(complete
                                and args.purpose == PURPOSE_CALIBRATION),
        # Raw poses, kept so the rotation AXIS can be recovered later. The
        # failed run stored only scalar magnitudes, which is why it could not
        # be told whether the 5.4 deg matched a joint axis or the camera mount.
        'zero_reference': reference,
        'cycles_requested': args.measure_return,
        'cycles_measured': len(good),
        'cycles': cycles,
        'note': 'Measured with the board CLAMPED, so this is the arm return '
                'error plus observation noise: the board-moved gate cannot go '
                'below it. The arm has no encoders.',
    }
    if len(good) < MIN_RETURN_CYCLES:
        artifact.update({'max_drift_mm': None, 'max_drift_deg': None,
                         'median_drift_mm': None, 'median_drift_deg': None,
                         'suggested_gate_mm': None, 'suggested_gate_deg': None})
        return artifact

    artifact.update(summarize_cycles(
        [(c['drift_mm'], c['drift_deg']) for c in good]))
    return artifact


def utc_now_iso():
    """Timezone-aware UTC. datetime.utcnow() is deprecated on the PC's
    Python 3.12 and this form works on the Nano's 3.6 too."""
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def git_commit():
    try:
        out = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            stderr=subprocess.STDOUT)
        return out.decode('utf-8', 'replace').strip()
    except Exception:
        return None


def read_raw_pose(block, label):
    """Validate a stored `{xyz, quat_xyzw}` pose and return its two parts.

    The quaternion norm is checked because `quat_angle_rad` normalises
    nothing: a zero or scaled quaternion would still produce a number, and
    that number would be a residual nobody could tell was wrong.
    """
    if not isinstance(block, dict):
        raise ValueError('%s is not a pose object: %r' % (label, block))
    xyz = block.get('xyz')
    quat = block.get('quat_xyzw')
    if (not isinstance(xyz, list) or len(xyz) != 3
            or not all(is_finite_number(v) for v in xyz)):
        raise ValueError('%s xyz is %r' % (label, xyz))
    if (not isinstance(quat, list) or len(quat) != 4
            or not all(is_finite_number(v) for v in quat)):
        raise ValueError('%s quat_xyzw is %r' % (label, quat))
    norm = math.sqrt(sum(value * value for value in quat))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError('%s quaternion norm is %.6f, not a unit rotation'
                         % (label, norm))
    return xyz, quat


def load_gate_artifact(path, expect_base_frame=None, expect_wrist_frame=None):
    """Read a return-repeatability artifact and derive the gate from it.

    The gate is RECOMPUTED from the raw per-cycle POSES -- the stored
    residuals are checked against that recomputation, never used as input, and
    the stored `suggested_gate_*` is not trusted either. A hand-edited file
    that claims a huge gate therefore cannot widen the threshold: it would have
    to carry raw poses that really are that far apart, and those poses are also
    what a later investigation reads to recover the rotation axis. Any stored
    summary that disagrees with the raw data means the file is inconsistent and
    is rejected rather than reconciled.

    The frame and joint context is checked against the session that is about
    to use it: a gate measured on a different geometry is not this arm's
    return error, however well-formed the file is.

    Raises ValueError with a specific reason rather than falling back to a
    default: a gate that quietly becomes a guess is the defect being fixed.
    """
    with open(path) as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError('artifact is not a JSON object')
    if payload.get('kind') != RETURN_ARTIFACT_KIND:
        raise ValueError('not a %s artifact (kind=%r)'
                         % (RETURN_ARTIFACT_KIND, payload.get('kind')))
    if payload.get('version') != RETURN_ARTIFACT_VERSION:
        raise ValueError('artifact version %r, this tool writes %d'
                         % (payload.get('version'), RETURN_ARTIFACT_VERSION))

    # Context that makes the number mean something. Absent fields are not
    # "defaults", they mean the file cannot be interpreted.
    units = payload.get('units')
    if not isinstance(units, dict):
        raise ValueError('artifact has no units block')
    expected_units = {
        'translation': 'mm',
        'rotation': 'deg',
        'joints': 'rad',
    }
    if any(units.get(quantity) != unit
           for quantity, unit in expected_units.items()):
        raise ValueError('artifact units are %r, expected mm/deg/rad'
                         % (units,))
    frames = payload.get('frames')
    if not isinstance(frames, dict):
        raise ValueError('artifact has no frames block')
    if payload.get('arm_joints') != ARM_JOINTS:
        raise ValueError('artifact arm_joints %r != this tool\'s %r'
                         % (payload.get('arm_joints'), ARM_JOINTS))
    if payload.get('complete') is not True:
        raise ValueError('artifact is not marked complete; an interrupted run '
                         'cannot set a gate')
    if payload.get('purpose') != PURPOSE_CALIBRATION:
        raise ValueError('artifact purpose is %r, not %r; a diagnostic run '
                         'cannot set a gate'
                         % (payload.get('purpose'), PURPOSE_CALIBRATION))
    if payload.get('usable_for_gate') is not True:
        raise ValueError('artifact is not marked usable_for_gate')

    envelope = payload.get('max_abs_rad')
    if (not is_finite_number(envelope) or envelope <= 0
            or envelope > ENVELOPE_CEILING_RAD):
        raise ValueError('artifact max_abs_rad is %r' % (envelope,))

    # Both the requested and the EXECUTED probe must be present and agree.
    # A v1 artifact could record probe_pose_rad=0.9 under a 0.3 rad envelope:
    # a motion that was clamped and therefore never happened, described as if
    # it had. The gate would then come from a calibration that did not occur.
    probes = {}
    for name in ('probe_pose_rad', 'executed_probe_pose_rad'):
        value = payload.get(name)
        if (not isinstance(value, list) or len(value) != len(ARM_JOINTS)
                or not all(is_finite_number(v) for v in value)):
            raise ValueError('artifact %s is %r' % (name, value))
        probes[name] = value
    for index, value in enumerate(probes['executed_probe_pose_rad']):
        if abs(value) > envelope + 1e-9:
            raise ValueError(
                'artifact executed probe joint %d is %r, outside its own '
                '%r rad envelope' % (index, value, envelope))
    for index, (asked, done) in enumerate(zip(probes['probe_pose_rad'],
                                              probes['executed_probe_pose_rad'])):
        if abs(asked - done) > 1e-9:
            raise ValueError(
                'artifact probe joint %d was requested as %r but executed as '
                '%r; the calibration did not perform the motion it records'
                % (index, asked, done))

    for label, expected, actual in (
            ('base', expect_base_frame, frames.get('base')),
            ('wrist', expect_wrist_frame, frames.get('wrist'))):
        if expected is not None and actual != expected:
            raise ValueError(
                'artifact %s frame is %r but this session uses %r; a gate '
                'measured on another geometry is not this arm\'s return error'
                % (label, actual, expected))

    # Recompute from the raw cycles. This is the only input that sets a gate.
    # v4 requires the raw poses to be present: a file carrying only scalar
    # magnitudes is what made the 2026-07-31 rotation impossible to attribute,
    # and a scalar nobody can re-derive is an assertion, not a measurement.
    reference_xyz, reference_quat = read_raw_pose(
        payload.get('zero_reference'), 'artifact zero_reference')
    cycles = payload.get('cycles')
    if not isinstance(cycles, list):
        raise ValueError('artifact has no raw cycles list to recompute from')
    good = []
    for entry in cycles:
        if not isinstance(entry, dict) or not entry.get('measured'):
            continue
        label = 'cycle %r' % entry.get('cycle')
        pose_xyz, pose_quat = read_raw_pose(entry.get('zero_pose'),
                                            '%s zero_pose' % label)
        delta = entry.get('delta_xyz_m')
        if (not isinstance(delta, list) or len(delta) != 3
                or not all(is_finite_number(v) for v in delta)):
            raise ValueError('%s delta_xyz_m is %r' % (label, delta))
        for axis, value in enumerate(delta):
            expected = pose_xyz[axis] - reference_xyz[axis]
            if abs(value - expected) > 1e-9:
                raise ValueError(
                    '%s delta_xyz_m[%d] is %r but its own pose and the '
                    'reference give %r' % (label, axis, value, expected))

        # THE residuals. Everything downstream -- the gate, the summary, the
        # provenance -- comes from these two lines and nothing else.
        drift_mm = vector_distance(pose_xyz, reference_xyz) * 1000.0
        drift_deg = math.degrees(quat_angle_rad(pose_quat, reference_quat))

        for name, stored, recomputed in (('drift_mm', entry.get('drift_mm'),
                                          drift_mm),
                                         ('drift_deg', entry.get('drift_deg'),
                                          drift_deg)):
            if not is_finite_number(stored):
                raise ValueError('%s %s is %r, not a finite number'
                                 % (label, name, stored))
            if stored < 0:
                raise ValueError('%s has a negative residual' % label)
            if abs(float(stored) - recomputed) > 1e-6:
                raise ValueError(
                    '%s %s is %r but its own raw poses give %.6f; the file is '
                    'inconsistent' % (label, name, stored, recomputed))
        good.append((drift_mm, drift_deg))
    if len(good) < MIN_RETURN_CYCLES:
        raise ValueError('artifact has %d usable measured cycle(s), need at '
                         'least %d' % (len(good), MIN_RETURN_CYCLES))

    stated = payload.get('cycles_measured')
    if stated != len(good):
        raise ValueError('artifact says %r measured cycles but carries %d'
                         % (stated, len(good)))

    # EVERY summary is recomputed, not just the gate. A forged max_drift_mm
    # does not move the threshold, but it would be copied into the session's
    # gate_provenance and become the record of how the gate was justified.
    summary = summarize_cycles(good)
    gate_mm = summary['suggested_gate_mm']
    gate_deg = summary['suggested_gate_deg']

    # A stored summary that disagrees with the raw data means the file was
    # edited or written by another version; do not silently prefer either.
    # An absent or null summary makes no claim -- the raw cycles already gave
    # the numbers, so there is nothing to contradict.
    for name, recomputed in (('suggested_gate_mm', gate_mm),
                             ('suggested_gate_deg', gate_deg),
                             ('max_drift_mm', summary['max_drift_mm']),
                             ('max_drift_deg', summary['max_drift_deg']),
                             ('median_drift_mm', summary['median_drift_mm']),
                             ('median_drift_deg', summary['median_drift_deg'])):
        stored = payload.get(name)
        if stored is None:
            continue
        # NaN must be caught before the comparison: every comparison with NaN
        # is False, so `abs(nan - x) > tol` would wave a forged value through.
        if not is_finite_number(stored):
            raise ValueError('artifact %s is %r, not a finite number'
                             % (name, stored))
        if abs(float(stored) - recomputed) > 1e-6:
            raise ValueError(
                'artifact %s is %r but its own cycles give %.6f; the file is '
                'inconsistent' % (name, stored, recomputed))

    # Provenance is built from the RECOMPUTED numbers plus the few fields that
    # are metadata rather than measurements. Nothing a forged file asserts
    # about its own residuals survives into the session record.
    summary['cycles_measured'] = len(good)
    summary['created_utc'] = payload.get('created_utc')
    summary['git_commit'] = payload.get('git_commit')
    summary['probe_pose_rad'] = list(probes['executed_probe_pose_rad'])
    summary['max_abs_rad'] = envelope
    return gate_mm, gate_deg, summary


# --------------------------------------------------------------------------
# Argument handling
# --------------------------------------------------------------------------

def build_parser():
    parser = argparse.ArgumentParser(
        description='Hand-eye capture session. Commands real motion.')
    parser.add_argument('--out', default='runs/hand_eye/session.json')
    parser.add_argument('--controller', default='/robot_arm_controller')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--wrist-frame', default='link_5')
    parser.add_argument('--max-reproj-px', type=float, default=1.0)
    parser.add_argument('--max-age-s', type=float, default=0.5)
    parser.add_argument('--observation-topic',
                        default='/vision_encoder/observation',
                        help='atomic BoardObservation topic; the only '
                             'acceptance input')
    parser.add_argument('--board-cols', type=int, default=BOARD_COLS)
    parser.add_argument('--board-rows', type=int, default=BOARD_ROWS)
    parser.add_argument('--square-size-mm', type=float,
                        default=BOARD_SQUARE_MM,
                        help='checked against the geometry the producer '
                             'reports; a mismatch stops the session')
    parser.add_argument('--move-seconds', type=float, default=5.0)
    parser.add_argument('--settle-seconds', type=float, default=2.5)
    parser.add_argument('--samples-per-pose', type=int, default=3)
    parser.add_argument('--max-abs-rad', type=float, default=0.30,
                        help='hard envelope; every commanded joint is clamped. '
                             'Must be positive and <= %.2f rad.'
                             % ENVELOPE_CEILING_RAD)
    parser.add_argument('--poses-json',
                        help='JSON list of 5-joint poses; overrides the built-in '
                             'list. Use after probing which poses keep the board '
                             'in view -- a blind list wastes bench time.')
    parser.add_argument('--check-every', type=int, default=4,
                        help='re-measure the board from zero every N poses')
    parser.add_argument('--zero-samples', type=int, default=3,
                        help='board observations averaged per zero-pose check')
    parser.add_argument('--gate-from', metavar='FILE',
                        help='return-repeatability artifact from '
                             '--measure-return; derives the drift gate from a '
                             'real measurement')
    parser.add_argument('--board-drift-mm', type=float,
                        help='drift gate, translation. NO DEFAULT: supply this '
                             'with --board-drift-deg, or use --gate-from.')
    parser.add_argument('--board-drift-deg', type=float,
                        help='drift gate, rotation. See --board-drift-mm.')
    parser.add_argument('--measure-return', type=int, metavar='N',
                        help='calibration mode: with the board CLAMPED, do N '
                             'excursion-and-return cycles and report the '
                             'deviation distribution. Captures no samples.')
    parser.add_argument('--return-out',
                        default='runs/hand_eye/return_repeatability.json',
                        help='where --measure-return writes its artifact')
    parser.add_argument('--purpose',
                        choices=[PURPOSE_DIAGNOSTIC, PURPOSE_CALIBRATION],
                        default=PURPOSE_DIAGNOSTIC,
                        help='what this run is FOR. Default diagnostic, whose '
                             'artifact cannot set a gate. Pass '
                             '"calibration" deliberately for a gate-setting '
                             'run.')
    parser.add_argument('--zero-approach', type=json.loads,
                        default=DEFAULT_RETURN_PROBE,
                        help='JSON 5-joint pose the arm passes through before '
                             'EVERY zero measurement, the reference included. '
                             'Fixes the approach direction so backlash cannot '
                             'masquerade as drift.')
    parser.add_argument('--dry-run', action='store_true',
                        help='print the plan and exit, commanding nothing')
    return parser


def validate_args(parser, args, poses):
    """Reject every malformed value BEFORE anything can be published.

    Everything here guards a number that ends up on a servo bus or decides
    whether bad data is accepted. Each check exists because the unchecked
    version had a concrete failure mode, noted inline.
    """
    def number(name, value, minimum, inclusive=True):
        if not is_finite_number(value):
            parser.error('%s must be a finite number, got %r' % (name, value))
        if inclusive and value < minimum:
            parser.error('%s must be >= %s, got %r' % (name, minimum, value))
        if not inclusive and value <= minimum:
            parser.error('%s must be > %s, got %r' % (name, minimum, value))

    def whole(name, value, minimum):
        if isinstance(value, bool) or not isinstance(value, int):
            parser.error('%s must be an integer, got %r' % (name, value))
        if value < minimum:
            parser.error('%s must be >= %d, got %r' % (name, minimum, value))

    # A negative envelope turns clamp_pose into "return +envelope always", so a
    # commanded zero pose becomes a full-scale move on all five joints.
    number('--max-abs-rad', args.max_abs_rad, 0.0, inclusive=False)
    if args.max_abs_rad > ENVELOPE_CEILING_RAD:
        parser.error('--max-abs-rad %r exceeds the %.2f rad ceiling; raising it '
                     'is a deliberate source edit, not a flag'
                     % (args.max_abs_rad, ENVELOPE_CEILING_RAD))
    # A tiny --move-seconds turns a calibration move into a slam.
    number('--move-seconds', args.move_seconds, MIN_MOVE_SECONDS)
    number('--settle-seconds', args.settle_seconds, 0.0)
    number('--max-age-s', args.max_age_s, 0.0, inclusive=False)
    number('--square-size-mm', args.square_size_mm, 0.0, inclusive=False)
    whole('--board-cols', args.board_cols, 2)
    whole('--board-rows', args.board_rows, 2)
    number('--max-reproj-px', args.max_reproj_px, 0.0, inclusive=False)
    # --check-every 0 is a modulo-by-zero in the pose loop, i.e. a crash with
    # the arm powered and mid-trajectory.
    whole('--check-every', args.check_every, 1)
    whole('--zero-samples', args.zero_samples, 1)
    whole('--samples-per-pose', args.samples_per_pose, 1)
    if args.measure_return is not None:
        whole('--measure-return', args.measure_return, 1)

    for label, pose_list in (('--poses-json', poses),
                             ('--zero-approach', [args.zero_approach])):
        for index, pose in enumerate(pose_list):
            if not isinstance(pose, (list, tuple)) or len(pose) != len(ARM_JOINTS):
                parser.error('%s entry %d must be %d numbers, got %r'
                             % (label, index, len(ARM_JOINTS), pose))
            for joint, value in enumerate(pose):
                # json.loads happily produces NaN and Infinity.
                if not is_finite_number(value):
                    parser.error('%s entry %d joint %d is not a finite number: '
                                 '%r' % (label, index, joint, value))
                # Reject rather than clamp. Clamping silently substitutes a
                # different experiment: the arm would move to the envelope
                # while the artifact and the sample record claimed the
                # requested angle, so the file would document a motion that
                # never happened.
                if abs(value) > args.max_abs_rad:
                    parser.error(
                        '%s entry %d joint %d is %r, outside the +-%.2f rad '
                        'envelope. Raise --max-abs-rad deliberately or fix the '
                        'pose; it will not be silently clamped.'
                        % (label, index, joint, value, args.max_abs_rad))

    if args.board_drift_mm is not None:
        number('--board-drift-mm', args.board_drift_mm, 0.0, inclusive=False)
    if args.board_drift_deg is not None:
        number('--board-drift-deg', args.board_drift_deg, 0.0, inclusive=False)


def resolve_gate(parser, args):
    """Decide the drift gate, or refuse to run.

    There is no default. The previous version shipped an 8 mm / 2 deg
    placeholder that was documented as a guess but still let a full capture
    session run, which is exactly how an unmeasured threshold ends up
    certifying a bad dataset.
    """
    explicit = (args.board_drift_mm is not None
                and args.board_drift_deg is not None)
    half = (args.board_drift_mm is not None) != (args.board_drift_deg is not None)
    if half:
        parser.error('--board-drift-mm and --board-drift-deg must be given '
                     'together; half a gate is not a gate')
    if args.gate_from and explicit:
        parser.error('--gate-from and explicit --board-drift-* both given; '
                     'pick one so the gate has a single provenance')
    if args.gate_from:
        try:
            gate_mm, gate_deg, summary = load_gate_artifact(
                args.gate_from, args.base_frame, args.wrist_frame)
        except (IOError, OSError, ValueError) as error:
            parser.error('--gate-from %s unusable: %s' % (args.gate_from, error))
        return gate_mm, gate_deg, {
            'source': 'artifact',
            'path': os.path.abspath(args.gate_from),
            # Recomputed from raw cycles by load_gate_artifact, not copied
            # from the file's own summary: a forged residual must not become
            # the record of how this gate was justified.
            'artifact_created_utc': summary['created_utc'],
            'artifact_git_commit': summary['git_commit'],
            'artifact_cycles_measured': summary['cycles_measured'],
            'artifact_max_drift_mm': summary['max_drift_mm'],
            'artifact_max_drift_deg': summary['max_drift_deg'],
            'artifact_median_drift_mm': summary['median_drift_mm'],
            'artifact_median_drift_deg': summary['median_drift_deg'],
            'artifact_executed_probe_rad': summary['probe_pose_rad'],
            'artifact_max_abs_rad': summary['max_abs_rad'],
            'values_recomputed_from_raw_cycles': True,
        }
    if explicit:
        return args.board_drift_mm, args.board_drift_deg, {
            'source': 'operator-supplied',
            'path': None,
        }
    parser.error(
        'no drift gate configured. The arm has no encoders, so the gate must '
        'be measured, not guessed. Clamp the board and run:\n'
        '    python3 %s --measure-return 8\n'
        'then pass --gate-from <artifact>, or supply --board-drift-mm and '
        '--board-drift-deg together if you know what they should be.'
        % os.path.basename(sys.argv[0]))


# --------------------------------------------------------------------------
# ROS session
# --------------------------------------------------------------------------

class HandEyeSession(Node):
    def __init__(self, args, gate_mm, gate_deg, gate_provenance):
        super(HandEyeSession, self).__init__('hand_eye_session')
        self.args = args
        self.gate_mm = gate_mm
        self.gate_deg = gate_deg
        self.gate_provenance = gate_provenance
        self.observation = None
        self.board_stamp_key = None
        self.board_arrival = 0.0
        # Consumed the moment a frame is CONSIDERED, not when it is accepted:
        # see grab(). A frame rejected by a gate must never be retried.
        self.consumed_stamp_key = None
        self.samples = []
        self.skipped = []
        self.checks = []
        self.zero_ref = None
        # Samples carry the span they were taken in. A span is verified only
        # once a LATER check passes; until then it is unproven.
        self.span = 0
        # Owned by the node rather than attached from outside, so no code path
        # can read a default of "not interrupted" from a missing attribute.
        # main() replaces this with the dict its signal handlers write to.
        self.interrupted = {'flag': False, 'signal': None}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            JointTrajectory, args.controller + '/joint_trajectory', 10)

        self.create_subscription(
            BoardObservation, args.observation_topic, self._on_observation,
            qos_profile_sensor_data)
        self.joint_positions = None
        # Counts USABLE joint states. This says a callback ran -- it does NOT
        # say when the publisher produced the value, which is why freshness is
        # decided by `joint_state_stamp` below and never by this counter.
        self.joint_state_seq = 0
        # Counts EVERY joint-state message, usable or not; the drain's quiet
        # test is about whether the queue is empty, not about content.
        self.joint_state_msgs = 0
        # Source stamp of the latest usable joint state, as (sec, nanosec), or
        # None when the latest one carried no usable stamp. Compared only
        # against other stamps from the same publisher, never against this
        # process's clock: the controller runs on the Jetson and this script
        # runs on the PC, so the two clocks are not known to agree.
        self.joint_state_stamp = None
        self.zero_return_reason = None
        # /joint_states lives on a node of its OWN, carrying nothing else: no
        # other subscription, no timer, no parameter services. That is what
        # makes the pre-command drain provable. Spinning a node that also
        # carries the camera and TF callbacks cannot distinguish "the joint
        # state queue is empty" from "an unrelated callback ran instead", and
        # Regression round 7 built the case where the difference decides whether a
        # stale zero echo confirms a return: three camera callbacks satisfied
        # the old quiet test while pre-command joint states were still queued.
        # A spin of THIS node either runs the joint-state callback or has
        # nothing to run.
        self.state_node = create_joint_state_listener()
        # Kept as an attribute because the messages are taken off it directly;
        # nothing spins this node, so the callback is invoked by _take_joint_
        # states() rather than by an executor.
        self.state_sub = self.state_node.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

    def destroy_node(self):
        # The listener is ours, so it is torn down with us rather than left to
        # the interpreter; main() only knows about one node.
        try:
            self.state_node.destroy_node()
        finally:
            super(HandEyeSession, self).destroy_node()

    # -- interrupt ---------------------------------------------------------

    def stop_requested(self):
        return bool(self.interrupted['flag'])

    def raise_if_interrupted(self, what):
        if self.stop_requested():
            raise SessionInterrupted(
                '%s refused: signal %s was received, and the only motion still '
                'allowed is the verified zero return'
                % (what, self.interrupted['signal']))

    # -- callbacks ---------------------------------------------------------

    def _on_joint_states(self, msg):
        """Classify and record one joint-state message.

        A message naming no arm joint is unrelated traffic, such as the
        gripper, and may be ignored. Once ANY arm joint is named, however, the
        message must carry every arm joint exactly once with a finite value.
        Treating a partial arm message as unrelated hid a nonzero joint inside
        a run of full zero readings and falsely confirmed the return.

        Returns `(classification, positions, reason)`. Only a complete arm
        reading has positions. A malformed arm reading has a reason that the
        confirmation path reports fail-closed.
        """
        self.joint_state_msgs += 1
        names = list(msg.name)
        raw_positions = list(msg.position)
        arm_names = [name for name in names if name in ARM_JOINTS]
        if not arm_names:
            return JOINT_STATE_OTHER, None, None

        # An arm-related message participates in the publisher-stamp fence even
        # when its payload is malformed. Otherwise a newer pre-command partial
        # message would leave the barrier behind its own source timestamp.
        self.joint_state_stamp = stamp_key(msg)
        malformed_shape = any((
            len(names) != len(raw_positions),
            set(arm_names) != set(ARM_JOINTS),
            len(arm_names) != len(set(arm_names)),
        ))
        if malformed_shape:
            return (JOINT_STATE_MALFORMED, None,
                    'partial or malformed arm joint list %r' % names)

        lookup = dict(zip(names, raw_positions))
        try:
            positions = [float(lookup[joint]) for joint in ARM_JOINTS]
        except (TypeError, ValueError, OverflowError) as error:
            return (JOINT_STATE_MALFORMED, None,
                    'arm position conversion failed: %s' % error)
        if not all(is_finite_number(value) for value in positions):
            return (JOINT_STATE_MALFORMED, None,
                    'arm positions contain a non-finite value')

        self.joint_positions = positions
        self.joint_state_seq += 1
        # The stamp the PUBLISHER wrote, not the time this callback ran. An
        # unstamped message leaves this None on purpose: the reader must refuse
        # it rather than fall back to arrival order, which is what let a queued
        # pre-command echo pass as fresh.
        return JOINT_STATE_COMPLETE, self.joint_positions, None

    def _on_observation(self, msg):
        self.observation = msg
        self.board_stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.board_arrival = time.monotonic()

    # -- motion ------------------------------------------------------------

    def spin_for(self, seconds):
        """Spin, but stop waiting once a stop was requested.

        A settle wait after a stop request only delays the zero return, and
        the arm is about to be commanded home anyway.
        """
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.stop_requested():
                return
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_to(self, positions, seconds, zero_return=False):
        """Publish a trajectory. Last line of defence before the servos.

        `zero_return` marks the ONE motion allowed after a stop request. The
        refusal lives here rather than in the loops because a flag that every
        caller must remember to read is not a guard: the capture loop did not
        read it, and a SIGINT mid-session drove all fifteen poses anyway. Both
        conditions are required -- the caller must declare the recovery AND
        the pose must actually be zero -- so the exemption cannot be borrowed
        to push some other pose through.
        """
        clamped = clamp_pose(positions, self.args.max_abs_rad)
        if self.stop_requested() and not (zero_return
                                          and clamped == [0.0] * 5):
            raise SessionInterrupted(
                'refusing to command %s after signal %s; only the verified '
                'zero return may still move the arm'
                % (['%+.2f' % v for v in clamped], self.interrupted['signal']))
        if not is_finite_number(seconds) or seconds < MIN_MOVE_SECONDS:
            raise UnsafeCommand('move duration must be >= %.2fs, got %r'
                                % (MIN_MOVE_SECONDS, seconds))
        if clamped != [float(v) for v in positions]:
            self.get_logger().warn(
                'pose clamped to +-%.2f rad envelope' % self.args.max_abs_rad)
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = clamped
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
        msg.points = [point]
        self.pub.publish(msg)
        return clamped

    # -- sampling ----------------------------------------------------------

    def grab(self):
        """Sample only when every gate passes; otherwise say why.

        Waits for an observation whose header.stamp has not been sampled
        before. Comparing stamps only for INEQUALITY means no clock is compared
        across machines -- it proves the frame is new, which is all that is
        needed. Every gate below reads the SAME message, so nothing about
        frame attribution has to be inferred.
        """
        wait_s = max(self.args.max_age_s * 4.0, 1.0)
        deadline = time.monotonic() + wait_s
        fresh = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.board_stamp_key is not None
                    and self.board_stamp_key != self.consumed_stamp_key):
                fresh = True
                break
        if not fresh:
            if self.board_stamp_key is None:
                return None, 'no observation published within %.2fs' % wait_s
            return None, ('no NEW observation within %.2fs; stamp %s repeated '
                          '(publisher stalled)'
                          % (wait_s, self.consumed_stamp_key))

        # BURN THE FRAME NOW, before any gate can reject it. If consumption
        # waited for acceptance, a frame rejected by a gate would still count
        # as unseen and could be retried later. Snapshot the message too, so a
        # callback landing mid-check cannot swap it under the gates.
        stamp_key = self.board_stamp_key
        observation = self.observation
        arrival = self.board_arrival
        self.consumed_stamp_key = stamp_key

        age = time.monotonic() - arrival
        if age > self.args.max_age_s:
            return None, 'observation %.2fs stale' % age

        # The producer and this tool must agree on what board is in frame. The
        # geometry travels with the observation precisely so this can be
        # checked rather than assumed; mismatched defaults previously failed
        # silently, reporting only "board not detected".
        if (observation.board_cols != self.args.board_cols
                or observation.board_rows != self.args.board_rows
                or abs(observation.square_size_mm
                       - self.args.square_size_mm) > 1e-3):
            return None, ('board mismatch: producer has %dx%d/%.2fmm, this '
                          'session expects %dx%d/%.2fmm'
                          % (observation.board_cols, observation.board_rows,
                             observation.square_size_mm, self.args.board_cols,
                             self.args.board_rows, self.args.square_size_mm))

        if not observation.detected:
            return None, ('board not detected (%d/%d corners)'
                          % (observation.corners_detected,
                             observation.corners_expected))

        # detected already means the producer's gates passed; this is a
        # separate, tighter gate for calibration data.
        if observation.reprojection_px > self.args.max_reproj_px:
            return None, ('reprojection %.3f px over the %.2f px gate'
                          % (observation.reprojection_px,
                             self.args.max_reproj_px))

        if observation.corners_detected != observation.corners_expected:
            return None, ('partial board: %d of %d corners'
                          % (observation.corners_detected,
                             observation.corners_expected))

        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.base_frame, self.args.wrist_frame, rclpy.time.Time())
        except Exception as error:
            return None, 'TF unavailable: %s' % error

        pose = observation.pose
        return {
            'base_to_wrist': {
                'xyz': [tf.transform.translation.x, tf.transform.translation.y,
                        tf.transform.translation.z],
                'quat_xyzw': [tf.transform.rotation.x, tf.transform.rotation.y,
                              tf.transform.rotation.z, tf.transform.rotation.w],
            },
            'cam_to_board': {
                'xyz': [pose.position.x, pose.position.y, pose.position.z],
                'quat_xyzw': [pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w],
            },
            'board_stamp': list(stamp_key),
            # This frame's own value, from the same message as the pose.
            'reprojection_px': float(observation.reprojection_px),
            'corners': [int(observation.corners_detected),
                        int(observation.corners_expected)],
            'camera_frame': observation.header.frame_id,
        }, 'ok'

    def return_to_zero_verified(self):
        """Command zero and wait until the controller echoes it back FRESHLY.

        The 2026-07-31 run showed why this is not optional: after SIGINT the
        process exited but /joint_states stayed at the probe command, so the
        arm was left commanded away from zero and had to be recovered with a
        separate trajectory. Publishing is not delivering.

        Only joint states the publisher PRODUCED after the zero command count.
        Accepting whatever value happened to be in hand would confirm a return
        on the strength of a reading taken before the command existed -- and
        the case where that is most likely is exactly the dangerous one, an arm
        whose state topic has gone quiet. `ZERO_RETURN_CONFIRM_SAMPLES`
        consecutive fresh readings are required; one out-of-tolerance reading
        resets the count rather than ending the wait, since the arm is still
        moving.

        Freshness is decided by the publisher's own `header.stamp`, never by
        callback order, and never by comparing that stamp with this process's
        clock. Review found the version that counted callbacks: a message that
        entered the DDS queue BEFORE the command but was delivered after it
        raised the callback counter, so three queued pre-command zero echoes
        could confirm a return while the arm sat at the probe pose. The barrier
        is therefore taken in the publisher's own stamp domain -- the newest
        stamp it had produced when the command went out, established by
        draining the queue first -- and a counted sample must carry a stamp
        strictly greater than the barrier and than every sample counted before
        it. Duplicate, backwards, and unstamped messages are not counted.

        Three honest limits. If the queue cannot be shown to have gone quiet,
        or if no stamped state arrived before the command, then there is no
        trustworthy barrier and nothing can be proven, so the return is REFUSED
        rather than assumed; the zero command is still sent in both cases.
        And a message produced in the gap between the drain and the publish
        call is still indistinguishable from a fresh one -- closing that needs
        a controller ACK, not a timestamp. That gap is sub-millisecond and,
        unlike the queue it replaces, cannot hold echoes from before the probe
        pose.

        /joint_states is an OPEN LOOP echo of commanded angles -- it proves the
        controller consumed the trajectory, not that the arm physically moved
        there. On this hardware that is the only feedback available, and the
        distinction is stated rather than hidden.

        Returns (delivered, positions), where positions is None when nothing
        countable arrived after the command. `zero_return_reason` carries why a
        refusal happened, so the operator is not told "nothing published" when
        the topic was in fact publishing unusably.
        """
        # A refusal here must not cost the arm its zero command, so the fence
        # is established BEFORE the command and reported AFTER it. Whatever is
        # wrong with the barrier, the arm is still told to come home.
        fence_error = None
        try:
            fenced = self._drain_joint_states()
        except Exception as error:
            fenced = False
            fence_error = (
                'the /joint_states fence failed before the zero command '
                '(%s: %s); zero was still commanded but its delivery cannot '
                'be confirmed' % (type(error).__name__, error))
        barrier = self.joint_state_stamp
        self.zero_return_reason = None
        self.move_to([0.0] * 5, self.args.move_seconds, zero_return=True)
        if fence_error is not None:
            self.zero_return_reason = fence_error
            return False, None
        if not fenced:
            self.zero_return_reason = (
                'the /joint_states queue never went quiet within %d messages, '
                'so the barrier may sit behind messages that predate the zero '
                'command' % ZERO_RETURN_DRAIN_MESSAGES)
            return False, None
        if barrier is None:
            self.zero_return_reason = (
                'no stamped /joint_states arrived before the zero command, so '
                'no message can be shown to postdate it')
            return False, None
        deadline = time.monotonic() + ZERO_RETURN_TIMEOUT_S
        confirmed = 0
        counted = barrier
        fresh_positions = None
        while time.monotonic() < deadline:
            try:
                samples, drained = self._take_joint_states()
            except Exception as error:
                self.zero_return_reason = (
                    'the /joint_states confirmation read failed after the '
                    'zero command (%s: %s); delivery cannot be confirmed'
                    % (type(error).__name__, error))
                return False, fresh_positions
            if not samples and drained:
                time.sleep(ZERO_RETURN_POLL_S)
                continue
            if not drained:
                # Regression round 10: the cap is not silence. A batch that stopped
                # at the message cap may have a nonzero echo queued behind it,
                # so confirming on it would trust a queue that was never shown
                # to be empty.
                self.zero_return_reason = (
                    'a /joint_states read hit the %d-message cap without the '
                    'queue going empty, so the arm state after the command is '
                    'not established' % ZERO_RETURN_DRAIN_MESSAGES)
                return False, fresh_positions
            # Every sample of the batch is judged before any success is
            # returned. Regression round 9: returning on the third zero from inside
            # this loop confirmed a return that a NEWER sample in the very
            # same read had already refuted -- the arm was back at the probe
            # pose and the tool said CONFIRMED. Only the run of zeros still
            # unbroken at the END of the batch can confirm anything.
            for classification, stamp, positions, issue in samples:
                if classification == JOINT_STATE_OTHER:
                    continue            # names the gripper, not the arm
                if classification == JOINT_STATE_MALFORMED:
                    self.zero_return_reason = (
                        'a /joint_states message after the zero command named '
                        'arm joints but was incomplete or malformed (%s); the '
                        'return cannot be confirmed' % issue)
                    return False, None
                # Regression round 10: a USABLE arm reading whose stamp cannot be
                # placed in order -- missing, or not newer than the last one
                # counted -- must not be skipped past. Skipping it hides
                # whatever it says: an out-of-tolerance reading with a bad
                # stamp would sit invisibly inside a run of fresh zeros and the
                # run would confirm. Its timestamp is untrustworthy, so the
                # ordering the gate rests on is broken; refuse rather than
                # guess where it belongs.
                if stamp is None or stamp <= counted:
                    self.zero_return_reason = (
                        'a /joint_states arm reading after the command '
                        'carried a stamp that cannot be ordered (missing, '
                        'duplicate or backwards); its timestamp is not '
                        'trustworthy and the return cannot be confirmed')
                    return False, positions
                counted = stamp
                fresh_positions = positions
                if all(abs(value) <= ZERO_RETURN_TOLERANCE_RAD
                       for value in positions):
                    confirmed += 1
                else:
                    confirmed = 0
            if confirmed >= ZERO_RETURN_CONFIRM_SAMPLES:
                return True, fresh_positions
        return False, fresh_positions

    def _take_joint_states(self):
        """Take every joint state the transport holds, without an executor.

        This is the whole freshness mechanism's foundation, so it is worth
        being exact about what it proves: `take_message` returning None is the
        transport's own statement that THIS subscription has nothing left. No
        wait set is consulted, so no service, timer or QoS event can be
        mistaken for "the queue is empty" -- the measured failure on
        Jazzy in round 8.

        Returns `(samples, drained)`. Every sample is a `(classification,
        stamp, positions, reason)` tuple, IN ORDER, so the caller judges every
        message rather than only the newest one in a batch. A take can return
        several messages at once, and folding them into one reading would
        quietly turn "three consecutive fresh zeros" into "three polls".
        Classification distinguishes unrelated traffic from a complete arm
        reading and from a malformed partial arm reading; conflating the first
        and third can hide a nonzero joint inside a confirmed zero run.

        `drained` is True only when the transport itself reported the queue
        empty (`take_message` returned None). It is False when the message cap
        was hit first, which does NOT prove the queue empty -- Regression round 10:
        the confirmation loop read a cap-limited batch as if it were the whole
        queue, so a nonzero echo queued behind 200 zeros was never seen and the
        return was confirmed while the arm sat at the probe pose. Reaching the
        cap is not silence; only None is.

        Raises if an executor has been attached to the listener node: an
        executor would consume messages through the callback behind this
        method's back, and then neither the count nor the order would describe
        the queue.
        """
        if self.state_node.executor is not None:
            raise UnsafeCommand(
                'an executor is attached to %s; joint states must be taken '
                'directly or a drained queue cannot be established'
                % JOINT_STATE_LISTENER_NAME)
        taken = []
        while len(taken) < ZERO_RETURN_DRAIN_MESSAGES:
            # The handle is a LOCK here, not a value: rclpy's __enter__
            # returns None, so the take goes through the handle itself.
            with self.state_sub.handle:
                result = self.state_sub.handle.take_message(
                    self.state_sub.msg_type, self.state_sub.raw)
            if result is None:
                return taken, True
            classification, positions, reason = self._on_joint_states(
                result[0])
            stamp = (self.joint_state_stamp
                     if classification != JOINT_STATE_OTHER else None)
            taken.append((classification, stamp, positions, reason))
        return taken, False

    def _drain_joint_states(self):
        """Empty the joint-state queue, and report whether that was PROVEN.

        Without this the barrier is the newest stamp this process has
        PROCESSED, while the queue may already hold newer pre-command messages;
        those would clear a barrier they actually predate, and a stale zero
        echo would confirm a return.

        Returns True when the transport reported the queue empty. Returns False
        when the message cap was reached first -- a publisher flooding faster
        than this drains, which leaves the barrier possibly behind pre-command
        messages, so the caller must refuse to confirm rather than trust it.
        """
        return self._take_joint_states()[1]

    def measure_zero(self):
        """Approach the zero pose the SAME way every time, then measure.

        The approach excursion is not decoration. The arm has no encoders, so
        where it physically rests at "commanded zero" depends on the direction
        it arrived from: backlash and gravity make an approach from above
        settle somewhere other than an approach from the side. An earlier
        version took the reference from whatever pose the arm happened to be
        in at startup and every later measurement from the probe pose, so the
        reference and the measurements were not comparable. The 2026-07-31 run
        showed exactly that signature -- every cycle about 62 mm from the
        reference, but only 3.7 mm from each other.

        Every zero measurement now runs this identical path, so a difference
        between two of them is a real difference, not an artefact of how the
        arm got there.

        Retries a bounded number of times: one stale frame is not evidence the
        board moved, but a run of failures is. Returns (reference, reason).
        """
        # The approach excursion is real motion, so a stop request has to be
        # refused here and not merely at the loop above: measure_zero() is
        # reached from the reference, from every board check and from every
        # repeatability cycle.
        self.raise_if_interrupted('board measurement')
        self.move_to(self.args.zero_approach, self.args.move_seconds)
        self.spin_for(self.args.move_seconds + self.args.settle_seconds)
        self.move_to([0.0] * 5, self.args.move_seconds)
        self.spin_for(self.args.move_seconds + self.args.settle_seconds)

        observations = []
        last_reason = 'no attempt made'
        attempts = self.args.zero_samples * 4
        for _ in range(attempts):
            self.raise_if_interrupted('board measurement')
            if len(observations) >= self.args.zero_samples:
                break
            sample, reason = self.grab()
            if sample is None:
                last_reason = reason
                self.spin_for(0.3)
                continue
            observations.append(sample['cam_to_board'])
            self.spin_for(0.3)

        if len(observations) < self.args.zero_samples:
            return None, ('only %d/%d zero-pose observations (%s)'
                          % (len(observations), self.args.zero_samples,
                             last_reason))
        return summarize_observations(observations), 'ok'

    def set_zero_reference(self):
        reference, reason = self.measure_zero()
        if reference is None:
            raise BoardMoved('zero-pose reference could not be taken: %s'
                             % reason)
        self.zero_ref = reference
        print('zero reference: xyz %s m, internal spread %.2f mm / %.3f deg'
              % (['%+.4f' % v for v in reference['xyz']],
                 reference['internal_spread_mm'],
                 reference['internal_spread_deg']))
        # A reference noisier than the gate makes the gate meaningless in that
        # axis: drift and noise would be indistinguishable.
        if reference['internal_spread_mm'] > self.gate_mm:
            raise BoardMoved(
                'reference translation noise (%.2f mm) exceeds the drift gate '
                '(%.2f mm); the gate could not tell drift from noise'
                % (reference['internal_spread_mm'], self.gate_mm))
        if reference['internal_spread_deg'] > self.gate_deg:
            raise BoardMoved(
                'reference rotation noise (%.3f deg) exceeds the drift gate '
                '(%.2f deg); the gate could not tell drift from noise'
                % (reference['internal_spread_deg'], self.gate_deg))

    def check_board(self, label):
        """Re-measure the board from zero and compare to the reference.

        Raises BoardMoved on drift OR on inability to measure. Both mean the
        samples taken since the last passing check are unproven.
        """
        print('\n-- board check (%s): returning to zero' % label)
        current, reason = self.measure_zero()
        if current is None:
            self.checks.append({'label': label, 'span': self.span,
                                'passed': False, 'reason': reason})
            raise BoardMoved('board check %s could not measure the board: %s'
                             % (label, reason))

        drift_mm = vector_distance(current['xyz'], self.zero_ref['xyz']) * 1000.0
        drift_deg = math.degrees(
            quat_angle_rad(current['quat_xyzw'], self.zero_ref['quat_xyzw']))
        passed = drift_mm <= self.gate_mm and drift_deg <= self.gate_deg
        record = {
            'label': label,
            'span': self.span,
            'passed': passed,
            'drift_mm': drift_mm,
            'drift_deg': drift_deg,
            'gate_mm': self.gate_mm,
            'gate_deg': self.gate_deg,
            'internal_spread_mm': current['internal_spread_mm'],
            'internal_spread_deg': current['internal_spread_deg'],
        }
        self.checks.append(record)
        print('   drift %.2f mm / %.3f deg  (gate %.2f mm / %.2f deg)  %s'
              % (drift_mm, drift_deg, self.gate_mm, self.gate_deg,
                 'OK' if passed else 'FAILED'))
        if not passed:
            raise BoardMoved(
                'board moved: %.2f mm / %.3f deg from the zero reference'
                % (drift_mm, drift_deg))
        # Everything up to here is now bounded by two passing checks.
        self.span += 1
        return record

    def save(self, path, aborted, abort_reason):
        verified, suspect = split_samples(self.samples, self.span)
        payload = {
            'meta': {
                'created_utc': utc_now_iso(),
                'git_commit': git_commit(),
                'base_frame': self.args.base_frame,
                'wrist_frame': self.args.wrist_frame,
                'max_reproj_px': self.args.max_reproj_px,
                'observation_topic': self.args.observation_topic,
                'board': {'cols': self.args.board_cols,
                          'rows': self.args.board_rows,
                          'square_size_mm': self.args.square_size_mm},
                'sample_count': len(verified),
                'suspect_count': len(suspect),
                'aborted': aborted,
                'abort_reason': abort_reason,
                'board_drift_gate_mm': self.gate_mm,
                'board_drift_gate_deg': self.gate_deg,
                'gate_provenance': self.gate_provenance,
                'check_every': self.args.check_every,
                'zero_reference': self.zero_ref,
                'skipped': self.skipped,
                'note': 'base_to_wrist is OPEN LOOP (commanded angles echoed; '
                        'no encoders). Board scatter measures the mechanism. '
                        '"samples" holds only spans closed by a passing '
                        'board-moved check; anything unproven is in '
                        '"suspect_samples" and must not be solved. '
                        'Pose, detection and reprojection all come from one '
                        'stamped BoardObservation, so every accepted sample '
                        'is attributed to a single camera frame.',
            },
            'board_checks': self.checks,
            'samples': verified,
            'suspect_samples': suspect,
        }
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        with open(path, 'w') as handle:
            json.dump(payload, handle, indent=2)
        print('wrote %d verified samples to %s' % (len(verified), path))
        if suspect:
            print('HELD BACK %d unverified samples in "suspect_samples" -- '
                  'no passing board check closed their span' % len(suspect))


def run_return_repeatability(node, args):
    """Measure the gate's own noise floor, with the board deliberately fixed.

    The zero-pose check cannot separate board motion from the arm's return
    error, so this excursion-and-return loop measures the two together with a
    stationary board. Whatever it reports IS the arm's contribution, and the
    drift gate has to sit above it.

    Returns a process exit code: nonzero when too few cycles were measured to
    set a threshold, because a calibration that silently measured nothing must
    not look like a success.
    """
    # Checked BEFORE the reference, because taking the reference is itself an
    # excursion: a signal arriving while the tool waits for vision must not be
    # answered by starting the approach move.
    node.raise_if_interrupted('return-repeatability run')
    node.set_zero_reference()
    print('\nboard must be CLAMPED and untouched for this measurement.')
    print('zero approach: %s (every measurement passes through it)'
          % ['%+.2f' % v for v in args.zero_approach])
    print('purpose: %s%s' % (args.purpose,
                             '' if args.purpose == PURPOSE_CALIBRATION
                             else ' (this artifact CANNOT set a gate)'))

    reference = dict(node.zero_ref)
    executed = clamp_pose(args.zero_approach, args.max_abs_rad)
    cycles = []

    def flush(complete=False):
        """Persist after every cycle, so an abort keeps what was measured."""
        write_json(args.return_out,
                   build_return_artifact(args, cycles, executed, reference,
                                         complete))

    flush()
    for index in range(args.measure_return):
        if node.stop_requested():
            print('\ninterrupted; stopping after %d cycle(s)' % len(cycles))
            break
        print('\n[%d/%d] excursion' % (index + 1, args.measure_return))
        # measure_zero() performs the approach excursion itself, so this cycle
        # is byte-for-byte the same operation that produced the reference.
        current, reason = node.measure_zero()
        if current is None:
            print('   could not measure at zero: %s' % reason)
            cycles.append({'cycle': index + 1, 'measured': False,
                           'reason': reason})
            flush()
            continue
        delta = [c - r for c, r in zip(current['xyz'], reference['xyz'])]
        drift_mm = vector_distance(current['xyz'], reference['xyz']) * 1000.0
        drift_deg = math.degrees(
            quat_angle_rad(current['quat_xyzw'], reference['quat_xyzw']))
        cycles.append({
            'cycle': index + 1, 'measured': True,
            'drift_mm': drift_mm, 'drift_deg': drift_deg,
            # Raw pose and delta, so the rotation AXIS is recoverable. Scalars
            # alone could not say whether 5.4 deg matched a joint or the mount.
            'zero_pose': {'xyz': list(current['xyz']),
                          'quat_xyzw': list(current['quat_xyzw'])},
            'delta_xyz_m': delta,
            'internal_spread_mm': current['internal_spread_mm'],
            'internal_spread_deg': current['internal_spread_deg']})
        flush()
        print('   return deviation %.2f mm / %.3f deg  (written to disk)'
              % (drift_mm, drift_deg))

    good = [c for c in cycles if c['measured']]
    print('\n=== return repeatability (%d/%d measured) ==='
          % (len(good), len(cycles)))

    finished = (len(cycles) == args.measure_return
                and not node.stop_requested())
    artifact = build_return_artifact(args, cycles, executed, reference,
                                     finished)

    if len(good) < MIN_RETURN_CYCLES or not finished:
        write_json(args.return_out, artifact)
        print('%d usable cycle(s) of %d requested; need at least %d and a '
              'complete run to state a threshold.'
              % (len(good), args.measure_return, MIN_RETURN_CYCLES))
        print('artifact written to %s but it CANNOT set a gate.'
              % args.return_out)
        return 1

    gate_mm = artifact['suggested_gate_mm']
    gate_deg = artifact['suggested_gate_deg']
    write_json(args.return_out, artifact)

    print('translation  median %.2f mm   max %.2f mm'
          % (artifact['median_drift_mm'], artifact['max_drift_mm']))
    print('rotation     median %.3f deg  max %.3f deg'
          % (artifact['median_drift_deg'], artifact['max_drift_deg']))
    print('\nThis is the gate\'s noise floor with a FIXED board.')
    print('artifact: %s' % args.return_out)
    # Only a calibration run is offered the --gate-from command. Printing it
    # after a diagnostic run hands over a line the loader will reject, which
    # reads as a tool bug at the moment the operator is trying to move on.
    if args.purpose == PURPOSE_CALIBRATION:
        print('use it directly:  --gate-from %s' % args.return_out)
        print('(that applies a gate of %.2f mm / %.2f deg)'
              % (gate_mm, gate_deg))
    else:
        print('purpose is %s, so this artifact CANNOT set a gate and '
              '--gate-from will refuse it.' % args.purpose)
        print('these numbers WOULD give a gate of %.2f mm / %.2f deg; to make '
              'one, rerun with --purpose %s.'
              % (gate_mm, gate_deg, PURPOSE_CALIBRATION))
    return 0


def write_json(path, payload):
    """Write atomically: a run that dies mid-write must not leave half a file.

    The 2026-07-31 run was interrupted and lost every completed cycle because
    the artifact was only written at the end. Now each cycle rewrites the whole
    file through a temp-and-rename, so an abort keeps everything measured so
    far and the file on disk is never partially written.
    """
    directory = os.path.dirname(path) or '.'
    os.makedirs(directory, exist_ok=True)
    temporary = path + '.tmp'
    with open(temporary, 'w') as handle:
        json.dump(payload, handle, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def install_stop_handlers(state):
    """Take SIGINT and SIGTERM back and make them set a flag, nothing more.

    rclpy's default SIGINT handler shuts the context down, which is exactly
    what stopped the zero return being delivered in the live run: the finally
    block then published through a dead context. These handlers only record
    the request, so the context stays alive long enough to command and verify
    zero. SIGTERM is included because a killed container must not skip the
    return either.

    Must be called AFTER rclpy.init(), which installs its own.
    """
    def _on_signal(signum, _frame):
        state['flag'] = True
        state['signal'] = signum
        print('\nsignal %d received; no further motion will be commanded '
              'except the zero return' % signum)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    return state


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    poses = DEFAULT_POSES
    if args.poses_json:
        with open(args.poses_json) as handle:
            poses = json.load(handle)

    validate_args(parser, args, poses)

    if args.measure_return is not None:
        # The calibration run still needs a usable zero reference, so its gate
        # is permissive by construction: it is measuring the floor, not
        # enforcing one. It never captures samples, so nothing is certified.
        gate_mm, gate_deg = float('inf'), float('inf')
        gate_provenance = {'source': 'calibration-run', 'path': None}
    else:
        gate_mm, gate_deg, gate_provenance = resolve_gate(parser, args)

    if args.dry_run:
        print('%d poses, envelope +-%.2f rad (ceiling %.2f):'
              % (len(poses), args.max_abs_rad, ENVELOPE_CEILING_RAD))
        for index, pose in enumerate(poses):
            print('  %2d  %s' % (index + 1, ['%+.2f' % v for v in pose]))
        if args.measure_return is not None:
            print('MODE: --measure-return %d, zero approach %s -> %s'
                  % (args.measure_return,
                     ['%+.2f' % v for v in args.zero_approach],
                     args.return_out))
            print('purpose: %s%s' % (args.purpose,
                                     '' if args.purpose == PURPOSE_CALIBRATION
                                     else ' — artifact CANNOT set a gate'))
        else:
            print('board check every %d poses, gate %.2f mm / %.2f deg (%s)'
                  % (args.check_every, gate_mm, gate_deg,
                     gate_provenance['source']))
        print('board %dx%d @ %.2f mm, observations from %s'
              % (args.board_cols, args.board_rows, args.square_size_mm,
                 args.observation_topic))
        print('move %.1fs, settle %.1fs, %d samples/pose'
              % (args.move_seconds, args.settle_seconds, args.samples_per_pose))
        return 0

    if ROS_IMPORT_ERROR is not None:
        sys.stderr.write('ROS 2 is required to run a session but could not be '
                         'imported: %s\n' % ROS_IMPORT_ERROR)
        return 2

    interrupted = {'flag': False, 'signal': None}
    rclpy.init()
    node = HandEyeSession(args, gate_mm, gate_deg, gate_provenance)
    install_stop_handlers(interrupted)
    node.interrupted = interrupted
    print('waiting for vision + TF...')
    node.spin_for(2.0)

    aborted, abort_reason, code = False, None, 0
    try:
        if args.measure_return is not None:
            # NOT `return code` -- the finally below can only raise the exit
            # code if the return happens after it runs. With an early return
            # Python fixes the value first, so a failed zero return printed
            # CUT THE SERVO RAIL and still exited 0.
            code = run_return_repeatability(node, args)
        else:
            print('board-moved gate: %.2f mm / %.2f deg (%s), checked every '
                  '%d poses'
                  % (gate_mm, gate_deg, gate_provenance['source'],
                     args.check_every))
            node.set_zero_reference()

            for index, pose in enumerate(poses):
                node.raise_if_interrupted('pose %d of %d'
                                          % (index + 1, len(poses)))
                print('\n[%2d/%d] -> %s' % (index + 1, len(poses),
                                            ['%+.2f' % v for v in pose]))
                executed = node.move_to(pose, args.move_seconds)
                node.spin_for(args.move_seconds + args.settle_seconds)
                taken = 0
                for _ in range(args.samples_per_pose):
                    sample, reason = node.grab()
                    if sample is None:
                        print('        skipped: %s' % reason)
                        node.skipped.append({'pose_index': index,
                                             'reason': reason})
                        break
                    sample['pose_index'] = index
                    sample['commanded'] = pose
                    sample['executed'] = executed
                    sample['span'] = node.span
                    node.samples.append(sample)
                    taken += 1
                if taken:
                    print('        captured %d (reproj %.3f px)'
                          % (taken, node.samples[-1]['reprojection_px']))

                is_last = index == len(poses) - 1
                if not is_last and (index + 1) % args.check_every == 0:
                    node.check_board('after pose %d' % (index + 1))

            node.check_board('final')
    except SessionInterrupted as error:
        aborted, code = True, 1
        abort_reason = ('stopped on signal before a closing board check; the '
                        'open span is unverified (%s)' % error)
        print('\n*** STOPPED ON SIGNAL: %s' % error)
    except BoardMoved as error:
        aborted, abort_reason, code = True, str(error), 1
        print('\n*** SESSION ABORTED: %s' % abort_reason)
        print('*** Samples since the last passing check are NOT usable.')
    except UnsafeCommand as error:
        aborted, abort_reason, code = True, 'unsafe command: %s' % error, 2
        print('\n*** SESSION ABORTED, refused to command: %s' % error)
    except KeyboardInterrupt:
        # Only reachable for a signal that lands between rclpy.init() and
        # install_stop_handlers(); nothing has moved yet at that point.
        aborted, code = True, 1
        abort_reason = ('interrupted before a closing board check; the open '
                        'span is unverified')
        print('\ninterrupted')
    finally:
        print('\nreturning to zero pose')
        try:
            delivered, positions = node.return_to_zero_verified()
            if delivered:
                print('zero return CONFIRMED by /joint_states echo %s'
                      % ['%+.4f' % v for v in positions])
            else:
                code = max(code, 3)
                print('\n*** ZERO RETURN NOT CONFIRMED within %.0fs.'
                      % ZERO_RETURN_TIMEOUT_S)
                print('*** /joint_states since the zero command: %s'
                      % (['%+.4f' % v for v in positions] if positions else
                         (node.zero_return_reason or
                          'NOTHING published after the command')))
                print('*** THE ARM MAY STILL BE COMMANDED AWAY FROM ZERO.')
                print('*** CUT THE SERVO RAIL before touching the arm.')
        except (UnsafeCommand, Exception) as error:
            code = max(code, 3)
            print('\n*** COULD NOT COMMAND ZERO POSE: %s' % error)
            print('*** CUT THE SERVO RAIL before touching the arm.')
        if node.samples:
            node.save(args.out, aborted, abort_reason)
        else:
            print('no samples captured')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
