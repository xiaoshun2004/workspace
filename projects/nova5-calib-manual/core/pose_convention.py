# -*- coding: utf-8 -*-
"""
core/pose_convention.py
-----------------------
Determine, rather than assume, how the Nova-5 encodes orientation.

The README of the reference bundle calls the rotation convention "the single
most common source of a plausible-looking but wrong hand-eye result", and
recommends a 90-degree single-axis move to pin it down. On this controller we
can do better, because the real-time feedback packet carries BOTH
representations of the same rotation:

    ToolVectorActual[3:6]  bytes 0624..0671   (rx, ry, rz)
    ActualQuaternion[0:4]  bytes 1384..1415   documented as [qw, qx, qy, qz]

A quaternion has no convention ambiguity beyond component order -- one bit,
and the two candidate orders disagree wildly for any non-trivial rotation. So
the quaternion is ground truth, and we simply search the space of Euler
conventions for the one that reproduces it. No robot motion required beyond
striking a few varied poses, and no chance of the operator mis-reading a
protractor.

Search space: 12 axis sequences x {intrinsic, extrinsic} x {deg, rad}, plus
the rotation-vector hypothesis, times 2 quaternion component orders. Proper
Euler sequences (xyx, zxz, ...) are included because they are legal and cheap
to test, not because a Dobot is expected to use one.

Degeneracy matters: near identity, or with a single sample, many conventions
agree. solve_convention() therefore requires several samples with genuine
angular spread and reports the margin between the best and the runner-up. A
narrow margin is not a result -- it is an instruction to strike more varied
poses.

If NOTHING matches, that is informative too: it means the quaternion and the
Cartesian vector describe different frames (e.g. quaternion on the flange,
ToolVectorActual on the taught TCP). diagnose_frame_mismatch() tests that
hypothesis explicitly against the Tool frame reported in the same packet.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from models import (
    EULER_ORDERS,
    PoseConvention,
    QUAT_ORDERS,
    Result,
    RobotPose,
    euler_to_matrix,
    matrix_to_rotvec,
    quat_to_matrix,
)
from transforms import rotation_angle_deg

# A convention is accepted when every sample agrees this closely. The bound is
# generous relative to controller rounding (poses are reported to ~3 decimals)
# but far tighter than the gap between two different conventions.
_MATCH_TOL_DEG = 0.5
# The runner-up must be at least this much worse, or the samples were too
# similar to distinguish the candidates.
_MIN_MARGIN_DEG = 2.0
# Samples must span at least this much rotation between the two most different
# of them, otherwise everything looks like the identity.
_MIN_SAMPLE_SPREAD_DEG = 30.0
_MIN_SAMPLES = 3
# Fixed, varied probe triples used to decide whether two candidates are the
# same rotation map written differently. Deterministic on purpose: a solve
# that depends on which random numbers came up is not a solve. Eight is
# comfortably more than the three degrees of freedom being separated.
_EQUIV_PROBES = (
    (12.0, 34.0, 56.0),
    (-78.0, 23.0, 41.0),
    (5.0, -95.0, 137.0),
    (163.0, 47.0, -29.0),
    (-11.0, -62.0, 88.0),
    (99.0, 8.0, -151.0),
    (37.5, 91.25, 13.75),
    (-124.0, -6.5, 72.0),
)
_EQUIV_TOL = 1e-9


@dataclass
class ConventionCandidate:
    convention: PoseConvention
    max_error_deg: float
    mean_error_deg: float

    def label(self) -> str:
        return self.convention.label()


def _sample_spread_deg(rotations: Sequence[np.ndarray]) -> float:
    """Largest geodesic angle between any two sample rotations."""
    worst = 0.0
    for i in range(len(rotations)):
        for j in range(i + 1, len(rotations)):
            worst = max(worst, rotation_angle_deg(rotations[i], rotations[j]))
    return worst


def _candidate_conventions() -> List[PoseConvention]:
    out: List[PoseConvention] = []
    for quat_order in QUAT_ORDERS:
        for degrees in (True, False):
            out.append(PoseConvention(order="xyz", intrinsic=False,
                                      degrees=degrees, rotvec=True,
                                      quat_order=quat_order))
            for order, intrinsic in itertools.product(EULER_ORDERS,
                                                      (False, True)):
                out.append(PoseConvention(order=order, intrinsic=intrinsic,
                                          degrees=degrees, rotvec=False,
                                          quat_order=quat_order))
    return out


def same_rotation_map(a: PoseConvention, b: PoseConvention) -> bool:
    """True when two candidates are genuinely the SAME map, written differently.

    Notational twins agree for EVERY input triple, so a handful of fixed
    varied probes settles it. This has to be tested rather than assumed:
    euler_to_matrix assigns the angles POSITIONALLY, so intrinsic "zyx" is
    Rz(rx)Ry(ry)Rx(rz) and NOT the extrinsic "xyz" reading of the same triple
    -- the two coincide only when rx == rz. A candidate that merely happens to
    fit the sample set is a real contender, not a twin, and dismissing it as
    one is how a wrong convention gets stamped verified.
    """
    if a.quat_order != b.quat_order:
        return False
    for rx, ry, rz in _EQUIV_PROBES:
        if not np.allclose(a.rotation(rx, ry, rz), b.rotation(rx, ry, rz),
                           rtol=0.0, atol=_EQUIV_TOL):
            return False
    return True


def score_convention(poses: Sequence[RobotPose],
                     convention: PoseConvention) -> Tuple[float, float]:
    """(max, mean) geodesic error between the Euler triple and the quaternion,
    in degrees, over the given poses."""
    errors = []
    for p in poses:
        errors.append(p.rotation_disagreement_deg(convention))
    if not errors:
        return (float("inf"), float("inf"))
    arr = np.asarray(errors, dtype=np.float64)
    if not np.all(np.isfinite(arr)):
        return (float("inf"), float("inf"))
    return (float(np.max(arr)), float(np.mean(arr)))


def solve_convention(poses: Sequence[RobotPose]
                     ) -> Result[List[ConventionCandidate]]:
    """Identify the controller's orientation convention from feedback packets.

    On success, .data is the ranked candidate list, best first, and
    .data[0].convention has verified=True set. On failure the message says
    exactly what to do about it, and .data still carries the ranking so the UI
    can show how close the contenders were.
    """
    usable = [p for p in poses if p.quat is not None and len(p.quat) == 4]
    if len(usable) < _MIN_SAMPLES:
        return Result.err_msg(
            "Need at least {} feedback samples carrying a quaternion "
            "(have {}). Move the arm to a few clearly different "
            "orientations and sample again.".format(_MIN_SAMPLES, len(usable)))

    # Variety is measured from the QUATERNIONS, not from the Euler triple.
    # Reading a radian-reporting controller's numbers as degrees would make
    # genuinely varied poses look nearly identical and reject a perfectly good
    # sample set. A quaternion carries no unit at all; the only unknown is
    # component order, so both are tried and the larger spread wins -- either
    # order showing real variety proves the poses differ.
    spread = max(
        _sample_spread_deg([quat_to_matrix(p.quat, order) for p in usable])
        for order in QUAT_ORDERS)
    if spread < _MIN_SAMPLE_SPREAD_DEG:
        return Result.err_msg(
            "Sampled orientations differ by only {:.1f} deg. Near a single "
            "orientation most conventions agree, so the result would be "
            "meaningless -- re-sample with at least {:.0f} deg of variety, "
            "rotating about more than one axis.".format(
                spread, _MIN_SAMPLE_SPREAD_DEG))

    ranked: List[ConventionCandidate] = []
    for cand in _candidate_conventions():
        max_err, mean_err = score_convention(usable, cand)
        ranked.append(ConventionCandidate(convention=cand,
                                          max_error_deg=max_err,
                                          mean_error_deg=mean_err))
    ranked.sort(key=lambda c: (c.max_error_deg, c.mean_error_deg))

    best = ranked[0]
    if best.max_error_deg > _MATCH_TOL_DEG:
        return Result.err(
            ranked,
            "No orientation convention reproduces the reported quaternion "
            "(best candidate {} is off by {:.2f} deg). The quaternion and the "
            "Cartesian vector most likely describe different frames -- run "
            "diagnose_frame_mismatch(), and fall back to the manual "
            "90-degree axis test if that is inconclusive.".format(
                best.label(), best.max_error_deg))

    # Distinguishability. A runner-up may only be set aside when it is the
    # SAME rotation map as the winner written differently -- proved probe by
    # probe, not assumed from the labels. Anything else that fits the samples
    # this well is a genuine contender and drives the margin to ~0, which is
    # precisely the ambiguity this gate exists to reject: co-fitting candidates
    # mean the samples did not separate them, e.g. rotating about one axis only.
    equivalents = [c for c in ranked[1:]
                   if c.max_error_deg <= _MATCH_TOL_DEG
                   and same_rotation_map(best.convention, c.convention)]
    equivalent_ids = set(id(c) for c in equivalents)
    contenders = [c for c in ranked[1:] if id(c) not in equivalent_ids]
    margin = (contenders[0].max_error_deg - best.max_error_deg
              if contenders else float("inf"))
    if margin < _MIN_MARGIN_DEG:
        return Result.err(
            ranked,
            "Convention is ambiguous: {} and {} both fit within {:.2f} deg. "
            "Sample more poses with rotation about a different "
            "axis.".format(best.label(), contenders[0].label(), margin))

    best.convention.verified = True
    best.convention.note = (
        "solved from {} feedback samples spanning {:.0f} deg; max error "
        "{:.3f} deg, next distinct candidate {:.1f} deg away{}".format(
            len(usable), spread, best.max_error_deg, margin,
            "" if not equivalents else
            "; {} equivalent notation(s) fit identically".format(
                len(equivalents))))
    return Result.ok(ranked)


def diagnose_frame_mismatch(poses: Sequence[RobotPose]
                            ) -> Result[str]:
    """Test whether the quaternion refers to a different frame than the
    Cartesian vector.

    If ToolVectorActual is the taught TCP but ActualQuaternion is the bare
    flange, then for the correct Euler convention

        R_quat = R_euler @ R_tool^-1     (or R_tool^-1 @ R_euler)

    where R_tool is the rotation part of the Tool frame reported in the same
    packet. Both compositions are tested; a hit names the relationship, which
    is enough to build correct transforms from either field.
    """
    usable = [p for p in poses
              if p.quat is not None and len(p.quat) == 4
              and p.tool_frame is not None and len(p.tool_frame) == 6]
    if len(usable) < _MIN_SAMPLES:
        return Result.err_msg(
            "Need at least {} samples carrying both a quaternion and the "
            "Tool frame block to test a frame mismatch.".format(_MIN_SAMPLES))

    best_label = ""
    best_err = float("inf")
    for cand in _candidate_conventions():
        for relation in ("pose_then_tool_inv", "tool_inv_then_pose"):
            errors = []
            for p in usable:
                R_q = quat_to_matrix(p.quat, cand.quat_order)
                R_e = cand.rotation(p.rx, p.ry, p.rz)
                tool = list(p.tool_frame)
                angles = tool[3:6]
                if cand.degrees:
                    angles = [math.radians(a) for a in angles]
                R_t = euler_to_matrix(angles, cand.order, cand.intrinsic)
                if relation == "pose_then_tool_inv":
                    R_pred = R_e @ R_t.T
                else:
                    R_pred = R_t.T @ R_e
                errors.append(rotation_angle_deg(R_q, R_pred))
            max_err = float(np.max(errors))
            if max_err < best_err:
                best_err = max_err
                best_label = "{} with quaternion = {}".format(
                    cand.label(),
                    "R_pose * R_tool^-1" if relation == "pose_then_tool_inv"
                    else "R_tool^-1 * R_pose")

    if best_err <= _MATCH_TOL_DEG:
        return Result.ok(
            "Frame mismatch confirmed: {} (max error {:.3f} deg). The "
            "quaternion is on a different frame than ToolVectorActual, so "
            "build transforms from the Euler triple and keep the quaternion "
            "only as a cross-check.".format(best_label, best_err))
    return Result.err_msg(
        "No frame-composition hypothesis fits either (best {:.2f} deg with "
        "{}). Fall back to the manual test: command a known 90-degree "
        "single-axis move and compare the matrix each candidate "
        "builds.".format(best_err, best_label))


def verify_against_known_move(before: RobotPose, after: RobotPose,
                              axis: str, angle_deg: float,
                              convention: PoseConvention,
                              tol_deg: float = 1.0,
                              tol_axis_deg: float = 10.0) -> Result[float]:
    """Independent confirmation using a commanded single-axis move.

    Belt and braces for the quaternion solve, and the only route available if
    the quaternion turns out to be unusable. Command a rotation of angle_deg
    about the named tool axis, sample before and after, and check that the
    relative rotation the convention reconstructs is the one that was asked
    for -- both its SIZE and its AXIS. Checking the magnitude alone is useless
    here: the dominant error class on this controller is an axis permutation
    (twelve of the candidate sequences differ only in which reported number
    turns which axis), and every one of them reproduces the right magnitude.
    Returns the angular discrepancy in degrees.
    """
    if axis not in ("x", "y", "z"):
        return Result.err_msg("axis must be x, y or z (got {!r})".format(axis))
    R_before = convention.rotation(before.rx, before.ry, before.rz)
    R_after = convention.rotation(after.rx, after.ry, after.rz)
    R_rel = R_before.T @ R_after
    rvec_norm = rotation_angle_deg(np.eye(3), R_rel)
    err = abs(rvec_norm - abs(float(angle_deg)))
    if err > tol_deg:
        return Result.err(
            err,
            "Commanded {:.1f} deg about {}, but {} reconstructs {:.2f} deg. "
            "The convention is wrong -- do not capture with it.".format(
                angle_deg, axis, convention.label(), rvec_norm))

    # R_rel = R_before^T R_after is a BODY-frame relative rotation, so its
    # Rodrigues axis is already expressed in the tool frame and compares
    # directly against the commanded unit axis. Sign is irrelevant (a turn of
    # -theta about -n is the same rotation), hence the absolute value. Below a
    # degree of total rotation the axis is numerical noise, so there is
    # nothing meaningful to compare.
    if rvec_norm > 1.0:
        rvec = np.asarray(matrix_to_rotvec(R_rel), dtype=np.float64)
        norm = float(np.linalg.norm(rvec))
        if norm > 1e-12:
            unit = rvec / norm
            wanted = np.zeros(3, dtype=np.float64)
            wanted["xyz".index(axis)] = 1.0
            dot = min(1.0, abs(float(np.dot(unit, wanted))))
            off_deg = float(np.degrees(np.arccos(dot)))
            if off_deg > tol_axis_deg:
                return Result.err(
                    err,
                    "Commanded {:.1f} deg about {}, and {} reconstructs the "
                    "right amount but about [{:.3f}, {:.3f}, {:.3f}], which "
                    "is {:.1f} deg away from tool {}. The convention permutes "
                    "the axes -- do not capture with it.".format(
                        angle_deg, axis, convention.label(),
                        unit[0], unit[1], unit[2], off_deg, axis))
    return Result.ok(err)
