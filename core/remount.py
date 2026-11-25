# -*- coding: utf-8 -*-
"""
core/remount.py
---------------
Replaying an authored pose set onto a camera that has moved on the flange.

THE ALGEBRA (worth reading once; everything here follows from it)

    X   = T_cam2mount      what calibrateHandEye returns
    G_i = T_mount2base     the authored pose i
    B   = T_board2base     fixed, permanently, by assumption

    view_i := T_board2cam,i = X^-1 . G_i^-1 . B

Remount the camera, X -> X'. The authored poses are unchanged, so

    view'_i = X'^-1 . G_i^-1 . B = (X^-1 X')^-1 . view_i

The view shifts by ONE rigid transform, identical at every pose. That is the
whole reason this module can exist: the damage is systematic, not per-pose, so
a single measured correction repairs the entire project.

The correction is a flange-frame RIGHT multiplication:

    G_i' = G_i . delta          with     delta = X . X'^-1

Check: X'^-1 (G_i delta)^-1 B = X'^-1 delta^-1 G_i^-1 B = X^-1 G_i^-1 B = view_i.
Exact, for every pose, for arbitrarily large remounts.

And delta is measurable without knowing either X or X'. Re-teach any single
pose -- jog until the live board view matches that pose's stored reference
view -- and read it off:

    delta = G_ref^-1 . G_new

WHAT NOT TO DO

Storing poses as home-relative deltas and replaying from a re-taught home,
i.e. G_i' = G_home' . (G_home^-1 G_i), is WRONG. It silently assumes delta
commutes with every stored relative pose, which it does not. Projects here
store ABSOLUTE poses and correct by right multiplication.

WHY EVERY POSE IS AN ANCHOR, NOT JUST HOME

delta is the same at every pose, so each re-taught pose yields an independent
estimate. Home is special only ergonomically: it has the most image margin, so
it is the pose most likely still to see the board before any correction. Once
the first estimate is applied and the board is back in view everywhere, the
rest of the project can refine it -- and the SPREAD of the per-pose estimates
is a free diagnostic:

    board moved too:  G_i' = delta_L . G_i . delta_R
                      => delta_i = G_i^-1 delta_L G_i delta_R, depends on i.

So a tight spread means a clean remount (apply the mean); a spread that grows
with distance from the anchor means the board moved, or the pose convention is
wrong, which produces the same signature and is worth catching early. The
board is nailed down by assumption in this setup, which makes a wide spread a
strong hint to check the convention first.

WITH ONE CONDITION, which summarise() enforces rather than assumes. That
diagnostic only works while each estimate is an INDEPENDENT measurement, i.e.
while the pose was re-observed against its stored reference view -- which is
what a finite DeltaEstimate.residual_mm / residual_deg records. Replay a pose
without that re-observation and its estimate is G_ref^-1 (G_ref delta) =
delta, identically: the spread collapses to arm repeatability and reports a
clean remount no matter where the board went. So a "clean remount" verdict is
only issued when at least two estimates carry board-verified residuals; the
rest say plainly that they are arm-side only.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import cv2
import numpy as np

from models import CameraIntrinsics, RemountRecord, Result
from transforms import (
    rotation_angle_deg,
    se3_inverse,
    se3_translation,
)

# Per-pose delta estimates scattering wider than this are not one rigid
# remount. Chosen to sit well above arm repeatability plus ChArUco pose noise
# (which together run a few tenths of a mm and well under a tenth of a degree
# at these ranges) and well below anything a moved board would produce.
_SPREAD_OK_MM = 3.0
_SPREAD_OK_DEG = 1.0
# A single auto-realign step is clamped to this, so a bad approximation of X
# cannot fling the arm across the cell.
_MAX_STEP_MM = 40.0
_MAX_STEP_DEG = 12.0
# The servo has converged when the live view matches the reference this well.
_CONVERGED_MM = 1.0
_CONVERGED_DEG = 0.3
_MAX_SERVO_ITERS = 6


@dataclass
class DeltaEstimate:
    """One pose's answer to "how far did the camera move on the flange"."""

    pose_index: int
    delta: np.ndarray               # 4x4, metres
    residual_mm: float = float("nan")   # view match achieved at this pose
    residual_deg: float = float("nan")


def delta_from_reteach(T_mount2base_ref: np.ndarray,
                       T_mount2base_new: np.ndarray) -> np.ndarray:
    """delta = G_ref^-1 . G_new, the flange-frame correction."""
    return se3_inverse(np.asarray(T_mount2base_ref, dtype=np.float64)) @ \
        np.asarray(T_mount2base_new, dtype=np.float64)


def apply_delta(T_mount2base: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """Corrected pose for replay: G_i' = G_i . delta.

    Right multiplication, always. See the module docstring for why the
    tempting relative-to-home form is wrong.
    """
    return np.asarray(T_mount2base, dtype=np.float64) @ \
        np.asarray(delta, dtype=np.float64)


def _mean_rotation(rotations: Sequence[np.ndarray]) -> np.ndarray:
    """Chordal mean: sum the matrices and project onto SO(3) via SVD. Order
    independent, unlike measuring everything against the first sample."""
    M = np.zeros((3, 3), dtype=np.float64)
    for R in rotations:
        M += np.asarray(R, dtype=np.float64)
    U, _s, Vt = np.linalg.svd(M)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1.0
        R_mean = U @ Vt
    return R_mean


def mean_delta(estimates: Sequence[DeltaEstimate]) -> np.ndarray:
    """Average several per-pose estimates of the same physical remount."""
    if not estimates:
        raise ValueError("mean_delta needs at least one estimate")
    R_mean = _mean_rotation([e.delta[:3, :3] for e in estimates])
    t_mean = np.mean([se3_translation(e.delta) for e in estimates], axis=0)
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = R_mean
    out[:3, 3] = t_mean
    return out


def delta_spread(estimates: Sequence[DeltaEstimate]) -> Tuple[float, float]:
    """(translation mm, rotation deg) scatter of the estimates about their
    mean. This is the board-moved / wrong-convention detector."""
    if len(estimates) < 2:
        return (0.0, 0.0)
    mean = mean_delta(estimates)
    t_mean = se3_translation(mean)
    R_mean = mean[:3, :3]
    spread_mm = max(float(np.linalg.norm(se3_translation(e.delta) - t_mean))
                    * 1000.0 for e in estimates)
    spread_deg = max(rotation_angle_deg(R_mean, e.delta[:3, :3])
                     for e in estimates)
    return (spread_mm, float(spread_deg))


def board_verified(estimates: Sequence[DeltaEstimate]) -> List[DeltaEstimate]:
    """The estimates that were actually confirmed against the board.

    residual_mm / residual_deg are only finite when the live view was compared
    to that pose's stored reference view, and only small when the servo
    converged onto it. Without that, an estimate is G_ref^-1 G_new computed
    from the pose the arm was COMMANDED to and the pose it reported reaching:
    pure arm-side arithmetic, which reproduces the anchor delta by
    construction. A set of those agreeing with each other says the arm
    repeats, and nothing whatsoever about where the camera is looking.
    """
    out: List[DeltaEstimate] = []
    for e in estimates:
        if (np.isfinite(e.residual_mm) and np.isfinite(e.residual_deg)
                and e.residual_mm <= _CONVERGED_MM
                and e.residual_deg <= _CONVERGED_DEG):
            out.append(e)
    return out


def summarise(estimates: Sequence[DeltaEstimate]) -> RemountRecord:
    """Fold per-pose estimates into the record stored in instance metadata,
    with a verdict written for a human rather than a log parser.

    A clean remount is only ever claimed on the strength of estimates that
    were checked against the board; otherwise the verdict says so in as many
    words, because a tight spread of unverified estimates is exactly what a
    moved board or a wrong convention also produces.
    """
    if not estimates:
        return RemountRecord(verdict="no re-teach performed")
    mean = mean_delta(estimates)
    spread_mm, spread_deg = delta_spread(estimates)
    shift_mm = float(np.linalg.norm(se3_translation(mean))) * 1000.0
    turn_deg = rotation_angle_deg(np.eye(3), mean[:3, :3])
    verified = board_verified(estimates)

    if len(estimates) == 1:
        verdict = ("single anchor (pose {}): camera moved {:.1f} mm / "
                   "{:.2f} deg on the flange. No cross-check available -- "
                   "re-teach a second pose to confirm.".format(
                       estimates[0].pose_index, shift_mm, turn_deg))
    elif spread_mm <= _SPREAD_OK_MM and spread_deg <= _SPREAD_OK_DEG:
        if len(verified) >= 2:
            verdict = ("consistent across {} anchors, {} of them verified "
                       "against the board: camera moved {:.1f} mm / {:.2f} "
                       "deg, estimates agree to {:.2f} mm / {:.3f} deg. Clean "
                       "remount; mean correction applied.".format(
                           len(estimates), len(verified), shift_mm, turn_deg,
                           spread_mm, spread_deg))
        else:
            verdict = ("consistent across {} anchors: camera moved {:.1f} mm "
                       "/ {:.2f} deg, estimates agree to {:.2f} mm / {:.3f} "
                       "deg -- but {} of the {} carry a board-verified "
                       "residual. UNVERIFIED: these estimates are arm-side "
                       "only, so the agreement proves the arm repeats, not "
                       "that the correction is right. A moved board or a "
                       "wrong pose convention would look exactly like this. "
                       "Re-teach a second pose against the ghosted reference "
                       "view before trusting it.".format(
                           len(estimates), shift_mm, turn_deg,
                           spread_mm, spread_deg,
                           len(verified), len(estimates)))
    else:
        verdict = ("INCONSISTENT across {} anchors ({:.1f} mm / {:.2f} deg "
                   "scatter, above the {:.1f} mm / {:.1f} deg limit). A pure "
                   "camera remount would give the same delta everywhere. "
                   "Most likely the board moved, or the pose convention is "
                   "wrong. Verify the convention, confirm the board is still "
                   "fixed, then re-author the project rather than trusting "
                   "this correction.".format(
                       len(estimates), spread_mm, spread_deg,
                       _SPREAD_OK_MM, _SPREAD_OK_DEG))

    return RemountRecord(
        delta=mean,
        anchor_indices=[e.pose_index for e in estimates],
        spread_mm=spread_mm,
        spread_deg=spread_deg,
        per_pose_deltas=[np.asarray(e.delta, float).tolist()
                         for e in estimates],
        per_pose_residuals=[[float(e.residual_mm), float(e.residual_deg)]
                            for e in estimates],
        verdict=verdict,
    )


def view_error(T_board2cam_live: np.ndarray,
               T_board2cam_ref: np.ndarray) -> Tuple[float, float]:
    """How far the live view is from the reference view: (mm, deg)."""
    live = np.asarray(T_board2cam_live, dtype=np.float64)
    ref = np.asarray(T_board2cam_ref, dtype=np.float64)
    d_mm = float(np.linalg.norm(se3_translation(live) - se3_translation(ref))) \
        * 1000.0
    d_deg = rotation_angle_deg(ref[:3, :3], live[:3, :3])
    return (d_mm, d_deg)


def servo_step(T_board2cam_live: np.ndarray,
               T_board2cam_ref: np.ndarray,
               X_prev: np.ndarray,
               max_step_mm: float = _MAX_STEP_MM,
               max_step_deg: float = _MAX_STEP_DEG) -> np.ndarray:
    """One iteration of the auto-realign: the flange-frame move that would
    bring the live view onto the reference view.

    Moving the flange G -> G.d changes the view by

        view_new = X'^-1 d^-1 X' . view_live

    Setting view_new = view_ref and solving for d gives

        d = X' . view_live . view_ref^-1 . X'^-1

    X' is unknown -- it is what we are about to calibrate -- so the previous
    instance's X stands in for it. That makes this a Newton step with an
    approximate Jacobian: each application overshoots or undershoots slightly,
    and re-observing and repeating converges in a handful of iterations for
    any plausible remount. The step is clamped so a poor X_prev cannot produce
    a large motion.
    """
    X = np.asarray(X_prev, dtype=np.float64)
    live = np.asarray(T_board2cam_live, dtype=np.float64)
    ref = np.asarray(T_board2cam_ref, dtype=np.float64)
    delta = X @ live @ se3_inverse(ref) @ se3_inverse(X)
    return clamp_delta(delta, max_step_mm, max_step_deg)


def clamp_delta(delta: np.ndarray, max_mm: float,
                max_deg: float) -> np.ndarray:
    """Scale a correction down until it is within the per-step limits.

    Rotation and translation are scaled by the same factor so the motion keeps
    its direction; a clamped step simply gets there over more iterations.
    """
    delta = np.asarray(delta, dtype=np.float64).reshape(4, 4)
    t = se3_translation(delta)
    dist_mm = float(np.linalg.norm(t)) * 1000.0
    rvec, _ = cv2.Rodrigues(delta[:3, :3])
    angle_deg = float(np.degrees(np.linalg.norm(rvec)))

    scale = 1.0
    if dist_mm > max_mm > 0.0:
        scale = min(scale, max_mm / dist_mm)
    if angle_deg > max_deg > 0.0:
        scale = min(scale, max_deg / angle_deg)
    if scale >= 1.0:
        return delta

    out = np.eye(4, dtype=np.float64)
    R_scaled, _ = cv2.Rodrigues(rvec * scale)
    out[:3, :3] = R_scaled
    out[:3, 3] = t * scale
    return out


def camera_from_delta(delta: np.ndarray, X_prev: np.ndarray) -> np.ndarray:
    """The remounted camera transform implied by a measured delta.

        delta = X . X'^-1        =>        X' = delta^-1 . X

    This is what makes the whole re-teach loop practical. servo_step() is only
    a contraction while X_prev is CLOSE to the true X' -- feed it a stale X
    after a large remount and it will wander instead of converging. But the
    first anchor does not need the servo: an operator matches the ghosted
    reference corners by hand, which yields delta directly, and that delta
    converts the previous instance's X into a near-exact X' for every
    remaining pose. So the order is always: one manual anchor, then servo the
    rest with the bootstrapped X'.
    """
    return se3_inverse(np.asarray(delta, dtype=np.float64)) @ \
        np.asarray(X_prev, dtype=np.float64)


def is_converged(T_board2cam_live: np.ndarray,
                 T_board2cam_ref: np.ndarray,
                 tol_mm: float = _CONVERGED_MM,
                 tol_deg: float = _CONVERGED_DEG) -> bool:
    d_mm, d_deg = view_error(T_board2cam_live, T_board2cam_ref)
    return d_mm <= tol_mm and d_deg <= tol_deg


def max_servo_iterations() -> int:
    return _MAX_SERVO_ITERS


# ---------------------------------------------------------------------------
# Authoring-time tolerance
# ---------------------------------------------------------------------------
def view_margin_deg(corners: np.ndarray, image_width: int, image_height: int,
                    intrinsics: CameraIntrinsics) -> float:
    """How much camera rotation this pose can absorb before the board starts
    leaving the frame, in degrees.

    The dominant term when the camera rotates by theta about the flange is the
    swing of the optical axis: the image content translates by roughly
    f.tan(theta) pixels, regardless of the lever arm. So the margin is the
    smallest distance from the detected corners to any image edge, converted
    back through the focal length:

        margin = atan(min_edge_distance_px / f)

    The optical-centre arc contributes f.|d_perp|/Z on top, which is second
    order at realistic lever arms and is deliberately ignored here -- this is
    a design aid for authoring poses with slack, not a safety gate.
    """
    pts = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    if pts.size == 0:
        return 0.0
    f = float(min(intrinsics.fx, intrinsics.fy))
    if f <= 0.0:
        return 0.0
    left = float(np.min(pts[:, 0]))
    right = float(image_width) - float(np.max(pts[:, 0]))
    top = float(np.min(pts[:, 1]))
    bottom = float(image_height) - float(np.max(pts[:, 1]))
    edge_px = max(0.0, min(left, right, top, bottom))
    return float(np.degrees(np.arctan(edge_px / f)))


def project_margin_report(margins: Sequence[float]) -> str:
    """One line naming the weakest pose, because that is what actually limits
    how much remount the project survives without a re-teach."""
    finite = [(i, m) for i, m in enumerate(margins) if np.isfinite(m)]
    if not finite:
        return "no margin data"
    worst_index, worst = min(finite, key=lambda pair: pair[1])
    best = max(m for _i, m in finite)
    return ("project tolerates about {:.1f} deg of camera remount before "
            "pose {} loses the board (best pose: {:.1f} deg)".format(
                worst, worst_index, best))


def check_distance_band(T_board2cam: np.ndarray, min_m: float,
                        max_m: float) -> Result[float]:
    """Working distance gate. Returns the distance in metres either way, so
    the UI can show the number alongside the complaint."""
    d = float(np.linalg.norm(se3_translation(T_board2cam)))
    if d < min_m:
        return Result.err(d, "board is {:.3f} m away, closer than the "
                             "project minimum {:.3f} m".format(d, min_m))
    if d > max_m:
        return Result.err(d, "board is {:.3f} m away, beyond the project "
                             "maximum {:.3f} m".format(d, max_m))
    return Result.ok(d)
