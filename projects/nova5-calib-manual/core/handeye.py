# -*- coding: utf-8 -*-
"""
handeye.py
----------
Eye-in-hand calibration logic (no Qt). Uses the modern OpenCV 4.8+
cv2.aruco.CharucoDetector API.

Pipeline per captured pose i:
  - The Nova-5 reports its Cartesian pose directly, so T_mount2base_i comes
    straight from the SDK -- no URDF, no forward kinematics. In OpenCV
    calibrateHandEye terms this is "gripper2base". "mount" here means whatever
    frame the controller reports, i.e. the taught TCP under the active Tool
    index, which is recorded with the calibration and is part of its validity.
  - From the image, detect the ChArUco board and solvePnP gives T_target2cam_i
    ("target2cam").

calibrateHandEye then returns T_cam2mount ("cam2gripper"), the camera pose in
the mount frame. Validation: because the board is fixed in the base frame,
T_target2base_i = T_mount2base_i @ T_cam2mount @ T_target2cam_i should be
identical across all poses; the spread of the board origin (mm) and orientation
(deg) across poses is the quality metric.
"""

from __future__ import annotations

import datetime
import json
import os
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

import intrinsics_math

from models import (
    CameraIntrinsics,
    CharucoBoardConfig,
    HandEyeResult,
    Result,
    RobotPose,
)
from transforms import (
    make_se3,
    rotation_angle_deg,
    rvec_tvec_to_se3,
    se3_compose,
    se3_translation,
)

# ---------------------------------------------------------------------------
# Supported OpenCV range, enforced at import.
# ---------------------------------------------------------------------------
# Lower bound: the CharucoDetector API arrived in 4.7, but
# CharucoBoard.matchImagePoints -- which every board pose below goes through
# -- is broken when called from Python before 4.8.0 (opencv/opencv PR #23138):
# the binding turns the (N, 1, 2) corner array into a Mat vector, which sends
# the call down the ArUco-MARKER branch and pairs image points with the wrong
# object points. So 4.8.0 is the real floor.
# Upper bound: OpenCV 5.0 reorganised calib3d and cv2.calibrateHandEye is not
# in the Python bindings at all -- the solve this whole app exists to perform
# simply is not reachable there. Finding that out as an AttributeError after a
# calibration run is the worst possible time, so it is checked here instead.
_OPENCV_MIN = (4, 8, 0)
_OPENCV_MAX_EXCLUSIVE = (5, 0, 0)


def _parse_version(text: str) -> tuple:
    parts = []
    for chunk in str(text).split(".")[:3]:
        digits = "".join(c for c in chunk if c.isdigit())
        parts.append(int(digits) if digits else 0)
    while len(parts) < 3:
        parts.append(0)
    return tuple(parts)


OPENCV_VERSION = _parse_version(cv2.__version__)
if not (_OPENCV_MIN <= OPENCV_VERSION < _OPENCV_MAX_EXCLUSIVE):
    raise ImportError(
        "This app needs opencv-python >= {} and < {}, but {} is installed.\n"
        "  * Below 4.7 there is no cv2.aruco.CharucoDetector, and below 4.8\n"
        "    CharucoBoard.matchImagePoints returns wrong object points when\n"
        "    called from Python (opencv/opencv PR #23138).\n"
        "  * From 5.0 the Python bindings no longer expose "
        "cv2.calibrateHandEye,\n"
        "    which is the solve this app is built around.\n"
        "Install a supported build:  pip install \"opencv-python>=4.8,<5\"\n"
        "Run tools/check_opencv.py for a full symbol report.".format(
            ".".join(str(v) for v in _OPENCV_MIN),
            ".".join(str(v) for v in _OPENCV_MAX_EXCLUSIVE),
            cv2.__version__))

# Pose quality gates. Few corners LOWER the reprojection error (fewer
# residuals to fit), so a low corner count games the quality gate -- the
# absolute minimum is paired with a fraction-of-board requirement and a
# minimum image-area coverage so tiny/partial detections are rejected.
_MIN_CORNERS = 8
_MIN_BOARD_FRACTION = 0.4       # of the board's total interior corners
_MIN_IMAGE_COVERAGE = 0.02      # detected-corner bbox area / image area
# Planar solvePnP has a two-fold pose ambiguity; accept the best IPPE
# solution only when the runner-up is clearly worse.
_AMBIGUITY_RATIO = 3.0
_MIN_ROT_AXES_DEG = 15.0        # min angle between motion rotation axes
_MIN_MOTION_ROT_DEG = 5.0       # min rotation for a pair to define an axis

_HANDEYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def make_board(cfg: CharucoBoardConfig) -> tuple:
    """Return (board, detector) for the given board configuration.

    Raises AttributeError if the dictionary name is not a valid cv2.aruco
    predefined dictionary (fail fast on a typo in the GUI).
    """
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.dictionary))
    board = cv2.aruco.CharucoBoard(
        (cfg.squares_x, cfg.squares_y),
        cfg.square_length_m,
        cfg.marker_length_m,
        dictionary,
    )
    detector = cv2.aruco.CharucoDetector(board)
    return board, detector


def as_xy(points) -> np.ndarray:
    """Any corner array -> a plain (N, 2) float64 array.

    OpenCV's Python bindings accept (N, 1, 2), (1, N, 2) and (N, 2) for point
    data, and different versions disagree about how to interpret them as a
    Mat: OpenCV 5 reads a (N, 1, 2) float array as an N-row two-CHANNEL matrix
    in some entry points and an N-row two-COLUMN single-channel matrix in
    others. That difference is invisible until a function compares
    corners.total() against ids.total() and gets 2N against N.

    So nothing in this module hands a raw detector array to a drawing
    function. Geometry is normalised here first, and drawn with primitives
    whose interpretation is not in question.
    """
    if points is None:
        return np.zeros((0, 2), dtype=np.float64)
    return np.asarray(points, dtype=np.float64).reshape(-1, 2)



@dataclass
class BoardDetection:
    found: bool
    n_corners: int
    charuco_corners: Optional[np.ndarray] = None
    charuco_ids: Optional[np.ndarray] = None
    marker_corners: Optional[list] = None
    marker_ids: Optional[np.ndarray] = None
    T_target2cam: Optional[np.ndarray] = None
    reproj_error: float = float("inf")
    message: str = ""               # why a pose was rejected, for the UI


@dataclass
class HandEyeSample:
    """One captured (robot pose, board pose) pair.

    `pose` keeps the RAW SDK numbers as reported. T_mount2base is derived from
    it through the instance's PoseConvention, so if the convention is ever
    found to be wrong the samples can be re-derived from disk instead of
    re-captured. `pose` is None only for image-only captures whose poses have
    not been typed in yet.
    """

    T_mount2base: np.ndarray
    T_target2cam: np.ndarray
    reproj_error: float
    n_corners: int
    pose: Optional[RobotPose] = None
    pose_index: int = -1        # authored project pose this replays, or -1
    image_path: str = ""        # raw capture frame on disk (audit / re-solve)
    overlay_path: str = ""      # same frame with the detection drawn on


@dataclass
class SampleResidual:
    index: int
    translation_mm: float  # board-origin distance from the consensus centroid
    rotation_deg: float    # board-rotation deviation from the consensus mean


def detect_board(
    bgr: np.ndarray,
    board,
    detector,
    intrinsics: Optional[CameraIntrinsics] = None,
    min_coverage: Optional[float] = None,
) -> BoardDetection:
    """Detect the ChArUco board and, if intrinsics are given, estimate its pose.

    min_coverage overrides the default image-area coverage gate
    (_MIN_IMAGE_COVERAGE); it is a fraction of the image area, configurable in
    the app settings because the right floor depends on the working distance
    the rig actually uses.
    """
    if board is None or detector is None:
        # make_board() raises on a bad dictionary name, so the tabs can hold a
        # None board -- and this runs on a timer, where an AttributeError
        # would take the window down instead of showing the reason.
        return BoardDetection(found=False, n_corners=0,
                              message="board/detector not configured")
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)

    n = 0 if charuco_ids is None else int(len(charuco_ids))
    det = BoardDetection(
        found=n > 0,
        n_corners=n,
        charuco_corners=charuco_corners,
        charuco_ids=charuco_ids,
        marker_corners=marker_corners,
        marker_ids=marker_ids,
    )
    if charuco_ids is not None:
        n_points = as_xy(charuco_corners).shape[0]
        if n_points != n:
            det.found = False
            det.message = ("detector returned {} corners but {} ids -- the "
                           "detection cannot be used".format(n_points, n))
            return det
    if intrinsics is not None and intrinsics.unsupported_model():
        # Kannala-Brandt / FTheta coefficients are a different polynomial
        # entirely. Handed to solvePnP as distCoeffs they would be absorbed
        # into a wrong pose that still reprojects well, because projectPoints
        # below applies the same wrong model -- so no pose is estimated at
        # all, and the message says why.
        det.message = ("camera reports the '{}' distortion model, which the "
                       "Brown-Conrady path cannot use -- calibrate the "
                       "intrinsics for this stream instead".format(
                           intrinsics.model))
        return det
    board_total = max(1, (board.getChessboardSize()[0] - 1)
                      * (board.getChessboardSize()[1] - 1))
    min_needed = max(_MIN_CORNERS,
                     int(round(_MIN_BOARD_FRACTION * board_total)))
    if intrinsics is None or n < min_needed:
        det.message = ("need {} of {} corners, have {}".format(
            min_needed, board_total, n) if intrinsics is not None else "")
        return det

    # Image-area coverage: a small far-away/partial board yields a badly
    # conditioned pose that still reprojects well.
    coverage_floor = (_MIN_IMAGE_COVERAGE if min_coverage is None
                      else float(min_coverage))
    pts = as_xy(charuco_corners)
    bbox = (pts[:, 0].max() - pts[:, 0].min()) * \
        (pts[:, 1].max() - pts[:, 1].min())
    coverage = float(bbox) / float(bgr.shape[0] * bgr.shape[1])
    if coverage < coverage_floor:
        det.message = "board covers {:.1f}% of image (< {:.1f}%)".format(
            coverage * 100.0, coverage_floor * 100.0)
        return det

    obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
    if obj_points is None or len(obj_points) < min_needed:
        return det

    K = intrinsics.camera_matrix()
    if intrinsics.needs_manual_undistort():
        # librealsense inverse-Brown-Conrady: its coefficients map observed
        # -> ideal, the opposite of OpenCV's convention. Undistort the
        # points exactly with that polynomial and solve with ZERO
        # coefficients; passing them through as distCoeffs would roughly
        # DOUBLE the distortion error in every pose.
        flat = img_points.reshape(-1, 2)
        uu, vv = intrinsics_math.ideal_from_observed(
            flat[:, 0], flat[:, 1], intrinsics.coeffs,
            intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
        img_points = np.stack([uu, vv], axis=1).astype(
            np.float32).reshape(-1, 1, 2)
        dist = intrinsics.zero_dist_coeffs()
    else:
        dist = intrinsics.dist_coeffs()
    # IPPE for planar boards returns BOTH solutions of the two-fold
    # ambiguity with their reprojection errors; a confident pose needs the
    # runner-up to be clearly worse (P6). ITERATIVE silently returns one
    # of the two.
    ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(
        obj_points, img_points, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok or len(rvecs) == 0:
        return det
    order = np.argsort([float(e) for e in np.asarray(errs).reshape(-1)])
    best = int(order[0])
    if len(order) > 1:
        e0 = float(np.asarray(errs).reshape(-1)[order[0]])
        e1 = float(np.asarray(errs).reshape(-1)[order[1]])
        if e0 > 1e-9 and (e1 / e0) < _AMBIGUITY_RATIO:
            det.message = ("ambiguous planar pose (error ratio "
                           "{:.2f} < {:.1f}) -- change the viewing "
                           "angle".format(e1 / e0, _AMBIGUITY_RATIO))
            return det
    rvec, tvec = rvecs[best], tvecs[best]

    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, K, dist)
    per_point = np.linalg.norm(
        projected.reshape(-1, 2) - img_points.reshape(-1, 2), axis=1
    )
    det.reproj_error = float(np.mean(per_point))
    det.T_target2cam = rvec_tvec_to_se3(rvec, tvec)
    return det


_AXIS_LENGTH_M = 0.03


def _draw_axes_ideal_to_observed(out: np.ndarray, det: BoardDetection,
                                 intrinsics: CameraIntrinsics) -> None:
    """Draw the pose axes for a pose that was solved in the IDEAL camera.

    On the exact-undistort path det.T_target2cam comes from undistorted points
    and ZERO coefficients, so it lives in the ideal pinhole camera: projecting
    the axes with the stored coefficients would bend them, and projecting with
    zeros (what cv2.drawFrameAxes would then do) puts them at ideal pixels on
    a frame -- and among corner circles -- that are at OBSERVED pixels. So the
    endpoints are projected with zero distortion and then mapped ideal ->
    observed with the model's own inverse.
    """
    obj = np.array([[0.0, 0.0, 0.0],
                    [_AXIS_LENGTH_M, 0.0, 0.0],
                    [0.0, _AXIS_LENGTH_M, 0.0],
                    [0.0, 0.0, _AXIS_LENGTH_M]], dtype=np.float64)
    cam = obj @ np.asarray(det.T_target2cam[:3, :3]).T \
        + np.asarray(det.T_target2cam[:3, 3])
    if np.any(cam[:, 2] <= 1e-9):
        return                      # an endpoint at or behind the pinhole
    u = intrinsics.fx * (cam[:, 0] / cam[:, 2]) + intrinsics.ppx
    v = intrinsics.fy * (cam[:, 1] / cam[:, 2]) + intrinsics.ppy
    try:
        uo, vo = intrinsics_math.observed_from_ideal(
            u, v, intrinsics.coeffs, intrinsics.fx, intrinsics.fy,
            intrinsics.ppx, intrinsics.ppy)
    except ValueError:
        return                      # no observed preimage: draw no axes
    pts = np.stack([np.asarray(uo).reshape(-1),
                    np.asarray(vo).reshape(-1)], axis=1)
    # Nothing sane to draw -- and cv2.line rejects coordinates outside the C
    # int range rather than clipping them.
    if not np.all(np.isfinite(pts)) or float(np.max(np.abs(pts))) > 1e6:
        return
    origin = (int(round(pts[0, 0])), int(round(pts[0, 1])))
    # cv2.drawFrameAxes' own colours, BGR: X red, Y green, Z blue.
    for index, colour in enumerate(((0, 0, 255), (0, 255, 0), (255, 0, 0))):
        end = (int(round(pts[index + 1, 0])), int(round(pts[index + 1, 1])))
        cv2.line(out, origin, end, colour, 3, lineType=cv2.LINE_AA)


def draw_overlay(bgr: np.ndarray, det: BoardDetection,
                 intrinsics: Optional[CameraIntrinsics]) -> np.ndarray:
    """Draw detected markers, ChArUco corners, and a pose axis onto a copy."""
    out = bgr.copy()

    if det.marker_corners is not None:
        marker_ids = (None if det.marker_ids is None
                      else np.asarray(det.marker_ids, dtype=int).reshape(-1))
        for index, quad in enumerate(det.marker_corners):
            pts = as_xy(quad)
            if pts.shape[0] < 3:
                continue
            cv2.polylines(out, [pts.astype(np.int32).reshape(-1, 1, 2)],
                          True, (0, 200, 0), 1, lineType=cv2.LINE_AA)
            if marker_ids is not None and index < marker_ids.size:
                centre = pts.mean(axis=0)
                cv2.putText(out, str(int(marker_ids[index])),
                            (int(centre[0]) - 6, int(centre[1]) + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 0), 1,
                            lineType=cv2.LINE_AA)

    corners = as_xy(det.charuco_corners)
    corner_ids = (None if det.charuco_ids is None
                  else np.asarray(det.charuco_ids, dtype=int).reshape(-1))
    for index, point in enumerate(corners):
        centre = (int(round(point[0])), int(round(point[1])))
        cv2.circle(out, centre, 4, (0, 0, 255), 1, lineType=cv2.LINE_AA)
        cv2.drawMarker(out, centre, (0, 0, 255), cv2.MARKER_CROSS, 6, 1,
                       line_type=cv2.LINE_AA)
        if corner_ids is not None and index < corner_ids.size:
            cv2.putText(out, str(int(corner_ids[index])),
                        (centre[0] + 5, centre[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1,
                        lineType=cv2.LINE_AA)

    if det.T_target2cam is not None and intrinsics is not None:
        if intrinsics.needs_manual_undistort():
            _draw_axes_ideal_to_observed(out, det, intrinsics)
        else:
            rvec, _ = cv2.Rodrigues(det.T_target2cam[:3, :3])
            tvec = det.T_target2cam[:3, 3].reshape(3, 1)
            cv2.drawFrameAxes(out, intrinsics.camera_matrix(),
                              intrinsics.dist_coeffs(), rvec, tvec,
                              _AXIS_LENGTH_M)
    return out



def draw_ghost_corners(bgr: np.ndarray, ref_corners, ref_ids,
                       det: Optional[BoardDetection] = None,
                       ghost_color=(60, 190, 255),
                       live_color=(80, 255, 120)) -> np.ndarray:
    """Overlay a stored reference detection on the live frame.

    This is what makes a manual re-teach constrain all six degrees of freedom.
    Centring the board in the image pins only about two; matching every stored
    corner to where it was pins the full board-to-camera pose, which is
    exactly the quantity the remount correction needs. Matching corners are
    joined by a line so the operator can see which way to jog.
    """
    out = bgr.copy()
    if ref_corners is None:
        return out
    ref = as_xy(ref_corners)
    ids = (None if ref_ids is None
           else np.asarray(ref_ids, dtype=int).reshape(-1))
    for point in ref:
        cv2.circle(out, (int(round(point[0])), int(round(point[1]))), 5,
                   ghost_color, 1, lineType=cv2.LINE_AA)

    if det is None or det.charuco_corners is None or det.charuco_ids is None:
        return out
    live = as_xy(det.charuco_corners)
    live_ids = np.asarray(det.charuco_ids, dtype=int).reshape(-1)
    lookup = {}
    if ids is not None and len(ids) == len(ref):
        lookup = {int(cid): ref[i] for i, cid in enumerate(ids)}
    for i, cid in enumerate(live_ids):
        point = live[i]
        cv2.circle(out, (int(round(point[0])), int(round(point[1]))), 3,
                   live_color, -1, lineType=cv2.LINE_AA)
        target = lookup.get(int(cid))
        if target is not None:
            cv2.line(out, (int(round(point[0])), int(round(point[1]))),
                     (int(round(target[0])), int(round(target[1]))),
                     live_color, 1, lineType=cv2.LINE_AA)
    return out


def corner_shift_px(a: Optional[BoardDetection],
                    b: Optional[BoardDetection]) -> float:
    """Largest movement of a shared corner between two detections, in pixels.

    The board-static gate: a detection taken while the arm is still ringing,
    or while somebody is leaning on the board, pairs a pose from instant A
    with geometry from instant B and leaves no trace in any later metric.
    """
    if (a is None or b is None or a.charuco_ids is None
            or b.charuco_ids is None):
        return float("inf")
    ids_a = np.asarray(a.charuco_ids, dtype=int).reshape(-1)
    ids_b = np.asarray(b.charuco_ids, dtype=int).reshape(-1)
    pts_a = as_xy(a.charuco_corners)
    pts_b = as_xy(b.charuco_corners)
    lookup = {int(cid): pts_b[i] for i, cid in enumerate(ids_b)}
    worst = 0.0
    shared = 0
    for i, cid in enumerate(ids_a):
        other = lookup.get(int(cid))
        if other is None:
            continue
        shared += 1
        worst = max(worst, float(np.linalg.norm(pts_a[i] - other)))
    if shared == 0:
        return float("inf")
    return worst


def _mean_rotation(rotations: list) -> np.ndarray:
    """Chordal mean rotation: projection of the summed matrices onto SO(3)
    via SVD. Symmetric reference for the spread metric (measuring against
    rotations[0] made the metric depend on sample order)."""
    M = np.zeros((3, 3))
    for R in rotations:
        M += np.asarray(R, dtype=float)
    U, _s, Vt = np.linalg.svd(M)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1.0
        R_mean = U @ Vt
    return R_mean


def check_rotation_diversity(T_mount2base_list: list) -> Result:
    """AX = XB observability gate: the gripper motions must rotate about at
    least two clearly different axes, or translation along the common axis
    is unobservable -- and the residual spread test cannot detect it.

    Uses all sample pairs; a pair contributes an axis when its relative
    rotation exceeds _MIN_MOTION_ROT_DEG. Passes when some two axes differ
    by more than _MIN_ROT_AXES_DEG.
    """
    axes = []
    n = len(T_mount2base_list)
    for i in range(n):
        Ri = np.asarray(T_mount2base_list[i])[:3, :3]
        for j in range(i + 1, n):
            Rj = np.asarray(T_mount2base_list[j])[:3, :3]
            rvec, _ = cv2.Rodrigues(Ri.T @ Rj)
            angle = float(np.linalg.norm(rvec))
            if np.degrees(angle) >= _MIN_MOTION_ROT_DEG:
                axes.append((rvec / angle).reshape(3))
    if len(axes) < 2:
        return Result.err_msg(
            "not enough rotation between samples (need pairs rotated by "
            ">= {:.0f} deg about some axis)".format(_MIN_MOTION_ROT_DEG))
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            cosang = float(np.clip(abs(np.dot(axes[i], axes[j])), 0.0, 1.0))
            if np.degrees(np.arccos(cosang)) >= _MIN_ROT_AXES_DEG:
                return Result.ok(None)
    return Result.err_msg(
        "all sample rotations share one axis (axes within {:.0f} deg of "
        "each other): translation along it is unobservable -- add poses "
        "rotated about a different axis".format(_MIN_ROT_AXES_DEG))


_MIN_SAMPLES = 5


def solve_handeye(samples: list[HandEyeSample], method: str = "TSAI") -> Result[HandEyeResult]:
    """Run cv2.calibrateHandEye over the samples and compute spread metrics.

    Guards: minimum sample count AND rotation-axis diversity (see
    check_rotation_diversity) before the solver runs.
    """
    if method not in _HANDEYE_METHODS:
        return Result.err_msg(f"Unknown hand-eye method '{method}'.")
    if len(samples) < _MIN_SAMPLES:
        return Result.err_msg(
            "Need at least {} valid poses for a trustworthy hand-eye "
            "calibration.".format(_MIN_SAMPLES))
    diversity = check_rotation_diversity([s.T_mount2base for s in samples])
    if diversity.is_err:
        return diversity

    R_g2b = [s.T_mount2base[:3, :3] for s in samples]
    t_g2b = [s.T_mount2base[:3, 3] for s in samples]
    R_t2c = [s.T_target2cam[:3, :3] for s in samples]
    t_t2c = [s.T_target2cam[:3, 3] for s in samples]

    R_c2m, t_c2m = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, method=_HANDEYE_METHODS[method]
    )
    T_cam2mount = make_se3(R_c2m, t_c2m)

    # Validation: board pose in base frame should be constant across poses.
    origins = []
    rotations = []
    for s in samples:
        T_target2base = se3_compose(s.T_mount2base, T_cam2mount, s.T_target2cam)
        origins.append(se3_translation(T_target2base))
        rotations.append(T_target2base[:3, :3])
    origins = np.array(origins)
    centroid = origins.mean(axis=0)
    origin_spread_mm = float(np.max(np.linalg.norm(origins - centroid, axis=1)) * 1000.0)
    R_mean = _mean_rotation(rotations)
    rot_spread_deg = float(max(rotation_angle_deg(R_mean, R)
                               for R in rotations))

    result = HandEyeResult(
        T_cam2mount=T_cam2mount,
        method=method,
        num_samples=len(samples),
        board_origin_spread_mm=origin_spread_mm,
        rotation_spread_deg=rot_spread_deg,
        timestamp=datetime.datetime.now().isoformat(timespec="seconds"),
    )
    return Result.ok(result)


def compute_residuals(samples: list[HandEyeSample], T_cam2mount: np.ndarray) -> list[SampleResidual]:
    """Per-pose agreement with the solved calibration. For each pose the board
    pose in the base frame is T_mount2base @ T_cam2mount @ T_target2cam; since the
    board is fixed, every pose should agree. The translation residual is the
    distance of that pose's board origin from the consensus centroid (its max is
    board_origin_spread_mm); the rotation residual is the deviation from the
    chordal mean board rotation (its max is rotation_spread_deg). Both references
    are the consensus rather than sample 0, so no sample is exonerated by being
    first and neither metric depends on sample order. Larger = worse."""
    origins = []
    rotations = []
    for s in samples:
        T_target2base = se3_compose(s.T_mount2base, T_cam2mount, s.T_target2cam)
        origins.append(se3_translation(T_target2base))
        rotations.append(T_target2base[:3, :3])
    origins = np.array(origins)
    centroid = origins.mean(axis=0)
    R_mean = _mean_rotation(rotations)
    residuals = []
    for i, (origin, rot) in enumerate(zip(origins, rotations)):
        t_mm = float(np.linalg.norm(origin - centroid) * 1000.0)
        r_deg = float(rotation_angle_deg(R_mean, rot))
        residuals.append(SampleResidual(index=i, translation_mm=t_mm, rotation_deg=r_deg))
    return residuals


def sample_to_dict(s: HandEyeSample) -> dict:
    return {
        "T_mount2base": np.asarray(s.T_mount2base, dtype=float).tolist(),
        "T_target2cam": np.asarray(s.T_target2cam, dtype=float).tolist(),
        "reproj_error": s.reproj_error,
        "n_corners": s.n_corners,
        "pose": None if s.pose is None else s.pose.to_dict(),
        "pose_index": int(s.pose_index),
        "image_path": s.image_path,
        "overlay_path": s.overlay_path,
    }


def sample_from_dict(d: dict) -> HandEyeSample:
    return HandEyeSample(
        T_mount2base=np.array(d["T_mount2base"], dtype=float),
        T_target2cam=np.array(d["T_target2cam"], dtype=float),
        reproj_error=float(d.get("reproj_error", 0.0)),
        n_corners=int(d.get("n_corners", 0)),
        pose=(None if d.get("pose") is None
              else RobotPose.from_dict(d["pose"])),
        pose_index=int(d.get("pose_index", -1)),
        image_path=str(d.get("image_path", "")),
        overlay_path=str(d.get("overlay_path", "")),
    )


def save_handeye(path: str, result: Optional[HandEyeResult],
                 samples: Optional[list[HandEyeSample]] = None,
                 camera: Optional[dict] = None,
                 board: Optional[dict] = None,
                 image_dir: str = "") -> None:
    """Write the calibration file.

    Besides the solve result and samples it records the CAMERA INTRINSICS
    (including resolution and distortion model) and the BOARD GEOMETRY the
    samples were captured with. Both are needed to know whether a stored
    calibration is valid for a later session: intrinsics scale with
    resolution, so the same T_cam2mount is wrong the moment the stream
    resolution changes.
    """
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    payload = {}
    if result is not None:
        payload["result"] = result.to_dict()
    if camera is not None:
        payload["camera"] = camera
    if board is not None:
        payload["board"] = board
    if image_dir:
        payload["image_dir"] = image_dir
    if samples is not None:
        payload["samples"] = [sample_to_dict(s) for s in samples]
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    os.replace(tmp, path)


def load_handeye(path: str) -> Result[HandEyeResult]:
    if not os.path.exists(path):
        return Result.err_msg(f"Hand-eye file not found: {path}")
    try:
        with open(path, "r", encoding="utf-8") as f:
            payload = json.load(f)
    except json.JSONDecodeError as exc:
        return Result.err_msg(f"Malformed hand-eye JSON: {exc}")
    if "result" not in payload:
        return Result.err_msg("Hand-eye file missing 'result' section.")
    return Result.ok(HandEyeResult.from_dict(payload["result"]))


def load_handeye_payload(path: str) -> Result:
    """Whole file: {"result"?, "samples"?, "camera"?, "board"?,
    "image_dir"?}. Older files simply lack the newer keys."""
    if not os.path.exists(path):
        return Result.err_msg(f"Hand-eye file not found: {path}")
    try:
        with open(path, "r", encoding="utf-8") as f:
            payload = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        return Result.err_msg(f"Malformed hand-eye JSON: {exc}")
    return Result.ok(payload)


def board_geometry(cfg) -> dict:
    """Board description for the calibration file: counts AND physical
    size, so a file can be checked against the board actually in use."""
    squares_x = int(getattr(cfg, "squares_x", 0))
    squares_y = int(getattr(cfg, "squares_y", 0))
    sq = float(getattr(cfg, "square_length_m", 0.0))
    return {
        "type": "charuco",
        "squares_x": squares_x,
        "squares_y": squares_y,
        "square_length_m": sq,
        "marker_length_m": float(getattr(cfg, "marker_length_m", 0.0)),
        "dictionary": str(getattr(cfg, "dictionary", "")),
        "width_m": round(squares_x * sq, 6),
        "height_m": round(squares_y * sq, 6),
        "interior_corners_x": max(0, squares_x - 1),
        "interior_corners_y": max(0, squares_y - 1),
    }