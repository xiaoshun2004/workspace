# -*- coding: utf-8 -*-
"""
api/intrinsic_calib.py
----------------------
Solve camera intrinsics from ChArUco views with board.matchImagePoints()
plus cv2.calibrateCamera().

Needed unconditionally for a USB camera (no factory values exist) and
optionally for a RealSense, where a careful solve can beat the factory numbers
-- or expose that the factory numbers are fine, which is also worth knowing.

The result is written to the intrinsics library as a record carrying its RMS,
its image count and its RESOLUTION. That last one is the whole reason the
library is keyed the way it is: fx, fy, ppx and ppy all scale with stream
size, so a set solved at 1280x720 is simply wrong at 640x480.

Output convention: OpenCV FORWARD Brown-Conrady coefficients, tagged
model="brown_conrady". That is the opposite sense from what a RealSense
reports for its colour stream, and tagging it explicitly is what stops the
rest of the app applying the exact-undistort path to coefficients that do not
need it.

Quality guidance, from the OpenCV documentation and hard experience:
  * 15-25 views, not 5. Fewer views leave the distortion terms unconstrained.
  * Tilt the board. A set of fronto-parallel views cannot separate focal
    length from working distance, and the solve will happily report a
    confident, wrong fx.
  * Fill the corners of the frame. Distortion is largest at the edges, so a
    board that never leaves the middle gives coefficients fitted to noise.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from models import CameraIntrinsics, Result

MIN_VIEWS = 6
RECOMMENDED_VIEWS = 15
# Per-view corner requirement. Small partial detections both fit well and
# condition badly, exactly as in the hand-eye path.
MIN_CORNERS_PER_VIEW = 8
MIN_BOARD_FRACTION = 0.35
# Above this the solve is not trustworthy; below 0.5 px is a good result.
MAX_ACCEPTABLE_RMS_PX = 1.0


@dataclass
class IntrinsicView:
    """One accepted view: its ChArUco corners and where they landed."""

    corners: np.ndarray                 # (N, 1, 2) float32
    ids: np.ndarray                     # (N, 1) int32
    n_corners: int
    image_path: str = ""
    coverage: float = 0.0               # fraction of image area spanned


@dataclass
class IntrinsicCalibResult:
    intrinsics: CameraIntrinsics
    rms_px: float
    n_views: int
    per_view_error_px: List[float] = field(default_factory=list)
    coverage_report: str = ""


def add_view(bgr: np.ndarray, board, detector,
             views: List[IntrinsicView]) -> Result[IntrinsicView]:
    """Detect the board and append the view if it is worth keeping."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _marker_corners, _marker_ids = detector.detectBoard(gray)
    n = 0 if ids is None else int(len(ids))
    total = max(1, (board.getChessboardSize()[0] - 1)
                * (board.getChessboardSize()[1] - 1))
    needed = max(MIN_CORNERS_PER_VIEW, int(round(MIN_BOARD_FRACTION * total)))
    if n < needed:
        return Result.err_msg(
            "need {} of {} corners, found {}".format(needed, total, n))

    pts = corners.reshape(-1, 2)
    span = ((pts[:, 0].max() - pts[:, 0].min())
            * (pts[:, 1].max() - pts[:, 1].min()))
    coverage = float(span) / float(bgr.shape[0] * bgr.shape[1])
    view = IntrinsicView(corners=corners, ids=ids, n_corners=n,
                         coverage=coverage)
    views.append(view)
    return Result.ok(view)


def coverage_report(views: Sequence[IntrinsicView], width: int,
                    height: int) -> str:
    """Where the accepted views actually put the board, as a 3x3 occupancy
    grid. Distortion lives at the edges, so empty corners in this grid are the
    single best predictor of coefficients that will not generalise."""
    if not views:
        return "no views"
    grid = np.zeros((3, 3), dtype=int)
    for view in views:
        pts = np.asarray(view.corners, dtype=np.float64).reshape(-1, 2)
        cx = float(np.mean(pts[:, 0])) / max(1.0, float(width))
        cy = float(np.mean(pts[:, 1])) / max(1.0, float(height))
        col = min(2, max(0, int(cx * 3.0)))
        row = min(2, max(0, int(cy * 3.0)))
        grid[row, col] += 1
    empty = int(np.sum(grid == 0))
    rows = " / ".join(" ".join(str(v) for v in row) for row in grid)
    if empty == 0:
        return "board visited all nine image regions [{}]".format(rows)
    return ("board never reached {} of the nine image regions [{}] -- "
            "distortion at the edges is under-constrained".format(empty, rows))


def solve_intrinsics(views: Sequence[IntrinsicView], board,
                     image_size: Tuple[int, int]) -> Result[IntrinsicCalibResult]:
    """Run the intrinsic solve over the accepted views.

    Built on board.matchImagePoints() plus cv2.calibrateCamera() rather than
    cv2.aruco.calibrateCameraCharuco(). Both exist in the supported OpenCV
    range, but the latter has been deprecated since 4.7 and warns; this pair
    is what it calls internally anyway, so nothing is lost by going direct.

    image_size is (width, height) and MUST be the size the views were captured
    at; the resulting intrinsics are only valid there.
    """
    if len(views) < MIN_VIEWS:
        return Result.err_msg(
            "Need at least {} views for an intrinsic calibration (have {}); "
            "{} or more is what actually produces stable "
            "coefficients.".format(MIN_VIEWS, len(views), RECOMMENDED_VIEWS))

    object_points = []
    image_points = []
    used: List[IntrinsicView] = []
    for view in views:
        obj, img = board.matchImagePoints(view.corners, view.ids)
        if obj is None or img is None or len(obj) < 4:
            continue
        object_points.append(np.asarray(obj, dtype=np.float32).reshape(-1, 1, 3))
        image_points.append(np.asarray(img, dtype=np.float32).reshape(-1, 1, 2))
        used.append(view)

    if len(used) < MIN_VIEWS:
        return Result.err_msg(
            "Only {} of {} views produced usable point correspondences; need "
            "at least {}.".format(len(used), len(views), MIN_VIEWS))

    try:
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            object_points, image_points,
            (int(image_size[0]), int(image_size[1])), None, None)
    except cv2.error as exc:
        return Result.err_msg("Intrinsic solve failed: {}".format(exc))

    if not np.isfinite(rms):
        return Result.err_msg(
            "Intrinsic solve returned a non-finite RMS; the view set is "
            "degenerate. Tilt the board and re-capture.")

    per_view = []
    for index in range(len(used)):
        projected, _ = cv2.projectPoints(object_points[index], rvecs[index],
                                         tvecs[index], K, dist)
        err = np.linalg.norm(
            projected.reshape(-1, 2) - image_points[index].reshape(-1, 2),
            axis=1)
        per_view.append(float(np.mean(err)))

    intrinsics = CameraIntrinsics(
        width=int(image_size[0]), height=int(image_size[1]),
        fx=float(K[0, 0]), fy=float(K[1, 1]),
        ppx=float(K[0, 2]), ppy=float(K[1, 2]),
        coeffs=[float(c) for c in np.asarray(dist).reshape(-1)[:5]],
        # OpenCV's own convention, which is the FORWARD one. Tagging it
        # explicitly keeps the exact-undistort path away from coefficients
        # that must not be inverted.
        model="brown_conrady")

    result = IntrinsicCalibResult(
        intrinsics=intrinsics, rms_px=float(rms), n_views=len(used),
        per_view_error_px=per_view,
        coverage_report=coverage_report(used, image_size[0], image_size[1]))

    if rms > MAX_ACCEPTABLE_RMS_PX:
        return Result.err(
            result,
            "Solved, but RMS is {:.2f} px (want under {:.1f}). {}. Re-capture "
            "with more tilt and better frame coverage before trusting "
            "these.".format(rms, MAX_ACCEPTABLE_RMS_PX,
                            result.coverage_report))
    return Result.ok(result)


def compare_to_factory(solved: CameraIntrinsics,
                       factory: Optional[CameraIntrinsics]) -> str:
    """One line on whether a solve actually disagrees with the device.

    Worth printing before replacing factory intrinsics: if the two agree to a
    pixel or so, the factory values were fine and the solve has only added a
    way to get it wrong later.
    """
    if factory is None:
        return "no factory intrinsics to compare against"
    if solved.resolution() != factory.resolution():
        return "resolutions differ, not comparable"
    d_f = max(abs(solved.fx - factory.fx), abs(solved.fy - factory.fy))
    d_p = max(abs(solved.ppx - factory.ppx), abs(solved.ppy - factory.ppy))
    if d_f < 2.0 and d_p < 2.0:
        return ("agrees with the factory values to {:.2f} px focal / {:.2f} px "
                "principal point -- the factory intrinsics were already "
                "good".format(d_f, d_p))
    return ("differs from the factory values by {:.1f} px focal / {:.1f} px "
            "principal point".format(d_f, d_p))