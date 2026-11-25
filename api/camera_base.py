# -*- coding: utf-8 -*-
"""
api/camera_base.py
------------------
The camera interface the UI talks to, so nothing above this layer knows
whether it is looking at a RealSense or a plain webcam.

The two backends differ in exactly two ways that matter to calibration:

  * INTRINSICS. A RealSense reports factory intrinsics for its colour stream.
    A USB camera reports nothing at all, so its intrinsics must come from the
    library (solved once with api/intrinsic_calib.py) or the app refuses to
    estimate board poses with it. has_factory_intrinsics() is that distinction.

  * DEPTH. Only a RealSense can turn a clicked pixel into a 3D point on its
    own. has_depth() is that distinction; the verify tab falls back to asking
    the operator for a distance when it is False.

Everything returns a Result. Nothing raises across this boundary.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np

from models import CameraConfig, CameraIntrinsics, CameraKind, Result


class CameraBase:
    """Common surface for every camera backend."""

    def __init__(self, config: CameraConfig) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._color: Optional[np.ndarray] = None
        self._timestamp: float = 0.0
        # Intrinsics supplied from the library, used when the device has none
        # of its own. Set by the UI after connect().
        self._override_intrinsics: Optional[CameraIntrinsics] = None

    # ------------------------------------------------------------ capability
    @staticmethod
    def kind() -> CameraKind:
        raise NotImplementedError

    def has_depth(self) -> bool:
        return False

    def has_factory_intrinsics(self) -> bool:
        return False

    # ------------------------------------------------------------- lifecycle
    @property
    def is_connected(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def connect(self) -> Result[None]:
        raise NotImplementedError

    def disconnect(self) -> Result[None]:
        raise NotImplementedError

    # -------------------------------------------------------------- frames
    def get_color(self) -> Result[np.ndarray]:
        with self._lock:
            if self._color is None:
                return Result.err_msg("No colour frame available yet.")
            return Result.ok(self._color.copy())

    def frame_age_s(self) -> float:
        """Seconds since the last frame arrived. The capture gate uses this:
        pairing a board detection from instant A with a robot pose from
        instant B is invisible in every metric afterwards."""
        with self._lock:
            if self._timestamp <= 0.0:
                return float("inf")
            return time.perf_counter() - self._timestamp

    def get_depth_colormap(self) -> Result[np.ndarray]:
        return Result.err_msg("This camera has no depth stream.")

    def depth_at(self, u: int, v: int, patch: int = 2) -> Result[float]:
        return Result.err_msg("This camera has no depth stream.")

    def deproject(self, u: float, v: float,
                  depth_m: float) -> Result[Tuple[float, float, float]]:
        """Pixel + distance -> 3D point in the camera optical frame.

        The default implementation is the pinhole model using whatever
        intrinsics are in force, which is what a USB camera gets once the
        operator supplies a distance by hand. Distortion is removed first when
        the model requires it.
        """
        intr = self.intrinsics()
        if intr.is_err:
            return Result.err_msg(intr.message)
        if depth_m <= 0.0:
            return Result.err_msg("Distance must be positive.")
        i = intr.data
        uu, vv = float(u), float(v)
        if i.needs_manual_undistort():
            uu, vv = _ideal_from_observed(uu, vv, i)
        elif i.has_distortion():
            pts = np.array([[[uu, vv]]], dtype=np.float64)
            und = cv2.undistortPoints(pts, i.camera_matrix(), i.dist_coeffs(),
                                      P=i.camera_matrix())
            uu, vv = float(und[0, 0, 0]), float(und[0, 0, 1])
        x = (uu - i.ppx) / i.fx * float(depth_m)
        y = (vv - i.ppy) / i.fy * float(depth_m)
        return Result.ok((x, y, float(depth_m)))

    def point_at(self, u: int, v: int,
                 patch: int = 2) -> Result[Tuple[float, float, float]]:
        d = self.depth_at(u, v, patch)
        if d.is_err:
            return Result.err_msg(d.message)
        return self.deproject(u, v, d.data)

    # ---------------------------------------------------------- intrinsics
    def set_intrinsics_override(self, intr: Optional[CameraIntrinsics]) -> None:
        """Install intrinsics from the library. Takes precedence over factory
        values, so a properly solved calibration can replace them for good."""
        self._override_intrinsics = intr

    def factory_intrinsics(self) -> Result[CameraIntrinsics]:
        return Result.err_msg("This camera reports no intrinsics.")

    def intrinsics(self) -> Result[CameraIntrinsics]:
        if self._override_intrinsics is not None:
            return Result.ok(self._override_intrinsics)
        return self.factory_intrinsics()

    # -------------------------------------------------------- live options
    def set_auto_exposure(self, enabled: bool) -> Result[None]:
        return Result.err_msg("Auto-exposure control not supported.")

    def set_exposure(self, value: int) -> Result[None]:
        return Result.err_msg("Exposure control not supported.")

    def set_gain(self, gain: int) -> Result[None]:
        return Result.err_msg("Gain control not supported.")

    def option_range(self, option_name: str
                     ) -> Result[Tuple[float, float, float, float]]:
        return Result.err_msg("Option ranges not supported.")


def _ideal_from_observed(u: float, v: float,
                         intr: CameraIntrinsics) -> Tuple[float, float]:
    """Local shim so camera_base does not import intrinsics_math at a point
    where it would create a cycle. Same polynomial, single evaluation."""
    x = (u - intr.ppx) / intr.fx
    y = (v - intr.ppy) / intr.fy
    c = list(intr.coeffs[:5]) + [0.0] * max(0, 5 - len(intr.coeffs))
    k1, k2, p1, p2, k3 = c[:5]
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y)
    return (xd * intr.fx + intr.ppx, yd * intr.fy + intr.ppy)
