"""App-wide angle unit and orientation format preference.

Two independent choices:

  * ANGLE UNIT -- radians or degrees. Applies to joint angles and to
    roll/pitch/yaw. Internally EVERYTHING stays in radians; this is a
    display/entry convention only, converted at the widget boundary. That
    rule matters: this project has already been bitten once by degrees
    reaching a radian API (the MovL execute bug), so no converted value is
    ever stored or sent.

  * ORIENTATION FORMAT -- roll/pitch/yaw or quaternion. Quaternions are
    display-only; the RPY spin boxes remain the entry path, because a
    freely typed 4-tuple is not generally a unit quaternion and silently
    normalising a user's input is worse than not offering it.

A single AngleFormat instance is shared by every tab and emits `changed`
so open windows re-render immediately instead of waiting for a refresh.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np
from PySide6.QtCore import QObject, Signal

RAD = "rad"
DEG = "deg"
RPY = "rpy"
QUAT = "quat"

ANGLE_UNITS = (RAD, DEG)
ORIENTATION_FORMATS = (RPY, QUAT)

_DEG_SUFFIX = "\u00b0"


class AngleFormat(QObject):
    """Shared preference object. One instance lives on MainWindow and is
    handed to the tabs; each connects to `changed` and re-renders."""

    changed = Signal()

    def __init__(self, unit: str = RAD, orientation: str = RPY,
                 parent=None) -> None:
        super().__init__(parent)
        self._unit = unit if unit in ANGLE_UNITS else RAD
        self._orientation = (orientation if orientation
                             in ORIENTATION_FORMATS else RPY)

    # ------------------------------------------------------------- access
    @property
    def unit(self) -> str:
        return self._unit

    @property
    def orientation(self) -> str:
        return self._orientation

    @property
    def is_deg(self) -> bool:
        return self._unit == DEG

    @property
    def is_quat(self) -> bool:
        return self._orientation == QUAT

    def set_unit(self, unit: str) -> None:
        if unit in ANGLE_UNITS and unit != self._unit:
            self._unit = unit
            self.changed.emit()

    def set_orientation(self, fmt: str) -> None:
        if fmt in ORIENTATION_FORMATS and fmt != self._orientation:
            self._orientation = fmt
            self.changed.emit()

    # ---------------------------------------------------------- conversion
    def to_display(self, radians: float) -> float:
        """Radians -> the displayed unit."""
        value = float(radians)
        return math.degrees(value) if self.is_deg else value

    def from_display(self, value: float) -> float:
        """Displayed unit -> radians. Every value leaving a widget must go
        through this before it touches anything that commands the robot."""
        value = float(value)
        return math.radians(value) if self.is_deg else value

    def suffix(self) -> str:
        return _DEG_SUFFIX if self.is_deg else " rad"

    def decimals(self) -> int:
        """Degrees need fewer decimals than radians for the same
        resolution (1 deg ~ 0.0175 rad)."""
        return 2 if self.is_deg else 4

    def step(self) -> float:
        return 1.0 if self.is_deg else 0.02

    def format_angle(self, radians, width: int = 0) -> str:
        if radians is None:
            return "-"
        value = self.to_display(float(radians))
        text = "{:+.{dp}f}{}".format(value, self.suffix(),
                                     dp=self.decimals())
        return text.rjust(width) if width else text

    # -------------------------------------------------------- orientation
    def orientation_headers(self) -> List[str]:
        if self.is_quat:
            return ["qx", "qy", "qz", "qw"]
        unit = " (deg)" if self.is_deg else " (rad)"
        return ["Roll" + unit, "Pitch" + unit, "Yaw" + unit]

    def orientation_values(self, R) -> List[float]:
        """Rotation matrix -> the values matching orientation_headers()."""
        if self.is_quat:
            return list(matrix_to_quat(R))
        roll, pitch, yaw = matrix_to_rpy_rad(R)
        return [self.to_display(roll), self.to_display(pitch),
                self.to_display(yaw)]

    def format_orientation(self, R) -> str:
        """One-line orientation string for labels and logs."""
        values = self.orientation_values(R)
        if self.is_quat:
            return ("qx={:+.5f} qy={:+.5f} qz={:+.5f} qw={:+.5f}"
                    .format(*values))
        dp = self.decimals()
        return ("r={:+.{dp}f}{u} p={:+.{dp}f}{u} y={:+.{dp}f}{u}"
                .format(*values, dp=dp, u=self.suffix()))

    # ------------------------------------------------------------ settings
    def load_from(self, settings) -> None:
        self._unit = getattr(settings, "angle_unit", RAD)
        if self._unit not in ANGLE_UNITS:
            self._unit = RAD
        self._orientation = getattr(settings, "orientation_format", RPY)
        if self._orientation not in ORIENTATION_FORMATS:
            self._orientation = RPY
        self.changed.emit()

    def write_to(self, settings) -> None:
        settings.angle_unit = self._unit
        settings.orientation_format = self._orientation


# ---------------------------------------------------------------------------
# Rotation conversions (radians; tf/URDF convention R = Rz @ Ry @ Rx)
# ---------------------------------------------------------------------------
def matrix_to_rpy_rad(R) -> Tuple[float, float, float]:
    R = np.asarray(R, dtype=float)
    sp = -float(np.clip(R[2, 0], -1.0, 1.0))
    pitch = math.asin(sp)
    if abs(sp) > 1.0 - 1e-9:                     # gimbal lock
        return (0.0, pitch, math.atan2(-R[0, 1], R[1, 1]))
    return (math.atan2(R[2, 1], R[2, 2]), pitch,
            math.atan2(R[1, 0], R[0, 0]))


def matrix_to_quat(R) -> Tuple[float, float, float, float]:
    """Rotation matrix -> (x, y, z, w), the ROS/tf ordering.

    Uses the largest-diagonal branch (Shepperd's method) rather than the
    naive trace formula, which loses precision when the trace approaches
    -1. The sign is normalised so w >= 0: q and -q are the same rotation,
    and pinning the sign keeps the displayed numbers stable instead of
    flipping as the robot moves through a boundary.
    """
    R = np.asarray(R, dtype=float)
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm > 1e-12:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (x, y, z, w)


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    """(x, y, z, w) -> rotation matrix. Input is normalised first."""
    x, y, z, w = (float(v) for v in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)
