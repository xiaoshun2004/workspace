# -*- coding: utf-8 -*-
"""
core/models.py
--------------
Shared data models for the Nova-5 eye-in-hand calibration app.

Pure data plus tiny helpers. No Qt, no robot SDK, no pyrealsense2 -- every
other module may depend on this freely (numpy is allowed for transform
payloads).

Result, CameraIntrinsics, CameraConfig, CharucoBoardConfig and HandEyeResult
are carried over unchanged from the Atom-W bundle; the joint/humanoid models
are gone. Everything below the "Nova-5" banner is new.

UNIT POLICY (read this once, then never think about it again)
------------------------------------------------------------
  * Every SE(3) transform in this codebase is METRES and its rotation is a
    plain 3x3 matrix. No exceptions.
  * The Dobot SDK speaks millimetres and (probably) degrees. That conversion
    happens exactly once, at the api/nova5_api.py boundary.
  * RobotPose stores the RAW SDK numbers as reported, and derives the matrix
    on demand through an explicit PoseConvention. That means a wrong
    convention guess can be corrected later without re-capturing a single
    image -- the raw feedback is always recoverable from disk.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Generic, List, Optional, Sequence, Tuple, TypeVar

import numpy as np

T = TypeVar("T")


# ---------------------------------------------------------------------------
# Result monad -- robot / camera I/O returns this instead of raising.
# ---------------------------------------------------------------------------
@dataclass
class Result(Generic[T]):
    """Result monad. Robot/camera I/O returns this instead of raising."""

    is_ok: bool
    data: Optional[T]
    message: str

    @classmethod
    def ok(cls, data: T) -> "Result[T]":
        return cls(True, data, "")

    @classmethod
    def ok_msg(cls, msg: str) -> "Result[T]":
        return cls(True, None, msg)

    @classmethod
    def err_msg(cls, msg: str) -> "Result[T]":
        return cls(False, None, msg)

    @classmethod
    def err(cls, data: T, msg: str) -> "Result[T]":
        return cls(False, data, msg)

    @property
    def is_err(self) -> bool:
        return not self.is_ok


# ---------------------------------------------------------------------------
# JSON helpers
# ---------------------------------------------------------------------------
def float_or_nan(value) -> float:
    """float(), but a JSON null reads back as NaN.

    NaN and Infinity are not valid JSON, so core/project_io.py writes every
    non-finite float as null. Any field whose default is NaN therefore has to
    accept null on the way back in rather than raising on float(None).
    """
    if value is None:
        return float("nan")
    return float(value)


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------
class CameraKind(str, Enum):
    USB = "usb"
    REALSENSE = "realsense"


class StreamMode(str, Enum):
    RGB = "rgb"
    DEPTH = "depth"
    BOTH = "both"


class ConnectionState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ESTOPPED = "estopped"


class CaptureMode(str, Enum):
    """How an instance's samples are being collected."""

    AUTO = "auto"           # replay a project's authored poses
    MANUAL = "manual"       # jog freely, capture when happy
    IMAGES_ONLY = "images"  # no robot; poses typed in later


class IntrinsicsSource(str, Enum):
    FACTORY = "factory"         # read from the RealSense device
    CALIBRATED = "calibrated"   # solved here with cv2.calibrateCamera
    IMPORTED = "imported"       # loaded from a user-supplied file


# ---------------------------------------------------------------------------
# Cameras
# ---------------------------------------------------------------------------
@dataclass
class CameraInfo:
    """A discovered camera device."""

    kind: CameraKind
    ident: str          # USB device index (as str) or RealSense serial number
    label: str          # human readable
    width: int = 0
    height: int = 0
    fps: int = 0

    def safe_name(self) -> str:
        """Filesystem-safe base name for this camera's output files."""
        raw = "{}_{}".format(self.kind.value, self.ident)
        return "".join(c if (c.isalnum() or c in "-_") else "_" for c in raw)


@dataclass
class CameraConfig:
    """Connect-time configuration. width/height/fps cannot change while
    streaming; exposure/gain can be set live."""

    kind: CameraKind = CameraKind.REALSENSE
    ident: Optional[str] = None     # serial (RealSense) or index (USB)
    width: int = 1280
    height: int = 720
    fps: int = 30
    auto_exposure: bool = True
    exposure: Optional[int] = None  # microseconds, only if auto_exposure False
    gain: Optional[int] = None

    def camera_key(self) -> str:
        """Identity used to look up stored intrinsics. Resolution is NOT part
        of the key -- an IntrinsicsRecord carries its own resolution and the
        library holds one record per (key, resolution)."""
        return "{}:{}".format(self.kind.value, self.ident if self.ident else "default")

    def to_dict(self) -> dict:
        return {
            "kind": self.kind.value, "ident": self.ident,
            "width": int(self.width), "height": int(self.height),
            "fps": int(self.fps), "auto_exposure": bool(self.auto_exposure),
            "exposure": self.exposure, "gain": self.gain,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CameraConfig":
        base = cls()
        return cls(
            kind=CameraKind(d.get("kind", base.kind.value)),
            ident=d.get("ident"),
            width=int(d.get("width", base.width)),
            height=int(d.get("height", base.height)),
            fps=int(d.get("fps", base.fps)),
            auto_exposure=bool(d.get("auto_exposure", base.auto_exposure)),
            exposure=d.get("exposure"), gain=d.get("gain"),
        )


@dataclass
class CameraIntrinsics:
    """Pinhole intrinsics for the colour stream (depth is aligned to colour).

    Carried over unchanged from the Atom-W bundle. The distortion-model
    classification below is the single most load-bearing part of this file:
    librealsense's inverse-Brown-Conrady coefficients map observed -> ideal,
    the OPPOSITE of OpenCV's distCoeffs, and handing them straight to
    solvePnP roughly DOUBLES the distortion error instead of removing it.
    """

    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    coeffs: List[float] = field(default_factory=list)
    model: str = ""

    def is_opencv_compatible(self) -> bool:
        """True when self.coeffs can be handed to OpenCV as distCoeffs
        directly. False does NOT mean unusable -- see needs_manual_undistort(),
        which is handled exactly."""
        name = (self.model or "").lower()
        if not name:
            return True                     # unknown: assume, but warn
        if "kannala" in name or "ftheta" in name:
            return False
        if "inverse" in name:
            return not any(abs(c) > 1e-12 for c in self.coeffs)
        return True

    def has_distortion(self) -> bool:
        return any(abs(float(c)) > 1e-12 for c in self.coeffs)

    def needs_manual_undistort(self) -> bool:
        """True for inverse-Brown-Conrady with real coefficients. The fix is
        exact rather than a refit: undistort the detected points with the
        model's own polynomial and give OpenCV zero coefficients."""
        return "inverse" in (self.model or "").lower() and self.has_distortion()

    def unsupported_model(self) -> bool:
        """Fisheye-style models we can neither pass through nor invert with
        the Brown-Conrady polynomial."""
        name = (self.model or "").lower()
        return "kannala" in name or "ftheta" in name

    def zero_dist_coeffs(self) -> np.ndarray:
        return np.zeros((5, 1), dtype=np.float64)

    def to_dict(self) -> dict:
        return {
            "width": int(self.width), "height": int(self.height),
            "fx": float(self.fx), "fy": float(self.fy),
            "ppx": float(self.ppx), "ppy": float(self.ppy),
            "coeffs": [float(c) for c in self.coeffs],
            "model": str(self.model),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CameraIntrinsics":
        return cls(
            width=int(d.get("width", 0)), height=int(d.get("height", 0)),
            fx=float(d.get("fx", 0.0)), fy=float(d.get("fy", 0.0)),
            ppx=float(d.get("ppx", 0.0)), ppy=float(d.get("ppy", 0.0)),
            coeffs=[float(c) for c in d.get("coeffs", [])],
            model=str(d.get("model", "")),
        )

    def resolution(self) -> Tuple[int, int]:
        return (int(self.width), int(self.height))

    def camera_matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.ppx], [0.0, self.fy, self.ppy],
             [0.0, 0.0, 1.0]], dtype=np.float64)

    def dist_coeffs(self) -> np.ndarray:
        c = list(self.coeffs[:5]) + [0.0] * max(0, 5 - len(self.coeffs))
        return np.array(c[:5], dtype=np.float64).reshape(5, 1)

    def hfov_deg(self) -> float:
        """Horizontal field of view, for the remount-margin report."""
        if self.fx <= 0.0:
            return 0.0
        return float(np.degrees(2.0 * np.arctan(0.5 * self.width / self.fx)))


@dataclass
class IntrinsicsRecord:
    """An entry in the reusable intrinsics library.

    Intrinsics are resolution-specific (fx, fy, ppx, ppy all scale), so the
    library is keyed by (camera_key, width, height) and a record is only ever
    offered for a stream at exactly its own resolution.
    """

    camera_key: str
    intrinsics: CameraIntrinsics
    source: IntrinsicsSource = IntrinsicsSource.FACTORY
    rms_px: float = float("nan")    # cv2.calibrateCamera RMS, if solved here
    n_images: int = 0
    timestamp: str = ""
    note: str = ""

    def key(self) -> Tuple[str, int, int]:
        return (self.camera_key, self.intrinsics.width, self.intrinsics.height)

    def to_dict(self) -> dict:
        return {
            "camera_key": self.camera_key,
            "intrinsics": self.intrinsics.to_dict(),
            "source": self.source.value,
            "rms_px": float(self.rms_px),
            "n_images": int(self.n_images),
            "timestamp": self.timestamp,
            "note": self.note,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "IntrinsicsRecord":
        return cls(
            camera_key=str(d.get("camera_key", "")),
            intrinsics=CameraIntrinsics.from_dict(d.get("intrinsics", {})),
            source=IntrinsicsSource(d.get("source", "factory")),
            rms_px=float_or_nan(d.get("rms_px", float("nan"))),
            n_images=int(d.get("n_images", 0)),
            timestamp=str(d.get("timestamp", "")),
            note=str(d.get("note", "")),
        )


# ---------------------------------------------------------------------------
# Calibration target
# ---------------------------------------------------------------------------
@dataclass
class CharucoBoardConfig:
    """Parameters for the ChArUco calibration target. Lengths in metres."""

    squares_x: int = 5
    squares_y: int = 7
    square_length_m: float = 0.030
    marker_length_m: float = 0.022
    dictionary: str = "DICT_4X4_50"

    def to_dict(self) -> dict:
        return {
            "squares_x": self.squares_x,
            "squares_y": self.squares_y,
            "square_length_m": self.square_length_m,
            "marker_length_m": self.marker_length_m,
            "dictionary": self.dictionary,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CharucoBoardConfig":
        base = cls()
        return cls(
            squares_x=int(d.get("squares_x", base.squares_x)),
            squares_y=int(d.get("squares_y", base.squares_y)),
            square_length_m=float(d.get("square_length_m", base.square_length_m)),
            marker_length_m=float(d.get("marker_length_m", base.marker_length_m)),
            dictionary=str(d.get("dictionary", base.dictionary)),
        )

    def matches(self, other: "CharucoBoardConfig") -> bool:
        return self.to_dict() == other.to_dict()


@dataclass
class HandEyeResult:
    """Result of cv2.calibrateHandEye plus validation metrics.

    T_cam2mount is the camera pose expressed in the MOUNT frame, which on the
    Nova-5 is whatever frame the pose feedback reports -- i.e. the taught TCP
    under the active Tool index, NOT necessarily the bare flange. The active
    Tool/User indices are recorded alongside in the instance metadata and are
    part of the calibration's validity conditions.
    """

    T_cam2mount: np.ndarray  # 4x4, metres
    method: str
    num_samples: int
    board_origin_spread_mm: float
    rotation_spread_deg: float
    timestamp: str = ""

    def to_dict(self) -> dict:
        return {
            "T_cam2mount": np.asarray(self.T_cam2mount, dtype=float).tolist(),
            "method": self.method,
            "num_samples": int(self.num_samples),
            "board_origin_spread_mm": float(self.board_origin_spread_mm),
            "rotation_spread_deg": float(self.rotation_spread_deg),
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "HandEyeResult":
        return cls(
            T_cam2mount=np.array(d["T_cam2mount"], dtype=float),
            method=str(d.get("method", "")),
            num_samples=int(d.get("num_samples", 0)),
            board_origin_spread_mm=float_or_nan(
                d.get("board_origin_spread_mm", float("nan"))),
            rotation_spread_deg=float_or_nan(
                d.get("rotation_spread_deg", float("nan"))),
            timestamp=str(d.get("timestamp", "")),
        )


# ===========================================================================
# Nova-5 specifics
# ===========================================================================

_EULER_AXES = {"x": 0, "y": 1, "z": 2}

# The 12 axis sequences that Tait-Bryan / proper-Euler conventions can use.
EULER_ORDERS = (
    "xyz", "xzy", "yxz", "yzx", "zxy", "zyx",
    "xyx", "xzx", "yxy", "yzy", "zxz", "zyz",
)

QUAT_ORDERS = ("wxyz", "xyzw")


def _elementary_rotation(axis: str, angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    if axis == "x":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == "y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    if axis == "z":
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    raise ValueError("axis must be one of x, y, z (got {!r})".format(axis))


def euler_to_matrix(angles_rad: Sequence[float], order: str,
                    intrinsic: bool) -> np.ndarray:
    """Compose three elementary rotations into a rotation matrix.

    `order` names the axis each angle turns about, POSITIONALLY: order[0] is
    the axis for angles[0]. So order="xyz" is the ordinary reading of a
    (rx, ry, rz) triple, and order="zyx" is the hypothesis that the first
    reported number is actually a rotation about z. Both are in the search
    space of core/pose_convention.py because controllers differ.

    Intrinsic means each rotation is about the axes of the frame produced by
    the previous one, so the matrices multiply left to right. Extrinsic means
    all three are about the fixed original axes, so they multiply right to
    left -- extrinsic "xyz" is therefore Rz(rz) @ Ry(ry) @ Rx(rx), the usual
    roll-pitch-yaw.
    """
    order = order.lower()
    if len(order) != 3 or any(ch not in _EULER_AXES for ch in order):
        raise ValueError("bad Euler order {!r}".format(order))
    a1, a2, a3 = (float(v) for v in angles_rad)
    R1 = _elementary_rotation(order[0], a1)
    R2 = _elementary_rotation(order[1], a2)
    R3 = _elementary_rotation(order[2], a3)
    if intrinsic:
        return R1 @ R2 @ R3
    return R3 @ R2 @ R1


def quat_to_matrix(q: Sequence[float], order: str = "wxyz") -> np.ndarray:
    """Quaternion -> rotation matrix. Input is normalised first.

    The Nova-5 real-time feedback packet documents its quaternion fields as
    [qw, qx, qy, qz]; ROS/tf and the app's own display helpers use (x, y, z, w).
    Passing the order explicitly at every call site is deliberate.
    """
    values = [float(v) for v in q]
    if order == "wxyz":
        w, x, y, z = values
    elif order == "xyzw":
        x, y, z, w = values
    else:
        raise ValueError("quat order must be wxyz or xyzw (got {!r})".format(order))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


_NEXT_AXIS = {0: 1, 1: 2, 2: 0}


def _axis_tuple(order: str):
    """(first axis index, parity, repetition) for an EXTRINSIC axis sequence."""
    order = order.lower()
    if len(order) != 3 or any(ch not in _EULER_AXES for ch in order):
        raise ValueError("bad Euler order {!r}".format(order))
    i = _EULER_AXES[order[0]]
    parity = 0 if _EULER_AXES[order[1]] == _NEXT_AXIS[i] else 1
    repetition = 1 if order[0] == order[2] else 0
    return i, parity, repetition


def matrix_to_euler(R, order: str, intrinsic: bool) -> Tuple[float, float, float]:
    """Inverse of euler_to_matrix, in radians.

    Needed to turn a computed transform back into the six numbers the
    controller expects, which is what every corrected replay pose has to go
    through. Uses the standard generic decomposition parameterised by (first
    axis, parity, repetition) so all twelve sequences are handled by one piece
    of code rather than twelve hand-written special cases.

    An intrinsic sequence "abc" is the extrinsic sequence "cba" with the angle
    triple reversed, so only the extrinsic case is implemented.

    Near gimbal lock the decomposition is not unique; the branch taken there
    sets the third angle to zero. Recomposing always reproduces the same
    rotation, which is the property that actually matters.
    """
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    seq = order.lower()[::-1] if intrinsic else order.lower()
    i, parity, repetition = _axis_tuple(seq)
    j = _NEXT_AXIS[i] if parity == 0 else _NEXT_AXIS[_NEXT_AXIS[i]]
    k = _NEXT_AXIS[j] if parity == 0 else _NEXT_AXIS[_NEXT_AXIS[j]]

    eps = 1e-9
    if repetition:
        sy = math.sqrt(R[i, j] * R[i, j] + R[i, k] * R[i, k])
        if sy > eps:
            a1 = math.atan2(R[i, j], R[i, k])
            a2 = math.atan2(sy, R[i, i])
            a3 = math.atan2(R[j, i], -R[k, i])
        else:
            a1 = math.atan2(-R[j, k], R[j, j])
            a2 = math.atan2(sy, R[i, i])
            a3 = 0.0
    else:
        cy = math.sqrt(R[i, i] * R[i, i] + R[j, i] * R[j, i])
        if cy > eps:
            a1 = math.atan2(R[k, j], R[k, k])
            a2 = math.atan2(-R[k, i], cy)
            a3 = math.atan2(R[j, i], R[i, i])
        else:
            a1 = math.atan2(-R[j, k], R[j, j])
            a2 = math.atan2(-R[k, i], cy)
            a3 = 0.0
    if parity:
        a1, a2, a3 = -a1, -a2, -a3
    if intrinsic:
        a1, a3 = a3, a1
    return (a1, a2, a3)


def matrix_to_rotvec(R) -> Tuple[float, float, float]:
    """Rotation matrix -> Rodrigues vector, in radians. Kept here rather than
    reaching for cv2 so models.py stays dependency-light."""
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    cos_theta = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return (0.0, 0.0, 0.0)
    if abs(math.pi - theta) < 1e-6:
        # Near 180 degrees atan2-style extraction degenerates; take the axis
        # from the symmetric part instead.
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        idx = int(np.argmax(axis))
        if axis[idx] > 1e-12:
            axis = A[:, idx] / axis[idx]
        axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
        return tuple(float(v) * theta for v in axis)
    factor = theta / (2.0 * math.sin(theta))
    return (float((R[2, 1] - R[1, 2]) * factor),
            float((R[0, 2] - R[2, 0]) * factor),
            float((R[1, 0] - R[0, 1]) * factor))


@dataclass
class PoseConvention:
    """How to turn the Nova-5's six Cartesian numbers into a rotation.

    This is the single most dangerous unknown in the whole pipeline: getting
    it wrong produces a hand-eye result that looks entirely plausible and is
    silently wrong. Nothing in this codebase guesses -- a convention is either
    solved for against the feedback packet's quaternion (see
    core/pose_convention.py) or explicitly set by the operator, and it is
    written into every project and instance file.

    rotvec=True means (rx, ry, rz) is a Rodrigues rotation vector and the
    order/intrinsic fields are ignored.
    """

    order: str = "xyz"
    intrinsic: bool = False
    degrees: bool = True
    rotvec: bool = False
    translation_scale: float = 0.001    # SDK units -> metres (mm by default)
    quat_order: str = "wxyz"
    verified: bool = False              # set once solve_convention() confirmed it
    note: str = ""

    def rotation(self, rx: float, ry: float, rz: float) -> np.ndarray:
        angles = [float(rx), float(ry), float(rz)]
        if self.degrees:
            angles = [math.radians(a) for a in angles]
        if self.rotvec:
            theta = math.sqrt(sum(a * a for a in angles))
            if theta < 1e-12:
                return np.eye(3)
            axis = np.asarray(angles, dtype=np.float64) / theta
            K = np.array([[0.0, -axis[2], axis[1]],
                          [axis[2], 0.0, -axis[0]],
                          [-axis[1], axis[0], 0.0]])
            return (np.eye(3) + math.sin(theta) * K
                    + (1.0 - math.cos(theta)) * (K @ K))
        return euler_to_matrix(angles, self.order, self.intrinsic)

    def angles_from_rotation(self, R) -> Tuple[float, float, float]:
        """Rotation matrix -> the (rx, ry, rz) triple this controller expects,
        in the controller's own unit. Exact inverse of rotation()."""
        if self.rotvec:
            angles = matrix_to_rotvec(R)
        else:
            angles = matrix_to_euler(R, self.order, self.intrinsic)
        if self.degrees:
            return tuple(math.degrees(a) for a in angles)
        return tuple(float(a) for a in angles)

    def se3_to_command(self, T) -> Tuple[float, float, float,
                                         float, float, float]:
        """A metres-and-matrices transform -> the six numbers to send.

        Every automated move goes through here, which is why the convention is
        pinned down before anything is allowed to move.
        """
        T = np.asarray(T, dtype=np.float64).reshape(4, 4)
        scale = float(self.translation_scale)
        if abs(scale) < 1e-15:
            raise ValueError("translation_scale must be non-zero")
        x, y, z = (T[:3, 3] / scale).tolist()
        rx, ry, rz = self.angles_from_rotation(T[:3, :3])
        return (float(x), float(y), float(z), rx, ry, rz)

    def command_to_se3(self, x: float, y: float, z: float, rx: float,
                       ry: float, rz: float) -> np.ndarray:
        """The six numbers -> a metres-and-matrices transform."""
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self.rotation(rx, ry, rz)
        T[:3, 3] = np.array([x, y, z], dtype=np.float64) * \
            float(self.translation_scale)
        return T

    def label(self) -> str:
        if self.rotvec:
            base = "rotation vector"
        else:
            base = "{} {}".format(
                "intrinsic" if self.intrinsic else "extrinsic",
                self.order.upper())
        return "{}, {}, quat {}".format(
            base, "deg" if self.degrees else "rad", self.quat_order)

    def to_dict(self) -> dict:
        return {
            "order": self.order, "intrinsic": bool(self.intrinsic),
            "degrees": bool(self.degrees), "rotvec": bool(self.rotvec),
            "translation_scale": float(self.translation_scale),
            "quat_order": self.quat_order, "verified": bool(self.verified),
            "note": self.note,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "PoseConvention":
        base = cls()
        return cls(
            order=str(d.get("order", base.order)),
            intrinsic=bool(d.get("intrinsic", base.intrinsic)),
            degrees=bool(d.get("degrees", base.degrees)),
            rotvec=bool(d.get("rotvec", base.rotvec)),
            translation_scale=float(
                d.get("translation_scale", base.translation_scale)),
            quat_order=str(d.get("quat_order", base.quat_order)),
            verified=bool(d.get("verified", base.verified)),
            note=str(d.get("note", base.note)),
        )


@dataclass
class RobotPose:
    """One reading of the arm's state, in RAW SDK UNITS as reported.

    Nothing here is converted. xyz are whatever the controller sent (mm on
    every Dobot controller seen so far) and rx/ry/rz likewise (degrees). The
    matrix is derived on demand via a PoseConvention, so a convention that
    turns out to be wrong can be corrected after the fact without discarding
    a single captured image.

    tool_index / user_index come from bytes 1013 / 1012 of the real-time
    feedback packet, and tool_frame / user_frame from the six-double blocks at
    1248 and 1200. They are recorded because T_cam2mount is only meaningful
    relative to the frame the controller was reporting at capture time: change
    the taught TCP and the calibration silently stops being true.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0
    joints: List[float] = field(default_factory=lambda: [0.0] * 6)
    quat: Optional[List[float]] = None      # as reported, order per convention
    tool_index: int = -1
    user_index: int = -1
    tool_frame: Optional[List[float]] = None
    user_frame: Optional[List[float]] = None
    timestamp: float = 0.0                  # host time.monotonic() at read
    source: str = ""                        # "feedback" | "getpose" | "manual"

    def xyz(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=np.float64)

    def rpy(self) -> np.ndarray:
        return np.array([self.rx, self.ry, self.rz], dtype=np.float64)

    def to_se3(self, convention: PoseConvention,
               prefer_quaternion: bool = True) -> np.ndarray:
        """T_mount2base, in METRES.

        When the packet carried a quaternion and prefer_quaternion is set, the
        rotation comes from the quaternion -- it has no convention ambiguity
        beyond component order, which is a single bit that the convention
        solver pins down. The Euler triple is then only used as a cross-check.
        """
        if prefer_quaternion and self.quat is not None and len(self.quat) == 4:
            R = quat_to_matrix(self.quat, convention.quat_order)
        else:
            R = convention.rotation(self.rx, self.ry, self.rz)
        Tm = np.eye(4, dtype=np.float64)
        Tm[:3, :3] = R
        Tm[:3, 3] = self.xyz() * float(convention.translation_scale)
        return Tm

    def rotation_disagreement_deg(self, convention: PoseConvention) -> float:
        """Geodesic angle between the quaternion rotation and the Euler-triple
        rotation. Should be ~0. A nonzero value means the convention is wrong,
        or the quaternion refers to a different frame than the Cartesian
        vector -- either way, stop and investigate before capturing."""
        if self.quat is None or len(self.quat) != 4:
            return float("nan")
        R_q = quat_to_matrix(self.quat, convention.quat_order)
        R_e = convention.rotation(self.rx, self.ry, self.rz)
        R_err = R_q.T @ R_e
        cos_theta = float(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))
        return float(np.degrees(np.arccos(cos_theta)))

    def to_dict(self) -> dict:
        return {
            "x": float(self.x), "y": float(self.y), "z": float(self.z),
            "rx": float(self.rx), "ry": float(self.ry), "rz": float(self.rz),
            "joints": [float(j) for j in self.joints],
            "quat": None if self.quat is None else [float(q) for q in self.quat],
            "tool_index": int(self.tool_index),
            "user_index": int(self.user_index),
            "tool_frame": (None if self.tool_frame is None
                           else [float(v) for v in self.tool_frame]),
            "user_frame": (None if self.user_frame is None
                           else [float(v) for v in self.user_frame]),
            "timestamp": float(self.timestamp),
            "source": str(self.source),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "RobotPose":
        return cls(
            x=float(d.get("x", 0.0)), y=float(d.get("y", 0.0)),
            z=float(d.get("z", 0.0)), rx=float(d.get("rx", 0.0)),
            ry=float(d.get("ry", 0.0)), rz=float(d.get("rz", 0.0)),
            joints=[float(j) for j in d.get("joints", [0.0] * 6)],
            quat=(None if d.get("quat") is None
                  else [float(q) for q in d["quat"]]),
            tool_index=int(d.get("tool_index", -1)),
            user_index=int(d.get("user_index", -1)),
            tool_frame=(None if d.get("tool_frame") is None
                        else [float(v) for v in d["tool_frame"]]),
            user_frame=(None if d.get("user_frame") is None
                        else [float(v) for v in d["user_frame"]]),
            timestamp=float(d.get("timestamp", 0.0)),
            source=str(d.get("source", "")),
        )


@dataclass
class FrameIdentity:
    """The Tool/User configuration a calibration is valid for.

    A hand-eye result answers "where is the camera relative to the frame the
    controller reports". Change the taught TCP, or switch User index, and that
    answer is wrong even though nothing physical moved. Every project and
    instance records this and refuses to mix.
    """

    tool_index: int = -1
    user_index: int = -1
    tool_frame: List[float] = field(default_factory=list)
    user_frame: List[float] = field(default_factory=list)

    @classmethod
    def from_pose(cls, pose: RobotPose) -> "FrameIdentity":
        return cls(
            tool_index=int(pose.tool_index), user_index=int(pose.user_index),
            tool_frame=list(pose.tool_frame or []),
            user_frame=list(pose.user_frame or []),
        )

    def compare(self, other: "FrameIdentity",
                tol_mm: float = 0.5, tol_deg: float = 0.1) -> Result[None]:
        """Explain the difference rather than just reporting one."""
        if self.tool_index != other.tool_index:
            return Result.err_msg(
                "Tool index changed: calibrated with Tool={}, live Tool={}. "
                "The calibration describes the camera relative to the taught "
                "TCP, so it does not transfer.".format(
                    self.tool_index, other.tool_index))
        if self.user_index != other.user_index:
            return Result.err_msg(
                "User index changed: calibrated with User={}, live User={}. "
                "Poses are expressed in the user frame, so the stored "
                "transforms refer to a different origin.".format(
                    self.user_index, other.user_index))
        for label, mine, theirs in (("Tool", self.tool_frame, other.tool_frame),
                                    ("User", self.user_frame, other.user_frame)):
            if not mine or not theirs or len(mine) != 6 or len(theirs) != 6:
                continue
            d_mm = max(abs(a - b) for a, b in zip(mine[:3], theirs[:3]))
            d_deg = max(abs(a - b) for a, b in zip(mine[3:], theirs[3:]))
            if d_mm > tol_mm or d_deg > tol_deg:
                return Result.err_msg(
                    "{} frame values were re-taught since calibration "
                    "(max {:.2f} mm / {:.2f} deg difference). Re-calibrate or "
                    "restore the previous {} frame.".format(
                        label, d_mm, d_deg, label))
        return Result.ok(None)

    def to_dict(self) -> dict:
        return {
            "tool_index": int(self.tool_index),
            "user_index": int(self.user_index),
            "tool_frame": [float(v) for v in self.tool_frame],
            "user_frame": [float(v) for v in self.user_frame],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "FrameIdentity":
        return cls(
            tool_index=int(d.get("tool_index", -1)),
            user_index=int(d.get("user_index", -1)),
            tool_frame=[float(v) for v in d.get("tool_frame", [])],
            user_frame=[float(v) for v in d.get("user_frame", [])],
        )


@dataclass
class SafetyConfig:
    """Every gate is individually defeatable and every one defaults to on.

    speed_factor is the controller-wide SpeedFactor percentage applied before
    any automated motion; the per-command SpeedL/AccL caps ride on top of it.
    """

    enabled: bool = True                    # master switch for the gates below
    speed_factor_pct: int = 10              # SpeedFactor() during calibration
    speed_l_pct: int = 10
    acc_l_pct: int = 10
    speed_j_pct: int = 10
    acc_j_pct: int = 10
    confirm_each_pose: bool = True          # first automated run of an instance
    check_workspace_box: bool = True
    check_joint_limits: bool = True
    require_enabled_before_move: bool = True
    max_step_mm: float = 150.0              # refuse a single move longer than this
    max_step_deg: float = 45.0
    workspace_min_mm: List[float] = field(
        default_factory=lambda: [-700.0, -700.0, -200.0])
    workspace_max_mm: List[float] = field(
        default_factory=lambda: [700.0, 700.0, 900.0])
    joint_min_deg: List[float] = field(
        default_factory=lambda: [-360.0, -135.0, -154.0, -360.0, -180.0, -360.0])
    joint_max_deg: List[float] = field(
        default_factory=lambda: [360.0, 135.0, 154.0, 360.0, 180.0, 360.0])
    # EnableRobot payload. Zeroed by default; set once a camera bracket's mass
    # is known, because the controller's collision detection depends on it.
    payload_kg: float = 0.0
    payload_center_mm: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0])
    collision_level: int = 3                # Dobot SetCollisionLevel, 0 = off

    def to_dict(self) -> dict:
        return {
            "enabled": bool(self.enabled),
            "speed_factor_pct": int(self.speed_factor_pct),
            "speed_l_pct": int(self.speed_l_pct),
            "acc_l_pct": int(self.acc_l_pct),
            "speed_j_pct": int(self.speed_j_pct),
            "acc_j_pct": int(self.acc_j_pct),
            "confirm_each_pose": bool(self.confirm_each_pose),
            "check_workspace_box": bool(self.check_workspace_box),
            "check_joint_limits": bool(self.check_joint_limits),
            "require_enabled_before_move": bool(self.require_enabled_before_move),
            "max_step_mm": float(self.max_step_mm),
            "max_step_deg": float(self.max_step_deg),
            "workspace_min_mm": [float(v) for v in self.workspace_min_mm],
            "workspace_max_mm": [float(v) for v in self.workspace_max_mm],
            "joint_min_deg": [float(v) for v in self.joint_min_deg],
            "joint_max_deg": [float(v) for v in self.joint_max_deg],
            "payload_kg": float(self.payload_kg),
            "payload_center_mm": [float(v) for v in self.payload_center_mm],
            "collision_level": int(self.collision_level),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "SafetyConfig":
        base = cls()
        out = cls()
        for key in base.to_dict():
            if key in d:
                setattr(out, key, d[key])
        return out


@dataclass
class CalibPose:
    """One authored pose of an auto-calibration project.

    T_mount2base and T_board2cam_ref are the reference view captured when the
    project was authored. ref_corners / ref_ids are the detected ChArUco
    corners in pixels: ghosting them over the live image is what lets an
    operator re-teach all six degrees of freedom by eye instead of merely
    centring the board.
    """

    index: int = 0
    name: str = ""
    is_home: bool = False
    pose: Optional[RobotPose] = None
    T_mount2base: Optional[np.ndarray] = None
    T_board2cam_ref: Optional[np.ndarray] = None
    ref_corners: Optional[np.ndarray] = None    # (N, 2) float32 pixels
    ref_ids: Optional[np.ndarray] = None        # (N, 1) int32
    n_corners: int = 0
    reproj_error: float = float("nan")
    board_distance_m: float = float("nan")
    margin_deg: float = float("nan")            # remount tolerance, see remount.py
    image_path: str = ""
    overlay_path: str = ""

    def to_dict(self) -> dict:
        return {
            "index": int(self.index),
            "name": self.name,
            "is_home": bool(self.is_home),
            "pose": None if self.pose is None else self.pose.to_dict(),
            "T_mount2base": (None if self.T_mount2base is None
                             else np.asarray(self.T_mount2base, float).tolist()),
            "T_board2cam_ref": (None if self.T_board2cam_ref is None
                                else np.asarray(self.T_board2cam_ref, float).tolist()),
            "ref_corners": (None if self.ref_corners is None
                            else np.asarray(self.ref_corners, float).reshape(-1, 2).tolist()),
            "ref_ids": (None if self.ref_ids is None
                        else np.asarray(self.ref_ids, int).reshape(-1).tolist()),
            "n_corners": int(self.n_corners),
            "reproj_error": float(self.reproj_error),
            "board_distance_m": float(self.board_distance_m),
            "margin_deg": float(self.margin_deg),
            "image_path": self.image_path,
            "overlay_path": self.overlay_path,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CalibPose":
        def _mat(key):
            v = d.get(key)
            return None if v is None else np.array(v, dtype=np.float64)
        corners = d.get("ref_corners")
        ids = d.get("ref_ids")
        return cls(
            index=int(d.get("index", 0)),
            name=str(d.get("name", "")),
            is_home=bool(d.get("is_home", False)),
            pose=(None if d.get("pose") is None
                  else RobotPose.from_dict(d["pose"])),
            T_mount2base=_mat("T_mount2base"),
            T_board2cam_ref=_mat("T_board2cam_ref"),
            ref_corners=(None if corners is None
                         else np.array(corners, dtype=np.float32).reshape(-1, 1, 2)),
            ref_ids=(None if ids is None
                     else np.array(ids, dtype=np.int32).reshape(-1, 1)),
            n_corners=int(d.get("n_corners", 0)),
            reproj_error=float_or_nan(d.get("reproj_error", float("nan"))),
            board_distance_m=float_or_nan(
                d.get("board_distance_m", float("nan"))),
            margin_deg=float_or_nan(d.get("margin_deg", float("nan"))),
            image_path=str(d.get("image_path", "")),
            overlay_path=str(d.get("overlay_path", "")),
        )


@dataclass
class CalibProject:
    """A reusable auto-calibration procedure: one home pose plus N further
    poses, authored once and replayed on every subsequent instance.

    The distance band is a soft gate: too tight and every remount fails the
    project, too loose and far-away poses give badly conditioned board poses.
    Defaults suit a D405 (short range module, roughly 7 cm minimum).
    """

    name: str = ""
    created: str = ""
    modified: str = ""
    board: CharucoBoardConfig = field(default_factory=CharucoBoardConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    convention: PoseConvention = field(default_factory=PoseConvention)
    frames: FrameIdentity = field(default_factory=FrameIdentity)
    poses: List[CalibPose] = field(default_factory=list)
    min_distance_m: float = 0.12
    max_distance_m: float = 0.45
    handeye_method: str = "TSAI"
    note: str = ""

    def home(self) -> Optional[CalibPose]:
        for p in self.poses:
            if p.is_home:
                return p
        return self.poses[0] if self.poses else None

    def renumber(self) -> None:
        for i, p in enumerate(self.poses):
            p.index = i

    def to_dict(self) -> dict:
        return {
            "name": self.name, "created": self.created,
            "modified": self.modified,
            "board": self.board.to_dict(),
            "camera": self.camera.to_dict(),
            "convention": self.convention.to_dict(),
            "frames": self.frames.to_dict(),
            "poses": [p.to_dict() for p in self.poses],
            "min_distance_m": float(self.min_distance_m),
            "max_distance_m": float(self.max_distance_m),
            "handeye_method": self.handeye_method,
            "note": self.note,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CalibProject":
        base = cls()
        return cls(
            name=str(d.get("name", "")),
            created=str(d.get("created", "")),
            modified=str(d.get("modified", "")),
            board=CharucoBoardConfig.from_dict(d.get("board", {})),
            camera=CameraConfig.from_dict(d.get("camera", {})),
            convention=PoseConvention.from_dict(d.get("convention", {})),
            frames=FrameIdentity.from_dict(d.get("frames", {})),
            poses=[CalibPose.from_dict(p) for p in d.get("poses", [])],
            min_distance_m=float(d.get("min_distance_m", base.min_distance_m)),
            max_distance_m=float(d.get("max_distance_m", base.max_distance_m)),
            handeye_method=str(d.get("handeye_method", base.handeye_method)),
            note=str(d.get("note", "")),
        )


@dataclass
class RemountRecord:
    """The correction applied when replaying a project onto a moved camera.

    delta is a flange-frame RIGHT multiplication: corrected = T_mount2base @ delta.
    See core/remount.py for why that form and not a relative-to-home replay.

    per_pose_residuals holds one [mm, deg] pair per anchor: how well the live
    board view matched that pose's stored reference view once the correction
    was applied. NaN means the anchor was never checked against the board, so
    its delta is arm-side arithmetic only -- which is the difference between
    a measured remount and a repeatable one. Old files carry no such block.
    """

    delta: Optional[np.ndarray] = None          # 4x4, metres
    anchor_indices: List[int] = field(default_factory=list)
    spread_mm: float = float("nan")
    spread_deg: float = float("nan")
    per_pose_deltas: List[List[List[float]]] = field(default_factory=list)
    per_pose_residuals: List[List[float]] = field(default_factory=list)
    verdict: str = ""                           # human-readable diagnosis

    def to_dict(self) -> dict:
        return {
            "delta": (None if self.delta is None
                      else np.asarray(self.delta, float).tolist()),
            "anchor_indices": [int(i) for i in self.anchor_indices],
            "spread_mm": float(self.spread_mm),
            "spread_deg": float(self.spread_deg),
            "per_pose_deltas": self.per_pose_deltas,
            "per_pose_residuals": [[float(v) for v in pair]
                                   for pair in self.per_pose_residuals],
            "verdict": self.verdict,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "RemountRecord":
        delta = d.get("delta")
        return cls(
            delta=None if delta is None else np.array(delta, dtype=np.float64),
            anchor_indices=[int(i) for i in d.get("anchor_indices", [])],
            spread_mm=float_or_nan(d.get("spread_mm", float("nan"))),
            spread_deg=float_or_nan(d.get("spread_deg", float("nan"))),
            per_pose_deltas=d.get("per_pose_deltas", []),
            per_pose_residuals=[[float_or_nan(v) for v in pair]
                                for pair in d.get("per_pose_residuals", [])],
            verdict=str(d.get("verdict", "")),
        )


@dataclass
class CalibInstance:
    """One run of a calibration: its samples, its solve, and everything needed
    to decide later whether it is still valid.

    Instances are the unit that gets saved, listed and compared. Two instances
    of the same project differ by whatever changed between them -- a nudged
    working distance, a re-clocked bracket, a different resolution.

    derived_from names the instance folder this one was re-solved from, empty
    when the samples were captured fresh. Re-solving an existing instance
    writes a NEW folder by default, so this is the only link back to the run
    the samples actually came from.
    """

    name: str = ""
    project_name: str = ""
    created: str = ""
    mode: CaptureMode = CaptureMode.AUTO
    board: CharucoBoardConfig = field(default_factory=CharucoBoardConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    intrinsics: Optional[CameraIntrinsics] = None
    intrinsics_source: IntrinsicsSource = IntrinsicsSource.FACTORY
    convention: PoseConvention = field(default_factory=PoseConvention)
    frames: FrameIdentity = field(default_factory=FrameIdentity)
    remount: RemountRecord = field(default_factory=RemountRecord)
    result: Optional[HandEyeResult] = None
    note: str = ""
    derived_from: str = ""      # source instance folder when re-solved

    def metadata_dict(self) -> dict:
        return {
            "name": self.name,
            "project_name": self.project_name,
            "created": self.created,
            "mode": self.mode.value,
            "board": self.board.to_dict(),
            "camera": self.camera.to_dict(),
            "intrinsics": (None if self.intrinsics is None
                           else self.intrinsics.to_dict()),
            "intrinsics_source": self.intrinsics_source.value,
            "convention": self.convention.to_dict(),
            "frames": self.frames.to_dict(),
            "remount": self.remount.to_dict(),
            "result": None if self.result is None else self.result.to_dict(),
            "note": self.note,
            "derived_from": self.derived_from,
        }

    @classmethod
    def from_metadata(cls, d: dict) -> "CalibInstance":
        intr = d.get("intrinsics")
        res = d.get("result")
        return cls(
            name=str(d.get("name", "")),
            project_name=str(d.get("project_name", "")),
            created=str(d.get("created", "")),
            mode=CaptureMode(d.get("mode", "auto")),
            board=CharucoBoardConfig.from_dict(d.get("board", {})),
            camera=CameraConfig.from_dict(d.get("camera", {})),
            intrinsics=None if intr is None else CameraIntrinsics.from_dict(intr),
            intrinsics_source=IntrinsicsSource(
                d.get("intrinsics_source", "factory")),
            convention=PoseConvention.from_dict(d.get("convention", {})),
            frames=FrameIdentity.from_dict(d.get("frames", {})),
            remount=RemountRecord.from_dict(d.get("remount", {})),
            result=None if res is None else HandEyeResult.from_dict(res),
            note=str(d.get("note", "")),
            derived_from=str(d.get("derived_from", "")),
        )
