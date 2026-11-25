# -*- coding: utf-8 -*-
"""
ui/settings.py
--------------
Persistent application settings.

One flat AppSettings dataclass holds every configurable parameter; each panel
reads and writes its own fields. Stored as JSON under
~/.config/nova5_calib/settings.json, loaded at startup, and saved immediately
whenever a control changes, so every knob survives a restart.

Nested structures (the pose convention, the safety configuration, the board,
the camera) are stored as plain dicts and hydrated through their own
from_dict()/to_dict(), so adding a field to one of those dataclasses does not
need a settings migration -- an old file simply falls back to the default for
whatever it lacks.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field

from models import (
    CameraConfig,
    CharucoBoardConfig,
    PoseConvention,
    SafetyConfig,
)

DEFAULT_ROOT = os.path.join(os.path.expanduser("~"), ".config", "nova5_calib")
_SETTINGS_PATH = os.path.join(DEFAULT_ROOT, "settings.json")


@dataclass
class AppSettings:
    # ------------------------------------------------------------- storage
    root_dir: str = DEFAULT_ROOT
    log_file_path: str = os.path.join(DEFAULT_ROOT, "app.log")
    last_project: str = ""
    last_instance_dir: str = ""

    # --------------------------------------------------------------- robot
    robot_ip: str = "192.168.5.1"
    robot_auto_connect: bool = False
    convention: dict = field(default_factory=lambda: PoseConvention().to_dict())
    safety: dict = field(default_factory=lambda: SafetyConfig().to_dict())
    # Jog step sizes for the on-screen jog pad.
    jog_linear_mm: float = 5.0
    jog_angular_deg: float = 2.0
    jog_continuous: bool = False
    # Tool/User indices used for every command. Recorded with the calibration
    # because T_cam2mount is only meaningful relative to the reported frame.
    tool_index: int = 0
    user_index: int = 0

    # -------------------------------------------------------------- camera
    camera: dict = field(default_factory=lambda: CameraConfig().to_dict())
    usb_probe_max_index: int = 6
    prefer_library_intrinsics: bool = False
    show_depth_view: bool = False

    # --------------------------------------------------------------- board
    board: dict = field(default_factory=lambda: CharucoBoardConfig().to_dict())

    # --------------------------------------------------------- calibration
    handeye_method: str = "TSAI"
    min_distance_m: float = 0.12
    max_distance_m: float = 0.45
    # Capture gates.
    max_reproj_px: float = 1.0
    max_frame_age_s: float = 0.30
    settle_s: float = 0.40
    require_board_static: bool = True
    board_motion_px: float = 1.5
    # Fraction of the image area the detected board must span before a pose
    # is estimated. Editable on the Calibrate tab: the right floor depends on
    # the rig's working distance, and 2% is only a sane default for close-in
    # D405 work.
    min_image_coverage: float = 0.02
    # Where the image-import dialog last looked.
    last_import_dir: str = ""
    # Auto run behaviour.
    confirm_home_centring: bool = True
    refine_each_pose: bool = True
    skip_failed_poses: bool = True
    move_timeout_s: float = 30.0

    # ------------------------------------------------------------- verify
    standoff_m: float = 0.10
    depth_patch_px: int = 2
    manual_depth_m: float = 0.25
    verify_confirm_move: bool = True

    # ------------------------------------------------------------ display
    angle_unit: str = "deg"          # "rad" | "deg" (display/entry only)
    orientation_format: str = "rpy"  # "rpy" | "quat" (display only)
    window_width: int = 1500
    window_height: int = 950

    # ----------------------------------------------------------- accessors
    def pose_convention(self) -> PoseConvention:
        return PoseConvention.from_dict(self.convention)

    def set_pose_convention(self, convention: PoseConvention) -> None:
        self.convention = convention.to_dict()

    def safety_config(self) -> SafetyConfig:
        return SafetyConfig.from_dict(self.safety)

    def set_safety_config(self, safety: SafetyConfig) -> None:
        self.safety = safety.to_dict()

    def board_config(self) -> CharucoBoardConfig:
        return CharucoBoardConfig.from_dict(self.board)

    def set_board_config(self, board: CharucoBoardConfig) -> None:
        self.board = board.to_dict()

    def camera_config(self) -> CameraConfig:
        return CameraConfig.from_dict(self.camera)

    def set_camera_config(self, camera: CameraConfig) -> None:
        self.camera = camera.to_dict()


def _reject_constant(name: str) -> float:
    """Refuse NaN/Infinity while parsing.

    json.load accepts those tokens by default, and a non-finite number in a
    distance or a joint limit is exactly the kind of value that produces a
    silently wrong gate later. Raising here routes the file to the defaults.
    """
    raise ValueError("settings.json contains {}".format(name))


def _has_non_finite(value) -> bool:
    return (isinstance(value, float) and not math.isfinite(value)) or \
        (isinstance(value, dict)
         and any(_has_non_finite(v) for v in value.values())) or \
        (isinstance(value, (list, tuple))
         and any(_has_non_finite(v) for v in value))


def _finite_only(data: dict) -> dict:
    """Drop any field holding a NaN or an Infinity.

    json.dump would write those as bare NaN/Infinity tokens, which are not
    valid JSON and which load() now refuses -- one such value would cost the
    operator every other setting in the file. A dropped key falls back to its
    default instead, the same rule an older settings file already follows.
    """
    return {key: value for key, value in data.items()
            if not _has_non_finite(value)}


class SettingsStore:
    """Load and atomically save AppSettings as JSON."""

    def __init__(self, path: str = _SETTINGS_PATH) -> None:
        self._path = path

    @property
    def path(self) -> str:
        return self._path

    def load(self) -> AppSettings:
        if not os.path.exists(self._path):
            return AppSettings()
        try:
            with open(self._path, "r", encoding="utf-8") as f:
                data = json.load(f, parse_constant=_reject_constant)
        except (ValueError, OSError):
            # A corrupt settings file must not stop the app from starting;
            # defaults are always a usable state. ValueError covers both
            # json.JSONDecodeError and the UnicodeDecodeError a file with a
            # non-UTF-8 byte raises -- neither is an OSError, and letting
            # either escape kills the app before there is a window to say so.
            return AppSettings()
        if not isinstance(data, dict):
            return AppSettings()
        settings = AppSettings()
        for key, value in data.items():
            if hasattr(settings, key):
                setattr(settings, key, value)
        return settings

    def save(self, settings: AppSettings) -> None:
        os.makedirs(os.path.dirname(self._path) or ".", exist_ok=True)
        tmp = self._path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(_finite_only(asdict(settings)), f, indent=2,
                      allow_nan=False)
        os.replace(tmp, self._path)


def load_settings(path: str = _SETTINGS_PATH) -> AppSettings:
    return SettingsStore(path).load()
