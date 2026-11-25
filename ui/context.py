# -*- coding: utf-8 -*-
"""
ui/context.py
-------------
The state every tab shares: settings, the log, the connected camera and arm,
the board and its detector, and the intrinsics library.

Passed to each tab rather than reached for globally, so a tab's dependencies
are visible in its constructor. The board and detector are rebuilt whenever
the board configuration changes, because a CharucoDetector is bound to the
board it was made from and silently detects nothing useful against a
different one.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QObject, Signal

import handeye
import project_io
from angle_format import AngleFormat
from app_log import AppLog
from camera_base import CameraBase
from models import CalibProject, CameraIntrinsics, Result
from nova5_api import Nova5Api
from settings import AppSettings, SettingsStore


class AppContext(QObject):
    """Shared, observable application state."""

    camera_changed = Signal()       # connected, disconnected or reconfigured
    robot_changed = Signal()
    board_changed = Signal()
    project_changed = Signal()

    def __init__(self, settings: AppSettings, store: SettingsStore,
                 log: AppLog, parent=None) -> None:
        super().__init__(parent)
        self.settings = settings
        self.store = store
        self.log = log
        self.angle_format = AngleFormat(unit=settings.angle_unit,
                                        orientation=settings.orientation_format)
        self.camera: Optional[CameraBase] = None
        self.api: Optional[Nova5Api] = None
        self.project: Optional[CalibProject] = None
        self.library = project_io.IntrinsicsLibrary(
            project_io.intrinsics_library_path(settings.root_dir))
        self._board = None
        self._detector = None
        built = self.rebuild_board()
        if built.is_err:
            # Nothing else checks the board at startup, and a None detector
            # raises from the live tick of every tab. The message rebuild_board
            # took the trouble to compose has to be visible somewhere.
            self.log.error(built.message)

    # ------------------------------------------------------------ settings
    def save_settings(self) -> None:
        """Auto-save. Called by every control that changes a value, so a knob
        survives a restart without anyone having to remember to save."""
        self.settings.angle_unit = self.angle_format.unit
        self.settings.orientation_format = self.angle_format.orientation
        self.store.save(self.settings)

    # --------------------------------------------------------------- board
    def rebuild_board(self) -> Result[None]:
        try:
            self._board, self._detector = handeye.make_board(
                self.settings.board_config())
        except AttributeError as exc:
            # A bad dictionary name is a typo in the settings, and failing
            # loudly here beats detecting nothing later and blaming the light.
            return Result.err_msg(
                "Board dictionary '{}' is not a cv2.aruco predefined "
                "dictionary: {}".format(
                    self.settings.board_config().dictionary, exc))
        self.board_changed.emit()
        return Result.ok(None)

    @property
    def board(self):
        return self._board

    @property
    def detector(self):
        return self._detector

    # -------------------------------------------------------------- camera
    def camera_connected(self) -> bool:
        return self.camera is not None and self.camera.is_connected

    def intrinsics(self) -> Result[CameraIntrinsics]:
        """Whatever intrinsics are in force for the camera actually streaming.

        A connected camera answers for itself. Its override already carries
        the library record the camera tab installed for the device and mode
        that were really opened, whereas the settings config follows the combo
        boxes and can name a camera or a resolution that is not live -- a
        library lookup keyed off it would hand back intrinsics for a different
        sensor and nothing downstream could tell. The config-keyed lookup is
        the fallback for when nothing is streaming at all.
        """
        if self.camera_connected():
            found = self.camera.intrinsics()
            if found.is_ok:
                return found
            return Result.err_msg(
                "{} Run an intrinsic calibration for this camera and "
                "resolution first.".format(found.message))
        config = self.settings.camera_config()
        record = self.library.find(config.camera_key(), config.width,
                                   config.height)
        if record is not None:
            return Result.ok(record.intrinsics)
        return Result.err_msg(
            "No camera connected, and the library holds no intrinsics for "
            "{} at {}x{}.".format(config.camera_key(), config.width,
                                  config.height))

    # --------------------------------------------------------------- robot
    def robot_connected(self) -> bool:
        return self.api is not None and self.api.is_connected

    def convention_ready(self) -> Result[None]:
        """Refuse automated motion until the orientation convention has been
        confirmed. Nothing else in the app produces a wrong answer this
        quietly."""
        convention = self.settings.pose_convention()
        if not convention.verified:
            return Result.err_msg(
                "The controller's orientation convention has not been "
                "verified. Run the wizard on the Robot tab first -- an "
                "unverified convention produces a hand-eye result that looks "
                "entirely plausible and is wrong.")
        return Result.ok(None)

    def shutdown(self) -> None:
        if self.camera is not None:
            self.camera.disconnect()
        if self.api is not None:
            self.api.disconnect()
        self.save_settings()
