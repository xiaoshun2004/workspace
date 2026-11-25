# -*- coding: utf-8 -*-
"""
ui/tab_camera.py
----------------
Camera selection, live view, and intrinsics.

The intrinsics half is the part that matters. A USB camera has none, so it
cannot estimate a board pose until a set is solved here and stored in the
library. A RealSense has factory values, which are usually good -- the solve
is offered anyway, and the result is compared against the factory numbers so
the operator can see whether replacing them actually bought anything.

Intrinsics are keyed by camera AND resolution throughout, because fx, fy, ppx
and ppy all scale with stream size.
"""

from __future__ import annotations

import datetime
from typing import List, Optional

import numpy as np
from PySide6.QtCore import QTimer, Slot
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

import camera_factory
import handeye
import intrinsic_calib
import remount
from constants import BAD_COLOR, IDLE_COLOR, LIVE_VIEW_HZ, OK_COLOR, WARN_COLOR
from context import AppContext
from models import CameraInfo, IntrinsicsRecord, IntrinsicsSource
from qt_helpers import ImageLabel, make_hline
from workers import CallableWorker


class CameraTab(QWidget):
    """Connect a camera, watch it, and give it intrinsics."""

    def __init__(self, ctx: AppContext, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._ctx = ctx
        self._devices: List[CameraInfo] = []
        self._worker: Optional[CallableWorker] = None
        self._intrinsic_views: List[intrinsic_calib.IntrinsicView] = []
        # (camera_key, width, height) recorded when the FIRST view is accepted.
        # Views taken from one camera cannot be solved as another, and the
        # solve has to be stored under the identity the frames came from
        # rather than whatever the combos happen to say at solve time.
        self._views_identity: Optional[tuple] = None

        self._view = ImageLabel()
        self._status = QLabel("No camera connected.")
        self._detect_label = QLabel("-")
        self._intrinsics_label = QLabel("-")
        self._library_label = QLabel("-")
        self._views_label = QLabel("0 views captured")

        self._device_combo = QComboBox()
        self._mode_combo = QComboBox()
        self._auto_exposure = QCheckBox("Auto exposure")
        self._exposure = QSpinBox()
        self._gain = QSpinBox()
        self._prefer_library = QCheckBox("Prefer stored intrinsics over factory")
        self._show_depth = QCheckBox("Show depth")

        self._connect_btn = QPushButton("Connect")
        self._refresh_btn = QPushButton("Refresh")
        self._capture_view_btn = QPushButton("Capture view")
        self._solve_btn = QPushButton("Solve intrinsics")
        self._clear_views_btn = QPushButton("Clear views")
        self._use_library_btn = QPushButton("Load from library")
        self._forget_btn = QPushButton("Forget stored")

        self._build()
        self._wire()
        self._load_settings()
        self.refresh_devices()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / LIVE_VIEW_HZ))

    # --------------------------------------------------------------- layout
    def _build(self) -> None:
        device_box = QGroupBox("Device")
        device_row = QHBoxLayout()
        device_row.addWidget(QLabel("Camera:"))
        self._device_combo.setMinimumWidth(280)
        device_row.addWidget(self._device_combo, 1)
        device_row.addWidget(QLabel("Mode:"))
        self._mode_combo.setMinimumWidth(150)
        device_row.addWidget(self._mode_combo)
        device_row.addWidget(self._refresh_btn)
        device_row.addWidget(self._connect_btn)
        device_layout = QVBoxLayout(device_box)
        device_layout.addLayout(device_row)

        exposure_row = QHBoxLayout()
        exposure_row.addWidget(self._auto_exposure)
        exposure_row.addWidget(QLabel("Exposure:"))
        self._exposure.setRange(1, 200000)
        self._exposure.setSingleStep(100)
        exposure_row.addWidget(self._exposure)
        exposure_row.addWidget(QLabel("Gain:"))
        self._gain.setRange(0, 512)
        exposure_row.addWidget(self._gain)
        exposure_row.addWidget(self._show_depth)
        exposure_row.addStretch(1)
        device_layout.addLayout(exposure_row)
        device_layout.addWidget(self._status)

        intrinsics_box = QGroupBox("Intrinsics")
        intrinsics_layout = QVBoxLayout(intrinsics_box)
        self._intrinsics_label.setWordWrap(True)
        self._library_label.setWordWrap(True)
        intrinsics_layout.addWidget(self._intrinsics_label)
        intrinsics_layout.addWidget(self._library_label)
        intrinsics_layout.addWidget(self._prefer_library)
        intrinsics_layout.addWidget(make_hline())
        intrinsics_layout.addWidget(QLabel(
            "Intrinsic calibration: capture {}+ views of the board, tilted "
            "and reaching the frame corners.".format(
                intrinsic_calib.RECOMMENDED_VIEWS)))
        capture_row = QHBoxLayout()
        capture_row.addWidget(self._capture_view_btn)
        capture_row.addWidget(self._solve_btn)
        capture_row.addWidget(self._clear_views_btn)
        capture_row.addWidget(self._use_library_btn)
        capture_row.addWidget(self._forget_btn)
        capture_row.addStretch(1)
        intrinsics_layout.addLayout(capture_row)
        intrinsics_layout.addWidget(self._views_label)

        left = QVBoxLayout()
        left.addWidget(device_box)
        left.addWidget(intrinsics_box)
        left.addStretch(1)

        right = QVBoxLayout()
        right.addWidget(self._view, 1)
        right.addWidget(self._detect_label)

        root = QHBoxLayout(self)
        panel = QWidget()
        panel.setLayout(left)
        panel.setMaximumWidth(520)
        root.addWidget(panel)
        root.addLayout(right, 1)

    def _wire(self) -> None:
        self._refresh_btn.clicked.connect(self.refresh_devices)
        self._connect_btn.clicked.connect(self._toggle_connection)
        self._device_combo.currentIndexChanged.connect(self._on_device_changed)
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        self._auto_exposure.toggled.connect(self._on_auto_exposure)
        self._exposure.valueChanged.connect(self._on_exposure)
        self._gain.valueChanged.connect(self._on_gain)
        self._prefer_library.toggled.connect(self._on_prefer_library)
        self._show_depth.toggled.connect(self._on_show_depth)
        self._capture_view_btn.clicked.connect(self._capture_intrinsic_view)
        self._solve_btn.clicked.connect(self._solve_intrinsics)
        self._clear_views_btn.clicked.connect(self._clear_views)
        self._use_library_btn.clicked.connect(self._load_from_library)
        self._forget_btn.clicked.connect(self._forget_stored)

    def _load_settings(self) -> None:
        config = self._ctx.settings.camera_config()
        self._auto_exposure.setChecked(config.auto_exposure)
        if config.exposure is not None:
            self._exposure.setValue(int(config.exposure))
        if config.gain is not None:
            self._gain.setValue(int(config.gain))
        self._prefer_library.setChecked(
            self._ctx.settings.prefer_library_intrinsics)
        self._show_depth.setChecked(self._ctx.settings.show_depth_view)

    # ------------------------------------------------------------ discovery
    @Slot()
    def refresh_devices(self) -> None:
        found = camera_factory.enumerate_cameras(
            self._ctx.settings.usb_probe_max_index)
        # An enumeration that failed or only half worked must say so: an empty
        # combo is otherwise indistinguishable from "nothing is plugged in".
        if found.is_err:
            self._ctx.log.error(found.message)
        elif found.message:
            self._ctx.log.warn(found.message)
        self._devices = list(found.data) if found.data else []
        self._device_combo.blockSignals(True)
        self._device_combo.clear()
        for info in self._devices:
            self._device_combo.addItem(info.label, info.ident)
        self._device_combo.blockSignals(False)
        if not self._devices:
            self._set_status("No cameras found.", BAD_COLOR)
            self._mode_combo.clear()
            return
        config = self._ctx.settings.camera_config()
        for row, info in enumerate(self._devices):
            if info.ident == config.ident and info.kind == config.kind:
                self._device_combo.setCurrentIndex(row)
                break
        self._on_device_changed()
        self._ctx.log.info("Found {} camera(s).".format(len(self._devices)))

    def _current_device(self) -> Optional[CameraInfo]:
        row = self._device_combo.currentIndex()
        if 0 <= row < len(self._devices):
            return self._devices[row]
        return None

    @Slot()
    def _on_device_changed(self) -> None:
        info = self._current_device()
        if info is None:
            return
        modes = camera_factory.supported_modes(info.kind, info.ident)
        # A mode query that failed leaves the combo empty, which the operator
        # would otherwise read as "this device offers nothing".
        if modes.is_err:
            self._ctx.log.error(modes.message)
        elif modes.message:
            self._ctx.log.warn(modes.message)
        self._mode_combo.blockSignals(True)
        self._mode_combo.clear()
        for mode in (modes.data or []):
            self._mode_combo.addItem("{}x{} @ {} fps".format(*mode), mode)
        self._mode_combo.blockSignals(False)
        config = self._ctx.settings.camera_config()
        wanted = (config.width, config.height, config.fps)
        row = self._mode_combo.findData(wanted)
        if row >= 0:
            self._mode_combo.setCurrentIndex(row)
        self._on_mode_changed()

    @Slot()
    def _on_mode_changed(self) -> None:
        info = self._current_device()
        mode = self._mode_combo.currentData()
        if info is None or mode is None:
            return
        config = self._ctx.settings.camera_config()
        config.kind = info.kind
        config.ident = info.ident
        config.width, config.height, config.fps = mode
        self._ctx.settings.set_camera_config(config)
        self._ctx.save_settings()
        # Both combos land here, so this is where a device or a resolution
        # change invalidates the captured views.
        if (self._views_identity is not None
                and self._views_identity != self._current_identity()):
            self._clear_views_for("the camera or the stream mode changed")
        self._update_library_label()

    # ----------------------------------------------------------- connection
    @Slot()
    def _toggle_connection(self) -> None:
        if self._ctx.camera_connected():
            self._ctx.camera.disconnect()
            self._ctx.camera = None
            self._ctx.camera_changed.emit()
            self._connect_btn.setText("Connect")
            self._set_status("Camera disconnected.", IDLE_COLOR)
            self._view.clear_image()
            self._clear_views_for("the camera was disconnected")
            return

        config = self._ctx.settings.camera_config()
        built = camera_factory.make_camera(config)
        if built.is_err:
            self._error(built.message)
            return
        camera = built.data
        self._set_status("Connecting" + "\u2026", WARN_COLOR)
        self._connect_btn.setEnabled(False)
        self._worker = CallableWorker(camera.connect, parent=self)
        self._worker.done.connect(
            lambda result: self._on_connected(camera, result))
        self._worker.start()

    def _on_connected(self, camera, result) -> None:
        self._connect_btn.setEnabled(True)
        if result.is_err:
            self._error(result.message)
            self._set_status(result.message, BAD_COLOR)
            return
        self._ctx.camera = camera
        self._clear_views_for("a camera was connected")
        self._apply_library_intrinsics_if_needed()
        self._ctx.camera_changed.emit()
        self._connect_btn.setText("Disconnect")
        self._set_status(result.message or "Camera connected.", OK_COLOR)
        self._ctx.log.success(result.message or "Camera connected.")
        self._update_intrinsics_label()

    def _apply_library_intrinsics_if_needed(self) -> None:
        config = self._ctx.settings.camera_config()
        record = self._ctx.library.find(config.camera_key(), config.width,
                                        config.height)
        if record is None:
            return
        if (self._ctx.settings.prefer_library_intrinsics
                or not self._ctx.camera.has_factory_intrinsics()):
            self._ctx.camera.set_intrinsics_override(record.intrinsics)
            self._ctx.log.info(
                "Using stored intrinsics for {} at {}x{} (RMS {:.2f} px).".format(
                    config.camera_key(), config.width, config.height,
                    record.rms_px))

    # ------------------------------------------------------------- live view
    @Slot()
    def _tick(self) -> None:
        if not self._ctx.camera_connected():
            return
        if self._show_depth.isChecked() and self._ctx.camera.has_depth():
            frame = self._ctx.camera.get_depth_colormap()
            if frame.is_ok:
                self._view.set_cv_image(frame.data)
            return
        frame = self._ctx.camera.get_color()
        if frame.is_err:
            return
        intrinsics = self._ctx.intrinsics()
        det = handeye.detect_board(
            frame.data, self._ctx.board, self._ctx.detector,
            intrinsics.data if intrinsics.is_ok else None,
            min_coverage=self._ctx.settings.min_image_coverage)
        image = handeye.draw_overlay(
            frame.data, det, intrinsics.data if intrinsics.is_ok else None)
        self._view.set_cv_image(image)
        self._detect_label.setText(self._detection_text(det, intrinsics))

    def _detection_text(self, det, intrinsics) -> str:
        if not det.found:
            return "No board detected."
        parts = ["{} corners".format(det.n_corners)]
        if intrinsics.is_err:
            parts.append("no intrinsics: " + intrinsics.message)
            return "  |  ".join(parts)
        if det.T_target2cam is None:
            parts.append(det.message or "pose not estimated")
            return "  |  ".join(parts)
        distance = float(np.linalg.norm(det.T_target2cam[:3, 3]))
        parts.append("reproj {:.2f} px".format(det.reproj_error))
        parts.append("distance {:.3f} m".format(distance))
        margin = remount.view_margin_deg(
            det.charuco_corners, intrinsics.data.width, intrinsics.data.height,
            intrinsics.data)
        parts.append("remount margin {:.1f} deg".format(margin))
        return "  |  ".join(parts)

    # ------------------------------------------------------------ intrinsics
    def _update_intrinsics_label(self) -> None:
        intrinsics = self._ctx.intrinsics()
        if intrinsics.is_err:
            self._intrinsics_label.setText(
                "<span style='color:{}'>{}</span>".format(
                    WARN_COLOR, intrinsics.message))
            return
        i = intrinsics.data
        warning = ""
        if i.unsupported_model():
            warning = ("  <span style='color:{}'>fisheye model cannot be "
                       "used</span>".format(BAD_COLOR))
        elif i.needs_manual_undistort():
            warning = ("  <span style='color:{}'>inverse Brown-Conrady: "
                       "undistorted exactly before solvePnP</span>".format(
                           WARN_COLOR))
        self._intrinsics_label.setText(
            "{}x{}  fx={:.2f} fy={:.2f}  ppx={:.2f} ppy={:.2f}<br>"
            "model '{}', HFOV {:.1f} deg{}".format(
                i.width, i.height, i.fx, i.fy, i.ppx, i.ppy, i.model,
                i.hfov_deg(), warning))
        self._update_library_label()

    def _update_library_label(self) -> None:
        config = self._ctx.settings.camera_config()
        record = self._ctx.library.find(config.camera_key(), config.width,
                                        config.height)
        if record is None:
            self._library_label.setText(
                "Library: nothing stored for {} at {}x{}.".format(
                    config.camera_key(), config.width, config.height))
            return
        self._library_label.setText(
            "Library: {} set from {}, RMS {:.2f} px, {} views, {}.".format(
                record.source.value, record.timestamp[:10] or "unknown",
                record.rms_px, record.n_images, record.note or "no note"))

    def _current_identity(self) -> tuple:
        """(camera_key, width, height) as currently configured."""
        config = self._ctx.settings.camera_config()
        return (config.camera_key(), int(config.width), int(config.height))

    def _clear_views_for(self, reason: str) -> None:
        """Drop the captured views, saying why.

        A view set is only meaningful for the one camera and resolution it was
        captured at, so anything that can change either of those throws it
        away rather than letting the sets mix.
        """
        if self._intrinsic_views:
            self._ctx.log.info("{} intrinsic view(s) discarded: {}.".format(
                len(self._intrinsic_views), reason))
        self._intrinsic_views = []
        self._views_identity = None
        self._update_views_label()

    @Slot()
    def _capture_intrinsic_view(self) -> None:
        if not self._ctx.camera_connected():
            self._error("Connect a camera first.")
            return
        identity = self._current_identity()
        if self._views_identity is not None and identity != self._views_identity:
            self._error(
                "The captured views belong to {} at {}x{}, but the camera is "
                "now configured as {} at {}x{}. Clear the views before "
                "capturing for a different camera or resolution.".format(
                    self._views_identity[0], self._views_identity[1],
                    self._views_identity[2], identity[0], identity[1],
                    identity[2]))
            return
        frame = self._ctx.camera.get_color()
        if frame.is_err:
            self._error(frame.message)
            return
        added = intrinsic_calib.add_view(frame.data, self._ctx.board,
                                         self._ctx.detector,
                                         self._intrinsic_views)
        if added.is_err:
            self._ctx.log.warn("View rejected: {}".format(added.message))
        else:
            if self._views_identity is None:
                self._views_identity = identity
                self._ctx.log.info(
                    "Capturing intrinsic views for {} at {}x{}; the solve is "
                    "stored under that identity.".format(*identity))
            self._ctx.log.info("View {} accepted ({} corners).".format(
                len(self._intrinsic_views), added.data.n_corners))
        self._update_views_label()

    def _update_views_label(self) -> None:
        identity = self._views_identity or self._current_identity()
        text = "{} views captured".format(len(self._intrinsic_views))
        if self._intrinsic_views:
            text += "  |  " + intrinsic_calib.coverage_report(
                self._intrinsic_views, identity[1], identity[2])
            text += "  |  {} at {}x{}".format(*identity)
        self._views_label.setText(text)

    @Slot()
    def _clear_views(self) -> None:
        self._clear_views_for("cleared by the operator")

    @Slot()
    def _solve_intrinsics(self) -> None:
        if not self._intrinsic_views or self._views_identity is None:
            self._error("Capture some views of the board first.")
            return
        # The identity recorded when the first view was accepted, never the
        # current combo state: the two can differ, and solving 1280x720 point
        # data as 640x480 stores focal lengths that are silently 2x wrong.
        camera_key, width, height = self._views_identity
        solved = intrinsic_calib.solve_intrinsics(
            self._intrinsic_views, self._ctx.board, (width, height))
        if solved.is_err and solved.data is None:
            self._error(solved.message)
            return
        result = solved.data
        factory = (self._ctx.camera.factory_intrinsics()
                   if self._ctx.camera_connected() else None)
        comparison = intrinsic_calib.compare_to_factory(
            result.intrinsics,
            factory.data if factory is not None and factory.is_ok else None)
        summary = ("RMS {:.3f} px over {} views.\n{}\n{}\n\nStore this set for "
                   "{} at {}x{}?".format(
                       result.rms_px, result.n_views, result.coverage_report,
                       comparison, camera_key, width, height))
        if solved.is_err:
            summary = solved.message + "\n\n" + summary
        answer = QMessageBox.question(
            self, "Intrinsic calibration", summary,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if answer != QMessageBox.StandardButton.Yes:
            return
        record = IntrinsicsRecord(
            camera_key=camera_key, intrinsics=result.intrinsics,
            source=IntrinsicsSource.CALIBRATED, rms_px=result.rms_px,
            n_images=result.n_views,
            timestamp=datetime.datetime.now().isoformat(timespec="seconds"),
            note=result.coverage_report)
        stored = self._ctx.library.put(record)
        if stored.is_err:
            self._error(stored.message)
            return
        self._ctx.log.success("Intrinsics stored for {} at {}x{}.".format(
            camera_key, width, height))
        if self._current_identity() == self._views_identity:
            self._load_from_library()
        else:
            self._ctx.log.warn(
                "The stored set belongs to {} at {}x{}, which is not the "
                "camera and mode now selected, so it was not loaded into the "
                "live camera.".format(camera_key, width, height))

    @Slot()
    def _load_from_library(self) -> None:
        if not self._ctx.camera_connected():
            self._error("Connect a camera first.")
            return
        config = self._ctx.settings.camera_config()
        record = self._ctx.library.find(config.camera_key(), config.width,
                                        config.height)
        if record is None:
            self._error("No stored intrinsics for {} at {}x{}.".format(
                config.camera_key(), config.width, config.height))
            return
        self._ctx.camera.set_intrinsics_override(record.intrinsics)
        self._ctx.camera_changed.emit()
        self._update_intrinsics_label()
        self._ctx.log.success("Loaded stored intrinsics.")

    @Slot()
    def _forget_stored(self) -> None:
        config = self._ctx.settings.camera_config()
        removed = self._ctx.library.remove(config.camera_key(), config.width,
                                           config.height)
        if removed.is_err:
            self._error(removed.message)
            return
        if self._ctx.camera is not None:
            self._ctx.camera.set_intrinsics_override(None)
        self._update_intrinsics_label()
        self._ctx.log.info("Stored intrinsics removed.")

    # --------------------------------------------------------------- options
    @Slot(bool)
    def _on_auto_exposure(self, enabled: bool) -> None:
        config = self._ctx.settings.camera_config()
        config.auto_exposure = enabled
        self._ctx.settings.set_camera_config(config)
        self._ctx.save_settings()
        if self._ctx.camera_connected():
            self._report(self._ctx.camera.set_auto_exposure(enabled))

    @Slot(int)
    def _on_exposure(self, value: int) -> None:
        config = self._ctx.settings.camera_config()
        config.exposure = int(value)
        self._ctx.settings.set_camera_config(config)
        self._ctx.save_settings()
        if self._ctx.camera_connected() and not self._auto_exposure.isChecked():
            self._report(self._ctx.camera.set_exposure(value))

    @Slot(int)
    def _on_gain(self, value: int) -> None:
        config = self._ctx.settings.camera_config()
        config.gain = int(value)
        self._ctx.settings.set_camera_config(config)
        self._ctx.save_settings()
        if self._ctx.camera_connected() and not self._auto_exposure.isChecked():
            self._report(self._ctx.camera.set_gain(value))

    @Slot(bool)
    def _on_prefer_library(self, enabled: bool) -> None:
        self._ctx.settings.prefer_library_intrinsics = enabled
        self._ctx.save_settings()
        if self._ctx.camera_connected():
            if enabled:
                self._apply_library_intrinsics_if_needed()
            elif self._ctx.camera.has_factory_intrinsics():
                self._ctx.camera.set_intrinsics_override(None)
            self._update_intrinsics_label()

    @Slot(bool)
    def _on_show_depth(self, enabled: bool) -> None:
        self._ctx.settings.show_depth_view = enabled
        self._ctx.save_settings()

    # ---------------------------------------------------------------- shutdown
    def stop_workers(self) -> None:
        """Stop the live tick and join this tab's worker. Never raises.

        Called on the close path, where a QThread still running when the
        widget tree is destroyed aborts the process.
        """
        self._timer.stop()
        worker = self._worker
        self._worker = None
        if worker is not None and worker.isRunning():
            # The connect call it is blocked in cannot be interrupted; the
            # cap is there so closing the app is not hostage to a wedged
            # camera driver.
            worker.requestInterruption()
            worker.wait(2000)

    # ---------------------------------------------------------------- helpers
    def _set_status(self, text: str, color: str) -> None:
        self._status.setText(
            "<span style='color:{}'>{}</span>".format(color, text))

    def _report(self, result) -> None:
        if result.is_err:
            self._ctx.log.warn(result.message)
        elif result.message:
            self._ctx.log.info(result.message)

    def _error(self, message: str) -> None:
        self._ctx.log.error(message)
        QMessageBox.warning(self, "Camera", message)
