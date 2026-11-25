# -*- coding: utf-8 -*-
"""
ui/tab_verify.py
----------------
The only test that means anything: click a feature, watch where the tool goes.

Spread metrics are residuals computed from the same data the solve used. They
catch gross errors, not systematic ones. A calibration is validated by
pointing at something whose position is known and measuring the miss.

The chain, all in metres:

    p_cam    = deproject(u, v, depth)
    p_base   = T_mount2base . T_cam2mount . p_cam

then the target is backed off along the camera ray by the standoff, so a
mistake stops short of the board rather than through it. Without depth (a USB
camera) the operator supplies the distance; the direction is still exact, so
the miss is informative even when the range is a guess.
"""

from __future__ import annotations

import os
from typing import List, Optional

import numpy as np
from PySide6.QtCore import QTimer, Slot
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

import handeye
import project_io
import transforms
from constants import BAD_COLOR, DANGER_QSS, LIVE_VIEW_HZ, OK_COLOR, WARN_COLOR
from context import AppContext
from models import CalibInstance, Result
from nova5_api import check_move
from qt_helpers import ImageLabel, make_hline
from workers import CallableWorker


class VerifyTab(QWidget):
    """Load a calibration and drive the tool to a clicked pixel."""

    def __init__(self, ctx: AppContext, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._ctx = ctx
        self._instance: Optional[CalibInstance] = None
        self._instance_dirs: List[str] = []
        self._worker: Optional[CallableWorker] = None
        self._last_target: Optional[np.ndarray] = None

        self._project_combo = QComboBox()
        self._instance_combo = QComboBox()
        self._load_btn = QPushButton("Load")
        self._standoff = QDoubleSpinBox()
        self._patch = QSpinBox()
        self._manual_depth = QDoubleSpinBox()
        self._use_manual_depth = QCheckBox("Use the distance below instead of depth")
        self._confirm = QCheckBox("Confirm before moving")
        self._move_btn = QPushButton("Move to last point")

        self._view = ImageLabel()
        self._detail = QLabel("Load a calibration, then click a pixel.")
        self._summary = QLabel("-")
        self._validity = QLabel("-")

        self._build()
        self._wire()
        self._load_settings()
        self.refresh_projects()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / LIVE_VIEW_HZ))

    # --------------------------------------------------------------- layout
    def _build(self) -> None:
        load_box = QGroupBox("Calibration")
        load_layout = QVBoxLayout(load_box)
        row = QHBoxLayout()
        row.addWidget(QLabel("Project:"))
        row.addWidget(self._project_combo, 1)
        row.addWidget(QLabel("Instance:"))
        row.addWidget(self._instance_combo, 1)
        row.addWidget(self._load_btn)
        load_layout.addLayout(row)
        for label in (self._summary, self._validity):
            label.setWordWrap(True)
            load_layout.addWidget(label)

        move_box = QGroupBox("Move to point")
        move_layout = QVBoxLayout(move_box)
        options = QHBoxLayout()
        options.addWidget(QLabel("Standoff m:"))
        self._standoff.setRange(0.0, 0.5)
        self._standoff.setSingleStep(0.01)
        self._standoff.setDecimals(3)
        options.addWidget(self._standoff)
        options.addWidget(QLabel("Depth patch px:"))
        self._patch.setRange(0, 15)
        options.addWidget(self._patch)
        options.addWidget(QLabel("Distance m:"))
        self._manual_depth.setRange(0.01, 5.0)
        self._manual_depth.setSingleStep(0.01)
        self._manual_depth.setDecimals(3)
        options.addWidget(self._manual_depth)
        options.addStretch(1)
        move_layout.addLayout(options)
        move_layout.addWidget(self._use_manual_depth)
        move_layout.addWidget(self._confirm)
        move_layout.addWidget(self._move_btn)
        danger = QLabel("This commands a real Cartesian move. The standoff "
                        "backs the target off along the camera ray so a bad "
                        "calibration stops short of the surface.")
        danger.setStyleSheet(DANGER_QSS)
        danger.setWordWrap(True)
        move_layout.addWidget(danger)
        self._detail.setWordWrap(True)
        move_layout.addWidget(make_hline())
        move_layout.addWidget(self._detail)

        left = QVBoxLayout()
        left.addWidget(load_box)
        left.addWidget(move_box)
        left.addStretch(1)

        root = QHBoxLayout(self)
        panel = QWidget()
        panel.setLayout(left)
        panel.setMaximumWidth(560)
        root.addWidget(panel)
        root.addWidget(self._view, 1)

    def _wire(self) -> None:
        self._project_combo.currentIndexChanged.connect(self._refresh_instances)
        self._load_btn.clicked.connect(self._load_instance)
        self._view.clicked.connect(self._on_pixel_clicked)
        self._move_btn.clicked.connect(self._move_to_last)
        self._standoff.valueChanged.connect(self._on_settings)
        self._patch.valueChanged.connect(self._on_settings)
        self._manual_depth.valueChanged.connect(self._on_settings)
        self._confirm.toggled.connect(self._on_settings)

    def _load_settings(self) -> None:
        settings = self._ctx.settings
        self._standoff.setValue(float(settings.standoff_m))
        self._patch.setValue(int(settings.depth_patch_px))
        self._manual_depth.setValue(float(settings.manual_depth_m))
        self._confirm.setChecked(settings.verify_confirm_move)

    # ------------------------------------------------------------- loading
    @Slot()
    def refresh_projects(self) -> None:
        self._project_combo.blockSignals(True)
        self._project_combo.clear()
        for name in project_io.list_projects(self._ctx.settings.root_dir):
            self._project_combo.addItem(name, name)
        self._project_combo.blockSignals(False)
        self._refresh_instances()

    @Slot()
    def _refresh_instances(self) -> None:
        self._instance_combo.clear()
        self._instance_dirs = []
        name = self._project_combo.currentData()
        if not name:
            return
        base = project_io.ProjectPaths(self._ctx.settings.root_dir,
                                       name).instances_dir
        for folder in project_io.list_instances(self._ctx.settings.root_dir,
                                                name):
            self._instance_combo.addItem(folder)
            self._instance_dirs.append(os.path.join(base, folder))

    @Slot()
    def _load_instance(self) -> None:
        row = self._instance_combo.currentIndex()
        if row < 0 or row >= len(self._instance_dirs):
            return
        # A target computed from the previous calibration must not survive
        # into this one: it encodes that instance's T_cam2mount and the arm
        # pose of the moment it was clicked.
        self._last_target = None
        loaded = project_io.load_instance(self._instance_dirs[row])
        if loaded.is_err:
            self._ctx.log.error(loaded.message)
            return
        instance, samples = loaded.data
        if instance.result is None:
            self._summary.setText(
                "<span style='color:{}'>That instance was never "
                "solved.</span>".format(BAD_COLOR))
            self._instance = None
            return
        self._instance = instance
        t = instance.result.T_cam2mount
        self._summary.setText(
            "T_cam2mount  x {:+.4f}  y {:+.4f}  z {:+.4f} m  |  {} over {} "
            "samples  |  spread {:.2f} mm / {:.2f} deg  |  captured at "
            "{}x{}".format(
                t[0, 3], t[1, 3], t[2, 3], instance.result.method,
                instance.result.num_samples,
                instance.result.board_origin_spread_mm,
                instance.result.rotation_spread_deg,
                instance.intrinsics.width if instance.intrinsics else 0,
                instance.intrinsics.height if instance.intrinsics else 0))
        self._ctx.log.success("Loaded calibration from {}".format(
            self._instance_dirs[row]))
        self._check_validity()

    def _check_validity(self) -> None:
        if self._instance is None:
            return
        live = self._ctx.intrinsics()
        if live.is_err:
            self._validity.setText(
                "<span style='color:{}'>{}</span>".format(WARN_COLOR,
                                                          live.message))
            return
        frames = None
        if self._ctx.robot_connected():
            state = self._ctx.api.latest()
            if state.is_ok:
                frames = state.data.frame_identity()
        applicable = project_io.check_instance_applicable(
            self._instance, live.data, frames,
            live_convention=self._ctx.settings.pose_convention())
        if applicable.is_err:
            self._validity.setText(
                "<span style='color:{}'>{}</span>".format(BAD_COLOR,
                                                          applicable.message))
        else:
            self._validity.setText(
                "<span style='color:{}'>Valid for the live camera and the "
                "current Tool/User frames.</span>".format(OK_COLOR))

    # ------------------------------------------------------------- clicking
    @Slot(int, int)
    def _on_pixel_clicked(self, u: int, v: int) -> None:
        # Every gate below can reject this click. Dropping the previous target
        # first means 'Move to last point' is dead until a click succeeds,
        # rather than commanding the arm to the pixel before this one.
        self._last_target = None
        if self._instance is None:
            self._detail.setText("Load a solved calibration first.")
            return
        if not self._ctx.camera_connected():
            self._detail.setText("Connect a camera first.")
            return

        if self._use_manual_depth.isChecked() or not self._ctx.camera.has_depth():
            distance = float(self._manual_depth.value())
            source = "operator-supplied distance"
        else:
            measured = self._ctx.camera.depth_at(u, v, int(self._patch.value()))
            if measured.is_err:
                self._detail.setText(measured.message)
                return
            distance = measured.data
            source = "measured depth"

        point = self._ctx.camera.deproject(float(u), float(v), distance)
        if point.is_err:
            self._detail.setText(point.message)
            return
        p_cam = np.asarray(point.data, dtype=np.float64)

        if not self._ctx.robot_connected():
            self._detail.setText(
                "Camera-frame point ({}) x {:+.4f}  y {:+.4f}  z {:+.4f} m. "
                "Connect the arm to map it into the base frame.".format(
                    source, *p_cam))
            return
        current = self._ctx.api.pose(self._ctx.settings.pose_convention())
        if current.is_err:
            self._detail.setText(current.message)
            return

        T_cam2base = transforms.se3_compose(current.data,
                                            self._instance.result.T_cam2mount)
        p_base = transforms.transform_point(T_cam2base, p_cam)
        cam_origin = transforms.se3_translation(T_cam2base)
        ray = p_base - cam_origin
        norm = float(np.linalg.norm(ray))
        if norm < 1e-6:
            self._detail.setText("Degenerate ray; pick another pixel.")
            return
        target = p_base - ray / norm * float(self._standoff.value())

        # Keep the current orientation: this is a pointing test, not a pose
        # test, and re-orienting would add a second thing to blame for a miss.
        T_target = np.array(current.data, dtype=np.float64, copy=True)
        T_target[:3, 3] = target
        self._last_target = T_target

        self._detail.setText(
            "pixel ({}, {})  |  {} {:.3f} m<br>"
            "camera frame  x {:+.4f}  y {:+.4f}  z {:+.4f} m<br>"
            "base frame    x {:+.4f}  y {:+.4f}  z {:+.4f} m<br>"
            "target after {:.0f} mm standoff  x {:+.4f}  y {:+.4f}  "
            "z {:+.4f} m".format(
                u, v, source, distance, p_cam[0], p_cam[1], p_cam[2],
                p_base[0], p_base[1], p_base[2],
                self._standoff.value() * 1000.0, target[0], target[1],
                target[2]))

    @Slot()
    def _move_to_last(self) -> None:
        if self._last_target is None:
            return
        if not self._ctx.robot_connected():
            QMessageBox.information(self, "Verify", "Connect the arm first.")
            return
        convention = self._ctx.settings.pose_convention()
        command = convention.se3_to_command(self._last_target)
        state = self._ctx.api.latest()
        if state.is_err:
            QMessageBox.warning(self, "Verify", state.message)
            return
        gate = check_move(command, state.data,
                          self._ctx.settings.safety_config())
        if gate.is_err:
            QMessageBox.warning(self, "Verify", gate.message)
            return
        if self._confirm.isChecked():
            answer = QMessageBox.question(
                self, "Move",
                "Move the tool to X {:.1f}  Y {:.1f}  Z {:.1f} mm?".format(
                    command[0], command[1], command[2]),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return
        self._worker = CallableWorker(
            self._ctx.api.move_l, command, self._ctx.settings.user_index,
            self._ctx.settings.tool_index, parent=self)
        self._worker.done.connect(self._on_move_done)
        self._worker.start()

    @Slot(object)
    def _on_move_done(self, result: Result) -> None:
        if result.is_err:
            self._ctx.log.error(result.message)
            return
        self._ctx.log.success(
            "Move commanded. Measure where the tool actually landed -- that "
            "number, not the residual spread, is the calibration's accuracy.")

    # ---------------------------------------------------------------- shutdown
    def stop_workers(self) -> None:
        """Stop the live tick and join this tab's move worker. Never raises."""
        self._timer.stop()
        worker = self._worker
        self._worker = None
        if worker is not None and worker.isRunning():
            # The move command it is blocked in cannot be interrupted; the cap
            # keeps closing the app from waiting on a wedged socket.
            worker.requestInterruption()
            worker.wait(2000)

    # ------------------------------------------------------------- live view
    @Slot()
    def _tick(self) -> None:
        if not self._ctx.camera_connected():
            return
        frame = self._ctx.camera.get_color()
        if frame.is_err:
            return
        intrinsics = self._ctx.intrinsics()
        det = handeye.detect_board(
            frame.data, self._ctx.board, self._ctx.detector,
            intrinsics.data if intrinsics.is_ok else None,
            min_coverage=self._ctx.settings.min_image_coverage)
        self._view.set_cv_image(handeye.draw_overlay(
            frame.data, det, intrinsics.data if intrinsics.is_ok else None))

    # -------------------------------------------------------------- settings
    @Slot()
    def _on_settings(self) -> None:
        settings = self._ctx.settings
        settings.standoff_m = float(self._standoff.value())
        settings.depth_patch_px = int(self._patch.value())
        settings.manual_depth_m = float(self._manual_depth.value())
        settings.verify_confirm_move = self._confirm.isChecked()
        self._ctx.save_settings()
