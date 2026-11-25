# -*- coding: utf-8 -*-
"""
ui/tab_robot.py
---------------
Connect to the Nova-5, watch it, jog it, and pin down its pose convention.

The convention wizard is the important part. The real-time feedback packet
reports the same rotation twice -- as (rx, ry, rz) at byte 624 and as a
quaternion at byte 1384 -- so the Euler convention can be SOLVED rather than
guessed. Strike four or five clearly different orientations, press Solve, and
the app searches 12 axis sequences x intrinsic/extrinsic x degrees/radians x
rotation-vector x both quaternion component orders for the one that reproduces
the quaternion.

Nothing else in this app can produce a wrong answer as quietly as a wrong
convention, which is why automated motion stays disabled until this passes.
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np
from PySide6.QtCore import QTimer, Slot
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

import pose_convention as pc
from angle_format_widget import AngleFormatSelector
from constants import (
    BAD_COLOR,
    DANGER_QSS,
    IDLE_COLOR,
    OK_COLOR,
    SYM_CHECK,
    SYM_WARN,
    WARN_COLOR,
)
from context import AppContext
from models import Result, RobotPose
from nova5_api import Nova5Api, check_move
from workers import CallableWorker

# Speed-factor changes are debounced by this much: valueChanged fires for every
# intermediate value while the operator holds the arrow, and each one is a
# blocking dashboard round trip.
SPEED_DEBOUNCE_MS = 300

JOG_AXES = ("X", "Y", "Z", "Rx", "Ry", "Rz")
JOINT_AXES = ("J1", "J2", "J3", "J4", "J5", "J6")


class RobotTab(QWidget):
    """Arm connection, live state, jogging, safety, and the pose convention."""

    def __init__(self, ctx: AppContext, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._ctx = ctx
        # Every in-flight command, so none of them can outlive the window and
        # so a finished one is reclaimed instead of accumulating as a child.
        self._workers: List[CallableWorker] = []
        self._jog_buttons: List[QPushButton] = []
        self._jog_busy = False
        self._pending_api: Optional[Nova5Api] = None
        self._convention_samples: List[RobotPose] = []

        self._ip = QLineEdit(ctx.settings.robot_ip)
        self._connect_btn = QPushButton("Connect")
        self._enable_btn = QPushButton("Enable")
        self._clear_btn = QPushButton("Clear error")
        self._status = QLabel("Not connected.")
        self._pose_label = QLabel("-")
        self._joints_label = QLabel("-")
        self._health_label = QLabel("-")
        self._safety_label = QLabel("-")

        self._speed_factor = QSpinBox()
        self._jog_linear = QDoubleSpinBox()
        self._jog_angular = QDoubleSpinBox()
        self._tool_index = QSpinBox()
        self._user_index = QSpinBox()

        self._safety_enabled = QCheckBox("Safety gates enabled")
        self._confirm_each = QCheckBox("Confirm every automated pose")
        self._check_box = QCheckBox("Check workspace box")
        self._check_joints = QCheckBox("Check joint limits")
        self._require_enabled = QCheckBox("Require arm enabled before moving")
        self._max_step_mm = QDoubleSpinBox()
        self._max_step_deg = QDoubleSpinBox()
        self._payload_kg = QDoubleSpinBox()

        self._convention_label = QLabel("-")
        self._sample_label = QLabel("0 samples")
        self._sample_btn = QPushButton("Sample this pose")
        self._solve_btn = QPushButton("Solve convention")
        self._clear_samples_btn = QPushButton("Clear samples")

        self._speed_timer = QTimer(self)
        self._speed_timer.setSingleShot(True)
        self._speed_timer.setInterval(SPEED_DEBOUNCE_MS)
        self._speed_timer.timeout.connect(self._send_speed_factor)

        self._build()
        self._wire()
        self._load_settings()
        self._update_convention_label()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(100)

    # --------------------------------------------------------------- layout
    def _build(self) -> None:
        connection = QGroupBox("Connection")
        row = QHBoxLayout()
        row.addWidget(QLabel("IP:"))
        row.addWidget(self._ip)
        row.addWidget(self._connect_btn)
        row.addWidget(self._enable_btn)
        row.addWidget(self._clear_btn)
        row.addStretch(1)
        layout = QVBoxLayout(connection)
        layout.addLayout(row)
        layout.addWidget(self._status)
        for label in (self._pose_label, self._joints_label,
                      self._safety_label, self._health_label):
            label.setWordWrap(True)
            layout.addWidget(label)
        layout.addWidget(AngleFormatSelector(self._ctx.angle_format,
                                             compact=True))

        jog = QGroupBox("Jog " + SYM_WARN + " moves the arm")
        jog_layout = QVBoxLayout(jog)
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed factor %:"))
        self._speed_factor.setRange(1, 100)
        speed_row.addWidget(self._speed_factor)
        speed_row.addWidget(QLabel("Step mm:"))
        self._jog_linear.setRange(0.1, 100.0)
        self._jog_linear.setSingleStep(1.0)
        speed_row.addWidget(self._jog_linear)
        speed_row.addWidget(QLabel("Step deg:"))
        self._jog_angular.setRange(0.1, 45.0)
        self._jog_angular.setSingleStep(0.5)
        speed_row.addWidget(self._jog_angular)
        speed_row.addStretch(1)
        jog_layout.addLayout(speed_row)

        frame_row = QHBoxLayout()
        frame_row.addWidget(QLabel("Tool index:"))
        self._tool_index.setRange(0, 9)
        frame_row.addWidget(self._tool_index)
        frame_row.addWidget(QLabel("User index:"))
        self._user_index.setRange(0, 9)
        frame_row.addWidget(self._user_index)
        frame_row.addWidget(QLabel(
            "(recorded with every calibration; changing the taught TCP "
            "invalidates it)"))
        frame_row.addStretch(1)
        jog_layout.addLayout(frame_row)

        grid = QGridLayout()
        for column, axis in enumerate(JOG_AXES):
            grid.addWidget(self._jog_button(axis, "+"), 0, column)
            grid.addWidget(self._jog_button(axis, "-"), 1, column)
        for column, axis in enumerate(JOINT_AXES):
            grid.addWidget(self._jog_button(axis, "+"), 2, column)
            grid.addWidget(self._jog_button(axis, "-"), 3, column)
        jog_layout.addLayout(grid)
        warning = QLabel("Jogging is a real motion. Keep a hand near the "
                         "physical e-stop.")
        warning.setStyleSheet(DANGER_QSS)
        jog_layout.addWidget(warning)

        safety = QGroupBox("Safety")
        safety_layout = QVBoxLayout(safety)
        for box in (self._safety_enabled, self._confirm_each, self._check_box,
                    self._check_joints, self._require_enabled):
            safety_layout.addWidget(box)
        limits = QHBoxLayout()
        limits.addWidget(QLabel("Max step mm:"))
        self._max_step_mm.setRange(0.0, 2000.0)
        self._max_step_mm.setSingleStep(10.0)
        limits.addWidget(self._max_step_mm)
        limits.addWidget(QLabel("Max joint step deg:"))
        self._max_step_deg.setRange(0.0, 360.0)
        limits.addWidget(self._max_step_deg)
        limits.addWidget(QLabel("Payload kg:"))
        self._payload_kg.setRange(0.0, 10.0)
        self._payload_kg.setSingleStep(0.05)
        self._payload_kg.setDecimals(3)
        limits.addWidget(self._payload_kg)
        limits.addStretch(1)
        safety_layout.addLayout(limits)
        safety_layout.addWidget(QLabel(
            "Payload feeds the controller's collision detection. Leaving it "
            "at zero with a camera bracket fitted makes the thresholds wrong "
            "in the unsafe direction."))

        convention = QGroupBox("Pose convention")
        convention_layout = QVBoxLayout(convention)
        self._convention_label.setWordWrap(True)
        convention_layout.addWidget(self._convention_label)
        convention_layout.addWidget(QLabel(
            "Move the arm to four or five clearly different orientations, "
            "sampling each. The quaternion in the feedback packet is ground "
            "truth; the Euler convention is solved against it."))
        sample_row = QHBoxLayout()
        sample_row.addWidget(self._sample_btn)
        sample_row.addWidget(self._solve_btn)
        sample_row.addWidget(self._clear_samples_btn)
        sample_row.addStretch(1)
        convention_layout.addLayout(sample_row)
        convention_layout.addWidget(self._sample_label)

        left = QVBoxLayout()
        left.addWidget(connection)
        left.addWidget(convention)
        left.addStretch(1)
        right = QVBoxLayout()
        right.addWidget(jog)
        right.addWidget(safety)
        right.addStretch(1)

        root = QHBoxLayout(self)
        left_panel = QWidget()
        left_panel.setLayout(left)
        right_panel = QWidget()
        right_panel.setLayout(right)
        root.addWidget(left_panel, 1)
        root.addWidget(right_panel, 1)

    def _jog_button(self, axis: str, sign: str) -> QPushButton:
        button = QPushButton("{}{}".format(axis, sign))
        button.setMaximumWidth(60)
        button.clicked.connect(
            lambda _checked=False, a=axis, s=sign: self._jog(a, s))
        # Kept so the whole pad can be disabled while a jog is in flight.
        self._jog_buttons.append(button)
        return button

    def _wire(self) -> None:
        self._connect_btn.clicked.connect(self._toggle_connection)
        self._enable_btn.clicked.connect(self._toggle_enable)
        self._clear_btn.clicked.connect(self._clear_error)
        self._ip.editingFinished.connect(self._on_ip)
        self._speed_factor.valueChanged.connect(self._on_speed_factor)
        self._jog_linear.valueChanged.connect(self._on_jog_steps)
        self._jog_angular.valueChanged.connect(self._on_jog_steps)
        self._tool_index.valueChanged.connect(self._on_frames)
        self._user_index.valueChanged.connect(self._on_frames)
        for box in (self._safety_enabled, self._confirm_each, self._check_box,
                    self._check_joints, self._require_enabled):
            box.toggled.connect(self._on_safety)
        for spin in (self._max_step_mm, self._max_step_deg, self._payload_kg):
            spin.valueChanged.connect(self._on_safety)
        self._sample_btn.clicked.connect(self._sample_convention)
        self._solve_btn.clicked.connect(self._solve_convention)
        self._clear_samples_btn.clicked.connect(self._clear_convention_samples)

    def _load_settings(self) -> None:
        settings = self._ctx.settings
        safety = settings.safety_config()
        self._speed_factor.setValue(int(safety.speed_factor_pct))
        self._jog_linear.setValue(float(settings.jog_linear_mm))
        self._jog_angular.setValue(float(settings.jog_angular_deg))
        self._tool_index.setValue(int(settings.tool_index))
        self._user_index.setValue(int(settings.user_index))
        self._safety_enabled.setChecked(safety.enabled)
        self._confirm_each.setChecked(safety.confirm_each_pose)
        self._check_box.setChecked(safety.check_workspace_box)
        self._check_joints.setChecked(safety.check_joint_limits)
        self._require_enabled.setChecked(safety.require_enabled_before_move)
        self._max_step_mm.setValue(float(safety.max_step_mm))
        self._max_step_deg.setValue(float(safety.max_step_deg))
        self._payload_kg.setValue(float(safety.payload_kg))

    # ----------------------------------------------------------- connection
    @Slot()
    def _toggle_connection(self) -> None:
        if self._ctx.robot_connected():
            api = self._ctx.api
            # Dropped from the context first: disconnect() blocks on both
            # command sockets and joins the feedback thread for up to 2 s, and
            # nothing else may reach for the arm while that is happening.
            self._ctx.api = None
            self._ctx.robot_changed.emit()
            self._connect_btn.setEnabled(False)
            self._set_status("Disconnecting" + "\u2026", WARN_COLOR)
            self._spawn(api.disconnect, slot=self._on_disconnected)
            return
        api = Nova5Api(self._ip.text().strip(),
                       self._ctx.settings.safety_config())
        self._pending_api = api
        self._set_status("Connecting" + "\u2026", WARN_COLOR)
        self._connect_btn.setEnabled(False)
        self._spawn(api.connect, slot=self._on_connected)

    @Slot(object)
    def _on_connected(self, result: Result) -> None:
        # A real slot rather than a lambda: this touches widgets, so it has to
        # be a queued delivery onto the UI thread. The api it belongs to is
        # held in _pending_api instead of being captured.
        api = self._pending_api
        self._pending_api = None
        self._connect_btn.setEnabled(True)
        if api is None:
            return
        if result.is_err:
            self._set_status(result.message, BAD_COLOR)
            self._ctx.log.error(result.message)
            return
        self._ctx.api = api
        self._ctx.robot_changed.emit()
        self._connect_btn.setText("Disconnect")
        self._set_status(result.message, OK_COLOR)
        self._ctx.log.success(result.message)
        # Blocking dashboard command: off the UI thread like every other one.
        self._spawn(api.set_speed_factor, int(self._speed_factor.value()))

    @Slot(object)
    def _on_disconnected(self, result: Result) -> None:
        self._connect_btn.setEnabled(True)
        self._connect_btn.setText("Connect")
        self._set_status("Disconnected.", IDLE_COLOR)
        self._report(result)

    @Slot()
    def _toggle_enable(self) -> None:
        if not self._require_api():
            return
        state = self._ctx.api.latest()
        enabled = state.is_ok and state.data.enabled
        action = self._ctx.api.disable if enabled else self._ctx.api.enable
        self._spawn(action)

    @Slot()
    def _clear_error(self) -> None:
        if not self._require_api():
            return
        self._spawn(self._ctx.api.clear_error)

    # ---------------------------------------------------------------- live
    @Slot()
    def _tick(self) -> None:
        if not self._ctx.robot_connected():
            return
        state = self._ctx.api.latest()
        if state.is_err:
            self._pose_label.setText(state.message)
            return
        s = state.data
        convention = self._ctx.settings.pose_convention()
        fmt = self._ctx.angle_format
        angles = [fmt.format_angle(np.radians(v)) if convention.degrees
                  else fmt.format_angle(v) for v in s.tcp[3:6]]
        self._pose_label.setText(
            "TCP  X {:+.2f}  Y {:+.2f}  Z {:+.2f} mm    "
            "Rx {}  Ry {}  Rz {}".format(
                s.tcp[0], s.tcp[1], s.tcp[2], *angles))
        self._joints_label.setText(
            "Joints  " + "  ".join(
                "J{} {:+7.2f}".format(i + 1, v)
                for i, v in enumerate(s.joints))
            + "    Tool {}  User {}".format(s.tool_index, s.user_index))

        disagreement = s.to_robot_pose().rotation_disagreement_deg(convention)
        flags = s.safety_flags()
        colour = OK_COLOR
        parts = [s.mode_name()]
        if s.enabled:
            parts.append("enabled")
        else:
            parts.append("disabled")
            colour = WARN_COLOR
        if s.error:
            parts.append("ERROR")
            colour = BAD_COLOR
        if s.collision:
            parts.append("COLLISION")
            colour = BAD_COLOR
        if flags:
            parts.append("safety: " + ", ".join(flags))
            if s.is_stopped_by_safety():
                colour = BAD_COLOR
        self._safety_label.setText(
            "<span style='color:{}'>{}</span>".format(colour, "  |  ".join(parts)))

        good, bad = self._ctx.api.packet_health()
        health = "packets {} good".format(good)
        if bad:
            health += ("  <span style='color:{}'>{} misaligned -- the "
                       "feedback stream is not trustworthy</span>".format(
                           BAD_COLOR, bad))
        if np.isfinite(disagreement):
            health += "  |  quaternion vs Euler: {:.3f} deg".format(disagreement)
        self._health_label.setText(health)
        self._enable_btn.setText("Disable" if s.enabled else "Enable")

    # ----------------------------------------------------------------- jog
    def _jog(self, axis: str, sign: str) -> None:
        if not self._require_api():
            return
        if self._jog_busy:
            # The pad is disabled while a jog is in flight; this is the
            # backstop. Two jogs in flight compute their absolute targets from
            # the same mid-motion feedback pose and one of the steps is lost.
            return
        state = self._ctx.api.latest()
        if state.is_err:
            self._ctx.log.warn(state.message)
            return
        if state.data.is_stopped_by_safety():
            self._ctx.log.error("Safety stop active; jog refused.")
            return
        direction = 1.0 if sign == "+" else -1.0
        safety = self._ctx.settings.safety_config()
        if axis.startswith("J"):
            index = int(axis[1]) - 1
            joints = list(state.data.joints)
            joints[index] += self._jog_angular.value() * direction
            # The Cartesian target of a joint move is not known without FK, so
            # the current TCP is passed: the workspace box then only confirms
            # where the arm already is, and the joint limits and the joint-step
            # limit are the gates that do the work.
            gate = check_move(state.data.tcp, state.data, safety, joints)
            call = (self._ctx.api.joint_move, (joints,))
        else:
            pose = list(state.data.tcp)
            index = JOG_AXES.index(axis)
            if index < 3:
                pose[index] += self._jog_linear.value() * direction
            else:
                convention = self._ctx.settings.pose_convention()
                if convention.rotvec:
                    # (rx, ry, rz) is a Rodrigues vector here, so adding a step
                    # to one component is not a rotation about that axis at
                    # all -- it changes the axis and the angle together.
                    self._ctx.log.error(
                        "This controller reports orientation as a rotation "
                        "vector, so an Rx/Ry/Rz jog would not be a rotation "
                        "about that axis. Jog the joints instead.")
                    return
                # The spin box is labelled in degrees; the controller's unit is
                # whatever the solved convention says it is.
                step = self._jog_angular.value() * direction
                pose[index] += (step if convention.degrees
                                else float(np.radians(step)))
            gate = check_move(pose, state.data, safety, None)
            call = (self._ctx.api.move_l, (pose, self._user_index.value(),
                                           self._tool_index.value()))
        # Jogging is a real motion, so it goes through the same gate as an
        # automated move. The gates do not stop applying because a human
        # pressed the button.
        if gate.is_err:
            self._ctx.log.error("Jog refused: {}".format(gate.message))
            return
        self._jog_busy = True
        self._set_jog_enabled(False)
        # Re-enabled on finished rather than on done, because finished arrives
        # even if the call never delivered a result -- the pad must not be able
        # to stay dead.
        self._spawn(call[0], *call[1], finished_slot=self._on_jog_finished)

    @Slot()
    def _on_jog_finished(self) -> None:
        self._jog_busy = False
        self._set_jog_enabled(True)

    def _set_jog_enabled(self, enabled: bool) -> None:
        for button in self._jog_buttons:
            button.setEnabled(enabled)

    # ---------------------------------------------------------- convention
    @Slot()
    def _sample_convention(self) -> None:
        if not self._require_api():
            return
        state = self._ctx.api.latest()
        if state.is_err:
            self._ctx.log.warn(state.message)
            return
        self._convention_samples.append(state.data.to_robot_pose())
        self._sample_label.setText(
            "{} samples".format(len(self._convention_samples)))
        self._ctx.log.info("Convention sample {} taken.".format(
            len(self._convention_samples)))

    @Slot()
    def _clear_convention_samples(self) -> None:
        self._convention_samples = []
        self._sample_label.setText("0 samples")

    @Slot()
    def _solve_convention(self) -> None:
        solved = pc.solve_convention(self._convention_samples)
        if solved.is_err:
            message = solved.message
            if solved.data:
                message += "\n\nBest candidates:\n" + "\n".join(
                    "  {:42s} max error {:.2f} deg".format(
                        c.label(), c.max_error_deg) for c in solved.data[:4])
            if "different frames" in solved.message:
                diagnosis = pc.diagnose_frame_mismatch(self._convention_samples)
                message += "\n\n" + (diagnosis.data if diagnosis.is_ok
                                     else diagnosis.message)
            QMessageBox.warning(self, "Pose convention", message)
            self._ctx.log.error(solved.message)
            return

        best = solved.data[0]
        answer = QMessageBox.question(
            self, "Pose convention",
            "Solved: {}\n\n{}\n\nAdopt this convention? Every transform in "
            "the app, and every automated move, will be built with it.".format(
                best.label(), best.convention.note),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._ctx.settings.set_pose_convention(best.convention)
        self._ctx.save_settings()
        self._update_convention_label()
        self._ctx.log.success("Pose convention set to {}.".format(best.label()))

    def _update_convention_label(self) -> None:
        convention = self._ctx.settings.pose_convention()
        if convention.verified:
            self._convention_label.setText(
                "<span style='color:{}'>{} {}</span><br>{}".format(
                    OK_COLOR, SYM_CHECK, convention.label(), convention.note))
        else:
            self._convention_label.setText(
                "<span style='color:{}'>{} unverified: assuming {}</span><br>"
                "Automated calibration runs stay disabled until this is "
                "solved.".format(WARN_COLOR, SYM_WARN, convention.label()))

    # ------------------------------------------------------------- settings
    @Slot()
    def _on_ip(self) -> None:
        self._ctx.settings.robot_ip = self._ip.text().strip()
        self._ctx.save_settings()

    @Slot()
    def _on_speed_factor(self) -> None:
        safety = self._ctx.settings.safety_config()
        safety.speed_factor_pct = int(self._speed_factor.value())
        # Jog and automated moves ride on the same global factor, so the
        # per-command caps track it rather than silently staying fast.
        safety.speed_l_pct = safety.speed_factor_pct
        safety.speed_j_pct = safety.speed_factor_pct
        self._ctx.settings.set_safety_config(safety)
        self._ctx.save_settings()
        if self._ctx.robot_connected():
            self._ctx.api.safety = safety
            # Debounced: holding the arrow emits a value per step, and each
            # command is a blocking round trip that queues behind the last.
            self._speed_timer.start()

    @Slot()
    def _send_speed_factor(self) -> None:
        if not self._ctx.robot_connected():
            return
        self._spawn(self._ctx.api.set_speed_factor,
                    int(self._speed_factor.value()))

    @Slot()
    def _on_jog_steps(self) -> None:
        self._ctx.settings.jog_linear_mm = float(self._jog_linear.value())
        self._ctx.settings.jog_angular_deg = float(self._jog_angular.value())
        self._ctx.save_settings()

    @Slot()
    def _on_frames(self) -> None:
        self._ctx.settings.tool_index = int(self._tool_index.value())
        self._ctx.settings.user_index = int(self._user_index.value())
        self._ctx.save_settings()

    @Slot()
    def _on_safety(self) -> None:
        safety = self._ctx.settings.safety_config()
        safety.enabled = self._safety_enabled.isChecked()
        safety.confirm_each_pose = self._confirm_each.isChecked()
        safety.check_workspace_box = self._check_box.isChecked()
        safety.check_joint_limits = self._check_joints.isChecked()
        safety.require_enabled_before_move = self._require_enabled.isChecked()
        safety.max_step_mm = float(self._max_step_mm.value())
        safety.max_step_deg = float(self._max_step_deg.value())
        safety.payload_kg = float(self._payload_kg.value())
        self._ctx.settings.set_safety_config(safety)
        self._ctx.save_settings()
        if self._ctx.robot_connected():
            self._ctx.api.safety = safety
        if not safety.enabled:
            self._ctx.log.warn(
                "Safety gates are OFF. Workspace, step-size and state checks "
                "are no longer applied to jogs or to automated moves.")

    # -------------------------------------------------------------- workers
    def _spawn(self, fn, *args, slot=None,
               finished_slot=None) -> CallableWorker:
        """Run one blocking API call off the UI thread.

        Every worker is tracked and reclaimed when it finishes: they are
        children of this tab, so without that a jogging session leaves one
        finished QThread per click alive until the window closes.
        """
        worker = CallableWorker(fn, *args, parent=self)
        worker.done.connect(self._report if slot is None else slot)
        if finished_slot is not None:
            worker.finished.connect(finished_slot)
        worker.finished.connect(self._on_worker_finished)
        self._workers.append(worker)
        worker.start()
        return worker

    @Slot()
    def _on_worker_finished(self) -> None:
        worker = self.sender()
        if worker in self._workers:
            self._workers.remove(worker)
        if worker is not None:
            worker.deleteLater()

    def stop_workers(self) -> None:
        """Join every in-flight command. Called before the window closes.

        A CallableWorker is a single blocking socket round trip with nothing to
        abort, so the cap is what bounds this: closing must not hang on a
        controller that has stopped answering, and a QThread must not still be
        in run() when the widget tree is destroyed.
        """
        self._speed_timer.stop()
        remaining = []
        for worker in list(self._workers):
            try:
                if worker.isRunning():
                    worker.wait(2000)
                if worker.isRunning():
                    # Still blocked on its socket. Keep it, so a second pass
                    # after the connection is closed -- which is what unblocks
                    # the read -- can still join it.
                    remaining.append(worker)
            except RuntimeError:
                # Already reclaimed by deleteLater; nothing left to join.
                pass
        self._workers = remaining

    # -------------------------------------------------------------- helpers
    def _require_api(self) -> bool:
        if self._ctx.robot_connected():
            return True
        QMessageBox.information(self, "Robot", "Connect to the arm first.")
        return False

    def _set_status(self, text: str, color: str) -> None:
        self._status.setText(
            "<span style='color:{}'>{}</span>".format(color, text))

    @Slot(object)
    def _report(self, result: Result) -> None:
        if result is None:
            return
        if result.is_err:
            self._ctx.log.error(result.message)
        elif result.message:
            self._ctx.log.info(result.message)
