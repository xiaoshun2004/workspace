# -*- coding: utf-8 -*-
"""
ui/main_window.py
-----------------
Tabs, a log that is always visible, and a STOP button that is always reachable.

STOP aborts a running automated calibration and sends ResetRobot(), which
halts motion and clears the queue. Both halves are needed: halting the arm
without aborting the run only means the run commands the next pose as soon as
the arm reports idle. It is not a substitute for the physical e-stop and the
tooltip says so: a software stop travels over the same TCP socket as
everything else, and a socket that has stopped answering is exactly when you
want the stop to work.
"""

from __future__ import annotations

from typing import List, Optional

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QDockWidget,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from app_log import LogView
from constants import (
    APP_NAME,
    APP_VERSION,
    BAD_COLOR,
    IDLE_COLOR,
    OK_COLOR,
    STOP_QSS,
    SYM_DOT,
    WARN_COLOR,
)
from context import AppContext
from tab_calibrate import CalibrateTab
from tab_camera import CameraTab
from tab_project import ProjectTab
from tab_robot import RobotTab
from tab_verify import VerifyTab
from workers import CallableWorker


def _join_worker(worker) -> bool:
    """Wait up to 2 s for one worker, returning True if it is still running.

    Never raises: a worker whose C++ object has already been reclaimed by
    deleteLater is gone, which is the outcome the join wanted anyway.
    """
    try:
        if worker.isRunning():
            worker.wait(2000)
        return bool(worker.isRunning())
    except RuntimeError:
        return False


class MainWindow(QMainWindow):
    def __init__(self, ctx: AppContext, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._ctx = ctx
        self._stop_workers: List[CallableWorker] = []
        self.setWindowTitle("{} {}".format(APP_NAME, APP_VERSION))
        self.resize(ctx.settings.window_width, ctx.settings.window_height)

        self._camera_dot = QLabel(SYM_DOT + " camera")
        self._robot_dot = QLabel(SYM_DOT + " robot")
        self._convention_dot = QLabel(SYM_DOT + " convention")
        self._stop_btn = QPushButton("STOP")
        self._stop_btn.setStyleSheet(STOP_QSS)
        self._stop_btn.setToolTip(
            "Aborts any automated run and sends ResetRobot(): halts motion "
            "and clears the queue. This travels over the same TCP socket as "
            "every other command -- it is not a replacement for the physical "
            "e-stop.")
        self._stop_btn.clicked.connect(self._stop)

        self.camera_tab = CameraTab(ctx)
        self.robot_tab = RobotTab(ctx)
        self.project_tab = ProjectTab(ctx)
        self.calibrate_tab = CalibrateTab(ctx)
        self.verify_tab = VerifyTab(ctx)

        tabs = QTabWidget()
        tabs.addTab(self.camera_tab, "Camera")
        tabs.addTab(self.robot_tab, "Robot")
        tabs.addTab(self.project_tab, "Projects")
        tabs.addTab(self.calibrate_tab, "Calibrate")
        tabs.addTab(self.verify_tab, "Verify")
        tabs.currentChanged.connect(self._on_tab_changed)
        self._tabs = tabs

        header = QHBoxLayout()
        for dot in (self._camera_dot, self._robot_dot, self._convention_dot):
            header.addWidget(dot)
        header.addStretch(1)
        header.addWidget(self._stop_btn)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addLayout(header)
        layout.addWidget(tabs, 1)
        self.setCentralWidget(central)

        dock = QDockWidget("Log", self)
        dock.setWidget(LogView(ctx.log))
        dock.setAllowedAreas(Qt.DockWidgetArea.BottomDockWidgetArea
                             | Qt.DockWidgetArea.RightDockWidgetArea)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start(300)

        ctx.log.info("{} {} started. Settings: {}".format(
            APP_NAME, APP_VERSION, ctx.store.path))

    @Slot(int)
    def _on_tab_changed(self, index: int) -> None:
        widget = self._tabs.widget(index)
        # Projects and instances are created on other tabs, so their lists
        # would otherwise go stale until a restart.
        if widget is self.verify_tab:
            self.verify_tab.refresh_projects()
        elif widget is self.project_tab:
            self.project_tab.refresh_projects()

    @Slot()
    def _stop(self) -> None:
        # The abort comes first and is only a flag set, so it lands even while
        # a per-pose confirm dialog owns the event loop. Without it the arm
        # halts and the run then commands the next pose seconds later.
        self.calibrate_tab.abort_run()
        if self._ctx.api is None:
            self._ctx.log.warn("Run aborted. No arm connection exists, so no "
                               "stop was sent to the controller.")
            return
        # Deliberately not gated on robot_connected(): that is False as soon as
        # the feedback thread dies, and a dead feedback thread must not be what
        # disables STOP -- ResetRobot travels on the dashboard socket, which is
        # a different socket. Sent from a worker because it is a blocking round
        # trip and the UI thread is what has to stay responsive here.
        worker = CallableWorker(self._ctx.api.stop, parent=self)
        worker.done.connect(self._on_stop_result)
        worker.finished.connect(self._on_stop_worker_finished)
        self._stop_workers.append(worker)
        worker.start()
        self._ctx.log.warn("STOP: run aborted; sending ResetRobot() to the "
                           "controller.")

    @Slot(object)
    def _on_stop_result(self, result) -> None:
        if result is None:
            return
        if result.is_err:
            self._ctx.log.error(
                "STOP was not confirmed by the controller: {}".format(
                    result.message))
            return
        self._ctx.log.warn("STOP confirmed: motion halted and the queue "
                           "cleared.")

    @Slot()
    def _on_stop_worker_finished(self) -> None:
        worker = self.sender()
        if worker in self._stop_workers:
            self._stop_workers.remove(worker)
        if worker is not None:
            worker.deleteLater()

    @Slot()
    def _refresh_status(self) -> None:
        self._set_dot(self._camera_dot, "camera",
                      self._ctx.camera_connected())
        connected = self._ctx.robot_connected()
        colour = OK_COLOR if connected else IDLE_COLOR
        text = "robot"
        if connected:
            state = self._ctx.api.latest()
            if state.is_ok:
                if state.data.is_stopped_by_safety():
                    colour = BAD_COLOR
                    text = "robot: " + ", ".join(state.data.safety_flags())
                elif state.data.error:
                    colour = BAD_COLOR
                    text = "robot: error"
                elif not state.data.enabled:
                    colour = WARN_COLOR
                    text = "robot: disabled"
                else:
                    text = "robot: " + state.data.mode_name()
            else:
                colour = WARN_COLOR
                text = "robot: no feedback"
        self._robot_dot.setText(
            "<span style='color:{}'>{} {}</span>".format(colour, SYM_DOT, text))

        verified = self._ctx.settings.pose_convention().verified
        self._convention_dot.setText(
            "<span style='color:{}'>{} convention {}</span>".format(
                OK_COLOR if verified else WARN_COLOR, SYM_DOT,
                "verified" if verified else "UNVERIFIED"))

    def _set_dot(self, label: QLabel, name: str, ok: bool) -> None:
        label.setText("<span style='color:{}'>{} {}</span>".format(
            OK_COLOR if ok else IDLE_COLOR, SYM_DOT, name))

    def closeEvent(self, event: QCloseEvent) -> None:
        self._ctx.settings.window_width = self.width()
        self._ctx.settings.window_height = self.height()
        # Order matters. Every worker is aborted and joined BEFORE the sockets
        # close, because ctx.shutdown() would otherwise pull the camera and the
        # arm out from under a thread that is still using them, and the widget
        # tree is destroyed moments later -- a QThread still in run() at that
        # point aborts the process.
        self._status_timer.stop()
        tabs = (self.robot_tab, self.camera_tab, self.project_tab,
                self.calibrate_tab, self.verify_tab)
        for tab in tabs:
            tab.stop_workers()
        self._join_stop_workers()
        if self._ctx.api is not None and self._arm_may_be_moving():
            # The controller keeps executing whatever is queued after this
            # process is gone, so clear it. Blocking is acceptable here: the
            # workers are joined, there is no UI left to freeze, and the socket
            # timeout bounds the wait.
            result = self._ctx.api.stop()
            if result.is_err:
                self._ctx.log.error(
                    "Could not clear the motion queue on close: {}".format(
                        result.message))
            else:
                self._ctx.log.warn("Motion queue cleared on close.")
        self._ctx.shutdown()
        # Second pass: shutdown() closes the sockets, and closing one under a
        # blocked recv is what unblocks it. A worker the first pass could not
        # join finishes now, which is still before the widget tree -- and its
        # QThread objects -- is destroyed.
        for tab in tabs:
            tab.stop_workers()
        self._join_stop_workers()
        super().closeEvent(event)

    def _join_stop_workers(self) -> None:
        """Join the STOP workers, keeping any that are still blocked."""
        self._stop_workers = [worker for worker in list(self._stop_workers)
                              if _join_worker(worker)]

    def _arm_may_be_moving(self) -> bool:
        """True unless the feedback stream positively says the arm is idle.

        Closing on a stale or dead feedback stream is exactly when a queued
        move is most likely to be left running, so 'cannot tell' counts as
        moving.
        """
        state = self._ctx.api.latest()
        return state.is_err or state.data.running
