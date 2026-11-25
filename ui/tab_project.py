# -*- coding: utf-8 -*-
"""
ui/tab_project.py
-----------------
Authoring the reusable procedure.

A project is one home pose -- the one with the board comfortably centred --
plus however many further poses the operator teaches by jogging. Each pose
stores the arm transform, the board-to-camera view it saw, the detected corner
pixels, and the raw image.

Two numbers on this tab decide whether the project is any good:

  ROTATION DIVERSITY. If every pose rotates about one axis, translation along
  that axis is unobservable and no residual metric will reveal it. The gate
  runs live so the problem is visible while poses are being added, not after
  a solve fails.

  REMOUNT MARGIN. How far the camera can move on the flange before each pose
  loses the board. Poses taught with the board filling the frame look great
  and survive nothing; the report names the weakest pose so it can be redone.
"""

from __future__ import annotations

import os
from typing import Optional

import numpy as np
from PySide6.QtCore import QTimer, Slot
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

import handeye
import project_io
import remount
from constants import (
    BAD_COLOR,
    LIVE_VIEW_HZ,
    OK_COLOR,
    SYM_HOME,
    WARN_COLOR,
)
from context import AppContext
from models import CalibPose, CalibProject, CharucoBoardConfig
from qt_helpers import ImageLabel, make_hline

ARUCO_DICTIONARIES = (
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_5X5_50",
    "DICT_5X5_100", "DICT_5X5_250", "DICT_6X6_50", "DICT_6X6_100",
    "DICT_6X6_250", "DICT_7X7_50", "DICT_APRILTAG_36h11",
)


def _convention_geometry(convention) -> tuple:
    """The fields of a PoseConvention that change what a raw pose MEANS.

    'verified' and 'note' are bookkeeping: they say how much the convention is
    trusted, not how the six controller numbers turn into a rotation, so a
    change in either does not invalidate a cached T_mount2base.
    """
    return (convention.order, bool(convention.intrinsic),
            bool(convention.degrees), bool(convention.rotvec),
            float(convention.translation_scale), convention.quat_order)


class ProjectTab(QWidget):
    """Create, edit and inspect auto-calibration projects."""

    def __init__(self, ctx: AppContext, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._ctx = ctx

        self._project_combo = QComboBox()
        self._new_btn = QPushButton("New")
        self._delete_btn = QPushButton("Delete")
        self._save_btn = QPushButton("Save")

        self._squares_x = QSpinBox()
        self._squares_y = QSpinBox()
        self._square_len = QDoubleSpinBox()
        self._marker_len = QDoubleSpinBox()
        self._dictionary = QComboBox()
        self._min_distance = QDoubleSpinBox()
        self._max_distance = QDoubleSpinBox()

        self._pose_list = QListWidget()
        self._add_home_btn = QPushButton(SYM_HOME + " Set home from current")
        self._add_pose_btn = QPushButton("Add pose from current")
        self._remove_pose_btn = QPushButton("Remove")
        self._retake_btn = QPushButton("Retake image")

        self._view = ImageLabel()
        self._live_label = QLabel("-")
        self._diversity_label = QLabel("-")
        self._margin_label = QLabel("-")

        self._build()
        self._wire()
        self._load_board_settings()
        self.refresh_projects()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / LIVE_VIEW_HZ))

    # --------------------------------------------------------------- layout
    def _build(self) -> None:
        project_box = QGroupBox("Project")
        project_row = QHBoxLayout()
        self._project_combo.setMinimumWidth(220)
        project_row.addWidget(self._project_combo, 1)
        project_row.addWidget(self._new_btn)
        project_row.addWidget(self._delete_btn)
        project_row.addWidget(self._save_btn)
        project_layout = QVBoxLayout(project_box)
        project_layout.addLayout(project_row)

        board_box = QGroupBox("Board")
        board_layout = QVBoxLayout(board_box)
        counts = QHBoxLayout()
        counts.addWidget(QLabel("Squares X:"))
        self._squares_x.setRange(2, 40)
        counts.addWidget(self._squares_x)
        counts.addWidget(QLabel("Y:"))
        self._squares_y.setRange(2, 40)
        counts.addWidget(self._squares_y)
        counts.addWidget(QLabel("Dictionary:"))
        for name in ARUCO_DICTIONARIES:
            self._dictionary.addItem(name)
        counts.addWidget(self._dictionary)
        counts.addStretch(1)
        board_layout.addLayout(counts)
        sizes = QHBoxLayout()
        sizes.addWidget(QLabel("Square m:"))
        for spin in (self._square_len, self._marker_len):
            spin.setRange(0.001, 1.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.001)
        sizes.addWidget(self._square_len)
        sizes.addWidget(QLabel("Marker m:"))
        sizes.addWidget(self._marker_len)
        sizes.addStretch(1)
        board_layout.addLayout(sizes)
        distances = QHBoxLayout()
        distances.addWidget(QLabel("Allowed distance m:"))
        for spin in (self._min_distance, self._max_distance):
            spin.setRange(0.02, 5.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
        distances.addWidget(self._min_distance)
        distances.addWidget(QLabel("to"))
        distances.addWidget(self._max_distance)
        distances.addStretch(1)
        board_layout.addLayout(distances)

        poses_box = QGroupBox("Poses")
        poses_layout = QVBoxLayout(poses_box)
        poses_layout.addWidget(self._pose_list, 1)
        buttons = QHBoxLayout()
        buttons.addWidget(self._add_home_btn)
        buttons.addWidget(self._add_pose_btn)
        buttons.addWidget(self._retake_btn)
        buttons.addWidget(self._remove_pose_btn)
        poses_layout.addLayout(buttons)
        self._diversity_label.setWordWrap(True)
        self._margin_label.setWordWrap(True)
        poses_layout.addWidget(make_hline())
        poses_layout.addWidget(self._diversity_label)
        poses_layout.addWidget(self._margin_label)

        left = QVBoxLayout()
        left.addWidget(project_box)
        left.addWidget(board_box)
        left.addWidget(poses_box, 1)

        right = QVBoxLayout()
        right.addWidget(self._view, 1)
        self._live_label.setWordWrap(True)
        right.addWidget(self._live_label)

        root = QHBoxLayout(self)
        panel = QWidget()
        panel.setLayout(left)
        panel.setMaximumWidth(620)
        root.addWidget(panel)
        root.addLayout(right, 1)

    def _wire(self) -> None:
        self._project_combo.currentIndexChanged.connect(self._on_project_selected)
        self._new_btn.clicked.connect(self._new_project)
        self._delete_btn.clicked.connect(self._delete_project)
        self._save_btn.clicked.connect(self._save_project)
        self._add_home_btn.clicked.connect(lambda: self._add_pose(home=True))
        self._add_pose_btn.clicked.connect(lambda: self._add_pose(home=False))
        self._remove_pose_btn.clicked.connect(self._remove_pose)
        self._retake_btn.clicked.connect(self._retake_image)
        for spin in (self._squares_x, self._squares_y):
            spin.valueChanged.connect(self._on_board_changed)
        for spin in (self._square_len, self._marker_len):
            spin.valueChanged.connect(self._on_board_changed)
        self._dictionary.currentIndexChanged.connect(self._on_board_changed)
        for spin in (self._min_distance, self._max_distance):
            spin.valueChanged.connect(self._on_distance_changed)

    def _load_board_settings(self) -> None:
        board = self._ctx.settings.board_config()
        for widget, value in ((self._squares_x, board.squares_x),
                              (self._squares_y, board.squares_y)):
            widget.blockSignals(True)
            widget.setValue(int(value))
            widget.blockSignals(False)
        for widget, value in ((self._square_len, board.square_length_m),
                              (self._marker_len, board.marker_length_m)):
            widget.blockSignals(True)
            widget.setValue(float(value))
            widget.blockSignals(False)
        row = self._dictionary.findText(board.dictionary)
        self._dictionary.blockSignals(True)
        self._dictionary.setCurrentIndex(max(0, row))
        self._dictionary.blockSignals(False)
        for widget, value in ((self._min_distance,
                               self._ctx.settings.min_distance_m),
                              (self._max_distance,
                               self._ctx.settings.max_distance_m)):
            widget.blockSignals(True)
            widget.setValue(float(value))
            widget.blockSignals(False)

    # -------------------------------------------------------------- projects
    @Slot()
    def refresh_projects(self) -> None:
        names = project_io.list_projects(self._ctx.settings.root_dir)
        self._project_combo.blockSignals(True)
        self._project_combo.clear()
        self._project_combo.addItem("(none)", "")
        for name in names:
            self._project_combo.addItem(name, name)
        self._project_combo.blockSignals(False)
        wanted = self._ctx.settings.last_project
        row = self._project_combo.findData(wanted)
        self._project_combo.setCurrentIndex(row if row >= 0 else 0)
        self._on_project_selected()

    @Slot()
    def _on_project_selected(self) -> None:
        name = self._project_combo.currentData()
        if not name:
            self._ctx.project = None
            self._ctx.project_changed.emit()
            self._refresh_pose_list()
            return
        loaded = project_io.load_project(self._ctx.settings.root_dir, name)
        if loaded.is_err:
            self._ctx.log.error(loaded.message)
            return
        self._ctx.project = loaded.data
        self._ctx.settings.last_project = name
        # Adopt the project's board: a project captured with one board and
        # replayed against another is silently wrong in the scale of every
        # translation.
        self._ctx.settings.set_board_config(loaded.data.board)
        self._ctx.settings.min_distance_m = loaded.data.min_distance_m
        self._ctx.settings.max_distance_m = loaded.data.max_distance_m
        self._ctx.save_settings()
        self._ctx.rebuild_board()
        self._load_board_settings()
        self._ctx.project_changed.emit()
        self._refresh_pose_list()
        self._ctx.log.info("Loaded project '{}' with {} poses.".format(
            name, len(loaded.data.poses)))

    @Slot()
    def _new_project(self) -> None:
        name, ok = QInputDialog.getText(self, "New project", "Project name:")
        if not ok or not name.strip():
            return
        typed = name.strip()
        # sanitize_name() is many-to-one, so "cell A", "cell.A" and "cell/A"
        # all land in the same directory. Saving a fresh project over it would
        # replace every authored pose with an empty list.
        safe = project_io.sanitize_name(typed)
        paths = project_io.ProjectPaths(self._ctx.settings.root_dir, typed)
        if os.path.exists(paths.json_path):
            note = ("" if safe == typed
                    else " ('{}' is stored as '{}')".format(typed, safe))
            answer = QMessageBox.question(
                self, "New project",
                "A project already exists in '{}'{}. Creating a new project "
                "here replaces its poses, reference views and arm transforms. "
                "This cannot be undone.\n\nContinue?".format(safe, note),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return
        project = CalibProject(
            name=typed,
            board=self._ctx.settings.board_config(),
            camera=self._ctx.settings.camera_config(),
            convention=self._ctx.settings.pose_convention(),
            min_distance_m=float(self._min_distance.value()),
            max_distance_m=float(self._max_distance.value()))
        saved = project_io.save_project(self._ctx.settings.root_dir, project)
        if saved.is_err:
            self._ctx.log.error(saved.message)
            return
        self._ctx.settings.last_project = project_io.sanitize_name(project.name)
        self._ctx.save_settings()
        self.refresh_projects()

    @Slot()
    def _delete_project(self) -> None:
        name = self._project_combo.currentData()
        if not name:
            return
        answer = QMessageBox.question(
            self, "Delete project",
            "Delete project '{}' and every instance captured under it? This "
            "cannot be undone.".format(name),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if answer != QMessageBox.StandardButton.Yes:
            return
        removed = project_io.delete_project(self._ctx.settings.root_dir, name)
        if removed.is_err:
            self._ctx.log.error(removed.message)
            return
        self._ctx.settings.last_project = ""
        self._ctx.save_settings()
        self.refresh_projects()

    @Slot()
    def _save_project(self) -> None:
        if self._ctx.project is None:
            return
        self._ctx.project.board = self._ctx.settings.board_config()
        self._ctx.project.min_distance_m = float(self._min_distance.value())
        self._ctx.project.max_distance_m = float(self._max_distance.value())
        self._stamp_convention()
        saved = project_io.save_project(self._ctx.settings.root_dir,
                                        self._ctx.project)
        if saved.is_err:
            self._ctx.log.error(saved.message)
            return
        self._ctx.log.success("Project saved to {}".format(saved.data))

    def _stamp_convention(self) -> None:
        """Adopt the current pose convention, re-deriving what it invalidates.

        T_mount2base is a cached derivative of the raw controller reading, so
        recording a new convention beside the old matrices would make the file
        describe rotations it no longer means -- and the replay would command
        them. Every pose that kept its raw reading is re-derived; a pose that
        did not cannot be, so the project keeps its original convention rather
        than becoming half one and half the other.
        """
        convention = self._ctx.settings.pose_convention()
        stored = self._ctx.project.convention
        if _convention_geometry(convention) == _convention_geometry(stored):
            self._ctx.project.convention = convention
            return
        orphans = [pose.name or "pose {}".format(pose.index)
                   for pose in self._ctx.project.poses
                   if pose.pose is None and pose.T_mount2base is not None]
        if orphans:
            QMessageBox.warning(
                self, "Save project",
                "The pose convention is now {}, but {} pose(s) store no raw "
                "controller reading, so their transforms cannot be "
                "re-derived: {}.\n\nThe project keeps its original convention "
                "({}). Retake those poses, or set the convention back before "
                "saving.".format(convention.label(), len(orphans),
                                 ", ".join(orphans), stored.label()))
            return
        derived = 0
        for pose in self._ctx.project.poses:
            if pose.pose is not None:
                pose.T_mount2base = pose.pose.to_se3(convention)
                derived += 1
        self._ctx.project.convention = convention
        if derived:
            self._ctx.log.info(
                "Pose convention changed to {}: re-derived {} pose "
                "transform(s) from their raw readings.".format(
                    convention.label(), derived))

    # ----------------------------------------------------------------- poses
    def _capture_pose(self, home: bool, index: int,
                      name: str) -> Optional[tuple]:
        """Build one CalibPose from what the camera and the arm report now.

        Returns (pose, raw_bgr, overlay_bgr), or None when any gate failed --
        every failure has already been shown to the operator. Nothing is
        appended, written or saved here, so a caller can capture first and
        only then touch the project.
        """
        if not self._ctx.camera_connected():
            QMessageBox.information(self, "Project", "Connect a camera first.")
            return None
        intrinsics = self._ctx.intrinsics()
        if intrinsics.is_err:
            QMessageBox.warning(self, "Project", intrinsics.message)
            return None
        frame = self._ctx.camera.get_color()
        if frame.is_err:
            QMessageBox.warning(self, "Project", frame.message)
            return None
        det = handeye.detect_board(
            frame.data, self._ctx.board, self._ctx.detector, intrinsics.data,
            min_coverage=self._ctx.settings.min_image_coverage)
        if det.T_target2cam is None:
            QMessageBox.warning(
                self, "Project",
                "No usable board pose: {}".format(det.message or "not detected"))
            return None
        band = remount.check_distance_band(
            det.T_target2cam, float(self._min_distance.value()),
            float(self._max_distance.value()))
        if band.is_err:
            answer = QMessageBox.question(
                self, "Project",
                band.message + "\n\nAdd the pose anyway?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return None

        pose = CalibPose(
            index=index,
            name=name,
            is_home=home,
            T_board2cam_ref=det.T_target2cam,
            ref_corners=det.charuco_corners,
            ref_ids=det.charuco_ids,
            n_corners=det.n_corners,
            reproj_error=det.reproj_error,
            board_distance_m=float(np.linalg.norm(det.T_target2cam[:3, 3])),
            margin_deg=remount.view_margin_deg(
                det.charuco_corners, intrinsics.data.width,
                intrinsics.data.height, intrinsics.data))

        if self._ctx.robot_connected():
            state = self._ctx.api.latest()
            if state.is_err:
                QMessageBox.warning(self, "Project", state.message)
                return None
            convention = self._ctx.settings.pose_convention()
            pose.pose = state.data.to_robot_pose()
            pose.T_mount2base = pose.pose.to_se3(convention)
            self._ctx.project.frames = state.data.frame_identity()
        else:
            self._ctx.log.warn(
                "No robot connected: pose stored with imagery only. Fill in "
                "the arm pose before this project can be replayed.")

        overlay = handeye.draw_overlay(frame.data, det, intrinsics.data)
        return (pose, frame.data, overlay)

    def _free_image_index(self) -> int:
        """The lowest pose_NNN slot that no file and no pose already owns.

        Removing a pose renumbers the survivors but leaves their PNGs and
        their stored paths alone, so the next authored pose's index is not
        free -- writing to it would overwrite another pose's reference view.
        """
        paths = project_io.ProjectPaths(self._ctx.settings.root_dir,
                                        self._ctx.project.name)
        claimed = set()
        for existing in self._ctx.project.poses:
            for path in (existing.image_path, existing.overlay_path):
                if path:
                    claimed.add(os.path.normpath(path))
        index = 0
        while True:
            candidates = (paths.pose_image(index, overlay=False),
                          paths.pose_image(index, overlay=True))
            free = True
            for candidate in candidates:
                if (os.path.exists(candidate)
                        or os.path.normpath(candidate) in claimed):
                    free = False
                    break
            if free:
                return index
            index += 1

    def _store_pose_images(self, pose: CalibPose, raw_bgr,
                           overlay_bgr) -> None:
        """Write a pose's reference imagery.

        A retake overwrites the pose's own files, which keeps project.json's
        paths valid and touches nothing else. A new pose gets a free slot.
        """
        if pose.image_path and pose.overlay_path:
            for path, image in ((pose.image_path, raw_bgr),
                                (pose.overlay_path, overlay_bgr)):
                if image is None:
                    continue
                written = project_io.save_image(path, image)
                if written.is_err:
                    self._ctx.log.error(written.message)
            return
        saved = project_io.save_pose_images(
            self._ctx.settings.root_dir, self._ctx.project.name,
            self._free_image_index(), raw_bgr, overlay_bgr)
        if saved.is_ok:
            pose.image_path, pose.overlay_path = saved.data
        else:
            self._ctx.log.error(saved.message)

    def _add_pose(self, home: bool) -> None:
        if self._ctx.project is None:
            QMessageBox.information(self, "Project",
                                    "Create or select a project first.")
            return
        count = len(self._ctx.project.poses)
        captured = self._capture_pose(
            home, count, "home" if home else "pose {}".format(count))
        if captured is None:
            return
        pose, raw, overlay = captured

        if home:
            for existing in self._ctx.project.poses:
                existing.is_home = False
        self._ctx.project.poses.append(pose)
        self._ctx.project.renumber()

        self._store_pose_images(pose, raw, overlay)
        self._save_project()
        self._refresh_pose_list()

    @Slot()
    def _remove_pose(self) -> None:
        row = self._pose_list.currentRow()
        if self._ctx.project is None or row < 0:
            return
        del self._ctx.project.poses[row]
        self._ctx.project.renumber()
        self._save_project()
        self._refresh_pose_list()

    @Slot()
    def _retake_image(self) -> None:
        row = self._pose_list.currentRow()
        if (self._ctx.project is None or row < 0
                or row >= len(self._ctx.project.poses)):
            return
        old = self._ctx.project.poses[row]
        # Capture FIRST. A retake that cannot see the board must leave the
        # authored pose -- its reference view, its corners and its arm
        # transform -- exactly as it was.
        captured = self._capture_pose(old.is_home, old.index, old.name)
        if captured is None:
            return
        pose, raw, overlay = captured
        # The replacement takes the original row, so the authored replay order
        # is preserved, and its own image files, so nothing else is disturbed.
        pose.image_path = old.image_path
        pose.overlay_path = old.overlay_path
        self._ctx.project.poses[row] = pose
        self._ctx.project.renumber()
        self._store_pose_images(pose, raw, overlay)
        self._save_project()
        self._refresh_pose_list()

    def _refresh_pose_list(self) -> None:
        self._pose_list.clear()
        if self._ctx.project is None:
            self._diversity_label.setText("-")
            self._margin_label.setText("-")
            return
        for pose in self._ctx.project.poses:
            marker = SYM_HOME + " " if pose.is_home else "   "
            detail = "{} corners, reproj {:.2f} px, {:.3f} m, margin {:.1f} deg".format(
                pose.n_corners, pose.reproj_error, pose.board_distance_m,
                pose.margin_deg)
            if pose.T_mount2base is None:
                detail += "   [no arm pose]"
            self._pose_list.addItem(QListWidgetItem(
                "{}{}: {}".format(marker, pose.name, detail)))
        self._update_quality_labels()

    def _update_quality_labels(self) -> None:
        poses = [p for p in self._ctx.project.poses
                 if p.T_mount2base is not None]
        if len(poses) < 2:
            self._diversity_label.setText(
                "Add at least five poses with rotation about more than one "
                "axis.")
        else:
            diversity = handeye.check_rotation_diversity(
                [p.T_mount2base for p in poses])
            colour = OK_COLOR if diversity.is_ok else BAD_COLOR
            text = ("rotation diversity OK" if diversity.is_ok
                    else diversity.message)
            self._diversity_label.setText(
                "<span style='color:{}'>{} pose(s) with transforms -- "
                "{}</span>".format(colour, len(poses), text))
        margins = [p.margin_deg for p in self._ctx.project.poses]
        if margins:
            self._margin_label.setText(remount.project_margin_report(margins))

    # ---------------------------------------------------------------- shutdown
    def stop_workers(self) -> None:
        """Stop the live tick. Never raises.

        This tab owns no worker threads -- authoring is all UI-thread work
        against the latest feedback packet -- but the close path calls this on
        every tab, and a timer still firing into a half-torn-down widget tree
        is its own kind of crash.
        """
        self._timer.stop()

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
        if intrinsics.is_err:
            self._live_label.setText(intrinsics.message)
            return
        if det.T_target2cam is None:
            self._live_label.setText(det.message or "No board detected.")
            return
        distance = float(np.linalg.norm(det.T_target2cam[:3, 3]))
        margin = remount.view_margin_deg(det.charuco_corners,
                                         intrinsics.data.width,
                                         intrinsics.data.height,
                                         intrinsics.data)
        band = remount.check_distance_band(det.T_target2cam,
                                           float(self._min_distance.value()),
                                           float(self._max_distance.value()))
        colour = OK_COLOR if band.is_ok else WARN_COLOR
        self._live_label.setText(
            "<span style='color:{}'>{} corners  |  reproj {:.2f} px  |  "
            "{:.3f} m  |  remount margin {:.1f} deg{}</span>".format(
                colour, det.n_corners, det.reproj_error, distance, margin,
                "" if band.is_ok else "  |  " + band.message))

    # -------------------------------------------------------------- settings
    @Slot()
    def _on_board_changed(self) -> None:
        board = CharucoBoardConfig(
            squares_x=int(self._squares_x.value()),
            squares_y=int(self._squares_y.value()),
            square_length_m=float(self._square_len.value()),
            marker_length_m=float(self._marker_len.value()),
            dictionary=self._dictionary.currentText())
        if board.marker_length_m >= board.square_length_m:
            self._ctx.log.warn(
                "Marker length must be smaller than the square length; the "
                "board will not detect.")
        self._ctx.settings.set_board_config(board)
        self._ctx.save_settings()
        rebuilt = self._ctx.rebuild_board()
        if rebuilt.is_err:
            self._ctx.log.error(rebuilt.message)

    @Slot()
    def _on_distance_changed(self) -> None:
        self._ctx.settings.min_distance_m = float(self._min_distance.value())
        self._ctx.settings.max_distance_m = float(self._max_distance.value())
        self._ctx.save_settings()
