# -*- coding: utf-8 -*-
"""
ui/tab_calibrate.py
-------------------
Where an instance is captured and solved.

Three capture modes:

  AUTO         replay an authored project. Anchor once by hand, then the
               worker walks the remaining poses, correcting each by the
               measured remount delta.
  MANUAL       jog wherever you like and capture when the gates are green.
  IMAGES ONLY  no arm at all. Capture frames now, type the poses in later --
               useful when the arm is being jogged from DobotStudio.

THE ANCHOR STEP, AND WHY IT COMES FIRST

Remounting the camera shifts every stored view by one fixed transform in
camera frame. The correction is a flange-frame right multiplication,
G_i' = G_i . delta, and delta is measurable by re-teaching a single pose:
jog until the ghosted reference corners sit on the live ones, then read the
arm pose back. That one measurement also converts the previous instance's
T_cam2mount into the current one, which is what lets the per-pose servo work
at all -- it is only a contraction near the truth.

Every pose reached afterwards yields its own estimate of the same delta, and
the SPREAD of those estimates is the diagnostic: tight means a clean remount,
wide means the board moved or the pose convention is wrong.
"""

from __future__ import annotations

import os
from typing import List, Optional

import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

import handeye
import project_io
import remount
from constants import (
    BAD_COLOR,
    DANGER_QSS,
    LIVE_VIEW_HZ,
    OK_COLOR,
    SYM_PLAY,
    SYM_STOP,
    WARN_COLOR,
)
from context import AppContext
from models import (
    CalibInstance,
    CalibPose,
    CaptureMode,
    IntrinsicsSource,
    Result,
    RobotPose,
)
from qt_helpers import ImageLabel
from workers import (
    AnchorWorker,
    AutoCalibWorker,
    CaptureGates,
    capture_sample,
    gate_still_image,
)

MODE_LABELS = {
    CaptureMode.AUTO: "Auto (replay a project)",
    CaptureMode.MANUAL: "Manual (jog and capture)",
    CaptureMode.IMAGES_ONLY: "Images only (no robot)",
}


class CalibrateTab(QWidget):
    """Capture an instance, solve it, and save it."""

    def __init__(self, ctx: AppContext, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._ctx = ctx
        self._samples: List[handeye.HandEyeSample] = []
        self._images: List[tuple] = []          # (raw, overlay) per sample
        self._estimates: List[remount.DeltaEstimate] = []
        # The anchor measurements taken for the CURRENT mount, kept apart from
        # _estimates: a run must start from these and nothing else, or its
        # remount record mixes two mounts' evidence.
        self._anchor_estimates: List[remount.DeltaEstimate] = []
        self._delta: Optional[np.ndarray] = None
        self._X_prev: Optional[np.ndarray] = None
        self._result = None
        self._paths: Optional[project_io.InstancePaths] = None
        self._worker: Optional[AutoCalibWorker] = None
        self._anchor_worker: Optional[AnchorWorker] = None
        self._anchor_pose: Optional[CalibPose] = None
        # Set when samples came from an instance on disk rather than the
        # camera. Saving then offers to write back over that instance instead
        # of minting a new folder.
        self._loaded_dir: str = ""
        self._loaded_project: str = ""
        self._loaded_folder: str = ""
        self._loaded_intrinsics = None
        self._open_dirs: List[str] = []
        # The view shows a stored frame while a sample is selected; the live
        # tick must not paint over it.
        self._previewing = False

        self._mode = QComboBox()
        self._open_project = QComboBox()
        self._open_instance = QComboBox()
        self._open_btn = QPushButton("Open instance")
        self._open_refresh_btn = QPushButton("Refresh")
        self._loaded_label = QLabel("Nothing loaded.")
        self._overwrite_check = QCheckBox("Save over the loaded instance")

        self._anchor_combo = QComboBox()
        self._ghost_check = QCheckBox("Ghost the reference corners")
        self._anchor_btn = QPushButton("Set anchor here")
        self._load_prev_btn = QPushButton("Seed refinement from previous result")
        self._run_btn = QPushButton(SYM_PLAY + " Run auto calibration")
        self._stop_btn = QPushButton(SYM_STOP + " Stop")
        self._capture_btn = QPushButton("Capture sample")
        self._import_btn = QPushButton("Import images...")
        self._import_poses_btn = QPushButton("Import poses CSV...")
        self._coverage_spin = QDoubleSpinBox()
        self._delete_btn = QPushButton("Delete sample")
        self._solve_btn = QPushButton("Solve")
        self._save_btn = QPushButton("Save instance")
        self._edit_pose_btn = QPushButton("Enter pose for sample")
        self._live_btn = QPushButton("Back to live view")

        self._progress = QProgressBar()
        self._sample_list = QListWidget()
        self._view = ImageLabel()
        self._live_label = QLabel("-")
        self._anchor_label = QLabel("No anchor measured.")
        self._result_label = QLabel("Not solved.")
        self._remount_label = QLabel("-")
        self._sample_detail = QLabel("-")

        self._build()
        self._wire()
        self._on_mode_changed()
        self.refresh_open_choices()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / LIVE_VIEW_HZ))

    # --------------------------------------------------------------- layout
    def _build(self) -> None:
        mode_box = QGroupBox("Capture mode")
        mode_layout = QVBoxLayout(mode_box)
        for mode, label in MODE_LABELS.items():
            self._mode.addItem(label, mode.value)
        mode_layout.addWidget(self._mode)

        open_box = QGroupBox("Open a saved instance")
        open_layout = QVBoxLayout(open_box)
        open_layout.addWidget(QLabel(
            "Load a previous run's samples, poses and images, then re-solve "
            "them. Useful when the captures were made elsewhere, or when a "
            "pose or a convention needs correcting after the fact."))
        open_row = QHBoxLayout()
        open_row.addWidget(self._open_project, 1)
        open_row.addWidget(self._open_instance, 1)
        open_row.addWidget(self._open_refresh_btn)
        open_row.addWidget(self._open_btn)
        open_layout.addLayout(open_row)
        self._loaded_label.setWordWrap(True)
        open_layout.addWidget(self._loaded_label)

        anchor_box = QGroupBox("Remount anchor")
        anchor_layout = QVBoxLayout(anchor_box)
        anchor_layout.addWidget(QLabel(
            "Jog until the ghosted reference corners sit on the live ones, "
            "then set the anchor. Matching all the corners pins all six "
            "degrees of freedom; merely centring the board pins about two."))
        anchor_row = QHBoxLayout()
        anchor_row.addWidget(QLabel("Anchor pose:"))
        anchor_row.addWidget(self._anchor_combo, 1)
        anchor_row.addWidget(self._anchor_btn)
        anchor_layout.addLayout(anchor_row)
        anchor_layout.addWidget(self._ghost_check)
        anchor_layout.addWidget(self._load_prev_btn)
        self._anchor_label.setWordWrap(True)
        anchor_layout.addWidget(self._anchor_label)

        run_box = QGroupBox("Run")
        run_layout = QVBoxLayout(run_box)
        run_row = QHBoxLayout()
        run_row.addWidget(self._run_btn)
        run_row.addWidget(self._stop_btn)
        run_row.addWidget(self._capture_btn)
        run_layout.addLayout(run_row)
        import_row = QHBoxLayout()
        import_row.addWidget(self._import_btn)
        import_row.addWidget(self._import_poses_btn)
        import_row.addStretch(1)
        run_layout.addLayout(import_row)
        self._import_btn.setToolTip(
            "Load existing image files (taken with this camera at the current "
            "resolution), detect the board in each, and add the good ones as "
            "samples. Assign each sample its arm pose afterwards -- select it "
            "and use 'Enter pose for sample', or batch-assign with 'Import "
            "poses CSV'.")
        self._import_poses_btn.setToolTip(
            "Assign arm poses to the samples from a CSV file: rows of "
            "x,y,z,rx,ry,rz applied in order to samples without a pose, or "
            "filename,x,y,z,rx,ry,rz matched to each sample's image file. "
            "Raw controller numbers, same as the pose-entry dialog.")
        coverage_row = QHBoxLayout()
        coverage_row.addWidget(QLabel("Min board coverage:"))
        self._coverage_spin.setRange(0.1, 50.0)
        self._coverage_spin.setDecimals(1)
        self._coverage_spin.setSingleStep(0.5)
        self._coverage_spin.setSuffix(" %")
        self._coverage_spin.setToolTip(
            "Smallest fraction of the image the detected board may span "
            "before a pose is estimated. A small far-away board yields a "
            "badly conditioned pose that still reprojects well, so this gate "
            "exists -- but the right floor depends on your working distance.")
        coverage_row.addWidget(self._coverage_spin)
        coverage_row.addStretch(1)
        run_layout.addLayout(coverage_row)
        run_layout.addWidget(self._progress)
        danger = QLabel("An auto run moves the arm through every stored pose.")
        danger.setStyleSheet(DANGER_QSS)
        run_layout.addWidget(danger)

        samples_box = QGroupBox("Samples")
        samples_layout = QVBoxLayout(samples_box)
        samples_layout.addWidget(self._sample_list, 1)
        sample_row = QHBoxLayout()
        sample_row.addWidget(self._edit_pose_btn)
        sample_row.addWidget(self._delete_btn)
        sample_row.addWidget(self._solve_btn)
        sample_row.addWidget(self._save_btn)
        samples_layout.addLayout(sample_row)
        samples_layout.addWidget(self._overwrite_check)
        for label in (self._result_label, self._remount_label):
            label.setWordWrap(True)
            samples_layout.addWidget(label)

        left = QVBoxLayout()
        left.addWidget(mode_box)
        left.addWidget(open_box)
        left.addWidget(anchor_box)
        left.addWidget(run_box)
        left.addWidget(samples_box, 1)

        right = QVBoxLayout()
        right.addWidget(self._view, 1)
        self._live_label.setWordWrap(True)
        self._sample_detail.setWordWrap(True)
        right.addWidget(self._live_label)
        right.addWidget(self._sample_detail)
        right.addWidget(self._live_btn)

        root = QHBoxLayout(self)
        panel = QWidget()
        panel.setLayout(left)
        panel.setMaximumWidth(640)
        root.addWidget(panel)
        root.addLayout(right, 1)

    def _wire(self) -> None:
        self._mode.currentIndexChanged.connect(self._on_mode_changed)
        self._open_project.currentIndexChanged.connect(self._refresh_open_list)
        self._open_refresh_btn.clicked.connect(self.refresh_open_choices)
        self._open_btn.clicked.connect(self._open_saved_instance)
        self._anchor_btn.clicked.connect(self._set_anchor)
        self._load_prev_btn.clicked.connect(self._load_previous)
        self._run_btn.clicked.connect(self._start_run)
        self._stop_btn.clicked.connect(self._stop_run)
        self._capture_btn.clicked.connect(self._capture_manual)
        self._import_btn.clicked.connect(self._import_images)
        self._import_poses_btn.clicked.connect(self._import_poses_csv)
        self._coverage_spin.setValue(
            float(self._ctx.settings.min_image_coverage) * 100.0)
        self._coverage_spin.valueChanged.connect(self._on_coverage_changed)
        self._delete_btn.clicked.connect(self._delete_sample)
        self._solve_btn.clicked.connect(self._solve)
        self._save_btn.clicked.connect(self._save)
        self._edit_pose_btn.clicked.connect(self._enter_pose)
        self._live_btn.clicked.connect(self._show_live)
        self._sample_list.currentRowChanged.connect(self._on_sample_selected)
        self._ctx.project_changed.connect(self._refresh_anchor_choices)
        self._load_prev_btn.setToolTip(
            "Reads only T_cam2mount from the newest solved instance, to seed "
            "the per-pose refinement servo. It does not load samples -- use "
            "'Open instance' above for that.")

    def _current_mode(self) -> CaptureMode:
        return CaptureMode(self._mode.currentData())

    # ------------------------------------------------------------ mode/state
    @Slot()
    def _on_mode_changed(self) -> None:
        mode = self._current_mode()
        auto = mode == CaptureMode.AUTO
        self._anchor_btn.setEnabled(auto)
        self._anchor_combo.setEnabled(auto)
        self._load_prev_btn.setEnabled(auto)
        self._run_btn.setEnabled(auto)
        self._stop_btn.setEnabled(False)
        self._capture_btn.setEnabled(not auto)
        # Importing files is the images-only workflow by definition: the
        # frames exist already and the poses arrive later, by hand or by CSV.
        self._import_btn.setEnabled(mode == CaptureMode.IMAGES_ONLY)
        # Pose entry stays available in every mode: a loaded instance can be
        # missing poses regardless of how it was captured, and a pose typed in
        # wrong is exactly the thing you come back to fix.
        self._edit_pose_btn.setEnabled(True)
        self._ghost_check.setChecked(auto)
        self._refresh_anchor_choices()

    @Slot()
    def _refresh_anchor_choices(self) -> None:
        self._anchor_combo.clear()
        if self._ctx.project is None:
            return
        for pose in self._ctx.project.poses:
            if pose.T_mount2base is None:
                continue
            self._anchor_combo.addItem(
                "{}{}".format(pose.name, " (home)" if pose.is_home else ""),
                pose.index)
        home = self._ctx.project.home()
        if home is not None:
            row = self._anchor_combo.findData(home.index)
            if row >= 0:
                self._anchor_combo.setCurrentIndex(row)

    def _selected_anchor(self) -> Optional[CalibPose]:
        if self._ctx.project is None:
            return None
        index = self._anchor_combo.currentData()
        for pose in self._ctx.project.poses:
            if pose.index == index:
                return pose
        return None

    def _gates(self) -> CaptureGates:
        settings = self._ctx.settings
        return CaptureGates(
            max_reproj_px=settings.max_reproj_px,
            max_frame_age_s=settings.max_frame_age_s,
            settle_s=settings.settle_s,
            require_board_static=settings.require_board_static,
            board_motion_px=settings.board_motion_px,
            min_distance_m=settings.min_distance_m,
            max_distance_m=settings.max_distance_m,
            min_image_coverage=settings.min_image_coverage)

    # ----------------------------------------------------- open an instance
    @Slot()
    def refresh_open_choices(self) -> None:
        self._open_project.blockSignals(True)
        self._open_project.clear()
        for name in project_io.list_projects(self._ctx.settings.root_dir):
            self._open_project.addItem(name, name)
        self._open_project.blockSignals(False)
        if self._ctx.project is not None:
            row = self._open_project.findData(
                project_io.sanitize_name(self._ctx.project.name))
            if row >= 0:
                self._open_project.setCurrentIndex(row)
        self._refresh_open_list()

    @Slot()
    def _refresh_open_list(self) -> None:
        self._open_instance.clear()
        self._open_dirs = []
        name = self._open_project.currentData()
        if not name:
            return
        base = project_io.ProjectPaths(self._ctx.settings.root_dir,
                                       name).instances_dir
        for folder in project_io.list_instances(self._ctx.settings.root_dir,
                                                name):
            self._open_instance.addItem(folder, folder)
            self._open_dirs.append(os.path.join(base, folder))

    @Slot()
    def _open_saved_instance(self) -> None:
        if self._run_active():
            QMessageBox.information(
                self, "Open instance",
                "A run is in progress. Loading an instance would replace the "
                "sample list the worker is appending to.")
            return
        row = self._open_instance.currentIndex()
        if row < 0 or row >= len(self._open_dirs):
            return
        if self._samples:
            answer = QMessageBox.question(
                self, "Open instance",
                "Discard the {} sample(s) currently in the list and load the "
                "saved instance?".format(len(self._samples)),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return

        directory = self._open_dirs[row]
        loaded = project_io.load_instance(directory)
        if loaded.is_err:
            QMessageBox.warning(self, "Open instance", loaded.message)
            self._ctx.log.error(loaded.message)
            return
        instance, samples = loaded.data

        self._samples = list(samples)
        # Images stay on disk and are read on demand; holding a whole run's
        # frames in memory buys nothing when every sample already knows its
        # own path.
        self._images = [(None, None) for _ in self._samples]
        self._result = instance.result
        self._loaded_dir = directory
        self._loaded_project = self._open_project.currentData() or ""
        self._loaded_folder = self._open_instance.currentData() or ""
        self._loaded_intrinsics = instance.intrinsics
        self._overwrite_check.setChecked(False)

        self._delta = instance.remount.delta
        self._estimates = []
        # These were measured against the mount that instance was captured on,
        # so they are shown and re-saved but never counted as an anchor for a
        # new run.
        self._anchor_estimates = []
        for index, matrix in zip(instance.remount.anchor_indices,
                                 instance.remount.per_pose_deltas):
            self._estimates.append(remount.DeltaEstimate(
                pose_index=int(index), delta=np.array(matrix, dtype=float)))
        self._remount_label.setText(instance.remount.verdict or "-")

        self._adopt_instance_settings(instance)
        self._refresh_sample_list()
        self._show_result(instance.result)

        missing = sum(1 for s in self._samples if s.pose is None)
        without_images = sum(1 for s in self._samples
                             if not os.path.exists(s.image_path or ""))
        notes = []
        if missing:
            notes.append("{} sample(s) have no arm pose -- select one and use "
                         "'Enter pose for sample'".format(missing))
        if without_images:
            notes.append("{} sample(s) have no image on disk".format(
                without_images))
        self._loaded_label.setText(
            "Loaded {} ({} samples, captured {}, mode {}){}{}".format(
                instance.name, len(self._samples),
                instance.created[:16] or "unknown", instance.mode.value,
                "" if not notes else ".  ", ".  ".join(notes)))
        self._ctx.log.success("Opened instance {}".format(directory))

    def _adopt_instance_settings(self, instance: CalibInstance) -> None:
        """Match the app to the instance so a re-solve reproduces it.

        Re-solving stored samples under a different board or a different
        distortion model does not fail -- it quietly produces a different
        answer. So the board is adopted (with a prompt, since it changes what
        the live view detects) and the intrinsics recorded in the instance are
        used for anything re-derived from its images.
        """
        current = self._ctx.settings.board_config()
        if not instance.board.matches(current):
            if self._run_active():
                # The worker keeps the board and detector it was constructed
                # with, so adopting another one now would record a geometry
                # the samples were never captured against.
                self._ctx.log.warn(
                    "A run is in progress, so the instance's board was NOT "
                    "adopted.")
            else:
                answer = QMessageBox.question(
                    self, "Open instance",
                    "This instance was captured with a different board "
                    "({}x{} squares, {:.1f} mm, {}) than the one currently "
                    "configured. Adopt the instance's board?".format(
                        instance.board.squares_x, instance.board.squares_y,
                        instance.board.square_length_m * 1000.0,
                        instance.board.dictionary),
                    QMessageBox.StandardButton.Yes
                    | QMessageBox.StandardButton.No)
                if answer == QMessageBox.StandardButton.Yes:
                    self._ctx.settings.set_board_config(instance.board)
                    self._ctx.save_settings()
                    rebuilt = self._ctx.rebuild_board()
                    if rebuilt.is_err:
                        self._ctx.log.error(rebuilt.message)

        stored = instance.convention
        if stored.verified and stored.to_dict() != \
                self._ctx.settings.pose_convention().to_dict():
            self._ctx.log.warn(
                "This instance was captured with pose convention '{}', which "
                "differs from the current one. Its samples keep their own raw "
                "poses, so the stored transforms are unaffected -- but a pose "
                "typed in now would be read with the CURRENT "
                "convention.".format(stored.label()))

    def _show_result(self, result) -> None:
        if result is None:
            self._result_label.setText("Not solved.")
            return
        t = result.T_cam2mount
        self._result_label.setText(
            "T_cam2mount  x {:+.4f}  y {:+.4f}  z {:+.4f} m<br>{} over {} "
            "samples  |  board-origin spread {:.2f} mm  |  rotation spread "
            "{:.2f} deg  |  solved {}".format(
                t[0, 3], t[1, 3], t[2, 3], result.method, result.num_samples,
                result.board_origin_spread_mm, result.rotation_spread_deg,
                result.timestamp[:16] or "unknown"))

    # ------------------------------------------------------ sample preview
    def _sample_index_at(self, row: int) -> int:
        """The _samples index a list row stands for, or -1 if it is not one.

        Rows and samples do not correspond: a pose that failed adds a row and
        no sample, so from the first failure on the two indices diverge. Every
        row therefore carries its own sample index, and selection and deletion
        resolve through that rather than through the row number.
        """
        item = self._sample_list.item(row) if row >= 0 else None
        if item is None:
            return -1
        stored = item.data(Qt.ItemDataRole.UserRole)
        if stored is None:
            return -1
        return int(stored)

    @Slot(int)
    def _on_sample_selected(self, row: int) -> None:
        index = self._sample_index_at(row)
        if index < 0 or index >= len(self._samples):
            # Either nothing is selected or the row is a failed pose. Leaving
            # _previewing set would hold another sample's frame on screen and
            # invite it being read as this row's.
            self._previewing = False
            self._sample_detail.setText(
                "-" if row < 0 else
                "This pose failed: no sample, no image, nothing to delete.")
            return
        sample = self._samples[index]
        self._previewing = True

        image = None
        if index < len(self._images) and self._images[index][1] is not None:
            image = self._images[index][1]
        elif index < len(self._images) and self._images[index][0] is not None:
            image = self._images[index][0]
        else:
            for path in (sample.overlay_path, sample.image_path):
                if path and os.path.exists(path):
                    image = cv2.imread(path)
                    if image is not None:
                        break
        if image is not None:
            self._view.set_cv_image(image)
        else:
            self._live_label.setText(
                "Sample {} has no image on disk; showing the last live "
                "frame.".format(index))

        self._sample_detail.setText(self._describe_sample(sample, index))

    def _describe_sample(self, sample: handeye.HandEyeSample,
                         index: int) -> str:
        distance = float(np.linalg.norm(sample.T_target2cam[:3, 3]))
        lines = ["sample {}  |  {} corners  |  reproj {:.2f} px  |  board at "
                 "{:.3f} m".format(index, sample.n_corners,
                                   sample.reproj_error, distance)]
        if sample.pose is None:
            lines.append("<span style='color:{}'>no arm pose recorded -- "
                         "enter it before solving</span>".format(WARN_COLOR))
        else:
            p = sample.pose
            lines.append(
                "pose  X {:+.2f}  Y {:+.2f}  Z {:+.2f}   "
                "Rx {:+.3f}  Ry {:+.3f}  Rz {:+.3f}   ({}, Tool {}, "
                "User {})".format(p.x, p.y, p.z, p.rx, p.ry, p.rz,
                                  p.source or "unknown", p.tool_index,
                                  p.user_index))
            t = sample.T_mount2base
            lines.append("T_mount2base translation  {:+.4f}  {:+.4f}  "
                         "{:+.4f} m".format(t[0, 3], t[1, 3], t[2, 3]))
        if sample.image_path:
            lines.append(os.path.basename(sample.image_path))
        return "<br>".join(lines)

    @Slot()
    def _show_live(self) -> None:
        self._sample_list.setCurrentRow(-1)
        self._previewing = False
        self._sample_detail.setText("-")

    # -------------------------------------------------------------- anchor
    @Slot()
    def _load_previous(self) -> None:
        if self._ctx.project is None:
            return
        found = project_io.latest_instance_with_result(
            self._ctx.settings.root_dir, self._ctx.project.name)
        if found.is_err:
            self._ctx.log.warn(found.message)
            self._anchor_label.setText(found.message)
            return
        instance, folder = found.data
        self._X_prev = instance.result.T_cam2mount
        self._anchor_label.setText(
            "Previous calibration loaded from {} ({} samples, spread "
            "{:.2f} mm). It seeds the per-pose refinement; it is not reused "
            "as a result.".format(
                instance.name, instance.result.num_samples,
                instance.result.board_origin_spread_mm))
        self._ctx.log.info("Loaded previous calibration from {}".format(folder))

    @Slot()
    def _set_anchor(self) -> None:
        pose = self._selected_anchor()
        if pose is None:
            QMessageBox.information(self, "Anchor",
                                    "Select a project pose to anchor against.")
            return
        if not (self._ctx.robot_connected() and self._ctx.camera_connected()):
            QMessageBox.information(self, "Anchor",
                                    "Connect both the arm and the camera.")
            return
        intrinsics = self._ctx.intrinsics()
        if intrinsics.is_err:
            QMessageBox.warning(self, "Anchor", intrinsics.message)
            return
        self._anchor_pose = pose
        self._anchor_worker = AnchorWorker(
            self._ctx.api, self._ctx.camera, pose, self._ctx.board,
            self._ctx.detector, intrinsics.data,
            self._ctx.settings.pose_convention(),
            min_coverage=self._ctx.settings.min_image_coverage, parent=self)
        self._anchor_worker.done.connect(self._on_anchor)
        self._anchor_worker.start()

    @Slot(object)
    def _on_anchor(self, result: Result) -> None:
        if result.data is None:
            self._anchor_label.setText(
                "<span style='color:{}'>{}</span>".format(BAD_COLOR,
                                                          result.message))
            self._ctx.log.error(result.message)
            return
        estimate = result.data
        if result.is_err:
            answer = QMessageBox.question(
                self, "Anchor", result.message + "\n\nUse it anyway?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return
        self._anchor_estimates = [estimate]
        self._estimates = list(self._anchor_estimates)
        self._delta = estimate.delta
        shift_mm = float(np.linalg.norm(estimate.delta[:3, 3])) * 1000.0
        record = remount.summarise(self._estimates)
        self._anchor_label.setText(
            "<span style='color:{}'>Anchor set on pose {}: camera moved "
            "{:.1f} mm on the flange. Every project pose will be corrected by "
            "this.</span>".format(OK_COLOR, estimate.pose_index, shift_mm))
        self._remount_label.setText(record.verdict)
        self._ctx.log.success("Remount delta measured at pose {}.".format(
            estimate.pose_index))

    # ------------------------------------------------------------- auto run
    @Slot()
    def _start_run(self) -> None:
        if self._ctx.project is None or not self._ctx.project.poses:
            QMessageBox.information(self, "Run",
                                    "Select a project with poses first.")
            return
        if not (self._ctx.robot_connected() and self._ctx.camera_connected()):
            QMessageBox.information(self, "Run",
                                    "Connect both the arm and the camera.")
            return
        ready = self._ctx.convention_ready()
        if ready.is_err:
            QMessageBox.warning(self, "Run", ready.message)
            return
        intrinsics = self._ctx.intrinsics()
        if intrinsics.is_err:
            QMessageBox.warning(self, "Run", intrinsics.message)
            return

        state = self._ctx.api.latest()
        if state.is_ok:
            frames_ok = self._ctx.project.frames.compare(
                state.data.frame_identity())
            if frames_ok.is_err:
                answer = QMessageBox.question(
                    self, "Run", frames_ok.message + "\n\nContinue anyway?",
                    QMessageBox.StandardButton.Yes
                    | QMessageBox.StandardButton.No)
                if answer != QMessageBox.StandardButton.Yes:
                    return

        # Only an anchor measured for the mount in front of us counts. A delta
        # rehydrated from a loaded instance belongs to the mount that instance
        # was captured on, and using it here would correct every pose by the
        # wrong transform while suppressing this very prompt.
        anchor_delta = (self._anchor_estimates[-1].delta
                        if self._anchor_estimates else None)
        if anchor_delta is None:
            answer = QMessageBox.question(
                self, "Run",
                "No remount anchor has been measured. The stored poses will "
                "be replayed exactly as authored, which is correct only if "
                "the camera has not moved on the flange since.\n\nContinue?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return

        settings = self._ctx.settings
        safety = settings.safety_config()
        self._samples = []
        self._images = []
        self._result = None
        self._sample_list.clear()
        # Every estimate this run appends must join the anchor's, not the
        # previous run's and not a loaded instance's.
        self._estimates = list(self._anchor_estimates)
        self._delta = anchor_delta
        self._forget_loaded_instance()

        self._worker = AutoCalibWorker(
            self._ctx.api, self._ctx.camera, self._ctx.project.poses,
            self._ctx.board, self._ctx.detector, intrinsics.data,
            settings.pose_convention(), safety, self._gates(),
            delta=self._delta, X_prev=self._X_prev,
            confirm_each=safety.confirm_each_pose,
            refine=settings.refine_each_pose,
            skip_failed=settings.skip_failed_poses,
            move_timeout_s=settings.move_timeout_s,
            tool_index=settings.tool_index, user_index=settings.user_index,
            parent=self)
        self._worker.progress.connect(self._on_progress)
        self._worker.log.connect(self._on_worker_log)
        self._worker.pose_finished.connect(self._on_pose_finished)
        self._worker.confirm_required.connect(self._on_confirm)
        self._worker.run_finished.connect(self._on_run_finished)
        self._run_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._set_run_controls_enabled(False)
        self._worker.start()

    def _run_active(self) -> bool:
        """True from the moment a run is started until its thread has ended."""
        return self._worker is not None and not self._worker.isFinished()

    def _forget_loaded_instance(self) -> None:
        """Drop the loaded instance's identity.

        Called when the sample set stops being that instance's. Keeping it
        would label the new folder a 'resolve', stamp derived_from with an
        instance that contributed nothing, and leave 'Save over the loaded
        instance' pointed at an unrelated run.
        """
        if self._loaded_dir:
            self._ctx.log.info(
                "Instance {} is no longer loaded: these samples are freshly "
                "captured.".format(self._loaded_folder or self._loaded_dir))
        self._loaded_dir = ""
        self._loaded_project = ""
        self._loaded_folder = ""
        self._loaded_intrinsics = None
        self._overwrite_check.setChecked(False)
        self._loaded_label.setText("Nothing loaded.")

    def _set_run_controls_enabled(self, enabled: bool) -> None:
        """Lock everything that mutates what the worker is writing into.

        The queued pose_finished slot appends onto _samples and _images, and
        Qt delivers queued signals inside a modal dialog's event loop too, so
        'Open instance' or 'Delete sample' mid-run splices two runs together.
        A run only starts in AUTO mode, so re-enabling is unconditional.
        """
        for widget in (self._mode, self._open_project, self._open_instance,
                       self._open_refresh_btn, self._open_btn,
                       self._anchor_combo, self._anchor_btn,
                       self._load_prev_btn, self._delete_btn,
                       self._edit_pose_btn, self._solve_btn, self._save_btn,
                       self._overwrite_check, self._import_btn,
                       self._import_poses_btn, self._coverage_spin):
            widget.setEnabled(enabled)
        if enabled:
            # Re-enabling is mode-aware: a run only starts in AUTO mode, where
            # image import stays off.
            self._import_btn.setEnabled(
                self._current_mode() == CaptureMode.IMAGES_ONLY)

    def abort_run(self) -> None:
        """Abort a running auto run. Safe when idle.

        The always-visible STOP calls this, so it must not care whether a run
        is in progress or what state it is in.
        """
        worker = self._worker
        if worker is None or worker.isFinished():
            return
        worker.abort()
        self._ctx.log.warn("Abort requested; stopping after the current "
                           "move.")

    def stop_workers(self) -> None:
        """Abort and join every worker this tab owns. Never raises.

        Called on the close path: a QThread still running when the widget tree
        is destroyed takes the process down with it.
        """
        self._timer.stop()
        worker = self._worker
        self._worker = None
        if worker is not None:
            worker.abort()
            if worker.isRunning():
                worker.wait(2000)
        anchor = self._anchor_worker
        self._anchor_worker = None
        if anchor is not None and anchor.isRunning():
            # AnchorWorker has no abort: it is one pose read and one frame.
            anchor.requestInterruption()
            anchor.wait(2000)

    @Slot()
    def _stop_run(self) -> None:
        self.abort_run()

    @Slot(int, int, str)
    def _on_progress(self, done: int, total: int, message: str) -> None:
        self._progress.setMaximum(max(1, total))
        self._progress.setValue(done)
        self._progress.setFormat("{}/{}  {}".format(done, total, message))

    @Slot(str, str)
    def _on_worker_log(self, message: str, level: str) -> None:
        self._ctx.log.log(message, level)

    @Slot(int, str)
    def _on_confirm(self, index: int, question: str) -> None:
        answer = QMessageBox.question(
            self, "Confirm move", question,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if self._worker is not None:
            self._worker.answer_confirm(
                answer == QMessageBox.StandardButton.Yes)

    @Slot(object)
    def _on_pose_finished(self, outcome) -> None:
        if not outcome.ok or outcome.sample is None:
            # -1: this row stands for no sample at all.
            item = QListWidgetItem("pose {}: FAILED {}".format(
                outcome.index, outcome.message))
            item.setData(Qt.ItemDataRole.UserRole, -1)
            self._sample_list.addItem(item)
            return
        self._samples.append(outcome.sample)
        self._images.append((outcome.raw_bgr, outcome.overlay_bgr))
        if outcome.delta is not None:
            self._estimates.append(remount.DeltaEstimate(
                pose_index=outcome.index, delta=outcome.delta,
                residual_mm=outcome.view_error_mm,
                residual_deg=outcome.view_error_deg))
        self._append_sample_row(outcome.sample, len(self._samples) - 1)

    @Slot(object)
    def _on_run_finished(self, result: Result) -> None:
        self._run_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._set_run_controls_enabled(True)
        if result.is_err:
            self._ctx.log.warn(result.message)
        else:
            self._ctx.log.success("Auto run finished with {} samples.".format(
                len(self._samples)))
        if len(self._estimates) > 1:
            record = remount.summarise(self._estimates)
            colour = OK_COLOR if "INCONSISTENT" not in record.verdict else BAD_COLOR
            self._remount_label.setText(
                "<span style='color:{}'>{}</span>".format(colour,
                                                          record.verdict))
        self._capture_images_note()

    def _capture_images_note(self) -> None:
        if self._samples:
            self._ctx.log.info(
                "{} sample frame(s) held; their images are written when the "
                "instance is saved.".format(len(self._samples)))

    # ---------------------------------------------------------- manual capture
    @Slot()
    def _capture_manual(self) -> None:
        if not self._ctx.camera_connected():
            QMessageBox.information(self, "Capture", "Connect a camera first.")
            return
        intrinsics = self._ctx.intrinsics()
        if intrinsics.is_err:
            QMessageBox.warning(self, "Capture", intrinsics.message)
            return
        mode = self._current_mode()
        api = (self._ctx.api if mode == CaptureMode.MANUAL
               and self._ctx.robot_connected() else None)
        if mode == CaptureMode.MANUAL and api is None:
            QMessageBox.information(
                self, "Capture",
                "Manual mode records the arm pose with each sample. Connect "
                "the arm, or switch to images-only mode.")
            return

        captured = capture_sample(
            self._ctx.camera, api, self._ctx.board, self._ctx.detector,
            intrinsics.data, self._ctx.settings.pose_convention(),
            self._gates())
        if captured.is_err:
            self._ctx.log.warn("Sample rejected: {}".format(captured.message))
            return
        sample, _det, raw, overlay = captured.data
        self._samples.append(sample)
        self._images.append((raw, overlay))
        self._result = None
        self._append_sample_row(sample, len(self._samples) - 1)
        self._ctx.log.success("Sample {} captured.".format(len(self._samples)))

    # ---------------------------------------------------------- image import
    @Slot()
    def _import_images(self) -> None:
        """Add samples from image files already on disk.

        The whole point of images-only mode, minus the live camera: frames
        captured earlier (or on another machine) are detected and gated
        exactly like a live capture, then their poses are typed in or
        batch-assigned from a CSV. Intrinsics must be known -- either the
        camera is connected, or the library holds a record for the configured
        camera and resolution -- and every file must match that resolution,
        because intrinsics scale with it.
        """
        intrinsics = self._ctx.intrinsics()
        print(1)
        if intrinsics.is_err:
            QMessageBox.warning(
                self, "Import images",
                "No camera intrinsics are available ({}). Connect the camera "
                "the images came from, or store its intrinsics in the "
                "library first -- without them no board pose can be "
                "estimated.".format(intrinsics.message))
            return
        print(2)
        start_dir = (self._ctx.settings.last_import_dir
                     or os.path.expanduser("~"))
        print(3)
        files, _filter = QFileDialog.getOpenFileNames(
            self, "Import images", start_dir,
            "Images (*.png *.jpg *.jpeg *.bmp);;All files (*)")
        print(4)
        if not files:
            print(4.5)
            return
        print(5)
        self._ctx.settings.last_import_dir = os.path.dirname(files[0])
        self._ctx.save_settings()

        intr = intrinsics.data
        gates = self._gates()
        added = 0
        skipped = []
        for path in files:
            name = os.path.basename(path)
            loaded = project_io.load_image(path)
            if loaded.is_err:
                skipped.append((name, loaded.message))
                continue
            bgr = loaded.data
            height, width = bgr.shape[:2]
            if (width, height) != (intr.width, intr.height):
                skipped.append((name, "is {}x{} but the intrinsics are for "
                                "{}x{} -- intrinsics scale with resolution"
                                .format(width, height, intr.width,
                                        intr.height)))
                continue
            gated = gate_still_image(bgr, self._ctx.board, self._ctx.detector,
                                     intr, gates)
            if gated.is_err:
                skipped.append((name, gated.message))
                continue
            sample, _det, overlay = gated.data
            # The source path doubles as the sample's identity until save
            # writes the instance's own copy: the CSV importer matches on it,
            # and the preview can re-read it from disk.
            sample.image_path = os.path.abspath(path)
            self._samples.append(sample)
            self._images.append((bgr, overlay))
            self._append_sample_row(sample, len(self._samples) - 1)
            added += 1

        if added:
            self._result = None
            self._result_label.setText("Not solved (sample set changed).")
        summary = "Imported {} of {} image(s) as samples.".format(
            added, len(files))
        if added:
            summary += ("\n\nEach needs its arm pose before solving: select "
                        "a sample and use 'Enter pose for sample', or "
                        "batch-assign with 'Import poses CSV'.")
        if skipped:
            shown = skipped[:12]
            summary += "\n\nSkipped:\n" + "\n".join(
                "  {} -- {}".format(name, reason) for name, reason in shown)
            if len(skipped) > len(shown):
                summary += "\n  ... and {} more (see the log)".format(
                    len(skipped) - len(shown))
        for name, reason in skipped:
            self._ctx.log.warn("Import skipped {}: {}".format(name, reason))
        self._ctx.log.log(
            "Imported {} of {} image(s).".format(added, len(files)),
            "SUCCESS" if added else "WARNING")
        QMessageBox.information(self, "Import images", summary)

    @Slot()
    def _import_poses_csv(self) -> None:
        """Batch-assign arm poses to the samples from a CSV file."""
        if not self._samples:
            QMessageBox.information(
                self, "Import poses",
                "There are no samples to assign poses to. Import or capture "
                "images first.")
            return
        start_dir = (self._ctx.settings.last_import_dir
                     or os.path.expanduser("~"))
        path, _filter = QFileDialog.getOpenFileName(
            self, "Import poses CSV", start_dir,
            "CSV files (*.csv *.txt);;All files (*)")
        if not path:
            return
        parsed = project_io.parse_pose_csv(path)
        if parsed.is_err:
            QMessageBox.warning(self, "Import poses", parsed.message)
            self._ctx.log.error(parsed.message)
            return
        rows = parsed.data
        convention = self._ctx.settings.pose_convention()

        def assign(sample, values) -> None:
            pose = RobotPose(
                x=values[0], y=values[1], z=values[2],
                rx=values[3], ry=values[4], rz=values[5],
                tool_index=self._ctx.settings.tool_index,
                user_index=self._ctx.settings.user_index,
                source="csv")
            sample.pose = pose
            sample.T_mount2base = pose.to_se3(convention,
                                              prefer_quaternion=False)

        assigned = 0
        notes = []
        if rows[0][0] is not None:
            by_name = {name.lower(): values for name, values in rows}
            matched_names = set()
            for sample in self._samples:
                key = os.path.basename(sample.image_path or "").lower()
                values = by_name.get(key)
                if values is None:
                    continue
                assign(sample, values)
                matched_names.add(key)
                assigned += 1
            unmatched = [name for name, _values in rows
                         if name.lower() not in matched_names]
            if unmatched:
                notes.append("{} row(s) matched no sample's image file: {}"
                             .format(len(unmatched),
                                     ", ".join(unmatched[:6])))
        else:
            waiting = [s for s in self._samples if s.pose is None]
            for sample, (_name, values) in zip(waiting, rows):
                assign(sample, values)
                assigned += 1
            if len(rows) > len(waiting):
                notes.append("{} row(s) were left over -- every sample "
                             "already had a pose".format(
                                 len(rows) - len(waiting)))
            elif len(waiting) > len(rows):
                notes.append("{} sample(s) still have no pose -- the file "
                             "had fewer rows".format(
                                 len(waiting) - len(rows)))

        if assigned:
            self._result = None
            self._result_label.setText("Not solved (sample set changed).")
            self._refresh_sample_list()
        still_missing = sum(1 for s in self._samples if s.pose is None)
        summary = ("Assigned {} pose(s) from {} row(s), as raw controller "
                   "numbers under convention {}.".format(
                       assigned, len(rows), convention.label()))
        if still_missing:
            summary += "\n{} sample(s) still have no pose.".format(
                still_missing)
        if notes:
            summary += "\n" + "\n".join(notes)
        self._ctx.log.log(summary.replace("\n", "  |  "),
                          "SUCCESS" if assigned else "WARNING")
        QMessageBox.information(self, "Import poses", summary)

    @Slot()
    def _on_coverage_changed(self) -> None:
        self._ctx.settings.min_image_coverage = \
            float(self._coverage_spin.value()) / 100.0
        self._ctx.save_settings()

    def _append_sample_row(self, sample: handeye.HandEyeSample,
                           index: int) -> None:
        """Add one row for the sample at `index` in _samples.

        `index` is the position in _samples in BOTH the append path and the
        rebuild path, so a row never changes what its number means, and it is
        stored in the item's data because from the first failed pose on the
        row number is no longer the sample index. The authored pose is named
        alongside: that identity survives a delete, the position does not.
        """
        distance = float(np.linalg.norm(sample.T_target2cam[:3, 3]))
        text = "{:02d}  {} corners  reproj {:.2f} px  {:.3f} m".format(
            index, sample.n_corners, sample.reproj_error, distance)
        if sample.pose_index >= 0:
            text += "   (pose {})".format(sample.pose_index)
        if sample.pose is None:
            text += "   [pose not set]"
        item = QListWidgetItem(text)
        item.setData(Qt.ItemDataRole.UserRole, index)
        self._sample_list.addItem(item)

    @Slot()
    def _delete_sample(self) -> None:
        index = self._sample_index_at(self._sample_list.currentRow())
        if index < 0 or index >= len(self._samples):
            return
        del self._samples[index]
        if index < len(self._images):
            del self._images[index]
        # Any stored result is now describing a sample set that no longer
        # exists; clearing it is not optional.
        self._result = None
        self._result_label.setText("Not solved (sample set changed).")
        self._show_live()
        self._refresh_sample_list()

    def _refresh_sample_list(self) -> None:
        self._sample_list.clear()
        for index, sample in enumerate(self._samples):
            self._append_sample_row(sample, index)

    @Slot()
    def _enter_pose(self) -> None:
        """Type in the arm pose for an image-only sample.

        The six numbers go in exactly as the controller reports them; the
        stored pose convention turns them into a transform, so a convention
        corrected later re-derives these without re-capturing anything.
        """
        index = self._sample_index_at(self._sample_list.currentRow())
        if index < 0 or index >= len(self._samples):
            return
        dialog = _PoseEntryDialog(self._ctx, self._samples[index].pose, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        pose = dialog.pose()
        convention = self._ctx.settings.pose_convention()
        self._samples[index].pose = pose
        self._samples[index].T_mount2base = pose.to_se3(
            convention, prefer_quaternion=False)
        self._result = None
        self._result_label.setText("Not solved (sample set changed).")
        self._refresh_sample_list()

    # --------------------------------------------------------------- solve
    @Slot()
    def _solve(self) -> None:
        missing = [i for i, s in enumerate(self._samples) if s.pose is None]
        if missing:
            QMessageBox.warning(
                self, "Solve",
                "Samples {} have no arm pose. Enter their poses before "
                "solving.".format(", ".join(str(i) for i in missing)))
            return
        solved = handeye.solve_handeye(self._samples,
                                       self._ctx.settings.handeye_method)
        if solved.is_err:
            self._result = None
            self._result_label.setText(
                "<span style='color:{}'>{}</span>".format(BAD_COLOR,
                                                          solved.message))
            self._ctx.log.error(solved.message)
            return
        self._result = solved.data
        residuals = handeye.compute_residuals(self._samples,
                                              solved.data.T_cam2mount)
        worst = max(residuals, key=lambda r: r.translation_mm)
        t = solved.data.T_cam2mount
        self._result_label.setText(
            "<span style='color:{}'>T_cam2mount  x {:+.4f}  y {:+.4f}  "
            "z {:+.4f} m<br>{} over {} samples  |  board-origin spread "
            "{:.2f} mm  |  rotation spread {:.2f} deg  |  worst sample #{} at "
            "{:.2f} mm</span><br>Spread is a residual, not validation: it is "
            "computed from the same data the solve used. Point at a known "
            "feature and measure where the tool lands.".format(
                OK_COLOR, t[0, 3], t[1, 3], t[2, 3], solved.data.method,
                solved.data.num_samples, solved.data.board_origin_spread_mm,
                solved.data.rotation_spread_deg, worst.index,
                worst.translation_mm))
        self._ctx.log.success("Hand-eye solved over {} samples.".format(
            solved.data.num_samples))

    # ---------------------------------------------------------------- save
    @Slot()
    def _save(self) -> None:
        if not self._samples:
            return
        # A re-solve may be done with no camera attached at all, so the
        # instance's own recorded intrinsics stand in. Saving a calibration
        # with no intrinsics block would make it impossible to check later
        # whether it still applies.
        live = self._ctx.intrinsics()
        intrinsics = (live.data if live.is_ok else self._loaded_intrinsics)
        if intrinsics is None:
            answer = QMessageBox.question(
                self, "Save instance",
                "No camera intrinsics are available ({}), so this instance "
                "cannot record which camera and resolution it belongs "
                "to.\n\nSave anyway?".format(live.message),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return

        overwrite = (self._overwrite_check.isChecked()
                     and bool(self._loaded_dir))
        if overwrite:
            answer = QMessageBox.question(
                self, "Save instance",
                "Overwrite {}?\n\nIts metadata and samples are replaced. "
                "Images already on disk are kept.".format(self._loaded_folder),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return
            project_name = self._loaded_project
            paths = project_io.instance_paths(
                self._ctx.settings.root_dir, project_name,
                self._loaded_folder)
        else:
            project_name = (self._ctx.project.name
                            if self._ctx.project is not None
                            else self._loaded_project or "unassigned")
            label = ("resolve" if self._loaded_dir
                     else self._current_mode().value)
            paths = project_io.new_instance_paths(
                self._ctx.settings.root_dir, project_name, label)

        for index, sample in enumerate(self._samples):
            raw, overlay = (self._images[index]
                            if index < len(self._images) else (None, None))
            if raw is None:
                # Re-solved from disk: the frames never came back into memory,
                # so leave the existing paths pointing where they already are.
                continue
            written = project_io.save_sample_images(paths, index, raw, overlay)
            if written.is_ok:
                sample.image_path, sample.overlay_path = written.data

        frames = self._ctx.project.frames if self._ctx.project else None
        state = (self._ctx.api.latest() if self._ctx.robot_connected()
                 else None)
        if state is not None and state.is_ok:
            frames = state.data.frame_identity()

        instance = CalibInstance(
            name="" if not overwrite else self._loaded_folder,
            project_name=project_name,
            mode=self._current_mode(),
            board=self._ctx.settings.board_config(),
            camera=self._ctx.settings.camera_config(),
            intrinsics=intrinsics,
            intrinsics_source=(IntrinsicsSource.CALIBRATED
                               if self._ctx.settings.prefer_library_intrinsics
                               else IntrinsicsSource.FACTORY),
            convention=self._ctx.settings.pose_convention(),
            frames=frames if frames is not None else CalibInstance().frames,
            remount=remount.summarise(self._estimates),
            result=self._result,
            derived_from=("" if overwrite else self._loaded_folder))
        saved = project_io.save_instance(paths, instance, self._samples)
        if saved.is_err:
            self._ctx.log.error(saved.message)
            return
        self._paths = paths
        self._loaded_dir = paths.dir
        self._loaded_project = project_name
        self._loaded_folder = paths.folder
        self._ctx.settings.last_instance_dir = paths.dir
        self._ctx.save_settings()
        self.refresh_open_choices()
        self._ctx.log.success("Instance saved to {}".format(saved.data))
        QMessageBox.information(
            self, "Saved",
            "Instance written to:\n{}\n\nThe metadata records the camera "
            "intrinsics and resolution, the board, the Tool/User frames and "
            "the pose convention, so this calibration can be checked for "
            "validity later.".format(saved.data))

    # ------------------------------------------------------------- live view
    @Slot()
    def _tick(self) -> None:
        if self._previewing:
            return          # a stored sample frame is on screen
        if not self._ctx.camera_connected():
            return
        frame = self._ctx.camera.get_color()
        if frame.is_err:
            return
        intrinsics = self._ctx.intrinsics()
        intr = intrinsics.data if intrinsics.is_ok else None
        det = handeye.detect_board(
            frame.data, self._ctx.board, self._ctx.detector, intr,
            min_coverage=self._ctx.settings.min_image_coverage)
        image = handeye.draw_overlay(frame.data, det, intr)

        anchor = self._selected_anchor()
        if (self._ghost_check.isChecked() and anchor is not None
                and anchor.ref_corners is not None):
            image = handeye.draw_ghost_corners(image, anchor.ref_corners,
                                               anchor.ref_ids, det)
        self._view.set_cv_image(image)

        parts = []
        if det.found:
            parts.append("{} corners".format(det.n_corners))
        if det.T_target2cam is not None:
            parts.append("reproj {:.2f} px".format(det.reproj_error))
            parts.append("{:.3f} m".format(
                float(np.linalg.norm(det.T_target2cam[:3, 3]))))
            if anchor is not None and anchor.T_board2cam_ref is not None:
                d_mm, d_deg = remount.view_error(det.T_target2cam,
                                                 anchor.T_board2cam_ref)
                converged = remount.is_converged(det.T_target2cam,
                                                 anchor.T_board2cam_ref)
                parts.append(
                    "<span style='color:{}'>vs reference: {:.1f} mm / "
                    "{:.2f} deg</span>".format(
                        OK_COLOR if converged else WARN_COLOR, d_mm, d_deg))
        elif det.message:
            parts.append(det.message)
        if not parts:
            parts.append("No board detected.")
        self._live_label.setText("  |  ".join(parts))


class _PoseEntryDialog(QDialog):
    """Type the six controller numbers for an image-only sample."""

    def __init__(self, ctx: AppContext, existing: Optional[RobotPose],
                 parent=None) -> None:
        super().__init__(parent)
        self._ctx = ctx
        self.setWindowTitle("Enter arm pose")

        convention = ctx.settings.pose_convention()
        unit = "deg" if convention.degrees else "rad"
        self._spins = []
        layout = QVBoxLayout(self)
        note = QLabel(
            "Enter the pose exactly as the controller reports it (mm and "
            "{}). It is stored raw and converted through the current "
            "convention: {}. Storing it raw means a convention corrected "
            "later re-derives this sample instead of wasting it.".format(
                unit, convention.label()))
        note.setWordWrap(True)
        layout.addWidget(note)
        for name, suffix in (("X", "mm"), ("Y", "mm"), ("Z", "mm"),
                             ("Rx", unit), ("Ry", unit), ("Rz", unit)):
            row = QHBoxLayout()
            row.addWidget(QLabel("{} ({}):".format(name, suffix)))
            spin = QDoubleSpinBox()
            spin.setRange(-100000.0, 100000.0)
            spin.setDecimals(4)
            row.addWidget(spin)
            layout.addLayout(row)
            self._spins.append(spin)
        if existing is not None:
            values = [existing.x, existing.y, existing.z,
                      existing.rx, existing.ry, existing.rz]
            for spin, value in zip(self._spins, values):
                spin.setValue(float(value))

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def pose(self) -> RobotPose:
        values = [float(spin.value()) for spin in self._spins]
        return RobotPose(x=values[0], y=values[1], z=values[2],
                         rx=values[3], ry=values[4], rz=values[5],
                         tool_index=self._ctx.settings.tool_index,
                         user_index=self._ctx.settings.user_index,
                         source="manual")