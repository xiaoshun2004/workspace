# -*- coding: utf-8 -*-
"""
ui/workers.py
-------------
Background threads. Everything that blocks -- socket round trips, arm motion,
waits for the arm to settle -- runs here, never on the UI thread.

AutoCalibWorker is the interesting one: it walks an authored project's poses,
correcting each by the measured remount delta, and captures a sample at each.
It never calls a widget method. It emits signals; the tab renders them.

Confirmation is a threading.Event rather than a dialog raised from the worker,
because a modal dialog created off the UI thread is undefined behaviour in Qt.
The worker asks and blocks; the tab answers from the UI thread.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np
from PySide6.QtCore import QThread, Signal

import handeye
import remount
from camera_base import CameraBase
from models import (
    CalibPose,
    CameraIntrinsics,
    PoseConvention,
    Result,
    SafetyConfig,
)
from nova5_api import Nova5Api, check_move


@dataclass
class CaptureGates:
    """Everything a frame must satisfy before it becomes a sample."""

    max_reproj_px: float = 1.0
    max_frame_age_s: float = 0.30
    settle_s: float = 0.40
    require_board_static: bool = True
    board_motion_px: float = 1.5
    min_distance_m: float = 0.12
    max_distance_m: float = 0.45
    # Fraction of the image the detected corners' bbox must span; forwarded
    # into detect_board so the capture gate and the live view agree.
    min_image_coverage: float = 0.02


def gate_still_image(bgr: np.ndarray, board, detector,
                     intrinsics: CameraIntrinsics,
                     gates: CaptureGates) -> Result[tuple]:
    """Detect and gate a frame that did NOT come from the live camera.

    The import-a-library workflow feeds files through the same reprojection
    and working-distance gates as a live capture -- a sample that skips them
    degrades the solve invisibly no matter where the pixels came from. The
    frame-age and board-static gates do not apply to a file on disk, which is
    exactly why this is a separate entry point rather than a flag on
    capture_sample.

    Returns (sample, detection, overlay) -- the sample carries no arm pose and
    an identity T_mount2base, to be filled in afterwards.
    """
    det = handeye.detect_board(bgr, board, detector, intrinsics,
                               min_coverage=gates.min_image_coverage)
    if det.T_target2cam is None:
        return Result.err_msg(det.message or "board pose not estimated")
    if not np.isfinite(det.reproj_error):
        return Result.err_msg("reprojection error is not finite")
    if det.reproj_error > gates.max_reproj_px:
        return Result.err_msg(
            "reprojection error {:.2f} px is above the {:.2f} px "
            "limit".format(det.reproj_error, gates.max_reproj_px))
    band = remount.check_distance_band(det.T_target2cam, gates.min_distance_m,
                                       gates.max_distance_m)
    if band.is_err:
        return Result.err_msg(band.message)
    sample = handeye.HandEyeSample(
        T_mount2base=np.eye(4), T_target2cam=det.T_target2cam,
        reproj_error=det.reproj_error, n_corners=det.n_corners,
        pose=None, pose_index=-1)
    overlay = handeye.draw_overlay(bgr, det, intrinsics)
    return Result.ok((sample, det, overlay))


@dataclass
class PoseOutcome:
    index: int
    ok: bool
    message: str
    sample: Optional[handeye.HandEyeSample] = None
    delta: Optional[np.ndarray] = None       # per-pose remount estimate
    view_error_mm: float = float("nan")
    view_error_deg: float = float("nan")
    # The frames the sample was taken from, carried back so the tab can write
    # them when the instance is saved. Without these an automated run would
    # produce a calibration with no imagery to audit it against, which is the
    # one thing you cannot reconstruct afterwards.
    raw_bgr: Optional[np.ndarray] = None
    overlay_bgr: Optional[np.ndarray] = None


def capture_sample(camera: CameraBase, api: Optional[Nova5Api],
                   board, detector, intrinsics: CameraIntrinsics,
                   convention: PoseConvention, gates: CaptureGates,
                   pose_index: int = -1) -> Result[tuple]:
    """Grab a frame, detect the board, gate it, and pair it with a pose.

    Returns (sample, detection, raw_bgr, overlay_bgr) so the caller can save
    the imagery. The gates here are the reason this is one function rather
    than inline code in three places: a sample that skips any of them is worse
    than no sample, because it degrades the solve invisibly.
    """
    first = camera.get_color()
    if first.is_err:
        return Result.err_msg(first.message)
    age = camera.frame_age_s()
    if age > gates.max_frame_age_s:
        return Result.err_msg(
            "Frame is {:.2f} s old (limit {:.2f} s); the pose read would not "
            "match it.".format(age, gates.max_frame_age_s))

    det = handeye.detect_board(first.data, board, detector, intrinsics,
                               min_coverage=gates.min_image_coverage)
    if det.T_target2cam is None:
        return Result.err_msg(det.message or "board pose not estimated")
    if not np.isfinite(det.reproj_error):
        return Result.err_msg("reprojection error is not finite")
    if det.reproj_error > gates.max_reproj_px:
        return Result.err_msg(
            "reprojection error {:.2f} px is above the {:.2f} px "
            "limit".format(det.reproj_error, gates.max_reproj_px))

    band = remount.check_distance_band(det.T_target2cam, gates.min_distance_m,
                                       gates.max_distance_m)
    if band.is_err:
        return Result.err_msg(band.message)

    # Board-static check: a second detection a moment later must agree. The
    # second look must be a genuinely NEW frame -- get_color() hands back
    # whatever the grab thread last stored, so on a slow stream the identical
    # image would be compared with itself, the shift would be exactly 0.0 and
    # the gate could never fail. A frame is new when its age is younger than
    # the time we have spent waiting for it.
    if gates.require_board_static:
        started = time.perf_counter()
        time.sleep(0.08)
        while camera.frame_age_s() >= time.perf_counter() - started:
            if time.perf_counter() - started > 1.0:
                return Result.err_msg(
                    "No new camera frame arrived within 1.0 s, so the "
                    "board-static check would compare one frame with itself. "
                    "The frame rate is too low for this gate -- raise it, or "
                    "shorten the exposure.")
            time.sleep(0.02)
        second_frame = camera.get_color()
        if second_frame.is_err:
            return Result.err_msg(second_frame.message)
        second = handeye.detect_board(second_frame.data, board, detector,
                                      intrinsics,
                                      min_coverage=gates.min_image_coverage)
        shift = handeye.corner_shift_px(det, second)
        if shift > gates.board_motion_px:
            return Result.err_msg(
                "board moved {:.1f} px between frames (limit {:.1f} px) -- "
                "still settling, or something is touching it".format(
                    shift, gates.board_motion_px))

    pose = None
    T_mount2base = np.eye(4)
    if api is not None:
        state = api.latest(max_age_s=gates.max_frame_age_s)
        if state.is_err:
            return Result.err_msg(state.message)
        pose = state.data.to_robot_pose()
        T_mount2base = pose.to_se3(convention)

    sample = handeye.HandEyeSample(
        T_mount2base=T_mount2base, T_target2cam=det.T_target2cam,
        reproj_error=det.reproj_error, n_corners=det.n_corners,
        pose=pose, pose_index=pose_index)
    overlay = handeye.draw_overlay(first.data, det, intrinsics)
    return Result.ok((sample, det, first.data, overlay))


class CallableWorker(QThread):
    """Run one blocking callable off the UI thread and deliver its return."""

    done = Signal(object)

    def __init__(self, fn, *args, parent=None, **kwargs) -> None:
        super().__init__(parent)
        self._fn = fn
        self._args = args
        self._kwargs = kwargs
        self.result = None

    def run(self) -> None:
        self.result = self._fn(*self._args, **self._kwargs)
        self.done.emit(self.result)


class AutoCalibWorker(QThread):
    """Replay an authored project and capture a sample at each pose."""

    progress = Signal(int, int, str)        # (done, total, message)
    log = Signal(str, str)                  # (message, level)
    pose_finished = Signal(object)          # PoseOutcome
    confirm_required = Signal(int, str)     # (pose index, question)
    run_finished = Signal(object)           # Result[list[PoseOutcome]]

    def __init__(self, api: Nova5Api, camera: CameraBase,
                 poses: Sequence[CalibPose], board, detector,
                 intrinsics: CameraIntrinsics, convention: PoseConvention,
                 safety: SafetyConfig, gates: CaptureGates,
                 delta: Optional[np.ndarray] = None,
                 X_prev: Optional[np.ndarray] = None,
                 confirm_each: bool = True, refine: bool = True,
                 skip_failed: bool = True, move_timeout_s: float = 30.0,
                 tool_index: int = 0, user_index: int = 0,
                 parent=None) -> None:
        super().__init__(parent)
        self._api = api
        self._camera = camera
        self._poses = list(poses)
        self._board = board
        self._detector = detector
        self._intrinsics = intrinsics
        self._convention = convention
        self._safety = safety
        self._gates = gates
        self._delta = delta
        self._X_prev = X_prev
        self._confirm_each = confirm_each
        self._refine = refine
        self._skip_failed = skip_failed
        self._move_timeout_s = move_timeout_s
        self._tool = int(tool_index)
        self._user = int(user_index)

        self._abort = threading.Event()
        self._confirmed = threading.Event()
        self._confirm_answer = False
        self.outcomes: List[PoseOutcome] = []

    # ------------------------------------------------------------- control
    def abort(self) -> None:
        self._abort.set()
        self._confirmed.set()

    def answer_confirm(self, accepted: bool) -> None:
        """Called from the UI thread in response to confirm_required."""
        self._confirm_answer = bool(accepted)
        self._confirmed.set()

    def _ask(self, index: int, question: str) -> bool:
        if not self._confirm_each:
            return True
        self._confirmed.clear()
        self.confirm_required.emit(index, question)
        while not self._confirmed.wait(timeout=0.1):
            if self._abort.is_set():
                return False
        return self._confirm_answer and not self._abort.is_set()

    # ---------------------------------------------------------------- move
    def _goto(self, T_target: np.ndarray,
              seed_joints: Optional[Sequence[float]]) -> Result[None]:
        """Move to a pose, preferring joint space so the arm configuration is
        reproduced rather than re-chosen."""
        command = self._convention.se3_to_command(T_target)
        state = self._api.latest()
        if state.is_err:
            return Result.err_msg(state.message)

        joints = None
        solved = self._api.inverse_solution(command, self._user, self._tool,
                                            seed_joints)
        if solved.is_ok and len(solved.data) == 6:
            joints = solved.data
        else:
            self.log.emit(
                "IK seed unavailable for this pose ({}); falling back to a "
                "Cartesian move, which may pick a different arm "
                "configuration.".format(solved.message), "WARNING")

        gate = check_move(command, state.data, self._safety, joints)
        if gate.is_err:
            return gate

        moved = (self._api.joint_move(joints) if joints is not None
                 else self._api.move_l(command, self._user, self._tool))
        if moved.is_err:
            return moved
        return self._api.wait_until_idle(self._move_timeout_s,
                                         self._gates.settle_s)

    def _refine_pose(self, pose: CalibPose,
                     X_guess: Optional[np.ndarray]) -> Result[np.ndarray]:
        """Servo onto this pose's stored reference view, returning the pose
        actually reached. Residual work only -- the closed-form delta has
        already done the heavy lifting.

        A servo that runs out of iterations without converging returns an
        error carrying that same reached pose, with the remaining view error
        in the message.
        """
        if X_guess is None or pose.T_board2cam_ref is None:
            return Result.err_msg("nothing to refine against")
        converged = False
        for _ in range(remount.max_servo_iterations()):
            if self._abort.is_set():
                return Result.err_msg("aborted")
            frame = self._camera.get_color()
            if frame.is_err:
                return Result.err_msg(frame.message)
            det = handeye.detect_board(
                frame.data, self._board, self._detector, self._intrinsics,
                min_coverage=self._gates.min_image_coverage)
            if det.T_target2cam is None:
                return Result.err_msg(det.message or "board not detected")
            if remount.is_converged(det.T_target2cam, pose.T_board2cam_ref):
                converged = True
                break
            step = remount.servo_step(det.T_target2cam, pose.T_board2cam_ref,
                                      X_guess)
            current = self._api.pose(self._convention)
            if current.is_err:
                return Result.err_msg(current.message)
            target = remount.apply_delta(current.data, step)
            moved = self._goto(target, None)
            if moved.is_err:
                return moved
        reached = self._api.pose(self._convention)
        if reached.is_err:
            return Result.err_msg(reached.message)
        if not converged:
            # Running out of iterations is not success. Measure where the
            # servo actually left the view and hand the numbers back: the
            # caller logs them as a warning and the run carries on, but a
            # pose that never settled must not read as clean.
            frame = self._camera.get_color()
            if frame.is_err:
                return Result.err(reached.data, frame.message)
            det = handeye.detect_board(
                frame.data, self._board, self._detector, self._intrinsics,
                min_coverage=self._gates.min_image_coverage)
            if det.T_target2cam is None:
                return Result.err(
                    reached.data,
                    "servo ran out of iterations and the board is no longer "
                    "detected, so the remaining view error is unknown")
            d_mm, d_deg = remount.view_error(det.T_target2cam,
                                             pose.T_board2cam_ref)
            if not remount.is_converged(det.T_target2cam,
                                        pose.T_board2cam_ref):
                return Result.err(
                    reached.data,
                    "servo did not converge in {} iterations: the view is "
                    "still {:.1f} mm / {:.2f} deg from the reference".format(
                        remount.max_servo_iterations(), d_mm, d_deg))
        return Result.ok(reached.data)

    # ----------------------------------------------------------------- run
    def run(self) -> None:
        """Guarded entry point. The walk itself is _walk_poses()."""
        try:
            self._walk_poses()
        except Exception as exc:
            # Deliberately broad: this is the top of a thread, so there is no
            # caller to propagate to. An escaping exception would kill the
            # thread with run_finished never emitted, and the tab would sit
            # with Run disabled and Stop enabled for the rest of the session
            # with no way back. Report it as a failed run instead.
            self.run_finished.emit(Result.err(
                self.outcomes,
                "Run stopped by an unexpected error -- {}: {}".format(
                    type(exc).__name__, exc)))

    def _walk_poses(self) -> None:
        total = len(self._poses)
        X_guess = self._X_prev
        if self._delta is not None and self._X_prev is not None:
            # One anchor is enough to turn a stale X into the current one.
            X_guess = remount.camera_from_delta(self._delta, self._X_prev)

        for step, pose in enumerate(self._poses):
            if self._abort.is_set():
                self.run_finished.emit(Result.err(
                    self.outcomes, "Run aborted after {} of {} poses.".format(
                        step, total)))
                return
            label = pose.name or "pose {}".format(pose.index)
            self.progress.emit(step, total, "moving to {}".format(label))

            if pose.T_mount2base is None:
                self._record(PoseOutcome(pose.index, False,
                                         "pose has no stored transform"))
                continue

            target = (remount.apply_delta(pose.T_mount2base, self._delta)
                      if self._delta is not None else pose.T_mount2base)
            seed = pose.pose.joints if pose.pose is not None else None

            if not self._ask(pose.index,
                             "Move to {} and capture?".format(label)):
                self.run_finished.emit(Result.err(
                    self.outcomes,
                    "Stopped by the operator at {}.".format(label)))
                return

            moved = self._goto(target, seed)
            if self._abort.is_set():
                # STOP arrived while the arm was moving. ResetRobot clears the
                # queue, which makes wait_until_idle report a clean finish for
                # the move that was just interrupted -- so the abort flag, not
                # the move's Result, is what has to end the run here. Capturing
                # at an interrupted position would be worse than no sample.
                self.run_finished.emit(Result.err(
                    self.outcomes,
                    "Run aborted during the move to {}.".format(label)))
                return
            if moved.is_err:
                self._record(PoseOutcome(pose.index, False, moved.message))
                if not self._skip_failed:
                    self.run_finished.emit(Result.err(
                        self.outcomes,
                        "Stopped at {}: {}".format(label, moved.message)))
                    return
                continue

            if self._refine and X_guess is not None:
                refined = self._refine_pose(pose, X_guess)
                if refined.is_err:
                    self.log.emit("{}: refine skipped ({})".format(
                        label, refined.message), "WARNING")

            self.progress.emit(step, total, "capturing at {}".format(label))
            captured = capture_sample(
                self._camera, self._api, self._board, self._detector,
                self._intrinsics, self._convention, self._gates, pose.index)
            if captured.is_err:
                self._record(PoseOutcome(pose.index, False, captured.message))
                if not self._skip_failed:
                    self.run_finished.emit(Result.err(
                        self.outcomes,
                        "Stopped at {}: {}".format(label, captured.message)))
                    return
                continue

            sample, det, raw, overlay = captured.data
            outcome = PoseOutcome(pose.index, True, "captured", sample=sample,
                                  raw_bgr=raw, overlay_bgr=overlay)
            # Every reached pose is an independent estimate of the remount.
            outcome.delta = remount.delta_from_reteach(pose.T_mount2base,
                                                       sample.T_mount2base)
            if pose.T_board2cam_ref is not None:
                d_mm, d_deg = remount.view_error(det.T_target2cam,
                                                 pose.T_board2cam_ref)
                outcome.view_error_mm = d_mm
                outcome.view_error_deg = d_deg
            self._record(outcome)

        self.progress.emit(total, total, "run complete")
        good = sum(1 for o in self.outcomes if o.ok)
        if good == 0:
            self.run_finished.emit(Result.err(
                self.outcomes, "No pose produced a usable sample."))
            return
        self.run_finished.emit(Result.ok(self.outcomes))

    def _record(self, outcome: PoseOutcome) -> None:
        self.outcomes.append(outcome)
        level = "SUCCESS" if outcome.ok else "WARNING"
        self.log.emit("pose {}: {}".format(outcome.index, outcome.message),
                      level)
        self.pose_finished.emit(outcome)


class AnchorWorker(QThread):
    """Measure the remount delta at the pose the arm is currently at.

    The operator has already jogged until the ghosted reference corners line
    up; this reads the pose back and does the arithmetic. Separate from the
    auto run because it is the step that has to happen first, by hand, and
    because it is the one that bootstraps everything else.
    """

    done = Signal(object)               # Result[remount.DeltaEstimate]

    def __init__(self, api: Nova5Api, camera: CameraBase, pose: CalibPose,
                 board, detector, intrinsics: CameraIntrinsics,
                 convention: PoseConvention,
                 min_coverage: Optional[float] = None, parent=None) -> None:
        super().__init__(parent)
        self._api = api
        self._camera = camera
        self._pose = pose
        self._board = board
        self._detector = detector
        self._intrinsics = intrinsics
        self._convention = convention
        # Same floor as the capture gate, so "anchored but not detectable"
        # and "capturable" cannot disagree about the same view.
        self._min_coverage = min_coverage

    def run(self) -> None:
        if self._pose.T_mount2base is None:
            self.done.emit(Result.err_msg(
                "The anchor pose has no stored transform to compare against."))
            return
        current = self._api.pose(self._convention)
        if current.is_err:
            self.done.emit(Result.err_msg(current.message))
            return

        estimate = remount.DeltaEstimate(
            pose_index=self._pose.index,
            delta=remount.delta_from_reteach(self._pose.T_mount2base,
                                             current.data))
        frame = self._camera.get_color()
        if frame.is_ok and self._pose.T_board2cam_ref is not None:
            det = handeye.detect_board(frame.data, self._board, self._detector,
                                       self._intrinsics,
                                       min_coverage=self._min_coverage)
            if det.T_target2cam is not None:
                d_mm, d_deg = remount.view_error(det.T_target2cam,
                                                 self._pose.T_board2cam_ref)
                estimate.residual_mm = d_mm
                estimate.residual_deg = d_deg
                if not remount.is_converged(det.T_target2cam,
                                            self._pose.T_board2cam_ref):
                    self.done.emit(Result.err(
                        estimate,
                        "Anchored, but the live view is still {:.1f} mm / "
                        "{:.2f} deg from the reference. Every pose inherits "
                        "this error -- keep jogging until the ghosted corners "
                        "sit on the live ones.".format(d_mm, d_deg)))
                    return
        self.done.emit(Result.ok(estimate))