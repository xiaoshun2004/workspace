# -*- coding: utf-8 -*-
"""Regression tests for the Nova-5 calibration core.

These are a specification of the invariants, not a smoke test. Each assertion
below corresponds to a claim made in a docstring somewhere in core/, and the
remount block in particular is the arithmetic proof that the correction form
used by the app is the right one and the tempting alternative is not.

Run: python3 tests/test_core.py
"""

import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "core"))
# api/ too: nova5_api imports numpy and core/models only, no SDK, so the
# reply parser can be tested here without hardware or stubs.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "api"))

import cv2
import numpy as np

import handeye
import intrinsics_math as im
import nova5_api
import pose_convention as pc
import project_io
import remount
from models import (
    CalibInstance,
    CalibPose,
    CalibProject,
    CameraConfig,
    CameraIntrinsics,
    CaptureMode,
    CharucoBoardConfig,
    FrameIdentity,
    IntrinsicsRecord,
    IntrinsicsSource,
    PoseConvention,
    RobotPose,
    euler_to_matrix,
)
from transforms import make_se3, rotation_angle_deg, se3_inverse

rng = np.random.default_rng(20260805)


def rand_se3(t_scale=0.4):
    rvec = rng.normal(size=3)
    rvec = rvec / np.linalg.norm(rvec) * rng.uniform(0.2, 2.0)
    R, _ = cv2.Rodrigues(rvec)
    return make_se3(R, rng.uniform(-t_scale, t_scale, size=3))


def view_of(X, G, B):
    """T_board2cam for camera-in-mount X, mount pose G, board pose B."""
    return se3_inverse(X) @ se3_inverse(G) @ B


def se3_close(A, B, tol=1e-9):
    return float(np.max(np.abs(np.asarray(A) - np.asarray(B)))) < tol


# =========================================================================
# 1. Remount algebra -- the load-bearing claim of the whole design
# =========================================================================
X = rand_se3(0.08)              # camera on the flange, before
X_new = rand_se3(0.08)          # camera on the flange, after a remount
B = rand_se3(0.8)               # board, fixed in the base frame
poses = [rand_se3(0.5) for _ in range(7)]

views_ref = [view_of(X, G, B) for G in poses]

# The remount shifts every view by ONE transform, identical at every pose.
shift = None
for G, ref in zip(poses, views_ref):
    got = se3_inverse(view_of(X_new, G, B) @ se3_inverse(ref))
    if shift is None:
        shift = got
    assert se3_close(shift, got), "view shift is not pose-independent"
assert se3_close(shift, se3_inverse(X) @ X_new)
print("PASS remount shifts every view by the same transform (X^-1 X')")

# The correction G_i -> G_i @ delta restores every view EXACTLY.
delta_true = X @ se3_inverse(X_new)
for G, ref in zip(poses, views_ref):
    assert se3_close(view_of(X_new, remount.apply_delta(G, delta_true), B), ref)
print("PASS right-multiplying by delta = X.X'^-1 reproduces every view exactly")

# delta is recoverable from a single re-teach, without knowing X or X'.
G_home = poses[0]
G_home_new = remount.apply_delta(G_home, delta_true)
assert se3_close(remount.delta_from_reteach(G_home, G_home_new), delta_true)
print("PASS delta recovered from one re-teach: G_ref^-1 . G_new")

# Every pose is an anchor: each yields the SAME delta.
estimates = [remount.DeltaEstimate(i, remount.delta_from_reteach(
    G, remount.apply_delta(G, delta_true))) for i, G in enumerate(poses)]
spread_mm, spread_deg = remount.delta_spread(estimates)
# 1e-4 deg, not 0: rotation_angle_deg goes through arccos near 1, where
# the trace formula loses about half its significant digits.
assert spread_mm < 1e-6 and spread_deg < 1e-4, (spread_mm, spread_deg)
assert se3_close(remount.mean_delta(estimates), delta_true)
rec = remount.summarise(estimates)
assert "consistent" in rec.verdict
print("PASS every pose is an anchor; per-pose estimates agree to "
      "{:.1e} mm / {:.1e} deg".format(spread_mm, spread_deg))

# The tempting alternative -- store home-relative poses, replay from a
# re-taught home -- is WRONG, because delta does not commute with the
# stored relative poses.
worst = 0.0
for G, ref in zip(poses[1:], views_ref[1:]):
    naive = G_home_new @ (se3_inverse(G_home) @ G)
    got = view_of(X_new, naive, B)
    worst = max(worst, float(np.linalg.norm(got[:3, 3] - ref[:3, 3])) * 1000.0)
assert worst > 1.0, "relative-replay error should be large, got {}".format(worst)
print("PASS home-relative replay is rejected: it misses by up to "
      "{:.0f} mm on this set".format(worst))

# A moved board produces a pose-DEPENDENT delta -- the diagnostic that tells
# the two failure modes apart.
B_moved = rand_se3(0.05) @ B
moved = []
for i, G in enumerate(poses):
    # exact correction if only the board moved: left multiplication
    G_fix = (B_moved @ se3_inverse(B)) @ G
    moved.append(remount.DeltaEstimate(i, remount.delta_from_reteach(G, G_fix)))
mm, deg = remount.delta_spread(moved)
assert mm > 3.0 or deg > 1.0, (mm, deg)
assert "INCONSISTENT" in remount.summarise(moved).verdict
print("PASS a moved board is caught by the spread ({:.0f} mm / {:.1f} deg "
      "scatter)".format(mm, deg))


# =========================================================================
# 2. Auto-realign servo
# =========================================================================
# Exact X: converges in a single step.
G = poses[3]
ref = views_ref[3]
step = remount.servo_step(view_of(X_new, G, B), ref, X_new,
                          max_step_mm=1e9, max_step_deg=1e9)
assert se3_close(view_of(X_new, remount.apply_delta(G, step), B), ref, 1e-9)
print("PASS servo step is exact when X is known")

def run_servo(X_true, X_guess, G_start, target):
    G_live = G_start
    used = 0
    for _ in range(remount.max_servo_iterations()):
        live = view_of(X_true, G_live, B)
        if remount.is_converged(live, target):
            break
        G_live = remount.apply_delta(
            G_live, remount.servo_step(live, target, X_guess))
        used += 1
    return G_live, used


# A small remount -- bracket nudged, not re-clocked -- converges from the
# previous instance's X directly.
X_nudged = X @ make_se3(cv2.Rodrigues(np.array([0.02, -0.03, 0.05]))[0],
                        np.array([0.004, -0.002, 0.003]))
G_live, iters = run_servo(X_nudged, X, poses[3],
                          view_of(X, poses[3], B))
d_mm, d_deg = remount.view_error(view_of(X_nudged, G_live, B),
                                 view_of(X, poses[3], B))
assert remount.is_converged(view_of(X_nudged, G_live, B),
                            view_of(X, poses[3], B)), (d_mm, d_deg)
print("PASS small remount: servo converges from a stale X in {} iterations "
      "({:.3f} mm / {:.3f} deg)".format(iters, d_mm, d_deg))

# A large remount does NOT converge from a stale X -- the step is only a
# contraction near the truth. This is why the first anchor is manual.
_G, _n = run_servo(X_new, X, poses[3], ref)
assert not remount.is_converged(view_of(X_new, _G, B), ref)
print("PASS large remount is honestly reported as not converging from a "
      "stale X")

# ...and one hand-matched anchor removes the need to servo that gap at all:
# delta corrects every pose in closed form, and yields an exact X' on the way.
X_bootstrapped = remount.camera_from_delta(delta_true, X)
assert se3_close(X_bootstrapped, X_new)
for i in (1, 4, 6):
    corrected = remount.apply_delta(poses[i], delta_true)
    assert se3_close(view_of(X_new, corrected, B), views_ref[i])
    # Arriving already on target, the servo asks for no motion at all.
    G_live, iters = run_servo(X_new, X_bootstrapped, corrected, views_ref[i])
    assert iters == 0 and se3_close(G_live, corrected)
print("PASS one manual anchor corrects every pose in closed form and "
      "bootstraps the new X exactly")

# The servo is left with only residual work, which fits inside the step clamp.
nudge = make_se3(cv2.Rodrigues(np.array([0.0, 0.05, 0.0]))[0],
                 np.array([0.005, 0.0, 0.0]))
G_off = remount.apply_delta(poses[4], delta_true) @ nudge
G_live, iters = run_servo(X_new, X_bootstrapped, G_off, views_ref[4])
assert remount.is_converged(view_of(X_new, G_live, B), views_ref[4])
print("PASS servo cleans up residual misalignment in {} iteration(s) inside "
      "the step clamp".format(iters))

# Clamping never exceeds the limits and never reverses direction.
big = make_se3(cv2.Rodrigues(np.array([0.0, 0.0, 1.2]))[0],
               np.array([0.5, 0.0, 0.0]))
clamped = remount.clamp_delta(big, 40.0, 12.0)
assert np.linalg.norm(clamped[:3, 3]) * 1000.0 <= 40.0 + 1e-9
assert rotation_angle_deg(np.eye(3), clamped[:3, :3]) <= 12.0 + 1e-9
assert float(np.dot(clamped[:3, 3], big[:3, 3])) > 0.0
print("PASS clamped steps respect both limits and keep their direction")


# =========================================================================
# 3. Pose convention solver
# =========================================================================
TRUTH = PoseConvention(order="xyz", intrinsic=False, degrees=True,
                       quat_order="wxyz")


def matrix_to_quat_wxyz(R):
    rvec, _ = cv2.Rodrigues(np.asarray(R, dtype=np.float64))
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    axis = (rvec.reshape(3) / theta)
    s = np.sin(theta / 2.0)
    return [float(np.cos(theta / 2.0)), float(axis[0] * s),
            float(axis[1] * s), float(axis[2] * s)]


def synth_poses(convention, triples):
    out = []
    for (rx, ry, rz) in triples:
        R = convention.rotation(rx, ry, rz)
        out.append(RobotPose(x=300.0, y=0.0, z=250.0, rx=rx, ry=ry, rz=rz,
                             quat=matrix_to_quat_wxyz(R),
                             tool_index=1, user_index=0,
                             tool_frame=[0.0] * 6, user_frame=[0.0] * 6,
                             source="feedback"))
    return out


varied = [(180.0, 0.0, 0.0), (150.0, 30.0, -40.0), (-170.0, -25.0, 80.0),
          (120.0, 10.0, 170.0), (-90.0, 45.0, 20.0)]
res = pc.solve_convention(synth_poses(TRUTH, varied))
assert res.is_ok, res.message
best = res.data[0].convention
assert best.verified
assert best.quat_order == "wxyz"
assert res.data[0].max_error_deg < 1e-4
# Reconstructs the same rotations, whatever notation it settled on.
# Tolerances on angles are 1e-4 deg rather than 1e-9 throughout: the
# geodesic angle goes through arccos near 1, which caps the achievable
# precision at roughly the square root of machine epsilon.
for p in synth_poses(TRUTH, varied):
    assert rotation_angle_deg(best.rotation(p.rx, p.ry, p.rz),
                              TRUTH.rotation(p.rx, p.ry, p.rz)) < 1e-4
print("PASS convention solved from quaternions: {}".format(best.label()))

# Rotation about ONE axis passes the spread gate but does not distinguish the
# candidates: several genuinely different conventions reproduce it exactly.
# They are not one rotation written in different notations -- the angles are
# assigned positionally, so a co-fitting candidate here disagrees elsewhere --
# and answering with whichever sorted first is how a wrong convention gets
# stamped verified.
single_axis = [(0.0, 0.0, 0.0), (0.0, 0.0, 37.512), (0.0, 0.0, 91.004),
               (0.0, 0.0, -63.229), (0.0, 0.0, 148.771)]
ambiguous = pc.solve_convention(synth_poses(TRUTH, single_axis))
assert ambiguous.is_err, "single-axis samples must not verify a convention"
assert "ambiguous" in ambiguous.message.lower(), ambiguous.message
print("PASS a single-axis sample set is refused as ambiguous, not answered")

# Degrees vs radians is never confused: the same numbers read as radians
# describe wildly different rotations.
rad_truth = PoseConvention(order="xyz", intrinsic=False, degrees=False,
                           quat_order="wxyz")
small = [(2.5, 0.4, -1.1), (1.2, -2.0, 0.7), (-0.6, 1.9, 2.4),
         (3.0, 0.9, -2.2)]
res_rad = pc.solve_convention(synth_poses(rad_truth, small))
assert res_rad.is_ok, res_rad.message
assert res_rad.data[0].convention.degrees is False
print("PASS radian-reporting controller is identified as radians")

# The xyzw / wxyz bit is pinned down too.
xyzw_truth = PoseConvention(order="xyz", intrinsic=False, degrees=True,
                            quat_order="xyzw")
flipped = []
for p in synth_poses(TRUTH, varied):
    w, x, y, z = p.quat
    p.quat = [x, y, z, w]
    flipped.append(p)
res_flip = pc.solve_convention(flipped)
assert res_flip.is_ok, res_flip.message
assert res_flip.data[0].convention.quat_order == "xyzw"
print("PASS quaternion component order is identified, not assumed")

# Degenerate input is refused rather than answered.
flat = pc.solve_convention(synth_poses(TRUTH, [(1.0, 0.5, 0.2),
                                               (2.0, 0.5, 0.3),
                                               (1.5, 0.6, 0.1)]))
assert flat.is_err and "differ by only" in flat.message
assert pc.solve_convention(synth_poses(TRUTH, varied[:2])).is_err
print("PASS near-identical or too-few samples are refused, not guessed")

# A frame mismatch is diagnosed rather than silently mis-solved.
R_tool_off = euler_to_matrix([np.radians(30.0), 0.0, 0.0], "xyz", False)
mismatched = []
for p in synth_poses(TRUTH, varied):
    R_pose = TRUTH.rotation(p.rx, p.ry, p.rz)
    p.quat = matrix_to_quat_wxyz(R_pose @ R_tool_off.T)
    p.tool_frame = [0.0, 0.0, 0.0, 30.0, 0.0, 0.0]
    mismatched.append(p)
bad = pc.solve_convention(mismatched)
assert bad.is_err and "different frames" in bad.message
diag = pc.diagnose_frame_mismatch(mismatched)
assert diag.is_ok and "R_pose * R_tool^-1" in diag.data
print("PASS quaternion-on-a-different-frame is detected and named")

# The independent 90-degree cross-check agrees, and rejects a wrong guess.
before = synth_poses(TRUTH, [(0.0, 0.0, 0.0)])[0]
R_after = TRUTH.rotation(0.0, 0.0, 0.0) @ euler_to_matrix(
    [0.0, 0.0, np.radians(90.0)], "xyz", False)
rvec, _ = cv2.Rodrigues(R_after)
after = RobotPose(rx=0.0, ry=0.0, rz=90.0, quat=matrix_to_quat_wxyz(R_after))
assert pc.verify_against_known_move(before, after, "z", 90.0, TRUTH).is_ok
wrong = PoseConvention(order="xyz", intrinsic=False, degrees=False)
assert pc.verify_against_known_move(before, after, "z", 90.0, wrong).is_err
print("PASS commanded-move cross-check confirms good and rejects bad")

# The dominant error class is an axis permutation, which reconstructs the
# right MAGNITUDE about the wrong axis: extrinsic "zyx" reads (0, 0, 90) as
# Rx(90). Checking only the angle would confirm it.
axis_swapped = PoseConvention(order="zyx", intrinsic=False, degrees=True)
swapped_check = pc.verify_against_known_move(before, after, "z", 90.0,
                                             axis_swapped)
assert swapped_check.is_err, "90 deg about x must not confirm a z command"
print("PASS the cross-check rejects the right angle about the wrong axis")


# =========================================================================
# 4. Distortion, intrinsics, board geometry (carried over, still true)
# =========================================================================
W, H, FX, FY, PPX, PPY = 1280, 720, 674.0, 674.0, 640.0, 360.0
coeffs = [0.12, -0.25, 0.001, -0.0008, 0.09]
assert im.roundtrip_error_px(coeffs, W, H, FX, FY, PPX, PPY) < 1e-9
intr = CameraIntrinsics(width=W, height=H, fx=FX, fy=FY, ppx=PPX, ppy=PPY,
                        coeffs=coeffs, model="inverse_brown_conrady")
assert intr.needs_manual_undistort() and not intr.is_opencv_compatible()
assert CameraIntrinsics.from_dict(intr.to_dict()).to_dict() == intr.to_dict()
assert 80.0 < intr.hfov_deg() < 95.0, intr.hfov_deg()
print("PASS inverse-Brown-Conrady handled exactly; HFOV {:.1f} deg".format(
    intr.hfov_deg()))

# The ideal -> observed direction is what draws an overlay back onto the raw
# frame, so it has to hold up at the distortion a wide-FOV module really
# reports. A fixed-point update diverges past |k1| ~ 0.25 at the frame corner;
# the damped Newton solve does not.
STRONG = [0.30, 0.0, 0.0, 0.0, 0.0]
strong_err = im.roundtrip_error_px(STRONG, 1280, 720, 640.0, 640.0,
                                   640.0, 360.0)
assert strong_err < 1e-6, strong_err
print("PASS distortion round-trip holds at k1 = 0.30 ({:.1e} px)".format(
    strong_err))

# And where the polynomial genuinely has no usable inverse -- beyond its fold,
# so the ideal point has no observed preimage on the physical branch -- that is
# raised, not returned as a plausible number.
try:
    im.observed_from_ideal(1200.0, 360.0, [-1.5, 0.0, 0.0, 0.0, 0.0],
                           640.0, 640.0, 640.0, 360.0)
    raise AssertionError("an unrecoverable inverse must not return quietly")
except ValueError as exc:
    assert "converge" in str(exc), str(exc)
print("PASS an unrecoverable distortion inverse raises instead of guessing")

board = CharucoBoardConfig(squares_x=8, squares_y=5, square_length_m=0.03,
                           marker_length_m=0.022, dictionary="DICT_5X5_100")
g = handeye.board_geometry(board)
assert g["width_m"] == 0.24 and g["interior_corners_x"] == 7
print("PASS board geometry records counts and physical size")


# =========================================================================
# 5. View margin and distance band
# =========================================================================
centred = np.array([[540.0, 260.0], [740.0, 260.0],
                    [740.0, 460.0], [540.0, 460.0]])
edge = np.array([[20.0, 260.0], [220.0, 260.0],
                 [220.0, 460.0], [20.0, 460.0]])
m_centred = remount.view_margin_deg(centred, W, H, intr)
m_edge = remount.view_margin_deg(edge, W, H, intr)
assert m_centred > m_edge > 0.0
assert 15.0 < m_centred < 35.0, m_centred
print("PASS view margin: centred board {:.1f} deg, board near the edge "
      "{:.1f} deg".format(m_centred, m_edge))
assert "pose 1" in remount.project_margin_report([m_centred, m_edge])

near = make_se3(np.eye(3), np.array([0.0, 0.0, 0.05]))
good = make_se3(np.eye(3), np.array([0.0, 0.0, 0.25]))
far = make_se3(np.eye(3), np.array([0.0, 0.0, 0.90]))
assert remount.check_distance_band(near, 0.12, 0.45).is_err
assert remount.check_distance_band(good, 0.12, 0.45).is_ok
assert remount.check_distance_band(far, 0.12, 0.45).is_err
print("PASS working-distance band gates both ends and reports the distance")


# =========================================================================
# 6. Persistence
# =========================================================================
tmp = tempfile.mkdtemp()
try:
    project = CalibProject(name="bench rig", board=board,
                           camera=CameraConfig(ident="123456"),
                           convention=TRUTH,
                           frames=FrameIdentity(tool_index=1, user_index=0,
                                                tool_frame=[0.0] * 6,
                                                user_frame=[0.0] * 6))
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    for i, G in enumerate(poses[:3]):
        saved = project_io.save_pose_images(tmp, project.name, i, frame, frame)
        assert saved.is_ok, saved.message
        raw, overlay = saved.data
        project.poses.append(CalibPose(
            index=i, is_home=(i == 0), T_mount2base=G,
            T_board2cam_ref=views_ref[i], n_corners=24, reproj_error=0.31,
            ref_corners=centred.astype(np.float32).reshape(-1, 1, 2),
            ref_ids=np.arange(4, dtype=np.int32).reshape(-1, 1),
            margin_deg=m_centred, image_path=raw, overlay_path=overlay,
            pose=RobotPose(x=300.0, y=10.0, z=250.0, rx=180.0, ry=0.0, rz=0.0,
                           quat=[1.0, 0.0, 0.0, 0.0], tool_index=1,
                           user_index=0, source="feedback")))
    assert project_io.save_project(tmp, project).is_ok

    # Stored paths are relative, so the folder can be moved or shared.
    raw_json = project_io.read_json(
        project_io.ProjectPaths(tmp, project.name).json_path).data
    assert not os.path.isabs(raw_json["poses"][0]["image_path"])

    loaded = project_io.load_project(tmp, project.name)
    assert loaded.is_ok, loaded.message
    lp = loaded.data
    assert lp.home().is_home and lp.home().index == 0
    assert se3_close(lp.poses[1].T_mount2base, poses[1])
    assert se3_close(lp.poses[1].T_board2cam_ref, views_ref[1])
    assert os.path.exists(lp.poses[0].image_path)
    assert lp.convention.to_dict() == TRUTH.to_dict()
    assert lp.poses[0].ref_corners.shape == (4, 1, 2)
    print("PASS project round-trips with relative image paths and reference "
          "views")

    ipaths = project_io.new_instance_paths(tmp, project.name, "run1")
    samples = []
    for i, (G, v) in enumerate(zip(poses, views_ref)):
        img = project_io.save_sample_images(ipaths, i, frame, frame)
        assert img.is_ok
        samples.append(handeye.HandEyeSample(
            T_mount2base=G, T_target2cam=v, reproj_error=0.4, n_corners=24,
            pose_index=i, image_path=img.data[0], overlay_path=img.data[1],
            pose=RobotPose(x=1.0, y=2.0, z=3.0, rx=180.0, ry=0.0, rz=0.0,
                           quat=[1.0, 0.0, 0.0, 0.0], tool_index=1,
                           user_index=0)))

    solved = handeye.solve_handeye(samples, method="TSAI")
    assert solved.is_ok, solved.message
    err_mm = float(np.linalg.norm(
        solved.data.T_cam2mount[:3, 3] - X[:3, 3])) * 1000.0
    err_deg = rotation_angle_deg(solved.data.T_cam2mount[:3, :3], X[:3, :3])
    assert err_mm < 1e-3 and err_deg < 1e-4, (err_mm, err_deg)
    print("PASS solver recovers the synthetic X to {:.1e} mm / "
          "{:.1e} deg".format(err_mm, err_deg))

    instance = CalibInstance(
        name="", project_name=project.name, mode=CaptureMode.AUTO,
        board=board, camera=project.camera, intrinsics=intr,
        convention=TRUTH, frames=project.frames,
        remount=remount.summarise(estimates), result=solved.data)
    assert project_io.save_instance(ipaths, instance, samples).is_ok

    back = project_io.load_instance(ipaths.dir)
    assert back.is_ok, back.message
    bi, bs = back.data
    assert bi.result is not None
    assert se3_close(bi.result.T_cam2mount, solved.data.T_cam2mount)
    assert bi.intrinsics.resolution() == (W, H)
    assert bi.frames.tool_index == 1
    assert se3_close(bi.remount.delta, delta_true)
    assert len(bs) == len(samples) and os.path.exists(bs[0].image_path)
    assert bs[0].pose is not None and bs[0].pose.tool_index == 1
    print("PASS instance round-trips result, intrinsics, frames, remount and "
          "{} samples".format(len(bs)))

    found = project_io.latest_instance_with_result(tmp, project.name)
    assert found.is_ok and found.data[0].result is not None
    print("PASS latest solved instance is discoverable for seeding the servo")

    # Reopen it and re-solve from the stored samples alone -- the workflow for
    # captures made elsewhere, or for fixing a pose after the fact.
    reopened = project_io.load_instance(ipaths.dir)
    assert reopened.is_ok
    ri, rs = reopened.data
    assert len(rs) == len(samples)
    assert all(s.pose is not None for s in rs)
    assert all(os.path.exists(s.image_path) for s in rs)
    assert all(os.path.exists(s.overlay_path) for s in rs)
    assert ri.board.matches(board) and ri.convention.to_dict() == TRUTH.to_dict()
    resolved = handeye.solve_handeye(rs, method="TSAI")
    assert resolved.is_ok, resolved.message
    assert se3_close(resolved.data.T_cam2mount, solved.data.T_cam2mount, 1e-9)
    print("PASS a reopened instance re-solves to the same T_cam2mount")

    # A mis-typed pose is what you reopen an instance to fix. Note what it
    # does and does not do: with otherwise exactly consistent samples the
    # TSAI solution barely moves, because it is over-determined -- but the
    # board-origin spread explodes. That is the whole argument for reading the
    # spread rather than trusting that the solve returned something.
    good_T = rs[2].T_mount2base.copy()
    rs[2].pose.rx += 3.0
    rs[2].T_mount2base = rs[2].pose.to_se3(TRUTH, prefer_quaternion=False)
    broken = handeye.solve_handeye(rs, method="TSAI")
    assert broken.is_ok, broken.message
    assert broken.data.board_origin_spread_mm > 100.0, \
        broken.data.board_origin_spread_mm
    print("PASS one bad pose leaves the solve looking fine but blows the "
          "spread to {:.0f} mm".format(broken.data.board_origin_spread_mm))

    rs[2].T_mount2base = good_T
    fixed = handeye.solve_handeye(rs, method="TSAI")
    assert fixed.is_ok
    assert fixed.data.board_origin_spread_mm < 1e-6
    assert se3_close(fixed.data.T_cam2mount, solved.data.T_cam2mount, 1e-9)
    print("PASS correcting the pose and re-solving restores agreement")

    # Save back over the same folder rather than minting a new one.
    same = project_io.instance_paths(tmp, project.name, ipaths.folder)
    assert same.dir == ipaths.dir
    ri.result = fixed.data
    ri.note = "re-solved with a corrected pose"
    assert project_io.save_instance(same, ri, rs).is_ok
    again = project_io.load_instance(ipaths.dir)
    assert again.is_ok
    assert se3_close(again.data[0].result.T_cam2mount, fixed.data.T_cam2mount)
    assert again.data[0].note == "re-solved with a corrected pose"
    assert len(project_io.list_instances(tmp, project.name)) == 1
    print("PASS in-place re-save overwrites instead of forking a folder")

    # Forking instead keeps the provenance link.
    fork = project_io.new_instance_paths(tmp, project.name, "resolve")
    ri.derived_from = ipaths.folder
    assert project_io.save_instance(fork, ri, rs).is_ok
    forked = project_io.load_instance(fork.dir)
    assert forked.is_ok and forked.data[0].derived_from == ipaths.folder
    assert len(project_io.list_instances(tmp, project.name)) == 2
    print("PASS a forked re-solve records what it was derived from")

    # Validity gates.
    assert project_io.check_instance_applicable(bi, intr, project.frames).is_ok
    half = CameraIntrinsics(width=640, height=480, fx=FX / 2, fy=FY / 2,
                            ppx=PPX / 2, ppy=PPY / 2, coeffs=coeffs,
                            model=intr.model)
    bad_res = project_io.check_instance_applicable(bi, half, project.frames)
    assert bad_res.is_err and "resolution" in bad_res.message
    retaught = FrameIdentity(tool_index=1, user_index=0,
                             tool_frame=[0, 0, 12.0, 0, 0, 0],
                             user_frame=[0.0] * 6)
    bad_tool = project_io.check_instance_applicable(bi, intr, retaught)
    assert bad_tool.is_err and "re-taught" in bad_tool.message
    other_tool = project_io.check_instance_applicable(
        bi, intr, FrameIdentity(tool_index=2, user_index=0))
    assert other_tool.is_err and "Tool index changed" in other_tool.message
    print("PASS stored calibration is refused on resolution or TCP change")

    # Intrinsics library.
    lib = project_io.IntrinsicsLibrary(project_io.intrinsics_library_path(tmp))
    assert lib.find("realsense:123456", W, H) is None
    lib.put(IntrinsicsRecord(camera_key="realsense:123456", intrinsics=intr,
                             source=IntrinsicsSource.CALIBRATED,
                             rms_px=0.21, n_images=18))
    lib2 = project_io.IntrinsicsLibrary(
        project_io.intrinsics_library_path(tmp))
    rec2 = lib2.find("realsense:123456", W, H)
    assert rec2 is not None and rec2.rms_px == 0.21
    assert lib2.find("realsense:123456", 640, 480) is None
    lib2.put(IntrinsicsRecord(camera_key="realsense:123456", intrinsics=intr,
                              source=IntrinsicsSource.FACTORY))
    assert len(lib2.records()) == 1, "one record per camera+resolution"
    print("PASS intrinsics library is keyed by camera AND resolution")
finally:
    shutil.rmtree(tmp)


# =========================================================================
# 7. Controller reply parsing
# =========================================================================
# The controller echoes the command's arguments verbatim, so a seeded
# InverseSolution reply carries a SECOND brace pair -- the JointNear list. The
# payload is the first pair, not everything up to the last brace: reading to
# the last one swallows the echo, the floats fail to parse, and replay falls
# back to a Cartesian move with the joint gates skipped.
seeded = ("0,{0,0,-90,0,90,0},InverseSolution(473.0,-141.0,469.0,-180.0,0.0,"
          "-90.0,0,0,1,{0,0,-90,0,90,0});")
reply = nova5_api.parse_response(seeded)
assert reply.is_ok, reply.message
assert len(reply.data) == 6, reply.data
assert [float(v) for v in reply.data] == [0.0, 0.0, -90.0, 0.0, 90.0, 0.0], \
    reply.data
# The unseeded form, and an empty payload, still behave.
assert len(nova5_api.parse_response(
    "0,{473.0,-141.0,469.0},InverseSolution(473.0,-141.0,469.0);").data) == 3
assert nova5_api.parse_response("0,{},MovL(-500,100,200,150,0,90);").data == []
assert nova5_api.parse_response("-1,{},MovL(1,2,3,4,5,6);").is_err
print("PASS a seeded InverseSolution reply parses past the echoed JointNear "
      "braces")


# =========================================================================
# 8. Image-library import helpers
# =========================================================================
# The import workflow reads image files and a batch pose CSV; both readers
# must fail with a reason rather than hand garbage to the sample list.
tmp = tempfile.mkdtemp(prefix="nova5_import_")
try:
    # load_image: missing, undecodable, and a genuine round trip.
    assert project_io.load_image(os.path.join(tmp, "missing.png")).is_err
    bad = os.path.join(tmp, "not_an_image.png")
    with open(bad, "w", encoding="utf-8") as f:
        f.write("this is not a png")
    assert project_io.load_image(bad).is_err
    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    frame[10:20, 10:30] = (0, 200, 0)
    good = os.path.join(tmp, "frame.png")
    assert project_io.save_image(good, frame).is_ok
    loaded = project_io.load_image(good)
    assert loaded.is_ok and loaded.data.shape == frame.shape
    assert np.array_equal(loaded.data, frame)
    print("PASS load_image round-trips and names both failure modes")

    # parse_pose_csv: in-order shape, with comment, header and blank lines.
    plain = os.path.join(tmp, "poses_plain.csv")
    with open(plain, "w", encoding="utf-8") as f:
        f.write("# poses jotted down from DobotStudio\n"
                "x,y,z,rx,ry,rz\n"
                "\n"
                "400.0,-120.5,300.0,180.0,0.0,-90.0\n"
                "410.0,-118.0,305.0,175.0,3.0,-88.0\n")
    rows = project_io.parse_pose_csv(plain)
    assert rows.is_ok, rows.message
    assert len(rows.data) == 2
    assert rows.data[0] == (None, [400.0, -120.5, 300.0, 180.0, 0.0, -90.0])
    # filename shape, matched by name later.
    named = os.path.join(tmp, "poses_named.csv")
    with open(named, "w", encoding="utf-8") as f:
        f.write("img_001.png,400.0,-120.5,300.0,180.0,0.0,-90.0\n"
                "img_002.png,410.0,-118.0,305.0,175.0,3.0,-88.0\n")
    rows = project_io.parse_pose_csv(named)
    assert rows.is_ok and rows.data[1][0] == "img_002.png"
    # Refusals: a non-numeric row past the header, a wrong column count, a
    # mixed file, a non-finite value, and an empty file -- each with an err,
    # never a silent skip that would misalign every row after it.
    for name, content in (
            ("bad_row.csv", "1,2,3,4,5,6\noops,2,3,4,5,6\n"),
            ("bad_cols.csv", "1,2,3,4,5\n"),
            ("mixed.csv", "1,2,3,4,5,6\nimg.png,1,2,3,4,5,6\n"),
            ("nonfinite.csv", "1,2,3,4,5,inf\n"),
            ("empty.csv", "# nothing here\n")):
        path = os.path.join(tmp, name)
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
        assert project_io.parse_pose_csv(path).is_err, name
    print("PASS pose CSV parses both shapes and refuses malformed files")
finally:
    shutil.rmtree(tmp)


# =========================================================================
# 9. Coverage gate override
# =========================================================================
# detect_board's image-coverage floor is settings-driven; the override must
# actually reach the gate rather than being accepted and ignored.
frame = np.full((120, 160, 3), 255, dtype=np.uint8)
board_cfg = CharucoBoardConfig()
board, detector = handeye.make_board(board_cfg)
none_det = handeye.detect_board(frame, board, detector, None,
                                min_coverage=0.5)
assert none_det.n_corners == 0     # blank frame: nothing to detect, no crash
print("PASS detect_board accepts a coverage override and survives a blank "
      "frame")

print("\nALL CORE TESTS PASSED")