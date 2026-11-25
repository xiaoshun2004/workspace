# Nova-5 eye-in-hand calibration

A standalone PySide6 app for calibrating a wrist-mounted camera on a Dobot
Nova-5, built for calibration that gets repeated often: author the procedure
once as a *project*, then replay it as many *instances* as you like.

```
python3 main.py
```

Requires `PySide6`, `opencv-python` (`>=4.8,<5`: 4.7 has the modern ChArUco
API but its `matchImagePoints` is broken from Python, and 5.x drops
`cv2.calibrateHandEye` — the bound is enforced at import in `core/handeye.py`),
`numpy`, and `pyrealsense2`. Imports fail fast: a missing dependency is an
ImportError at startup with a traceback, never a runtime surprise.

---

## The two ideas the app is built around

### 1. The pose convention is solved, not assumed

Getting the controller's orientation convention wrong produces a hand-eye
result that looks entirely plausible and is silently wrong. The usual advice
is to command a known 90-degree move and check.

This controller makes that unnecessary. The real-time feedback packet reports
the same rotation twice — as `(rx, ry, rz)` at byte 624 and as a quaternion at
byte 1384 — and a quaternion has no convention ambiguity beyond component
order. So the Euler convention is *searched for*: 12 axis sequences x
intrinsic/extrinsic x degrees/radians x rotation-vector x both quaternion
orders, scored against the quaternion.

Strike four or five varied orientations on the **Robot** tab, press Solve, and
the result is written into every project and instance file. Automated runs
stay disabled until it passes. If nothing matches, that is diagnostic too —
`diagnose_frame_mismatch()` tests whether the quaternion is on a different
frame than `ToolVectorActual` and names the relationship.

### 2. Remounting the camera is a solved problem, not a re-calibration

With `X = T_cam2mount`, `G_i = T_mount2base`, `B = T_board2base` fixed:

```
view_i = X^-1 . G_i^-1 . B
```

Remount the camera, `X -> X'`, and every stored view shifts by **one** rigid
transform, identical at every pose: `view'_i = (X^-1 X')^-1 . view_i`. The
correction is a flange-frame right multiplication,

```
G_i' = G_i . delta          delta = X . X'^-1
```

which reproduces every view exactly, for arbitrarily large remounts. And
`delta` is measurable without knowing either `X` or `X'` — re-teach one pose
and read it off: `delta = G_ref^-1 . G_new`.

**The naive alternative is wrong.** Storing home-relative poses and replaying
from a re-taught home assumes `delta` commutes with each stored relative pose,
which it does not; `tests/test_core.py` shows it missing by up to 1.4 m on a
random pose set. Projects here store absolute poses and correct by right
multiplication.

Every pose reached afterwards gives an independent estimate of the same
`delta`. The **spread** of those estimates is a free diagnostic: tight means a
clean remount, wide means the board moved or the convention is wrong. It only
means that much when each pose was actually re-verified against the board: if
the arm simply drove to the corrected pose and no servo re-observed the board,
every estimate equals the anchor delta by construction and the spread measures
arm repeatability, so the verdict says so instead of calling it clean.

---

## Workflow

1. **Camera tab** — pick a device and mode. A USB camera has no intrinsics, so
   solve them here first (15+ tilted views reaching the frame corners); they
   are stored in a library keyed by camera *and resolution*. RealSense factory
   intrinsics work out of the box; the solve is offered anyway and compared
   against them.
2. **Robot tab** — connect, then run the convention wizard. Set the payload:
   the controller computes collision detection against it, so leaving it at
   zero with a bracket fitted makes the thresholds wrong in the unsafe
   direction.
3. **Projects tab** — create a project, jog to a pose with the board nicely
   centred, "Set home from current", then add more poses. Two live readouts
   matter: **rotation diversity** (single-axis rotation makes translation
   along that axis unobservable, and no residual metric will reveal it) and
   **remount margin** (how far the camera can move before each pose loses the
   board — the report names the weakest pose).
4. **Calibrate tab** — pick Auto, seed the refinement from the previous
   result, jog until the ghosted reference corners sit on the live ones, set
   the anchor, run. Or use Manual (jog and capture) or Images-only (no arm;
   type the poses in later).

   Images-only also takes files: **Import images** loads an existing library
   of frames (taken with this camera at the current resolution — intrinsics
   scale with resolution, so a mismatched file is refused by name), runs each
   through the same detection and quality gates as a live capture, and adds
   the survivors as samples. Assign poses one at a time with *Enter pose for
   sample*, or all at once with **Import poses CSV**: rows of
   `x,y,z,rx,ry,rz` applied in order to samples without a pose, or
   `filename,x,y,z,rx,ry,rz` matched to each sample's image file — raw
   controller numbers either way, interpreted through the solved convention.
   Then Solve, exactly as if the frames had been captured live.

   The **min board coverage** gate (how much of the image the detected board
   must span before a pose is estimated) is editable here too; 2% is a sane
   floor for close-in D405 work, but the right value depends on your working
   distance, and both live capture and file import honour it.

   **Open instance** loads a saved run back in: its samples, poses and images.
   Select a sample to see the frame it was captured from and the exact numbers
   the controller reported; fix a pose with *Enter pose for sample*, delete a
   bad sample, then re-solve. Save writes a new instance by default (recording
   `derived_from`), or ticks over the loaded one. This is separate from *Seed
   refinement from previous result*, which reads only `T_cam2mount` to
   bootstrap the servo and loads no samples at all.
5. **Verify tab** — load the instance, click a feature, move the tool to it.
   This is the only test that means anything. Residual spread is computed from
   the same data the solve used; it catches gross errors, not systematic ones.

---

## Layout

```
core/          no Qt, no SDK -- cv2 + numpy only
  models.py            Result, CameraIntrinsics, CharucoBoardConfig,
                       HandEyeResult (from the Atom-W bundle) + PoseConvention,
                       RobotPose, FrameIdentity, SafetyConfig, CalibPose,
                       CalibProject, CalibInstance, RemountRecord, Euler maths
  transforms.py        SE(3) helpers, verbatim
  intrinsics_math.py   Brown-Conrady both directions; the inverse is a damped
                       Newton solve that reports non-convergence
  handeye.py           ChArUco detection, IPPE ambiguity gate, calibrateHandEye,
                       rotation-diversity check, ghost overlay
  remount.py           delta estimation, spread diagnosis, servo, view margins
  pose_convention.py   the convention solver
  project_io.py        on-disk layout, atomic writes, intrinsics library
api/           the only files importing a hardware SDK
  nova5_api.py         feedback dtype, dashboard/motion sockets, safety gate
  realsense_api.py     aligned colour + depth
  usb_camera_api.py    UVC, with resolution enforcement
  camera_base.py       shared interface;  camera_factory.py  discovery
  intrinsic_calib.py   cv2.calibrateCameraCharuco
ui/            PySide6
  main_window.py, tab_camera, tab_robot, tab_project, tab_calibrate,
  tab_verify, workers.py, context.py, settings.py, constants.py,
  app_log.py, qt_helpers.py, angle_format*.py
tests/
  test_core.py         remount algebra, convention solver, distortion,
                       persistence, validity gates, reply parsing
  smoke_test.py        stubs PySide6 + pyrealsense2, imports every module
```

## On-disk layout

```
~/.config/nova5_calib/
  settings.json
  intrinsics_library.json          one record per (camera, resolution)
  projects/<name>/
    project.json                   board, camera, convention, frames, poses
    poses/pose_000_raw.png  pose_000_overlay.png  ...
    instances/<stamp>_<label>/
      metadata.json                intrinsics + resolution + distortion model,
                                   board geometry, Tool/User identity, pose
                                   convention, remount record, solve result
      handeye.json                 result + samples
      images/sample_000_raw.png  sample_000_overlay.png  ...
```

Image paths are stored **relative** and resolved on load, so a run can be
copied to another machine for review — which is the whole reason the images
exist. Writes are atomic: an interrupted save leaves the previous good file
intact.

---

## Details that cost real debugging time

Carried over from the Atom-W bundle, still true here:

1. **`inverse_brown_conrady` is not OpenCV's convention.** RealSense colour
   streams commonly report coefficients that map *observed -> ideal*, the
   opposite of `distCoeffs`. Passing them straight to `solvePnP` roughly
   **doubles** the distortion error, and `solvePnP` absorbs part of it into the
   pose so the reprojection error still looks fine. The fix is exact: undistort
   with the model's own polynomial, solve with zero coefficients.
2. **Planar boards have a two-fold pose ambiguity.** `SOLVEPNP_ITERATIVE`
   silently returns one of two solutions; `solvePnPGeneric` with `IPPE` returns
   both, and the sample is rejected unless the runner-up is ~3x worse.
3. **Few corners *lower* reprojection error.** Gate on a fraction of the
   board's corners and a minimum image-area coverage, not just on pixels.
4. **AX = XB needs rotation-axis diversity**, checked before every solve.
5. **Intrinsics are resolution-specific.** Recorded with the resolution;
   a stored calibration is refused against a different live stream.
6. **Any change to the inputs invalidates the result.** Deleting a sample or
   entering a pose clears the stored solve.
7. **Gate the capture, not just the display** — reprojection error, frame
   staleness, board-static between two frames, and the working-distance band.
8. **Save the images.** One directory per run, raw plus overlay per sample.
9. **Spread metrics are residuals, not validation.** The real test is physical
   — and worth being concrete about, because the test suite demonstrates it:
   corrupting one sample's arm pose in an otherwise consistent set leaves the
   TSAI solution unmoved to machine precision while the board-origin spread
   jumps to 1.3 m. The solve succeeding tells you nothing; the spread does.

New here:

10. **The vendor's feedback dtype hides the safety bytes.** `dobot_api.py` has
    correct offsets for the common fields but lumps 1038-1119 and 1416-1439
    into reserved blobs — which contain `CollisionState` (1038) and
    `SafetyState` (1420). `NOVA5_FEEDBACK_DTYPE` exposes them.
11. **`TestValue` is checked on every packet**, not once. A slipped stream
    would otherwise present as plausible-looking wrong poses.
12. **The taught TCP is part of the calibration's identity.** `X` is relative
    to whatever frame the controller reports, so the Tool/User indices *and*
    their six-double frame values are recorded, and a calibration is refused
    if the TCP was re-taught since.
13. **Replay goes through `InverseSolution` with `JointNear`.** A six-axis arm
    reaches most poses in several configurations; without a seed, replaying an
    authored pose can turn a 20 mm move into a swing across the cell.
14. **Poses are stored raw.** `RobotPose` keeps the controller's own numbers
    and derives transforms through the convention on demand, so a convention
    corrected later re-derives every sample instead of wasting them.
15. **No raw detector array is ever handed to a drawing function.** OpenCV 5
    reads an `(N, 1, 2)` float array as an N-row two-COLUMN matrix in some
    entry points and an N-row two-CHANNEL matrix in others, so
    `drawDetectedCornersCharuco` compares `total()` of 2N against N and
    asserts. Overlays are drawn with `circle`/`polylines`/`putText` on
    geometry normalised through `handeye.as_xy()`.
16. **The intrinsic solve avoids `calibrateCameraCharuco`**, deprecated in 4.7
    and removed in 5.x, in favour of `matchImagePoints` plus
    `cv2.calibrateCamera` -- which is what it called internally anyway.
17. **Overlay axes are mapped back into the frame they are drawn on.** On the
    exact-undistort path the pose comes from undistorted points and zero
    coefficients, so it lives in the *ideal* camera: projecting the axes with
    the stored coefficients would bend them off the board, and projecting with
    zeros would leave them at ideal pixels on a raw frame whose corners — and
    every other overlay element — are at observed pixels. So the endpoints are
    projected with zero distortion and then mapped ideal -> observed through
    `intrinsics_math.observed_from_ideal`, whose damped-Newton inverse says so
    (rather than returning garbage) when the model cannot be inverted.

---

## Safety

Every gate is individually defeatable in the Robot tab and every one defaults
to on: workspace box, joint limits, single-step size limits, arm-enabled
requirement, per-pose confirmation on automated runs. Speed factor defaults to
10%. The STOP button sends `ResetRobot()` — it travels over the same TCP
socket as everything else and is **not** a substitute for the physical e-stop.

Turning a gate off logs a warning. That is a decision the operator makes in
the settings, not something the code does quietly because a check was
inconvenient.

---

## Testing

```
python3 tests/test_core.py            # no hardware needed
python3 tests/smoke_test.py           # every module under SDK stubs
```

`test_core.py` proves the remount correction is exact, that the naive
alternative is not, that a moved board is caught by the estimate spread, that
the convention solver identifies axis order, angle unit and quaternion
component order, refuses degenerate input and refuses a single-axis sample set
as ambiguous, that the commanded-move cross-check rejects the right angle
about the wrong axis, that the distortion inverse converges for strong
coefficients and raises rather than returning garbage where it cannot, that a
seeded `InverseSolution` reply parses past the echoed `JointNear` braces, and
that instances round-trip with their validity metadata intact, that a saved
instance reopens and re-solves to the same answer, and that a corrected pose
can be written back either in place or as a new instance carrying its
provenance.

A footnote on boards: ChArUco's corner-ordering pattern changed in OpenCV 4.6.
If a board printed from an older tool refuses to detect, that is the reason,
and `cv2.aruco.CharucoBoard.setLegacyPattern(True)` is the switch.