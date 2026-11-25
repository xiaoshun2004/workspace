# -*- coding: utf-8 -*-
"""
api/realsense_api.py
--------------------
Intel RealSense wrapper built directly on pyrealsense2, adapted from the
Atom-W bundle onto the CameraBase interface.

Deliberately not LeRobot's camera class: it does not align depth to colour,
which is exactly what pixel -> 3D needs.

Responsibilities:
  * Enumerate devices and their exact supported (width, height, fps) modes, so
    the GUI can reject a bad combination BEFORE start() -- an unsupported mode
    otherwise makes the SDK throw at pipeline start.
  * Stream aligned colour (BGR) plus depth (z16) on a background thread.
  * Report colour intrinsics, including the distortion model name, because
    RS2_DISTORTION_INVERSE_BROWN_CONRADY needs the exact-undistort path.
  * Live exposure/gain. Resolution and fps are connect-time only.

One of the two files in this project allowed to import a hardware SDK.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs

from camera_base import CameraBase
from models import CameraConfig, CameraIntrinsics, CameraKind, Result

logger = logging.getLogger(__name__)

# Depth display range for the TURBO colormap, in metres. The D405 is a
# close-range module (roughly 0.07 to 0.5 m), so a small range gives contrast.
DEPTH_VIS_MIN_M = 0.0
DEPTH_VIS_MAX_M = 0.6


def colorize_depth(depth_u16: np.ndarray, depth_scale: float,
                   vmin_m: float = DEPTH_VIS_MIN_M,
                   vmax_m: float = DEPTH_VIS_MAX_M) -> np.ndarray:
    """Raw z16 depth -> TURBO BGR image for display."""
    depth_m = depth_u16.astype(np.float32) * float(depth_scale)
    span = max(vmax_m - vmin_m, 1e-6)
    norm = np.clip((depth_m - vmin_m) / span, 0.0, 1.0)
    gray = (norm * 255.0).astype(np.uint8)
    colored = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    colored[depth_u16 == 0] = (0, 0, 0)     # mark no-depth pixels black
    return colored


class RealSenseCamera(CameraBase):
    """Background-threaded RealSense streamer with aligned colour + depth."""

    def __init__(self, config: CameraConfig) -> None:
        super().__init__(config)
        self._pipeline: Optional[rs.pipeline] = None
        self._profile: Optional[rs.pipeline_profile] = None
        self._align: Optional[rs.align] = None
        self._intr_rs: Optional[rs.intrinsics] = None
        self._depth_scale: float = 0.001
        self._depth: Optional[np.ndarray] = None
        # Guards _stop_pipeline(), which both disconnect() and the read loop
        # can reach. Separate from the frame lock so stopping never waits on a
        # frame copy.
        self._pipeline_lock = threading.Lock()

    @staticmethod
    def kind() -> CameraKind:
        return CameraKind.REALSENSE

    def has_depth(self) -> bool:
        return True

    def has_factory_intrinsics(self) -> bool:
        return True

    # ------------------------------------------------------------ discovery
    @staticmethod
    def list_devices() -> Result[List[Dict[str, Any]]]:
        try:
            ctx = rs.context()
            out: List[Dict[str, Any]] = []
            for dev in ctx.query_devices():
                out.append({
                    "serial": dev.get_info(rs.camera_info.serial_number),
                    "name": dev.get_info(rs.camera_info.name),
                    "product_line": dev.get_info(rs.camera_info.product_line),
                })
            return Result.ok(out)
        except RuntimeError as exc:
            return Result.err_msg(
                "Failed to enumerate RealSense devices: {}".format(exc))

    @staticmethod
    def list_supported_modes(serial: Optional[str] = None
                             ) -> Result[Dict[str, List[tuple]]]:
        """Supported (width, height, fps) tuples for colour and depth."""
        try:
            ctx = rs.context()
            devices = list(ctx.query_devices())
            if not devices:
                return Result.err_msg("No RealSense devices found.")
            device = None
            if serial is None:
                device = devices[0]
            else:
                for dev in devices:
                    if dev.get_info(rs.camera_info.serial_number) == serial:
                        device = dev
                        break
            if device is None:
                return Result.err_msg(
                    "No RealSense device with serial {}.".format(serial))

            color_modes = set()
            depth_modes = set()
            for sensor in device.query_sensors():
                for prof in sensor.get_stream_profiles():
                    if not prof.is_video_stream_profile():
                        continue
                    vp = prof.as_video_stream_profile()
                    mode = (vp.width(), vp.height(), vp.fps())
                    if vp.stream_type() == rs.stream.color:
                        color_modes.add(mode)
                    elif vp.stream_type() == rs.stream.depth:
                        depth_modes.add(mode)
            return Result.ok({"color": sorted(color_modes),
                              "depth": sorted(depth_modes)})
        except RuntimeError as exc:
            return Result.err_msg(
                "Failed to query supported modes: {}".format(exc))

    # ------------------------------------------------------------ lifecycle
    def connect(self) -> Result[None]:
        if self.is_connected:
            return Result.ok_msg("Camera already connected.")

        mode = (self.config.width, self.config.height, self.config.fps)
        modes = self.list_supported_modes(self.config.ident)
        if modes.is_err:
            return Result.err_msg(modes.message)
        # Depth is always enabled at the same resolution because move-to-pixel
        # needs it, so the depth mode is validated unconditionally.
        supported_depth = set(modes.data.get("depth", []))
        if mode not in supported_depth:
            return Result.err_msg(
                "Requested DEPTH mode {}x{}@{} is not supported. Supported "
                "depth modes: {}".format(
                    mode[0], mode[1], mode[2],
                    ", ".join("{}x{}@{}".format(*m)
                              for m in sorted(supported_depth))))
        supported_color = set(modes.data.get("color", []))
        if mode not in supported_color:
            return Result.err_msg(
                "Requested colour mode {}x{}@{} is not supported. Supported: "
                "{}".format(mode[0], mode[1], mode[2],
                            ", ".join("{}x{}@{}".format(*m)
                                      for m in sorted(supported_color))))

        try:
            self._pipeline = rs.pipeline()
            rs_config = rs.config()
            if self.config.ident:
                rs_config.enable_device(str(self.config.ident))
            rs_config.enable_stream(rs.stream.color, self.config.width,
                                    self.config.height, rs.format.bgr8,
                                    self.config.fps)
            rs_config.enable_stream(rs.stream.depth, self.config.width,
                                    self.config.height, rs.format.z16,
                                    self.config.fps)
            self._profile = self._pipeline.start(rs_config)
        except RuntimeError as exc:
            self._pipeline = None
            self._profile = None
            return Result.err_msg(
                "Failed to start camera pipeline: {}".format(exc))

        self._align = rs.align(rs.stream.color)
        self._depth_scale = float(
            self._profile.get_device().first_depth_sensor().get_depth_scale())
        self._intr_rs = self._profile.get_stream(
            rs.stream.color).as_video_stream_profile().get_intrinsics()
        self._apply_initial_options()

        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop,
                                        name="realsense_read", daemon=True)
        self._thread.start()

        deadline = time.time() + 3.0
        while time.time() < deadline:
            with self._lock:
                have = self._color is not None and self._depth is not None
            if have:
                return Result.ok_msg("Camera connected.")
            time.sleep(0.05)
        self.disconnect()
        return Result.err_msg(
            "Camera started but produced no frames within timeout.")

    def _stop_pipeline(self) -> None:
        """Stop the pipeline exactly once, from whichever of disconnect() and
        the read loop reaches it first. A started pipeline holds the device, so
        leaving it running after the reader dies makes the D405 impossible to
        reopen; stopping an already-stopped pipeline throws, hence the guard.
        """
        with self._pipeline_lock:
            pipeline = self._pipeline
            self._pipeline = None
            self._profile = None
            self._align = None
        if pipeline is None:
            return
        try:
            pipeline.stop()
        except RuntimeError as exc:
            logger.warning("Error stopping pipeline: %s", exc)

    def disconnect(self) -> Result[None]:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_pipeline()
        with self._lock:
            self._color = None
            self._depth = None
        return Result.ok_msg("Camera disconnected.")

    # ------------------------------------------------------------ read loop
    def _read_loop(self) -> None:
        # Bound once: _stop_pipeline() clears the attributes, and it may run on
        # the disconnecting thread while this loop is still between frames.
        pipeline = self._pipeline
        align = self._align
        failures = 0
        while not self._stop.is_set():
            try:
                # Shorter than disconnect()'s 2.0 s join, so the loop always
                # notices the stop flag before pipeline.stop() runs under it.
                frames = pipeline.wait_for_frames(timeout_ms=500)
                aligned = align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()
                if not depth_frame or not color_frame:
                    continue
                color = np.asanyarray(color_frame.get_data())
                depth = np.asanyarray(depth_frame.get_data())
                with self._lock:
                    self._color = color
                    self._depth = depth
                    self._timestamp = time.perf_counter()
                failures = 0
            except RuntimeError as exc:
                # Survive a transient frame failure, but do not spin forever
                # on a dead camera.
                failures += 1
                if failures <= 10:
                    logger.warning("Camera frame read failed: %s", exc)
                else:
                    # Giving up is an abnormal exit, so release the device
                    # here: is_connected is thread liveness, so nothing above
                    # will call disconnect() once this thread is gone, and a
                    # still-started pipeline would fail every reconnect until
                    # the app is restarted.
                    logger.error("Too many consecutive read failures; "
                                 "stopping the pipeline so the camera can be "
                                 "reconnected.")
                    self._stop.set()
                    self._stop_pipeline()
                    break

    # ------------------------------------------------------------ accessors
    def get_frames(self) -> Result[Tuple[np.ndarray, np.ndarray]]:
        with self._lock:
            if self._color is None or self._depth is None:
                return Result.err_msg("No frames available yet.")
            return Result.ok((self._color.copy(), self._depth.copy()))

    def get_depth_colormap(self) -> Result[np.ndarray]:
        with self._lock:
            if self._depth is None:
                return Result.err_msg("No depth frame available yet.")
            depth = self._depth.copy()
        return Result.ok(colorize_depth(depth, self._depth_scale))

    @property
    def depth_scale(self) -> float:
        return self._depth_scale

    def factory_intrinsics(self) -> Result[CameraIntrinsics]:
        if self._intr_rs is None:
            return Result.err_msg("Camera not connected; no intrinsics.")
        i = self._intr_rs
        return Result.ok(CameraIntrinsics(
            width=i.width, height=i.height, fx=i.fx, fy=i.fy, ppx=i.ppx,
            ppy=i.ppy, coeffs=list(i.coeffs),
            model=str(getattr(getattr(i, "model", None), "name",
                              getattr(i, "model", "") or ""))))

    def depth_at(self, u: int, v: int, patch: int = 2) -> Result[float]:
        """Median valid depth (metres) in a (2*patch+1) square around (u, v).

        A median over a patch rather than the single pixel: one dead pixel on
        a board corner would otherwise send the arm somewhere surprising.
        """
        with self._lock:
            if self._depth is None:
                return Result.err_msg("No depth frame available yet.")
            depth = self._depth
            h, w = depth.shape[:2]
            if not (0 <= u < w and 0 <= v < h):
                return Result.err_msg(
                    "Pixel ({}, {}) out of bounds ({}x{}).".format(u, v, w, h))
            u0, u1 = max(0, u - patch), min(w, u + patch + 1)
            v0, v1 = max(0, v - patch), min(h, v + patch + 1)
            window = depth[v0:v1, u0:u1].astype(np.float32)
        valid = window[window > 0]
        if valid.size == 0:
            return Result.err_msg(
                "No valid depth at pixel ({}, {}).".format(u, v))
        return Result.ok(float(np.median(valid)) * self._depth_scale)

    def deproject(self, u: float, v: float,
                  depth_m: float) -> Result[Tuple[float, float, float]]:
        """Pixel + depth -> 3D point in the camera optical frame.

        Uses librealsense's own deprojection when the device intrinsics are in
        force, since that handles the inverse-Brown-Conrady model natively. If
        the operator has installed library intrinsics instead, the pinhole
        path in CameraBase is the consistent choice.
        """
        if self._override_intrinsics is not None or self._intr_rs is None:
            return super().deproject(u, v, depth_m)
        if depth_m <= 0.0:
            return Result.err_msg("Depth must be positive.")
        try:
            xyz = rs.rs2_deproject_pixel_to_point(
                self._intr_rs, [float(u), float(v)], float(depth_m))
        except RuntimeError as exc:
            return Result.err_msg(
                "Deprojection failed (distortion model '{}'): {}".format(
                    getattr(getattr(self._intr_rs, "model", None), "name",
                            "unknown"), exc))
        return Result.ok((float(xyz[0]), float(xyz[1]), float(xyz[2])))

    # ------------------------------------------------------------ live options
    @staticmethod
    def _serves_color(sensor) -> bool:
        """True if this sensor is the one that produces the colour stream."""
        try:
            for prof in sensor.get_stream_profiles():
                if prof.stream_type() == rs.stream.color:
                    return True
        except RuntimeError:
            return False
        return False

    def _option_sensor(self, option) -> Optional[Any]:
        """The sensor to address for a live option.

        Every option here exists to control the COLOUR image -- it is what
        board detection and the intrinsic solve use. On a D405 that is trivial:
        one Stereo Module serves colour and depth. But this app offers whatever
        rs.context() reports, and on a D415/D435 query_sensors() lists the
        Stereo Module first and it supports exposure/gain/auto-exposure too, so
        taking the first match would silently write the DEPTH sensor. Prefer
        the sensor that actually carries the colour stream, and fall back to
        the first supporting sensor only when none does.
        """
        if self._profile is None:
            return None
        fallback = None
        for sensor in self._profile.get_device().query_sensors():
            if not sensor.supports(option):
                continue
            if self._serves_color(sensor):
                return sensor
            if fallback is None:
                fallback = sensor
        return fallback

    @staticmethod
    def _log_option_result(name: str, result: Result[None]) -> None:
        if result.is_err:
            logger.warning("Could not apply %s at connect: %s", name,
                           result.message)

    def _clamp_to_range(self, option_name: str, value: int) -> int:
        """Pull a configured value into the device's own range for the option.

        librealsense throws rather than clamping when set_option is handed an
        out-of-range value, and the UI spin boxes are deliberately wider than
        any single device -- so settings carried over from another camera would
        otherwise be rejected outright.
        """
        rng = self.option_range(option_name)
        if rng.is_err:
            return int(value)
        low, high = rng.data[0], rng.data[1]
        clamped = int(round(min(max(float(value), low), high)))
        if clamped != int(value):
            logger.warning("Configured %s %s is outside this device's range "
                           "%.0f..%.0f; using %s instead.", option_name, value,
                           low, high, clamped)
        return clamped

    def _apply_initial_options(self) -> None:
        """Best effort: connect() still succeeds if the device refuses one of
        these, but every refusal is logged rather than dropped -- a silently
        rejected value leaves the Camera tab showing a number the device is not
        actually using."""
        if self.config.auto_exposure:
            self._log_option_result("auto-exposure",
                                    self.set_auto_exposure(True))
            return
        self._log_option_result("auto-exposure", self.set_auto_exposure(False))
        if self.config.exposure is not None:
            self._log_option_result("exposure", self.set_exposure(
                self._clamp_to_range("exposure", self.config.exposure)))
        if self.config.gain is not None:
            self._log_option_result("gain", self.set_gain(
                self._clamp_to_range("gain", self.config.gain)))

    def set_auto_exposure(self, enabled: bool) -> Result[None]:
        sensor = self._option_sensor(rs.option.enable_auto_exposure)
        if sensor is None:
            return Result.err_msg("Auto-exposure not supported on this device.")
        try:
            sensor.set_option(rs.option.enable_auto_exposure,
                              1.0 if enabled else 0.0)
            return Result.ok_msg(
                "Auto-exposure {}.".format("on" if enabled else "off"))
        except RuntimeError as exc:
            return Result.err_msg("Failed to set auto-exposure: {}".format(exc))

    def set_exposure(self, value: int) -> Result[None]:
        sensor = self._option_sensor(rs.option.exposure)
        if sensor is None:
            return Result.err_msg("Exposure not supported on this device.")
        try:
            sensor.set_option(rs.option.exposure, float(value))
            return Result.ok_msg("Exposure set to {} us.".format(value))
        except RuntimeError as exc:
            return Result.err_msg("Failed to set exposure: {}".format(exc))

    def set_gain(self, gain: int) -> Result[None]:
        sensor = self._option_sensor(rs.option.gain)
        if sensor is None:
            return Result.err_msg("Gain not supported on this device.")
        try:
            sensor.set_option(rs.option.gain, float(gain))
            return Result.ok_msg("Gain set to {}.".format(gain))
        except RuntimeError as exc:
            return Result.err_msg("Failed to set gain: {}".format(exc))

    def option_range(self, option_name: str
                     ) -> Result[Tuple[float, float, float, float]]:
        option_map = {"exposure": rs.option.exposure, "gain": rs.option.gain}
        if option_name not in option_map:
            return Result.err_msg("Unknown option '{}'.".format(option_name))
        sensor = self._option_sensor(option_map[option_name])
        if sensor is None:
            return Result.err_msg(
                "Option '{}' not supported.".format(option_name))
        try:
            r = sensor.get_option_range(option_map[option_name])
            return Result.ok((float(r.min), float(r.max), float(r.step),
                              float(r.default)))
        except RuntimeError as exc:
            return Result.err_msg("Failed to read option range: {}".format(exc))
