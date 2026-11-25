# -*- coding: utf-8 -*-
"""
api/usb_camera_api.py
---------------------
Plain UVC webcam over cv2.VideoCapture.

Two things this backend is careful about, both of which bite silently:

1. A USB CAMERA HAS NO INTRINSICS. It cannot estimate a board pose until
   intrinsics are supplied, and the honest failure is a refusal rather than a
   guess. has_factory_intrinsics() is False, so the app requires an entry in
   the intrinsics library -- solved once with api/intrinsic_calib.py -- before
   it will let you capture calibration samples with one.

2. cv2 ACCEPTS A RESOLUTION REQUEST AND MAY IGNORE IT. Setting
   CAP_PROP_FRAME_WIDTH to something the device does not support usually
   returns True and then streams at a different size. Since intrinsics are
   resolution-specific, silently streaming at 640x480 while believing you are
   at 1280x720 corrupts every board pose. connect() therefore reads the
   properties back and refuses if they differ from what was asked for --
   which only works if the backend is DSHOW, see _CAPTURE_BACKEND.
"""

from __future__ import annotations

import logging
import math
import sys
import threading
import time
from typing import List, Optional

import cv2

from camera_base import CameraBase
from models import CameraConfig, CameraIntrinsics, CameraInfo, CameraKind, Result

logger = logging.getLogger(__name__)

# Common modes offered in the UI. The device may support others; these are
# what the dropdown starts with, and any of them can be overridden by typing.
COMMON_MODES = [
    (640, 480, 30), (800, 600, 30), (1280, 720, 30), (1280, 720, 60),
    (1920, 1080, 30), (1920, 1080, 60), (2560, 1440, 30), (3840, 2160, 30),
]

# Every VideoCapture in this module is opened with an explicit backend. On
# Windows the default CAP_ANY resolves to MSMF, which is the one backend that
# defeats the resolution guard in connect(): it commonly captures at the device
# default and RESCALES the frame to the requested size, so frame.shape matches
# what was asked for while the pixels are interpolated (opencv/opencv#12822),
# and on other devices it refuses mode sets DSHOW accepts (#20378, #25243).
# DSHOW does neither, and it is also the backend whose exposure conventions
# set_exposure/set_auto_exposure below are written against. Off Windows
# CAP_ANY picks V4L2, which is the correct choice there.
_USE_DSHOW = sys.platform == "win32"
_CAPTURE_BACKEND = cv2.CAP_DSHOW if _USE_DSHOW else cv2.CAP_ANY

# CAP_PROP_AUTO_EXPOSURE is backend-specific, not an OpenCV-wide enumeration.
# DSHOW's setProperty (cap_dshow.cpp) does `enabled = cvRound(propVal) == 1`
# and sends CameraControl_Flags_Manual for anything else, so 1 is on and 0 is
# off. V4L2 instead takes its own EXPOSURE_AUTO codes, 3 = aperture priority
# (auto) and 1 = manual -- and 1 is precisely the value DSHOW reads as AUTO,
# so sending the V4L2 pair to Windows inverts the control.
_AUTO_EXPOSURE_ON = 1.0 if _USE_DSHOW else 3.0
_AUTO_EXPOSURE_OFF = 0.0 if _USE_DSHOW else 1.0

# Bounds for CAP_PROP_EXPOSURE under DSHOW, whose unit is log base 2 of the
# exposure time in SECONDS: -13 is about 122 us and -1 is 0.5 s, which brackets
# anything a UVC webcam will accept.
_DSHOW_EXPOSURE_MIN = -13
_DSHOW_EXPOSURE_MAX = -1


class UsbCamera(CameraBase):
    """Background-threaded UVC camera."""

    def __init__(self, config: CameraConfig) -> None:
        super().__init__(config)
        self._cap: Optional[cv2.VideoCapture] = None

    @staticmethod
    def kind() -> CameraKind:
        return CameraKind.USB

    def has_depth(self) -> bool:
        return False

    def has_factory_intrinsics(self) -> bool:
        return False

    # ------------------------------------------------------------ discovery
    @staticmethod
    def list_devices(max_index: int = 6) -> List[CameraInfo]:
        """Probe indices 0..max_index-1 by opening and closing each.

        Slow and chatty on some systems, which is why the ceiling is a setting
        rather than a constant.
        """
        found: List[CameraInfo] = []
        for index in range(max(0, int(max_index))):
            cap = cv2.VideoCapture(index, _CAPTURE_BACKEND)
            if cap.isOpened():
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = int(round(cap.get(cv2.CAP_PROP_FPS) or 0))
                found.append(CameraInfo(
                    kind=CameraKind.USB, ident=str(index),
                    label="USB camera {} ({}x{})".format(index, width, height),
                    width=width, height=height, fps=fps))
            cap.release()
        return found

    # ------------------------------------------------------------ lifecycle
    def connect(self) -> Result[None]:
        if self.is_connected:
            return Result.ok_msg("Camera already connected.")
        try:
            index = int(self.config.ident if self.config.ident is not None else 0)
        except ValueError:
            return Result.err_msg(
                "USB camera identifier must be a device index, got "
                "{!r}.".format(self.config.ident))

        cap = cv2.VideoCapture(index, _CAPTURE_BACKEND)
        if not cap.isOpened():
            cap.release()
            return Result.err_msg(
                "Could not open USB camera index {}.".format(index))

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.config.width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.config.height))
        cap.set(cv2.CAP_PROP_FPS, float(self.config.fps))

        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            return Result.err_msg(
                "USB camera {} opened but produced no frame.".format(index))

        # Read the ACTUAL size off the frame, not off the properties: the
        # properties can report the request rather than the reality.
        actual_h, actual_w = frame.shape[:2]
        if (actual_w, actual_h) != (self.config.width, self.config.height):
            cap.release()
            return Result.err_msg(
                "Asked for {}x{} but the camera is streaming {}x{}. "
                "Intrinsics are resolution-specific, so the app will not "
                "continue with a silently different size -- pick a mode the "
                "device actually supports.".format(
                    self.config.width, self.config.height, actual_w, actual_h))

        self._cap = cap
        with self._lock:
            self._color = frame
            self._timestamp = time.perf_counter()
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop,
                                        name="usb_camera_read", daemon=True)
        self._thread.start()
        return Result.ok_msg("USB camera {} connected at {}x{}.".format(
            index, actual_w, actual_h))

    def disconnect(self) -> Result[None]:
        """Ask the reader to stop and wait for it. The RELEASE is the reader's
        job, never this method's: cap.read() has no timeout and a stalled or
        unplugged UVC device blocks in the driver for far longer than the join,
        so releasing here would run release() concurrently with an in-flight
        read on the same cv2.VideoCapture."""
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                # Leave _thread and _cap alone: the handle is still in use, and
                # is_connected must keep reporting the thread that owns it.
                return Result.err_msg(
                    "USB camera reader did not stop within 2.0 s; the device "
                    "is probably stalled or unplugged. The capture will be "
                    "released when the read in progress returns.")
        self._thread = None
        with self._lock:
            self._color = None
        return Result.ok_msg("USB camera disconnected.")

    def _read_loop(self) -> None:
        """Owns the capture for the whole life of the thread. Whichever way the
        loop ends -- the stop flag from disconnect(), or giving up on a device
        that has stopped producing frames -- the finally block is the single
        place the handle is released, so the OS device is never left claimed by
        a thread that is no longer running."""
        cap = self._cap
        failures = 0
        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    failures += 1
                    if failures > 30:
                        logger.error("USB camera stopped returning frames; "
                                     "releasing the capture.")
                        break
                    time.sleep(0.02)
                    continue
                failures = 0
                with self._lock:
                    self._color = frame
                    self._timestamp = time.perf_counter()
        finally:
            self._stop.set()
            cap.release()
            self._cap = None

    # ---------------------------------------------------------- intrinsics
    def factory_intrinsics(self) -> Result[CameraIntrinsics]:
        return Result.err_msg(
            "A USB camera reports no intrinsics. Run the intrinsic "
            "calibration for this camera at {}x{}, or load a stored set from "
            "the library, before capturing hand-eye samples.".format(
                self.config.width, self.config.height))

    # -------------------------------------------------------- live options
    def set_auto_exposure(self, enabled: bool) -> Result[None]:
        """UVC auto-exposure values are backend-specific, not driver-specific,
        and the two backends disagree on the same numbers -- see
        _AUTO_EXPOSURE_ON. The value actually written is reported back rather
        than hidden, because nothing else in the app can tell whether the lock
        took."""
        cap = self._cap
        if cap is None:
            return Result.err_msg("Camera not connected.")
        value = _AUTO_EXPOSURE_ON if enabled else _AUTO_EXPOSURE_OFF
        if not cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, value):
            return Result.err_msg(
                "This camera or driver rejected the auto-exposure setting.")
        return Result.ok_msg(
            "Auto-exposure {} (CAP_PROP_AUTO_EXPOSURE = {:.0f}).".format(
                "on" if enabled else "off", value))

    def set_exposure(self, value: int) -> Result[None]:
        """`value` is MICROSECONDS -- the unit CameraConfig.exposure and the
        one shared spin box use. DSHOW does not take microseconds: it hands the
        value straight to IAMCameraControl::Set(CameraControl_Exposure), which
        Microsoft documents as log base 2 of the exposure time in seconds
        (-3 = 1/8 s). Passing 8000 there asks for 2^8000 seconds, so the
        conversion happens here and the message states what was written."""
        cap = self._cap
        if cap is None:
            return Result.err_msg("Camera not connected.")
        if value <= 0:
            return Result.err_msg(
                "Exposure must be a positive number of microseconds.")
        if not _USE_DSHOW:
            if not cap.set(cv2.CAP_PROP_EXPOSURE, float(value)):
                return Result.err_msg(
                    "This camera rejected the exposure value.")
            return Result.ok_msg("Exposure set to {} us.".format(value))
        raw = int(round(math.log(float(value) * 1e-6, 2)))
        raw = max(_DSHOW_EXPOSURE_MIN, min(_DSHOW_EXPOSURE_MAX, raw))
        if not cap.set(cv2.CAP_PROP_EXPOSURE, float(raw)):
            return Result.err_msg("This camera rejected the exposure value.")
        return Result.ok_msg(
            "Exposure set to {} (CAP_PROP_EXPOSURE, log2 seconds) = about "
            "{:.0f} us; {} us was requested.".format(
                raw, (2.0 ** raw) * 1e6, value))

    def set_gain(self, gain: int) -> Result[None]:
        cap = self._cap
        if cap is None:
            return Result.err_msg("Camera not connected.")
        if not cap.set(cv2.CAP_PROP_GAIN, float(gain)):
            return Result.err_msg("This camera rejected the gain value.")
        return Result.ok_msg("Gain set to {}.".format(gain))
