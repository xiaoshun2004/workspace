# -*- coding: utf-8 -*-
"""
api/camera_factory.py
---------------------
Discovery and construction. A separate module purely so camera_base does not
have to import its own subclasses -- which would be a cycle, and the only
honest way out of it would be an import inside a function.
"""

from __future__ import annotations

from typing import List

from camera_base import CameraBase
from models import CameraConfig, CameraInfo, CameraKind, Result
from realsense_api import RealSenseCamera
from usb_camera_api import COMMON_MODES, UsbCamera


def enumerate_cameras(max_usb_index: int = 6) -> Result[List[CameraInfo]]:
    """Every camera the app can offer, RealSense first.

    A failed RealSense enumeration is not a failure of this call -- a
    webcam-only setup is normal, and librealsense being unable to see the D405
    (stale driver binding, or the RealSense Viewer holding the device) must not
    hide the webcams. So the result stays ok and carries the backend's message,
    which is the only place the operator can learn why the D405 is missing.
    Only an enumeration that produced nothing at all is an error.

    USB probing opens and closes each index in turn, which is slow and noisy
    on some systems -- hence the ceiling, which is a setting, not a constant.
    """
    found: List[CameraInfo] = []
    devices = RealSenseCamera.list_devices()
    if devices.is_ok:
        for dev in devices.data:
            found.append(CameraInfo(
                kind=CameraKind.REALSENSE, ident=dev["serial"],
                label="{} ({})".format(dev["name"], dev["serial"])))
    found.extend(UsbCamera.list_devices(max_usb_index))
    if devices.is_ok:
        return Result.ok(found)
    if not found:
        return Result.err(found, devices.message)
    # Ok WITH a message: the list is usable, and the caller logs the reason a
    # RealSense may be absent from it. Built directly because Result.ok() takes
    # no message and ok_msg() takes no data.
    return Result(True, found, devices.message)


def supported_modes(kind: CameraKind, ident: str) -> Result[List[tuple]]:
    """Modes to offer for a device. RealSense reports its own exactly; a USB
    camera cannot be asked, so it gets the common list and connect() enforces
    what actually arrives.

    Errors carry the backend's message instead of collapsing to an empty list:
    "the query failed" and "this device has no usable modes" need different
    handling, and a caller that cannot tell them apart leaves the operator with
    an empty mode combo and no explanation. `data` is a list either way, so a
    caller that populates a combo before checking is_err still gets a list.
    """
    if kind == CameraKind.REALSENSE:
        modes = RealSenseCamera.list_supported_modes(ident)
        if modes.is_err:
            return Result.err([], modes.message)
        color = set(modes.data.get("color", []))
        depth = set(modes.data.get("depth", []))
        # Only modes available on BOTH streams are usable here, since depth is
        # always enabled alongside colour at the same resolution.
        both = sorted(color & depth)
        if not both:
            return Result.err(both, (
                "RealSense {} reports no mode available on both the colour "
                "and depth streams, so there is nothing to offer.".format(
                    ident)))
        return Result.ok(both)
    return Result.ok(list(COMMON_MODES))


def make_camera(config: CameraConfig) -> Result[CameraBase]:
    if config.kind == CameraKind.REALSENSE:
        return Result.ok(RealSenseCamera(config))
    if config.kind == CameraKind.USB:
        return Result.ok(UsbCamera(config))
    return Result.err_msg("Unsupported camera kind: {}".format(config.kind))
