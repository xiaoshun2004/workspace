#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/check_opencv.py
---------------------
Report which of the OpenCV symbols this app uses actually exist in the
installed build.

The supported range is opencv-python >= 4.8, < 5, enforced at import in
core/handeye.py. This script is the detailed version of that check: run it
against a fresh virtualenv to confirm a build before spending an afternoon
capturing samples with it.

    python3 tools/check_opencv.py

It searches cv2 and every cv2 submodule for each required name, so a symbol
that exists under a different path is reported with its location rather than
as missing.
"""

import sys
import types

import cv2

# (name, what breaks without it)
REQUIRED = [
    ("aruco.getPredefinedDictionary", "board construction"),
    ("aruco.CharucoBoard", "board construction"),
    ("aruco.CharucoDetector", "board detection"),
    ("solvePnPGeneric", "board pose with the two-fold ambiguity gate"),
    ("SOLVEPNP_IPPE", "board pose with the two-fold ambiguity gate"),
    ("projectPoints", "reprojection error"),
    ("calibrateCamera", "intrinsic calibration"),
    ("undistortPoints", "pixel to ray for non-RealSense cameras"),
    ("Rodrigues", "rotation vector conversions throughout"),
    ("calibrateHandEye", "THE hand-eye solve"),
    ("CALIB_HAND_EYE_TSAI", "hand-eye method selection"),
    ("CALIB_HAND_EYE_PARK", "hand-eye method selection"),
    ("CALIB_HAND_EYE_HORAUD", "hand-eye method selection"),
    ("CALIB_HAND_EYE_ANDREFF", "hand-eye method selection"),
    ("CALIB_HAND_EYE_DANIILIDIS", "hand-eye method selection"),
    ("RQDecomp3x3", "unused, reported for completeness"),
    ("cvtColor", "everything imaging"),
    ("imread", "reading stored sample images"),
    ("imwrite", "saving sample images"),
    ("VideoCapture", "USB cameras"),
    ("applyColorMap", "depth colourmap"),
    ("COLORMAP_TURBO", "depth colourmap"),
    ("drawFrameAxes", "pose axes overlay"),
    ("polylines", "overlays"),
    ("circle", "overlays"),
    ("putText", "overlays"),
    ("drawMarker", "overlays"),
    ("line", "overlays"),
    ("getPerspectiveTransform", "tests only"),
    ("warpPerspective", "tests only"),
]


def submodules():
    """Every module-like attribute of cv2, one level deep."""
    found = {}
    for name in dir(cv2):
        if name.startswith("_"):
            continue
        value = getattr(cv2, name, None)
        if isinstance(value, types.ModuleType) or (
                hasattr(value, "__name__") and hasattr(value, "__dict__")
                and not isinstance(value, type) and not callable(value)):
            found[name] = value
    return found


def resolve(dotted):
    """Walk a dotted name from cv2. Returns the object or None."""
    node = cv2
    for part in dotted.split("."):
        node = getattr(node, part, None)
        if node is None:
            return None
    return node


def search_everywhere(leaf):
    """Where else does this name live? Returns a list of dotted paths."""
    hits = []
    if hasattr(cv2, leaf):
        hits.append("cv2." + leaf)
    for name, module in submodules().items():
        if hasattr(module, leaf):
            hits.append("cv2.{}.{}".format(name, leaf))
    return hits


# 4.8 floor: CharucoBoard.matchImagePoints is broken from Python before 4.8
# (opencv/opencv PR #23138), matching the guard in core/handeye.py.
SUPPORTED_MIN = (4, 8, 0)
SUPPORTED_MAX_EXCLUSIVE = (5, 0, 0)


def parse_version(text):
    parts = []
    for chunk in str(text).split(".")[:3]:
        digits = "".join(c for c in chunk if c.isdigit())
        parts.append(int(digits) if digits else 0)
    while len(parts) < 3:
        parts.append(0)
    return tuple(parts)


def main():
    print("OpenCV {}".format(cv2.__version__))
    print("Python {}".format(sys.version.split()[0]))
    version = parse_version(cv2.__version__)
    supported = SUPPORTED_MIN <= version < SUPPORTED_MAX_EXCLUSIVE
    print("Supported range: >= {} and < {}  ->  {}".format(
        ".".join(str(v) for v in SUPPORTED_MIN),
        ".".join(str(v) for v in SUPPORTED_MAX_EXCLUSIVE),
        "OK" if supported else "OUT OF RANGE"))
    mods = sorted(submodules())
    print("cv2 submodules: {}".format(", ".join(mods) if mods else "(none)"))
    print()

    missing = []
    moved = []
    for dotted, purpose in REQUIRED:
        leaf = dotted.split(".")[-1]
        if resolve(dotted) is not None:
            print("  OK       cv2.{}".format(dotted))
            continue
        elsewhere = [h for h in search_everywhere(leaf)
                     if h != "cv2." + dotted]
        if elsewhere:
            moved.append((dotted, elsewhere, purpose))
            print("  MOVED    cv2.{}  ->  {}".format(
                dotted, ", ".join(elsewhere)))
        else:
            missing.append((dotted, purpose))
            print("  MISSING  cv2.{}   ({})".format(dotted, purpose))

    print()
    if moved:
        print("{} symbol(s) exist under a different path.".format(len(moved)))
    if missing:
        print("{} symbol(s) not found anywhere in this build:".format(
            len(missing)))
        for dotted, purpose in missing:
            print("    cv2.{:32s} needed for: {}".format(dotted, purpose))
    if not moved and not missing:
        print("Every symbol resolves where the app expects it.")

    optional = {"RQDecomp3x3", "getPerspectiveTransform", "warpPerspective"}
    fatal = [d for d, _p in missing if d.split(".")[-1] not in optional]
    if fatal:
        print("\nFATAL: the app cannot run without: {}".format(
            ", ".join(fatal)))
        return 1
    if not supported:
        print("\nEvery symbol resolved, but this OpenCV is outside the "
              "supported range and the app will refuse to start. Install "
              "one in range:  pip install \"opencv-python>=4.8,<5\"")
        return 1
    print("\nNothing the app depends on is missing.")
    return 0


if __name__ == "__main__":
    sys.exit(main())