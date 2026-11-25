# -*- coding: utf-8 -*-
"""Import-time smoke test.

PySide6 and pyrealsense2 are not installed on every machine that wants to
check this code, and neither is a Nova-5. This installs minimal stand-ins for
both SDKs and then imports every module, which catches the whole class of
mistakes that only surface at import: a name imported from the wrong module, a
constant referenced as a class attribute, a typo in an `except` clause.

It is NOT a substitute for running the app. It proves the modules load and
their cross-references resolve, nothing more.

    python3 tests/smoke_test.py
"""

import os
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _sub in ("core", "api", "ui"):
    sys.path.insert(0, os.path.join(_ROOT, _sub))


class _Signal:
    """Class-level Signal(...) placeholder that behaves per instance."""

    def __init__(self, *args, **kwargs):
        self._slots = []

    def __get__(self, obj, objtype=None):
        return self

    def connect(self, slot):
        self._slots.append(slot)

    def disconnect(self, *args):
        self._slots = []

    def emit(self, *args, **kwargs):
        for slot in list(self._slots):
            slot(*args, **kwargs)


def _slot(*args, **kwargs):
    def wrap(fn):
        return fn
    return wrap


class _Enum:
    """Attribute bag that answers to any nested enum name."""

    def __getattr__(self, name):
        value = _Enum()
        setattr(self, name, value)
        return value

    def __or__(self, other):
        return self

    def __eq__(self, other):
        return self is other

    def __hash__(self):
        return id(self)


class _QtObject:
    def __init__(self, *args, **kwargs):
        pass

    def __getattr__(self, name):
        # Any method call on a stub widget is a no-op returning another stub.
        def anything(*args, **kwargs):
            return _QtObject()
        return anything


def _make_widget_class(name):
    namespace = {}
    for enum_name in ("Shape", "Shadow", "AlignmentFlag", "Orientation",
                      "MouseButton", "AspectRatioMode", "TransformationMode",
                      "Format", "StandardButton", "DialogCode", "WindowType",
                      "WindowModality", "DockWidgetArea", "SelectionType",
                      "MoveOperation", "Policy", "DragDropMode", "RenderHint"):
        namespace[enum_name] = _Enum()
    return type(name, (_QtObject,), namespace)


def install_stubs():
    """Register stub PySide6 and pyrealsense2 modules in sys.modules."""
    qtcore = types.ModuleType("PySide6.QtCore")
    qtgui = types.ModuleType("PySide6.QtGui")
    qtwidgets = types.ModuleType("PySide6.QtWidgets")
    pyside = types.ModuleType("PySide6")

    qtcore.Signal = _Signal
    qtcore.Slot = _slot
    qtcore.Qt = _make_widget_class("Qt")()
    for name in ("QObject", "QThread", "QTimer", "QPoint", "QSize"):
        setattr(qtcore, name, _make_widget_class(name))

    for name in ("QImage", "QPixmap", "QTextCursor", "QMouseEvent",
                 "QCloseEvent", "QColor", "QFont"):
        setattr(qtgui, name, _make_widget_class(name))

    widget_names = (
        "QApplication", "QWidget", "QMainWindow", "QLabel", "QPushButton",
        "QCheckBox", "QComboBox", "QSpinBox", "QDoubleSpinBox", "QLineEdit",
        "QTextEdit", "QListWidget", "QListWidgetItem", "QTableWidget",
        "QGroupBox", "QFrame", "QVBoxLayout", "QHBoxLayout", "QGridLayout",
        "QTabWidget", "QMessageBox", "QProgressBar", "QDockWidget",
        "QInputDialog", "QDialog", "QDialogButtonBox", "QSplitter",
        "QSizePolicy", "QFileDialog",
    )
    for name in widget_names:
        setattr(qtwidgets, name, _make_widget_class(name))

    pyside.QtCore = qtcore
    pyside.QtGui = qtgui
    pyside.QtWidgets = qtwidgets
    sys.modules["PySide6"] = pyside
    sys.modules["PySide6.QtCore"] = qtcore
    sys.modules["PySide6.QtGui"] = qtgui
    sys.modules["PySide6.QtWidgets"] = qtwidgets

    rs = types.ModuleType("pyrealsense2")
    for name in ("pipeline", "config", "context", "align", "stream", "format",
                 "option", "camera_info", "intrinsics", "sensor",
                 "pipeline_profile"):
        setattr(rs, name, _make_widget_class(name))
    rs.rs2_deproject_pixel_to_point = lambda *args: [0.0, 0.0, 0.0]
    sys.modules["pyrealsense2"] = rs


MODULES = [
    # core: no Qt, no SDK
    "transforms", "intrinsics_math", "models", "handeye", "remount",
    "pose_convention", "project_io",
    # api: SDK boundary
    "camera_base", "realsense_api", "usb_camera_api", "camera_factory",
    "intrinsic_calib", "nova5_api",
    # ui
    "constants", "settings", "app_log", "angle_format", "angle_format_widget",
    "qt_helpers", "context", "workers", "tab_camera", "tab_robot",
    "tab_project", "tab_calibrate", "tab_verify", "main_window",
]


def main():
    install_stubs()
    failures = []
    for name in MODULES:
        try:
            __import__(name)
            print("OK   import {}".format(name))
        except Exception as exc:                # noqa: BLE001 - reporting tool
            failures.append((name, exc))
            print("FAIL import {}: {}: {}".format(
                name, type(exc).__name__, exc))
    print()
    if failures:
        print("{} module(s) failed to import.".format(len(failures)))
        return 1
    print("ALL {} MODULES IMPORT CLEANLY".format(len(MODULES)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
