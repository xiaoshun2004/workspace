#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py
-------
Entry point for the Nova-5 eye-in-hand calibration app.

    python3 main.py

sys.path is extended here, at module level in a standalone script, so that
core/, api/ and ui/ can use flat imports throughout. Everything else imports
its dependencies directly at the top of the file: a missing pyrealsense6 or
PySide6 should be an ImportError with a traceback at startup, not a mystery
half an hour into a calibration run.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for _sub in ("core", "api", "ui"):
    _path = os.path.join(_HERE, _sub)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from PySide6.QtWidgets import QApplication

from app_log import AppLog
from constants import APP_NAME, DARK_QSS
from context import AppContext
from main_window import MainWindow
from settings import SettingsStore


def main() -> int:
    store = SettingsStore()
    settings = store.load()
    os.makedirs(settings.root_dir, exist_ok=True)
    log = AppLog(settings.log_file_path)

    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setStyleSheet(DARK_QSS)

    ctx = AppContext(settings, store, log)
    window = MainWindow(ctx)
    window.show()

    if settings.robot_auto_connect:
        log.info("Auto-connect is enabled; use the Robot tab to connect.")
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
