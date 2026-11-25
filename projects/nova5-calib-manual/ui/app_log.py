"""Embedded log that is ALSO written to a file simultaneously.

AppLog is a QObject hub: any component emits log lines through it (thread-safe
via queued signal), every line is appended to the log file immediately, and
any number of LogView widgets (one per tab if desired) display the stream.
"""

from __future__ import annotations

import os
import threading
import time

from PySide6.QtCore import QObject, Signal, Slot
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import QTextEdit

from constants import LOG_COLORS


class AppLog(QObject):
    """Central log hub. log() may be called from any thread."""

    line = Signal(str, str)   # (message, level)

    def __init__(self, file_path: str, parent=None) -> None:
        super().__init__(parent)
        self._file_path = file_path
        self._file_lock = threading.Lock()
        os.makedirs(os.path.dirname(file_path) or ".", exist_ok=True)

    def log(self, message: str, level: str = "INFO") -> None:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        text = "[{}] [{}] {}".format(stamp, level, message)
        # File write is synchronous so the file always mirrors the UI.
        with self._file_lock:
            try:
                with open(self._file_path, "a", encoding="utf-8") as f:
                    f.write(text + "\n")
            except OSError:
                pass  # never let logging kill the app
        self.line.emit(message, level)

    # Convenience wrappers
    def info(self, msg: str) -> None:
        self.log(msg, "INFO")

    def warn(self, msg: str) -> None:
        self.log(msg, "WARNING")

    def error(self, msg: str) -> None:
        self.log(msg, "ERROR")

    def success(self, msg: str) -> None:
        self.log(msg, "SUCCESS")


def _html_escape(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class LogView(QTextEdit):
    """Read-only colored view of an AppLog stream."""

    MAX_BLOCKS = 800

    def __init__(self, app_log: AppLog, parent=None) -> None:
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMinimumHeight(90)
        app_log.line.connect(self.append_line)

    @Slot(str, str)
    def append_line(self, message: str, level: str) -> None:
        color = LOG_COLORS.get(level, "#d4d4d4")
        stamp = time.strftime("%H:%M:%S")
        self.append(
            '<span style="color:#666;">[{}]</span> '
            '<span style="color:{};">{}</span>'.format(
                stamp, color, _html_escape(message)))
        doc = self.document()
        if doc.blockCount() > self.MAX_BLOCKS:
            cursor = QTextCursor(doc.firstBlock())
            cursor.select(QTextCursor.SelectionType.BlockUnderCursor)
            cursor.removeSelectedText()
        self.moveCursor(QTextCursor.MoveOperation.End)
