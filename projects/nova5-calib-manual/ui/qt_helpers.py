# -*- coding: utf-8 -*-
"""
qt_helpers.py
-------------
Small Qt utilities shared by the GUI widgets: numpy-image to QPixmap
conversion, a generic worker QThread for running blocking api calls off the UI
thread, and an ImageLabel that maps mouse clicks back to image pixel
coordinates (used for click-to-pick).

Per project rules: all Qt enums are fully qualified; all blocking robot/camera
work runs in a QThread; widget methods are only ever called from the UI thread.
"""

from __future__ import annotations

from typing import Any, Callable, Optional

import cv2
import numpy as np
from PySide6.QtCore import QPoint, Qt, QThread, Signal
from PySide6.QtGui import QImage, QMouseEvent, QPixmap
from PySide6.QtWidgets import QFrame, QLabel


def bgr_to_qpixmap(bgr: np.ndarray) -> QPixmap:
    """Convert an OpenCV BGR uint8 image to a QPixmap (detached copy)."""
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    rgb = np.ascontiguousarray(rgb)
    qimg = QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888)
    return QPixmap.fromImage(qimg.copy())


def make_hline() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setFrameShadow(QFrame.Shadow.Sunken)
    return line


def make_vline() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.Shape.VLine)
    line.setFrameShadow(QFrame.Shadow.Sunken)
    return line


class CallableThread(QThread):
    """Run a blocking callable off the UI thread and deliver its return value.

    The callable typically returns a Result. Connect to `done` to receive it.
    """

    done = Signal(object)

    def __init__(self, fn: Callable[..., Any], *args: Any, parent: Optional[Any] = None, **kwargs: Any):
        super().__init__(parent)
        self._fn = fn
        self._args = args
        self._kwargs = kwargs
        self.result: Any = None

    def run(self) -> None:
        self.result = self._fn(*self._args, **self._kwargs)
        self.done.emit(self.result)


class ImageLabel(QLabel):
    """QLabel that displays a cv image scaled-to-fit and reports clicks in the
    original image's pixel coordinates via the `clicked` signal."""

    clicked = Signal(int, int)  # (u, v) in source image pixels

    def __init__(self, parent: Optional[Any] = None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 240)
        self._src_w = 0
        self._src_h = 0
        self._disp_w = 0
        self._disp_h = 0
        self._off_x = 0
        self._off_y = 0

    def set_cv_image(self, bgr: np.ndarray) -> None:
        self._src_h, self._src_w = bgr.shape[:2]
        pix = bgr_to_qpixmap(bgr)
        area = self.size()
        scaled = pix.scaled(
            area.width(),
            area.height(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._disp_w = scaled.width()
        self._disp_h = scaled.height()
        self._off_x = max(0, (area.width() - self._disp_w) // 2)
        self._off_y = max(0, (area.height() - self._disp_h) // 2)
        self.setPixmap(scaled)

    def clear_image(self) -> None:
        self._src_w = self._src_h = 0
        self.clear()

    def _to_source(self, pos: QPoint) -> Optional[tuple[int, int]]:
        if self._src_w == 0 or self._disp_w == 0:
            return None
        x = pos.x() - self._off_x
        y = pos.y() - self._off_y
        if x < 0 or y < 0 or x >= self._disp_w or y >= self._disp_h:
            return None
        u = int(x * self._src_w / self._disp_w)
        v = int(y * self._src_h / self._disp_h)
        u = max(0, min(self._src_w - 1, u))
        v = max(0, min(self._src_h - 1, v))
        return u, v

    def mousePressEvent(self, ev: QMouseEvent) -> None:
        if ev.button() == Qt.MouseButton.LeftButton:
            mapped = self._to_source(ev.position().toPoint())
            if mapped is not None:
                self.clicked.emit(mapped[0], mapped[1])
        super().mousePressEvent(ev)
