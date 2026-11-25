"""Small reusable selector for the shared angle/orientation preference.

Drop one into any tab; they all drive the same AngleFormat object, so
changing it in the Joints tab updates the FK inspector, the IK tab and the
MovL dialog at once. Selecting the current value is a no-op, so the widgets
can be rebuilt from the shared state without feedback loops.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Slot
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QWidget

import angle_format
from angle_format import AngleFormat


class AngleFormatSelector(QWidget):
    def __init__(self, fmt: AngleFormat, show_orientation: bool = True,
                 compact: bool = False, parent: Optional[QWidget] = None
                 ) -> None:
        super().__init__(parent)
        self._fmt = fmt
        self._show_orientation = show_orientation

        self._unit_combo = QComboBox()
        self._unit_combo.addItem("radians", angle_format.RAD)
        self._unit_combo.addItem("degrees", angle_format.DEG)
        self._unit_combo.setToolTip(
            "Display and entry unit for joint angles and roll/pitch/yaw. "
            "Values are always stored and sent in radians.")
        self._unit_combo.currentIndexChanged.connect(self._on_unit)

        self._orient_combo = QComboBox()
        self._orient_combo.addItem("roll/pitch/yaw", angle_format.RPY)
        self._orient_combo.addItem("quaternion", angle_format.QUAT)
        self._orient_combo.setToolTip(
            "How orientations are DISPLAYED. Pose entry stays on the "
            "roll/pitch/yaw boxes -- a hand-typed quaternion is rarely a "
            "unit quaternion, and silently renormalising it would hide the "
            "mistake.")
        self._orient_combo.currentIndexChanged.connect(self._on_orient)

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        if not compact:
            row.addWidget(QLabel("Angles:"))
        row.addWidget(self._unit_combo)
        if show_orientation:
            if not compact:
                row.addWidget(QLabel("Orientation:"))
            row.addWidget(self._orient_combo)

        fmt.changed.connect(self._sync)
        self._sync()

    @Slot()
    def _sync(self) -> None:
        for combo, value in ((self._unit_combo, self._fmt.unit),
                             (self._orient_combo, self._fmt.orientation)):
            index = combo.findData(value)
            if index >= 0 and index != combo.currentIndex():
                combo.blockSignals(True)
                combo.setCurrentIndex(index)
                combo.blockSignals(False)

    @Slot(int)
    def _on_unit(self, _index: int) -> None:
        self._fmt.set_unit(self._unit_combo.currentData())

    @Slot(int)
    def _on_orient(self, _index: int) -> None:
        self._fmt.set_orientation(self._orient_combo.currentData())
