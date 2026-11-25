# -*- coding: utf-8 -*-
"""Shared constants for the Nova-5 calibration app.

Unicode symbols use escape sequences (pure ASCII source per project rules).
"""

import math

APP_NAME = "Nova-5 Eye-in-Hand Calibration"
APP_VERSION = "1.0.0"

# ---------------------------------------------------------------------------
# Unicode symbols
# ---------------------------------------------------------------------------
SYM_DOT = "\u25cf"      # filled circle (status dot)
SYM_PLAY = "\u25b6"     # play
SYM_STOP = "\u25a0"     # stop
SYM_UP = "\u25b2"
SYM_DOWN = "\u25bc"
SYM_ELLIPSIS = "\u2026"
SYM_ARROW = "\u2192"
SYM_DASH = "\u2014"
SYM_DEG = "\u00b0"
SYM_THETA = "\u03b8"
SYM_WARN = "\u26a0"
SYM_CHECK = "\u2713"
SYM_CROSS = "\u2717"
SYM_HOME = "\u2302"

R2D = 180.0 / math.pi
D2R = math.pi / 180.0

# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------
STALE_S = 1.0
LIVE_VIEW_HZ = 20

LOG_COLORS = {
    "DEBUG": "#888888",
    "INFO": "#d4d4d4",
    "WARNING": "#ffd700",
    "ERROR": "#ff6b6b",
    "SUCCESS": "#4ec94e",
}

OK_COLOR = "#4ec94e"
WARN_COLOR = "#ffd700"
BAD_COLOR = "#ff6b6b"
IDLE_COLOR = "#888888"

# Board-detection overlay colours, BGR for cv2.
GHOST_BGR = (60, 190, 255)      # stored reference corners
LIVE_BGR = (80, 255, 120)       # live detection

# ---------------------------------------------------------------------------
# Dark theme
# ---------------------------------------------------------------------------
DARK_QSS = (
    "QWidget{background:#1e1e1e;color:#e0e0e0;font-size:12px;}"
    "QTabWidget::pane{border:1px solid #444;}"
    "QTabBar::tab{background:#2a2a2a;padding:6px 14px;margin-right:2px;}"
    "QTabBar::tab:selected{background:#383838;border-bottom:2px solid #2196f3;}"
    "QTableWidget{gridline-color:#3a3a3a;alternate-background-color:#252525;}"
    "QTableWidget::item{padding:2px 5px;}"
    "QHeaderView::section{background:#252525;padding:5px;"
    "  border:1px solid #3a3a3a;}"
    "QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox{"
    "  background:#2a2a2a;border:1px solid #555;padding:4px;border-radius:3px;}"
    "QLineEdit:focus,QSpinBox:focus,QDoubleSpinBox:focus{border-color:#2196f3;}"
    "QPushButton{"
    "  background:#1565c0;border:none;padding:6px 18px;"
    "  border-radius:4px;min-width:70px;}"
    "QPushButton:hover{background:#1976d2;}"
    "QPushButton:disabled{background:#444;color:#777;}"
    "QGroupBox{border:1px solid #3a3a3a;margin-top:14px;}"
    "QGroupBox::title{subcontrol-origin:margin;padding:0 6px;color:#aaa;}"
    "QTextEdit{background:#141414;border:1px solid #3a3a3a;}"
    "QListWidget{background:#141414;border:1px solid #3a3a3a;}"
    "QScrollBar:vertical{background:#1e1e1e;width:10px;}"
    "QScrollBar::handle:vertical{background:#555;border-radius:5px;"
    "  min-height:20px;}"
    "QCheckBox::indicator{width:13px;height:13px;}"
    "QCheckBox::indicator:unchecked{background:#333;border:1px solid #666;}"
    "QCheckBox::indicator:checked{background:#2196f3;border:1px solid #2196f3;}"
    "QSplitter::handle{background:#2a2a2a;}"
    "QProgressBar{border:1px solid #444;border-radius:3px;background:#2a2a2a;"
    "  text-align:center;}"
    "QProgressBar::chunk{background:#4ec94e;border-radius:2px;}"
)

# Anything that moves the arm or defeats a safety gate.
DANGER_QSS = "color:#ff6b6b;font-weight:600;"
STOP_QSS = ("QPushButton{background:#b71c1c;font-weight:700;padding:8px 22px;}"
            "QPushButton:hover{background:#d32f2f;}")
