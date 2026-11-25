"""PySide6 GUI for controlling the LeBai LMG-90 gripper through DOBOT Nova."""

import sys
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from nova_client import NovaClient
from gripper import LebaiGripper


class GripperWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("乐白 LMG-90 夹爪控制上位机")
        self.resize(760, 600)

        self.nova = None
        self.gripper = None
        self.connected = False

        self._build_ui()
        self._set_connected(False)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.poll_status)
        self._timer.setInterval(500)

    # ---------- UI construction ----------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        root.addWidget(self._build_conn_group())
        mid = QHBoxLayout()
        mid.addWidget(self._build_control_group(), 1)
        mid.addWidget(self._build_status_group(), 1)
        root.addLayout(mid)
        root.addWidget(self._build_log_group(), 2)

        self.statusBar().showMessage("未连接")

    def _build_conn_group(self):
        box = QGroupBox("连接")
        lay = QHBoxLayout(box)

        lay.addWidget(QLabel("IP"))
        self.ip_edit = QLineEdit("192.168.5.1")
        lay.addWidget(self.ip_edit, 1)

        lay.addWidget(QLabel("端口"))
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(29999)
        lay.addWidget(self.port_spin)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.on_connect)
        lay.addWidget(self.connect_btn)

        self.conn_state = QLabel("○")
        lay.addWidget(self.conn_state)
        return box

    def _build_control_group(self):
        box = QGroupBox("夹爪控制")
        lay = QVBoxLayout(box)

        row_btn = QHBoxLayout()
        self.open_btn = QPushButton("张开 (100)")
        self.open_btn.clicked.connect(lambda: self.gripper_action("open"))
        self.close_btn = QPushButton("闭合 (0)")
        self.close_btn.clicked.connect(lambda: self.gripper_action("close"))
        row_btn.addWidget(self.open_btn)
        row_btn.addWidget(self.close_btn)
        lay.addLayout(row_btn)

        self.ctrl_widgets = []
        self.pos_slider, self.pos_spin = self._make_axis(
            "目标位置", 0, 100, 100, lambda: self.gripper_action("position"), lay
        )
        self.force_slider, self.force_spin = self._make_axis(
            "力度", 0, 100, 50, lambda: self.gripper_action("force"), lay
        )
        self.speed_slider, self.speed_spin = self._make_axis(
            "速度", 0, 100, 80, lambda: self.gripper_action("speed"), lay
        )

        self.find_btn = QPushButton("找行程")
        self.find_btn.clicked.connect(lambda: self.gripper_action("find_stroke"))
        lay.addWidget(self.find_btn)
        return box

    def _make_axis(self, name, low, high, default, on_apply, parent_layout):
        row = QHBoxLayout()
        row.addWidget(QLabel(name))
        slider = QSlider(Qt.Horizontal)
        slider.setRange(low, high)
        slider.setValue(default)
        spin = QSpinBox()
        spin.setRange(low, high)
        spin.setValue(default)
        btn = QPushButton("下发")
        btn.clicked.connect(on_apply)
        slider.valueChanged.connect(spin.setValue)
        spin.valueChanged.connect(slider.setValue)
        row.addWidget(slider, 1)
        row.addWidget(spin)
        row.addWidget(btn)
        parent_layout.addLayout(row)
        self.ctrl_widgets += [slider, spin, btn]
        return slider, spin

    def _build_status_group(self):
        box = QGroupBox("状态")
        lay = QVBoxLayout(box)

        self.pos_label = QLabel("当前位置: -")
        self.force_label = QLabel("当前力矩: -")
        self.done_label = QLabel("指令完成: -")
        self.calib_label = QLabel("未找行程: -")
        for lbl in (self.pos_label, self.force_label, self.done_label, self.calib_label):
            lay.addWidget(lbl)

        self.auto_refresh = QCheckBox("自动刷新 (500ms)")
        self.auto_refresh.setChecked(True)
        self.auto_refresh.toggled.connect(self._on_auto_refresh)
        lay.addWidget(self.auto_refresh)
        lay.addStretch(1)
        return box

    def _build_log_group(self):
        box = QGroupBox("日志")
        lay = QVBoxLayout(box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        lay.addWidget(self.log_view)
        return box

    # ---------- connection ----------

    def on_connect(self):
        if self.connected:
            self.on_disconnect()
            return
        host = self.ip_edit.text().strip()
        port = self.port_spin.value()
        self.log("连接 {}:{} ...".format(host, port))
        try:
            nova = NovaClient(host, port, timeout=3.0)
            nova.connect()
            gripper = LebaiGripper(nova)
            index = gripper.init()
        except Exception as exc:
            self.log("连接失败: {}".format(exc))
            try:
                nova.close()
            except Exception:
                pass
            return
        self.nova = nova
        self.gripper = gripper
        self.connected = True
        self._set_connected(True)
        self.log("已连接，Modbus index={}".format(index))
        self.statusBar().showMessage("已连接 {}:{}".format(host, port))
        self._timer.start()

    def on_disconnect(self):
        self._timer.stop()
        if self.gripper is not None:
            try:
                self.gripper.release()
            except Exception as exc:
                self.log("release 失败: {}".format(exc))
        if self.nova is not None:
            try:
                self.nova.close()
            except Exception:
                pass
        self.gripper = None
        self.nova = None
        self.connected = False
        self._set_connected(False)
        self.statusBar().showMessage("未连接")
        self.log("已断开")
        self._reset_status_labels()

    def _set_connected(self, value):
        self.connected = value
        self.connect_btn.setText("断开" if value else "连接")
        self.conn_state.setText("●" if value else "○")
        self.conn_state.setStyleSheet("color: #2e7d32;" if value else "color: #999;")
        for w in (self.open_btn, self.close_btn, self.find_btn) + tuple(self.ctrl_widgets):
            w.setEnabled(value)

    # ---------- actions ----------

    def gripper_action(self, kind):
        if self.gripper is None:
            return
        try:
            if kind == "open":
                self.gripper.open()
                self.log("张开 (位置=100)")
            elif kind == "close":
                self.gripper.close()
                self.log("闭合 (位置=0)")
            elif kind == "position":
                self.gripper.set_position(self.pos_spin.value())
                self.log("目标位置={}".format(self.pos_spin.value()))
            elif kind == "force":
                self.gripper.set_force(self.force_spin.value())
                self.log("力度={}".format(self.force_spin.value()))
            elif kind == "speed":
                self.gripper.set_speed(self.speed_spin.value())
                self.log("速度={}".format(self.speed_spin.value()))
            elif kind == "find_stroke":
                self.log("触发找行程 ...")
                ok = self.gripper.find_stroke()
                self.log("找行程结果: {}".format("成功" if ok else "超时"))
            self.poll_status()
        except Exception as exc:
            self.log("操作失败: {}".format(exc))

    # ---------- polling ----------

    def poll_status(self):
        if not self.connected or self.gripper is None:
            return
        try:
            st = self.gripper.get_status()
            self.pos_label.setText("当前位置: {}".format(st["position"]))
            self.force_label.setText("当前力矩: {}".format(st["force"]))
            self.done_label.setText("指令完成: {}".format("是" if st["done"] else "否"))
            self.calib_label.setText("未找行程: {}".format("是" if st["not_calibrated"] else "否"))
        except Exception as exc:
            self.log("读取状态失败: {}".format(exc))

    def _on_auto_refresh(self, checked):
        if checked and self.connected:
            self._timer.start()
        else:
            self._timer.stop()

    def _reset_status_labels(self):
        self.pos_label.setText("当前位置: -")
        self.force_label.setText("当前力矩: -")
        self.done_label.setText("指令完成: -")
        self.calib_label.setText("未找行程: -")

    def log(self, message):
        stamp = time.strftime("%H:%M:%S")
        self.log_view.appendPlainText("[{}] {}".format(stamp, message))


def main():
    app = QApplication(sys.argv)
    window = GripperWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
