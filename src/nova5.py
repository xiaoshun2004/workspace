#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot Nova 5 运动控制驱动 —— TCP/IP 协议 (V3, 参照官方 TCP-IP-Python-V3)

使用前提:
  1. 电脑与 Nova 5 控制器 LAN1 口网线直连
  2. 电脑固定 IP 设成与 192.168.5.1 同网段 (如 192.168.5.100)
  3. 在 DobotStudio Pro 中把机器人模式切成 "TCP/IP 模式"
  4. `ping 192.168.5.1` 能通

端口:
  29999  Dashboard 控制 (使能/下使能/清错)
  30003  Move       运动 (MovJ/MovL/SpeedL/SpeedJ)
  30004  Feed       状态反馈 (本模块暂未使用)
"""
import socket
import time


class Nova5:
    IP_DEFAULT = "192.168.5.1"
    PORT_DASHBOARD = 29999
    PORT_MOVE = 30003
    PORT_FEED = 30004

    def __init__(self, ip=IP_DEFAULT):
        self.ip = ip
        self._dash = None
        self._move = None
        self._feed = None

    # ---------- 连接 ----------
    def connect(self, timeout=5):
        self._dash = socket.create_connection((self.ip, self.PORT_DASHBOARD), timeout)
        self._move = socket.create_connection((self.ip, self.PORT_MOVE), timeout)
        self._feed = socket.create_connection((self.ip, self.PORT_FEED), timeout)
        time.sleep(0.2)
        print(f"[Nova5] 已连接 {self.ip}")

    def close(self):
        for s in (self._dash, self._move, self._feed):
            if s:
                try:
                    s.close()
                except Exception:
                    pass
        self._dash = self._move = self._feed = None
        print("[Nova5] 已断开")

    # ---------- 底层收发 ----------
    def _send(self, sock, cmd, timeout=3):
        if sock is None:
            raise RuntimeError("[Nova5] 未连接, 先调用 connect()")
        sock.settimeout(timeout)
        sock.sendall((cmd + "\n").encode("utf-8"))
        buf = b""
        while not buf.endswith(b"\n"):
            buf += sock.recv(1024)
        resp = buf.decode("utf-8", "ignore").strip()
        if resp.lower().startswith("error"):
            raise RuntimeError(f"[Nova5] 指令失败 {cmd!r} -> {resp}")
        return resp

    def _dash_cmd(self, cmd):
        return self._send(self._dash, cmd)

    def _move_cmd(self, cmd):
        return self._send(self._move, cmd)

    # ---------- Dashboard ----------
    def enable(self):
        self._dash_cmd("EnableRobot()")
        print("[Nova5] 已使能")

    def disable(self):
        self._dash_cmd("DisableRobot()")
        print("[Nova5] 已下使能")

    def clear_error(self):
        self._dash_cmd("ClearError()")

    def reset_robot(self):
        self._dash_cmd("ResetRobot()")

    # ---------- 速度 ----------
    def set_linear_speed(self, mmps):
        """直线/圆弧运动速度 mm/s。"""
        self._move_cmd(f"SpeedL({mmps})")

    def set_joint_speed(self, percent):
        """关节运动速度比例 1-100。"""
        self._move_cmd(f"SpeedJ({percent})")

    # ---------- 运动 ----------
    @staticmethod
    def _fmt_r(r):
        """姿态参数: 接受 float(度) 或 [rx,ry,rz] 列表。"""
        if isinstance(r, (list, tuple)):
            return "[" + ",".join(f"{v}" for v in r) + "]"
        return f"{r}"

    def mov_j(self, x, y, z, r, user=0, tool=0):
        cmd = f"MovJ({x},{y},{z},{self._fmt_r(r)},0,{user},{tool})"
        return self._move_cmd(cmd)

    def mov_l(self, x, y, z, r, user=0, tool=0):
        cmd = f"MovL({x},{y},{z},{self._fmt_r(r)},0,{user},{tool})"
        return self._move_cmd(cmd)

    def sync(self, timeout=30):
        return self._move_cmd("Sync()", timeout=timeout)


if __name__ == "__main__":
    # 自测: 连接并低速移动到安全点 (运行前确保机械臂周围无障碍)
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--ip", default=Nova5.IP_DEFAULT)
    args = p.parse_args()

    arm = Nova5(args.ip)
    arm.connect()
    arm.clear_error()
    arm.enable()
    arm.set_linear_speed(80)
    arm.set_joint_speed(50)
    arm.mov_l(0, 0, 50, [0, 0, 0])
    arm.sync()
    arm.disable()
    arm.close()
    print("自测完成")
