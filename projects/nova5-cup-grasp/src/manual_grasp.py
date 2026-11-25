#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手动/自动抓取模式。

默认深度触发（几何法，推荐）：俯拍相机看不到杯心的夹爪（杯壁遮挡），
所以盯"杯子周围的环形带"。夹爪手指在抓取高度(离桌面约 40mm)时, 深度
落在"桌面深度 - 15~70mm"的带内；手在高处、桌面、杯底都不在此带。
夹爪一靠近杯侧, 带内出现像素 -> 持续 N 帧即自动合爪。

用法:
  python3 manual_grasp.py                       # 深度带触发(默认)
  python3 manual_grasp.py --trigger motion      # 旧的运动检测(备选)
  python3 manual_grasp.py --no-auto             # 纯手动(按键)

按键(在本终端直接按, 无需回车):
  a  自动开关     g  合爪(手动)     o  张爪(手动)
  s  打印状态     q  退出
"""
import argparse
import queue
import sys
import termios
import threading
import time
import tty

import cv2
import numpy as np

from camera import D405Camera
from config import GRIPPER_FORCE, GRIPPER_GRASP, GRIPPER_OPEN, YOLO_CONF
from detect import detect_hsv, detect_yolo
from gripper import Gripper

# ---------- 默认参数(可用命令行覆盖) ----------
TABLE_WARMUP = 10       # 启动时用多少帧估桌面深度
BAND_CLOSE = 15         # 抓取高度带: 深度比桌面近 15mm 以内(不含)
BAND_FAR = 70           # 抓取高度带: 深度比桌面近 70mm 以内(含)
CUP_R = 110             # 杯轮廓半径(px, 排除杯子自身)
RING = 70               # 环形带宽度(px)
MIN_PX = 40             # 带内最少像素数(夹爪出现即>0, 抗噪)
AUTO_FRAMES = 3         # 触发所需连续帧数
AUTO_HOLD_MS = 2500     # 合爪后保持时长
AUTO_COOLDOWN_S = 1.5   # 张开后冷却
SETTLE_FRAMES = 5       # 冷却后需恢复稳定多少帧才重新布防
MAX_LOST_FRAMES = 60    # 杯子连续丢失多少帧后清空中心

# 运动触发(备选)参数
FG_THRESH = 15
BG_ALPHA = 0.01
AUTO_CENTER_BOX = 35
AUTO_COVER = 0.35


class StdinKeys:
    """从终端读取单键输入(无需回车)。非终端环境退化为整行输入。"""

    def __init__(self):
        self.q = queue.Queue()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        if not sys.stdin.isatty():
            for line in sys.stdin:
                for ch in line.strip().lower():
                    self.q.put(ch)
            return
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if not ch:
                    break
                self.q.put(ch.lower())
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def get(self):
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return None


def ring_count(depth, cx, cy, table_depth, cup_r, ring_w, band_close, band_far):
    """计算"杯子外环带"内属于抓取高度带的像素数。"""
    r = cup_r + ring_w
    y0, y1 = max(0, cy - r), cy + r + 1
    x0, x1 = max(0, cx - r), cx + r + 1
    crop = depth[y0:y1, x0:x1]
    if crop.size == 0:
        return 0
    band = ((crop > 0) &
            (crop >= table_depth - band_far) &
            (crop <= table_depth - band_close))
    gy, gx = np.ogrid[:crop.shape[0], :crop.shape[1]]
    dist = np.hypot(gy - (cy - y0), gx - (cx - x0))
    ring = band & (dist > cup_r) & (dist <= r)
    return int(ring.sum())


def main():
    p = argparse.ArgumentParser(description="手动/自动抓取模式")
    p.add_argument("--method", default="yolo", choices=["yolo", "hsv"])
    p.add_argument("--conf", type=float, default=YOLO_CONF)
    p.add_argument("--trigger", default="depth", choices=["depth", "motion"])
    p.add_argument("--no-auto", action="store_true", help="关闭自动抓取")
    p.add_argument("--band-close", type=float, default=BAND_CLOSE)
    p.add_argument("--band-far", type=float, default=BAND_FAR)
    p.add_argument("--cup-r", type=int, default=CUP_R)
    p.add_argument("--ring", type=int, default=RING)
    p.add_argument("--min-px", type=int, default=MIN_PX)
    p.add_argument("--frames", type=int, default=AUTO_FRAMES)
    p.add_argument("--hold-ms", type=int, default=AUTO_HOLD_MS)
    p.add_argument("--fg-thresh", type=int, default=FG_THRESH)
    p.add_argument("--center-box", type=int, default=AUTO_CENTER_BOX)
    p.add_argument("--cover", type=float, default=AUTO_COVER)
    args = p.parse_args()

    auto = not args.no_auto
    cam = D405Camera()
    cam.start()
    g = None

    # 预热并估算桌面深度(mm)
    valid_depths = []
    for _ in range(TABLE_WARMUP):
        _, d = cam.get_frame()
        valid_depths.append(d[d > 0])
    valid_depths = np.concatenate(valid_depths)
    if valid_depths.size == 0:
        cam.stop()
        raise RuntimeError("深度无效: 相机没有有效深度数据")
    table_depth = float(np.median(valid_depths))
    print(f"桌面深度 ≈ {table_depth:.0f}mm, 抓取高度带 = "
          f"{table_depth - args.band_far:.0f} ~ {table_depth - args.band_close:.0f}mm")

    state = "idle"               # idle / holding / cooldown / settle
    dev_frames = 0
    settle_frames = 0
    lost_frames = 0
    hold_deadline = 0.0
    cooldown_deadline = 0.0
    last_cup = None
    bg = None                    # 运动触发: 灰度背景

    try:
        g = Gripper()
        g.open(GRIPPER_OPEN)
        print(f"触发方式: {args.trigger}  自动: {'开' if auto else '关'}")
        print("按键: [a]自动开关 [g]合爪 [o]张爪 [s]状态 [q]退出")
        keys = StdinKeys()

        while True:
            color, depth = cam.get_frame()
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            if bg is None:
                bg = gray.astype(np.float32)
            cv2.accumulateWeighted(gray, bg, BG_ALPHA)
            fg = cv2.absdiff(gray, bg.astype(np.uint8)) > args.fg_thresh

            res = (detect_yolo(color, conf=args.conf) if args.method == "yolo"
                   else detect_hsv(color))
            if res:
                last_cup = (int(res["cx"]), int(res["cy"]))
                lost_frames = 0
            else:
                lost_frames += 1
                if lost_frames >= MAX_LOST_FRAMES:
                    last_cup = None

            # ---- 触发信号 ----
            trigger_hit = False
            ring_px = 0
            if args.trigger == "depth" and last_cup:
                cx, cy = last_cup
                ring_px = ring_count(depth, cx, cy, table_depth,
                                     args.cup_r, args.ring,
                                     args.band_close, args.band_far)
                dev_frames = dev_frames + 1 if ring_px >= args.min_px else 0
                if dev_frames >= args.frames:
                    trigger_hit = True
                    dev_frames = 0
            elif args.trigger == "motion" and last_cup:
                cx, cy = last_cup
                y0, y1 = max(0, cy - args.center_box), cy + args.center_box + 1
                x0, x1 = max(0, cx - args.center_box), cx + args.center_box + 1
                box = fg[y0:y1, x0:x1]
                cover = float(box.mean()) > args.cover if box.size else 0.0
                dev_frames = dev_frames + 1 if cover else 0
                if dev_frames >= args.frames:
                    trigger_hit = True
                    dev_frames = 0

            # ---- 状态机 ----
            now = time.time()
            if state == "idle":
                if auto and trigger_hit:
                    print("→ 夹爪已靠近杯侧, 自动合爪")
                    g.close(GRIPPER_GRASP, GRIPPER_FORCE)
                    state = "holding"
                    hold_deadline = now + args.hold_ms / 1000.0
            elif state == "holding":
                if now >= hold_deadline:
                    print("→ 自动张开")
                    g.open(GRIPPER_OPEN)
                    state = "cooldown"
                    cooldown_deadline = now + AUTO_COOLDOWN_S
                    if args.trigger == "motion":
                        bg = gray.astype(np.float32)
            elif state == "cooldown":
                if now >= cooldown_deadline:
                    state = "settle"
                    settle_frames = 0
            elif state == "settle":
                # 等环形带恢复干净(夹爪离开), 防止立刻重复合爪
                stable = True
                if args.trigger == "depth" and last_cup:
                    cx, cy = last_cup
                    stable = ring_count(depth, cx, cy, table_depth,
                                        args.cup_r, args.ring,
                                        args.band_close,
                                        args.band_far) < args.min_px // 2
                settle_frames = settle_frames + 1 if stable else 0
                if settle_frames >= SETTLE_FRAMES:
                    state = "idle"

            disp = color.copy()
            if last_cup:
                cx, cy = last_cup
                cv2.circle(disp, (cx, cy), 8, (0, 0, 255), 2)
                cv2.putText(disp, f"cup({cx},{cy})", (cx + 12, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                if args.trigger == "depth":
                    cv2.circle(disp, (cx, cy), args.cup_r, (255, 255, 0), 1)
                    cv2.circle(disp, (cx, cy), args.cup_r + args.ring,
                               (0, 255, 255), 1)
            status = f"{args.trigger} ring:{ring_px} {state}"
            cv2.putText(disp, status + " AUTO:" + ("ON" if auto else "OFF") +
                        " [a]开关 [g]合 [o]张 [q]退出", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("manual_grasp", disp)

            key = cv2.waitKey(30) & 0xFF
            cmd = keys.get()
            if cmd is not None:
                key = ord(cmd)
            if key == ord("\x03"):  # Ctrl+C
                break
            if key == ord("q"):
                break
            if key == ord("a"):
                auto = not auto
                print("自动抓取:", "开" if auto else "关")
            elif key == ord("g"):
                g.close(GRIPPER_GRASP, GRIPPER_FORCE)
                state = "holding"
                hold_deadline = now + args.hold_ms / 1000.0
            elif key == ord("o"):
                g.open(GRIPPER_OPEN)
                state = "cooldown"
                cooldown_deadline = now + AUTO_COOLDOWN_S
                if args.trigger == "motion":
                    bg = gray.astype(np.float32)
            elif key == ord("s"):
                print("夹爪状态(位置, 力度, 校准):", g.status(),
                      " 环形带像素:", ring_px)
    finally:
        if g:
            try:
                g.open(GRIPPER_OPEN)
            except Exception:
                pass
        cam.stop()
        cv2.destroyAllWindows()
        print("已退出")


if __name__ == "__main__":
    main()
