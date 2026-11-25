#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""交互式单应标定工具。

流程:
  1. 运行本脚本, 相机画面弹出
  2. 用机械臂把夹爪尖端移到工作台 4 个点(建议覆盖画面四角)
  3. 画面上左键点击对应位置 → 终端输入该点的机器人 X,Y 坐标(mm)
  4. ≥4 个点后按 c 计算并保存 calib.json (打印残差, <3mm 为佳)
  5. q 退出

运行: python3 calibrate_gui.py
"""
import argparse
from pathlib import Path

import cv2

from calibration import HomographyCalib
from camera import D405Camera
from config import CALIB_PATH


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=str(CALIB_PATH))
    args = p.parse_args()

    cam = D405Camera()
    cam.start()
    calib = HomographyCalib((cam.width, cam.height))
    pending_clicks = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pending_clicks.append((x, y))

    cv2.namedWindow("calib")
    cv2.setMouseCallback("calib", on_mouse)
    print("左键点击画面取点, 终端输入该点的机器人 X Y 坐标(mm)")
    print("≥4 个点后按 c 计算保存, 按 q 退出")
    print("-" * 50)

    try:
        while True:
            color, _ = cam.get_frame()
            disp = color.copy()
            for (u, v), (x, y) in zip(calib.pixels, calib.robot_pts):
                cv2.circle(disp, (int(u), int(v)), 5, (0, 0, 255), -1)
                cv2.putText(disp, f"({x:.0f},{y:.0f})", (int(u) + 8, int(v) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(disp, f"点: {len(calib.pixels)}/4+   [c]计算保存  [q]退出",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("calib", disp)

            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            if key == ord("c"):
                try:
                    calib.compute()
                    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
                    calib.save(args.out)
                    mean, mx = calib.reproj_error()
                    print(f"已保存 {args.out}  残差 平均{mean:.2f}mm / 最大{mx:.2f}mm")
                except ValueError as e:
                    print("无法计算:", e)

            if pending_clicks:
                u, v = pending_clicks.pop(0)
                try:
                    txt = input(f"[点 {len(calib.pixels) + 1}] 像素({u},{v}) 的机器人坐标 X Y(mm): ")
                    xr, yr = map(float, txt.replace(",", " ").split())
                    calib.add_point(u, v, xr, yr)
                    print(f"  已记录 像素({u},{v}) -> 机器人({xr},{yr})  (共 {len(calib.pixels)} 个点)")
                except (ValueError, IndexError):
                    print("  输入格式错误, 用空格分隔两个数字, 如: 150 40")
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
