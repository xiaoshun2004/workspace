#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v0 主流程：YOLO 检测纸杯 -> 单应 -> 侧抓 -> 固定点放置。

用法:
  python3 pick_cup.py --dry-run      # 只检测+换算坐标, 不动臂(先跑这个)
  python3 pick_cup.py                # 检测到杯子执行一次取放
  python3 pick_cup.py --loop         # 循环取放
  python3 pick_cup.py --method hsv   # 用 HSV 兜底检测
"""
import argparse
import time

from calibration import HomographyCalib
from camera import D405Camera
from config import (
    APPROACH_BACK, ARM_IP, GRASP_R, GRIPPER_FORCE, GRIPPER_GRASP,
    GRIPPER_OPEN, HOME_R, PLACE, SPEED_JOINT, SPEED_LINEAR,
    WORK_X_MAX, WORK_X_MIN, WORK_Y_MAX, WORK_Y_MIN,
    Z_GRASP, Z_PLACE, Z_SAFE,
)
from detect import detect_cup
from gripper import Gripper
from nova5 import Nova5


def cycle(arm, g, x, y):
    """执行一次完整取放：预抓取 -> 侧抓 -> 抬升 -> 放置。"""
    px, py = PLACE
    print(f"→ 移到预抓取点上方 ({x + APPROACH_BACK:.0f}, {y:.0f}, {Z_SAFE})")
    arm.mov_l(x + APPROACH_BACK, y, Z_SAFE, GRASP_R)
    arm.sync()

    print(f"→ 下降到抓取高度(杯旁) ({x + APPROACH_BACK:.0f}, {y:.0f}, {Z_GRASP})")
    arm.mov_l(x + APPROACH_BACK, y, Z_GRASP, GRASP_R)
    arm.sync()

    print(f"→ 水平接近杯子 ({x:.0f}, {y:.0f})")
    arm.mov_l(x, y, Z_GRASP, GRASP_R)
    arm.sync()

    print(f"→ 合爪 (位置 {GRIPPER_GRASP}%, 力度 {GRIPPER_FORCE})")
    g.close(GRIPPER_GRASP, GRIPPER_FORCE)

    print(f"→ 抬升 ({x:.0f}, {y:.0f}, {Z_SAFE})")
    arm.mov_l(x, y, Z_SAFE, GRASP_R)
    arm.sync()

    print(f"→ 移到放置点上方 ({px}, {py})")
    arm.mov_l(px, py, Z_SAFE, GRASP_R)
    arm.sync()

    print("→ 腕部回正")
    arm.mov_l(px, py, Z_SAFE, HOME_R)
    arm.sync()

    print(f"→ 下降到放置高度 ({px}, {py}, {Z_PLACE})")
    arm.mov_l(px, py, Z_PLACE, HOME_R)
    arm.sync()

    print(f"→ 张爪 (位置 {GRIPPER_OPEN}%)")
    g.open(GRIPPER_OPEN)

    print(f"→ 抬升回安全高度 ({px}, {py}, {Z_SAFE})")
    arm.mov_l(px, py, Z_SAFE, HOME_R)
    arm.sync()
    print("✅ 完成一次取放")


def main():
    p = argparse.ArgumentParser(description="v0 纸杯视觉抓取主流程")
    p.add_argument("--dry-run", action="store_true",
                   help="只检测+换算坐标, 不动臂(建议先跑)")
    p.add_argument("--loop", action="store_true", help="循环取放")
    p.add_argument("--calib", default=None, help="标定文件路径")
    p.add_argument("--method", default=None, choices=["yolo", "hsv"])
    p.add_argument("--arm-ip", default=ARM_IP)
    args = p.parse_args()

    from config import CALIB_PATH, DETECT_METHOD

    calib_path = args.calib or str(CALIB_PATH)
    method = args.method or DETECT_METHOD

    calib = HomographyCalib.load(calib_path)
    cam = D405Camera()
    cam.start()
    arm = Nova5(args.arm_ip)
    g = None

    try:
        arm.connect()
        arm.clear_error()
        arm.enable()
        arm.set_linear_speed(SPEED_LINEAR)
        arm.set_joint_speed(SPEED_JOINT)
        arm.mov_l(0, 0, Z_SAFE, HOME_R)  # 先回中立安全点
        arm.sync()

        g = Gripper()
        g.open(GRIPPER_OPEN)

        done_once = False
        while True:
            color, _ = cam.get_frame()
            res = detect_cup(color, method)

            disp = color.copy()
            if res:
                cx, cy = res["cx"], res["cy"]
                x, y = calib.pixel_to_robot(cx, cy)
                cv2.circle(disp, (int(cx), int(cy)), 8, (0, 0, 255), 2)
                cv2.putText(disp, f"robot({x:.0f},{y:.0f})mm", (int(cx) + 12, int(cy) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                print(f"检测到纸杯({res['method']}) 像素({cx:.0f},{cy:.0f}) "
                      f"-> 机器人({x:.1f},{y:.1f})mm")

                in_range = (WORK_X_MIN <= x <= WORK_X_MAX and
                            WORK_Y_MIN <= y <= WORK_Y_MAX)
                if not in_range:
                    print("!! 目标超出工作范围, 跳过")
                elif args.dry_run:
                    print(f"[dry-run] 目标有效: 预抓取点=({x + APPROACH_BACK:.0f},"
                          f"{y:.0f},{Z_GRASP})  抓取点=({x:.0f},{y:.0f},{Z_GRASP})")
                else:
                    cycle(arm, g, x, y)
                    done_once = True
            else:
                cv2.putText(disp, f"no cup ({method})", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("pick_cup", disp)

            key = cv2.waitKey(30) & 0xFF
            if key == ord("q") or (not args.loop and done_once):
                break
            time.sleep(0.05)
    finally:
        if g:
            try:
                g.open(GRIPPER_OPEN)
            except Exception:
                pass
        arm.disable()
        arm.close()
        cam.stop()
        cv2.destroyAllWindows()
        print("已安全退出")


if __name__ == "__main__":
    main()
