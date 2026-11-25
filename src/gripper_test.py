#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪独立测试（不依赖相机/机械臂）。

用法:
  python3 gripper_test.py                 # 只读状态
  python3 gripper_test.py --open          # 张开到 100
  python3 gripper_test.py --close 72      # 闭合到 72(自锁到位, 不需要力度)
  python3 gripper_test.py --sweep         # 100->0->100 循环
  python3 gripper_test.py --calibrate     # 执行一次校准
  python3 gripper_test.py --speed 60      # 设置开合速度(0-100)
"""
import argparse
import time

from gripper import Gripper


def main():
    p = argparse.ArgumentParser(description="LMG-90 夹爪独立测试")
    p.add_argument("--port", default=None, help="串口(默认自动探测)")
    p.add_argument("--open", action="store_true", help="张开到 100%")
    p.add_argument("--close", type=int, default=None, metavar="POS",
                   help="闭合到指定位置 0-100")
    p.add_argument("--force", type=int, default=None, help="夹持力 0-100(可选)")
    p.add_argument("--sweep", action="store_true", help="循环 100->0->100")
    p.add_argument("--calibrate", action="store_true", help="执行校准")
    p.add_argument("--speed", type=int, default=None, help="设置开合速度")
    args = p.parse_args()

    g = Gripper(args.port)
    print("状态(位置, 力度, 校准):", g.status())

    if args.speed is not None:
        g.set_speed(args.speed)
        print(f"已设置速度 {args.speed}")
    if args.calibrate:
        print("执行校准...")
        g.calibrate()
        time.sleep(5)
        print("校准状态:", g.is_calibrated())
    if args.open:
        g.open(100)
    if args.close is not None:
        g.close(args.close, args.force)
    if args.sweep:
        for pos in (100, 75, 50, 25, 0, 50, 100):
            g.set_position(pos)
            time.sleep(0.6)
            print(f"  -> 位置 {g.get_position()}")
        g.open(100)

    print("最终状态(位置, 力度, 校准):", g.status())


if __name__ == "__main__":
    main()
