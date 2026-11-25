#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
乐白独立夹爪 (RS485/Modbus RTU) 命令行控制工具
接口参考: lebai-sdk-manual.md 中 # lebai::l_master::Gripper Class Reference

用法示例:
  python3 gripper_cli.py open            # 张开到最大(100)
  python3 gripper_cli.py open 60         # 张开到 60%
  python3 gripper_cli.py close           # 闭合(0)
  python3 gripper_cli.py close 20        # 闭到 20%
  python3 gripper_cli.py force 50        # 设置夹持力道 50%
  python3 gripper_cli.py speed 80        # 设置开合速度 80%(持久保存)
  python3 gripper_cli.py calibrate       # 执行一次校准
  python3 gripper_cli.py auto-calibrate on   # 打开开机自动校准
  python3 gripper_cli.py status          # 查看当前状态

通用选项:
  --port /dev/ttyACM0   指定串口(默认自动探测)
  --timeout 10          等待到位超时秒数(默认 5)
"""

import argparse
import glob
import sys
import time

from pylebai import gripper


def find_port():
    """自动探测夹爪串口, 返回设备路径或 None。"""
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/serial/by-id/*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


def connect(port):
    """连接夹爪, 端口不存在时给出明确报错。"""
    if port is None:
        sys.exit("未找到夹爪串口。请用 --port 指定, 例如 /dev/ttyACM0")
    if not glob.glob(port):
        sys.exit(f"串口不存在: {port}。可用设备: "
                 f"{glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')}")
    print(f"连接夹爪 {port} ...")
    return gripper.Gripper(port)


def read_retry(fn, retries=3, delay=0.1):
    """读取夹爪寄存器并自动重试, 规避 Modbus RTU 偶发超时。"""
    for i in range(retries):
        try:
            return fn()
        except Exception as e:
            last = e
            time.sleep(delay)
    raise RuntimeError(f"连续 {retries} 次读取失败: {last}")


def wait_reach(g, target, timeout=5.0, tolerance=3):
    """轮询夹爪位置, 直到接近目标或超时。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        pos = read_retry(g.get_current_position, retries=2)
        if abs(pos - target) <= tolerance:
            print(f"到位: 位置 {pos}")
            return True
        time.sleep(0.1)
    print(f"提示: 位置 {read_retry(g.get_current_position)} 未在 {timeout}s 内到达 "
          f"{target}, 请检查夹爪是否被物体卡住")
    return False


def cmd_open(g, args):
    pos = args.position
    print(f"张开到 {pos}% ...")
    g.set_position(pos)
    wait_reach(g, pos, args.timeout)


def cmd_close(g, args):
    pos = args.position
    print(f"闭合到 {pos}% ...")
    g.set_position(pos)
    wait_reach(g, pos, args.timeout)


def cmd_force(g, args):
    g.set_force(args.value)
    print(f"已设置夹持力道 {args.value}%")

    # 读取寄存器确认(请求寄存器 40006)
    print(f"当前力道: {g.get_current_force()}%")


def cmd_speed(g, args):
    g.set_velocity(args.value, persistent=True)
    print(f"已设置开合速度 {args.value}% (持久保存)")


def cmd_calibrate(g, args):
    print("执行校准 ...")
    g.do_calibration()
    time.sleep(2)
    print(f"校准状态: {g.is_calibrated()}")


def cmd_auto_calibrate(g, args):
    if args.value == "on":
        g.turn_on_auto_calibration()
        print("已打开开机自动校准")
    elif args.value == "off":
        g.turn_off_auto_calibration()
        print("已关闭开机自动校准")
    else:
        sys.exit("auto-calibrate 参数需为 on 或 off")


def cmd_status(g, args):
    print(f"已校准   : {read_retry(g.is_calibrated)}")
    print(f"当前位置 : {read_retry(g.get_current_position)}%")
    print(f"当前力道 : {read_retry(g.get_current_force)}%")
    print(f"当前速度 : {read_retry(lambda: g.get_current_velocity(persistent=False))}%")


def build_parser():
    p = argparse.ArgumentParser(
        description="乐白独立夹爪控制工具。不带子命令时进入交互模式。")
    p.add_argument("--port", default=None, help="夹爪串口(默认自动探测)")
    p.add_argument("--timeout", type=float, default=5.0, help="等待到位超时(秒)")
    sub = p.add_subparsers(dest="cmd")

    sp = sub.add_parser("open", help="张开到指定幅度(0-100, 默认100)")
    sp.add_argument("position", nargs="?", type=int, default=100)
    sp.set_defaults(func=cmd_open)

    sp = sub.add_parser("close", help="闭合到指定幅度(0-100, 默认0)")
    sp.add_argument("position", nargs="?", type=int, default=0)
    sp.set_defaults(func=cmd_close)

    sp = sub.add_parser("force", help="设置夹持力道(0-100)")
    sp.add_argument("value", type=int)
    sp.set_defaults(func=cmd_force)

    sp = sub.add_parser("speed", help="设置开合速度(0-100)")
    sp.add_argument("value", type=int)
    sp.set_defaults(func=cmd_speed)

    sp = sub.add_parser("calibrate", help="执行一次校准")
    sp.set_defaults(func=cmd_calibrate)

    sp = sub.add_parser("auto-calibrate", help="设置开机自动校准 on/off")
    sp.add_argument("value", choices=["on", "off"])
    sp.set_defaults(func=cmd_auto_calibrate)

    sp = sub.add_parser("status", help="查看当前状态")
    sp.set_defaults(func=cmd_status)
    return p


HELP_TEXT = """可用命令:
  open [幅度]          张开(默认100)
  close [幅度]         闭合(默认0)
  force <0-100>        设置夹持力道
  speed <0-100>        设置开合速度(持久保存)
  calibrate            执行校准
  auto <on|off>        开关开机自动校准
  status               查看当前状态
  help                 显示帮助
  quit / exit          退出
"""


def interactive(g, args):
    """交互模式: 运行后输入命令, 例如 open 80 / close / status。"""
    print("已连接, 进入交互模式。输入 help 查看命令, quit 退出。")
    while True:
        try:
            line = input("gripper> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        try:
            if cmd in ("open", "close"):
                pos = int(parts[1]) if len(parts) > 1 else (100 if cmd == "open" else 0)
                g.set_position(pos)
                wait_reach(g, pos, args.timeout)
            elif cmd == "force":
                g.set_force(int(parts[1]))
                print(f"已设置力道 {parts[1]}%, 当前: {g.get_current_force()}%")
            elif cmd == "speed":
                g.set_velocity(int(parts[1]), persistent=True)
                print(f"已设置速度 {parts[1]}% (持久保存)")
            elif cmd == "calibrate":
                g.do_calibration()
                time.sleep(2)
                print(f"校准状态: {g.is_calibrated()}")
            elif cmd == "auto":
                if len(parts) < 2:
                    print("用法: auto on|off")
                elif parts[1] == "on":
                    g.turn_on_auto_calibration()
                    print("已打开开机自动校准")
                else:
                    g.turn_off_auto_calibration()
                    print("已关闭开机自动校准")
            elif cmd == "status":
                cmd_status(g, args)
            elif cmd in ("quit", "exit", "q"):
                break
            elif cmd == "help":
                print(HELP_TEXT)
            else:
                print(f"未知命令: {cmd} (输入 help 查看可用命令)")
        except (IndexError, ValueError):
            print("参数错误, 例如: open 80 / force 50")
        except Exception as e:
            print(f"错误: {e}")


def main():
    args = build_parser().parse_args()
    port = args.port or find_port()
    g = connect(port)
    if args.cmd:
        args.func(g, args)
    else:
        interactive(g, args)


if __name__ == "__main__":
    main()
