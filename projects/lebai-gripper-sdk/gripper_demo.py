#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
乐白夹爪 键盘交互 Demo —— 抓取/释放面包模型

按键:
  g             抓取(闭合夹爪)
  r             释放(张开夹爪)
  ↑ / ↓         夹持力度 +10 / -10
  ← / →         开合速度 -10 / +10
  h             半开(夹到 50%, 便于观察)
  q             退出

运行方式(请在真实终端中执行, 不要用 ! 前缀):
  python3 gripper_demo.py
"""
import curses
import locale
import threading
import time

from gripper_cli import find_port, connect, read_retry

GRASP_POS = 0        # 抓取目标: 0 = 完全闭合
RELEASE_POS = 100    # 释放目标: 100 = 完全张开
HALF_POS = 50        # 半开位置
STEP = 10            # 力度/速度每次调整步长

# Modbus RTU 是半双工串口, 后台轮询线程与主线程必须互斥访问总线,
# 否则帧碰撞会导致偶发超时抛异常。
_lock = threading.Lock()


def clamp(v, lo=0, hi=100):
    return max(lo, min(hi, v))


def safe_write(state, g, fn, ok_msg):
    """在锁内执行串口写操作; 出错只在界面提示, 不崩溃退出。"""
    with _lock:
        try:
            fn()
            state["msg"] = ok_msg
        except Exception as e:
            state["msg"] = f"{ok_msg} 失败: {type(e).__name__}: {e}"


def draw(stdscr, state):
    stdscr.clear()
    h, _ = stdscr.getmaxyx()
    try:
        stdscr.addstr(0, 0, "=== 乐白夹爪 抓取 Demo ===", curses.A_BOLD)
        stdscr.addstr(2, 0, f"夹爪位置 : {state['position']:>3}%   (0=闭合, 100=张开)")
        stdscr.addstr(3, 0, f"夹持力度 : {state['force']:>3}%   [↑↓ 调整]")
        stdscr.addstr(4, 0, f"开合速度 : {state['speed']:>3}%   [←→ 调整]")
        stdscr.addstr(6, 0, f"> {state['msg']}")
        if h >= 12:
            stdscr.addstr(8, 0, "-" * 40)
            stdscr.addstr(9, 0, " g=抓取  r=释放  h=半开")
            stdscr.addstr(10, 0, " ↑↓=力度  ←→=速度  q=退出")
    except curses.error:
        pass  # 终端太小导致绘制失败时忽略
    stdscr.refresh()


def main(stdscr):
    locale.setlocale(locale.LC_ALL, "")
    curses.curs_set(0)
    stdscr.timeout(100)  # 每 100ms 检查一次按键

    port = find_port()
    g = connect(port)

    state = {
        "position": read_retry(g.get_current_position, retries=3),
        "force": read_retry(g.get_current_force, retries=3),
        "speed": read_retry(lambda: g.get_current_velocity(persistent=False), retries=3),
        "msg": "就绪 · 按 g 抓取面包, r 释放",
    }
    init_speed = state["speed"]

    stop = threading.Event()

    def poll():
        """后台线程持续刷新夹爪实际位置(与主线程互斥访问串口)。"""
        while not stop.is_set():
            with _lock:
                try:
                    state["position"] = read_retry(
                        g.get_current_position, retries=2, delay=0.05)
                except Exception:
                    state["msg"] = "位置读取失败, 检查串口连接"
            time.sleep(0.2)

    threading.Thread(target=poll, daemon=True).start()

    try:
        while True:
            key = stdscr.getch()
            if key == ord("q"):
                if state["speed"] != init_speed:
                    # 退出时把最终速度持久化一次(掉电保持写入较慢, 只在此刻执行)
                    state["msg"] = "退出中, 保存速度..."
                    draw(stdscr, state)
                    safe_write(state, g,
                               lambda: g.set_velocity(state["speed"], persistent=True),
                               f"速度已保存 {state['speed']}%")
                state["msg"] = "退出中..."
                break
            elif key in (ord("g"), ord("G")):
                safe_write(state, g, lambda: g.set_position(GRASP_POS),
                           "抓取中 -> 闭合夹爪")
            elif key in (ord("r"), ord("R")):
                safe_write(state, g, lambda: g.set_position(RELEASE_POS),
                           "释放中 -> 张开夹爪")
            elif key in (ord("h"), ord("H")):
                safe_write(state, g, lambda: g.set_position(HALF_POS),
                           f"半开 -> {HALF_POS}%")
            elif key == curses.KEY_UP:
                state["force"] = clamp(state["force"] + STEP)
                target = state["force"]
                safe_write(state, g, lambda: g.set_force(target),
                           f"力度 +{STEP} -> {target}%")
            elif key == curses.KEY_DOWN:
                state["force"] = clamp(state["force"] - STEP)
                target = state["force"]
                safe_write(state, g, lambda: g.set_force(target),
                           f"力度 -{STEP} -> {target}%")
            elif key == curses.KEY_RIGHT:
                state["speed"] = clamp(state["speed"] + STEP)
                target = state["speed"]
                # 交互用易失写入(persistent=False, ~3ms 即时), 退出时才持久化
                safe_write(state, g,
                           lambda: g.set_velocity(target, persistent=False),
                           f"速度 +{STEP} -> {target}%")
            elif key == curses.KEY_LEFT:
                state["speed"] = clamp(state["speed"] - STEP)
                target = state["speed"]
                safe_write(state, g,
                           lambda: g.set_velocity(target, persistent=False),
                           f"速度 -{STEP} -> {target}%")
            draw(stdscr, state)
    finally:
        stop.set()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        print("\n已退出")
