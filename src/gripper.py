#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""乐白 LMG-90 独立夹爪封装。

直接走 RS485 Modbus RTU（纯 Python + pyserial），替代不稳定且会段错误的
pylebai 原生库。寄存器定义来自乐白 SDK 文档：
  0x9C40 设置开合位置(0-100)   0x9C41 设置夹持力(0-100)
  0x9C45 读当前位置            0x9C46 读当前力度
  0x9C47 执行校准              0x9C48 读校准状态
  0x9C4A/0x9C4B 速度(非掉电/掉电)  0x9C9A 自动校准开关
"""
import glob
import time

import serial


REG_POS_W = 0x9C40   # 设置位置
REG_FORCE_W = 0x9C41  # 设置力度
REG_POS_R = 0x9C45   # 读位置
REG_FORCE_R = 0x9C46  # 读力度
REG_CAL = 0x9C47     # 执行校准
REG_CAL_ST = 0x9C48  # 校准状态
REG_SPEED = 0x9C4A   # 速度(非掉电)


def find_port():
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/serial/by-id/*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


def _crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


class ModbusRTU:
    """最小 Modbus RTU 客户端（只实现读/写单个保持寄存器）。"""

    def __init__(self, port, baud=115200, slave=1, timeout=0.3, retries=5):
        self.ser = serial.Serial(port, baud, timeout=timeout,
                                 bytesize=8, parity="N", stopbits=1)
        self.slave = slave
        self.retries = retries

    def close(self):
        self.ser.close()

    def _transact(self, request, resp_len):
        last_err = None
        for attempt in range(self.retries):
            self.ser.reset_input_buffer()
            try:
                self.ser.write(bytes(request))
                resp = self.ser.read(resp_len)
                if len(resp) != resp_len:
                    raise TimeoutError(f"响应不完整 {len(resp)}/{resp_len} 字节")
                if resp[0] != self.slave:
                    raise OSError(f"从站地址不符: {resp[0]}")
                if _crc16(resp[:-2]) != int.from_bytes(resp[-2:], "little"):
                    raise OSError("CRC 校验失败")
                return resp
            except Exception as e:
                last_err = e
                time.sleep(0.08)
        raise RuntimeError(f"Modbus 通信失败({self.retries} 次): {last_err}")

    def read_holding(self, reg):
        req = [self.slave, 0x03, (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01]
        crc = _crc16(req)
        resp = self._transact(req + [crc & 0xFF, (crc >> 8) & 0xFF], 7)
        return resp[3] * 256 + resp[4]

    def write_holding(self, reg, value):
        req = [self.slave, 0x06, (reg >> 8) & 0xFF, reg & 0xFF,
               (value >> 8) & 0xFF, value & 0xFF]
        crc = _crc16(req)
        self._transact(req + [crc & 0xFF, (crc >> 8) & 0xFF], 8)


class Gripper:
    def __init__(self, port=None):
        self.port = port or find_port()
        if self.port is None:
            raise RuntimeError("未找到夹爪串口, 请用 port= 指定")
        self.modbus = ModbusRTU(self.port)
        print(f"[LMG-90] 已连接 {self.port} (RS485 Modbus RTU)")

    def set_speed(self, speed):
        self.modbus.write_holding(REG_SPEED, speed)

    def set_force(self, force):
        self.modbus.write_holding(REG_FORCE_W, force)

    def set_position(self, pos):
        self.modbus.write_holding(REG_POS_W, pos)

    def get_position(self):
        return self.modbus.read_holding(REG_POS_R)

    def get_force(self):
        return self.modbus.read_holding(REG_FORCE_R)

    def calibrate(self):
        self.modbus.write_holding(REG_CAL, 1)

    def is_calibrated(self):
        return self.modbus.read_holding(REG_CAL_ST)

    def open(self, pos=100, timeout=8.0, tolerance=3):
        print(f"[LMG-90] 张开到 {pos}%")
        self.set_position(pos)
        self._wait_reach(pos, timeout, tolerance)

    def close(self, pos, force=None, timeout=8.0, tolerance=3):
        print(f"[LMG-90] 闭合到 {pos}%" +
              (f", 力度 {force}" if force is not None else ", 不控力度"))
        if force is not None:
            self.set_force(force)
        self.set_position(pos)
        self._wait_reach(pos, timeout, tolerance)

    def _wait_reach(self, target, timeout=8.0, tolerance=3):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                pos = self.get_position()
            except RuntimeError:
                time.sleep(0.1)
                continue
            if abs(pos - target) <= tolerance:
                print(f"[LMG-90] 到位: 位置 {pos}")
                return True
            time.sleep(0.1)
        print(f"[LMG-90] 提示: 位置未在 {timeout}s 内到达 {target} "
              f"(当前 {self.get_position()}), 检查是否卡住")
        return False

    def status(self):
        return self.get_position(), self.get_force(), self.is_calibrated()


if __name__ == "__main__":
    g = Gripper()
    print("状态(位置, 力度, 校准):", g.status())
