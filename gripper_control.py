"""
乐白独立夹爪 (RS485/Modbus RTU) Python 控制示例
使用 pylebai.gripper.Gripper 直接通过串口控制
"""

from pylebai import gripper
import time


def main():
    # 串口设备路径
    # Linux: 通常为 /dev/ttyUSB0 或 /dev/ttyACM0
    # Windows: COM3 等
    PORT = "/dev/ttyACM0"

    print(f"连接夹爪 {PORT}...")
    g = gripper.Gripper(PORT)

    # --- 校准 ---
    print("执行校准...")
    g.do_calibration()
    time.sleep(2)
    print(f"校准状态: {g.is_calibrated()}")

    # --- 张开 ---
    print("张开到 80%")
    g.set_position(80)
    time.sleep(1)

    # --- 设置力道 ---
    print("设置力道 50%")
    g.set_force(50)

    # --- 闭合 ---
    print("闭合")
    g.set_position(0)
    time.sleep(1)

    # --- 读取状态 ---
    pos = g.get_current_position()
    force = g.get_current_force()
    vel = g.get_current_velocity(persistent=False)
    print(f"位置: {pos}, 力道: {force}, 速度: {vel}")


if __name__ == "__main__":
    main()
