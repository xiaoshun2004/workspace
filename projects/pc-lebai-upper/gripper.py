"""LeBai LMG-90 gripper control through the Nova Modbus bridge.

The Nova controller exposes a local Modbus bridge at 127.0.0.1:60000.
The PC creates the master over TCP (Dashboard) and reads/writes the
gripper holding registers; the controller converts to RTU and drives
the tool RS485 (end-effector connector).
"""

import time

BRIDGE_IP = "127.0.0.1"
BRIDGE_PORT = 60000
DEFAULT_SLAVE_ID = 1

REG_POSITION_CMD = 40000
REG_FORCE_CMD = 40001
REG_CURRENT_POSITION = 40005
REG_CURRENT_FORCE = 40006
REG_EXECUTION_DONE = 40007
REG_FIND_STROKE_CMD = 40008
REG_NOT_CALIBRATED = 40009
REG_SPEED_CMD = 40010


class GripperError(Exception):
    pass


class LebaiGripper:
    def __init__(self, nova, slave_id=DEFAULT_SLAVE_ID, is_rtu=1):
        if not hasattr(nova, "send"):
            raise TypeError("nova must expose a send(command) method")
        self._nova = nova
        self.slave_id = slave_id
        self.is_rtu = is_rtu
        self.index = None

    # ---------- lifecycle ----------

    def init(self, baud=115200, parity="N", stop_bits=1, identify=1):
        err, _, _ = self._nova.send(
            "SetTool485({},{},{},{})".format(baud, parity, stop_bits, identify)
        )
        if err != 0:
            raise GripperError("SetTool485 failed, err={}".format(err))

        err, values, _ = self._nova.send(
            "ModbusCreate({},{},{},{})".format(BRIDGE_IP, BRIDGE_PORT, self.slave_id, self.is_rtu)
        )
        if err != 0 or not values:
            raise GripperError("ModbusCreate failed, err={}".format(err))

        self.index = int(values[0])
        return self.index

    def release(self):
        if self.index is None:
            return 0
        try:
            err, _, _ = self._nova.send("ModbusClose({})".format(self.index))
            return err
        finally:
            self.index = None

    # ---------- low level ----------

    def _read_u16(self, address):
        if self.index is None:
            raise GripperError("gripper not initialized")
        err, values, _ = self._nova.send(
            "GetHoldRegs({},{},1,U16)".format(self.index, address)
        )
        if err != 0 or not values:
            raise GripperError("GetHoldRegs({}) failed, err={}".format(address, err))
        return int(values[0])

    def _write_u16(self, address, value):
        if self.index is None:
            raise GripperError("gripper not initialized")
        err, _, _ = self._nova.send(
            "SetHoldRegs({},{},1,{{{}}},U16)".format(self.index, address, int(value))
        )
        if err != 0:
            raise GripperError("SetHoldRegs({}) failed, err={}".format(address, err))
        return err

    # ---------- status ----------

    def get_position(self):
        return self._read_u16(REG_CURRENT_POSITION)

    def get_force(self):
        return self._read_u16(REG_CURRENT_FORCE)

    def is_done(self):
        return self._read_u16(REG_EXECUTION_DONE) == 1

    def needs_calibration(self):
        return self._read_u16(REG_NOT_CALIBRATED) == 1

    def get_status(self):
        return {
            "position": self.get_position(),
            "force": self.get_force(),
            "done": self.is_done(),
            "not_calibrated": self.needs_calibration(),
        }

    # ---------- control ----------

    def set_force(self, value):
        self._check_range(value)
        self._write_u16(REG_FORCE_CMD, value)

    def set_speed(self, value):
        self._check_range(value)
        self._write_u16(REG_SPEED_CMD, value)

    def set_position(self, value):
        self._check_range(value)
        self._write_u16(REG_POSITION_CMD, value)

    def open(self, target=100):
        self.set_position(target)

    def close(self, target=0):
        self.set_position(target)

    def wait_done(self, timeout_ms=5000, poll_ms=200):
        deadline = time.time() + timeout_ms / 1000.0
        while time.time() < deadline:
            if self.is_done():
                return True
            time.sleep(poll_ms / 1000.0)
        return False

    def find_stroke(self, timeout_ms=10000, poll_ms=200):
        if self.needs_calibration():
            self._write_u16(REG_FIND_STROKE_CMD, 1)
        return self.wait_done(timeout_ms=timeout_ms, poll_ms=poll_ms)

    @staticmethod
    def _check_range(value):
        if not (0 <= int(value) <= 100):
            raise GripperError("value out of range 0..100: {}".format(value))
