# -*- coding: utf-8 -*-
"""
api/nova5_api.py
----------------
Dobot Nova-5 over the TCP/IP secondary-development protocol.

  29999  dashboard   settings, queries, enable/disable
  30003  motion      MovL / MovJ / ServoJ / jog
  30004  feedback    1440-byte status packet every 8 ms

WHY THIS FILE DEFINES ITS OWN FEEDBACK DTYPE

The vendor's dobot_api.py numpy dtype has correct offsets for the fields most
people use -- ToolVectorActual at 624, QActual at 432, Tool/User at 1013/1012,
ActualQuaternion at 1384 all land where the protocol document says they do.
But it lumps bytes 1038..1119 into one `reserve2` blob and 1416..1439 into
`reserve3`, and those blobs contain:

    CollisionState        byte 1038
    ArmApproachState      byte 1039
    J4/J5/J6ApproachState bytes 1040..1042
    SafetyState           byte 1420   (emergency / protective stop word)

Those are precisely the fields a calibration app that moves the arm on its own
needs to watch. So the dtype below is built from explicit offsets against the
protocol table, exposing them.

TestValue at byte 48 is documented as 0x0123456789ABCDEF. It is checked on
EVERY packet, not once at startup: it is a free guarantee that the structure
is still aligned and the stream has not slipped, and a slipped stream would
otherwise present as plausible-looking wrong poses.

ORIENTATION UNITS

Nothing here assumes what rx/ry/rz mean. Poses are returned as raw SDK numbers
in a RobotPose, and core/pose_convention.py decides how to read them. The one
assumption made anywhere is translation_scale = 0.001 (millimetres), which the
convention carries and which the wizard reports so it can be corrected.
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from models import (
    FrameIdentity,
    PoseConvention,
    Result,
    RobotPose,
    SafetyConfig,
)

logger = logging.getLogger(__name__)

DASHBOARD_PORT = 29999
MOVE_PORT = 30003
FEEDBACK_PORT = 30004
FEEDBACK_PACKET_BYTES = 1440
TEST_VALUE = 0x0123456789ABCDEF

# Built from the protocol's byte-position table. Little-endian throughout.
NOVA5_FEEDBACK_DTYPE = np.dtype({
    "names": [
        "message_size", "digital_inputs", "digital_outputs", "robot_mode",
        "timestamp", "run_time", "test_value", "speed_scaling", "v_robot",
        "i_robot", "program_state", "q_target", "qd_target", "qdd_target",
        "i_target", "m_target", "q_actual", "qd_actual", "i_actual",
        "actual_tcp_force", "tool_vector_actual", "tcp_speed_actual",
        "tcp_force", "tool_vector_target", "tcp_speed_target",
        "motor_temperatures", "joint_modes", "v_actual", "user_index",
        "tool_index", "run_queued_cmd", "pause_cmd_flag", "velocity_ratio",
        "acceleration_ratio", "brake_status", "enable_status", "drag_status",
        "running_status", "error_status", "jog_status", "robot_type",
        "collision_state", "arm_approach_state", "j4_approach_state",
        "j5_approach_state", "j6_approach_state", "vibration_dis_z",
        "current_command_id", "m_actual", "load", "center_x", "center_y",
        "center_z", "user_frame", "tool_frame", "six_force_value",
        "target_quaternion", "actual_quaternion", "auto_manual_mode",
        "safety_state", "safe_state",
    ],
    "formats": [
        "<u2", "<u8", "<u8", "<u8",
        "<u8", "<u8", "<u8", "<f8", "<f8",
        "<f8", "<f8", "(6,)<f8", "(6,)<f8", "(6,)<f8",
        "(6,)<f8", "(6,)<f8", "(6,)<f8", "(6,)<f8", "(6,)<f8",
        "(6,)<f8", "(6,)<f8", "(6,)<f8",
        "(6,)<f8", "(6,)<f8", "(6,)<f8",
        "(6,)<f8", "(6,)<f8", "(6,)<f8", "i1",
        "i1", "i1", "i1", "i1",
        "i1", "i1", "i1", "i1",
        "i1", "i1", "i1", "i1",
        "i1", "i1", "i1",
        "i1", "i1", "<f8",
        "<u8", "(6,)<f8", "<f8", "<f8", "<f8",
        "<f8", "(6,)<f8", "(6,)<f8", "(6,)<f8",
        "(4,)<f8", "(4,)<f8", "i1",
        "u1", "u1",
    ],
    "offsets": [
        0, 8, 16, 24,
        32, 40, 48, 64, 88,
        96, 104, 192, 240, 288,
        336, 384, 432, 480, 528,
        576, 624, 672,
        720, 768, 816,
        864, 912, 960, 1012,
        1013, 1014, 1015, 1016,
        1017, 1025, 1026, 1027,
        1028, 1029, 1030, 1031,
        1038, 1039, 1040,
        1041, 1042, 1104,
        1112, 1120, 1168, 1176, 1184,
        1192, 1200, 1248, 1304,
        1352, 1384, 1416,
        1420, 1421,
    ],
    "itemsize": FEEDBACK_PACKET_BYTES,
})

# SafetyState (byte 1420) is documented as a bitfield whose bits are ACTIVE
# LOW. The mapping below follows the protocol table. It is decoded but
# deliberately not trusted for anything irreversible until confirmed against
# the real controller -- press the physical e-stop and watch the byte.
#
# A byte that reads 0 is treated as "no safety data" rather than as every
# fault at once. Byte 1420 is a real V4 field, but an older stream leaves it
# reserved and zero-filled, and under the active-low decode that would report
# all eight conditions and block every motion path forever.
SAFETY_BITS = {
    0: "emergency stop",
    1: "protective stop",
    2: "reduced mode",
    3: "not stopped",
    4: "in motion",
    5: "system emergency stop",
    6: "user emergency stop",
    7: "outside safe home",
}

# Set the first time a zero SafetyState is seen, so the warning is logged once
# rather than on every feedback packet.
_safety_state_zero_warned = False


def _warn_safety_state_zero() -> None:
    global _safety_state_zero_warned
    if _safety_state_zero_warned:
        return
    _safety_state_zero_warned = True
    logger.warning(
        "SafetyState (byte 1420) reads 0, which this build treats as no "
        "safety data rather than as all eight faults. Expect this on a "
        "pre-V4 feedback stream: safety-stop detection from the packet is "
        "unavailable, so the physical e-stop is the only stop that counts.")

# V4 table (controller V440 and above, which is what this file targets --
# see SafetyState at byte 1420 and the quaternion at 1384). Mode 11 is
# COLLISION in V4; the older V3 table called it jog.
ROBOT_MODES = {
    1: "init", 2: "brake open", 3: "reserved", 4: "disabled",
    5: "enabled idle", 6: "drag", 7: "running", 8: "single move",
    9: "error", 10: "pause", 11: "collision",
}


@dataclass
class Nova5State:
    """One decoded feedback packet, in raw SDK units."""

    joints: List[float] = field(default_factory=lambda: [0.0] * 6)
    tcp: List[float] = field(default_factory=lambda: [0.0] * 6)
    quaternion: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    tool_index: int = -1
    user_index: int = -1
    tool_frame: List[float] = field(default_factory=lambda: [0.0] * 6)
    user_frame: List[float] = field(default_factory=lambda: [0.0] * 6)
    robot_mode: int = 0
    enabled: bool = False
    running: bool = False
    error: bool = False
    collision: bool = False
    approach_flags: List[bool] = field(default_factory=lambda: [False] * 4)
    safety_state: int = 0xFF
    motor_temperatures: List[float] = field(default_factory=lambda: [0.0] * 6)
    load_kg: float = 0.0
    received_at: float = 0.0

    def to_robot_pose(self) -> RobotPose:
        return RobotPose(
            x=self.tcp[0], y=self.tcp[1], z=self.tcp[2],
            rx=self.tcp[3], ry=self.tcp[4], rz=self.tcp[5],
            joints=list(self.joints), quat=list(self.quaternion),
            tool_index=self.tool_index, user_index=self.user_index,
            tool_frame=list(self.tool_frame), user_frame=list(self.user_frame),
            timestamp=self.received_at, source="feedback")

    def frame_identity(self) -> FrameIdentity:
        return FrameIdentity(tool_index=self.tool_index,
                             user_index=self.user_index,
                             tool_frame=list(self.tool_frame),
                             user_frame=list(self.user_frame))

    def safety_flags(self) -> List[str]:
        """Active safety conditions, decoded active-low.

        A zero byte means the stream carries no safety data (see SAFETY_BITS),
        so it reports no flags instead of the full set the decode would
        otherwise produce.
        """
        if int(self.safety_state) == 0:
            _warn_safety_state_zero()
            return []
        return [name for bit, name in SAFETY_BITS.items()
                if not (self.safety_state >> bit) & 1]

    def mode_name(self) -> str:
        return ROBOT_MODES.get(int(self.robot_mode),
                               "mode {}".format(self.robot_mode))

    def is_stopped_by_safety(self) -> bool:
        """True only when the byte actually reports a stop.

        A zero SafetyState yields no flags and therefore False here: absent
        safety data must not be read as a permanent stop.
        """
        flags = self.safety_flags()
        return any("emergency" in f or "protective" in f for f in flags)


def parse_response(text: str) -> Result[List[str]]:
    """Split 'ErrorID,{v1,v2,...},Cmd();' into its brace-delimited values.

    The payload is the FIRST brace pair after the ErrorID, not the outermost
    one: the controller echoes the command with its arguments verbatim, and
    those arguments can themselves contain braces (InverseSolution with a
    JointNear seed ends in ',1,{j1,...,j6})'). Scanning to the last '}' would
    swallow the echo and make every seeded call unparsable.

    A non-zero ErrorID is an error, and it is returned as one: the controller
    accepting a socket write says nothing about whether it accepted the
    command.
    """
    if not text:
        return Result.err_msg("No response from controller.")
    body = text.strip().rstrip(";")
    head, _, rest = body.partition(",")
    try:
        error_id = int(head.strip())
    except ValueError:
        return Result.err_msg("Unparsable response: {!r}".format(text))
    values: List[str] = []
    start = rest.find("{")
    end = rest.find("}", start + 1) if start >= 0 else -1
    if 0 <= start < end:
        inner = rest[start + 1:end].strip()
        if inner:
            values = [v.strip() for v in inner.split(",")]
    if error_id != 0:
        return Result.err(values,
                          "Controller returned ErrorID {}: {}".format(
                              error_id, text.strip()))
    return Result.ok(values)


# How many stale replies one command will step over before the socket is
# declared out of sync. Each one is a reply to a command that already timed
# out, so a handful is a hiccup and a stream of them is a broken connection.
MAX_STALE_RESPONSES = 8


def command_name(text: str) -> str:
    """The command identifier in 'Name(args)', or in a reply that echoes it.

    Taken as the identifier immediately before the LAST '(' so that it works
    on the whole reply 'ErrorID,{values},Name(args);' as well as on the
    command that was sent. Returns "" when there is no name to find, which the
    caller reads as 'cannot verify' rather than as a mismatch.
    """
    open_paren = text.rfind("(")
    if open_paren < 0:
        return ""
    start = open_paren
    while start > 0 and (text[start - 1].isalnum() or text[start - 1] == "_"):
        start -= 1
    return text[start:open_paren]


class _Socket:
    """One command socket with a lock, so send and receive stay paired.

    Replies are framed on the terminating ';' and matched to the command that
    was sent by the name the controller echoes back. That is what makes a
    timeout survivable: the reply to a command that already timed out arrives
    with the wrong name, is discarded, and the caller still gets its own
    answer. Nothing is drained, closed or reconnected on a timeout -- the
    socket and its receive buffer are kept exactly as they are, because the
    late reply is the buffer's problem and the name check solves it.

    close() may be called from another thread while send_recv is blocked in
    recv; closing the raw socket under it is what unblocks the read.
    """

    def __init__(self, ip: str, port: int, timeout: float = 5.0) -> None:
        self.ip = ip
        self.port = port
        self._timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._buffer = ""

    def connect(self) -> Result[None]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self._timeout)
            sock.connect((self.ip, self.port))
        except (OSError, socket.timeout) as exc:
            return Result.err_msg("Cannot connect to {}:{} -- {}".format(
                self.ip, self.port, exc))
        self._buffer = ""
        self._sock = sock
        return Result.ok(None)

    def close(self) -> None:
        # Cleared before the close so a concurrent send_recv sees the socket
        # go away; the close itself is what unblocks it if it is mid-recv.
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def _take_response(self) -> Optional[str]:
        """One complete ';'-terminated reply off the buffer, or None."""
        end = self._buffer.find(";")
        if end < 0:
            return None
        response = self._buffer[:end + 1]
        self._buffer = self._buffer[end + 1:]
        return response.strip()

    def send_recv(self, command: str) -> Result[List[str]]:
        expected = command_name(command)
        with self._lock:
            # Local reference: close() can null the attribute at any moment.
            sock = self._sock
            if sock is None:
                return Result.err_msg(
                    "Not connected to port {}.".format(self.port))
            try:
                sock.settimeout(self._timeout)
                sock.sendall(command.encode("utf-8"))
            except (OSError, socket.timeout) as exc:
                return Result.err_msg(
                    "Command '{}' failed on port {}: {}".format(
                        command, self.port, exc))
            deadline = time.time() + self._timeout
            discarded = 0
            while True:
                response = self._take_response()
                if response is None:
                    remaining = deadline - time.time()
                    if remaining <= 0.0:
                        return Result.err_msg(
                            "Command '{}' timed out on port {} after "
                            "{:.1f} s.".format(command, self.port,
                                               self._timeout))
                    try:
                        sock.settimeout(remaining)
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        return Result.err_msg(
                            "Command '{}' timed out on port {} after "
                            "{:.1f} s.".format(command, self.port,
                                               self._timeout))
                    except OSError as exc:
                        return Result.err_msg(
                            "Command '{}' failed on port {}: {}".format(
                                command, self.port, exc))
                    if not chunk:
                        return Result.err_msg(
                            "Port {} was closed by the controller while "
                            "waiting for '{}'.".format(self.port, command))
                    self._buffer += chunk.decode("utf-8", errors="replace")
                    continue
                echoed = command_name(response)
                if expected and echoed and echoed != expected:
                    discarded += 1
                    logger.warning(
                        "Port %d: discarding a stale %s reply while waiting "
                        "for %s.", self.port, echoed, expected)
                    if discarded > MAX_STALE_RESPONSES:
                        return Result.err_msg(
                            "Port {} is out of sync: {} unrelated replies "
                            "while waiting for '{}'.".format(
                                self.port, discarded, command))
                    continue
                return parse_response(response)


class Nova5Api:
    """Dashboard + motion + feedback, all behind Result.

    The feedback socket is read by a background thread at the controller's own
    rate; latest() is a snapshot, never a blocking read. Dashboard and motion
    commands block on their socket and so must be called from a worker thread
    in the GUI, never from the UI thread.
    """

    def __init__(self, ip: str = "192.168.5.1",
                 safety: Optional[SafetyConfig] = None) -> None:
        self.ip = ip
        self.safety = safety if safety is not None else SafetyConfig()
        self._dashboard = _Socket(ip, DASHBOARD_PORT)
        self._move = _Socket(ip, MOVE_PORT)
        self._feed_sock: Optional[socket.socket] = None
        self._feed_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._state_lock = threading.Lock()
        self._state: Optional[Nova5State] = None
        self._bad_packets = 0
        self._good_packets = 0

    # ------------------------------------------------------------ lifecycle
    @property
    def is_connected(self) -> bool:
        return (self._dashboard.is_open
                and self._feed_thread is not None
                and self._feed_thread.is_alive())

    def connect(self) -> Result[None]:
        if self.is_connected:
            return Result.ok_msg("Already connected.")
        opened = self._dashboard.connect()
        if opened.is_err:
            return opened
        # MOVE_PORT (30003) is intentionally not connected: it is not
        # listening on this controller, and this manual-calibration workflow
        # only reads poses via 29999/30004 and never sends motion commands.
        try:
            feed = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            feed.settimeout(2.0)
            feed.connect((self.ip, FEEDBACK_PORT))
        except (OSError, socket.timeout) as exc:
            self._dashboard.close()
            self._move.close()
            return Result.err_msg(
                "Cannot open the feedback port {}: {}".format(
                    FEEDBACK_PORT, exc))
        self._feed_sock = feed
        self._stop.clear()
        self._bad_packets = 0
        self._good_packets = 0
        self._feed_thread = threading.Thread(target=self._feed_loop,
                                             name="nova5_feedback",
                                             daemon=True)
        self._feed_thread.start()

        deadline = time.time() + 3.0
        while time.time() < deadline:
            if self.latest().is_ok:
                return Result.ok_msg("Connected to Nova-5 at {}.".format(self.ip))
            time.sleep(0.05)
        self.disconnect()
        return Result.err_msg(
            "Connected the sockets but no valid feedback packet arrived. "
            "Check that the controller is in TCP/IP secondary-development "
            "mode.")

    def disconnect(self) -> Result[None]:
        self._stop.set()
        if self._feed_thread is not None:
            self._feed_thread.join(timeout=2.0)
        self._feed_thread = None
        if self._feed_sock is not None:
            try:
                self._feed_sock.close()
            except OSError:
                pass
        self._feed_sock = None
        self._dashboard.close()
        self._move.close()
        with self._state_lock:
            self._state = None
        return Result.ok_msg("Disconnected.")

    # ------------------------------------------------------------- feedback
    def _feed_loop(self) -> None:
        reason = self._read_feedback()
        if reason is None:
            return          # Requested stop; disconnect() does the tidying.
        self._abandon_connection(reason)

    def _read_feedback(self) -> Optional[str]:
        """Decode packets until stopped or the stream dies.

        Returns None when the stop was requested, otherwise the reason the
        stream ended, for _abandon_connection to report.
        """
        sock = self._feed_sock
        if sock is None:
            return None
        buffer = b""
        while not self._stop.is_set():
            try:
                chunk = sock.recv(FEEDBACK_PACKET_BYTES * 4)
            except socket.timeout:
                continue
            except OSError as exc:
                if self._stop.is_set():
                    return None
                return "the feedback socket failed ({})".format(exc)
            if not chunk:
                return "the controller closed the feedback socket"
            buffer += chunk
            # Consume whole packets and keep only the newest; falling behind
            # on a status stream is not worth the latency of catching up.
            while len(buffer) >= FEEDBACK_PACKET_BYTES:
                packet = buffer[:FEEDBACK_PACKET_BYTES]
                buffer = buffer[FEEDBACK_PACKET_BYTES:]
                self._decode(packet)
        return None

    def _abandon_connection(self, reason: str) -> None:
        """Tear the whole connection down after the feedback stream died.

        The command sockets go with it. is_connected is already False the
        moment this thread exits, so leaving 29999 and 30003 established would
        occupy the controller's single dashboard slot with connections no
        caller can reach: disconnect() is the only thing that closes them and
        the UI only calls it while it believes the robot is connected.
        """
        logger.error(
            "Robot connection lost -- %s. Closing the dashboard and motion "
            "sockets so the controller's slot is freed. The arm is no longer "
            "monitored: any queued motion keeps running unwatched, so stop it "
            "at the pendant and reconnect before moving again.", reason)
        self._stop.set()
        sock = self._feed_sock
        self._feed_sock = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        self._dashboard.close()
        self._move.close()
        with self._state_lock:
            self._state = None

    def _decode(self, packet: bytes) -> None:
        raw = np.frombuffer(packet, dtype=NOVA5_FEEDBACK_DTYPE, count=1)[0]
        if int(raw["test_value"]) != TEST_VALUE:
            # Checked on every packet: a slipped stream would otherwise show
            # up as plausible but wrong poses rather than as an error.
            self._bad_packets += 1
            return
        self._good_packets += 1
        state = Nova5State(
            joints=[float(v) for v in raw["q_actual"]],
            tcp=[float(v) for v in raw["tool_vector_actual"]],
            quaternion=[float(v) for v in raw["actual_quaternion"]],
            tool_index=int(raw["tool_index"]),
            user_index=int(raw["user_index"]),
            tool_frame=[float(v) for v in raw["tool_frame"]],
            user_frame=[float(v) for v in raw["user_frame"]],
            robot_mode=int(raw["robot_mode"]),
            enabled=bool(int(raw["enable_status"])),
            running=bool(int(raw["running_status"])),
            error=bool(int(raw["error_status"])),
            collision=bool(int(raw["collision_state"])),
            approach_flags=[bool(int(raw["arm_approach_state"])),
                            bool(int(raw["j4_approach_state"])),
                            bool(int(raw["j5_approach_state"])),
                            bool(int(raw["j6_approach_state"]))],
            safety_state=int(raw["safety_state"]),
            motor_temperatures=[float(v) for v in raw["motor_temperatures"]],
            load_kg=float(raw["load"]),
            received_at=time.perf_counter(),
        )
        with self._state_lock:
            self._state = state

    def latest(self, max_age_s: float = 1.0) -> Result[Nova5State]:
        """Newest decoded packet, refused when stale.

        A stale pose paired with a fresh image is the failure mode that leaves
        no trace in any later metric, so staleness is an error here rather
        than a warning.
        """
        with self._state_lock:
            state = self._state
        if state is None:
            return Result.err_msg("No feedback packet received yet.")
        age = time.perf_counter() - state.received_at
        if age > max_age_s:
            return Result.err_msg(
                "Feedback is {:.2f} s stale (limit {:.2f} s).".format(
                    age, max_age_s))
        return Result.ok(state)

    def packet_health(self) -> Tuple[int, int]:
        """(good, bad) packet counts since connect. A rising bad count means
        the stream is misaligned and nothing downstream should be trusted."""
        return (self._good_packets, self._bad_packets)

    def pose(self, convention: PoseConvention,
             max_age_s: float = 1.0) -> Result[np.ndarray]:
        """T_mount2base in metres, from the newest feedback packet."""
        state = self.latest(max_age_s)
        if state.is_err:
            return Result.err_msg(state.message)
        return Result.ok(state.data.to_robot_pose().to_se3(convention))

    # ------------------------------------------------------------ dashboard
    def enable(self) -> Result[None]:
        """EnableRobot with the configured payload.

        The payload is not cosmetic: the controller's collision detection is
        computed against it, so an unset payload with a camera bracket bolted
        on makes collision thresholds wrong in the unsafe direction.
        """
        cx, cy, cz = (list(self.safety.payload_center_mm) + [0.0, 0.0, 0.0])[:3]
        if self.safety.payload_kg > 0.0:
            command = "EnableRobot({:f},{:f},{:f},{:f})".format(
                self.safety.payload_kg, cx, cy, cz)
        else:
            command = "EnableRobot()"
        return self._dashboard.send_recv(command)

    def disable(self) -> Result[None]:
        return self._dashboard.send_recv("DisableRobot()")

    def clear_error(self) -> Result[None]:
        return self._dashboard.send_recv("ClearError()")

    def stop(self) -> Result[None]:
        """ResetRobot: halts motion and clears the motion queue.

        This is the software stop wired to the STOP button. It is not a
        substitute for the physical e-stop and the UI says so.
        """
        return self._dashboard.send_recv("ResetRobot()")

    def emergency_stop(self) -> Result[None]:
        return self._dashboard.send_recv("EmergencyStop()")

    def set_speed_factor(self, percent: int) -> Result[None]:
        value = int(max(1, min(100, percent)))
        return self._dashboard.send_recv("SpeedFactor({:d})".format(value))

    def set_collision_level(self, level: int) -> Result[None]:
        return self._dashboard.send_recv(
            "SetCollisionLevel({:d})".format(int(max(0, min(5, level)))))

    def robot_mode(self) -> Result[List[str]]:
        return self._dashboard.send_recv("RobotMode()")

    def get_error_id(self) -> Result[List[str]]:
        return self._dashboard.send_recv("GetErrorID()")

    def get_pose(self, user: Optional[int] = None,
                 tool: Optional[int] = None) -> Result[List[float]]:
        """GetPose over the dashboard socket.

        Kept as a cross-check against the feedback packet rather than as the
        primary source: the packet carries the quaternion too, and having two
        independent readings is what makes the convention solvable.
        """
        if user is None or tool is None:
            command = "GetPose()"
        else:
            command = "GetPose({:d},{:d})".format(int(user), int(tool))
        response = self._dashboard.send_recv(command)
        if response.is_err:
            return Result.err_msg(response.message)
        try:
            return Result.ok([float(v) for v in response.data])
        except ValueError:
            return Result.err_msg(
                "GetPose returned unparsable values: {}".format(response.data))

    def inverse_solution(self, pose: Sequence[float], user: int, tool: int,
                         joint_near: Optional[Sequence[float]] = None
                         ) -> Result[List[float]]:
        """Cartesian -> joints, seeded so the arm keeps its configuration.

        A six-axis arm reaches most poses in several elbow/wrist
        configurations. Commanding a Cartesian target without a seed lets the
        controller pick, and replaying an authored pose set is exactly where a
        silent configuration flip turns a 20 mm move into a swing across the
        cell. JointNear pins it to the configuration the pose was authored in.
        """
        x, y, z, rx, ry, rz = (float(v) for v in pose)
        command = "InverseSolution({:f},{:f},{:f},{:f},{:f},{:f},{:d},{:d}".format(
            x, y, z, rx, ry, rz, int(user), int(tool))
        if joint_near is not None:
            near = ",".join("{:f}".format(float(j)) for j in joint_near)
            command += ",1,{{{}}}".format(near)
        command += ")"
        response = self._dashboard.send_recv(command)
        if response.is_err:
            return Result.err_msg(response.message)
        try:
            return Result.ok([float(v) for v in response.data])
        except ValueError:
            return Result.err_msg(
                "InverseSolution returned unparsable values: {}".format(
                    response.data))

    # --------------------------------------------------------------- motion
    def _motion_params(self, linear: bool) -> List[str]:
        if linear:
            return ["SpeedL={:d}".format(int(self.safety.speed_l_pct)),
                    "AccL={:d}".format(int(self.safety.acc_l_pct))]
        return ["SpeedJ={:d}".format(int(self.safety.speed_j_pct)),
                "AccJ={:d}".format(int(self.safety.acc_j_pct))]

    def move_l(self, pose: Sequence[float], user: int = 0,
               tool: int = 0) -> Result[None]:
        x, y, z, rx, ry, rz = (float(v) for v in pose)
        params = ["User={:d}".format(int(user)), "Tool={:d}".format(int(tool))]
        params.extend(self._motion_params(linear=True))
        command = "MovL({:f},{:f},{:f},{:f},{:f},{:f},{})".format(
            x, y, z, rx, ry, rz, ",".join(params))
        return self._move.send_recv(command)

    def move_j(self, pose: Sequence[float], user: int = 0,
               tool: int = 0) -> Result[None]:
        x, y, z, rx, ry, rz = (float(v) for v in pose)
        params = ["User={:d}".format(int(user)), "Tool={:d}".format(int(tool))]
        params.extend(self._motion_params(linear=False))
        command = "MovJ({:f},{:f},{:f},{:f},{:f},{:f},{})".format(
            x, y, z, rx, ry, rz, ",".join(params))
        return self._move.send_recv(command)

    def joint_move(self, joints: Sequence[float]) -> Result[None]:
        """JointMovJ: the safest way to replay an authored pose, because the
        configuration is reproduced exactly rather than re-solved."""
        if len(joints) != 6:
            return Result.err_msg("Need six joint values.")
        command = "JointMovJ({},{})".format(
            ",".join("{:f}".format(float(j)) for j in joints),
            ",".join(self._motion_params(linear=False)))
        return self._move.send_recv(command)

    def relative_move_tool(self, offsets: Sequence[float],
                           tool: int = 0) -> Result[None]:
        """RelMovLTool: a move expressed in the tool frame.

        This is the natural command for the remount correction, which is a
        flange-frame right multiplication by construction.
        """
        if len(offsets) != 6:
            return Result.err_msg("Need six offsets (x, y, z, rx, ry, rz).")
        command = "RelMovLTool({},{:d},{})".format(
            ",".join("{:f}".format(float(v)) for v in offsets), int(tool),
            ",".join(self._motion_params(linear=True)))
        return self._move.send_recv(command)

    def jog(self, axis_id: str, coord_type: int = 1, user: int = 0,
            tool: int = 0) -> Result[None]:
        """MoveJog. axis_id like 'J1+' or 'X-'; empty string stops."""
        if not axis_id:
            return self._move.send_recv("MoveJog()")
        return self._move.send_recv(
            "MoveJog({},CoordType={:d},User={:d},Tool={:d})".format(
                axis_id, int(coord_type), int(user), int(tool)))

    def stop_jog(self) -> Result[None]:
        return self._move.send_recv("MoveJog()")

    def sync(self) -> Result[None]:
        """Block until the motion queue drains.

        The motion socket's timeout bounds how long this can hang; a move
        slower than that returns a timeout error rather than wedging the
        worker thread forever.
        """
        return self._move.send_recv("Sync()")

    def wait_until_idle(self, timeout_s: float = 30.0,
                        settle_s: float = 0.25) -> Result[None]:
        """Wait for motion to finish using the feedback stream.

        Preferred over Sync() when a settle time is wanted anyway: reading
        running_status costs nothing and does not tie up the motion socket.
        The settle exists because a board detection taken the instant motion
        stops is taken while the arm is still ringing.
        """
        deadline = time.time() + float(timeout_s)
        # A move can start and finish between two feedback packets, so the
        # never-ran case is decided on a short grace window rather than on the
        # whole timeout -- waiting the full timeout_s for a 2 mm servo step is
        # dead time, not caution.
        grace_deadline = time.time() + max(1.0, 4.0 * float(settle_s))
        seen_moving = False
        while time.time() < deadline:
            state = self.latest()
            if state.is_err:
                return Result.err_msg(state.message)
            if state.data.error:
                return Result.err_msg(
                    "Controller is in an error state ({}); motion abandoned.".format(
                        state.data.mode_name()))
            if state.data.is_stopped_by_safety():
                return Result.err_msg(
                    "Safety stop active: {}.".format(
                        ", ".join(state.data.safety_flags())))
            if state.data.running:
                seen_moving = True
            elif seen_moving:
                time.sleep(float(settle_s))
                return Result.ok(None)
            elif time.time() >= grace_deadline:
                # The move may have been too short to catch in the running
                # flag -- or it never started, which reads the same from here.
                time.sleep(float(settle_s))
                return Result.ok_msg("Motion never reported running; assuming "
                                     "a very short move completed.")
            time.sleep(0.02)
        if not seen_moving:
            # Only reachable when timeout_s is shorter than the grace window.
            time.sleep(float(settle_s))
            return Result.ok_msg("Motion never reported running; assuming a "
                                 "very short move completed.")
        return Result.err_msg(
            "Motion did not finish within {:.0f} s.".format(timeout_s))


# ---------------------------------------------------------------------------
# Safety gate
# ---------------------------------------------------------------------------
def check_move(target_pose: Sequence[float], state: Nova5State,
               safety: SafetyConfig,
               joints: Optional[Sequence[float]] = None) -> Result[None]:
    """Everything that must be true before an automated move is issued.

    Every gate is individually defeatable through SafetyConfig and every one
    defaults to on. Turning one off is a decision the operator makes in the
    settings, not something the code does quietly because a check was
    inconvenient.
    """
    if not safety.enabled:
        return Result.ok_msg("Safety gates disabled by configuration.")

    if state.error:
        return Result.err_msg(
            "Controller is in an error state ({}). Clear the error before "
            "moving.".format(state.mode_name()))
    if state.collision:
        return Result.err_msg(
            "Controller reports a collision state. Clear it before moving.")
    if state.is_stopped_by_safety():
        return Result.err_msg("Safety stop active: {}.".format(
            ", ".join(state.safety_flags())))
    if safety.require_enabled_before_move and not state.enabled:
        return Result.err_msg("Arm is not enabled.")

    x, y, z = (float(v) for v in target_pose[:3])
    if safety.check_workspace_box:
        lo = list(safety.workspace_min_mm)
        hi = list(safety.workspace_max_mm)
        for value, low, high, name in zip((x, y, z), lo, hi, "XYZ"):
            if value < low or value > high:
                return Result.err_msg(
                    "Target {} = {:.1f} mm is outside the configured "
                    "workspace box [{:.0f}, {:.0f}].".format(
                        name, value, low, high))

    current = np.array(state.tcp[:3], dtype=np.float64)
    step_mm = float(np.linalg.norm(np.array([x, y, z]) - current))
    if safety.max_step_mm > 0.0 and step_mm > safety.max_step_mm:
        return Result.err_msg(
            "Move of {:.0f} mm exceeds the {:.0f} mm single-step limit. Break "
            "it up or raise the limit deliberately.".format(
                step_mm, safety.max_step_mm))

    if joints is not None and safety.check_joint_limits:
        for i, value in enumerate(joints):
            low = safety.joint_min_deg[i] if i < len(safety.joint_min_deg) else -360.0
            high = safety.joint_max_deg[i] if i < len(safety.joint_max_deg) else 360.0
            if value < low or value > high:
                return Result.err_msg(
                    "Solved J{} = {:.1f} deg is outside the configured limit "
                    "[{:.0f}, {:.0f}].".format(i + 1, value, low, high))
        if state.joints:
            worst = max(abs(float(a) - float(b))
                        for a, b in zip(joints, state.joints))
            if safety.max_step_deg > 0.0 and worst > safety.max_step_deg:
                return Result.err_msg(
                    "Largest joint change is {:.0f} deg, over the {:.0f} deg "
                    "step limit -- this usually means the arm would flip "
                    "configuration.".format(worst, safety.max_step_deg))
    return Result.ok(None)
