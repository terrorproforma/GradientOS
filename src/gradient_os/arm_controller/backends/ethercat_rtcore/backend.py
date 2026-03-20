from __future__ import annotations

import array
import mmap
import os
import select
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

from ...actuator_interface import ActuatorBackend
from .runtime import rtcore_drive_profile_id_to_name
from ....joint_zero_offsets import load_joint_zero_offsets, save_joint_zero_offsets


def _fourcc(a: str, b: str, c: str, d: str) -> int:
    return (
        (ord(a) & 0xFF) << 0
        | (ord(b) & 0xFF) << 8
        | (ord(c) & 0xFF) << 16
        | (ord(d) & 0xFF) << 24
    )


_MAGIC_GIPC = _fourcc("G", "I", "P", "C")
_MAGIC_GSHM = _fourcc("G", "S", "H", "M")
_MAGIC_RING = _fourcc("R", "I", "N", "G")

_VER_MAJOR = 1
_VER_MINOR = 0

_ROLE_CONTROLLER = 1

_GRADIENT_MAX_AXES = 16

_MSG_STATUS_HELLO = 0x0201
_MSG_STATUS_SNAPSHOT = 0x0202
_MSG_STATUS_AXIS_CONFIG = 0x0203

# Command ring message types (v1)
_MSG_CMD_ARM = 0x0101
_MSG_CMD_AXIS_ENABLE = 0x0102
_MSG_CMD_AXIS_DISABLE = 0x0103
_MSG_CMD_FAULT_RESET = 0x0104
_MSG_CMD_SET_MODE = 0x0106

_MODE_CSP = 8

_HELLO_STRUCT = struct.Struct("<IHHIIQ4Q")  # 56 bytes
_WELCOME_STRUCT = struct.Struct("<IHHI II 4x QQ IIII Q 4Q")  # 96 bytes (includes padding after reserved0)

_SHM_HEADER_STRUCT = struct.Struct("<IHHIIIIQQIIIII4x8Q")  # 128 bytes (includes padding before reserved2)
_RING_HEADER_STRUCT = struct.Struct("<7I")  # 28 bytes
_MSG_HEADER_STRUCT = struct.Struct("<HHIQQ")  # 24 bytes

_CMD_ARM_STRUCT = struct.Struct("<II")
_CMD_AXIS_MASK_STRUCT = struct.Struct("<II")
_CMD_SET_MODE_STRUCT = struct.Struct("<II")
_STATUS_HELLO_STRUCT = struct.Struct("<QQQIIII")
_AXIS_CONFIG_STRUCT = struct.Struct("<II16I16d16i16B16x16d")  # 424 bytes
_STATUS_SNAPSHOT_HEADER_STRUCT = struct.Struct("<IIIIqqQ")


def _align_up(value: int, alignment: int) -> int:
    return ((value + alignment - 1) // alignment) * alignment


def _now_monotonic_ns() -> int:
    # Use monotonic clock to match RTCore.
    return time.monotonic_ns()


@dataclass(frozen=True)
class _ShmHeader:
    kind: int
    num_axes: int
    cycle_ns: int
    topology_hash: int
    ring_offset: int
    ring_capacity: int
    ring_msg_bytes: int
    setpoint_offset: int


@dataclass(frozen=True)
class _AxisConfig:
    num_axes: int
    counts_per_unit: list[float]
    sign: list[int]


class EthercatRTCoreBackend(ActuatorBackend):
    """
    ActuatorBackend proxy to the RTCore daemon (`gradient-rt-motion`).

    This class intentionally does *not* perform any EtherCAT I/O itself.
    It only performs:
    - IPC handshake (UDS + SCM_RIGHTS)
    - setpoint slot writes
    - status ring reads (best-effort, non-RT)
    """

    def __init__(
        self,
        robot_config: dict,
        socket_path: str = "/run/gradient-rt-motion/ipc.sock",
    ) -> None:
        self._robot_config = robot_config
        self._socket_path = socket_path

        self._robot_id = str(robot_config.get("robot_id", "unknown")).strip() or "unknown"
        self._num_joints = int(robot_config.get("num_logical_joints", 6))
        self._master_offsets_rad = load_joint_zero_offsets(
            self._robot_id,
            num_joints=self._num_joints,
            defaults=robot_config.get("logical_joint_master_offsets_rad", [0.0] * self._num_joints),
        )

        self._initialized = False
        self._connected = False
        self._rt_num_axes: int = 0

        # Mapping: RTCore axis index -> GradientOS logical joint index (0-based).
        # Default policy is direct ordering axis0->J1, axis1->J2, ... up to min(num_axes, num_joints).
        # For hardware bring-up/custom wiring, override via GRADIENT_RTCORE_CONTROL_JOINTS.
        self._axis_to_joint: list[int] = []

        # Command ring sequencing (producer-owned).
        self._cmd_seq = 1

        # Auto-arm on successful IPC connect (mirrors how serial servos become usable after init).
        self._auto_arm = os.environ.get("GRADIENT_RTCORE_AUTO_ARM", "0").lower() not in ("0", "false", "no")
        # Optional: restrict which axes are armed/enabled on connect.
        # Examples:
        #   GRADIENT_RTCORE_AUTO_ARM_MASK=0x1   (only axis 0, i.e. slave position 0)
        #   GRADIENT_RTCORE_AUTO_ARM_MASK=0x3   (axes 0 and 1)
        raw_mask = os.environ.get("GRADIENT_RTCORE_AUTO_ARM_MASK", "").strip()
        self._auto_arm_mask: Optional[int] = None
        if raw_mask:
            try:
                # base=0 accepts 0x.. hex or decimal.
                self._auto_arm_mask = int(raw_mask, 0)
            except ValueError:
                print(f"[EtherCAT RTCore] WARNING: invalid GRADIENT_RTCORE_AUTO_ARM_MASK='{raw_mask}'")

        self._sock: Optional[socket.socket] = None

        self._cmd_shm_fd: Optional[int] = None
        self._status_shm_fd: Optional[int] = None
        self._cmd_eventfd: Optional[int] = None
        self._status_eventfd: Optional[int] = None

        self._cmd_shm: Optional[mmap.mmap] = None
        self._status_shm: Optional[mmap.mmap] = None

        self._cmd_hdr: Optional[_ShmHeader] = None
        self._status_hdr: Optional[_ShmHeader] = None
        self._robot_axis_config = self._build_axis_config_from_robot_config(robot_config)
        self._runtime_axis_config: Optional[_AxisConfig] = None
        self._axis_config: Optional[_AxisConfig] = self._robot_axis_config
        self._axis_config_mismatch_logged = False
        self._axis_config_event = threading.Event()
        self._status_snapshot_event = threading.Event()
        self._status_lock = threading.Lock()
        if self._axis_config is not None:
            self._axis_config_event.set()

        # Latest known axis counts from STATUS_SNAPSHOT (pos_counts per axis).
        self._axis_counts: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_statusword: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_error_code: list[int] = [0] * _GRADIENT_MAX_AXES
        self._rt_drive_profile_code = 0
        self._rt_drive_profile_id: Optional[str] = None
        self._last_wkc_expected = 0
        self._last_wkc_actual = 0
        self._last_master_state = 0

        # Latest commanded joint positions (radians) as a safe fallback for getters.
        self._last_joint_setpoint_rad: list[float] = [0.0] * self._num_joints

        self._status_thread: Optional[threading.Thread] = None
        self._status_stop = threading.Event()

        # Setpoint slot sequence (writer-owned).
        self._setpoint_seq = 0

    # -------------------------------------------------------------------------
    # ActuatorBackend required API
    # -------------------------------------------------------------------------

    def initialize(self) -> bool:
        try:
            ok = self._connect_ipc()
            self._initialized = bool(ok)
            return bool(ok)
        except Exception as e:
            # Keep the controller alive, but report as not connected.
            print(f"[EtherCAT RTCore] WARNING: IPC init failed: {e}")
            self._connected = False
            self._initialized = False
            return False

    def shutdown(self) -> None:
        self._best_effort_safe_power_down()
        self._status_stop.set()
        self._axis_config_event.clear()
        self._status_snapshot_event.clear()
        if self._status_thread and self._status_thread.is_alive():
            self._status_thread.join(timeout=1.0)

        self._status_thread = None

        if self._cmd_shm is not None:
            try:
                self._cmd_shm.close()
            except Exception:
                pass
        if self._status_shm is not None:
            try:
                self._status_shm.close()
            except Exception:
                pass

        self._cmd_shm = None
        self._status_shm = None

        for fd_name in ("_cmd_shm_fd", "_status_shm_fd", "_cmd_eventfd", "_status_eventfd"):
            fd = getattr(self, fd_name)
            if isinstance(fd, int):
                try:
                    os.close(fd)
                except Exception:
                    pass
            setattr(self, fd_name, None)

        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None

        self._connected = False
        self._initialized = False
        with self._status_lock:
            self._rt_drive_profile_code = 0
            self._rt_drive_profile_id = None

    def safe_power_down(self) -> bool:
        self._best_effort_safe_power_down()
        return True

    def safe_power_up(self) -> bool:
        self._best_effort_safe_power_up()
        return True

    def get_live_drive_profile_id(self) -> Optional[str]:
        with self._status_lock:
            return self._rt_drive_profile_id

    def reset_faults(self, logical_joint_index: Optional[int] = None) -> bool:
        if not self._connected:
            print("[EtherCAT RTCore] WARNING: cannot reset faults while RTCore is disconnected")
            return False

        if self._rt_num_axes <= 0:
            print("[EtherCAT RTCore] WARNING: cannot reset faults without any configured axes")
            return False

        if logical_joint_index is None:
            axis_mask = (1 << self._rt_num_axes) - 1
            label = "all axes"
        else:
            joint_i = int(logical_joint_index)
            if joint_i < 0 or joint_i >= self._num_joints:
                print(f"[EtherCAT RTCore] WARNING: joint index out of range for fault reset: {joint_i}")
                return False
            axis_mask = 0
            for axis_i, mapped_joint in enumerate(self._axis_to_joint):
                if mapped_joint == joint_i:
                    axis_mask |= (1 << axis_i)
            if axis_mask == 0:
                print(
                    "[EtherCAT RTCore] WARNING: cannot reset faults for joint"
                    f" {joint_i + 1}; no mapped RTCore axes"
                )
                return False
            label = f"joint {joint_i + 1}"

        self._send_cmd_fault_reset(axis_mask=axis_mask)
        print(
            "[EtherCAT RTCore] Fault reset requested:"
            f" target={label} axis_mask=0x{axis_mask:x}"
        )
        return True

    def _best_effort_safe_power_down(self) -> None:
        if not self._connected:
            return

        try:
            axis_mask = (1 << self._rt_num_axes) - 1 if self._rt_num_axes > 0 else 0
            if axis_mask:
                self._send_cmd_axis_disable(axis_mask=axis_mask)
            self._send_cmd_arm(False)
            # Give the RTCore thread a brief window to consume the disable/disarm command
            # before we tear down IPC resources on the Python side.
            time.sleep(0.05)
        except Exception as e:
            print(f"[EtherCAT RTCore] WARNING: safe power-down failed during shutdown: {e}")

    def _best_effort_safe_power_up(self) -> None:
        if not self._connected:
            return
        try:
            axis_mask_all = (1 << self._rt_num_axes) - 1 if self._rt_num_axes > 0 else 0
            axis_mask = axis_mask_all
            if self._auto_arm_mask is not None:
                axis_mask = int(self._auto_arm_mask) & int(axis_mask_all)
            if not axis_mask:
                print("[EtherCAT RTCore] WARNING: safe power-up skipped; no axes selected")
                return
            self._send_cmd_arm(True)
            self._send_cmd_set_mode(axis_mask=axis_mask, mode=_MODE_CSP)
            self._send_cmd_axis_enable(axis_mask=axis_mask)
        except Exception as e:
            print(f"[EtherCAT RTCore] WARNING: safe power-up failed: {e}")

    @property
    def num_joints(self) -> int:
        return self._num_joints

    @property
    def is_initialized(self) -> bool:
        return self._initialized and self._connected

    @property
    def encoder_resolution(self) -> int:
        # EtherCAT RTCore does not use "servo encoder resolution" values here.
        # Returning 0 discourages use in legacy clamp logic.
        return 0

    def set_joint_positions(
        self,
        positions_rad: list[float],
        speed: float,
        acceleration: float,
    ) -> None:
        if len(positions_rad) != self._num_joints:
            raise ValueError(f"Expected {self._num_joints} joint positions, got {len(positions_rad)}")

        self._last_joint_setpoint_rad = list(positions_rad)

        if not self._connected:
            raise RuntimeError("RTCore not connected (cannot send setpoints)")

        if self._rt_num_axes <= 0:
            raise RuntimeError("RTCore did not report a valid num_axes")

        # Build per-axis setpoint vector from the configured mapping.
        axis_q: list[float] = [0.0] * self._rt_num_axes
        for axis_i, joint_i in enumerate(self._axis_to_joint):
            if 0 <= joint_i < len(positions_rad):
                axis_q[axis_i] = float(positions_rad[joint_i]) + self._master_offset_for_joint(joint_i)

        axis_mask = (1 << self._rt_num_axes) - 1
        self._write_setpoint(axis_q, axis_mask=axis_mask)

    def get_joint_positions(self, verbose: bool = False) -> list[float]:
        if self._connected and self._axis_config is None:
            # Give the status thread a brief chance to receive scaling data.
            self._wait_for_feedback_ready(timeout_s=0.25, require_live_wkc=False)
        if self._connected and self._axis_config is not None:
            positions = self.raw_to_joint_positions(self.sync_read_positions())
            if verbose:
                print("[EtherCAT RTCore] get_joint_positions() (connected) -> live feedback")
            return positions
        if verbose:
            state = "connected" if self._connected else "disconnected"
            print(f"[EtherCAT RTCore] get_joint_positions() ({state}) -> last setpoint")
        return list(self._last_joint_setpoint_rad)

    def prepare_sync_write_commands(
        self,
        positions_rad: list[float],
        speed: int = 4095,
        accel: int = 0,
    ) -> list[tuple]:
        # This backend does not support raw SYNC_WRITE-style commands from Python.
        # Return a backend-private command tuple consumed by sync_write().
        return [("setpoint_rad", list(positions_rad))]

    def sync_write(self, commands: list[tuple]) -> None:
        # Accept our private command format only.
        if not commands:
            return

        kind = commands[0][0] if isinstance(commands[0], tuple) and commands[0] else None
        if kind != "setpoint_rad":
            raise NotImplementedError(
                "ethercat_rtcore does not accept raw sync_write tuples; "
                "use set_joint_positions() / prepare_sync_write_commands()."
            )

        positions_rad = commands[0][1]
        if not isinstance(positions_rad, list):
            raise ValueError("Invalid setpoint command format")

        self.set_joint_positions(positions_rad, speed=0.0, acceleration=0.0)

    def sync_read_positions(self, timeout_s: Optional[float] = None) -> dict[int, int]:
        # Return last known raw counts for each exposed axis (index -> pos_counts).
        if not self._connected:
            return {}

        return {i: int(self._axis_counts[i]) for i in range(self._rt_num_axes)}

    def raw_to_joint_positions(self, raw_positions: dict[int, int]) -> list[float]:
        positions = list(self._last_joint_setpoint_rad)
        for axis_i, joint_i in enumerate(self._axis_to_joint):
            raw = raw_positions.get(axis_i)
            if raw is None or not (0 <= joint_i < self._num_joints):
                continue
            physical_q = self._axis_q_from_counts(axis_i, int(raw))
            if physical_q is None:
                continue
            positions[joint_i] = physical_q - self._master_offset_for_joint(joint_i)
        return positions

    def set_single_actuator_position(
        self,
        actuator_id: int,
        position_rad: float,
        speed: int,
        accel: int,
    ) -> None:
        # In EtherCAT/RTCore backend, actuator_id == RTCore axis index.
        if actuator_id < 0 or actuator_id >= self._rt_num_axes:
            raise ValueError(f"Axis index out of range: {actuator_id}")

        if not self._connected:
            raise RuntimeError("RTCore not connected (cannot send setpoints)")

        axis_q = [0.0] * self._rt_num_axes
        axis_q[actuator_id] = float(position_rad)
        axis_mask = 1 << actuator_id
        self._write_setpoint(axis_q, axis_mask=axis_mask)

    def read_single_actuator_position(self, actuator_id: int) -> Optional[int]:
        if not self._connected:
            return None
        if actuator_id < 0 or actuator_id >= self._rt_num_axes:
            return None
        return int(self._axis_counts[actuator_id])

    def set_current_position_as_zero(self, actuator_id: int) -> bool:
        axis_i = int(actuator_id)
        if axis_i < 0 or axis_i >= len(self._axis_to_joint):
            return False
        joint_i = self._axis_to_joint[axis_i]
        if joint_i < 0 or joint_i >= self._num_joints:
            return False
        return self.set_logical_joint_current_position_as_zero(joint_i)

    def set_logical_joint_current_position_as_zero(self, logical_joint_index: int) -> bool:
        joint_i = int(logical_joint_index)
        if joint_i < 0 or joint_i >= self._num_joints:
            print(f"[EtherCAT RTCore] WARNING: joint index out of range for zeroing: {joint_i}")
            return False
        if not self._connected:
            print("[EtherCAT RTCore] WARNING: cannot zero joint while RTCore is disconnected")
            return False

        physical_samples: list[float] = []
        for axis_i, mapped_joint in enumerate(self._axis_to_joint):
            if mapped_joint != joint_i:
                continue
            physical_q = self._axis_q_from_counts(axis_i, int(self._axis_counts[axis_i]))
            if physical_q is not None:
                physical_samples.append(physical_q)

        if not physical_samples:
            print(
                "[EtherCAT RTCore] WARNING: cannot zero joint"
                f" {joint_i + 1}; no live scaled feedback available yet"
            )
            return False

        new_offset = float(sum(physical_samples) / len(physical_samples))
        self._master_offsets_rad[joint_i] = new_offset
        self._last_joint_setpoint_rad[joint_i] = 0.0
        save_joint_zero_offsets(
            self._robot_id,
            self._master_offsets_rad,
            actor=f"ethercat_rtcore:joint{joint_i + 1}",
        )
        print(
            "[EtherCAT RTCore] Captured logical zero:"
            f" joint={joint_i + 1} physical_q={new_offset:.6f} rad"
        )
        return True

    def set_pid_gains(self, actuator_id: int, kp: int, ki: int, kd: int) -> bool:
        # Not supported here (should be RTCore commissioning / SDO templates).
        return False

    def apply_joint_limits(self) -> bool:
        # Limits are enforced in RTCore using axis config + soft limit commands.
        return False

    def get_present_actuator_ids(self) -> set[int]:
        if not self._connected:
            return set()
        return set(range(self._rt_num_axes))

    def ping_actuator(self, actuator_id: int) -> bool:
        return actuator_id in self.get_present_actuator_ids()

    # -------------------------------------------------------------------------
    # IPC internals
    # -------------------------------------------------------------------------

    def _connect_ipc(self) -> bool:
        if self._connected:
            return True

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            sock.connect(self._socket_path)
        except FileNotFoundError:
            print(f"[EtherCAT RTCore] RTCore socket not found: {self._socket_path}")
            sock.close()
            return False
        except Exception as e:
            print(f"[EtherCAT RTCore] Failed to connect to RTCore socket: {e}")
            sock.close()
            return False

        # Send HELLO.
        hello = _HELLO_STRUCT.pack(
            _MAGIC_GIPC,
            _VER_MAJOR,
            _VER_MINOR,
            _HELLO_STRUCT.size,
            _ROLE_CONTROLLER,
            os.getpid(),
            0,
            0,
            0,
            0,
        )
        sock.sendall(hello)

        # Receive WELCOME + FDs.
        fd_size = array.array("i").itemsize
        data, ancdata, _flags, _addr = sock.recvmsg(
            _WELCOME_STRUCT.size,
            socket.CMSG_SPACE(fd_size * 4),
        )
        if len(data) != _WELCOME_STRUCT.size:
            sock.close()
            raise RuntimeError(f"WELCOME size mismatch (got {len(data)} bytes)")

        fds: list[int] = []
        for level, ctype, cmsg_data in ancdata:
            if level == socket.SOL_SOCKET and ctype == socket.SCM_RIGHTS:
                arr = array.array("i")
                arr.frombytes(cmsg_data[: len(cmsg_data) - (len(cmsg_data) % fd_size)])
                fds.extend(arr.tolist())

        if len(fds) < 4:
            sock.close()
            raise RuntimeError(f"Expected 4 SCM_RIGHTS fds, got {len(fds)}")

        (
            magic,
            vmaj,
            vmin,
            bytes_len,
            num_axes,
            _reserved0,
            cycle_ns,
            topology_hash,
            cmd_ring_capacity,
            cmd_msg_bytes,
            status_ring_capacity,
            status_msg_bytes,
            build_id_hash,
            *_rest,
        ) = _WELCOME_STRUCT.unpack(data)

        if magic != _MAGIC_GIPC or vmaj != _VER_MAJOR or vmin != _VER_MINOR or bytes_len != _WELCOME_STRUCT.size:
            sock.close()
            raise RuntimeError("WELCOME validation failed (magic/ver/bytes)")

        self._sock = sock
        self._rt_num_axes = int(num_axes)
        self._cmd_shm_fd, self._status_shm_fd, self._cmd_eventfd, self._status_eventfd = fds[:4]

        # Map shared memory.
        self._cmd_shm = self._map_fd(self._cmd_shm_fd)
        self._status_shm = self._map_fd(self._status_shm_fd)

        self._cmd_hdr = self._parse_shm_header(self._cmd_shm)
        self._status_hdr = self._parse_shm_header(self._status_shm)

        # Basic sanity checks.
        if self._cmd_hdr.kind != 1 or self._status_hdr.kind != 2:
            raise RuntimeError("SHM kind mismatch (cmd/status)")
        if self._cmd_hdr.num_axes != self._status_hdr.num_axes:
            raise RuntimeError("SHM num_axes mismatch")

        self._connected = True
        self._initialized = True

        self._axis_to_joint = self._resolve_axis_to_joint_map(self._rt_num_axes, self._num_joints)

        print(
            "[EtherCAT RTCore] Connected:"
            f" num_axes={num_axes} cycle_ns={cycle_ns} topo_hash=0x{topology_hash:016x}"
            f" build=0x{build_id_hash:016x}"
            f" cmd_ring={cmd_ring_capacity}x{cmd_msg_bytes} status_ring={status_ring_capacity}x{status_msg_bytes}"
        )

        self._start_status_thread()

        feedback_timeout_s = self._env_float("GRADIENT_RTCORE_FEEDBACK_READY_TIMEOUT_S", 2.0)
        if feedback_timeout_s > 0.0:
            if self._wait_for_feedback_ready(timeout_s=feedback_timeout_s, require_live_wkc=False):
                print(
                    "[EtherCAT RTCore] Feedback ready:"
                    f" axis_config=1 snapshot=1 wkc={self._last_wkc_actual}/{self._last_wkc_expected}"
                    f" master_state={self._last_master_state}"
                )
            else:
                print(
                    "[EtherCAT RTCore] WARNING: Timed out waiting for RTCore status feedback"
                    f" ({self._feedback_ready_summary()})"
                )

        # Default safety policy: connect disarmed unless the environment opts back
        # into the legacy auto-arm behavior.
        if self._auto_arm:
            try:
                self._best_effort_safe_power_up()
            except Exception as e:
                print(f"[EtherCAT RTCore] WARNING: auto-arm failed: {e}")

        return True

    def _resolve_axis_to_joint_map(self, num_axes: int, num_joints: int) -> list[int]:
        """
        Determine how GradientOS joint vectors map to RTCore axes.

        Env override (1-based joint numbers):
          GRADIENT_RTCORE_CONTROL_JOINTS="3,4"
        """
        raw = os.environ.get("GRADIENT_RTCORE_CONTROL_JOINTS", "").strip()
        if raw:
            parts = [p.strip() for p in raw.split(",") if p.strip()]
            joints: list[int] = []
            for p in parts:
                try:
                    j1 = int(p)
                except ValueError:
                    continue
                joints.append(j1 - 1)  # convert to 0-based
            if len(joints) == num_axes and all(0 <= j < num_joints for j in joints):
                return joints
            print(
                f"[EtherCAT RTCore] WARNING: GRADIENT_RTCORE_CONTROL_JOINTS='{raw}' "
                f"does not match num_axes={num_axes}; using defaults."
            )

        # Default mapping policy: axis0..axisN maps to joint0..jointN in order.
        return list(range(min(num_axes, num_joints)))

    def _env_float(self, name: str, default: float) -> float:
        raw = os.environ.get(name, str(default)).strip()
        try:
            return float(raw)
        except Exception:
            return float(default)

    def _feedback_ready_summary(self) -> str:
        with self._status_lock:
            return (
                f"axis_config={1 if self._axis_config is not None else 0} "
                f"snapshot={1 if self._status_snapshot_event.is_set() else 0} "
                f"wkc={self._last_wkc_actual}/{self._last_wkc_expected} "
                f"master_state={self._last_master_state}"
            )

    def _wait_for_feedback_ready(self, timeout_s: float, *, require_live_wkc: bool) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() <= deadline:
            with self._status_lock:
                have_axis_cfg = self._axis_config is not None
                have_snapshot = self._status_snapshot_event.is_set()
                live_wkc = self._last_wkc_actual > 0
            if have_axis_cfg and have_snapshot and (live_wkc or not require_live_wkc):
                return True
            time.sleep(0.01)
        return False

    def _cmd_ring_offsets(self) -> tuple[int, int]:
        assert self._cmd_hdr is not None
        ring_hdr_off = self._cmd_hdr.ring_offset
        ring_entries_off = ring_hdr_off + _align_up(_RING_HEADER_STRUCT.size, 8)
        return ring_hdr_off, ring_entries_off

    def _cmd_ring_write(self, msg_type: int, payload: bytes) -> None:
        if self._cmd_shm is None or self._cmd_hdr is None:
            raise RuntimeError("cmd_shm not mapped")

        ring_hdr_off, ring_entries_off = self._cmd_ring_offsets()
        hdr_bytes = self._cmd_shm[ring_hdr_off : ring_hdr_off + _RING_HEADER_STRUCT.size]
        magic, capacity, msg_bytes, write_idx, read_idx, dropped, reserved0 = _RING_HEADER_STRUCT.unpack(hdr_bytes)
        if magic != _MAGIC_RING:
            raise RuntimeError("cmd ring header magic mismatch")
        if capacity == 0 or msg_bytes == 0:
            raise RuntimeError("cmd ring has invalid sizing")

        if (write_idx - read_idx) >= capacity:
            # Producer increments dropped on overflow.
            dropped += 1
            self._cmd_shm[ring_hdr_off + 20 : ring_hdr_off + 24] = struct.pack("<I", dropped)
            raise RuntimeError("cmd ring overflow")

        slot = write_idx % capacity
        off = ring_entries_off + (slot * msg_bytes)

        # Construct message: header + payload + zero padding to msg_bytes.
        time_ns = _now_monotonic_ns()
        header = _MSG_HEADER_STRUCT.pack(
            int(msg_type) & 0xFFFF,
            0,
            _MSG_HEADER_STRUCT.size + len(payload),
            int(self._cmd_seq),
            int(time_ns),
        )
        self._cmd_seq += 1

        blob = header + payload
        if len(blob) > msg_bytes:
            raise RuntimeError("cmd payload exceeds ring slot size")
        blob = blob.ljust(msg_bytes, b"\x00")
        self._cmd_shm[off : off + msg_bytes] = blob

        # Publish new write_idx (producer-owned).
        write_idx += 1
        self._cmd_shm[ring_hdr_off + 12 : ring_hdr_off + 16] = struct.pack("<I", write_idx)

        # Wake RTCore helper thread.
        if self._cmd_eventfd is not None:
            try:
                os.write(self._cmd_eventfd, struct.pack("<Q", 1))
            except Exception:
                pass

    def _send_cmd_arm(self, arm: bool) -> None:
        self._cmd_ring_write(_MSG_CMD_ARM, _CMD_ARM_STRUCT.pack(1 if arm else 0, 0))

    def _send_cmd_axis_enable(self, axis_mask: int) -> None:
        self._cmd_ring_write(_MSG_CMD_AXIS_ENABLE, _CMD_AXIS_MASK_STRUCT.pack(int(axis_mask), 0))

    def _send_cmd_axis_disable(self, axis_mask: int) -> None:
        self._cmd_ring_write(_MSG_CMD_AXIS_DISABLE, _CMD_AXIS_MASK_STRUCT.pack(int(axis_mask), 0))

    def _send_cmd_fault_reset(self, axis_mask: int) -> None:
        self._cmd_ring_write(_MSG_CMD_FAULT_RESET, _CMD_AXIS_MASK_STRUCT.pack(int(axis_mask), 0))

    def _send_cmd_set_mode(self, axis_mask: int, mode: int) -> None:
        self._cmd_ring_write(_MSG_CMD_SET_MODE, _CMD_SET_MODE_STRUCT.pack(int(axis_mask), int(mode)))

    def _map_fd(self, fd: int) -> mmap.mmap:
        st = os.fstat(fd)
        if st.st_size <= 0:
            raise RuntimeError("Shared memory fd has zero size")
        return mmap.mmap(fd, st.st_size, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)

    def _parse_shm_header(self, mm: mmap.mmap) -> _ShmHeader:
        data = mm[: _SHM_HEADER_STRUCT.size]
        (
            magic,
            vmaj,
            vmin,
            bytes_len,
            kind,
            num_axes,
            _reserved0,
            cycle_ns,
            topology_hash,
            ring_offset,
            ring_capacity,
            ring_msg_bytes,
            setpoint_offset,
            _reserved1,
            *_rest,
        ) = _SHM_HEADER_STRUCT.unpack(data)

        if magic != _MAGIC_GSHM or vmaj != _VER_MAJOR or vmin != _VER_MINOR or bytes_len != _SHM_HEADER_STRUCT.size:
            raise RuntimeError("SHM header validation failed (magic/ver/bytes)")

        return _ShmHeader(
            kind=int(kind),
            num_axes=int(num_axes),
            cycle_ns=int(cycle_ns),
            topology_hash=int(topology_hash),
            ring_offset=int(ring_offset),
            ring_capacity=int(ring_capacity),
            ring_msg_bytes=int(ring_msg_bytes),
            setpoint_offset=int(setpoint_offset),
        )

    def _parse_axis_config(self, payload: bytes) -> _AxisConfig:
        (
            num_axes,
            _reserved0,
            *rest,
        ) = _AXIS_CONFIG_STRUCT.unpack_from(payload, 0)
        idx = 0
        idx += _GRADIENT_MAX_AXES  # counts_per_rev
        idx += _GRADIENT_MAX_AXES  # gear_ratio
        sign = [int(x) for x in rest[idx : idx + _GRADIENT_MAX_AXES]]
        idx += _GRADIENT_MAX_AXES
        idx += _GRADIENT_MAX_AXES  # axis_type
        counts_per_unit = [float(x) for x in rest[idx : idx + _GRADIENT_MAX_AXES]]
        return _AxisConfig(
            num_axes=int(num_axes),
            counts_per_unit=counts_per_unit,
            sign=sign,
        )

    def _parse_status_hello(self, payload: bytes) -> tuple[Optional[str], int, int]:
        (
            _build_id_hash,
            _topology_hash,
            _cycle_ns,
            _num_axes,
            drive_profile_code,
            wkc_expected,
            _reserved0,
        ) = _STATUS_HELLO_STRUCT.unpack_from(payload, 0)
        return (
            rtcore_drive_profile_id_to_name(drive_profile_code),
            int(drive_profile_code),
            int(wkc_expected),
        )

    def _build_axis_config_from_robot_config(self, robot_config: dict) -> Optional[_AxisConfig]:
        counts_per_radian = list(robot_config.get("actuator_counts_per_radian", []))
        if not counts_per_radian:
            counts_per_rev = list(robot_config.get("actuator_encoder_counts_per_rev", []))
            gear_ratios = list(robot_config.get("actuator_gear_ratios", []))
            counts_per_radian = []
            for idx in range(max(len(counts_per_rev), len(gear_ratios))):
                cpr = int(counts_per_rev[idx]) if idx < len(counts_per_rev) else 0
                ratio = float(gear_ratios[idx]) if idx < len(gear_ratios) else 1.0
                if cpr <= 0 or ratio <= 0.0:
                    counts_per_radian.append(0.0)
                else:
                    counts_per_radian.append((float(cpr) * ratio) / (2.0 * 3.141592653589793))

        if not counts_per_radian:
            return None

        signs_raw = list(robot_config.get("actuator_position_signs", []))
        if not signs_raw:
            inverted_ids = set(robot_config.get("inverted_actuator_ids", set()))
            actuator_ids = list(robot_config.get("actuator_ids", []))
            signs_raw = [-1 if actuator_id in inverted_ids else 1 for actuator_id in actuator_ids]

        num_axes = min(
            int(robot_config.get("num_physical_actuators", len(counts_per_radian))),
            len(counts_per_radian),
        )
        if num_axes <= 0:
            return None

        counts_per_unit = [0.0] * _GRADIENT_MAX_AXES
        signs = [0] * _GRADIENT_MAX_AXES
        for idx in range(min(num_axes, _GRADIENT_MAX_AXES)):
            counts_per_unit[idx] = float(counts_per_radian[idx])
            raw_sign = int(signs_raw[idx]) if idx < len(signs_raw) else 1
            signs[idx] = 1 if raw_sign >= 0 else -1

        return _AxisConfig(
            num_axes=min(num_axes, _GRADIENT_MAX_AXES),
            counts_per_unit=counts_per_unit,
            sign=signs,
        )

    def _maybe_warn_runtime_axis_config_mismatch(self, runtime_cfg: _AxisConfig) -> None:
        if self._robot_axis_config is None or self._axis_config_mismatch_logged:
            return
        robot_cfg = self._robot_axis_config
        compare_axes = min(robot_cfg.num_axes, runtime_cfg.num_axes, _GRADIENT_MAX_AXES)
        for axis_i in range(compare_axes):
            robot_cpu = float(robot_cfg.counts_per_unit[axis_i])
            runtime_cpu = float(runtime_cfg.counts_per_unit[axis_i])
            robot_sign = int(robot_cfg.sign[axis_i])
            runtime_sign = int(runtime_cfg.sign[axis_i])
            cpu_mismatch = abs(robot_cpu - runtime_cpu) > max(1.0, abs(robot_cpu) * 1e-3)
            sign_mismatch = robot_sign != runtime_sign
            if cpu_mismatch or sign_mismatch:
                print(
                    "[EtherCAT RTCore] WARNING: RTCore axis scaling differs from robot config; "
                    f"keeping robot-config scaling for Python feedback conversion "
                    f"(axis={axis_i} robot_cpu={robot_cpu:.6f} runtime_cpu={runtime_cpu:.6f} "
                    f"robot_sign={robot_sign} runtime_sign={runtime_sign})"
                )
                self._axis_config_mismatch_logged = True
                return

    def _master_offset_for_joint(self, logical_joint_idx: int) -> float:
        if 0 <= logical_joint_idx < len(self._master_offsets_rad):
            return float(self._master_offsets_rad[logical_joint_idx])
        return 0.0

    def _axis_q_from_counts(self, axis_i: int, raw_counts: int) -> Optional[float]:
        cfg = self._axis_config
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes):
            return None
        sign = int(cfg.sign[axis_i])
        counts_per_unit = float(cfg.counts_per_unit[axis_i])
        if sign not in (-1, 1) or counts_per_unit <= 0.0:
            return None
        return float(raw_counts) / (float(sign) * counts_per_unit)

    def _start_status_thread(self) -> None:
        if self._status_thread and self._status_thread.is_alive():
            return

        self._status_stop.clear()
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self._status_thread.start()

    def _status_loop(self) -> None:
        if self._status_eventfd is None or self._status_shm is None or self._status_hdr is None:
            return

        # Pre-compute ring offsets.
        ring_hdr_offset = self._status_hdr.ring_offset
        ring_entries_offset = ring_hdr_offset + _align_up(_RING_HEADER_STRUCT.size, 8)

        while not self._status_stop.is_set():
            # Wait for status_eventfd (or poll at low rate).
            r, _w, _x = select.select([self._status_eventfd], [], [], 0.25)
            if r:
                try:
                    os.read(self._status_eventfd, 8)  # drain counter
                except Exception:
                    pass

            try:
                self._drain_status_ring(ring_hdr_offset, ring_entries_offset)
            except Exception:
                # Keep best-effort reader alive.
                pass

    def _drain_status_ring(self, ring_hdr_offset: int, ring_entries_offset: int) -> None:
        assert self._status_shm is not None
        assert self._status_hdr is not None

        hdr_bytes = self._status_shm[ring_hdr_offset : ring_hdr_offset + _RING_HEADER_STRUCT.size]
        magic, capacity, msg_bytes, write_idx, read_idx, _dropped, _reserved0 = _RING_HEADER_STRUCT.unpack(hdr_bytes)
        if magic != _MAGIC_RING:
            return
        if capacity == 0 or msg_bytes == 0:
            return

        # Consume available entries.
        while read_idx < write_idx:
            slot = read_idx % capacity
            off = ring_entries_offset + (slot * msg_bytes)
            blob = self._status_shm[off : off + msg_bytes]

            mtype, _mflags, mbytes, _seq, _t_ns = _MSG_HEADER_STRUCT.unpack_from(blob, 0)
            payload = blob[_MSG_HEADER_STRUCT.size : min(msg_bytes, mbytes)]

            if mtype == _MSG_STATUS_HELLO and len(payload) >= _STATUS_HELLO_STRUCT.size:
                try:
                    live_profile_id, drive_profile_code, wkc_expected = self._parse_status_hello(payload)
                    with self._status_lock:
                        self._rt_drive_profile_code = int(drive_profile_code)
                        self._rt_drive_profile_id = live_profile_id
                        self._last_wkc_expected = int(wkc_expected)
                except Exception:
                    pass

            if mtype == _MSG_STATUS_AXIS_CONFIG and len(payload) >= _AXIS_CONFIG_STRUCT.size:
                try:
                    parsed_cfg = self._parse_axis_config(payload)
                    self._runtime_axis_config = parsed_cfg
                    if self._robot_axis_config is None:
                        self._axis_config = parsed_cfg
                    else:
                        self._maybe_warn_runtime_axis_config_mismatch(parsed_cfg)
                    self._axis_config_event.set()
                except Exception:
                    pass

            if mtype == _MSG_STATUS_SNAPSHOT and len(payload) >= _STATUS_SNAPSHOT_HEADER_STRUCT.size:
                # Snapshot layout:
                #   0..39: header
                #   40.. : axes[16] where each axis is 28 bytes; pos_counts at offset 0 in each axis
                (
                    _num_axes,
                    wkc_expected,
                    wkc_actual,
                    master_state,
                    _dc_offset_ns,
                    _cycle_jitter_ns,
                    _topology_hash,
                ) = _STATUS_SNAPSHOT_HEADER_STRUCT.unpack_from(payload, 0)
                with self._status_lock:
                    self._last_wkc_expected = int(wkc_expected)
                    self._last_wkc_actual = int(wkc_actual)
                    self._last_master_state = int(master_state)
                for axis_i in range(min(self._rt_num_axes, _GRADIENT_MAX_AXES)):
                    axis_off = 40 + axis_i * 28
                    if axis_off + 10 <= len(payload):
                        pos_counts, _torque_raw, statusword, error_code = struct.unpack_from(
                            "<ihHH",
                            payload,
                            axis_off,
                        )
                        self._axis_counts[axis_i] = int(pos_counts)
                        self._axis_statusword[axis_i] = int(statusword)
                        self._axis_error_code[axis_i] = int(error_code)
                self._status_snapshot_event.set()

            read_idx += 1

        # Publish new read_idx (consumer-owned).
        # RingHeader layout (u32):
        #   magic(0), capacity(4), msg_bytes(8), write_idx(12), read_idx(16), dropped(20), reserved0(24)
        self._status_shm[ring_hdr_offset + 16 : ring_hdr_offset + 20] = struct.pack("<I", read_idx)

    def _write_setpoint(self, positions_rad: list[float], axis_mask: int) -> None:
        assert self._cmd_shm is not None
        assert self._cmd_hdr is not None

        if self._cmd_hdr.setpoint_offset == 0:
            raise RuntimeError("RTCore did not provide a setpoint slot")

        # SetpointSlotV1:
        #   uint64 seq
        #   uint64 target_time_ns
        #   double q[16]
        #   uint32 axis_mask
        #   uint32 reserved
        slot_off = self._cmd_hdr.setpoint_offset

        target_time_ns = _now_monotonic_ns() + int(self._cmd_hdr.cycle_ns)

        # Write payload first (excluding seq).
        self._cmd_shm[slot_off + 8 : slot_off + 16] = struct.pack("<Q", target_time_ns)

        # q array (16 doubles).
        q = [0.0] * _GRADIENT_MAX_AXES
        for i, v in enumerate(positions_rad[: _GRADIENT_MAX_AXES]):
            q[i] = float(v)
        self._cmd_shm[slot_off + 16 : slot_off + 16 + 16 * 8] = struct.pack("<16d", *q)

        self._cmd_shm[slot_off + 16 + 16 * 8 : slot_off + 16 + 16 * 8 + 4] = struct.pack("<I", int(axis_mask))
        self._cmd_shm[slot_off + 16 + 16 * 8 + 4 : slot_off + 16 + 16 * 8 + 8] = struct.pack("<I", 0)

        # Publish seq last.
        self._setpoint_seq += 1
        self._cmd_shm[slot_off : slot_off + 8] = struct.pack("<Q", self._setpoint_seq)

        # Wake RTCore helper thread.
        if self._cmd_eventfd is not None:
            try:
                os.write(self._cmd_eventfd, struct.pack("<Q", 1))
            except Exception:
                pass

