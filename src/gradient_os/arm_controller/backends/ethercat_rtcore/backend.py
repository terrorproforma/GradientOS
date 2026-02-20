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

_MSG_STATUS_SNAPSHOT = 0x0202

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

        self._num_joints = int(robot_config.get("num_logical_joints", 6))

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
        self._auto_arm = os.environ.get("GRADIENT_RTCORE_AUTO_ARM", "1").lower() not in ("0", "false", "no")
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

        # Latest known axis counts from STATUS_SNAPSHOT (pos_counts per axis).
        self._axis_counts: list[int] = [0] * _GRADIENT_MAX_AXES

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
        self._status_stop.set()
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
                axis_q[axis_i] = float(positions_rad[joint_i])

        axis_mask = (1 << self._rt_num_axes) - 1
        self._write_setpoint(axis_q, axis_mask=axis_mask)

    def get_joint_positions(self, verbose: bool = False) -> list[float]:
        # We cannot convert counts->rad until axis scaling config is implemented.
        # For now we return the last commanded setpoint (safe, deterministic).
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
        # Until scaling is implemented, return current joint setpoint.
        return list(self._last_joint_setpoint_rad)

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
        # Not supported (calibration is drive/vendor specific and belongs in RTCore commissioning).
        return False

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

        # Mirror serial backend behavior: after init, the system is usable.
        if self._auto_arm:
            try:
                axis_mask_all = (1 << self._rt_num_axes) - 1 if self._rt_num_axes > 0 else 0
                axis_mask = axis_mask_all
                if self._auto_arm_mask is not None:
                    axis_mask = int(self._auto_arm_mask) & int(axis_mask_all)
                self._send_cmd_arm(True)
                self._send_cmd_set_mode(axis_mask=axis_mask, mode=_MODE_CSP)
                self._send_cmd_axis_enable(axis_mask=axis_mask)
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

            if mtype == _MSG_STATUS_SNAPSHOT and len(payload) >= 40:
                # Snapshot layout:
                #   0..39: header
                #   40.. : axes[16] where each axis is 28 bytes; pos_counts at offset 0 in each axis
                for axis_i in range(min(self._rt_num_axes, _GRADIENT_MAX_AXES)):
                    axis_off = 40 + axis_i * 28
                    if axis_off + 4 <= len(payload):
                        (pos_counts,) = struct.unpack_from("<i", payload, axis_off)
                        self._axis_counts[axis_i] = int(pos_counts)

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

