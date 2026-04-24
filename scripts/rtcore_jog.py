#!/usr/bin/env python3
"""
Minimal RTCore jog/bring-up CLI (no GradientOS controller required).

Talks directly to the RTCore IPC socket (`/run/gradient-rt-motion/ipc.sock`) using
the ABI in `src/gradient_rt_motion/ipc_v1.hpp`.

Typical use:
  # In one terminal (root):
  sudo /usr/local/bin/gradient-rt-motion --num-axes 2 --max-rpm 100

  # In another terminal (pi):
  ./scripts/rtcore_jog.py status
  ./scripts/rtcore_jog.py arm --enable-mask 0x1
  ./scripts/rtcore_jog.py jog --axis 0 --delta-rad 0.01
"""

from __future__ import annotations

import argparse
import array
import json
import mmap
import os
import re
import select
import shlex
import socket
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional


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
# Keep in lockstep with `src/gradient_rt_motion/ipc_v1.hpp::kVerMinor` and
# `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py::_VER_MINOR`.
# Bumped 2026-04-20 alongside the StatusExtendedSnapshotV1 addition; the RTCore
# server rejects the HELLO + closes the socket when this drifts behind, which
# surfaces as `WELCOME size mismatch (got 0 bytes)` to CLI callers and breaks
# the `start-stack.sh` fault-reset preflight.
_VER_MINOR = 1
_ROLE_CONTROLLER = 1

_MAX_AXES = 16

# Msg types (v1)
_MSG_CMD_ARM = 0x0101
_MSG_CMD_AXIS_ENABLE = 0x0102
_MSG_CMD_AXIS_DISABLE = 0x0103
_MSG_CMD_FAULT_RESET = 0x0104
_MSG_CMD_SET_MODE = 0x0106
_MSG_CMD_TRAJECTORY_BEGIN = 0x0120
_MSG_CMD_TRAJECTORY_POINT = 0x0121
_MSG_CMD_TRAJECTORY_COMMIT = 0x0122

_MSG_STATUS_SNAPSHOT = 0x0202
_MSG_STATUS_AXIS_CONFIG = 0x0203

_MODE_CSP = 8
_TRAJ_POINTF_LAST_POINT = 1 << 1

# ABI structs
_HELLO_STRUCT = struct.Struct("<IHHIIQ4Q")  # 56 bytes
_WELCOME_STRUCT = struct.Struct("<IHHI II 4x QQ IIII Q 4Q")  # 96 bytes

_SHM_HEADER_STRUCT = struct.Struct("<IHHIIIIQQIIIII4x8Q")  # 128 bytes
_RING_HEADER_STRUCT = struct.Struct("<7I")  # 28 bytes
_MSG_HEADER_STRUCT = struct.Struct("<HHIQQ")  # 24 bytes

_CMD_ARM_STRUCT = struct.Struct("<II")
_CMD_AXIS_MASK_STRUCT = struct.Struct("<II")
_CMD_FAULT_RESET_STRUCT = struct.Struct("<II")
_CMD_SET_MODE_STRUCT = struct.Struct("<II")
_CMD_TRAJECTORY_BEGIN_STRUCT = struct.Struct("<QIIII")
_TRAJECTORY_POINT_STRUCT = struct.Struct("<QIIQ16d16dII")
_CMD_TRAJECTORY_CONTROL_STRUCT = struct.Struct("<QII")

# StatusSnapshotV1: 40-byte header + 16*AxisStatusV1 (28 bytes each) = 488 bytes.
_AXIS_STATUS_STRUCT = struct.Struct("<i h H H B B H 2x I I I")  # 28 bytes
_SNAP_HEADER_STRUCT = struct.Struct("<IIIIqqQ")  # 40 bytes

# StatusAxisConfigV1: see src/gradient_rt_motion/ipc_v1.hpp
_AXIS_CONFIG_STRUCT = struct.Struct("<II16I16d16i16B16x16d")  # 424 bytes


def _align_up(value: int, alignment: int) -> int:
    return ((value + alignment - 1) // alignment) * alignment


def _now_ns() -> int:
    return time.monotonic_ns()


@dataclass(frozen=True)
class _Welcome:
    num_axes: int
    cycle_ns: int
    topology_hash: int
    build_id_hash: int


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
class AxisStatus:
    pos_counts: int
    torque_raw: int
    statusword: int
    error_code: int
    mode_display: int
    ds402_state: int
    di_bits: int
    axis_fault_flags: int
    brake_state: int


@dataclass(frozen=True)
class StatusSnapshot:
    num_axes: int
    wkc_expected: int
    wkc_actual: int
    master_state: int
    dc_offset_ns: int
    cycle_jitter_ns: int
    topology_hash: int
    axes: list[AxisStatus]


@dataclass
class _ConsoleDiagState:
    sample_count: int = 0
    prev_overrun: Optional[int] = None
    prev_lost_frames: Optional[int] = None
    last_lost_frames: Optional[int] = None


@dataclass(frozen=True)
class AxisConfig:
    num_axes: int
    counts_per_rev: list[int]
    gear_ratio: list[float]
    sign: list[int]
    axis_type: list[int]
    counts_per_unit: list[float]


class RTCoreClient:
    def __init__(self, socket_path: str) -> None:
        self._socket_path = socket_path
        self._sock: Optional[socket.socket] = None

        self._cmd_shm: Optional[mmap.mmap] = None
        self._status_shm: Optional[mmap.mmap] = None
        self._cmd_hdr: Optional[_ShmHeader] = None
        self._status_hdr: Optional[_ShmHeader] = None

        self._cmd_eventfd: Optional[int] = None
        self._status_eventfd: Optional[int] = None

        self._cmd_seq = 1
        self._next_traj_id = 1

        self.welcome: Optional[_Welcome] = None
        self.axis_config: Optional[AxisConfig] = None

    def close(self) -> None:
        for mm_name in ("_cmd_shm", "_status_shm"):
            mm = getattr(self, mm_name)
            if mm is not None:
                try:
                    mm.close()
                except Exception:
                    pass
            setattr(self, mm_name, None)

        for fd_name in ("_cmd_eventfd", "_status_eventfd"):
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
        self.axis_config = None

    def __enter__(self) -> "RTCoreClient":
        self.connect()
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.close()

    def connect(self) -> None:
        if self._sock is not None:
            return

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        sock.connect(self._socket_path)

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

        # Receive WELCOME + fds (cmd_shm, status_shm, cmd_eventfd, status_eventfd).
        fd_size = array.array("i").itemsize
        data, ancdata, _flags, _addr = sock.recvmsg(
            _WELCOME_STRUCT.size,
            socket.CMSG_SPACE(fd_size * 4),
        )
        if len(data) != _WELCOME_STRUCT.size:
            raise RuntimeError(f"WELCOME size mismatch (got {len(data)} bytes)")

        fds: list[int] = []
        for level, ctype, cmsg_data in ancdata:
            if level == socket.SOL_SOCKET and ctype == socket.SCM_RIGHTS:
                arr = array.array("i")
                arr.frombytes(cmsg_data[: len(cmsg_data) - (len(cmsg_data) % fd_size)])
                fds.extend(arr.tolist())
        if len(fds) < 4:
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
            _cmd_ring_capacity,
            _cmd_msg_bytes,
            _status_ring_capacity,
            _status_msg_bytes,
            build_id_hash,
            *_rest,
        ) = _WELCOME_STRUCT.unpack(data)
        if magic != _MAGIC_GIPC or vmaj != _VER_MAJOR or vmin != _VER_MINOR or bytes_len != _WELCOME_STRUCT.size:
            raise RuntimeError("WELCOME validation failed (magic/ver/bytes)")

        self.welcome = _Welcome(
            num_axes=int(num_axes),
            cycle_ns=int(cycle_ns),
            topology_hash=int(topology_hash),
            build_id_hash=int(build_id_hash),
        )

        self._sock = sock
        cmd_shm_fd, status_shm_fd, cmd_eventfd, status_eventfd = fds[:4]
        self._cmd_eventfd = int(cmd_eventfd)
        self._status_eventfd = int(status_eventfd)

        self._cmd_shm = self._map_fd(cmd_shm_fd)
        self._status_shm = self._map_fd(status_shm_fd)
        self._cmd_hdr = self._parse_shm_header(self._cmd_shm)
        self._status_hdr = self._parse_shm_header(self._status_shm)

        if self._cmd_hdr.kind != 1 or self._status_hdr.kind != 2:
            raise RuntimeError("SHM kind mismatch (cmd/status)")

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

    def _cmd_ring_offsets(self) -> tuple[int, int]:
        assert self._cmd_hdr is not None
        ring_hdr_off = self._cmd_hdr.ring_offset
        ring_entries_off = ring_hdr_off + _align_up(_RING_HEADER_STRUCT.size, 8)
        return ring_hdr_off, ring_entries_off

    def _status_ring_offsets(self) -> tuple[int, int]:
        assert self._status_hdr is not None
        ring_hdr_off = self._status_hdr.ring_offset
        ring_entries_off = ring_hdr_off + _align_up(_RING_HEADER_STRUCT.size, 8)
        return ring_hdr_off, ring_entries_off

    def _cmd_ring_write(self, msg_type: int, payload: bytes) -> None:
        assert self._cmd_shm is not None
        ring_hdr_off, ring_entries_off = self._cmd_ring_offsets()
        hdr_bytes = self._cmd_shm[ring_hdr_off : ring_hdr_off + _RING_HEADER_STRUCT.size]
        magic, capacity, msg_bytes, write_idx, read_idx, dropped, reserved0 = _RING_HEADER_STRUCT.unpack(hdr_bytes)
        if magic != _MAGIC_RING:
            raise RuntimeError("cmd ring header magic mismatch")
        if capacity == 0 or msg_bytes == 0:
            raise RuntimeError("cmd ring has invalid sizing")

        if (write_idx - read_idx) >= capacity:
            dropped += 1
            self._cmd_shm[ring_hdr_off + 20 : ring_hdr_off + 24] = struct.pack("<I", dropped)
            raise RuntimeError("cmd ring overflow")

        slot = write_idx % capacity
        off = ring_entries_off + (slot * msg_bytes)

        header = _MSG_HEADER_STRUCT.pack(
            int(msg_type) & 0xFFFF,
            0,
            _MSG_HEADER_STRUCT.size + len(payload),
            int(self._cmd_seq),
            int(_now_ns()),
        )
        self._cmd_seq += 1

        blob = header + payload
        if len(blob) > msg_bytes:
            raise RuntimeError("cmd payload exceeds ring slot size")
        blob = blob.ljust(msg_bytes, b"\x00")
        self._cmd_shm[off : off + msg_bytes] = blob

        # Publish write_idx.
        write_idx += 1
        self._cmd_shm[ring_hdr_off + 12 : ring_hdr_off + 16] = struct.pack("<I", write_idx)

        # Wake helper thread.
        if self._cmd_eventfd is not None:
            os.write(self._cmd_eventfd, struct.pack("<Q", 1))

    def arm(self, enable: bool, enable_mask: Optional[int] = None) -> None:
        # Arm/disarm the master, optionally setting an initial axis enable mask.
        self._cmd_ring_write(_MSG_CMD_ARM, _CMD_ARM_STRUCT.pack(1 if enable else 0, 0))
        if enable and enable_mask is not None:
            self.set_enable_mask(enable_mask)
            # RTCore currently hardcodes CSP mode to 8 when enabled; keep this for future-proofing.
            self._cmd_ring_write(_MSG_CMD_SET_MODE, _CMD_SET_MODE_STRUCT.pack(int(enable_mask), int(_MODE_CSP)))

    def set_enable_mask(self, axis_mask: int) -> None:
        self._cmd_ring_write(_MSG_CMD_AXIS_ENABLE, _CMD_AXIS_MASK_STRUCT.pack(int(axis_mask), 0))

    def disable_mask(self, axis_mask: int) -> None:
        self._cmd_ring_write(_MSG_CMD_AXIS_DISABLE, _CMD_AXIS_MASK_STRUCT.pack(int(axis_mask), 0))

    def fault_reset(self, axis_mask: int = 0) -> None:
        # Request a DS402 fault reset pulse. axis_mask=0 means "all axes".
        self._cmd_ring_write(_MSG_CMD_FAULT_RESET, _CMD_FAULT_RESET_STRUCT.pack(int(axis_mask), 0))

    def write_setpoint(self, q: list[float], axis_mask: int) -> None:
        traj_id = int(self._next_traj_id)
        self._next_traj_id += 1

        qq = [0.0] * _MAX_AXES
        for i, v in enumerate(q[:_MAX_AXES]):
            qq[i] = float(v)
        qd = [0.0] * _MAX_AXES

        self._cmd_ring_write(
            _MSG_CMD_TRAJECTORY_BEGIN,
            _CMD_TRAJECTORY_BEGIN_STRUCT.pack(int(traj_id), int(axis_mask), 0, 1, 0),
        )
        self._cmd_ring_write(
            _MSG_CMD_TRAJECTORY_POINT,
            _TRAJECTORY_POINT_STRUCT.pack(
                int(traj_id),
                0,
                _TRAJ_POINTF_LAST_POINT,
                0,
                *qq,
                *qd,
                int(axis_mask),
                0,
            ),
        )
        self._cmd_ring_write(
            _MSG_CMD_TRAJECTORY_COMMIT,
            _CMD_TRAJECTORY_CONTROL_STRUCT.pack(int(traj_id), 0, 0),
        )

    def read_status_snapshot(self, timeout_s: float = 0.5) -> Optional[StatusSnapshot]:
        if self._status_eventfd is None or self._status_shm is None or self._status_hdr is None:
            return None

        # Wait for status_eventfd or timeout.
        r, _w, _x = select.select([self._status_eventfd], [], [], float(timeout_s))
        if r:
            try:
                os.read(self._status_eventfd, 8)
            except Exception:
                pass

        ring_hdr_off, ring_entries_off = self._status_ring_offsets()
        hdr_bytes = self._status_shm[ring_hdr_off : ring_hdr_off + _RING_HEADER_STRUCT.size]
        magic, capacity, msg_bytes, write_idx, read_idx, _dropped, _reserved0 = _RING_HEADER_STRUCT.unpack(hdr_bytes)
        if magic != _MAGIC_RING or capacity == 0 or msg_bytes == 0:
            return None

        latest: Optional[StatusSnapshot] = None

        while read_idx < write_idx:
            slot = read_idx % capacity
            off = ring_entries_off + (slot * msg_bytes)
            blob = self._status_shm[off : off + msg_bytes]

            mtype, _mflags, mbytes, _seq, _t_ns = _MSG_HEADER_STRUCT.unpack_from(blob, 0)
            payload = blob[_MSG_HEADER_STRUCT.size : min(msg_bytes, mbytes)]

            if mtype == _MSG_STATUS_AXIS_CONFIG and len(payload) >= _AXIS_CONFIG_STRUCT.size:
                try:
                    self.axis_config = _parse_axis_config(payload)
                except Exception:
                    # Best-effort; ignore malformed config.
                    pass

            if mtype == _MSG_STATUS_SNAPSHOT and len(payload) >= (_SNAP_HEADER_STRUCT.size + _MAX_AXES * _AXIS_STATUS_STRUCT.size):
                latest = _parse_snapshot(payload)

            read_idx += 1

        # Publish new read_idx (consumer-owned).
        self._status_shm[ring_hdr_off + 16 : ring_hdr_off + 20] = struct.pack("<I", read_idx)
        return latest


def _parse_snapshot(payload: bytes) -> StatusSnapshot:
    (
        num_axes,
        wkc_expected,
        wkc_actual,
        master_state,
        dc_offset_ns,
        cycle_jitter_ns,
        topo_hash,
    ) = _SNAP_HEADER_STRUCT.unpack_from(payload, 0)

    axes: list[AxisStatus] = []
    base = _SNAP_HEADER_STRUCT.size
    for i in range(_MAX_AXES):
        off = base + i * _AXIS_STATUS_STRUCT.size
        (
            pos_counts,
            torque_raw,
            statusword,
            error_code,
            mode_display,
            ds402_state,
            _reserved0,
            di_bits,
            axis_fault_flags,
            brake_state,
        ) = _AXIS_STATUS_STRUCT.unpack_from(payload, off)
        axes.append(
            AxisStatus(
                pos_counts=int(pos_counts),
                torque_raw=int(torque_raw),
                statusword=int(statusword),
                error_code=int(error_code),
                mode_display=int(mode_display),
                ds402_state=int(ds402_state),
                di_bits=int(di_bits),
                axis_fault_flags=int(axis_fault_flags),
                brake_state=int(brake_state),
            )
        )

    return StatusSnapshot(
        num_axes=int(num_axes),
        wkc_expected=int(wkc_expected),
        wkc_actual=int(wkc_actual),
        master_state=int(master_state),
        dc_offset_ns=int(dc_offset_ns),
        cycle_jitter_ns=int(cycle_jitter_ns),
        topology_hash=int(topo_hash),
        axes=axes,
    )


_AXIS_TYPE_ROTARY = 1
_AXIS_TYPE_LINEAR = 2


def _parse_axis_config(payload: bytes) -> AxisConfig:
    (
        num_axes,
        _reserved0,
        *rest,
    ) = _AXIS_CONFIG_STRUCT.unpack_from(payload, 0)
    idx = 0
    counts_per_rev = [int(x) for x in rest[idx : idx + _MAX_AXES]]
    idx += _MAX_AXES
    gear_ratio = [float(x) for x in rest[idx : idx + _MAX_AXES]]
    idx += _MAX_AXES
    sign = [int(x) for x in rest[idx : idx + _MAX_AXES]]
    idx += _MAX_AXES
    axis_type = [int(x) for x in rest[idx : idx + _MAX_AXES]]
    idx += _MAX_AXES
    counts_per_unit = [float(x) for x in rest[idx : idx + _MAX_AXES]]
    return AxisConfig(
        num_axes=int(num_axes),
        counts_per_rev=counts_per_rev,
        gear_ratio=gear_ratio,
        sign=sign,
        axis_type=axis_type,
        counts_per_unit=counts_per_unit,
    )


def _axis_unit_name(axis_type: int) -> str:
    if int(axis_type) == _AXIS_TYPE_LINEAR:
        return "m"
    return "rad"


def _get_axis_scale(
    client: RTCoreClient,
    axis: int,
    *,
    fallback_counts_per_unit: float,
    fallback_sign: int,
) -> tuple[int, float, int]:
    cfg = client.axis_config
    if cfg is not None and 0 <= int(axis) < _MAX_AXES:
        try:
            sgn = int(cfg.sign[int(axis)])
            cpu = float(cfg.counts_per_unit[int(axis)])
            at = int(cfg.axis_type[int(axis)])
            if sgn in (-1, 1) and cpu > 0.0:
                return sgn, cpu, at
        except Exception:
            pass
    return int(fallback_sign), float(fallback_counts_per_unit), _AXIS_TYPE_ROTARY


_A6EC_CODEBOOK_CACHE: dict | None = None


def _load_a6ec_codebook() -> dict | None:
    """
    Load the extracted A6-EC manual codebook (best-effort).

    Files are generated from the PDF by:
      scripts/extract_a6ec_manual_codes.py
    """
    global _A6EC_CODEBOOK_CACHE
    if _A6EC_CODEBOOK_CACHE is not None:
        return _A6EC_CODEBOOK_CACHE

    # Default location in this repo.
    codebook_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "docs", "resources", "a6ec_manual_codes.json")
    )

    try:
        with open(codebook_path, "r", encoding="utf-8") as f:
            _A6EC_CODEBOOK_CACHE = json.load(f)
            return _A6EC_CODEBOOK_CACHE
    except Exception:
        # Missing file or parse failure; keep best-effort behavior.
        _A6EC_CODEBOOK_CACHE = {}
        return _A6EC_CODEBOOK_CACHE


def _bus_fault_name_603f(error_code_u16: int) -> Optional[str]:
    if int(error_code_u16) == 0:
        return None
    book = _load_a6ec_codebook() or {}
    try:
        bus = book.get("tables", {}).get("bus_fault_codes", {})
        key = f"0X{int(error_code_u16) & 0xFFFF:04X}"
        ent = bus.get(key)
        if isinstance(ent, dict):
            name = ent.get("name")
            if isinstance(name, str) and name.strip():
                return name.strip()
    except Exception:
        return None
    return None


def _counts_per_rad(counts_per_rev: int, gear_ratio: float) -> float:
    return (float(counts_per_rev) * float(gear_ratio)) / (2.0 * 3.141592653589793)


def _print_status(s: StatusSnapshot) -> None:
    print(
        f"RTCore status: num_axes={s.num_axes} wkc={s.wkc_actual}/{s.wkc_expected} "
        f"master_state={s.master_state} topo=0x{s.topology_hash:016x}"
    )
    for i in range(s.num_axes):
        a = s.axes[i]
        bus_name = _bus_fault_name_603f(a.error_code)
        err_suffix = f" ({bus_name})" if bus_name else ""
        print(
            "  "
            f"axis{i}: pos_counts={a.pos_counts} "
            f"sw=0x{a.statusword:04x} ds402={a.ds402_state} "
            f"err=0x{a.error_code:04x}{err_suffix} mode_disp={a.mode_display} "
            f"torque_raw={a.torque_raw} di=0x{a.di_bits:08x}"
        )


def _format_status_one_line(s: StatusSnapshot) -> str:
    parts = [
        f"wkc={s.wkc_actual}/{s.wkc_expected} master_state={s.master_state} topo=0x{s.topology_hash:016x}"
    ]
    for i in range(s.num_axes):
        a = s.axes[i]
        parts.append(f"a{i}:pos={a.pos_counts} sw=0x{a.statusword:04x} ds={a.ds402_state} err=0x{a.error_code:04x}")
    return " | ".join(parts)


def _try_int(value: object) -> Optional[int]:
    try:
        return int(value)  # type: ignore[arg-type]
    except Exception:
        return None


def _try_float(value: object) -> Optional[float]:
    try:
        return float(value)  # type: ignore[arg-type]
    except Exception:
        return None


def _load_metrics(path: str) -> dict:
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            return data
    except Exception:
        pass
    return {}


def _read_ethercat_lost_frames(cmd: str, timeout_s: float = 1.5) -> Optional[int]:
    try:
        proc = subprocess.run(
            shlex.split(cmd),
            capture_output=True,
            text=True,
            check=False,
            timeout=float(timeout_s),
        )
    except Exception:
        return None
    if int(proc.returncode) != 0:
        return None
    m = re.search(r"^\s*Lost frames:\s*(\d+)\s*$", proc.stdout, flags=re.MULTILINE)
    if not m:
        return None
    return _try_int(m.group(1))


def _fmt_delta(current: Optional[int], previous: Optional[int]) -> tuple[str, Optional[int]]:
    if current is None or previous is None:
        return "(n/a)", None
    delta = int(current) - int(previous)
    sign = "+" if delta >= 0 else ""
    return f"({sign}{delta})", delta


def _build_console_diag_suffix(
    status: StatusSnapshot,
    state: _ConsoleDiagState,
    *,
    metrics_path: str,
    ethercat_cmd: str,
    ethercat_every: int,
) -> str:
    metrics = _load_metrics(metrics_path)
    overrun = _try_int(metrics.get("rt_overrun_count"))
    rt_hz = _try_float(metrics.get("rt_hz"))
    last_jitter_ns = _try_int(metrics.get("rt_last_jitter_ns"))
    max_jitter_ns = _try_int(metrics.get("rt_max_abs_jitter_ns"))

    every = max(1, int(ethercat_every))
    if state.sample_count % every == 0 or state.last_lost_frames is None:
        polled = _read_ethercat_lost_frames(ethercat_cmd)
        if polled is not None:
            state.last_lost_frames = int(polled)
    lost_frames = state.last_lost_frames

    overrun_delta_txt, overrun_delta = _fmt_delta(overrun, state.prev_overrun)
    lost_delta_txt, lost_delta = _fmt_delta(lost_frames, state.prev_lost_frames)

    if overrun is not None:
        state.prev_overrun = int(overrun)
    if lost_frames is not None:
        state.prev_lost_frames = int(lost_frames)

    alerts: list[str] = []
    if overrun_delta is not None and overrun_delta > 0:
        alerts.append(f"OVERRUN+{overrun_delta}")
    if status.wkc_expected > 0 and status.wkc_actual != status.wkc_expected:
        alerts.append("WKC_MISMATCH")
    if lost_delta is not None and lost_delta > 0:
        alerts.append(f"LOST+{lost_delta}")
    alert_text = ",".join(alerts) if alerts else "-"

    overrun_txt = "n/a" if overrun is None else str(int(overrun))
    lost_txt = "n/a" if lost_frames is None else str(int(lost_frames))
    hz_txt = "n/a" if rt_hz is None else f"{float(rt_hz):.1f}"

    if last_jitter_ns is None or max_jitter_ns is None:
        jitter_txt = "n/a/n/a"
    else:
        jitter_txt = f"{abs(int(last_jitter_ns))/1000.0:.1f}/{int(max_jitter_ns)/1000.0:.1f}"

    state.sample_count += 1
    return (
        f"diag: ov={overrun_txt}{overrun_delta_txt} "
        f"lost={lost_txt}{lost_delta_txt} "
        f"rt_hz={hz_txt} j_us={jitter_txt} alerts={alert_text}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="RTCore jog/bring-up CLI")
    ap.add_argument("--socket", default="/run/gradient-rt-motion/ipc.sock", help="RTCore IPC socket path")

    sub = ap.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("status", help="Print latest RTCore status snapshot")
    sp.add_argument("--timeout", type=float, default=0.5)

    sp = sub.add_parser("watch", help="Continuously print RTCore status snapshots")
    sp.add_argument("--rate-hz", type=float, default=10.0)
    sp.add_argument("--timeout", type=float, default=0.5)

    sp = sub.add_parser("console", help="Watch + send commands in one connection (single-client RTCore)")
    sp.add_argument("--rate-hz", type=float, default=2.0)
    sp.add_argument("--timeout", type=float, default=0.5)
    sp.add_argument("--counts-per-rev", type=int, default=131072)
    sp.add_argument("--gear-ratio", type=float, default=1.0)
    sp.add_argument("--sign", type=int, choices=(-1, 1), default=1)
    sp.add_argument("--no-watch", action="store_true", help="Disable periodic status printing (type 'status' manually)")
    sp.add_argument("--no-diag", action="store_true", help="Disable diagnostic suffix in watch output")
    sp.add_argument("--diag-metrics-path", default="/run/gradient-rt-motion/metrics.json", help="Path to RTCore metrics.json")
    sp.add_argument(
        "--diag-ethercat-cmd",
        default="sudo ethercat master",
        help="Command used to read EtherCAT lost frames",
    )
    sp.add_argument(
        "--diag-ethercat-every",
        type=int,
        default=2,
        help="Poll EtherCAT lost frames every N printed status lines (default: 2)",
    )

    sp = sub.add_parser("test", help="Motion validation console (alias of console)")
    sp.add_argument("--rate-hz", type=float, default=2.0)
    sp.add_argument("--timeout", type=float, default=0.5)
    sp.add_argument("--counts-per-rev", type=int, default=131072)
    sp.add_argument("--gear-ratio", type=float, default=1.0)
    sp.add_argument("--sign", type=int, choices=(-1, 1), default=1)
    sp.add_argument("--no-watch", action="store_true", help="Disable periodic status printing (type 'status' manually)")
    sp.add_argument("--no-diag", action="store_true", help="Disable diagnostic suffix in watch output")
    sp.add_argument("--diag-metrics-path", default="/run/gradient-rt-motion/metrics.json", help="Path to RTCore metrics.json")
    sp.add_argument(
        "--diag-ethercat-cmd",
        default="sudo ethercat master",
        help="Command used to read EtherCAT lost frames",
    )
    sp.add_argument(
        "--diag-ethercat-every",
        type=int,
        default=2,
        help="Poll EtherCAT lost frames every N printed status lines (default: 2)",
    )

    sp = sub.add_parser("arm", help="Arm RTCore and optionally set enable mask")
    sp.add_argument("--enable-mask", default=None, help="Axis enable bitmask (e.g. 0x1, 0x3)")

    sp = sub.add_parser("disarm", help="Disarm RTCore")

    sp = sub.add_parser("enable", help="Set axis enable mask (overwrites mask)")
    sp.add_argument("--mask", required=True, help="Axis enable bitmask (e.g. 0x1, 0x3)")

    sp = sub.add_parser("disable", help="Clear bits from the axis enable mask")
    sp.add_argument("--mask", required=True, help="Axis bitmask to clear (e.g. 0x2)")

    sp = sub.add_parser("fault_reset", help="Request DS402 fault reset pulse (axis_mask=0 means all)")
    sp.add_argument("--mask", default="0", help="Axis bitmask to reset (e.g. 0x1). Use 0 for all axes.")

    sp = sub.add_parser("set", help="Set an absolute target for one axis (q units, default rad)")
    sp.add_argument("--axis", type=int, required=True)
    sp.add_argument("--q", type=float, required=True, help="Absolute target in q units (rad for rotary)")

    sp = sub.add_parser("jog", help="Jog one axis relative to current feedback")
    sp.add_argument("--axis", type=int, required=True)
    g = sp.add_mutually_exclusive_group(required=True)
    g.add_argument("--delta-rad", type=float, help="Relative delta in q units (rotary=rad, linear=m)")
    g.add_argument("--delta-counts", type=int, help="Relative delta in raw counts")
    sp.add_argument("--counts-per-rev", type=int, default=131072)
    sp.add_argument("--gear-ratio", type=float, default=1.0)
    sp.add_argument("--sign", type=int, choices=(-1, 1), default=1, help="Sign used in RTCore scaling")
    sp.add_argument("--timeout", type=float, default=0.5, help="Status snapshot wait timeout")

    args = ap.parse_args()

    try:
        with RTCoreClient(args.socket) as c:
            if args.cmd == "status":
                # On initial connect, RTCore first emits HELLO/AXIS_CONFIG and then begins
                # periodic STATUS_SNAPSHOT (typically within ~50-100ms). So "status" waits
                # up to --timeout for the first snapshot.
                timeout_s = float(args.timeout)
                deadline = time.monotonic() + max(0.0, timeout_s)
                snap: Optional[StatusSnapshot] = None
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    snap = c.read_status_snapshot(timeout_s=min(0.2, remaining))
                    if snap is not None:
                        break
                    time.sleep(0.01)

                if snap is None:
                    print("No status snapshot received (is RTCore running?)", file=sys.stderr)
                    return 2
                _print_status(snap)
                return 0

            if args.cmd == "watch":
                period_s = 1.0 / max(0.1, float(args.rate_hz))
                last_print = 0.0
                while True:
                    snap = c.read_status_snapshot(timeout_s=float(args.timeout))
                    if snap is not None:
                        now = time.monotonic()
                        if (now - last_print) >= period_s:
                            _print_status(snap)
                            last_print = now
                    else:
                        time.sleep(min(0.05, period_s))

            if args.cmd in {"console", "test"}:
                # Single RTCore client policy means: one connection must do both watch + commands.
                try:
                    import readline  # type: ignore
                except Exception:
                    readline = None  # type: ignore

                prompt = "rtcore> "
                watch_enabled = not bool(args.no_watch)
                period_s = 1.0 / max(0.1, float(args.rate_hz))
                diag_enabled = not bool(args.no_diag)
                diag_metrics_path = str(args.diag_metrics_path)
                diag_ethercat_cmd = str(args.diag_ethercat_cmd)
                diag_ethercat_every = max(1, int(args.diag_ethercat_every))

                fallback_cpu = _counts_per_rad(int(args.counts_per_rev), float(args.gear_ratio))
                if fallback_cpu <= 0:
                    raise ValueError("Invalid counts-per-rad (check --counts-per-rev/--gear-ratio)")

                io_lock = threading.Lock()
                print_lock = threading.Lock()
                stop_evt = threading.Event()
                input_active = threading.Event()
                diag_lock = threading.Lock()
                diag_state = _ConsoleDiagState()

                last_snap: Optional[StatusSnapshot] = None
                last_print = 0.0
                last_diag_line = "diag: disabled"

                def safe_print_line(line: str) -> None:
                    # Print a line without wrecking the current input prompt.
                    with print_lock:
                        if input_active.is_set() and readline is not None:
                            buf = readline.get_line_buffer()  # type: ignore[attr-defined]
                            sys.stdout.write("\r\033[K")
                            sys.stdout.write(line + "\n")
                            sys.stdout.write(prompt + buf)
                            sys.stdout.flush()
                        else:
                            print(line, flush=True)

                def safe_print_block(lines: list[str]) -> None:
                    with print_lock:
                        if input_active.is_set() and readline is not None:
                            buf = readline.get_line_buffer()  # type: ignore[attr-defined]
                            sys.stdout.write("\r\033[K")
                            for ln in lines:
                                sys.stdout.write(ln + "\n")
                            sys.stdout.write(prompt + buf)
                            sys.stdout.flush()
                        else:
                            for ln in lines:
                                print(ln, flush=True)

                def status_loop() -> None:
                    nonlocal last_snap, last_print, last_diag_line
                    while not stop_evt.is_set():
                        snap: Optional[StatusSnapshot] = None
                        with io_lock:
                            snap = c.read_status_snapshot(timeout_s=min(float(args.timeout), 0.2))
                        if snap is not None:
                            last_snap = snap
                            now = time.monotonic()
                            if watch_enabled and (now - last_print) >= period_s:
                                line = _format_status_one_line(snap)
                                if diag_enabled:
                                    with diag_lock:
                                        last_diag_line = _build_console_diag_suffix(
                                            snap,
                                            diag_state,
                                            metrics_path=diag_metrics_path,
                                            ethercat_cmd=diag_ethercat_cmd,
                                            ethercat_every=diag_ethercat_every,
                                        )
                                        line += f" | {last_diag_line}"
                                safe_print_line(line)
                                last_print = now
                        else:
                            time.sleep(0.02)

                safe_print_block(
                    [
                        "RTCore console (single RTCore client). Commands:",
                        "  help | status | w [on|off] | arm [0xMASK] | disarm",
                        "  enable 0xMASK | disable 0xMASK",
                        "  set AXIS Q | jog AXIS DELTA_Q | jogc AXIS DELTA_COUNTS",
                        "  reset [0xMASK] | config",
                        "  quit",
                        f"  diagnostics: {'on' if diag_enabled else 'off'} "
                        f"(use --no-diag to disable)",
                    ]
                )

                t = threading.Thread(target=status_loop, daemon=True)
                t.start()
                try:
                    while True:
                        input_active.set()
                        try:
                            line = input(prompt)
                        finally:
                            input_active.clear()

                        line = line.strip()
                        if not line:
                            continue
                        parts = line.split()
                        cmd0 = parts[0].lower()

                        if cmd0 in {"quit", "exit"}:
                            return 0
                        if cmd0 == "help":
                            safe_print_block(
                                [
                                    "Commands: help, status, w [on|off], arm [0xMASK], disarm, enable 0xMASK, disable 0xMASK,",
                                    "          set AXIS Q, jog AXIS DELTA_Q, jogc AXIS DELTA_COUNTS, reset [0xMASK], config, quit",
                                    "          watch lines include diagnostics by default (use --no-diag to disable)",
                                ]
                            )
                            continue
                        if cmd0 in {"w", "watch"}:
                            # Default: toggle.
                            if len(parts) == 1:
                                watch_enabled = not watch_enabled
                                safe_print_line(f"watch {'on' if watch_enabled else 'off'}")
                                continue
                            # Optional explicit.
                            if parts[1].lower() not in {"on", "off"}:
                                safe_print_line("Usage: w [on|off]")
                                continue
                            watch_enabled = (parts[1].lower() == "on")
                            safe_print_line(f"watch {'on' if watch_enabled else 'off'}")
                            continue
                        if cmd0 == "status":
                            snap = last_snap
                            if snap is None:
                                with io_lock:
                                    snap = c.read_status_snapshot(timeout_s=float(args.timeout))
                            if snap is None:
                                safe_print_line("No status snapshot received yet.")
                                continue
                            # Print full per-axis block (readable, but not spammy).
                            lines: list[str] = []
                            lines.append(
                                f"RTCore status: num_axes={snap.num_axes} wkc={snap.wkc_actual}/{snap.wkc_expected} "
                                f"master_state={snap.master_state} topo=0x{snap.topology_hash:016x}"
                            )
                            for i in range(snap.num_axes):
                                a = snap.axes[i]
                                bus_name = _bus_fault_name_603f(a.error_code)
                                err_suffix = f" ({bus_name})" if bus_name else ""
                                lines.append(
                                    "  "
                                    f"axis{i}: pos_counts={a.pos_counts} sw=0x{a.statusword:04x} ds402={a.ds402_state} "
                                    f"err=0x{a.error_code:04x}{err_suffix} mode_disp={a.mode_display} torque_raw={a.torque_raw} "
                                    f"di=0x{a.di_bits:08x}"
                                )
                            if diag_enabled:
                                with diag_lock:
                                    last_diag_line = _build_console_diag_suffix(
                                        snap,
                                        diag_state,
                                        metrics_path=diag_metrics_path,
                                        ethercat_cmd=diag_ethercat_cmd,
                                        ethercat_every=diag_ethercat_every,
                                    )
                                    lines.append(f"  {last_diag_line}")
                            safe_print_block(lines)
                            continue

                        if cmd0 in {"config", "cfg"}:
                            cfg = c.axis_config
                            if cfg is None:
                                safe_print_line("No axis config received yet.")
                                continue
                            lines: list[str] = []
                            lines.append(f"Axis config: num_axes={cfg.num_axes}")
                            for i in range(int(cfg.num_axes)):
                                at = int(cfg.axis_type[i])
                                at_name = "linear" if at == _AXIS_TYPE_LINEAR else "rotary"
                                unit = _axis_unit_name(at)
                                lines.append(
                                    "  "
                                    f"axis{i}: type={at_name} sign={int(cfg.sign[i])} "
                                    f"cpr={int(cfg.counts_per_rev[i])} gr={float(cfg.gear_ratio[i])} "
                                    f"counts_per_{unit}={float(cfg.counts_per_unit[i]):.6f}"
                                )
                            safe_print_block(lines)
                            continue

                        try:
                            if cmd0 == "arm":
                                mask = int(parts[1], 0) if len(parts) >= 2 else None
                                with io_lock:
                                    c.arm(True, enable_mask=mask)
                                safe_print_line("armed")
                                continue
                            if cmd0 == "disarm":
                                with io_lock:
                                    c.arm(False)
                                safe_print_line("disarmed")
                                continue
                            if cmd0 == "enable":
                                if len(parts) < 2:
                                    safe_print_line("Usage: enable 0xMASK")
                                    continue
                                with io_lock:
                                    c.set_enable_mask(int(parts[1], 0))
                                safe_print_line("enabled mask set")
                                continue
                            if cmd0 == "disable":
                                if len(parts) < 2:
                                    safe_print_line("Usage: disable 0xMASK")
                                    continue
                                with io_lock:
                                    c.disable_mask(int(parts[1], 0))
                                safe_print_line("disabled mask cleared")
                                continue
                            if cmd0 == "set":
                                if len(parts) < 3:
                                    safe_print_line("Usage: set AXIS Q")
                                    continue
                                axis = int(parts[1])
                                qv = float(parts[2])
                                q = [0.0] * _MAX_AXES
                                q[axis] = qv
                                with io_lock:
                                    c.write_setpoint(q, axis_mask=(1 << axis))
                                safe_print_line(f"set axis{axis} q={qv}")
                                continue
                            if cmd0 in {"reset", "fault_reset", "faultreset"}:
                                mask = int(parts[1], 0) if len(parts) >= 2 else 0
                                with io_lock:
                                    c.fault_reset(mask)
                                safe_print_line(f"fault reset requested (mask=0x{mask:x})")
                                continue
                            if cmd0 in {"jog", "jogc"}:
                                if len(parts) < 3:
                                    safe_print_line("Usage: jog AXIS DELTA_Q  |  jogc AXIS DELTA_COUNTS")
                                    continue
                                axis = int(parts[1])
                                snap = last_snap
                                if snap is None:
                                    with io_lock:
                                        snap = c.read_status_snapshot(timeout_s=float(args.timeout))
                                if snap is None:
                                    safe_print_line("No status snapshot yet; cannot jog.")
                                    continue
                                if axis < 0 or axis >= snap.num_axes:
                                    safe_print_line(f"Axis {axis} not present (num_axes={snap.num_axes})")
                                    continue
                                cur_counts = int(snap.axes[axis].pos_counts)
                                sgn, cpu, at = _get_axis_scale(
                                    c,
                                    axis,
                                    fallback_counts_per_unit=float(fallback_cpu),
                                    fallback_sign=int(args.sign),
                                )
                                if cmd0 == "jogc":
                                    delta_counts = int(parts[2])
                                    target_counts = cur_counts + delta_counts
                                    target_q = float(target_counts) / (float(sgn) * float(cpu))
                                else:
                                    delta_q = float(parts[2])
                                    target_q = (float(cur_counts) / (float(sgn) * float(cpu))) + delta_q
                                q = [0.0] * _MAX_AXES
                                q[axis] = float(target_q)
                                with io_lock:
                                    c.write_setpoint(q, axis_mask=(1 << axis))
                                safe_print_line(
                                    f"jog axis{axis}: pos_counts={cur_counts} -> q_target={target_q:.6f} {_axis_unit_name(at)}"
                                )
                                continue

                            safe_print_line(f"Unknown command: {cmd0} (type 'help')")
                        except Exception as e:
                            safe_print_line(f"Command error: {e}")
                finally:
                    stop_evt.set()
                    t.join(timeout=0.5)
                return 0

            if args.cmd == "arm":
                mask = int(args.enable_mask, 0) if args.enable_mask is not None else None
                c.arm(True, enable_mask=mask)
                return 0

            if args.cmd == "disarm":
                c.arm(False)
                return 0

            if args.cmd == "enable":
                mask = int(args.mask, 0)
                c.set_enable_mask(mask)
                return 0

            if args.cmd == "disable":
                mask = int(args.mask, 0)
                c.disable_mask(mask)
                return 0

            if args.cmd == "fault_reset":
                mask = int(args.mask, 0)
                c.fault_reset(mask)
                return 0

            if args.cmd == "set":
                axis = int(args.axis)
                if axis < 0 or axis >= _MAX_AXES:
                    raise ValueError("--axis out of range")
                q = [0.0] * _MAX_AXES
                q[axis] = float(args.q)
                c.write_setpoint(q, axis_mask=(1 << axis))
                return 0

            if args.cmd == "jog":
                axis = int(args.axis)
                if axis < 0 or axis >= _MAX_AXES:
                    raise ValueError("--axis out of range")
                snap = c.read_status_snapshot(timeout_s=float(args.timeout))
                if snap is None:
                    print("No status snapshot received (cannot jog without feedback)", file=sys.stderr)
                    return 2
                if axis >= snap.num_axes:
                    print(f"Axis {axis} not present (num_axes={snap.num_axes})", file=sys.stderr)
                    return 2

                cur_counts = int(snap.axes[axis].pos_counts)
                fallback_cpu = _counts_per_rad(int(args.counts_per_rev), float(args.gear_ratio))
                if fallback_cpu <= 0:
                    raise ValueError("Invalid counts-per-rad (check --counts-per-rev/--gear-ratio)")
                sgn, cpu, at = _get_axis_scale(
                    c,
                    axis,
                    fallback_counts_per_unit=float(fallback_cpu),
                    fallback_sign=int(args.sign),
                )
                if cpu <= 0:
                    raise ValueError("Invalid counts-per-unit (RTCore config)")

                if args.delta_counts is not None:
                    target_counts = cur_counts + int(args.delta_counts)
                    target_q = float(target_counts) / (float(sgn) * float(cpu))
                else:
                    target_q = (float(cur_counts) / (float(sgn) * float(cpu))) + float(args.delta_rad)

                q = [0.0] * _MAX_AXES
                q[axis] = float(target_q)
                c.write_setpoint(q, axis_mask=(1 << axis))
                print(f"Jog axis{axis}: pos_counts={cur_counts} -> q_target={target_q:.6f} {_axis_unit_name(at)}")
                return 0

        return 0
    except FileNotFoundError:
        print(f"RTCore socket not found: {args.socket}", file=sys.stderr)
        return 2
    except (ConnectionResetError, BrokenPipeError) as e:
        print(f"RTCore rejected the connection: {e}", file=sys.stderr)
        print(
            "Hint: RTCore is single-client. Stop any running controller / rtcore_jog session, "
            "or restart RTCore, then try again.",
            file=sys.stderr,
        )
        return 2
    except PermissionError as e:
        print(f"Permission error: {e}", file=sys.stderr)
        print("Hint: run as user 'pi' (socket is 0660 root:pi).", file=sys.stderr)
        return 2
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

