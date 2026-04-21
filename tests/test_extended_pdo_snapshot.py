"""Regression tests for Phase 1 of the canonical-truth stability work.

Phase 1 adds a new IPC message (``StatusExtendedSnapshotV1``) that carries
the extended A6-EC 0x2040 PDO diagnostic pack: multi-turn low/high, bus
voltage, load rate, IGBT/motor temperature, position error, and the two
"why isn't the drive moving" diagnostic enums. These tests prove the
Python side parses the on-wire bytes correctly, stores them in the
backend state, and exposes them via scaled accessors.

Scope: this phase is plumbing only. No canonical-truth consumer logic is
wired into the shaft-frame gate yet (that's Phase 3); we simply verify
that the bytes flow end-to-end through ``_ingest_extended_snapshot_payload``
into the per-axis ``_axis_*`` arrays and back out through the scaled
accessors.
"""

from __future__ import annotations

import struct

import pytest

from gradient_os.arm_controller.backends.ethercat_rtcore.backend import (
    EthercatRTCoreBackend,
    _AXIS_STATUS_EXT_STRUCT,
    _GRADIENT_MAX_AXES,
    _MSG_STATUS_EXTENDED_SNAPSHOT,
    _STATUS_EXTENDED_SNAPSHOT_HEADER_STRUCT,
)
from gradient_os.arm_controller.robots.gradient05.config import Gradient05Config


def _build_extended_snapshot_payload(
    *,
    num_axes: int,
    valid_axis_mask: int,
    sample_time_ns: int,
    axes: list[dict[str, int]] | None = None,
) -> bytes:
    """Build a StatusExtendedSnapshotV1 byte payload matching the C++
    struct layout in ipc_v1.hpp. Axes not provided are zeroed."""
    header = _STATUS_EXTENDED_SNAPSHOT_HEADER_STRUCT.pack(
        int(num_axes), int(valid_axis_mask), int(sample_time_ns)
    )
    axes_bytes = bytearray()
    axes = axes or []
    for axis_i in range(_GRADIENT_MAX_AXES):
        entry = axes[axis_i] if axis_i < len(axes) else {}
        axes_bytes += _AXIS_STATUS_EXT_STRUCT.pack(
            int(entry.get("position_error_counts", 0)),
            int(entry.get("multi_turn_lo", 0)),
            int(entry.get("multi_turn_hi", 0)),
            int(entry.get("bus_voltage_raw", 0)),
            int(entry.get("load_rate_raw", 0)),
            int(entry.get("igbt_temp_raw", 0)),
            int(entry.get("motor_temp_raw", 0)),
            int(entry.get("drive_not_ready_bits", 0)),
            int(entry.get("motor_not_rotating_code", 0)),
        )
    return header + bytes(axes_bytes)


def test_axis_status_ext_struct_size_is_24_bytes() -> None:
    """Matches the static_assert in ipc_v1.hpp."""
    assert _AXIS_STATUS_EXT_STRUCT.size == 24


def test_status_extended_snapshot_header_size_is_16_bytes() -> None:
    """num_axes(u32) + valid_axis_mask(u32) + sample_time_ns(u64)."""
    assert _STATUS_EXTENDED_SNAPSHOT_HEADER_STRUCT.size == 16


def test_status_extended_snapshot_full_payload_is_400_bytes() -> None:
    """Full on-wire payload: 16-byte header + 16 axes × 24 bytes = 400."""
    payload = _build_extended_snapshot_payload(
        num_axes=6, valid_axis_mask=0x3F, sample_time_ns=1_000_000_000
    )
    assert len(payload) == 400


def test_message_type_extended_snapshot_is_206() -> None:
    """ipc v1.1 assigned 0x0206 to the new message type."""
    assert _MSG_STATUS_EXTENDED_SNAPSHOT == 0x0206


def test_parse_extended_snapshot_roundtrip(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    """Hand-build a payload, feed it through ``_ingest_extended_snapshot_payload``,
    assert every field is read back correctly for the axes flagged valid."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    axis0 = {
        "position_error_counts": 42,
        "multi_turn_lo": 123456,
        "multi_turn_hi": -7,
        "bus_voltage_raw": 4800,  # 480.0 V
        "load_rate_raw": 235,      # 23.5 %
        "igbt_temp_raw": 42,
        "motor_temp_raw": 55,
        "drive_not_ready_bits": 0x02,
        "motor_not_rotating_code": 3,
    }
    axis5 = {
        "position_error_counts": -99,
        "multi_turn_lo": 11,
        "multi_turn_hi": 0,
        "bus_voltage_raw": 4799,
        "load_rate_raw": 0,
        "igbt_temp_raw": 38,
        "motor_temp_raw": 51,
        "drive_not_ready_bits": 0,
        "motor_not_rotating_code": 0,
    }
    axes: list[dict[str, int]] = [dict() for _ in range(_GRADIENT_MAX_AXES)]
    axes[0] = axis0
    axes[5] = axis5

    payload = _build_extended_snapshot_payload(
        num_axes=6,
        valid_axis_mask=(1 << 0) | (1 << 5),  # only axes 0 and 5 valid
        sample_time_ns=1_700_000_123,
        axes=axes,
    )

    backend._ingest_extended_snapshot_payload(payload)

    # Axis 0 is flagged valid, so all fields should be populated and the
    # updated timestamp recorded.
    assert backend._axis_position_error_counts[0] == 42
    assert backend._axis_multi_turn_lo[0] == 123456
    assert backend._axis_multi_turn_hi[0] == -7
    assert backend._axis_bus_voltage_raw[0] == 4800
    assert backend._axis_load_rate_raw[0] == 235
    assert backend._axis_igbt_temp_raw[0] == 42
    assert backend._axis_motor_temp_raw[0] == 55
    assert backend._axis_drive_not_ready_bits[0] == 0x02
    assert backend._axis_motor_not_rotating_code[0] == 3
    assert backend._axis_extended_updated_ns[0] == 1_700_000_123

    # Axis 5 is flagged valid.
    assert backend._axis_position_error_counts[5] == -99
    assert backend._axis_motor_temp_raw[5] == 51
    assert backend._axis_extended_updated_ns[5] == 1_700_000_123

    # Axes 1-4 were NOT in the valid mask; their updated_ns must stay 0
    # so Phase 3 consumers know they are not live.
    for axis_i in (1, 2, 3, 4):
        assert backend._axis_extended_updated_ns[axis_i] == 0
        assert backend._axis_bus_voltage_raw[axis_i] == 0
        assert backend._axis_multi_turn_lo[axis_i] == 0

    # Global book-keeping reflects the most recent snapshot.
    assert backend._axis_extended_latest_sample_ns == 1_700_000_123
    assert backend._axis_extended_valid_mask_last == ((1 << 0) | (1 << 5))


def test_ingest_tolerates_short_payload_without_raising(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    """An under-size payload (e.g. truncated ring entry) must not raise
    through ``_ingest_extended_snapshot_payload`` — the backend caller
    already catches exceptions, but the ingest path itself should also
    be defensive."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    # Return cleanly, no state mutation.
    backend._ingest_extended_snapshot_payload(b"\x00\x00\x00")
    assert backend._axis_extended_latest_sample_ns == 0


def test_axis_bus_voltage_v_scales_0p1v(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    """Raw 4800 should scale to 480.0 V per the A6-EC manual units."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    axes = [{"bus_voltage_raw": 4800} for _ in range(_GRADIENT_MAX_AXES)]
    payload = _build_extended_snapshot_payload(
        num_axes=6,
        valid_axis_mask=0x3F,
        sample_time_ns=1_700_000_000,
        axes=axes,
    )
    backend._ingest_extended_snapshot_payload(payload)
    for axis_i in range(6):
        assert backend._axis_bus_voltage_v(axis_i) == pytest.approx(480.0)


def test_axis_load_rate_pct_scales_0p1pct(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    """Raw 235 should scale to 23.5 %."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    axes = [{"load_rate_raw": 235} for _ in range(_GRADIENT_MAX_AXES)]
    payload = _build_extended_snapshot_payload(
        num_axes=6,
        valid_axis_mask=0x3F,
        sample_time_ns=1_700_000_000,
        axes=axes,
    )
    backend._ingest_extended_snapshot_payload(payload)
    assert backend._axis_load_rate_pct(0) == pytest.approx(23.5)


def test_axis_temperature_accessors_pass_through_integer_degrees(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    """IGBT / motor temperature are INT at 1 deg C per count; accessors
    return plain ints."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    axes = [{"igbt_temp_raw": 42, "motor_temp_raw": 55} for _ in range(_GRADIENT_MAX_AXES)]
    payload = _build_extended_snapshot_payload(
        num_axes=6, valid_axis_mask=0x3F, sample_time_ns=1_700_000_000, axes=axes
    )
    backend._ingest_extended_snapshot_payload(payload)
    assert backend._axis_igbt_temp_c(2) == 42
    assert backend._axis_motor_temp_c(2) == 55


def test_accessors_return_none_when_axis_never_updated(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    """Before any extended snapshot arrives, every accessor must return
    None. Distinguishes "drive rejected extended mapping" (None) from
    "drive says 0.0 V" (0.0)."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    for axis_i in range(6):
        assert backend._axis_bus_voltage_v(axis_i) is None
        assert backend._axis_load_rate_pct(axis_i) is None
        assert backend._axis_igbt_temp_c(axis_i) is None
        assert backend._axis_motor_temp_c(axis_i) is None
        assert backend._axis_position_error_counts_or_none(axis_i) is None
        assert backend._axis_drive_not_ready_bits_or_none(axis_i) is None
        assert backend._axis_motor_not_rotating_code_or_none(axis_i) is None
        assert backend._axis_multi_turn_counts_from_pdo(axis_i) is None


def test_axis_multi_turn_counts_from_pdo_signs_extend_correctly(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    """Multi-turn high 32 bits = -1 and low 32 bits = 0 should combine
    to a signed i64 of -2^32 (the sign-extension is critical for the
    canonical-truth math used downstream in Phase 3)."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    # Case 1: hi=-1, lo=0 => -2^32
    axes1 = [{"multi_turn_lo": 0, "multi_turn_hi": -1} for _ in range(_GRADIENT_MAX_AXES)]
    payload1 = _build_extended_snapshot_payload(
        num_axes=6, valid_axis_mask=0x3F, sample_time_ns=1_000, axes=axes1
    )
    backend._ingest_extended_snapshot_payload(payload1)
    assert backend._axis_multi_turn_counts_from_pdo(0) == -(1 << 32)

    # Case 2: hi=0, lo=0x7FFFFFFF (max positive i32) => 2^31 - 1
    axes2 = [{"multi_turn_lo": 0x7FFFFFFF, "multi_turn_hi": 0} for _ in range(_GRADIENT_MAX_AXES)]
    payload2 = _build_extended_snapshot_payload(
        num_axes=6, valid_axis_mask=0x3F, sample_time_ns=2_000, axes=axes2
    )
    backend._ingest_extended_snapshot_payload(payload2)
    assert backend._axis_multi_turn_counts_from_pdo(0) == 0x7FFFFFFF

    # Case 3: hi=1, lo=0 => exactly 2^32
    axes3 = [{"multi_turn_lo": 0, "multi_turn_hi": 1} for _ in range(_GRADIENT_MAX_AXES)]
    payload3 = _build_extended_snapshot_payload(
        num_axes=6, valid_axis_mask=0x3F, sample_time_ns=3_000, axes=axes3
    )
    backend._ingest_extended_snapshot_payload(payload3)
    assert backend._axis_multi_turn_counts_from_pdo(0) == (1 << 32)

    # Case 4: hi=-1, lo=-1 (signed i32 representation of 0xFFFFFFFF)
    # should combine to -1 as signed i64. The `lo & 0xFFFFFFFF` mask in
    # ``_axis_multi_turn_counts_from_pdo`` is what makes this work.
    axes4 = [{"multi_turn_lo": -1, "multi_turn_hi": -1} for _ in range(_GRADIENT_MAX_AXES)]
    payload4 = _build_extended_snapshot_payload(
        num_axes=6, valid_axis_mask=0x3F, sample_time_ns=4_000, axes=axes4
    )
    backend._ingest_extended_snapshot_payload(payload4)
    assert backend._axis_multi_turn_counts_from_pdo(0) == -1


def test_absolute_axis_q_prefers_pdo_when_extended_data_is_fresh(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """Phase 3 (2026-04-20): ``_absolute_axis_q_from_metrics`` must
    return the atomic PDO multi-turn result with source tag
    ``pdo_multi_turn_atomic`` when the extended snapshot is fresh."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._axis_config = backend._build_axis_config_from_robot_config(
        Gradient05Config().get_config_dict()
    )
    assert backend._axis_config is not None, "Gradient05Config must yield an axis config"

    # Seed extended snapshot freshness and known multi-turn counts for axis 0.
    backend._axis_multi_turn_lo[0] = 123456
    backend._axis_multi_turn_hi[0] = 0
    backend._axis_extended_updated_ns[0] = 1_000_000  # arbitrary monotonic stamp

    fake_now = [1_000_000 + 10_000]  # 10 µs after the snapshot — well within 50 ms
    import gradient_os.arm_controller.backends.ethercat_rtcore.backend as backend_module

    monkeypatch.setattr(backend_module.time, "monotonic_ns", lambda: fake_now[0])

    from gradient_os.arm_controller.backends.ethercat_rtcore.backend import (
        _ABSOLUTE_SOURCE_PDO_MULTI_TURN,
        _AbsoluteFeedbackAxisMetrics,
    )

    metrics = _AbsoluteFeedbackAxisMetrics()
    result = backend._absolute_axis_q_from_metrics(0, metrics)
    assert result is not None
    axis_q, source, counts = result
    assert source == _ABSOLUTE_SOURCE_PDO_MULTI_TURN
    assert counts == 123456
    # axis_q derived from counts / (sign * counts_per_unit).
    sign = int(backend._axis_config.sign[0])
    cpu = float(backend._axis_config.counts_per_unit[0])
    assert axis_q == pytest.approx(123456.0 / (sign * cpu))


def test_absolute_axis_q_falls_back_to_sdo_when_pdo_stale(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """If the last extended-snapshot sample is older than
    ``_PDO_MULTI_TURN_FRESH_NS``, the backend must fall through to the
    legacy SDO path instead of returning stale PDO data."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._axis_config = backend._build_axis_config_from_robot_config(
        Gradient05Config().get_config_dict()
    )
    assert backend._axis_config is not None

    backend._axis_multi_turn_lo[0] = 999999
    backend._axis_multi_turn_hi[0] = 0
    backend._axis_extended_updated_ns[0] = 1_000_000_000

    import gradient_os.arm_controller.backends.ethercat_rtcore.backend as backend_module
    from gradient_os.arm_controller.backends.ethercat_rtcore.backend import (
        _AbsoluteFeedbackAxisMetrics,
        _PDO_MULTI_TURN_FRESH_NS,
    )

    # Advance monotonic time past the freshness window.
    fake_now = 1_000_000_000 + _PDO_MULTI_TURN_FRESH_NS + 1
    monkeypatch.setattr(backend_module.time, "monotonic_ns", lambda: fake_now)

    # With no SDO metrics present, the fallback returns None (empty metrics).
    metrics = _AbsoluteFeedbackAxisMetrics()
    result = backend._absolute_axis_q_from_metrics(0, metrics)
    assert result is None, "PDO stale path should NOT return PDO counts"


def test_absolute_axis_q_skips_pdo_when_axis_never_updated(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """Axes where the drive rejected the extended PDO mapping have
    ``_axis_extended_updated_ns == 0``; those axes must fall through to
    the SDO path rather than synthesising bogus PDO counts."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._axis_config = backend._build_axis_config_from_robot_config(
        Gradient05Config().get_config_dict()
    )

    backend._axis_multi_turn_lo[0] = 5555
    backend._axis_multi_turn_hi[0] = 0
    backend._axis_extended_updated_ns[0] = 0  # never received

    from gradient_os.arm_controller.backends.ethercat_rtcore.backend import (
        _AbsoluteFeedbackAxisMetrics,
    )

    metrics = _AbsoluteFeedbackAxisMetrics()
    result = backend._absolute_axis_q_from_metrics(0, metrics)
    # No PDO data, no SDO data -> the function must return None rather
    # than fabricating a reading.
    assert result is None


def test_ingest_preserves_prior_values_when_axis_not_in_mask(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    """A dropped RT cycle (axis not in valid_axis_mask) must not stale-zero
    previously-valid per-axis data."""
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json")
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    # First snapshot: all six axes valid.
    axes_a = [{"bus_voltage_raw": 4800, "motor_temp_raw": 50} for _ in range(_GRADIENT_MAX_AXES)]
    backend._ingest_extended_snapshot_payload(
        _build_extended_snapshot_payload(
            num_axes=6, valid_axis_mask=0x3F, sample_time_ns=1_000, axes=axes_a
        )
    )
    assert backend._axis_bus_voltage_v(3) == pytest.approx(480.0)

    # Second snapshot: axis 3 dropped from mask.
    axes_b = [{"bus_voltage_raw": 5000, "motor_temp_raw": 60} for _ in range(_GRADIENT_MAX_AXES)]
    backend._ingest_extended_snapshot_payload(
        _build_extended_snapshot_payload(
            num_axes=6,
            valid_axis_mask=0x3F & ~(1 << 3),
            sample_time_ns=2_000,
            axes=axes_b,
        )
    )
    # Axes 0, 1, 2, 4, 5 have fresh values.
    assert backend._axis_bus_voltage_v(0) == pytest.approx(500.0)
    # Axis 3 still holds the prior 4800 raw.
    assert backend._axis_bus_voltage_raw[3] == 4800
    # Its updated_ns still reflects the first snapshot — Phase 3 uses
    # this to decide whether to trust it or fall back to SDO.
    assert backend._axis_extended_updated_ns[3] == 1_000
