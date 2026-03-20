#pragma once

#include <cstddef>
#include <cstdint>

// IPC ABI v1 for Python <-> RTCore.
//
// Source of truth:
// - RTOS-ETHERCAT-PLAN/RTOS-ETHERCAT-plan.md §15.4
//
// Rules:
// - Little-endian
// - Fixed-size structs
// - Shared memory ring slots are fixed-size (default 512 bytes)

namespace gradient::ipc::v1 {

constexpr uint32_t fourcc(char a, char b, char c, char d) noexcept {
  return (static_cast<uint32_t>(static_cast<unsigned char>(a)) << 0) |
         (static_cast<uint32_t>(static_cast<unsigned char>(b)) << 8) |
         (static_cast<uint32_t>(static_cast<unsigned char>(c)) << 16) |
         (static_cast<uint32_t>(static_cast<unsigned char>(d)) << 24);
}

constexpr uint32_t kMagicGipc = fourcc('G', 'I', 'P', 'C');
constexpr uint32_t kMagicGshm = fourcc('G', 'S', 'H', 'M');
constexpr uint32_t kMagicRing = fourcc('R', 'I', 'N', 'G');

constexpr uint16_t kVerMajor = 1;
constexpr uint16_t kVerMinor = 0;

constexpr uint32_t kRoleController = 1;
constexpr uint32_t kRoleObserver = 2; // reserved (future)

constexpr uint32_t kShmKindCmd = 1;
constexpr uint32_t kShmKindStatus = 2;

// v1 supports up to 6 arm axes + external axes + tool/end-effector axes.
constexpr uint32_t GRADIENT_MAX_AXES = 16;
constexpr uint32_t GRADIENT_MAX_IO_DEVICES = 8;

constexpr uint32_t GRADIENT_RING_MSG_BYTES = 512;
constexpr uint32_t GRADIENT_CMD_RING_CAPACITY = 128;
constexpr uint32_t GRADIENT_STATUS_RING_CAPACITY = 1024;

// ----- UDS handshake -----

// UDS payload for HELLO (no FDs).
struct HelloV1 {
  uint32_t magic;     // 'GIPC'
  uint16_t ver_major; // 1
  uint16_t ver_minor; // 0
  uint32_t bytes;     // sizeof(HelloV1)
  uint32_t role;      // 1=controller (writer), 2=observer (future)
  uint64_t pid;       // client pid (debug)
  uint64_t reserved[4];
};
static_assert(sizeof(HelloV1) == 56, "HelloV1 size must match spec");

// UDS payload for WELCOME (plus SCM_RIGHTS FDs).
struct WelcomeV1 {
  uint32_t magic;     // 'GIPC'
  uint16_t ver_major; // 1
  uint16_t ver_minor; // 0
  uint32_t bytes;     // sizeof(WelcomeV1)

  uint32_t num_axes; // number of motion axes exposed (arm + external + tool)
  uint32_t reserved0;

  uint64_t cycle_ns;      // 1_000_000 nominal
  uint64_t topology_hash; // expected bus identity/order hash

  uint32_t cmd_ring_capacity;
  uint32_t cmd_msg_bytes; // fixed size per ring entry
  uint32_t status_ring_capacity;
  uint32_t status_msg_bytes; // fixed size per ring entry

  uint64_t build_id_hash; // RTCore build identifier (e.g. git sha hash)
  uint64_t reserved1[4];
};
static_assert(sizeof(WelcomeV1) == 96, "WelcomeV1 size must match spec");

// ----- Shared memory layout -----

struct ShmHeaderV1 {
  uint32_t magic;     // 'GSHM'
  uint16_t ver_major; // 1
  uint16_t ver_minor; // 0
  uint32_t bytes;     // sizeof(ShmHeaderV1)
  uint32_t kind;      // 1=cmd_shm, 2=status_shm

  uint32_t num_axes; // <= GRADIENT_MAX_AXES
  uint32_t reserved0;
  uint64_t cycle_ns; // 1_000_000 nominal
  uint64_t topology_hash;

  uint32_t ring_offset;    // byte offset from shm base
  uint32_t ring_capacity;  // number of entries
  uint32_t ring_msg_bytes; // fixed size per entry

  // cmd_shm only:
  uint32_t setpoint_offset; // 0 if not present
  uint32_t reserved1;

  uint64_t reserved2[8];
};
static_assert(sizeof(ShmHeaderV1) == 128, "ShmHeaderV1 size must match spec");

struct RingHeaderV1 {
  uint32_t magic;    // 'RING'
  uint32_t capacity; // N entries
  uint32_t msg_bytes; // fixed
  uint32_t write_idx; // producer-owned (SPSC)
  uint32_t read_idx;  // consumer-owned (SPSC)
  uint32_t dropped;   // producer increments on overflow
  uint32_t reserved0;
};
static_assert(sizeof(RingHeaderV1) == 28, "RingHeaderV1 size must match spec");

// ----- Message primitives -----

struct MsgHeader {
  uint16_t type;  // enum MsgTypeV1 (numeric values in spec)
  uint16_t flags; // type-specific
  uint32_t bytes; // total bytes of this message (header+payload), fixed per type in v1
  uint64_t seq;   // monotonic per-writer
  uint64_t time_ns; // CLOCK_MONOTONIC timestamp at producer
};
static_assert(sizeof(MsgHeader) == 24, "MsgHeader size must match spec");

// “Latest-wins” setpoint; NOT a ring. Writer updates fields then increments seq.
struct SetpointSlotV1 {
  uint64_t seq;            // atomic in spec (writer increments after writing payload)
  uint64_t target_time_ns; // when this setpoint should be achieved (CLOCK_MONOTONIC)
  double q[GRADIENT_MAX_AXES]; // axis targets in configured units (rad for rotary, m for linear, etc.)
  uint32_t axis_mask;      // bit i=1 means axis i valid
  uint32_t reserved;
};
static_assert(sizeof(SetpointSlotV1) == 152, "SetpointSlotV1 size must match spec");

// ----- Status payloads -----

struct AxisStatusV1 {
  int32_t pos_counts; // 0x6064
  int16_t torque_raw; // 0x6077 (raw units)
  uint16_t statusword; // 0x6041
  uint16_t error_code; // 0x603F
  uint8_t mode_display; // 0x6061
  uint8_t ds402_state;  // derived enum (RTCore-decoded)
  uint16_t reserved0;
  uint32_t di_bits; // 0x60FD
  uint32_t axis_fault_flags;
  uint32_t brake_state; // enum BrakeStateV1
};
static_assert(sizeof(AxisStatusV1) == 28, "AxisStatusV1 size must match spec");

struct StatusSnapshotV1 {
  uint32_t num_axes; // <= GRADIENT_MAX_AXES
  uint32_t wkc_expected;
  uint32_t wkc_actual;
  uint32_t master_state; // enum MasterStateV1
  int64_t dc_offset_ns;
  int64_t cycle_jitter_ns;
  uint64_t topology_hash;
  AxisStatusV1 axes[GRADIENT_MAX_AXES];
};
static_assert(sizeof(StatusSnapshotV1) == 488, "StatusSnapshotV1 size must match spec");

struct EventV1 {
  uint32_t code; // enum EventCodeV1
  uint32_t axis; // 0..(num_axes-1) or 0xFFFFFFFF for global
  int64_t value0;
  int64_t value1;
};
static_assert(sizeof(EventV1) == 24, "EventV1 size must match spec");

struct StatusHelloV1 {
  uint64_t build_id_hash;
  uint64_t topology_hash;
  uint64_t cycle_ns;
  uint32_t num_axes;
  uint32_t drive_profile_id; // e.g. 1=a6ec_ds402
  uint32_t wkc_expected;
  uint32_t reserved0;
};
static_assert(sizeof(StatusHelloV1) == 40, "StatusHelloV1 size must match spec");

// Axis configuration snapshot (emitted on connect/start).
//
// This is intended for bring-up tools (e.g. jog CLI) so they can:
// - interpret raw counts as q-units,
// - display the active scaling per axis,
// - avoid requiring duplicate CLI flags that must match RTCore.
//
// `counts_per_unit` is the derived scale used by RTCore:
// - rotary: counts_per_unit = counts_per_rev * gear_ratio / (2π)  [counts / rad]
// - linear: counts_per_unit = counts_per_rev * gear_ratio / lead_m_per_rev  [counts / m]
struct StatusAxisConfigV1 {
  uint32_t num_axes; // <= GRADIENT_MAX_AXES
  uint32_t reserved0;

  uint32_t counts_per_rev[GRADIENT_MAX_AXES];
  double gear_ratio[GRADIENT_MAX_AXES];
  int32_t sign[GRADIENT_MAX_AXES]; // +1 or -1
  uint8_t axis_type[GRADIENT_MAX_AXES]; // enum AxisTypeV1
  uint8_t reserved1[16];
  double counts_per_unit[GRADIENT_MAX_AXES];
};
static_assert(sizeof(StatusAxisConfigV1) == 424, "StatusAxisConfigV1 size must match spec");

// ----- Command payloads -----

struct CmdArmV1 {
  uint32_t arm; // 0=false, 1=true
  uint32_t reserved;
};
static_assert(sizeof(CmdArmV1) == 8, "CmdArmV1 size must match spec");

struct CmdAxisMaskV1 {
  uint32_t axis_mask;
  uint32_t reserved;
};
static_assert(sizeof(CmdAxisMaskV1) == 8, "CmdAxisMaskV1 size must match spec");

struct CmdFaultResetV1 {
  // 0 means "all axes" (RTCore expands to (1<<num_axes)-1).
  uint32_t axis_mask;
  uint32_t reserved;
};
static_assert(sizeof(CmdFaultResetV1) == 8, "CmdFaultResetV1 size must match spec");

struct CmdSetModeV1 {
  uint32_t axis_mask;
  uint32_t mode; // v1: 8=CSP only
};
static_assert(sizeof(CmdSetModeV1) == 8, "CmdSetModeV1 size must match spec");

struct CmdSoftLimitsV1 {
  uint32_t axis_mask;
  uint32_t reserved0;
  double min_q[GRADIENT_MAX_AXES];
  double max_q[GRADIENT_MAX_AXES];
};
static_assert(sizeof(CmdSoftLimitsV1) == 264, "CmdSoftLimitsV1 size must match spec");

struct CmdRequestBundleV1 {
  uint32_t reason; // optional hint (0=manual)
  uint32_t reserved;
};
static_assert(sizeof(CmdRequestBundleV1) == 8, "CmdRequestBundleV1 size must match spec");

struct CmdIoWriteV1 {
  uint32_t io_id;
  uint32_t reserved0;
  uint64_t do_mask;
  uint64_t do_value;
  uint64_t apply_time_ns; // 0=apply ASAP
};
static_assert(sizeof(CmdIoWriteV1) == 32, "CmdIoWriteV1 size must match spec");

// ----- I/O snapshot payload -----

struct IoDevStatusV1 {
  uint32_t io_id;
  uint32_t width_bits;
  uint64_t di_bits;
  uint64_t do_bits;
  uint32_t fault_flags;
  uint32_t reserved0;
};
static_assert(sizeof(IoDevStatusV1) == 32, "IoDevStatusV1 size must match spec");

struct IoSnapshotV1 {
  uint32_t num_devices;
  uint32_t reserved0;
  IoDevStatusV1 dev[GRADIENT_MAX_IO_DEVICES];
};
static_assert(sizeof(IoSnapshotV1) == 264, "IoSnapshotV1 size must match spec");

// ----- Enumerations (numeric constants) -----

// MsgHeader.type values (v1).
enum : uint16_t {
  MSG_CMD_ARM = 0x0101,
  MSG_CMD_AXIS_ENABLE = 0x0102,
  MSG_CMD_AXIS_DISABLE = 0x0103,
  MSG_CMD_FAULT_RESET = 0x0104,
  MSG_CMD_SET_SOFT_LIMITS = 0x0105,
  MSG_CMD_SET_MODE = 0x0106,
  MSG_CMD_REQUEST_BUNDLE = 0x0107,
  MSG_CMD_IO_WRITE = 0x0110,

  MSG_STATUS_HELLO = 0x0201,
  MSG_STATUS_AXIS_CONFIG = 0x0203,
  MSG_STATUS_SNAPSHOT = 0x0202,
  MSG_STATUS_IO_SNAPSHOT = 0x0210,
  MSG_EVENT = 0x02FF,
};

// StatusSnapshotV1.master_state values (v1).
enum : uint32_t {
  MASTER_UNKNOWN = 0,
  MASTER_INIT = 1,
  MASTER_PREOP = 2,
  MASTER_SAFEOP = 3,
  MASTER_OP = 4,
  MASTER_ERROR = 5,
};

// StatusHelloV1.drive_profile_id values (v1).
enum : uint32_t {
  DRIVE_PROFILE_UNKNOWN = 0,
  DRIVE_PROFILE_A6EC_DS402 = 1,
  DRIVE_PROFILE_CIA402 = 2,
};

// AxisStatusV1.brake_state values (v1).
enum : uint32_t {
  BRAKE_UNKNOWN = 0,
  BRAKE_RELEASED = 1,
  BRAKE_APPLIED = 2,
  BRAKE_WAIT_RELEASE_DELAY = 3,
  BRAKE_WAIT_HOLD_DELAY = 4,
};

// Axis "unit" typing (v1).
enum : uint8_t {
  AXIS_TYPE_UNKNOWN = 0,
  AXIS_TYPE_ROTARY = 1, // q is radians
  AXIS_TYPE_LINEAR = 2, // q is meters
};

// AxisStatusV1.ds402_state values (v1).
enum : uint8_t {
  DS402_UNKNOWN = 0,
  DS402_NOT_READY = 1,
  DS402_SWITCH_ON_DISABLED = 2,
  DS402_READY_TO_SWITCH_ON = 3,
  DS402_SWITCHED_ON = 4,
  DS402_OPERATION_ENABLED = 5,
  DS402_QUICK_STOP_ACTIVE = 6,
  DS402_FAULT_REACTION_ACTIVE = 7,
  DS402_FAULT = 8,
};

// EventV1.code values (v1).
enum : uint32_t {
  EVT_INFO = 0x0001,
  EVT_WARN = 0x0002,
  EVT_ERROR = 0x0003,

  EVT_TOPOLOGY_MISMATCH = 0x0100,
  EVT_SLAVE_NOT_OP = 0x0101,
  EVT_WKC_MISMATCH = 0x0102,
  EVT_DC_UNSTABLE = 0x0103,
  EVT_SETPOINT_STALE = 0x0104,
  EVT_CMD_RING_OVERFLOW = 0x0105,
  EVT_DISARMED = 0x0106,
  EVT_ARMED = 0x0107,
  EVT_IO_FAULT = 0x0110,
};

} // namespace gradient::ipc::v1

