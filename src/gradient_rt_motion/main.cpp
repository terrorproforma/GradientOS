#include "ipc_v1.hpp"

#include <atomic>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <limits>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <grp.h>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

using gradient::ipc::v1::AxisStatusV1;

#include "ecrt_detect.hpp"
#include "ds402.hpp"

#ifndef MFD_CLOEXEC
#define MFD_CLOEXEC 0x0001U
#endif

namespace {

std::atomic<bool> g_stop{false};
// Real drives can settle a few counts away from the commanded final setpoint
// even after the final trajectory point is due. Exact equality caused valid
// queued moves to remain "executing" until the Python-side wait timed out.
constexpr int32_t kTrajectoryCompletionToleranceCounts = 128;

extern "C" void handle_signal(int) {
  g_stop.store(true, std::memory_order_relaxed);
}

uint64_t now_monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

double clamp_double_to_i32_range(double value) {
  const double min_v = static_cast<double>(std::numeric_limits<int32_t>::min());
  const double max_v = static_cast<double>(std::numeric_limits<int32_t>::max());
  if (value < min_v) {
    return min_v;
  }
  if (value > max_v) {
    return max_v;
  }
  return value;
}

int32_t clamp_round_to_i32(double value) {
  const double clamped = clamp_double_to_i32_range(value);
  return static_cast<int32_t>(std::llround(clamped));
}

int32_t clamp_i64_to_i32(int64_t value) {
  if (value < static_cast<int64_t>(std::numeric_limits<int32_t>::min())) {
    return std::numeric_limits<int32_t>::min();
  }
  if (value > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
    return std::numeric_limits<int32_t>::max();
  }
  return static_cast<int32_t>(value);
}

// RTCore writes drive-facing CSP targets in the same raw PDO count frame that
// the drive publishes through 0x6064 and expects on 0x607A. Controller-side
// logical targets still include persisted native-home truth, so RTCore must
// subtract that offset exactly once when it converts queued targets into CSP
// wire counts. Mirroring live feedback back to a hold target must stay in raw
// PDO counts, otherwise SAFE_POWER_UP injects a step roughly equal to 0x607C.
int32_t csp_wire_counts_from_feedback(int32_t feedback_pos_counts) {
  return feedback_pos_counts;
}

double controller_target_to_csp_wire_counts(double controller_target_counts,
                                            int32_t native_home_offset_counts) {
  return controller_target_counts - static_cast<double>(native_home_offset_counts);
}

int64_t shortest_periodic_error_counts(int64_t error_counts, uint32_t period_counts) {
  if (period_counts == 0) {
    return error_counts;
  }
  const int64_t period = static_cast<int64_t>(period_counts);
  const int64_t half_turn = period / 2;
  if (half_turn <= 0) {
    return error_counts;
  }
  int64_t wrapped = (error_counts + half_turn) % period;
  if (wrapped < 0) {
    wrapped += period;
  }
  return wrapped - half_turn;
}

double wrap_counts_into_period_double(double counts, double period_counts) {
  if (!(period_counts > 0.0)) {
    return counts;
  }
  double wrapped = std::fmod(counts, period_counts);
  if (wrapped < 0.0) {
    wrapped += period_counts;
  }
  return wrapped;
}

double shortest_periodic_error_counts_double(double error_counts, double period_counts) {
  if (!(period_counts > 0.0)) {
    return error_counts;
  }
  const double half_turn = 0.5 * period_counts;
  if (!(half_turn > 0.0)) {
    return error_counts;
  }
  double wrapped = std::fmod(error_counts + half_turn, period_counts);
  if (wrapped < 0.0) {
    wrapped += period_counts;
  }
  return wrapped - half_turn;
}

void logf(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  std::fprintf(stderr, "[gradient-rt-motion] ");
  std::vfprintf(stderr, fmt, args);
  std::fprintf(stderr, "\n");
  va_end(args);
}

uint32_t master_state_from_al_states(unsigned int al_states, bool link_up) {
  if (!link_up) {
    return gradient::ipc::v1::MASTER_ERROR;
  }
  if ((al_states & 0x8u) != 0u) {
    return gradient::ipc::v1::MASTER_OP;
  }
  if ((al_states & 0x4u) != 0u) {
    return gradient::ipc::v1::MASTER_SAFEOP;
  }
  if ((al_states & 0x2u) != 0u) {
    return gradient::ipc::v1::MASTER_PREOP;
  }
  if ((al_states & 0x1u) != 0u) {
    return gradient::ipc::v1::MASTER_INIT;
  }
  return gradient::ipc::v1::MASTER_INIT;
}

const char* al_state_label(uint8_t al_state) {
  switch (al_state) {
    case 0x01:
      return "INIT";
    case 0x02:
      return "PREOP";
    case 0x04:
      return "SAFEOP";
    case 0x08:
      return "OP";
    default:
      return "UNKNOWN";
  }
}

size_t align_up(size_t value, size_t alignment) {
  return (value + alignment - 1) / alignment * alignment;
}

struct AxisConfig {
  // Raw scaling inputs.
  uint32_t counts_per_rev = 131072; // common 17-bit encoder counts per rev
  double gear_ratio = 1.0;
  int sign = +1; // +1 or -1 (mechanical orientation)
  bool feedback_counts_wrap = false; // compare feedback modulo the configured wrap period when true
  // Wrap the emitted CSP target (0x607A) into the configured single-turn
  // window [0, wrap_period) when true. When false, the host-supplied
  // continuous target is passed through unchanged, which is what the
  // A6-EC Chapter 5 Figure 5-1 specifies for Absolute Position Rotation
  // Mode (target = continuous linear ramp, feedback = sawtooth). Only
  // affects CSP command emission; feedback comparison and completion
  // checks still use `feedback_counts_wrap` so they stay modulo-RM.
  bool command_counts_wrap = false;
  uint8_t axis_type = gradient::ipc::v1::AXIS_TYPE_ROTARY; // q is radians by default
  double lead_m_per_rev = 0.0; // only used when axis_type==AXIS_TYPE_LINEAR

  // Derived scaling (RTCore uses this for q->counts conversion).
  double counts_per_unit = 0.0; // counts per rad (rotary) or counts per meter (linear)

  // Derived safety clamps from --max-rpm (motor rpm). 0 => disabled.
  int32_t max_step_counts_per_cycle = 0;
  uint32_t max_profile_vel_counts_per_s = 0;
};

uint32_t wrapped_axis_period_counts(const AxisConfig& axis) {
  const uint32_t fallback_period_counts = axis.counts_per_rev;
  if (axis.axis_type != gradient::ipc::v1::AXIS_TYPE_ROTARY || !(axis.counts_per_unit > 0.0)) {
    return fallback_period_counts;
  }

  // Wrapped rotary commands and completion checks both need the geared/output-shaft
  // period so seam-crossing motion stays in the same frame used to derive
  // counts_per_unit and the drive's 6064/607A presentation.
  constexpr double kCompletionWrapTwoPi = 6.28318530717958647692;
  const long long derived_period_counts =
      std::llround(axis.counts_per_unit * kCompletionWrapTwoPi);
  if (derived_period_counts > 0 &&
      derived_period_counts <= static_cast<long long>(std::numeric_limits<uint32_t>::max())) {
    return static_cast<uint32_t>(derived_period_counts);
  }
  return fallback_period_counts;
}

int32_t wrap_counts_into_period(int64_t counts, uint32_t period_counts) {
  if (period_counts == 0) {
    return clamp_i64_to_i32(counts);
  }
  const int64_t period = static_cast<int64_t>(period_counts);
  int64_t wrapped = counts % period;
  if (wrapped < 0) {
    wrapped += period;
  }
  return clamp_i64_to_i32(wrapped);
}

int32_t advance_csp_hold_target_counts(int32_t current_counts,
                                       int32_t desired_counts,
                                       int32_t max_step_counts,
                                       uint32_t wrap_period_counts) {
  if (wrap_period_counts == 0) {
    if (max_step_counts <= 0) {
      return desired_counts;
    }
    const int64_t cur = static_cast<int64_t>(current_counts);
    const int64_t desired = static_cast<int64_t>(desired_counts);
    const int64_t delta = desired - cur;
    const int64_t max_step = static_cast<int64_t>(max_step_counts);
    if (delta > max_step) {
      return clamp_i64_to_i32(cur + max_step);
    }
    if (delta < -max_step) {
      return clamp_i64_to_i32(cur - max_step);
    }
    return desired_counts;
  }

  const int32_t current_wrapped = wrap_counts_into_period(current_counts, wrap_period_counts);
  const int32_t desired_wrapped = wrap_counts_into_period(desired_counts, wrap_period_counts);
  if (max_step_counts <= 0) {
    return desired_wrapped;
  }
  const int64_t delta =
      shortest_periodic_error_counts(static_cast<int64_t>(desired_wrapped) -
                                         static_cast<int64_t>(current_wrapped),
                                     wrap_period_counts);
  const int64_t max_step = static_cast<int64_t>(max_step_counts);
  if (delta > max_step) {
    return wrap_counts_into_period(static_cast<int64_t>(current_wrapped) + max_step,
                                   wrap_period_counts);
  }
  if (delta < -max_step) {
    return wrap_counts_into_period(static_cast<int64_t>(current_wrapped) - max_step,
                                   wrap_period_counts);
  }
  return desired_wrapped;
}

struct PdoLayoutEntry {
  std::string semantic;
  uint16_t index = 0;
  uint8_t subindex = 0;
  uint8_t bits = 0;
};

enum class StartupSdoValueType : uint8_t {
  kNone = 0,
  kU16 = 1,
};

struct StartupSdoConfig {
  bool valid = false;
  std::string key;
  StartupSdoValueType type = StartupSdoValueType::kNone;
  uint16_t index = 0;
  uint8_t subindex = 0;
  std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> values{};
};

struct StartupSdoFeedback {
  std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> configured{};
  std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> commanded{};
  std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> readback_valid{};
  std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> readback{};
  std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> verified{};
};

constexpr uint32_t kMaxStartupSdoDescriptors = 8;

enum class SdoScalarType : uint8_t {
  kNone = 0,
  kU8 = 1,
  kI8 = 2,
  kU16 = 3,
  kI16 = 4,
  kU32 = 5,
  kI32 = 6,
};

struct SdoObjectSpec {
  bool valid = false;
  uint16_t index = 0;
  uint8_t subindex = 0;
  SdoScalarType type = SdoScalarType::kNone;
};

constexpr uint32_t kMaxAbsoluteFeedbackFields = 16;

struct AbsoluteFeedbackFieldConfig {
  bool valid = false;
  std::string key;
  SdoObjectSpec object{};
};

struct AbsoluteFeedbackConfig {
  bool valid = false;
  uint32_t field_count = 0;
  std::array<AbsoluteFeedbackFieldConfig, kMaxAbsoluteFeedbackFields> fields{};
};

enum class NativeHomeOpKind : uint8_t {
  kNone = 0,
  kSetMode = 1,
  kWriteSdo = 2,
  kControlwordSequence = 3,
  kWaitStatusword = 4,
  kRefreshTruth = 5,
  kRestoreMode = 6,
  kWaitSdo = 7,
  kReleaseServiceOverride = 8,
  kWriteSdoWrapFraction = 9,
};

struct NativeHomeOp {
  NativeHomeOpKind kind = NativeHomeOpKind::kNone;
  SdoObjectSpec object{};
  int32_t value_i32 = 0;
  uint32_t fraction_numerator = 0;
  uint32_t fraction_denominator = 0;
  std::array<uint16_t, 8> controlword_values{};
  uint32_t controlword_count = 0;
  uint16_t wait_all_set_mask = 0;
  uint16_t wait_all_clear_mask = 0;
};

struct NativeHomeConfig {
  bool valid = false;
  uint32_t steady_state_mode = 8;
  uint32_t commissioning_mode = 6;
  SdoObjectSpec truth_source{};
  std::vector<NativeHomeOp> transaction{};
};

struct AbsoluteFeedbackFieldSample {
  uint8_t valid = 0;
  int32_t value = 0;
};

struct AbsoluteFeedbackAxis {
  std::array<AbsoluteFeedbackFieldSample, kMaxAbsoluteFeedbackFields> fields{};
};

struct DrivePdoConfig {
  std::string label = "custom";
  uint16_t rx_pdo = 0;
  uint16_t tx_pdo = 0;
  uint8_t rx_sync_index = 2;
  uint8_t tx_sync_index = 3;
  std::vector<PdoLayoutEntry> rx_entries;
  std::vector<PdoLayoutEntry> tx_entries;
};

struct Options {
  std::string socket_path = "/run/gradient-rt-motion/ipc.sock";
  // Filesystem group that is allowed to connect to the IPC socket (0660).
  // Default matches the appliance user/group.
  std::string ipc_group = "pi";
  uint64_t cycle_ns = 1000000; // 1 kHz
  uint32_t num_axes = 6;       // default arm axes for early scaffolding
  uint32_t drive_profile_id = gradient::ipc::v1::DRIVE_PROFILE_UNKNOWN;
  uint32_t slave_vendor_id = 0;
  uint32_t slave_product_code = 0;
  uint32_t slave_revision_no = 0;
  uint8_t rx_sync_index = 2;
  uint8_t tx_sync_index = 3;
  uint64_t dc_cycle_multiple_ns = 0;

  // Axis scaling (bring-up defaults; tuned via commissioning).
  // Per-axis lists can be provided via the CLI (comma-separated).
  std::array<AxisConfig, gradient::ipc::v1::GRADIENT_MAX_AXES> axis{};
  std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_position{};
  uint32_t feedback_wrap_axis_mask = 0;
  // Per-axis bitmask controlling whether emitted CSP targets (0x607A) are
  // wrapped into the single-turn window. The sentinel value UINT32_MAX
  // means "mirror feedback_wrap_axis_mask"; any explicit value (including
  // 0) overrides the mirror. Back-compat: a deployment that only sets
  // --feedback-wrap-axis-mask retains the pre-2026-04-19 behavior where
  // command emission also wrapped.
  uint32_t command_wrap_axis_mask = std::numeric_limits<uint32_t>::max();

  // Safety: cap commanded motor speed (rpm). 0 disables clamping.
  double max_rpm = 100.0;

  // EtherCAT bring-up policy.
  uint16_t rx_pdo = 0;
  uint16_t tx_pdo = 0;
  bool use_dc = true;
  bool disable_output_watchdog = false;
  bool split_domains_per_axis = false;
  bool explicit_pdo_config = false;
  bool queue_split_domains_round_robin = false;
  uint32_t wait_before_safeop_ms = 250;
  uint32_t preop_to_safeop_timeout_ms = 5000;
  uint32_t safeop_to_op_timeout_ms = 5000;
  uint32_t startup_passive_ms = 0;
  uint32_t startup_skip_domain_queue_ms = 0;

  // Fast-trace writer: when enabled, a dedicated thread snapshots per-axis
  // feedback + last commanded 0x607A at a configurable Hz and writes a
  // compact JSONL file. Designed for post-mortem seam/whip analysis where
  // HTTP/SDO sampling is too coarse. Default disabled. See:
  //   docs of plan/j6_seam_whip_verification_b8c230f3
  // All three fields must be set for the thread to start.
  //   fast_trace_path: absolute path to the output JSONL. Typically under
  //     /run/gradient-rt-motion/ (tmpfs) to avoid actual disk IO.
  //   fast_trace_hz: target Hz. The thread sleeps with CLOCK_MONOTONIC /
  //     TIMER_ABSTIME and drops frames if the RT kernel delays it.
  //   fast_trace_axis_mask: bitmask of axis indexes to include per sample.
  //     0 means "all num_axes axes". The file is compact per-axis.
  std::string fast_trace_path;
  uint32_t fast_trace_hz = 0;
  uint32_t fast_trace_axis_mask = 0;
};

std::string format_pdo_profile_label(uint16_t rx_pdo, uint16_t tx_pdo) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "0x%04x/0x%04x",
                static_cast<unsigned int>(rx_pdo),
                static_cast<unsigned int>(tx_pdo));
  return std::string(buf);
}

bool parse_u32(const char* s, uint32_t* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  unsigned long v = std::strtoul(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0' || v > 0xFFFFFFFFUL) {
    return false;
  }
  *out = static_cast<uint32_t>(v);
  return true;
}

bool parse_env_flag(const char* name, bool default_value) {
  if (!name || !*name) {
    return default_value;
  }
  const char* raw = std::getenv(name);
  if (!raw || !*raw) {
    return default_value;
  }
  std::string value(raw);
  for (char& ch : value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  return default_value;
}

bool parse_u64(const char* s, uint64_t* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  unsigned long long v = std::strtoull(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0') {
    return false;
  }
  *out = static_cast<uint64_t>(v);
  return true;
}

bool parse_double(const char* s, double* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  double v = std::strtod(s, &end);
  if (errno != 0 || end == s || *end != '\0') {
    return false;
  }
  *out = v;
  return true;
}

bool parse_i32(const char* s, int* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  long v = std::strtol(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0') {
    return false;
  }
  *out = static_cast<int>(v);
  return true;
}

bool parse_u16_auto(const char* s, uint16_t* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  unsigned long v = std::strtoul(s, &end, 0);
  if (errno != 0 || end == s || *end != '\0' ||
      v > static_cast<unsigned long>(std::numeric_limits<uint16_t>::max())) {
    return false;
  }
  *out = static_cast<uint16_t>(v);
  return true;
}

bool parse_u32_auto(const char* s, uint32_t* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  unsigned long v = std::strtoul(s, &end, 0);
  if (errno != 0 || end == s || *end != '\0' ||
      v > static_cast<unsigned long>(std::numeric_limits<uint32_t>::max())) {
    return false;
  }
  *out = static_cast<uint32_t>(v);
  return true;
}

bool parse_i64_auto(const char* s, int64_t* out) {
  if (!s || !*s || !out) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  long long v = std::strtoll(s, &end, 0);
  if (errno != 0 || end == s || *end != '\0') {
    return false;
  }
  *out = static_cast<int64_t>(v);
  return true;
}

uint32_t drive_profile_token_hash(const std::string& token) {
  uint32_t value = 0x811C9DC5u;
  for (char c : token) {
    value ^= static_cast<uint8_t>(c);
    value *= 0x01000193u;
  }
  return value == 0 ? gradient::ipc::v1::DRIVE_PROFILE_UNKNOWN : value;
}

bool parse_drive_profile_token(const std::string& token, uint32_t* out) {
  if (!out) {
    return false;
  }
  std::string t = token;
  const size_t first = t.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return false;
  }
  const size_t last = t.find_last_not_of(" \t\r\n");
  t = t.substr(first, last - first + 1);
  if (t.empty()) {
    return false;
  }
  for (char& c : t) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  uint32_t numeric = 0;
  if (parse_u32_auto(t.c_str(), &numeric)) {
    *out = numeric;
    return true;
  }
  *out = drive_profile_token_hash(t);
  return true;
}

std::string trim_ascii_ws(const std::string& s) {
  const size_t first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const size_t last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::string ascii_lower_copy(std::string value) {
  for (char& c : value) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  return value;
}

std::vector<std::string> split_csv_strict(const std::string& spec) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= spec.size()) {
    size_t comma = spec.find(',', start);
    if (comma == std::string::npos) {
      comma = spec.size();
    }
    std::string tok = trim_ascii_ws(spec.substr(start, comma - start));
    if (tok.empty()) {
      return {}; // invalid (empty token)
    }
    out.push_back(tok);
    if (comma >= spec.size()) {
      break;
    }
    start = comma + 1;
  }
  return out;
}

std::vector<std::string> split_delim_strict(const std::string& spec, char delim) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= spec.size()) {
    size_t pos = spec.find(delim, start);
    if (pos == std::string::npos) {
      pos = spec.size();
    }
    std::string tok = trim_ascii_ws(spec.substr(start, pos - start));
    if (tok.empty()) {
      return {};
    }
    out.push_back(tok);
    if (pos >= spec.size()) {
      break;
    }
    start = pos + 1;
  }
  return out;
}

bool parse_u32_csv_allow_zero(const std::string& spec, std::vector<uint32_t>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    uint32_t v = 0;
    if (!parse_u32_auto(tok.c_str(), &v)) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

bool parse_pdo_layout_spec(const std::string& spec, std::vector<PdoLayoutEntry>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  if (trim_ascii_ws(spec).empty()) {
    return false;
  }
  const auto entries = split_delim_strict(spec, ';');
  if (entries.empty()) {
    return false;
  }
  for (const auto& entry_spec : entries) {
    const auto fields = split_delim_strict(entry_spec, '|');
    if (fields.size() != 4) {
      return false;
    }
    PdoLayoutEntry entry{};
    entry.semantic = trim_ascii_ws(fields[0]);
    uint16_t index = 0;
    uint16_t subindex = 0;
    uint32_t bits = 0;
    if (entry.semantic.empty() ||
        !parse_u16_auto(fields[1].c_str(), &index) ||
        !parse_u16_auto(fields[2].c_str(), &subindex) ||
        !parse_u32_auto(fields[3].c_str(), &bits) ||
        bits == 0 || bits > 255u) {
      return false;
    }
    entry.index = index;
    entry.subindex = static_cast<uint8_t>(subindex & 0xFFu);
    entry.bits = static_cast<uint8_t>(bits);
    out->push_back(entry);
  }
  return !out->empty();
}

bool parse_startup_sdo_config_spec(const std::string& spec,
                                   uint32_t num_axes,
                                   std::vector<StartupSdoConfig>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const std::string trimmed = trim_ascii_ws(spec);
  if (trimmed.empty()) {
    return true;
  }
  const auto descriptors = split_delim_strict(trimmed, ';');
  if (descriptors.empty() ||
      descriptors.size() > static_cast<size_t>(kMaxStartupSdoDescriptors)) {
    return false;
  }

  for (const auto& descriptor_spec : descriptors) {
    const auto fields = split_delim_strict(descriptor_spec, '|');
    if (fields.size() != 5) {
      return false;
    }
    StartupSdoConfig cfg{};
    cfg.key = trim_ascii_ws(fields[0]);
    std::string type = trim_ascii_ws(fields[1]);
    for (char& c : type) {
      if (c >= 'A' && c <= 'Z') {
        c = static_cast<char>(c - 'A' + 'a');
      }
    }
    if (cfg.key.empty() || type != "u16") {
      return false;
    }
    for (const auto& existing : *out) {
      if (existing.key == cfg.key) {
        return false;
      }
    }
    uint16_t index = 0;
    uint16_t subindex = 0;
    std::vector<uint32_t> values{};
    if (!parse_u16_auto(fields[2].c_str(), &index) ||
        !parse_u16_auto(fields[3].c_str(), &subindex) ||
        !parse_u32_csv_allow_zero(fields[4], &values)) {
      return false;
    }
    if (values.size() == 1) {
      for (uint32_t i = 0; i < num_axes; ++i) {
        cfg.values[i] = values[0];
      }
    } else if (values.size() == static_cast<size_t>(num_axes)) {
      for (uint32_t i = 0; i < num_axes; ++i) {
        cfg.values[i] = values[i];
      }
    } else {
      return false;
    }
    cfg.valid = true;
    cfg.type = StartupSdoValueType::kU16;
    cfg.index = index;
    cfg.subindex = static_cast<uint8_t>(subindex & 0xFFu);
    out->push_back(std::move(cfg));
  }
  return !out->empty();
}

bool parse_sdo_scalar_type_token(const std::string& token, SdoScalarType* out) {
  if (!out) {
    return false;
  }
  const std::string lowered = ascii_lower_copy(trim_ascii_ws(token));
  if (lowered == "u8") {
    *out = SdoScalarType::kU8;
    return true;
  }
  if (lowered == "i8") {
    *out = SdoScalarType::kI8;
    return true;
  }
  if (lowered == "u16") {
    *out = SdoScalarType::kU16;
    return true;
  }
  if (lowered == "i16") {
    *out = SdoScalarType::kI16;
    return true;
  }
  if (lowered == "u32") {
    *out = SdoScalarType::kU32;
    return true;
  }
  if (lowered == "i32") {
    *out = SdoScalarType::kI32;
    return true;
  }
  return false;
}

bool parse_absolute_feedback_config_spec(const std::string& spec,
                                        AbsoluteFeedbackConfig* out) {
  if (!out) {
    return false;
  }
  *out = AbsoluteFeedbackConfig{};
  const std::string trimmed = trim_ascii_ws(spec);
  if (trimmed.empty()) {
    return true;
  }
  const auto descriptors = split_delim_strict(trimmed, ';');
  if (descriptors.empty() ||
      descriptors.size() > static_cast<size_t>(kMaxAbsoluteFeedbackFields)) {
    return false;
  }
  AbsoluteFeedbackConfig cfg{};
  for (size_t idx = 0; idx < descriptors.size(); ++idx) {
    const auto fields = split_delim_strict(descriptors[idx], '|');
    if (fields.size() != 4) {
      return false;
    }
    const std::string key = trim_ascii_ws(fields[0]);
    uint16_t index = 0;
    uint16_t subindex = 0;
    SdoScalarType type = SdoScalarType::kNone;
    if (key.empty() ||
        !parse_u16_auto(fields[1].c_str(), &index) ||
        !parse_u16_auto(fields[2].c_str(), &subindex) ||
        !parse_sdo_scalar_type_token(fields[3], &type)) {
      return false;
    }
    for (size_t prev = 0; prev < idx; ++prev) {
      if (cfg.fields[prev].valid && cfg.fields[prev].key == key) {
        return false;
      }
    }
    cfg.fields[idx].valid = true;
    cfg.fields[idx].key = key;
    cfg.fields[idx].object.valid = true;
    cfg.fields[idx].object.index = index;
    cfg.fields[idx].object.subindex = static_cast<uint8_t>(subindex & 0xFFu);
    cfg.fields[idx].object.type = type;
  }
  cfg.field_count = static_cast<uint32_t>(descriptors.size());
  cfg.valid = cfg.field_count > 0;
  *out = cfg;
  return true;
}

bool parse_native_home_config_spec(const std::string& spec, NativeHomeConfig* out) {
  if (!out) {
    return false;
  }
  *out = NativeHomeConfig{};
  const std::string trimmed = trim_ascii_ws(spec);
  if (trimmed.empty()) {
    return true;
  }
  const auto descriptors = split_delim_strict(trimmed, ';');
  if (descriptors.empty()) {
    return false;
  }

  NativeHomeConfig cfg{};
  for (const auto& descriptor : descriptors) {
    const auto fields = split_delim_strict(descriptor, '|');
    if (fields.empty()) {
      return false;
    }
    const std::string entry_kind = ascii_lower_copy(trim_ascii_ws(fields[0]));
    if (entry_kind == "steady_state_mode") {
      uint32_t mode = 0;
      if (fields.size() != 2 || !parse_u32_auto(fields[1].c_str(), &mode)) {
        return false;
      }
      cfg.steady_state_mode = mode;
      continue;
    }
    if (entry_kind == "commissioning_mode") {
      uint32_t mode = 0;
      if (fields.size() != 2 || !parse_u32_auto(fields[1].c_str(), &mode)) {
        return false;
      }
      cfg.commissioning_mode = mode;
      continue;
    }
    if (entry_kind == "truth_source") {
      uint16_t index = 0;
      uint16_t subindex = 0;
      SdoScalarType type = SdoScalarType::kNone;
      if (fields.size() != 4 ||
          !parse_u16_auto(fields[1].c_str(), &index) ||
          !parse_u16_auto(fields[2].c_str(), &subindex) ||
          !parse_sdo_scalar_type_token(fields[3], &type)) {
        return false;
      }
      cfg.truth_source.valid = true;
      cfg.truth_source.index = index;
      cfg.truth_source.subindex = static_cast<uint8_t>(subindex & 0xFFu);
      cfg.truth_source.type = type;
      continue;
    }
    if (entry_kind != "op" || fields.size() < 2) {
      return false;
    }

    const std::string op_name = ascii_lower_copy(trim_ascii_ws(fields[1]));
    NativeHomeOp op{};
    if (op_name == "set_mode") {
      int64_t value = 0;
      if (fields.size() != 3 || !parse_i64_auto(fields[2].c_str(), &value)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kSetMode;
      op.value_i32 = static_cast<int32_t>(value);
    } else if (op_name == "restore_mode") {
      int64_t value = 0;
      if (fields.size() != 3 || !parse_i64_auto(fields[2].c_str(), &value)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kRestoreMode;
      op.value_i32 = static_cast<int32_t>(value);
    } else if (op_name == "write_sdo") {
      uint16_t index = 0;
      uint16_t subindex = 0;
      SdoScalarType type = SdoScalarType::kNone;
      int64_t value = 0;
      if (fields.size() != 6 ||
          !parse_u16_auto(fields[2].c_str(), &index) ||
          !parse_u16_auto(fields[3].c_str(), &subindex) ||
          !parse_sdo_scalar_type_token(fields[4], &type) ||
          !parse_i64_auto(fields[5].c_str(), &value)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kWriteSdo;
      op.object.valid = true;
      op.object.index = index;
      op.object.subindex = static_cast<uint8_t>(subindex & 0xFFu);
      op.object.type = type;
      op.value_i32 = static_cast<int32_t>(value);
    } else if (op_name == "write_sdo_wrap_fraction") {
      uint16_t index = 0;
      uint16_t subindex = 0;
      SdoScalarType type = SdoScalarType::kNone;
      uint32_t numerator = 0;
      uint32_t denominator = 0;
      if (fields.size() != 7 ||
          !parse_u16_auto(fields[2].c_str(), &index) ||
          !parse_u16_auto(fields[3].c_str(), &subindex) ||
          !parse_sdo_scalar_type_token(fields[4], &type) ||
          !parse_u32_auto(fields[5].c_str(), &numerator) ||
          !parse_u32_auto(fields[6].c_str(), &denominator) ||
          denominator == 0 ||
          numerator > denominator) {
        return false;
      }
      op.kind = NativeHomeOpKind::kWriteSdoWrapFraction;
      op.object.valid = true;
      op.object.index = index;
      op.object.subindex = static_cast<uint8_t>(subindex & 0xFFu);
      op.object.type = type;
      op.fraction_numerator = numerator;
      op.fraction_denominator = denominator;
    } else if (op_name == "controlword_sequence") {
      std::vector<uint32_t> values{};
      if (fields.size() != 3 || !parse_u32_csv_allow_zero(fields[2], &values) || values.empty() ||
          values.size() > op.controlword_values.size()) {
        return false;
      }
      op.kind = NativeHomeOpKind::kControlwordSequence;
      op.controlword_count = static_cast<uint32_t>(values.size());
      for (size_t i = 0; i < values.size(); ++i) {
        if (values[i] > static_cast<uint32_t>(std::numeric_limits<uint16_t>::max())) {
          return false;
        }
        op.controlword_values[i] = static_cast<uint16_t>(values[i] & 0xFFFFu);
      }
    } else if (op_name == "wait_statusword") {
      uint16_t all_set_mask = 0;
      uint16_t all_clear_mask = 0;
      if (fields.size() != 4 ||
          !parse_u16_auto(fields[2].c_str(), &all_set_mask) ||
          !parse_u16_auto(fields[3].c_str(), &all_clear_mask)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kWaitStatusword;
      op.wait_all_set_mask = all_set_mask;
      op.wait_all_clear_mask = all_clear_mask;
    } else if (op_name == "wait_sdo") {
      uint16_t index = 0;
      uint16_t subindex = 0;
      SdoScalarType type = SdoScalarType::kNone;
      int64_t value = 0;
      if (fields.size() != 6 ||
          !parse_u16_auto(fields[2].c_str(), &index) ||
          !parse_u16_auto(fields[3].c_str(), &subindex) ||
          !parse_sdo_scalar_type_token(fields[4], &type) ||
          !parse_i64_auto(fields[5].c_str(), &value)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kWaitSdo;
      op.object.valid = true;
      op.object.index = index;
      op.object.subindex = static_cast<uint8_t>(subindex & 0xFFu);
      op.object.type = type;
      op.value_i32 = static_cast<int32_t>(value);
    } else if (op_name == "release_service_override") {
      if (fields.size() != 2) {
        return false;
      }
      op.kind = NativeHomeOpKind::kReleaseServiceOverride;
    } else if (op_name == "refresh_truth") {
      if (fields.size() != 2) {
        return false;
      }
      op.kind = NativeHomeOpKind::kRefreshTruth;
    } else {
      return false;
    }
    cfg.transaction.push_back(op);
  }

  if (!cfg.truth_source.valid || cfg.transaction.empty()) {
    return false;
  }
  cfg.valid = true;
  *out = cfg;
  return true;
}

bool parse_axis_type_token(const std::string& token, uint8_t* out) {
  if (!out) {
    return false;
  }
  std::string t = token;
  for (char& c : t) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  if (t == "r" || t == "rot" || t == "rotary") {
    *out = gradient::ipc::v1::AXIS_TYPE_ROTARY;
    return true;
  }
  if (t == "l" || t == "lin" || t == "linear") {
    *out = gradient::ipc::v1::AXIS_TYPE_LINEAR;
    return true;
  }
  return false;
}

bool parse_u32_csv(const std::string& spec, std::vector<uint32_t>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    uint32_t v = 0;
    if (!parse_u32(tok.c_str(), &v) || v == 0) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

bool parse_double_csv(const std::string& spec, std::vector<double>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    double v = 0.0;
    if (!parse_double(tok.c_str(), &v)) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

bool parse_sign_csv(const std::string& spec, std::vector<int>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    int v = 0;
    if (!parse_i32(tok.c_str(), &v) || (v != 1 && v != -1)) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

bool parse_axis_type_csv(const std::string& spec, std::vector<uint8_t>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    uint8_t v = gradient::ipc::v1::AXIS_TYPE_UNKNOWN;
    if (!parse_axis_type_token(tok, &v)) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

template <typename T, typename Setter>
bool apply_per_axis(const std::vector<T>& vals, uint32_t num_axes, Setter setter) {
  if (vals.empty()) {
    return true;
  }
  if (vals.size() == 1) {
    for (uint32_t i = 0; i < num_axes; ++i) {
      setter(i, vals[0]);
    }
    return true;
  }
  if (vals.size() != static_cast<size_t>(num_axes)) {
    return false;
  }
  for (uint32_t i = 0; i < num_axes; ++i) {
    setter(i, vals[i]);
  }
  return true;
}

bool finalize_axis_config(Options* opt,
                          const std::string& counts_per_rev_spec,
                          const std::string& gear_ratio_spec,
                          const std::string& sign_spec,
                          const std::string& axis_type_spec,
                          const std::string& lead_m_per_rev_spec) {
  if (!opt) {
    return false;
  }

  // Parse optional per-axis specs (comma-separated). A single value broadcasts to all axes.
  std::vector<uint32_t> cpr{};
  if (!counts_per_rev_spec.empty()) {
    if (!parse_u32_csv(counts_per_rev_spec, &cpr)) {
      logf("ERROR: invalid --counts-per-rev (expected N or N1,N2,...)");
      return false;
    }
    if (!apply_per_axis<uint32_t>(cpr, opt->num_axes, [&](uint32_t i, uint32_t v) {
          opt->axis[i].counts_per_rev = v;
        })) {
      logf("ERROR: --counts-per-rev list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  std::vector<double> gr{};
  if (!gear_ratio_spec.empty()) {
    if (!parse_double_csv(gear_ratio_spec, &gr)) {
      logf("ERROR: invalid --gear-ratio (expected R or R1,R2,...)");
      return false;
    }
    if (!apply_per_axis<double>(gr, opt->num_axes, [&](uint32_t i, double v) {
          if (v <= 0.0) {
            v = 0.0;
          }
          opt->axis[i].gear_ratio = v;
        })) {
      logf("ERROR: --gear-ratio list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  std::vector<int> sgn{};
  if (!sign_spec.empty()) {
    if (!parse_sign_csv(sign_spec, &sgn)) {
      logf("ERROR: invalid --sign (expected +1/-1 or S1,S2,...)");
      return false;
    }
    if (!apply_per_axis<int>(sgn, opt->num_axes, [&](uint32_t i, int v) {
          opt->axis[i].sign = v;
        })) {
      logf("ERROR: --sign list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  std::vector<uint8_t> at{};
  if (!axis_type_spec.empty()) {
    if (!parse_axis_type_csv(axis_type_spec, &at)) {
      logf("ERROR: invalid --axis-type (expected rotary|linear, optionally comma-separated)");
      return false;
    }
    if (!apply_per_axis<uint8_t>(at, opt->num_axes, [&](uint32_t i, uint8_t v) {
          opt->axis[i].axis_type = v;
        })) {
      logf("ERROR: --axis-type list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  std::vector<double> lead{};
  if (!lead_m_per_rev_spec.empty()) {
    if (!parse_double_csv(lead_m_per_rev_spec, &lead)) {
      logf("ERROR: invalid --lead-m-per-rev (expected M or M1,M2,...)");
      return false;
    }
    if (!apply_per_axis<double>(lead, opt->num_axes, [&](uint32_t i, double v) {
          opt->axis[i].lead_m_per_rev = v;
        })) {
      logf("ERROR: --lead-m-per-rev list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  // Validate and derive counts_per_unit and safety clamps.
  constexpr double kTwoPi = 6.28318530717958647692;
  const double period_s = static_cast<double>(opt->cycle_ns) / 1e9;
  for (uint32_t i = 0; i < opt->num_axes; ++i) {
    AxisConfig& a = opt->axis[i];
    if (a.counts_per_rev == 0) {
      logf("ERROR: axis%u counts_per_rev must be > 0", i);
      return false;
    }
    if (a.gear_ratio <= 0.0) {
      logf("ERROR: axis%u gear_ratio must be > 0", i);
      return false;
    }
    if (a.sign != 1 && a.sign != -1) {
      logf("ERROR: axis%u sign must be +1 or -1", i);
      return false;
    }

    if (a.axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY) {
      a.counts_per_unit =
          (static_cast<double>(a.counts_per_rev) * a.gear_ratio) / kTwoPi; // counts/rad
    } else if (a.axis_type == gradient::ipc::v1::AXIS_TYPE_LINEAR) {
      if (a.lead_m_per_rev <= 0.0) {
        logf("ERROR: axis%u is linear; provide --lead-m-per-rev (m/rev) for that axis", i);
        return false;
      }
      a.counts_per_unit =
          (static_cast<double>(a.counts_per_rev) * a.gear_ratio) / a.lead_m_per_rev; // counts/m
    } else {
      logf("ERROR: axis%u has unknown axis_type (use --axis-type rotary|linear)", i);
      return false;
    }
    if (!(a.counts_per_unit > 0.0)) {
      logf("ERROR: axis%u counts_per_unit invalid (check scaling inputs)", i);
      return false;
    }

    // Safety clamp derived from motor rpm limit.
    if (opt->max_rpm <= 0.0) {
      a.max_step_counts_per_cycle = 0;
      a.max_profile_vel_counts_per_s = std::numeric_limits<uint32_t>::max();
    } else {
      const double max_counts_per_s =
          (opt->max_rpm / 60.0) * static_cast<double>(a.counts_per_rev);
      const long long step = std::llround(max_counts_per_s * period_s);
      if (step <= 0) {
        a.max_step_counts_per_cycle = 1;
      } else if (step > static_cast<long long>(std::numeric_limits<int32_t>::max())) {
        a.max_step_counts_per_cycle = std::numeric_limits<int32_t>::max();
      } else {
        a.max_step_counts_per_cycle = static_cast<int32_t>(step);
      }

      const long long v = std::llround(max_counts_per_s);
      if (v <= 0) {
        a.max_profile_vel_counts_per_s = 1u;
      } else if (v > static_cast<long long>(std::numeric_limits<uint32_t>::max())) {
        a.max_profile_vel_counts_per_s = std::numeric_limits<uint32_t>::max();
      } else {
        a.max_profile_vel_counts_per_s = static_cast<uint32_t>(v);
      }
    }
  }

  // Populate unused axis slots with sane derived values for tooling.
  for (uint32_t i = opt->num_axes; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
    AxisConfig& a = opt->axis[i];
    if (a.axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY) {
      a.counts_per_unit =
          (static_cast<double>(a.counts_per_rev) * a.gear_ratio) / kTwoPi; // counts/rad
    } else {
      // Keep defaults for unused axes.
      a.counts_per_unit =
          (static_cast<double>(a.counts_per_rev) * a.gear_ratio) / kTwoPi; // counts/rad
    }
    if (opt->max_rpm <= 0.0) {
      a.max_step_counts_per_cycle = 0;
      a.max_profile_vel_counts_per_s = std::numeric_limits<uint32_t>::max();
    } else {
      const double max_counts_per_s =
          (opt->max_rpm / 60.0) * static_cast<double>(a.counts_per_rev);
      const long long step = std::llround(max_counts_per_s * period_s);
      if (step <= 0) {
        a.max_step_counts_per_cycle = 1;
      } else if (step > static_cast<long long>(std::numeric_limits<int32_t>::max())) {
        a.max_step_counts_per_cycle = std::numeric_limits<int32_t>::max();
      } else {
        a.max_step_counts_per_cycle = static_cast<int32_t>(step);
      }

      const long long v = std::llround(max_counts_per_s);
      if (v <= 0) {
        a.max_profile_vel_counts_per_s = 1u;
      } else if (v > static_cast<long long>(std::numeric_limits<uint32_t>::max())) {
        a.max_profile_vel_counts_per_s = std::numeric_limits<uint32_t>::max();
      } else {
        a.max_profile_vel_counts_per_s = static_cast<uint32_t>(v);
      }
    }
  }

  return true;
}

bool finalize_slave_positions(Options* opt, const std::string& slave_positions_spec) {
  if (!opt) {
    return false;
  }

  for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
    opt->slave_position[i] = static_cast<uint16_t>(i);
  }

  if (slave_positions_spec.empty()) {
    return true;
  }

  const auto toks = split_csv_strict(slave_positions_spec);
  if (toks.size() != static_cast<size_t>(opt->num_axes)) {
    logf("ERROR: --slave-positions list length must match --num-axes (%u)", opt->num_axes);
    return false;
  }

  for (uint32_t i = 0; i < opt->num_axes; ++i) {
    uint32_t pos = 0;
    if (!parse_u32(toks[i].c_str(), &pos) ||
        pos > static_cast<uint32_t>(std::numeric_limits<uint16_t>::max())) {
      logf("ERROR: invalid slave position '%s' in --slave-positions", toks[i].c_str());
      return false;
    }
    opt->slave_position[i] = static_cast<uint16_t>(pos);
  }

  for (uint32_t i = 0; i < opt->num_axes; ++i) {
    for (uint32_t j = i + 1; j < opt->num_axes; ++j) {
      if (opt->slave_position[i] == opt->slave_position[j]) {
        logf("ERROR: duplicate slave position %u in --slave-positions",
             static_cast<unsigned int>(opt->slave_position[i]));
        return false;
      }
    }
  }

  return true;
}

void print_usage(const char* argv0) {
  std::fprintf(
      stderr,
      "Usage: %s [--socket-path PATH] [--cycle-ns NS] [--num-axes N] "
      "[--counts-per-rev N[,N..]] [--gear-ratio R[,R..]] [--sign S[,S..]] "
      "[--feedback-wrap-axis-mask MASK] [--command-wrap-axis-mask MASK] "
      "[--axis-type T[,T..]] [--lead-m-per-rev M[,M..]] [--drive-profile ID] [--max-rpm RPM] "
      "[--startup-sdo-config SPEC] "
      "[--absolute-feedback-config SPEC] "
      "[--native-home-config SPEC] "
      "[--slave-vendor-id VID] [--slave-product-code PID] [--slave-revision-no REV] "
      "[--rx-sync-index N] [--tx-sync-index N] [--dc-cycle-multiple-ns NS] "
      "[--slave-positions P[,P..]] "
      "[--rx-pdo ID] [--tx-pdo ID] [--rx-pdo-layout SPEC] [--tx-pdo-layout SPEC] "
      "[--no-dc] [--disable-output-watchdog] "
      "[--split-domains-per-axis] [--queue-split-domains-round-robin] [--explicit-pdo-config] "
      "[--wait-before-safeop-ms MS] [--preop-safeop-timeout-ms MS] "
      "[--safeop-op-timeout-ms MS] [--startup-passive-ms MS] "
      "[--startup-skip-domain-queue-ms MS] [--ipc-group NAME] "
      "[--fast-trace-path PATH] [--fast-trace-hz HZ] [--fast-trace-axis-mask MASK]\n\n"
      "Defaults:\n"
      "  --socket-path /run/gradient-rt-motion/ipc.sock\n"
      "  --cycle-ns     1000000\n"
      "  --num-axes     6\n"
      "  --counts-per-rev 131072\n"
      "  --gear-ratio   1.0\n"
      "  --sign         +1\n"
      "  --feedback-wrap-axis-mask 0x0\n"
      "  --command-wrap-axis-mask  (unset -> mirror feedback mask)\n"
      "  --axis-type    rotary\n"
      "  --drive-profile <profile-or-id>\n"
      "  --max-rpm      100\n"
      "  --startup-sdo-config key|u16|0xINDEX|0xSUB|V[,V..][;key|u16|0xINDEX|0xSUB|V[,V..]...]\n"
      "  --absolute-feedback-config key|0xINDEX|0xSUB|TYPE;...\n"
      "  --native-home-config descriptor string for commissioning-only native-home transactions\n"
      "  --slave-vendor-id  0x....\n"
      "  --slave-product-code 0x....\n"
      "  --slave-revision-no 0x....\n"
      "  --rx-sync-index 2\n"
      "  --tx-sync-index 3\n"
      "  --dc-cycle-multiple-ns 0\n"
      "  --slave-positions 0,1,2,...\n"
      "  --rx-pdo       0x....\n"
      "  --tx-pdo       0x....\n"
      "  --rx-pdo-layout semantic|0xINDEX|0xSUB|BITS;...\n"
      "  --tx-pdo-layout semantic|0xINDEX|0xSUB|BITS;...\n"
      "  --no-dc        disabled by default\n"
      "  --disable-output-watchdog off\n"
      "  --split-domains-per-axis off\n"
      "  --queue-split-domains-round-robin off\n"
      "  --explicit-pdo-config off\n"
      "  --wait-before-safeop-ms 250\n"
      "  --preop-safeop-timeout-ms 5000\n"
      "  --safeop-op-timeout-ms 5000\n"
      "  --startup-passive-ms 0\n"
      "  --startup-skip-domain-queue-ms 0\n"
      "  --ipc-group    pi\n"
      "  --fast-trace-path    (empty -> disabled)\n"
      "  --fast-trace-hz      0 (0 -> disabled)\n"
      "  --fast-trace-axis-mask 0 (0 -> all num_axes axes)\n",
      argv0);
}

struct ShmRegion {
  int fd = -1;
  void* base = nullptr;
  size_t bytes = 0;

  ShmRegion() = default;
  ShmRegion(const ShmRegion&) = delete;
  ShmRegion& operator=(const ShmRegion&) = delete;

  ShmRegion(ShmRegion&& other) noexcept
      : fd(other.fd), base(other.base), bytes(other.bytes) {
    other.fd = -1;
    other.base = nullptr;
    other.bytes = 0;
  }

  ShmRegion& operator=(ShmRegion&& other) noexcept {
    if (this == &other) {
      return *this;
    }
    reset();
    fd = other.fd;
    base = other.base;
    bytes = other.bytes;
    other.fd = -1;
    other.base = nullptr;
    other.bytes = 0;
    return *this;
  }

  void reset() {
    if (base && bytes) {
      munmap(base, bytes);
    }
    if (fd >= 0) {
      close(fd);
    }
    fd = -1;
    base = nullptr;
    bytes = 0;
  }

  ~ShmRegion() { reset(); }
};

int set_cloexec(int fd) {
  int flags = fcntl(fd, F_GETFD);
  if (flags < 0) {
    return -1;
  }
  return fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
}

ShmRegion create_memfd_region(const char* name, size_t bytes) {
  ShmRegion region;

  int fd = -1;
#ifdef __linux__
  // memfd_create() is declared when _GNU_SOURCE is enabled.
  fd = memfd_create(name, MFD_CLOEXEC);
#endif
  if (fd < 0) {
    logf("ERROR: memfd_create('%s') failed: %s", name, std::strerror(errno));
    return region;
  }

  if (ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
    logf("ERROR: ftruncate(memfd:%s, %zu) failed: %s", name, bytes,
         std::strerror(errno));
    close(fd);
    return region;
  }

  void* base =
      mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (base == MAP_FAILED) {
    logf("ERROR: mmap(memfd:%s, %zu) failed: %s", name, bytes,
         std::strerror(errno));
    close(fd);
    return region;
  }

  region.fd = fd;
  region.base = base;
  region.bytes = bytes;
  return region;
}

bool eventfd_write_one(int efd) {
  uint64_t one = 1;
  ssize_t n = write(efd, &one, sizeof(one));
  if (n == static_cast<ssize_t>(sizeof(one))) {
    return true;
  }
  if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
    return false;
  }
  return false;
}

bool eventfd_drain(int efd) {
  uint64_t value = 0;
  while (true) {
    ssize_t n = read(efd, &value, sizeof(value));
    if (n == static_cast<ssize_t>(sizeof(value))) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
      return true;
    }
    return (n == 0);
  }
}

struct RingView {
  gradient::ipc::v1::RingHeaderV1* header = nullptr;
  uint8_t* entries = nullptr;
  uint32_t capacity = 0;
  uint32_t msg_bytes = 0;
};

RingView make_ring_view(void* shm_base, const gradient::ipc::v1::ShmHeaderV1* hdr) {
  RingView view;
  if (!shm_base || !hdr) {
    return view;
  }
  auto* base = static_cast<uint8_t*>(shm_base);
  auto* ring_hdr =
      reinterpret_cast<gradient::ipc::v1::RingHeaderV1*>(base + hdr->ring_offset);

  const size_t ring_hdr_bytes_aligned = align_up(sizeof(*ring_hdr), 8);
  view.header = ring_hdr;
  view.entries = base + hdr->ring_offset + ring_hdr_bytes_aligned;
  view.capacity = hdr->ring_capacity;
  view.msg_bytes = hdr->ring_msg_bytes;
  return view;
}

bool ring_write(RingView ring,
                uint16_t type,
                const void* payload,
                size_t payload_bytes,
                uint64_t* seq_counter,
                uint64_t time_ns) {
  if (!ring.header || !ring.entries || ring.capacity == 0 || ring.msg_bytes == 0) {
    return false;
  }
  // SPSC ring: producer owns write_idx, consumer owns read_idx.
  const uint32_t w = ring.header->write_idx;
  const uint32_t r = ring.header->read_idx;
  if ((w - r) >= ring.capacity) {
    ring.header->dropped += 1;
    return false;
  }

  const uint32_t slot = w % ring.capacity;
  uint8_t* slot_ptr = ring.entries + static_cast<size_t>(slot) * ring.msg_bytes;
  std::memset(slot_ptr, 0, ring.msg_bytes);

  auto* mh = reinterpret_cast<gradient::ipc::v1::MsgHeader*>(slot_ptr);
  mh->type = type;
  mh->flags = 0;
  mh->bytes = static_cast<uint32_t>(sizeof(*mh) + payload_bytes);
  mh->seq = (*seq_counter)++;
  mh->time_ns = time_ns;

  if (payload && payload_bytes > 0) {
    std::memcpy(slot_ptr + sizeof(*mh), payload, payload_bytes);
  }

  // Publish.
  ring.header->write_idx = w + 1;
  return true;
}

} // namespace

int main(int argc, char** argv) {
  Options opt;
  std::string counts_per_rev_spec;
  std::string gear_ratio_spec;
  std::string sign_spec;
  std::string axis_type_spec;
  std::string lead_m_per_rev_spec;
  std::string slave_positions_spec;
  std::string rx_pdo_layout_spec;
  std::string tx_pdo_layout_spec;
  std::string startup_sdo_config_spec;
  std::string absolute_feedback_config_spec;
  std::string native_home_config_spec;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
    if (arg == "--socket-path" && i + 1 < argc) {
      opt.socket_path = argv[++i];
      continue;
    }
    if (arg == "--cycle-ns" && i + 1 < argc) {
      if (!parse_u64(argv[++i], &opt.cycle_ns) || opt.cycle_ns == 0) {
        logf("ERROR: invalid --cycle-ns");
        return 2;
      }
      continue;
    }
    if (arg == "--num-axes" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.num_axes) ||
          opt.num_axes == 0 || opt.num_axes > gradient::ipc::v1::GRADIENT_MAX_AXES) {
        logf("ERROR: invalid --num-axes");
        return 2;
      }
      continue;
    }
    if (arg == "--counts-per-rev" && i + 1 < argc) {
      counts_per_rev_spec = argv[++i];
      continue;
    }
    if (arg == "--gear-ratio" && i + 1 < argc) {
      gear_ratio_spec = argv[++i];
      continue;
    }
    if (arg == "--sign" && i + 1 < argc) {
      sign_spec = argv[++i];
      continue;
    }
    if (arg == "--feedback-wrap-axis-mask" && i + 1 < argc) {
      if (!parse_u32_auto(argv[++i], &opt.feedback_wrap_axis_mask)) {
        logf("ERROR: invalid --feedback-wrap-axis-mask");
        return 2;
      }
      continue;
    }
    if (arg == "--command-wrap-axis-mask" && i + 1 < argc) {
      // Explicit 0 means "do not wrap 0x607A on any axis" (continuous
      // command emission). Omitting this flag leaves the mask at its
      // sentinel default which mirrors --feedback-wrap-axis-mask for
      // back-compat with older configs.
      if (!parse_u32_auto(argv[++i], &opt.command_wrap_axis_mask)) {
        logf("ERROR: invalid --command-wrap-axis-mask");
        return 2;
      }
      continue;
    }
    if (arg == "--axis-type" && i + 1 < argc) {
      axis_type_spec = argv[++i];
      continue;
    }
    if (arg == "--lead-m-per-rev" && i + 1 < argc) {
      lead_m_per_rev_spec = argv[++i];
      continue;
    }
    if (arg == "--drive-profile" && i + 1 < argc) {
      if (!parse_drive_profile_token(argv[++i], &opt.drive_profile_id)) {
        logf("ERROR: invalid --drive-profile");
        return 2;
      }
      continue;
    }
    if (arg == "--max-rpm" && i + 1 < argc) {
      if (!parse_double(argv[++i], &opt.max_rpm) || opt.max_rpm < 0.0) {
        logf("ERROR: invalid --max-rpm (must be >= 0)");
        return 2;
      }
      continue;
    }
    if (arg == "--fast-trace-path" && i + 1 < argc) {
      opt.fast_trace_path = argv[++i];
      continue;
    }
    if (arg == "--fast-trace-hz" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.fast_trace_hz)) {
        logf("ERROR: invalid --fast-trace-hz");
        return 2;
      }
      continue;
    }
    if (arg == "--fast-trace-axis-mask" && i + 1 < argc) {
      if (!parse_u32_auto(argv[++i], &opt.fast_trace_axis_mask)) {
        logf("ERROR: invalid --fast-trace-axis-mask");
        return 2;
      }
      continue;
    }
    if (arg == "--startup-sdo-config" && i + 1 < argc) {
      startup_sdo_config_spec = argv[++i];
      continue;
    }
    if (arg == "--absolute-feedback-config" && i + 1 < argc) {
      absolute_feedback_config_spec = argv[++i];
      continue;
    }
    if (arg == "--native-home-config" && i + 1 < argc) {
      native_home_config_spec = argv[++i];
      continue;
    }
    if (arg == "--slave-vendor-id" && i + 1 < argc) {
      if (!parse_u32_auto(argv[++i], &opt.slave_vendor_id)) {
        logf("ERROR: invalid --slave-vendor-id");
        return 2;
      }
      continue;
    }
    if (arg == "--slave-product-code" && i + 1 < argc) {
      if (!parse_u32_auto(argv[++i], &opt.slave_product_code)) {
        logf("ERROR: invalid --slave-product-code");
        return 2;
      }
      continue;
    }
    if (arg == "--slave-revision-no" && i + 1 < argc) {
      if (!parse_u32_auto(argv[++i], &opt.slave_revision_no)) {
        logf("ERROR: invalid --slave-revision-no");
        return 2;
      }
      continue;
    }
    if (arg == "--rx-sync-index" && i + 1 < argc) {
      uint32_t value = 0;
      if (!parse_u32_auto(argv[++i], &value) || value > 255u) {
        logf("ERROR: invalid --rx-sync-index");
        return 2;
      }
      opt.rx_sync_index = static_cast<uint8_t>(value);
      continue;
    }
    if (arg == "--tx-sync-index" && i + 1 < argc) {
      uint32_t value = 0;
      if (!parse_u32_auto(argv[++i], &value) || value > 255u) {
        logf("ERROR: invalid --tx-sync-index");
        return 2;
      }
      opt.tx_sync_index = static_cast<uint8_t>(value);
      continue;
    }
    if (arg == "--dc-cycle-multiple-ns" && i + 1 < argc) {
      if (!parse_u64(argv[++i], &opt.dc_cycle_multiple_ns)) {
        logf("ERROR: invalid --dc-cycle-multiple-ns");
        return 2;
      }
      continue;
    }
    if (arg == "--slave-positions" && i + 1 < argc) {
      slave_positions_spec = argv[++i];
      continue;
    }
    if (arg == "--rx-pdo" && i + 1 < argc) {
      if (!parse_u16_auto(argv[++i], &opt.rx_pdo)) {
        logf("ERROR: invalid --rx-pdo");
        return 2;
      }
      continue;
    }
    if (arg == "--tx-pdo" && i + 1 < argc) {
      if (!parse_u16_auto(argv[++i], &opt.tx_pdo)) {
        logf("ERROR: invalid --tx-pdo");
        return 2;
      }
      continue;
    }
    if (arg == "--rx-pdo-layout" && i + 1 < argc) {
      rx_pdo_layout_spec = argv[++i];
      continue;
    }
    if (arg == "--tx-pdo-layout" && i + 1 < argc) {
      tx_pdo_layout_spec = argv[++i];
      continue;
    }
    if (arg == "--no-dc") {
      opt.use_dc = false;
      continue;
    }
    if (arg == "--disable-output-watchdog") {
      opt.disable_output_watchdog = true;
      continue;
    }
    if (arg == "--split-domains-per-axis") {
      opt.split_domains_per_axis = true;
      continue;
    }
    if (arg == "--queue-split-domains-round-robin") {
      opt.queue_split_domains_round_robin = true;
      continue;
    }
    if (arg == "--explicit-pdo-config") {
      opt.explicit_pdo_config = true;
      continue;
    }
    if (arg == "--wait-before-safeop-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.wait_before_safeop_ms)) {
        logf("ERROR: invalid --wait-before-safeop-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--preop-safeop-timeout-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.preop_to_safeop_timeout_ms)) {
        logf("ERROR: invalid --preop-safeop-timeout-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--safeop-op-timeout-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.safeop_to_op_timeout_ms)) {
        logf("ERROR: invalid --safeop-op-timeout-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--startup-passive-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.startup_passive_ms)) {
        logf("ERROR: invalid --startup-passive-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--startup-skip-domain-queue-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.startup_skip_domain_queue_ms)) {
        logf("ERROR: invalid --startup-skip-domain-queue-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--ipc-group" && i + 1 < argc) {
      opt.ipc_group = argv[++i];
      continue;
    }

    logf("ERROR: unknown arg: %s", arg.c_str());
    print_usage(argv[0]);
    return 2;
  }

  if (!finalize_axis_config(&opt,
                            counts_per_rev_spec,
                            gear_ratio_spec,
                            sign_spec,
                            axis_type_spec,
                            lead_m_per_rev_spec)) {
    return 2;
  }
  // Command-wrap sentinel: when the CLI did not explicitly set
  // --command-wrap-axis-mask, mirror the feedback mask. This preserves
  // pre-2026-04-19 behavior for older configs while letting profiles
  // that have validated continuous 607A on live hardware opt into
  // unwrapped command emission per-axis.
  const uint32_t effective_command_wrap_axis_mask =
      (opt.command_wrap_axis_mask == std::numeric_limits<uint32_t>::max())
          ? opt.feedback_wrap_axis_mask
          : opt.command_wrap_axis_mask;
  for (uint32_t axis_i = 0; axis_i < opt.num_axes; ++axis_i) {
    opt.axis[axis_i].feedback_counts_wrap =
        (opt.feedback_wrap_axis_mask & (1u << axis_i)) != 0u;
    opt.axis[axis_i].command_counts_wrap =
        (effective_command_wrap_axis_mask & (1u << axis_i)) != 0u;
  }
  if (!finalize_slave_positions(&opt, slave_positions_spec)) {
    return 2;
  }
  DrivePdoConfig drive_pdo{};
  drive_pdo.rx_pdo = opt.rx_pdo;
  drive_pdo.tx_pdo = opt.tx_pdo;
  drive_pdo.rx_sync_index = opt.rx_sync_index;
  drive_pdo.tx_sync_index = opt.tx_sync_index;
  drive_pdo.label = format_pdo_profile_label(opt.rx_pdo, opt.tx_pdo);
  if (opt.rx_pdo == 0 || opt.tx_pdo == 0 ||
      !parse_pdo_layout_spec(rx_pdo_layout_spec, &drive_pdo.rx_entries) ||
      !parse_pdo_layout_spec(tx_pdo_layout_spec, &drive_pdo.tx_entries)) {
    logf("ERROR: invalid drive PDO loader config "
         "(need non-zero --rx-pdo/--tx-pdo and valid --rx-pdo-layout/--tx-pdo-layout)");
    return 2;
  }
  std::vector<StartupSdoConfig> startup_sdos{};
  if (!parse_startup_sdo_config_spec(startup_sdo_config_spec, opt.num_axes, &startup_sdos)) {
    logf("ERROR: invalid --startup-sdo-config");
    return 2;
  }
  AbsoluteFeedbackConfig absolute_feedback_cfg{};
  if (!parse_absolute_feedback_config_spec(absolute_feedback_config_spec, &absolute_feedback_cfg)) {
    logf("ERROR: invalid --absolute-feedback-config");
    return 2;
  }
  NativeHomeConfig native_home_cfg{};
  if (!parse_native_home_config_spec(native_home_config_spec, &native_home_cfg)) {
    logf("ERROR: invalid --native-home-config");
    return 2;
  }
  const bool metrics_startup_readback_enabled =
      parse_env_flag("GRADIENT_RT_METRICS_STARTUP_READBACK_ENABLED", true);
  const bool metrics_native_home_refresh_enabled =
      parse_env_flag("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", true);
  const bool metrics_absolute_feedback_poll_enabled =
      parse_env_flag("GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED", true);
  if (opt.slave_vendor_id == 0 || opt.slave_product_code == 0) {
    logf("ERROR: EtherCAT slave identity is incomplete "
         "(provide --slave-vendor-id and --slave-product-code)");
    return 2;
  }

#if GRADIENT_HAVE_ECRT
  if (opt.use_dc && opt.dc_cycle_multiple_ns > 0 &&
      (opt.cycle_ns % opt.dc_cycle_multiple_ns) != 0) {
    logf("ERROR: --cycle-ns (%llu) must be a multiple of configured drive DC quantum %llu ns",
         static_cast<unsigned long long>(opt.cycle_ns),
         static_cast<unsigned long long>(opt.dc_cycle_multiple_ns));
    return 2;
  }
#endif

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  // Lock memory to avoid page faults in the RT loop.
  // In production, systemd sets LimitMEMLOCK=infinity.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    logf("WARNING: mlockall(MCL_CURRENT|MCL_FUTURE) failed: %s", std::strerror(errno));
  }

  // Create parent directory for the socket.
  try {
    std::filesystem::path sock_path(opt.socket_path);
    std::filesystem::path parent = sock_path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
  } catch (const std::exception& e) {
    logf("ERROR: failed to create socket directory for %s: %s",
         opt.socket_path.c_str(), e.what());
    return 1;
  }

  // Create server socket.
  int server_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (server_fd < 0) {
    logf("ERROR: socket(AF_UNIX, SOCK_SEQPACKET) failed: %s", std::strerror(errno));
    return 1;
  }
  set_cloexec(server_fd);

  // Bind path (replace any stale socket file).
  unlink(opt.socket_path.c_str());

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (opt.socket_path.size() >= sizeof(addr.sun_path)) {
    logf("ERROR: socket path too long: %s", opt.socket_path.c_str());
    close(server_fd);
    return 1;
  }
  std::strncpy(addr.sun_path, opt.socket_path.c_str(), sizeof(addr.sun_path) - 1);

  if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    logf("ERROR: bind(%s) failed: %s", opt.socket_path.c_str(), std::strerror(errno));
    close(server_fd);
    return 1;
  }

  // Best-effort permissions for the IPC socket:
  // - chmod 0660 so only owner/group can connect
  // - chown the socket's group to `--ipc-group` (default: `pi`) so the Python
  //   controller (runs unprivileged) can connect.
  gid_t ipc_gid = static_cast<gid_t>(-1);
  try {
    gid_t desired_gid = static_cast<gid_t>(-1);

    if (!opt.ipc_group.empty()) {
      if (group* g = ::getgrnam(opt.ipc_group.c_str())) {
        desired_gid = g->gr_gid;
      } else {
        logf("WARNING: --ipc-group '%s' not found; falling back to parent directory group",
             opt.ipc_group.c_str());
      }
    }

    std::filesystem::path sock_path(opt.socket_path);
    std::filesystem::path parent = sock_path.parent_path();
    if (!parent.empty()) {
      struct stat st {};
      if (::stat(parent.c_str(), &st) == 0) {
        if (desired_gid == static_cast<gid_t>(-1)) {
          desired_gid = st.st_gid;
        }
      }
    }

    // Keep uid, update gid.
    if (desired_gid != static_cast<gid_t>(-1) &&
        ::chown(opt.socket_path.c_str(), static_cast<uid_t>(-1), desired_gid) != 0) {
      logf("WARNING: chown(%s) failed: %s", opt.socket_path.c_str(), std::strerror(errno));
    }
    ipc_gid = desired_gid;
  } catch (const std::exception& e) {
    logf("WARNING: failed to adjust socket ownership for %s: %s", opt.socket_path.c_str(), e.what());
  }
  chmod(opt.socket_path.c_str(), 0660);

  if (listen(server_fd, 4) != 0) {
    logf("ERROR: listen() failed: %s", std::strerror(errno));
    unlink(opt.socket_path.c_str());
    close(server_fd);
    return 1;
  }

  logf("Listening on %s", opt.socket_path.c_str());
#if !GRADIENT_HAVE_ECRT
  logf("NOTE: IgH libecrt headers not found; running IPC-only mode.");
#endif

  // RT scaffolding: threads exist even before EtherCAT loop is implemented.
  std::atomic<uint64_t> rt_cycle_counter{0};
  std::atomic<int64_t> rt_last_jitter_ns{0};
  std::atomic<int64_t> rt_max_abs_jitter_ns{0};
  std::atomic<uint64_t> rt_overrun_count{0};

  constexpr uint32_t kMaxTrajectoryPoints = 4096;

  // Shared state (helper thread produces; RT thread consumes).
  struct TrajectoryPointRuntime {
    uint64_t t_from_start_ns = 0;
    uint32_t axis_mask = 0;
    uint32_t flags = 0;
    // Stored in RTCore's raw CSP wire frame: the same count space the drive
    // publishes on 0x6064 and expects on 0x607A.
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> target_counts{};
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> velocity_counts_per_s{};
  };

  struct PendingTrajectoryUpload {
    bool active = false;
    uint64_t traj_id = 0;
    uint32_t axis_mask = 0;
    uint32_t expected_points = 0;
    uint32_t point_count = 0;
    std::array<TrajectoryPointRuntime, kMaxTrajectoryPoints> points{};
  };

  struct CommittedTrajectory {
    std::atomic<uint64_t> seq{0};
    uint64_t traj_id = 0;
    uint64_t cmd_seq = 0;
    uint32_t axis_mask = 0;
    uint32_t point_count = 0;
    std::array<TrajectoryPointRuntime, kMaxTrajectoryPoints> points{};
  };

  struct JogCommandRuntime {
    std::atomic<uint64_t> seq{0};
    uint64_t cmd_seq = 0;
    uint32_t axis_mask = 0;
    uint32_t flags = 0;
    uint64_t timeout_ns = 0;
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> velocity_counts_per_s{};
  };

  struct LatestFeedback {
    std::atomic<uint64_t> seq{0};
    uint32_t wkc_actual = 0;
    uint32_t wkc_expected = 0;
    uint32_t master_state = gradient::ipc::v1::MASTER_INIT;
    uint32_t responding_slaves = 0;
    uint32_t online_slaves = 0;
    uint32_t operational_slaves = 0;
    int64_t dc_offset_ns = 0;
    int64_t cycle_jitter_ns = 0;
    uint8_t link_up = 0;
    uint8_t master_al_states = 0;
    uint8_t domain_wc_state = 0;
    uint8_t startup_ready = 0;
    uint8_t startup_passive_active = 0;
    uint8_t startup_skip_domain_queue_active = 0;
    uint32_t startup_elapsed_ms = 0;
    uint32_t startup_reset_count = 0;
    std::array<StartupSdoFeedback, kMaxStartupSdoDescriptors> startup_drive_config_feedback{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_state{};
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_position_offset{};
    std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_last_abort_code{};
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pos_counts{};
    std::array<int16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> torque_raw{};
    std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> statusword{};
    std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> error_code{};
    std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> manufacturer_error_code{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> mode_display{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ds402_state{};
    std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> di_bits{};
    // Last value written to 0x607A (target_pos) on the wire this cycle.
    // Populated in the RT loop alongside the feedback fields so the fast-trace
    // thread can observe the command stream at cycle rate without reading SDO.
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> target_pos_counts{};
    std::array<AbsoluteFeedbackAxis, gradient::ipc::v1::GRADIENT_MAX_AXES> absolute_feedback{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_al_state{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_online{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_operational{};
  };

  struct LatestJogDebug {
    std::atomic<uint64_t> seq{0};
    uint32_t num_axes = 0;
    uint32_t active_jog = 0;
    uint32_t active_jog_axis_mask = 0;
    uint32_t command_sp_mask = 0;
    uint32_t have_hold_mask = 0;
    uint32_t have_jog_target_mask = 0;
    uint32_t snap_hold_mask = 0;
    uint32_t stop_arrest_mask = 0;
    uint32_t latest_cmd_axis_mask = 0;
    uint32_t latest_cmd_flags = 0;
    uint32_t last_stop_reason = gradient::ipc::v1::JOG_STOP_REASON_NONE;
    uint32_t last_stop_axis_mask = 0;
    uint64_t sample_time_ns = 0;
    uint64_t active_jog_cmd_seq = 0;
    uint64_t latest_jog_seq_seen = 0;
    uint64_t active_jog_deadline_ns = 0;
    uint64_t latest_cmd_timeout_ns = 0;
    uint64_t last_stop_time_ns = 0;
    uint64_t last_stop_cmd_seq = 0;
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> feedback_pos_counts{};
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> hold_target_counts{};
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> output_target_counts{};
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> output_target_velocity_counts_per_s{};
  };

  constexpr int32_t kNoModeOverride = std::numeric_limits<int32_t>::min();
  std::atomic<bool> armed{false};
  std::atomic<uint32_t> axis_enable_mask{0};
  std::array<std::atomic<int32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> desired_mode_of_operation{};
  std::atomic<uint32_t> service_mode_axis_mask{0};
  std::array<std::atomic<int32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> service_mode_override{};
  std::array<std::atomic<uint32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> service_controlword_override{};
  std::atomic<uint32_t> fault_reset_request{0}; // bitmask; helper thread -> RT thread
  std::atomic<uint64_t> trajectory_abort_request{0}; // 0 means none, UINT64_MAX means any active trajectory
  std::atomic<uint32_t> native_home_active_axis_mask{0};
  CommittedTrajectory committed_trajectory{};
  JogCommandRuntime latest_jog_command{};
  LatestFeedback latest_feedback{};
  LatestJogDebug latest_jog_debug{};
  std::atomic<uint32_t> motion_active_mode{gradient::ipc::v1::MOTION_MODE_IDLE};
  std::atomic<uint32_t> motion_exec_state{gradient::ipc::v1::EXEC_STATE_IDLE};
  std::atomic<uint64_t> motion_active_traj_id{0};
  std::atomic<uint32_t> motion_current_point_index{std::numeric_limits<uint32_t>::max()};
  std::atomic<uint32_t> motion_queue_depth{0};
  std::atomic<uint32_t> motion_last_event_code{0};
  std::atomic<uint32_t> motion_underrun_count{0};
  std::atomic<uint32_t> motion_stale_command_flag{0};
  std::atomic<uint32_t> motion_done{1};
  std::atomic<uint32_t> motion_capability_flags{
      gradient::ipc::v1::MOTION_CAP_TRAJECTORY_UPLOAD |
      gradient::ipc::v1::MOTION_CAP_JOG_COMMAND};
  std::atomic<uint64_t> motion_active_command_seq{0};
  std::atomic<uint64_t> motion_last_update_ns{0};
#if GRADIENT_HAVE_ECRT
  ec_master_t* shared_master = nullptr;
  // Serialize non-RT SDO traffic against master teardown so the metrics/helper
  // threads cannot race ecrt_release_master() during stop/start cycles.
  std::mutex shared_master_sdo_mutex;
#endif

  for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
    desired_mode_of_operation[i].store(8, std::memory_order_relaxed);
    service_mode_override[i].store(kNoModeOverride, std::memory_order_relaxed);
    service_controlword_override[i].store(0, std::memory_order_relaxed);
  }

  auto write_sdo_scalar_axis = [&](uint32_t axis,
                                   uint16_t index,
                                   uint8_t subindex,
                                   SdoScalarType type,
                                   int32_t value,
                                   uint32_t* abort_out) {
#if GRADIENT_HAVE_ECRT
    std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
    if (g_stop.load(std::memory_order_relaxed) || !shared_master) {
      return false;
    }
    uint8_t raw[sizeof(int32_t)] = {};
    size_t raw_size = 0;
    switch (type) {
      case SdoScalarType::kU8:
        raw[0] = static_cast<uint8_t>(value & 0xFF);
        raw_size = sizeof(uint8_t);
        break;
      case SdoScalarType::kI8:
        raw[0] = static_cast<uint8_t>(static_cast<int8_t>(value));
        raw_size = sizeof(int8_t);
        break;
      case SdoScalarType::kU16:
        EC_WRITE_U16(raw, static_cast<uint16_t>(value & 0xFFFF));
        raw_size = sizeof(uint16_t);
        break;
      case SdoScalarType::kI16:
        EC_WRITE_S16(raw, static_cast<int16_t>(value));
        raw_size = sizeof(int16_t);
        break;
      case SdoScalarType::kU32:
        EC_WRITE_U32(raw, static_cast<uint32_t>(value));
        raw_size = sizeof(uint32_t);
        break;
      case SdoScalarType::kI32:
        EC_WRITE_S32(raw, value);
        raw_size = sizeof(int32_t);
        break;
      case SdoScalarType::kNone:
      default:
        return false;
    }
    uint32_t abort_code = 0;
    const int rc = ecrt_master_sdo_download(
        shared_master,
        opt.slave_position[axis],
        index,
        subindex,
        raw,
        raw_size,
        &abort_code);
    if (abort_out) {
      *abort_out = abort_code;
    }
    return rc == 0;
#else
    (void)axis;
    (void)index;
    (void)subindex;
    (void)type;
    (void)value;
    if (abort_out) {
      *abort_out = 0;
    }
    return false;
#endif
  };

  auto read_sdo_scalar_axis = [&](uint32_t axis,
                                  uint16_t index,
                                  uint8_t subindex,
                                  SdoScalarType type,
                                  int32_t* value_out,
                                  uint32_t* abort_out) {
#if GRADIENT_HAVE_ECRT
    std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
    if (!value_out || g_stop.load(std::memory_order_relaxed) || !shared_master) {
      return false;
    }
    uint8_t raw[sizeof(int32_t)] = {};
    size_t raw_size = 0;
    uint32_t abort_code = 0;
    const int rc = ecrt_master_sdo_upload(
        shared_master,
        opt.slave_position[axis],
        index,
        subindex,
        raw,
        sizeof(raw),
        &raw_size,
        &abort_code);
    if (abort_out) {
      *abort_out = abort_code;
    }
    if (rc != 0) {
      return false;
    }
    switch (type) {
      case SdoScalarType::kU8:
        if (raw_size < sizeof(uint8_t)) {
          return false;
        }
        *value_out = static_cast<int32_t>(raw[0]);
        return true;
      case SdoScalarType::kI8:
        if (raw_size < sizeof(int8_t)) {
          return false;
        }
        *value_out = static_cast<int32_t>(static_cast<int8_t>(raw[0]));
        return true;
      case SdoScalarType::kU16:
        if (raw_size < sizeof(uint16_t)) {
          return false;
        }
        *value_out = static_cast<int32_t>(EC_READ_U16(raw));
        return true;
      case SdoScalarType::kI16:
        if (raw_size < sizeof(int16_t)) {
          return false;
        }
        *value_out = static_cast<int32_t>(EC_READ_S16(raw));
        return true;
      case SdoScalarType::kU32:
        if (raw_size < sizeof(uint32_t)) {
          return false;
        }
        *value_out = static_cast<int32_t>(EC_READ_U32(raw));
        return true;
      case SdoScalarType::kI32:
        if (raw_size < sizeof(int32_t)) {
          return false;
        }
        *value_out = EC_READ_S32(raw);
        return true;
      case SdoScalarType::kNone:
      default:
        return false;
    }
#else
    (void)axis;
    (void)index;
    (void)subindex;
    (void)type;
    (void)value_out;
    if (abort_out) {
      *abort_out = 0;
    }
    return false;
#endif
  };

  auto read_native_home_truth_axis = [&](uint32_t axis, int32_t* truth_out) {
#if GRADIENT_HAVE_ECRT
    if (!truth_out || !native_home_cfg.valid || !native_home_cfg.truth_source.valid) {
      return false;
    }
    uint32_t abort_code = 0;
    if (!read_sdo_scalar_axis(axis,
                              native_home_cfg.truth_source.index,
                              native_home_cfg.truth_source.subindex,
                              native_home_cfg.truth_source.type,
                              truth_out,
                              &abort_code)) {
      logf("WARNING: native_home truth upload failed axis=%u slave_pos=%u index=0x%04x sub=0x%02x abort=0x%08x",
           axis,
           static_cast<unsigned int>(opt.slave_position[axis]),
           static_cast<unsigned int>(native_home_cfg.truth_source.index),
           static_cast<unsigned int>(native_home_cfg.truth_source.subindex),
           static_cast<unsigned int>(abort_code));
      return false;
    }
    // 607C is the drive's persisted home/reference value in the wire frame:
    // after HM35, 6064 at home becomes 607C. Host-side
    // native_home_position_offset, however, is the additive correction that
    // maps raw wire counts back into the logical zero-centered frame
    // (logical = raw + offset). Therefore a positive 607C must be imported as
    // a negative host offset.
    if (native_home_cfg.truth_source.index == 0x607C &&
        native_home_cfg.truth_source.subindex == 0x00) {
      *truth_out = clamp_i64_to_i32(-static_cast<int64_t>(*truth_out));
    }
    return true;
#else
    (void)axis;
    (void)truth_out;
    return false;
#endif
  };

  auto read_absolute_feedback_field_axis = [&](uint32_t axis,
                                               uint16_t index,
                                               uint8_t subindex,
                                               SdoScalarType type,
                                               AbsoluteFeedbackFieldSample* out) {
    if (!out) {
      return false;
    }
#if GRADIENT_HAVE_ECRT
    int32_t value = 0;
    uint32_t abort_code = 0;
    if (!read_sdo_scalar_axis(axis, index, subindex, type, &value, &abort_code)) {
      out->valid = 0u;
      out->value = 0;
      return false;
    }
    out->valid = 1u;
    out->value = value;
    return true;
#else
    (void)axis;
    (void)index;
    (void)subindex;
    (void)type;
    out->valid = 0u;
    out->value = 0;
    return false;
#endif
  };

  constexpr int kExitCodeMasterReservationFailed = 75;
  std::atomic<int> process_exit_code{0};

  std::thread rt_thread([&]() {
    pthread_setname_np(pthread_self(), "rt-cycle");

    // Best-effort affinity to CPUs 2-3 (matches plan defaults).
    const unsigned int cpu_count = std::thread::hardware_concurrency();
    if (cpu_count >= 4) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(2, &cpuset);
      CPU_SET(3, &cpuset);
      if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
        logf("WARNING: failed to pin rt-cycle thread to CPU2-CPU3: %s", std::strerror(errno));
      }
    }

    // Best-effort SCHED_FIFO. In production the systemd unit should grant RT priority.
    sched_param sp{};
    sp.sched_priority = 90;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      logf("WARNING: failed to set SCHED_FIFO for rt-cycle thread: %s", std::strerror(errno));
    }

    const uint64_t period = opt.cycle_ns;
    const double period_s = static_cast<double>(period) / 1e9;
    uint64_t next_ns = now_monotonic_ns();
    bool shutdown_active = false;
    uint64_t shutdown_until_ns = 0;
#if GRADIENT_HAVE_ECRT
    // Grace window to push DS402 disable (controlword=0) before dropping the bus.
    constexpr uint64_t kShutdownGraceNs = 250000000ULL; // 250 ms
    bool active_trajectory = false;
    bool active_jog = false;
    uint64_t active_traj_id_rt = 0;
    uint64_t active_commit_seq_seen = 0;
    uint64_t active_jog_seq_seen = 0;
    uint64_t active_jog_cmd_seq = 0;
    uint64_t active_jog_deadline_ns = 0;
    uint64_t latest_jog_timeout_ns = 0;
    uint64_t active_traj_start_ns = 0;
    uint64_t active_traj_cmd_seq = 0;
    uint64_t last_jog_stop_time_ns = 0;
    uint64_t last_jog_stop_cmd_seq = 0;
    uint32_t active_traj_axis_mask = 0;
    uint32_t active_traj_point_count = 0;
    uint32_t active_traj_segment_index = 0;
    uint32_t active_jog_axis_mask = 0;
    uint32_t latest_jog_axis_mask = 0;
    uint32_t latest_jog_flags = 0;
    uint32_t last_jog_stop_reason = gradient::ipc::v1::JOG_STOP_REASON_NONE;
    uint32_t last_jog_stop_axis_mask = 0;
    std::array<TrajectoryPointRuntime, kMaxTrajectoryPoints> active_traj_points{};
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> active_jog_velocity_counts_per_s{};
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> jog_target_counts_float{};
    std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> have_jog_target{};
#endif

#if GRADIENT_HAVE_ECRT
    ec_master_t*& master = shared_master;
#endif

#if GRADIENT_HAVE_ECRT
    // -----------------------------------------------------------------------
    // IgH libecrt setup (descriptor-driven EtherCAT DS402 bring-up)
    // -----------------------------------------------------------------------
    //
    // NOTE: This block compiles only when IgH headers are present. It is
    // intentionally "init-only" work; the cyclic loop below avoids allocation.
    ec_domain_t* domain = nullptr;
    std::array<ec_domain_t*, gradient::ipc::v1::GRADIENT_MAX_AXES> axis_domain{};
    uint8_t* domain_pd = nullptr;
    std::array<uint8_t*, gradient::ipc::v1::GRADIENT_MAX_AXES> axis_domain_pd{};
    ec_master_state_t master_diag_state{};
    ec_domain_state_t domain_state{};

    constexpr unsigned int kInvalidOffset = std::numeric_limits<unsigned int>::max();
    constexpr size_t kMaxPdoRegsPerAxis = 32;

    struct AxisOffsets {
      unsigned int cw = kInvalidOffset;
      unsigned int target_pos = kInvalidOffset;
      unsigned int target_vel = kInvalidOffset;
      unsigned int target_torque = kInvalidOffset;
      unsigned int mode = kInvalidOffset;
      unsigned int tp_func = kInvalidOffset;
      unsigned int max_profile_vel = kInvalidOffset;

      unsigned int err = kInvalidOffset;
      unsigned int manufacturer_err = kInvalidOffset;
      unsigned int sw = kInvalidOffset;
      unsigned int pos = kInvalidOffset;
      unsigned int torque = kInvalidOffset;
      unsigned int mode_disp = kInvalidOffset;
      unsigned int tp_status = kInvalidOffset;
      unsigned int tp_pos1 = kInvalidOffset;
      unsigned int tp_pos2 = kInvalidOffset;
      unsigned int di = kInvalidOffset;
    };

    std::array<ec_slave_config_t*, gradient::ipc::v1::GRADIENT_MAX_AXES> sc{};
    std::array<ec_slave_config_state_t, gradient::ipc::v1::GRADIENT_MAX_AXES> sc_state{};
    std::array<AxisOffsets, gradient::ipc::v1::GRADIENT_MAX_AXES> off{};
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> hold_target_counts{};
    std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> have_hold{};
    std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> jog_stop_quick_stop_cycles_left{};
    std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> jog_stop_arrest_cycles_left{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> fault_reset_left{};
    struct SlaveDiagSnapshot {
      bool valid = false;
      bool online = false;
      bool operational = false;
      uint8_t al_state = 0;
    };
    struct MasterDiagSnapshot {
      bool valid = false;
      bool link_up = false;
      unsigned int responding = 0;
      unsigned int al_states = 0;
      unsigned int domain_wc_state = 0;
      unsigned int domain_working_counter = 0;
      uint32_t online_slaves = 0;
      uint32_t operational_slaves = 0;
    };
    std::array<SlaveDiagSnapshot, gradient::ipc::v1::GRADIENT_MAX_AXES> prev_slave_diag{};
    MasterDiagSnapshot prev_master_diag{};
    constexpr uint8_t kFaultResetPulseCycles = 20; // ~20ms at 1kHz
    constexpr uint16_t kJogQuickStopCycles = 300;  // ~300ms at 1kHz
    constexpr uint16_t kJogStopArrestCycles = 200; // ~200ms at 1kHz
    constexpr uint64_t kStartupLogIntervalNs = 1000000000ULL; // 1s
    constexpr uint64_t kStartupResetDelayNs = 1000000000ULL;  // 1s
    constexpr uint64_t kStartupDetailedDiagBaseNs = 1500000000ULL;       // 1.5s
    constexpr uint64_t kStartupDetailedDiagPostResumeNs = 1000000000ULL; // 1.0s
    constexpr uint64_t kStartupTimeoutMinNs = 5000000000ULL;             // 5s
    const uint64_t startup_timeout_ns =
        std::max<uint64_t>(
            kStartupTimeoutMinNs,
            (static_cast<uint64_t>(opt.preop_to_safeop_timeout_ms) +
             static_cast<uint64_t>(opt.safeop_to_op_timeout_ms)) *
                1000000ULL);
    const uint64_t startup_skip_domain_queue_ns =
        static_cast<uint64_t>(opt.startup_skip_domain_queue_ms) * 1000000ULL;
    const uint64_t startup_detailed_diag_ns =
        (startup_skip_domain_queue_ns + kStartupDetailedDiagPostResumeNs >
         kStartupDetailedDiagBaseNs)
            ? (startup_skip_domain_queue_ns + kStartupDetailedDiagPostResumeNs)
            : kStartupDetailedDiagBaseNs;

    std::vector<ec_pdo_entry_info_t> rx_pdo_entries{};
    rx_pdo_entries.reserve(drive_pdo.rx_entries.size());
    for (const auto& entry : drive_pdo.rx_entries) {
      rx_pdo_entries.push_back(ec_pdo_entry_info_t{entry.index, entry.subindex, entry.bits});
    }
    std::vector<ec_pdo_entry_info_t> tx_pdo_entries{};
    tx_pdo_entries.reserve(drive_pdo.tx_entries.size());
    for (const auto& entry : drive_pdo.tx_entries) {
      tx_pdo_entries.push_back(ec_pdo_entry_info_t{entry.index, entry.subindex, entry.bits});
    }
    std::array<ec_pdo_info_t, 2> runtime_pdos{{
        {drive_pdo.rx_pdo, static_cast<unsigned int>(rx_pdo_entries.size()), rx_pdo_entries.data()},
        {drive_pdo.tx_pdo, static_cast<unsigned int>(tx_pdo_entries.size()), tx_pdo_entries.data()},
    }};
    std::array<ec_sync_info_t, 3> runtime_syncs{{
        {drive_pdo.rx_sync_index,
         EC_DIR_OUTPUT,
         1,
         runtime_pdos.data() + 0,
         opt.disable_output_watchdog ? EC_WD_DISABLE : EC_WD_ENABLE},
        {drive_pdo.tx_sync_index,
         EC_DIR_INPUT,
         1,
         runtime_pdos.data() + 1,
         EC_WD_DISABLE},
        {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE},
    }};
    logf("EtherCAT bring-up config: pdo_profile=%s rx=0x%04x tx=0x%04x rx_sync=%u tx_sync=%u dc=%u "
         "output_watchdog=%u split_domains_per_axis=%u queue_split_domains_round_robin=%u "
         "explicit_pdo_config=%u "
         "wait_before_safeop_ms=%u preop_safeop_timeout_ms=%u safeop_op_timeout_ms=%u "
         "startup_passive_ms=%u startup_skip_domain_queue_ms=%u startup_diag_window_ms=%llu",
         drive_pdo.label.c_str(),
         static_cast<unsigned int>(drive_pdo.rx_pdo),
         static_cast<unsigned int>(drive_pdo.tx_pdo),
         static_cast<unsigned int>(drive_pdo.rx_sync_index),
         static_cast<unsigned int>(drive_pdo.tx_sync_index),
         opt.use_dc ? 1u : 0u,
         opt.disable_output_watchdog ? 0u : 1u,
         opt.split_domains_per_axis ? 1u : 0u,
         opt.queue_split_domains_round_robin ? 1u : 0u,
         opt.explicit_pdo_config ? 1u : 0u,
         opt.wait_before_safeop_ms,
         opt.preop_to_safeop_timeout_ms,
         opt.safeop_to_op_timeout_ms,
         opt.startup_passive_ms,
         opt.startup_skip_domain_queue_ms,
         static_cast<unsigned long long>(startup_detailed_diag_ns / 1000000ULL));
    logf("Metrics SDO toggles: startup_readback=%u native_home_refresh=%u absolute_feedback_poll=%u",
         metrics_startup_readback_enabled ? 1u : 0u,
         metrics_native_home_refresh_enabled ? 1u : 0u,
         metrics_absolute_feedback_poll_enabled ? 1u : 0u);
    for (uint32_t i = 0; i < opt.num_axes; ++i) {
      logf("Axis%u -> EtherCAT slave position %u",
           i,
           static_cast<unsigned int>(opt.slave_position[i]));
    }

    bool ecrt_ok = false;
    const uint32_t expected_wkc = 2 * opt.num_axes;
    bool startup_ready = false;
    bool startup_process_data_live_logged = false;
    bool startup_ready_logged = false;
    bool startup_timeout_logged = false;
    bool startup_wait_logged = false;
    bool startup_reset_issued = false;
    uint32_t startup_reset_count = 0;
    uint64_t startup_begin_ns = 0;
    uint64_t startup_cycle_counter = 0;
    uint64_t last_startup_log_ns = 0;
    bool startup_passive_prev = false;
    bool startup_skip_domain_queue_prev = false;
    // Per-axis safety clamps (max rpm -> counts/s) are computed in finalize_axis_config().

    auto aggregate_domain_state = [&](ec_domain_state_t* out) {
      if (!out) {
        return;
      }
      *out = ec_domain_state_t{};
      bool have_domain_state = false;
      auto merge_domain_state = [&](ec_domain_t* dom) {
        if (!dom) {
          return;
        }
        ec_domain_state_t cur{};
        ecrt_domain_state(dom, &cur);
        if (!have_domain_state || cur.wc_state < out->wc_state) {
          out->wc_state = cur.wc_state;
        }
        out->working_counter += cur.working_counter;
        have_domain_state = true;
      };
      if (opt.split_domains_per_axis) {
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          merge_domain_state(axis_domain[i]);
        }
      } else {
        merge_domain_state(domain);
      }
    };

    auto log_phase_summary = [&](const char* phase, uint64_t elapsed_ns, uint64_t cycle) {
      if (!master) {
        return;
      }
      ec_master_state_t phase_master_state{};
      ecrt_master_state(master, &phase_master_state);
      ec_domain_state_t phase_domain_state{};
      aggregate_domain_state(&phase_domain_state);

      std::array<ec_slave_config_state_t, gradient::ipc::v1::GRADIENT_MAX_AXES> phase_sc_state{};
      uint32_t phase_online_slaves = 0;
      uint32_t phase_operational_slaves = 0;
      for (uint32_t i = 0; i < opt.num_axes; ++i) {
        if (sc[i] && ecrt_slave_config_state(sc[i], &phase_sc_state[i]) == 0) {
          if (phase_sc_state[i].online) {
            phase_online_slaves++;
          }
          if (phase_sc_state[i].operational) {
            phase_operational_slaves++;
          }
        }
      }

      logf("EtherCAT phase=%s cycle=%llu elapsed_ms=%llu link_up=%u responding=%u/%u online=%u/%u "
           "operational=%u/%u master_al=0x%x domain_wc=%u wkc=%u/%u",
           phase,
           static_cast<unsigned long long>(cycle),
           static_cast<unsigned long long>(elapsed_ns / 1000000ULL),
           phase_master_state.link_up ? 1u : 0u,
           phase_master_state.slaves_responding,
           opt.num_axes,
           phase_online_slaves,
           opt.num_axes,
           phase_operational_slaves,
           opt.num_axes,
           phase_master_state.al_states,
           static_cast<unsigned int>(phase_domain_state.wc_state),
           static_cast<unsigned int>(phase_domain_state.working_counter),
           expected_wkc);
      for (uint32_t i = 0; i < opt.num_axes; ++i) {
        if (!sc[i]) {
          continue;
        }
        logf("EtherCAT phase=%s axis=%u slave_pos=%u online=%u operational=%u al=%s(0x%x)",
             phase,
             i,
             static_cast<unsigned int>(opt.slave_position[i]),
             phase_sc_state[i].online ? 1u : 0u,
             phase_sc_state[i].operational ? 1u : 0u,
             al_state_label(static_cast<uint8_t>(phase_sc_state[i].al_state)),
             static_cast<unsigned int>(phase_sc_state[i].al_state));
      }
    };

    auto log_registered_offsets = [&]() {
      for (uint32_t i = 0; i < opt.num_axes; ++i) {
        if (!sc[i]) {
          continue;
        }
        logf("EtherCAT phase=pdo_register axis=%u slave_pos=%u offsets cw=%u target_pos=%u sw=%u pos=%u err=%u mfg_err=%u di=%u",
             i,
             static_cast<unsigned int>(opt.slave_position[i]),
             off[i].cw,
             off[i].target_pos,
             off[i].sw,
             off[i].pos,
             off[i].err,
             off[i].manufacturer_err,
             off[i].di);
      }
    };

    auto rx_offset_for_semantic = [&](AxisOffsets& axis_offsets, const std::string& semantic)
        -> unsigned int* {
      if (semantic == "cw") {
        return &axis_offsets.cw;
      }
      if (semantic == "target_pos") {
        return &axis_offsets.target_pos;
      }
      if (semantic == "target_vel") {
        return &axis_offsets.target_vel;
      }
      if (semantic == "target_torque") {
        return &axis_offsets.target_torque;
      }
      if (semantic == "mode") {
        return &axis_offsets.mode;
      }
      if (semantic == "tp_func") {
        return &axis_offsets.tp_func;
      }
      if (semantic == "max_profile_vel") {
        return &axis_offsets.max_profile_vel;
      }
      return nullptr;
    };
    auto tx_offset_for_semantic = [&](AxisOffsets& axis_offsets, const std::string& semantic)
        -> unsigned int* {
      if (semantic == "err") {
        return &axis_offsets.err;
      }
      if (semantic == "manufacturer_err") {
        return &axis_offsets.manufacturer_err;
      }
      if (semantic == "sw") {
        return &axis_offsets.sw;
      }
      if (semantic == "pos") {
        return &axis_offsets.pos;
      }
      if (semantic == "torque") {
        return &axis_offsets.torque;
      }
      if (semantic == "mode_disp") {
        return &axis_offsets.mode_disp;
      }
      if (semantic == "tp_status") {
        return &axis_offsets.tp_status;
      }
      if (semantic == "tp_pos1") {
        return &axis_offsets.tp_pos1;
      }
      if (semantic == "tp_pos2") {
        return &axis_offsets.tp_pos2;
      }
      if (semantic == "di") {
        return &axis_offsets.di;
      }
      return nullptr;
    };

    auto fill_axis_regs = [&](uint32_t axis, auto& regs, size_t& reg_i) {
      const uint16_t slave_pos = opt.slave_position[axis];
      auto add_reg = [&](uint16_t index, uint8_t subindex, unsigned int* offset) {
        regs[reg_i++] = {0,
                         slave_pos,
                         opt.slave_vendor_id,
                         opt.slave_product_code,
                         index,
                         subindex,
                         offset,
                         nullptr};
      };
      for (const auto& entry : drive_pdo.rx_entries) {
        if (unsigned int* offset = rx_offset_for_semantic(off[axis], entry.semantic)) {
          add_reg(entry.index, entry.subindex, offset);
        }
      }
      for (const auto& entry : drive_pdo.tx_entries) {
        if (unsigned int* offset = tx_offset_for_semantic(off[axis], entry.semantic)) {
          add_reg(entry.index, entry.subindex, offset);
        }
      }
    };

    auto apply_pdo_profile = [&](uint32_t axis, uint16_t slave_pos) {
      if (!opt.explicit_pdo_config) {
        if (ecrt_slave_config_pdos(sc[axis], EC_END, runtime_syncs.data())) {
          logf("ERROR: ecrt_slave_config_pdos failed for axis=%u slave_pos=%u", axis, slave_pos);
          return false;
        }
        logf("EtherCAT config phase=slave_config_pdos axis=%u slave_pos=%u profile=%s mode=wrapper ok",
             axis,
             slave_pos,
             drive_pdo.label.c_str());
        return true;
      }

      for (size_t sync_i = 0; runtime_syncs[sync_i].index != 0xff; ++sync_i) {
        const ec_sync_info_t& sync = runtime_syncs[sync_i];
        if (ecrt_slave_config_sync_manager(sc[axis], sync.index, sync.dir, sync.watchdog_mode)) {
          logf("ERROR: ecrt_slave_config_sync_manager failed for axis=%u slave_pos=%u sync=%u",
               axis,
               slave_pos,
               static_cast<unsigned int>(sync.index));
          return false;
        }
        if (ecrt_slave_config_pdo_assign_clear(sc[axis], sync.index)) {
          logf("ERROR: ecrt_slave_config_pdo_assign_clear failed for axis=%u slave_pos=%u sync=%u",
               axis,
               slave_pos,
               static_cast<unsigned int>(sync.index));
          return false;
        }
        for (unsigned int pdo_i = 0; pdo_i < sync.n_pdos; ++pdo_i) {
          const ec_pdo_info_t& pdo = sync.pdos[pdo_i];
          if (ecrt_slave_config_pdo_assign_add(sc[axis], sync.index, pdo.index)) {
            logf("ERROR: ecrt_slave_config_pdo_assign_add failed for axis=%u slave_pos=%u sync=%u pdo=0x%04x",
                 axis,
                 slave_pos,
                 static_cast<unsigned int>(sync.index),
                 static_cast<unsigned int>(pdo.index));
            return false;
          }
        }
      }

      logf("EtherCAT config phase=slave_config_pdos axis=%u slave_pos=%u profile=%s mode=explicit_assign ok",
           axis,
           slave_pos,
           drive_pdo.label.c_str());
      return true;
    };

    auto schedule_axis_startup_sdos = [&](uint32_t axis, uint16_t slave_pos) {
      if (startup_sdos.empty()) {
        return true;
      }
      for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
        const auto& descriptor = startup_sdos[descriptor_i];
        auto& feedback = latest_feedback.startup_drive_config_feedback[descriptor_i];
        feedback.configured[axis] = descriptor.valid ? 1u : 0u;
        feedback.commanded[axis] = descriptor.valid ? descriptor.values[axis] : 0u;
        if (descriptor.type != StartupSdoValueType::kU16) {
          logf("ERROR: unsupported startup SDO type for axis=%u slave_pos=%u key=%s",
               axis,
               slave_pos,
               descriptor.key.c_str());
          return false;
        }
        if (descriptor.values[axis] > static_cast<uint32_t>(std::numeric_limits<uint16_t>::max())) {
          logf("ERROR: startup SDO value out of range for axis=%u slave_pos=%u key=%s value=%u",
               axis,
               slave_pos,
               descriptor.key.c_str(),
               static_cast<unsigned int>(descriptor.values[axis]));
          return false;
        }
        const uint16_t value = static_cast<uint16_t>(descriptor.values[axis]);
        const int rc = ecrt_slave_config_sdo16(sc[axis], descriptor.index, descriptor.subindex, value);
        if (rc != 0) {
          logf("ERROR: ecrt_slave_config_sdo16 failed for axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x value=%u rc=%d",
               axis,
               slave_pos,
               descriptor.key.c_str(),
               static_cast<unsigned int>(descriptor.index),
               static_cast<unsigned int>(descriptor.subindex),
               static_cast<unsigned int>(value),
               rc);
          return false;
        }
        logf("EtherCAT config phase=slave_config_sdo axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x value=%u ok",
             axis,
             slave_pos,
             descriptor.key.c_str(),
             static_cast<unsigned int>(descriptor.index),
             static_cast<unsigned int>(descriptor.subindex),
             static_cast<unsigned int>(value));
      }
      return true;
    };

    auto register_axis_offsets_explicit = [&](uint32_t axis, ec_domain_t* target_domain) {
      const uint16_t slave_pos = opt.slave_position[axis];
      auto reg = [&](uint8_t sync_index,
                     unsigned int entry_pos,
                     unsigned int* offset,
                     const char* field_name) {
        const int rc =
            ecrt_slave_config_reg_pdo_entry_pos(sc[axis], sync_index, 0, entry_pos, target_domain, nullptr);
        if (rc < 0) {
          logf("ERROR: ecrt_slave_config_reg_pdo_entry_pos failed for axis=%u slave_pos=%u "
               "sync=%u entry_pos=%u field=%s rc=%d",
               axis,
               slave_pos,
               static_cast<unsigned int>(sync_index),
               entry_pos,
               field_name,
               rc);
          return false;
        }
        *offset = static_cast<unsigned int>(rc);
        return true;
      };

      for (size_t entry_pos = 0; entry_pos < drive_pdo.rx_entries.size(); ++entry_pos) {
        const auto& entry = drive_pdo.rx_entries[entry_pos];
        if (unsigned int* offset = rx_offset_for_semantic(off[axis], entry.semantic)) {
          if (!reg(drive_pdo.rx_sync_index,
                   static_cast<unsigned int>(entry_pos),
                   offset,
                   entry.semantic.c_str())) {
            return false;
          }
        }
      }
      for (size_t entry_pos = 0; entry_pos < drive_pdo.tx_entries.size(); ++entry_pos) {
        const auto& entry = drive_pdo.tx_entries[entry_pos];
        if (unsigned int* offset = tx_offset_for_semantic(off[axis], entry.semantic)) {
          if (!reg(drive_pdo.tx_sync_index,
                   static_cast<unsigned int>(entry_pos),
                   offset,
                   entry.semantic.c_str())) {
            return false;
          }
        }
      }

      return true;
    };

    master = ecrt_request_master(0);
    if (!master) {
      logf("ERROR: ecrt_request_master(0) failed; exit_code=%d (master reservation failed, likely stale owner or hung EtherCAT kernel task)",
           kExitCodeMasterReservationFailed);
      process_exit_code.store(kExitCodeMasterReservationFailed, std::memory_order_relaxed);
      g_stop.store(true, std::memory_order_relaxed);
      return;
    } else {
      logf("EtherCAT config phase=request_master ok");
      bool have_domains = true;
      if (opt.split_domains_per_axis) {
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          axis_domain[i] = ecrt_master_create_domain(master);
          if (!axis_domain[i]) {
            logf("ERROR: ecrt_master_create_domain failed for axis=%u slave_pos=%u",
                 i,
                 static_cast<unsigned int>(opt.slave_position[i]));
            have_domains = false;
            break;
          }
          logf("EtherCAT config phase=create_domain axis=%u slave_pos=%u ok",
               i,
               static_cast<unsigned int>(opt.slave_position[i]));
        }
      } else {
        domain = ecrt_master_create_domain(master);
        if (!domain) {
          logf("ERROR: ecrt_master_create_domain failed");
          have_domains = false;
        } else {
          logf("EtherCAT config phase=create_domain ok");
        }
      }
      if (have_domains) {
        bool slave_config_ok = true;
        // Configure each slave at (alias=0, position=i).
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          const uint16_t slave_pos = opt.slave_position[i];
          sc[i] = ecrt_master_slave_config(master, 0, slave_pos,
                                           opt.slave_vendor_id,
                                           opt.slave_product_code);
          if (!sc[i]) {
            logf("ERROR: ecrt_master_slave_config failed for axis=%u slave_pos=%u", i, slave_pos);
            slave_config_ok = false;
            break;
          }
          logf("EtherCAT config phase=slave_config axis=%u slave_pos=%u ok", i, slave_pos);
          log_phase_summary("after_slave_config", 0, 0);

          // Give the drive a conservative PLC-like convergence window before SAFEOP/OP.
#ifdef EC_HAVE_FLAGS
          if (opt.wait_before_safeop_ms > 0) {
            const int rc = ecrt_slave_config_flag(
                sc[i], "WaitBeforeSAFEOPms", static_cast<int32_t>(opt.wait_before_safeop_ms));
            if (rc != 0) {
              logf("WARNING: ecrt_slave_config_flag(WaitBeforeSAFEOPms=%u) failed for axis=%u slave_pos=%u rc=%d",
                   opt.wait_before_safeop_ms,
                   i,
                   slave_pos,
                   rc);
            }
          }
#endif
#ifdef EC_HAVE_STATE_TIMEOUT
          if (opt.preop_to_safeop_timeout_ms > 0) {
            const int rc = ecrt_slave_config_state_timeout(
                sc[i], EC_AL_STATE_PREOP, EC_AL_STATE_SAFEOP, opt.preop_to_safeop_timeout_ms);
            if (rc != 0) {
              logf("WARNING: ecrt_slave_config_state_timeout(PREOP->SAFEOP=%u ms) failed for axis=%u slave_pos=%u rc=%d",
                   opt.preop_to_safeop_timeout_ms,
                   i,
                   slave_pos,
                   rc);
            }
          }
          if (opt.safeop_to_op_timeout_ms > 0) {
            const int rc = ecrt_slave_config_state_timeout(
                sc[i], EC_AL_STATE_SAFEOP, EC_AL_STATE_OP, opt.safeop_to_op_timeout_ms);
            if (rc != 0) {
              logf("WARNING: ecrt_slave_config_state_timeout(SAFEOP->OP=%u ms) failed for axis=%u slave_pos=%u rc=%d",
                   opt.safeop_to_op_timeout_ms,
                   i,
                   slave_pos,
                   rc);
            }
          }
#endif

          // Assign the requested PDO profile before activation.
          if (!apply_pdo_profile(i, slave_pos)) {
            slave_config_ok = false;
            break;
          }
          log_phase_summary("after_slave_config_pdos", 0, 0);

          if (!schedule_axis_startup_sdos(i, slave_pos)) {
            slave_config_ok = false;
            break;
          }

          if (opt.use_dc) {
            // DC assign_activate 0x0300 (SYNC0). Shift left as 0 for now.
            // TODO: tune sync0_shift based on measured line delay/jitter.
            const int rc = ecrt_slave_config_dc(sc[i], 0x0300, period, 0, 0, 0);
            if (rc != 0) {
              logf("ERROR: ecrt_slave_config_dc failed for axis=%u slave_pos=%u rc=%d",
                   i,
                   slave_pos,
                   rc);
              slave_config_ok = false;
              break;
            }
            logf("EtherCAT config phase=slave_config_dc axis=%u slave_pos=%u assign_activate=0x0300 cycle_ns=%llu ok",
                 i,
                 slave_pos,
                 static_cast<unsigned long long>(period));
            log_phase_summary("after_slave_config_dc", 0, 0);
          }
        }

        if (slave_config_ok) {
          // Register PDO entries -> domain offsets.
          // NOTE: ec_pdo_entry_reg_t layout differs across some IgH versions.
          // If compilation fails here once IgH is installed, adjust field count/order.
          bool domain_regs_ok = true;
          if (opt.explicit_pdo_config) {
            for (uint32_t i = 0; i < opt.num_axes; ++i) {
              ec_domain_t* target_domain = opt.split_domains_per_axis ? axis_domain[i] : domain;
              if (!register_axis_offsets_explicit(i, target_domain)) {
                domain_regs_ok = false;
                break;
              }
              logf("EtherCAT config phase=domain_reg_pdo_entry_pos axis=%u slave_pos=%u ok",
                   i,
                   static_cast<unsigned int>(opt.slave_position[i]));
            }
          } else {
            if (opt.split_domains_per_axis) {
              for (uint32_t i = 0; i < opt.num_axes; ++i) {
                std::array<ec_pdo_entry_reg_t, kMaxPdoRegsPerAxis + 1> regs{};
                size_t reg_i = 0;
                fill_axis_regs(i, regs, reg_i);
                regs[reg_i] = {};
                if (ecrt_domain_reg_pdo_entry_list(axis_domain[i], regs.data())) {
                  logf("ERROR: ecrt_domain_reg_pdo_entry_list failed for axis=%u slave_pos=%u",
                       i,
                       static_cast<unsigned int>(opt.slave_position[i]));
                  domain_regs_ok = false;
                  break;
                }
                logf("EtherCAT config phase=domain_reg_pdo_entry_list axis=%u slave_pos=%u regs=%zu ok",
                     i,
                     static_cast<unsigned int>(opt.slave_position[i]),
                     reg_i);
              }
            } else {
              std::array<ec_pdo_entry_reg_t,
                         (gradient::ipc::v1::GRADIENT_MAX_AXES * kMaxPdoRegsPerAxis) + 1>
                  regs{};
              size_t reg_i = 0;
              for (uint32_t i = 0; i < opt.num_axes; ++i) {
                fill_axis_regs(i, regs, reg_i);
              }
              regs[reg_i] = {};
              if (ecrt_domain_reg_pdo_entry_list(domain, regs.data())) {
                logf("ERROR: ecrt_domain_reg_pdo_entry_list failed");
                domain_regs_ok = false;
              } else {
                logf("EtherCAT config phase=domain_reg_pdo_entry_list ok regs=%zu", reg_i);
              }
            }
          }
          if (!domain_regs_ok) {
          } else {
            const uint64_t send_interval_us_u64 =
                opt.cycle_ns <= 1000ULL ? 1ULL : ((opt.cycle_ns + 999ULL) / 1000ULL);
            const size_t send_interval_us = static_cast<size_t>(send_interval_us_u64);
            log_registered_offsets();
            log_phase_summary("after_domain_reg_pdo_entry_list", 0, 0);
            const int send_interval_rc = ecrt_master_set_send_interval(master, send_interval_us);
            if (send_interval_rc != 0) {
              logf("ERROR: ecrt_master_set_send_interval(%zu us) failed rc=%d",
                   send_interval_us,
                   send_interval_rc);
            } else {
              logf("EtherCAT config phase=set_send_interval send_interval_us=%zu ok",
                   send_interval_us);
            }
            if (send_interval_rc != 0) {
              // Do not activate if the master rejected the advertised cycle interval.
            } else if (ecrt_master_activate(master)) {
              logf("ERROR: ecrt_master_activate failed");
            } else {
              bool have_domain_pd = true;
              if (opt.split_domains_per_axis) {
                for (uint32_t i = 0; i < opt.num_axes; ++i) {
                  axis_domain_pd[i] = ecrt_domain_data(axis_domain[i]);
                  if (!axis_domain_pd[i]) {
                    logf("ERROR: ecrt_domain_data returned null for axis=%u slave_pos=%u",
                         i,
                         static_cast<unsigned int>(opt.slave_position[i]));
                    have_domain_pd = false;
                    break;
                  }
                }
              } else {
                domain_pd = ecrt_domain_data(domain);
                if (!domain_pd) {
                  logf("ERROR: ecrt_domain_data returned null");
                  have_domain_pd = false;
                }
              }
              if (!have_domain_pd) {
              } else {
                if (!startup_sdos.empty()) {
                  for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
                    auto& feedback = latest_feedback.startup_drive_config_feedback[descriptor_i];
                    for (uint32_t i = 0; i < opt.num_axes; ++i) {
                      feedback.readback_valid[i] = 0u;
                      feedback.readback[i] = 0u;
                      feedback.verified[i] = 0u;
                    }
                  }
                  logf("EtherCAT startup readback deferred for %zu descriptor(s); metrics thread will verify after startup_ready to avoid blocking rt-cycle during bring-up",
                       startup_sdos.size());
                }
                ecrt_ok = true;
                startup_begin_ns = now_monotonic_ns();
                startup_cycle_counter = 0;
                logf("IgH libecrt active (num_axes=%u, expected_wkc=%u)", opt.num_axes, expected_wkc);
                log_phase_summary("after_master_activate", 0, 0);
              }
            }
          }
        }
      }
    }
#endif  // GRADIENT_HAVE_ECRT

    // Initialization above can take seconds (EtherCAT config, activation, etc.).
    // Rebase the absolute sleep schedule here so the cyclic loop does not try to
    // "catch up" by blasting thousands of immediate iterations after startup.
    next_ns = now_monotonic_ns();

    while (true) {
      // Measure wakeup jitter for the previous absolute sleep target (`next_ns`).
      // This measures how late/early we were relative to the requested schedule.
      const uint64_t wake_target_ns = next_ns;
      const uint64_t wake_ns = now_monotonic_ns();
      const int64_t jitter_ns =
          static_cast<int64_t>(wake_ns) - static_cast<int64_t>(wake_target_ns);
      rt_last_jitter_ns.store(jitter_ns, std::memory_order_relaxed);
      const int64_t abs_jitter_ns = (jitter_ns >= 0) ? jitter_ns : -jitter_ns;
      int64_t prev_max = rt_max_abs_jitter_ns.load(std::memory_order_relaxed);
      while (abs_jitter_ns > prev_max &&
             !rt_max_abs_jitter_ns.compare_exchange_weak(
                 prev_max, abs_jitter_ns, std::memory_order_relaxed)) {
      }
      if (jitter_ns > static_cast<int64_t>(period)) {
        rt_overrun_count.fetch_add(1, std::memory_order_relaxed);
      }

      next_ns += period;

#if GRADIENT_HAVE_ECRT
      if (ecrt_ok) {
        // --- EtherCAT cyclic loop (1 kHz) ---
        const uint64_t app_time_ns = next_ns;
        const uint64_t diag_now_ns = wake_ns;
        startup_cycle_counter++;
        const uint64_t startup_elapsed_ns_pre =
            startup_begin_ns != 0 && diag_now_ns >= startup_begin_ns ? (diag_now_ns - startup_begin_ns) : 0;
        const bool startup_passive_active =
            opt.startup_passive_ms > 0 &&
            startup_elapsed_ns_pre < (static_cast<uint64_t>(opt.startup_passive_ms) * 1000000ULL);
        const bool startup_skip_domain_queue_active =
            startup_skip_domain_queue_ns > 0 &&
            startup_elapsed_ns_pre < startup_skip_domain_queue_ns;

        if (startup_passive_active != startup_passive_prev) {
          logf("EtherCAT startup passive_outputs=%u cycle=%llu elapsed_ms=%llu",
               startup_passive_active ? 1u : 0u,
               static_cast<unsigned long long>(startup_cycle_counter),
               static_cast<unsigned long long>(startup_elapsed_ns_pre / 1000000ULL));
          startup_passive_prev = startup_passive_active;
        }
        if (startup_skip_domain_queue_active != startup_skip_domain_queue_prev) {
          logf("EtherCAT startup domain_queue_suppressed=%u cycle=%llu elapsed_ms=%llu",
               startup_skip_domain_queue_active ? 1u : 0u,
               static_cast<unsigned long long>(startup_cycle_counter),
               static_cast<unsigned long long>(startup_elapsed_ns_pre / 1000000ULL));
          startup_skip_domain_queue_prev = startup_skip_domain_queue_active;
        }

        // If a shutdown is requested (SIGINT/SIGTERM), disable all axes for a short
        // grace window while we still have cyclic communication. This reduces the
        // chance of the drive faulting when the master drops out of OP.
        if (g_stop.load(std::memory_order_relaxed) && !shutdown_active) {
          shutdown_active = true;
          shutdown_until_ns = diag_now_ns + kShutdownGraceNs;
          armed.store(false, std::memory_order_relaxed);
          axis_enable_mask.store(0, std::memory_order_relaxed);
          service_mode_axis_mask.store(0, std::memory_order_relaxed);
          native_home_active_axis_mask.store(0, std::memory_order_relaxed);
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            desired_mode_of_operation[axis].store(0, std::memory_order_relaxed);
            service_mode_override[axis].store(kNoModeOverride, std::memory_order_relaxed);
            service_controlword_override[axis].store(0, std::memory_order_relaxed);
          }
        }

        ecrt_master_application_time(master, app_time_ns);
        ecrt_master_receive(master);
        if (opt.split_domains_per_axis) {
          for (uint32_t i = 0; i < opt.num_axes; ++i) {
            ecrt_domain_process(axis_domain[i]);
          }
        } else {
          ecrt_domain_process(domain);
        }

        std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> target_counts{};
        std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> target_velocity_counts_per_s{};
        std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> output_target_counts{};
        std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> output_target_velocity_counts_per_s{};
        uint32_t sp_mask = 0;
        uint32_t snap_jog_hold_to_feedback_mask = 0;
        const uint64_t abort_req =
            trajectory_abort_request.exchange(0, std::memory_order_acq_rel);
        if (active_trajectory &&
            (abort_req == std::numeric_limits<uint64_t>::max() || abort_req == active_traj_id_rt)) {
          active_trajectory = false;
          active_traj_axis_mask = 0;
          active_traj_point_count = 0;
          active_traj_segment_index = 0;
          motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_TRAJECTORY, std::memory_order_relaxed);
          motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_ABORTED, std::memory_order_relaxed);
          motion_active_traj_id.store(active_traj_id_rt, std::memory_order_relaxed);
          motion_current_point_index.store(std::numeric_limits<uint32_t>::max(), std::memory_order_relaxed);
          motion_queue_depth.store(0, std::memory_order_relaxed);
          motion_last_event_code.store(gradient::ipc::v1::EVT_TRAJECTORY_ABORTED, std::memory_order_relaxed);
          motion_stale_command_flag.store(0, std::memory_order_relaxed);
          motion_done.store(1, std::memory_order_relaxed);
          motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
        }

        const uint64_t committed_seq = committed_trajectory.seq.load(std::memory_order_acquire);
        if (committed_seq != 0 && committed_seq != active_commit_seq_seen) {
          active_commit_seq_seen = committed_seq;
          if (active_jog) {
            last_jog_stop_reason = gradient::ipc::v1::JOG_STOP_REASON_TRAJECTORY_PREEMPT;
            last_jog_stop_time_ns = diag_now_ns;
            last_jog_stop_cmd_seq = active_jog_cmd_seq;
            last_jog_stop_axis_mask = active_jog_axis_mask;
          }
          active_trajectory = true;
          active_jog = false;
          active_jog_axis_mask = 0;
          active_jog_deadline_ns = 0;
          have_jog_target.fill(false);
          jog_stop_quick_stop_cycles_left.fill(0);
          jog_stop_arrest_cycles_left.fill(0);
          active_traj_id_rt = committed_trajectory.traj_id;
          active_traj_cmd_seq = committed_trajectory.cmd_seq;
          active_traj_axis_mask = committed_trajectory.axis_mask;
          active_traj_point_count = committed_trajectory.point_count;
          active_traj_segment_index = 0;
          active_traj_start_ns = diag_now_ns;
          for (uint32_t i = 0; i < active_traj_point_count; ++i) {
            active_traj_points[i] = committed_trajectory.points[i];
          }
          motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_TRAJECTORY, std::memory_order_relaxed);
          motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_EXECUTING, std::memory_order_relaxed);
          motion_active_traj_id.store(active_traj_id_rt, std::memory_order_relaxed);
          motion_current_point_index.store(0, std::memory_order_relaxed);
          motion_queue_depth.store(
              active_traj_point_count > 0 ? (active_traj_point_count - 1) : 0,
              std::memory_order_relaxed);
          motion_last_event_code.store(
              gradient::ipc::v1::EVT_TRAJECTORY_EXECUTING, std::memory_order_relaxed);
          motion_stale_command_flag.store(0, std::memory_order_relaxed);
          motion_done.store(0, std::memory_order_relaxed);
          motion_active_command_seq.store(active_traj_cmd_seq, std::memory_order_relaxed);
          motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
        }

        const uint64_t jog_seq = latest_jog_command.seq.load(std::memory_order_acquire);
        if (jog_seq != 0 && jog_seq != active_jog_seq_seen) {
          active_jog_seq_seen = jog_seq;
          active_jog_cmd_seq = latest_jog_command.cmd_seq;
          latest_jog_axis_mask = latest_jog_command.axis_mask;
          latest_jog_flags = latest_jog_command.flags;
          latest_jog_timeout_ns = latest_jog_command.timeout_ns;
          const uint32_t jog_flags = latest_jog_command.flags;
          const bool stop_requested =
              (jog_flags & gradient::ipc::v1::JOG_FLAG_STOP) != 0u;
          const bool quick_stop_requested =
              (jog_flags & gradient::ipc::v1::JOG_FLAG_QUICK_STOP) != 0u;
          if (stop_requested) {
            const uint32_t stop_axis_mask =
                active_jog_axis_mask != 0 ? active_jog_axis_mask : latest_jog_command.axis_mask;
            for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
              if ((stop_axis_mask & (1u << i)) != 0u) {
                if (quick_stop_requested) {
                  jog_stop_quick_stop_cycles_left[i] = std::max<uint16_t>(
                      jog_stop_quick_stop_cycles_left[i],
                      kJogQuickStopCycles);
                }
                jog_stop_arrest_cycles_left[i] = std::max<uint16_t>(
                    jog_stop_arrest_cycles_left[i],
                    kJogStopArrestCycles);
              }
            }
            snap_jog_hold_to_feedback_mask |= stop_axis_mask;
            last_jog_stop_reason = gradient::ipc::v1::JOG_STOP_REASON_CMD_STOP;
            last_jog_stop_time_ns = diag_now_ns;
            last_jog_stop_cmd_seq = active_jog_cmd_seq;
            last_jog_stop_axis_mask = stop_axis_mask;
            active_jog = false;
            active_jog_axis_mask = 0;
            active_jog_deadline_ns = 0;
            have_jog_target.fill(false);
            if (!active_trajectory) {
              motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_IDLE, std::memory_order_relaxed);
              motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_IDLE, std::memory_order_relaxed);
              motion_active_traj_id.store(0, std::memory_order_relaxed);
              motion_current_point_index.store(std::numeric_limits<uint32_t>::max(),
                                               std::memory_order_relaxed);
              motion_queue_depth.store(0, std::memory_order_relaxed);
              motion_last_event_code.store(0, std::memory_order_relaxed);
              motion_stale_command_flag.store(0, std::memory_order_relaxed);
              motion_done.store(1, std::memory_order_relaxed);
              motion_active_command_seq.store(active_jog_cmd_seq, std::memory_order_relaxed);
              motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
            }
          } else {
            active_jog = (jog_flags & gradient::ipc::v1::JOG_FLAG_ACTIVE) != 0u;
            active_jog_axis_mask = latest_jog_command.axis_mask;
            active_jog_deadline_ns =
                diag_now_ns + std::max<uint64_t>(latest_jog_command.timeout_ns, period);
            for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
              if ((active_jog_axis_mask & (1u << i)) != 0u) {
                jog_stop_quick_stop_cycles_left[i] = 0;
                jog_stop_arrest_cycles_left[i] = 0;
              }
              active_jog_velocity_counts_per_s[i] = latest_jog_command.velocity_counts_per_s[i];
              if ((active_jog_axis_mask & (1u << i)) == 0u) {
                have_jog_target[i] = false;
              }
            }
          }
        }

        if (active_trajectory && active_traj_point_count > 0) {
          std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> target_counts_interp{};
          auto load_point_targets = [&](const TrajectoryPointRuntime& point) {
            sp_mask = point.axis_mask;
            target_counts_interp = point.target_counts;
            target_velocity_counts_per_s = point.velocity_counts_per_s;
          };
          // The trajectory interpolation step picks the shortest-periodic
          // path between two queued waypoints and wraps the interpolated
          // output into [0, period). That is the right behavior for
          // wrapped-COMMAND axes (the host emits targets in [0, RM) and
          // the drive wraps the same way). For continuous-COMMAND axes
          // (A6-EC Absolute Position Rotation Mode per Chapter 5 Figure
          // 5-1) the host already emits a continuous, monotonic target
          // and the host's `_enforce_trajectory_step_within_half_rm`
          // guarantees consecutive waypoints stay within RM/2. Plain
          // linear interpolation between them produces the correct
          // continuous wire stream; wrapping it would resurrect the
          // 2026-04-19 Move-B+ "long-way around the seam" bug because
          // the per-cycle wrapped output discontinuity (+131 -> RM-20)
          // is interpreted by the drive as a forward leap of ~RM,
          // overriding C10.16=0 nearest-path selection.
          auto interpolation_wrap_period_for_axis = [&](uint32_t axis_i) -> uint32_t {
            if (axis_i >= opt.num_axes || axis_i >= gradient::ipc::v1::GRADIENT_MAX_AXES) {
              return 0;
            }
            return opt.axis[axis_i].command_counts_wrap
                ? wrapped_axis_period_counts(opt.axis[axis_i])
                : 0u;
          };
          auto segment_velocity_for_axis = [&](const TrajectoryPointRuntime& p0,
                                               const TrajectoryPointRuntime& p1,
                                               uint32_t axis_i) -> double {
            const uint64_t dt_ns = p1.t_from_start_ns - p0.t_from_start_ns;
            if (dt_ns == 0) {
              return 0.0;
            }
            const double dt_s = static_cast<double>(dt_ns) / 1e9;
            const bool p0_has_velocity =
                (p0.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) != 0u;
            const bool p1_has_velocity =
                (p1.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) != 0u;
            if (p0_has_velocity && p1_has_velocity) {
              return 0.5 * (p0.velocity_counts_per_s[axis_i] + p1.velocity_counts_per_s[axis_i]);
            }
            if (p0_has_velocity) {
              return p0.velocity_counts_per_s[axis_i];
            }
            if (p1_has_velocity) {
              return p1.velocity_counts_per_s[axis_i];
            }
            // Mirror the per-cycle interpolation policy: continuous-
            // command axes use the linear delta (host has already
            // emitted a continuous, monotonic ramp), wrapped-command
            // axes use the shortest-periodic delta (host emitted
            // wrapped targets that may straddle the seam).
            const uint32_t wrap_period_counts = interpolation_wrap_period_for_axis(axis_i);
            if (wrap_period_counts > 0) {
              const double period = static_cast<double>(wrap_period_counts);
              const double p0_wrapped =
                  wrap_counts_into_period_double(p0.target_counts[axis_i], period);
              const double p1_wrapped =
                  wrap_counts_into_period_double(p1.target_counts[axis_i], period);
              return shortest_periodic_error_counts_double(p1_wrapped - p0_wrapped, period) / dt_s;
            }
            return (p1.target_counts[axis_i] - p0.target_counts[axis_i]) / dt_s;
          };

          sp_mask = active_traj_axis_mask;
          if (active_traj_point_count == 1) {
            load_point_targets(active_traj_points[0]);
            motion_current_point_index.store(0, std::memory_order_relaxed);
            motion_queue_depth.store(0, std::memory_order_relaxed);
          } else {
            const uint64_t elapsed_ns =
                (diag_now_ns >= active_traj_start_ns) ? (diag_now_ns - active_traj_start_ns) : 0;
            while ((active_traj_segment_index + 1) < active_traj_point_count &&
                   elapsed_ns >= active_traj_points[active_traj_segment_index + 1].t_from_start_ns) {
              active_traj_segment_index += 1;
            }
            uint32_t current_index = active_traj_segment_index;
            if (current_index >= active_traj_point_count) {
              current_index = active_traj_point_count - 1;
            }
            motion_current_point_index.store(current_index, std::memory_order_relaxed);
            motion_queue_depth.store(
                (active_traj_point_count > current_index + 1)
                    ? (active_traj_point_count - current_index - 1)
                    : 0,
                std::memory_order_relaxed);
            if ((current_index + 1) < active_traj_point_count) {
              const auto& p0 = active_traj_points[current_index];
              const auto& p1 = active_traj_points[current_index + 1];
              if (p1.t_from_start_ns <= p0.t_from_start_ns) {
                logf("FAULT_EXEC_E1 diag_now_ns=%llu traj_id=%llu current_index=%u "
                     "p0_t_from_start_ns=%llu p1_t_from_start_ns=%llu",
                     static_cast<unsigned long long>(diag_now_ns),
                     static_cast<unsigned long long>(active_traj_id_rt),
                     static_cast<unsigned int>(current_index),
                     static_cast<unsigned long long>(p0.t_from_start_ns),
                     static_cast<unsigned long long>(p1.t_from_start_ns));
                active_trajectory = false;
                sp_mask = 0;
                motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_FAULTED, std::memory_order_relaxed);
                motion_last_event_code.store(
                    gradient::ipc::v1::EVT_TRAJECTORY_FAULTED, std::memory_order_relaxed);
                motion_done.store(1, std::memory_order_relaxed);
                motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
              } else if (elapsed_ns <= p0.t_from_start_ns) {
                load_point_targets(p0);
                for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                  if ((sp_mask & (1u << i)) == 0u) {
                    continue;
                  }
                  target_velocity_counts_per_s[i] = segment_velocity_for_axis(p0, p1, i);
                }
              } else if (elapsed_ns >= p1.t_from_start_ns) {
                load_point_targets(p1);
                for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                  if ((sp_mask & (1u << i)) == 0u) {
                    continue;
                  }
                  target_velocity_counts_per_s[i] = segment_velocity_for_axis(p0, p1, i);
                }
              } else {
                const double alpha =
                    static_cast<double>(elapsed_ns - p0.t_from_start_ns) /
                    static_cast<double>(p1.t_from_start_ns - p0.t_from_start_ns);
                sp_mask = p0.axis_mask | p1.axis_mask;
                for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                  if ((sp_mask & (1u << i)) == 0u) {
                    target_counts_interp[i] = 0.0;
                    target_velocity_counts_per_s[i] = 0.0;
                    continue;
                  }
                  const uint32_t wrap_period_counts = interpolation_wrap_period_for_axis(i);
                  if (wrap_period_counts > 0) {
                    const double period = static_cast<double>(wrap_period_counts);
                    const double p0_wrapped =
                        wrap_counts_into_period_double(p0.target_counts[i], period);
                    const double p1_wrapped =
                        wrap_counts_into_period_double(p1.target_counts[i], period);
                    const double delta =
                        shortest_periodic_error_counts_double(p1_wrapped - p0_wrapped, period);
                    target_counts_interp[i] =
                        wrap_counts_into_period_double(p0_wrapped + (delta * alpha), period);
                  } else {
                    target_counts_interp[i] =
                        p0.target_counts[i] + ((p1.target_counts[i] - p0.target_counts[i]) * alpha);
                  }
                  const bool p0_has_velocity =
                      (p0.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) != 0u;
                  const bool p1_has_velocity =
                      (p1.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) != 0u;
                  if (p0_has_velocity && p1_has_velocity) {
                    target_velocity_counts_per_s[i] =
                        p0.velocity_counts_per_s[i] +
                        ((p1.velocity_counts_per_s[i] - p0.velocity_counts_per_s[i]) * alpha);
                  } else {
                    target_velocity_counts_per_s[i] = segment_velocity_for_axis(p0, p1, i);
                  }
                }
              }
            } else {
              load_point_targets(active_traj_points[active_traj_point_count - 1]);
            }
          }
          for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
            if ((sp_mask & (1u << i)) == 0u) {
              target_counts[i] = 0;
              target_velocity_counts_per_s[i] = 0.0;
              continue;
            }
            // Only wrap the emitted CSP target into the single-turn
            // window when the axis profile requests it. Axes using
            // continuous-command emission (e.g. A6-EC Absolute Position
            // Rotation Mode per Chapter 5 Figure 5-1) let the
            // continuous host-space interpolation pass through to 0x607A
            // unchanged, so the drive can absorb seam crossings via its
            // own internal modulus + C10.16 shortest-path handling.
            const uint32_t command_wrap_period = i < opt.num_axes &&
                    opt.axis[i].command_counts_wrap
                ? wrapped_axis_period_counts(opt.axis[i])
                : 0u;
            if (command_wrap_period > 0) {
              target_counts_interp[i] = wrap_counts_into_period_double(
                  target_counts_interp[i], static_cast<double>(command_wrap_period));
            }
            target_counts[i] = clamp_round_to_i32(target_counts_interp[i]);
          }
        } else if (active_jog) {
          bool any_nonzero_velocity = false;
          for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
            if ((active_jog_axis_mask & (1u << i)) == 0u) {
              continue;
            }
            if (std::abs(active_jog_velocity_counts_per_s[i]) > 1e-6) {
              any_nonzero_velocity = true;
              break;
            }
          }

          if (active_jog_deadline_ns != 0 && diag_now_ns > active_jog_deadline_ns) {
            for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
              if ((active_jog_axis_mask & (1u << i)) != 0u) {
                jog_stop_quick_stop_cycles_left[i] = std::max<uint16_t>(
                    jog_stop_quick_stop_cycles_left[i],
                    kJogQuickStopCycles);
                jog_stop_arrest_cycles_left[i] = std::max<uint16_t>(
                    jog_stop_arrest_cycles_left[i],
                    kJogStopArrestCycles);
              }
            }
            snap_jog_hold_to_feedback_mask |= active_jog_axis_mask;
            last_jog_stop_reason = gradient::ipc::v1::JOG_STOP_REASON_TIMEOUT;
            last_jog_stop_time_ns = diag_now_ns;
            last_jog_stop_cmd_seq = active_jog_cmd_seq;
            last_jog_stop_axis_mask = active_jog_axis_mask;
            active_jog = false;
            active_jog_axis_mask = 0;
            active_jog_deadline_ns = 0;
            have_jog_target.fill(false);
            motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_JOG, std::memory_order_relaxed);
            motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_ABORTED, std::memory_order_relaxed);
            motion_active_traj_id.store(0, std::memory_order_relaxed);
            motion_current_point_index.store(std::numeric_limits<uint32_t>::max(),
                                             std::memory_order_relaxed);
            motion_queue_depth.store(0, std::memory_order_relaxed);
            motion_last_event_code.store(gradient::ipc::v1::EVT_JOG_TIMEOUT, std::memory_order_relaxed);
            motion_stale_command_flag.store(1, std::memory_order_relaxed);
            motion_done.store(1, std::memory_order_relaxed);
            motion_active_command_seq.store(active_jog_cmd_seq, std::memory_order_relaxed);
            motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          } else {
            motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_JOG, std::memory_order_relaxed);
            motion_exec_state.store(
                any_nonzero_velocity ? gradient::ipc::v1::EXEC_STATE_EXECUTING
                                     : gradient::ipc::v1::EXEC_STATE_ACCEPTED,
                std::memory_order_relaxed);
            motion_active_traj_id.store(0, std::memory_order_relaxed);
            motion_current_point_index.store(std::numeric_limits<uint32_t>::max(),
                                             std::memory_order_relaxed);
            motion_queue_depth.store(0, std::memory_order_relaxed);
            motion_last_event_code.store(0, std::memory_order_relaxed);
            motion_stale_command_flag.store(0, std::memory_order_relaxed);
            motion_done.store(0, std::memory_order_relaxed);
            motion_active_command_seq.store(active_jog_cmd_seq, std::memory_order_relaxed);
            motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);

            if (any_nonzero_velocity) {
              sp_mask = active_jog_axis_mask;
              for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                if ((active_jog_axis_mask & (1u << i)) == 0u) {
                  continue;
                }
                if (!have_jog_target[i]) {
                  jog_target_counts_float[i] = static_cast<double>(
                      csp_wire_counts_from_feedback(latest_feedback.pos_counts[i]));
                  have_jog_target[i] = true;
                }
                jog_target_counts_float[i] += active_jog_velocity_counts_per_s[i] * period_s;
                const uint32_t wrap_period_counts =
                    opt.axis[i].feedback_counts_wrap ? wrapped_axis_period_counts(opt.axis[i]) : 0;
                if (wrap_period_counts > 0) {
                  jog_target_counts_float[i] = wrap_counts_into_period_double(
                      jog_target_counts_float[i], static_cast<double>(wrap_period_counts));
                }
                target_counts[i] = static_cast<int32_t>(std::llround(jog_target_counts_float[i]));
                target_velocity_counts_per_s[i] = active_jog_velocity_counts_per_s[i];
              }
            }
          }
        }

        const bool is_armed = armed.load(std::memory_order_relaxed);
        const uint32_t en_mask = axis_enable_mask.load(std::memory_order_relaxed);

        // Latch any new fault reset requests from the helper thread.
        const uint32_t fr_req = fault_reset_request.exchange(0, std::memory_order_acq_rel);
        if (fr_req != 0) {
          for (uint32_t i = 0; i < opt.num_axes; ++i) {
            if ((fr_req & (1u << i)) != 0u) {
              fault_reset_left[i] = kFaultResetPulseCycles;
            }
          }
        }

        // Per-axis DS402 sequencing + CSP targets.
        uint32_t have_hold_mask = 0;
        uint32_t have_jog_target_mask = 0;
        uint32_t stop_arrest_mask = 0;
        for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
          if (jog_stop_arrest_cycles_left[i] > 0) {
            stop_arrest_mask |= (1u << i);
          }
        }
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          uint8_t* axis_pd = opt.split_domains_per_axis ? axis_domain_pd[i] : domain_pd;
          const uint16_t sw = EC_READ_U16(axis_pd + off[i].sw);
          const uint16_t err = EC_READ_U16(axis_pd + off[i].err);
          const uint32_t manufacturer_err = off[i].manufacturer_err != kInvalidOffset
                                              ? EC_READ_U32(axis_pd + off[i].manufacturer_err)
                                              : 0;
          const int32_t pos = EC_READ_S32(axis_pd + off[i].pos);
          const int16_t torque = off[i].torque != kInvalidOffset
                                     ? EC_READ_S16(axis_pd + off[i].torque)
                                     : 0;
          const uint8_t mode_disp = off[i].mode_disp != kInvalidOffset
                                        ? EC_READ_U8(axis_pd + off[i].mode_disp)
                                        : 0;
          const uint32_t di = EC_READ_U32(axis_pd + off[i].di);

          (void)err; // TODO: publish/report faults.

          const gradient::ds402::State st = gradient::ds402::decode_statusword(sw);
          uint16_t cw = 0;
          int32_t target_pos_out = pos;
          int32_t target_vel_out = 0;
          int16_t target_torque_out = 0;
          int8_t mode_out = 0;
          uint16_t tp_func_out = 0;
          uint32_t max_profile_vel_out = 0;
          const int32_t feedback_wire_target_counts = csp_wire_counts_from_feedback(pos);
          const bool service_mode_active =
              (service_mode_axis_mask.load(std::memory_order_relaxed) & (1u << i)) != 0u;
          const int32_t commanded_mode = desired_mode_of_operation[i].load(std::memory_order_relaxed);

          if (!startup_passive_active) {
            if (service_mode_active) {
              hold_target_counts[i] = feedback_wire_target_counts;
              have_hold[i] = false;
              cw = static_cast<uint16_t>(
                  service_controlword_override[i].load(std::memory_order_relaxed) & 0xFFFFu);
              target_pos_out = feedback_wire_target_counts;
              target_vel_out = 0;
              target_torque_out = 0;
              const int32_t service_mode = service_mode_override[i].load(std::memory_order_relaxed);
              mode_out = static_cast<int8_t>(
                  (service_mode == kNoModeOverride ? commanded_mode : service_mode));
              tp_func_out = 0;
              max_profile_vel_out = opt.axis[i].max_profile_vel_counts_per_s;
            } else {
              const bool want_enable = is_armed && ((en_mask & (1u << i)) != 0u);
              bool want_fault_reset = (fault_reset_left[i] > 0);
              const bool want_jog_quick_stop =
                  want_enable && (jog_stop_quick_stop_cycles_left[i] > 0) &&
                  !active_jog && !active_trajectory;
              if (want_fault_reset && st != gradient::ds402::State::Fault) {
                // Stop pulsing once the drive leaves FAULT (or if it never was in FAULT).
                fault_reset_left[i] = 0;
                want_fault_reset = false;
              }

              // Hold-target logic:
              // - While not operation-enabled (or not wanting enable), keep the drive-facing CSP target
              //   mirrored to raw feedback so 0x607A stays in the same wire frame as 0x6064.
              // - Once operation-enabled, latch once, then track commanded targets with optional per-cycle clamp.
              if ((snap_jog_hold_to_feedback_mask & (1u << i)) != 0u) {
                // When jog stops or expires we must immediately collapse the CSP hold target
                // onto live feedback, otherwise the axis can keep chasing the last jog target.
                hold_target_counts[i] = feedback_wire_target_counts;
                have_hold[i] = true;
              }
              if (jog_stop_arrest_cycles_left[i] > 0) {
                // Briefly keep re-latching hold to live feedback after a jog stop/timeout so
                // the drive does not keep chasing a stale frozen CSP target.
                hold_target_counts[i] = feedback_wire_target_counts;
                have_hold[i] = true;
              }
              if (want_enable && st == gradient::ds402::State::OperationEnabled) {
                if (!have_hold[i]) {
                  hold_target_counts[i] = feedback_wire_target_counts;
                  have_hold[i] = true;
                }
                if ((sp_mask & (1u << i)) != 0u) {
                  const int32_t desired = target_counts[i];
                  const int32_t max_step = opt.axis[i].max_step_counts_per_cycle;
                  const double desired_velocity = target_velocity_counts_per_s[i];
                  // Select the modulo frame for stepping based on the
                  // command-wrap policy:
                  //   - command_counts_wrap=true: step in [0, RM) using
                  //     shortest-periodic math so seam-adjacent targets
                  //     do not become synthetic near-one-revolution ramps
                  //     on the wire.
                  //   - command_counts_wrap=false: step in linear
                  //     continuous counts; 0x607A drifts with the
                  //     trajectory and the drive absorbs wrap internally.
                  //     The linear `max_step` clamp in
                  //     advance_csp_hold_target_counts still bounds the
                  //     per-cycle wire delta.
                  const uint32_t wrap_period_counts =
                      opt.axis[i].command_counts_wrap ? wrapped_axis_period_counts(opt.axis[i]) : 0;
                  hold_target_counts[i] = advance_csp_hold_target_counts(
                      hold_target_counts[i], desired, max_step, wrap_period_counts);
                  const double max_velocity_from_step = static_cast<double>(max_step) / period_s;
                  if (max_step > 0) {
                    if (desired_velocity > max_velocity_from_step) {
                      target_vel_out = clamp_round_to_i32(max_velocity_from_step);
                    } else if (desired_velocity < -max_velocity_from_step) {
                      target_vel_out = clamp_round_to_i32(-max_velocity_from_step);
                    } else {
                      target_vel_out = clamp_round_to_i32(desired_velocity);
                    }
                  } else {
                    target_vel_out = clamp_round_to_i32(desired_velocity);
                  }
                }
              } else {
                // Track feedback until OP; keeps 0x607A aligned during state transitions.
                hold_target_counts[i] = feedback_wire_target_counts;
                have_hold[i] = false;
              }
              cw = gradient::ds402::controlword_for_enable(st, want_enable, want_fault_reset);
              if (want_jog_quick_stop &&
                  (st == gradient::ds402::State::OperationEnabled ||
                   st == gradient::ds402::State::QuickStopActive)) {
                cw = gradient::ds402::CW_QUICK_STOP;
              }
              if (want_fault_reset && st == gradient::ds402::State::Fault && fault_reset_left[i] > 0) {
                fault_reset_left[i]--;
              }
              target_pos_out = hold_target_counts[i];
              target_torque_out = 0;
              mode_out = static_cast<int8_t>(commanded_mode);
              tp_func_out = 0;
              max_profile_vel_out = opt.axis[i].max_profile_vel_counts_per_s;
            }
          } else {
            // During passive startup we still exchange cyclic PDO frames, but avoid
            // DS402 state-driving writes so we can distinguish process-data transport
            // problems from higher-level controlword/mode/target sequencing problems.
            hold_target_counts[i] = feedback_wire_target_counts;
            have_hold[i] = false;
          }

          // Outputs (RxPDO 0x1702).
          EC_WRITE_U16(axis_pd + off[i].cw, cw);
          EC_WRITE_S32(axis_pd + off[i].target_pos, target_pos_out);
          if (off[i].target_vel != kInvalidOffset) {
            EC_WRITE_S32(axis_pd + off[i].target_vel, target_vel_out);
          }
          if (off[i].target_torque != kInvalidOffset) {
            EC_WRITE_S16(axis_pd + off[i].target_torque, target_torque_out);
          }
          if (off[i].mode != kInvalidOffset) {
            EC_WRITE_S8(axis_pd + off[i].mode, mode_out);
          }
          EC_WRITE_U16(axis_pd + off[i].tp_func, tp_func_out);
          if (off[i].max_profile_vel != kInvalidOffset) {
            EC_WRITE_U32(axis_pd + off[i].max_profile_vel, max_profile_vel_out);
          }

          output_target_counts[i] = target_pos_out;
          output_target_velocity_counts_per_s[i] = target_vel_out;
          if (have_hold[i]) {
            have_hold_mask |= (1u << i);
          }
          if (have_jog_target[i]) {
            have_jog_target_mask |= (1u << i);
          }
          if (jog_stop_quick_stop_cycles_left[i] > 0) {
            jog_stop_quick_stop_cycles_left[i]--;
          }
          if (jog_stop_arrest_cycles_left[i] > 0) {
            jog_stop_arrest_cycles_left[i]--;
          }

          // Publish raw feedback (counts + status) for STATUS_SNAPSHOT.
          latest_feedback.pos_counts[i] = pos;
          latest_feedback.torque_raw[i] = torque;
          latest_feedback.statusword[i] = sw;
          latest_feedback.error_code[i] = err;
          latest_feedback.manufacturer_error_code[i] = manufacturer_err;
          latest_feedback.mode_display[i] = mode_disp;
          latest_feedback.ds402_state[i] = static_cast<uint8_t>(st);
          latest_feedback.di_bits[i] = di;
          // Mirror the wire-frame 0x607A we just emitted so the fast-trace
          // thread can observe the command stream at cycle rate.
          latest_feedback.target_pos_counts[i] = target_pos_out;
        }

        latest_jog_debug.num_axes = opt.num_axes;
        latest_jog_debug.active_jog = active_jog ? 1u : 0u;
        latest_jog_debug.active_jog_axis_mask = active_jog_axis_mask;
        latest_jog_debug.command_sp_mask = sp_mask;
        latest_jog_debug.have_hold_mask = have_hold_mask;
        latest_jog_debug.have_jog_target_mask = have_jog_target_mask;
        latest_jog_debug.snap_hold_mask = snap_jog_hold_to_feedback_mask;
        latest_jog_debug.stop_arrest_mask = stop_arrest_mask;
        latest_jog_debug.latest_cmd_axis_mask = latest_jog_axis_mask;
        latest_jog_debug.latest_cmd_flags = latest_jog_flags;
        latest_jog_debug.last_stop_reason = last_jog_stop_reason;
        latest_jog_debug.last_stop_axis_mask = last_jog_stop_axis_mask;
        latest_jog_debug.sample_time_ns = diag_now_ns;
        latest_jog_debug.active_jog_cmd_seq = active_jog_cmd_seq;
        latest_jog_debug.latest_jog_seq_seen = active_jog_seq_seen;
        latest_jog_debug.active_jog_deadline_ns = active_jog_deadline_ns;
        latest_jog_debug.latest_cmd_timeout_ns = latest_jog_timeout_ns;
        latest_jog_debug.last_stop_time_ns = last_jog_stop_time_ns;
        latest_jog_debug.last_stop_cmd_seq = last_jog_stop_cmd_seq;
        latest_jog_debug.feedback_pos_counts = latest_feedback.pos_counts;
        latest_jog_debug.hold_target_counts = hold_target_counts;
        latest_jog_debug.output_target_counts = output_target_counts;
        latest_jog_debug.output_target_velocity_counts_per_s = output_target_velocity_counts_per_s;
        latest_jog_debug.seq.store(diag_now_ns, std::memory_order_release);

        if (active_trajectory && active_traj_point_count > 0) {
          motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_TRAJECTORY, std::memory_order_relaxed);
          motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_EXECUTING, std::memory_order_relaxed);
          motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          const auto& final_point = active_traj_points[active_traj_point_count - 1];
          const uint64_t elapsed_ns =
              (diag_now_ns >= active_traj_start_ns) ? (diag_now_ns - active_traj_start_ns) : 0;
          const bool final_due = elapsed_ns >= final_point.t_from_start_ns;
          bool any_faulted = false;
          uint32_t first_faulted_axis = std::numeric_limits<uint32_t>::max();
          uint16_t first_faulted_error_code = 0;
          uint8_t first_faulted_ds402_state = 0;
          bool all_axes_at_target = final_due;
          for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
            if ((final_point.axis_mask & (1u << i)) == 0u) {
              continue;
            }
            const uint8_t ds402 = latest_feedback.ds402_state[i];
            if (latest_feedback.error_code[i] != 0 ||
                ds402 == gradient::ipc::v1::DS402_FAULT ||
                ds402 == gradient::ipc::v1::DS402_FAULT_REACTION_ACTIVE) {
              any_faulted = true;
              if (first_faulted_axis == std::numeric_limits<uint32_t>::max()) {
                first_faulted_axis = i;
                first_faulted_error_code = latest_feedback.error_code[i];
                first_faulted_ds402_state = ds402;
              }
            }
            const uint32_t wrap_period_counts =
                opt.axis[i].feedback_counts_wrap ? wrapped_axis_period_counts(opt.axis[i]) : 0;
            int32_t final_target_counts = clamp_round_to_i32(final_point.target_counts[i]);
            if (wrap_period_counts > 0) {
              final_target_counts = wrap_counts_into_period(final_target_counts, wrap_period_counts);
            }
            const int32_t final_feedback_counts =
                csp_wire_counts_from_feedback(latest_feedback.pos_counts[i]);
            const int64_t error_counts =
                static_cast<int64_t>(final_feedback_counts) -
                static_cast<int64_t>(final_target_counts);
            const int64_t comparable_error_counts =
                wrap_period_counts > 0 ? shortest_periodic_error_counts(error_counts, wrap_period_counts)
                                       : error_counts;
            if (comparable_error_counts <
                    -static_cast<int64_t>(kTrajectoryCompletionToleranceCounts) ||
                comparable_error_counts >
                    static_cast<int64_t>(kTrajectoryCompletionToleranceCounts)) {
              all_axes_at_target = false;
            }
          }
          if (any_faulted) {
            logf("FAULT_EXEC_E2 diag_now_ns=%llu traj_id=%llu final_due=%u axis_index=%u "
                 "error_code=0x%04X ds402_state=%u",
                 static_cast<unsigned long long>(diag_now_ns),
                 static_cast<unsigned long long>(active_traj_id_rt),
                 final_due ? 1u : 0u,
                 static_cast<unsigned int>(first_faulted_axis),
                 static_cast<unsigned int>(first_faulted_error_code),
                 static_cast<unsigned int>(first_faulted_ds402_state));
            active_trajectory = false;
            motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_FAULTED, std::memory_order_relaxed);
            motion_last_event_code.store(
                gradient::ipc::v1::EVT_TRAJECTORY_FAULTED, std::memory_order_relaxed);
            motion_done.store(1, std::memory_order_relaxed);
            motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          } else if (all_axes_at_target) {
            active_trajectory = false;
            motion_current_point_index.store(active_traj_point_count - 1, std::memory_order_relaxed);
            motion_queue_depth.store(0, std::memory_order_relaxed);
            motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_COMPLETED, std::memory_order_relaxed);
            motion_last_event_code.store(
                gradient::ipc::v1::EVT_TRAJECTORY_COMPLETED, std::memory_order_relaxed);
            motion_done.store(1, std::memory_order_relaxed);
            motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          }
        }

        if (!startup_skip_domain_queue_active) {
          if (opt.split_domains_per_axis) {
            if (opt.queue_split_domains_round_robin) {
              const uint32_t axis_i = static_cast<uint32_t>((startup_cycle_counter - 1) % opt.num_axes);
              ecrt_domain_queue(axis_domain[axis_i]);
            } else {
              for (uint32_t i = 0; i < opt.num_axes; ++i) {
                ecrt_domain_queue(axis_domain[i]);
              }
            }
          } else {
            ecrt_domain_queue(domain);
          }
        }

        // DC sync helpers (13.4.4): keep reference clock stable.
        if (opt.use_dc) {
          static uint64_t dc_ref_sync_ctr = 0;
          if ((dc_ref_sync_ctr++ % 1000ULL) == 0) { // ~1 Hz
            ecrt_master_sync_reference_clock(master);
          }
          ecrt_master_sync_slave_clocks(master);
        }

        ecrt_master_send(master);

        // Snapshot master/domain state for diagnostics (non-RT users consume at 10 Hz).
        ecrt_master_state(master, &master_diag_state);
        aggregate_domain_state(&domain_state);
        uint32_t online_slaves = 0;
        uint32_t operational_slaves = 0;
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          sc_state[i] = ec_slave_config_state_t{};
          if (sc[i] && ecrt_slave_config_state(sc[i], &sc_state[i]) == 0) {
            if (sc_state[i].online) {
              online_slaves++;
            }
            if (sc_state[i].operational) {
              operational_slaves++;
            }
          }
          latest_feedback.slave_online[i] = sc_state[i].online ? 1 : 0;
          latest_feedback.slave_operational[i] = sc_state[i].operational ? 1 : 0;
          latest_feedback.slave_al_state[i] = static_cast<uint8_t>(sc_state[i].al_state);
        }

        const uint64_t startup_elapsed_ns = startup_elapsed_ns_pre;
        if (startup_elapsed_ns <= startup_detailed_diag_ns) {
          MasterDiagSnapshot cur_master_diag{};
          cur_master_diag.valid = true;
          cur_master_diag.link_up = master_diag_state.link_up != 0;
          cur_master_diag.responding = master_diag_state.slaves_responding;
          cur_master_diag.al_states = master_diag_state.al_states;
          cur_master_diag.domain_wc_state = static_cast<unsigned int>(domain_state.wc_state);
          cur_master_diag.domain_working_counter =
              static_cast<unsigned int>(domain_state.working_counter);
          cur_master_diag.online_slaves = online_slaves;
          cur_master_diag.operational_slaves = operational_slaves;

          if (!prev_master_diag.valid ||
              prev_master_diag.link_up != cur_master_diag.link_up ||
              prev_master_diag.responding != cur_master_diag.responding ||
              prev_master_diag.al_states != cur_master_diag.al_states ||
              prev_master_diag.domain_wc_state != cur_master_diag.domain_wc_state ||
              prev_master_diag.domain_working_counter != cur_master_diag.domain_working_counter ||
              prev_master_diag.online_slaves != cur_master_diag.online_slaves ||
              prev_master_diag.operational_slaves != cur_master_diag.operational_slaves) {
            logf("EtherCAT startup transition cycle=%llu elapsed_ms=%llu passive_outputs=%u "
                 "queue_suppressed=%u link_up=%u responding=%u/%u "
                 "online=%u/%u operational=%u/%u master_al=0x%x domain_wc=%u wkc=%u/%u",
                 static_cast<unsigned long long>(startup_cycle_counter),
                 static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
                 startup_passive_active ? 1u : 0u,
                 startup_skip_domain_queue_active ? 1u : 0u,
                 cur_master_diag.link_up ? 1u : 0u,
                 cur_master_diag.responding,
                 opt.num_axes,
                 cur_master_diag.online_slaves,
                 opt.num_axes,
                 cur_master_diag.operational_slaves,
                 opt.num_axes,
                 cur_master_diag.al_states,
                 cur_master_diag.domain_wc_state,
                 cur_master_diag.domain_working_counter,
                 expected_wkc);
            prev_master_diag = cur_master_diag;
          }

          for (uint32_t i = 0; i < opt.num_axes; ++i) {
            if (!sc[i]) {
              continue;
            }
            SlaveDiagSnapshot cur_slave_diag{};
            cur_slave_diag.valid = true;
            cur_slave_diag.online = sc_state[i].online != 0;
            cur_slave_diag.operational = sc_state[i].operational != 0;
            cur_slave_diag.al_state = static_cast<uint8_t>(sc_state[i].al_state);
            if (!prev_slave_diag[i].valid ||
                prev_slave_diag[i].online != cur_slave_diag.online ||
                prev_slave_diag[i].operational != cur_slave_diag.operational ||
                prev_slave_diag[i].al_state != cur_slave_diag.al_state) {
              logf("EtherCAT startup slave cycle=%llu elapsed_ms=%llu axis=%u slave_pos=%u online=%u "
                   "operational=%u al=%s(0x%x)",
                   static_cast<unsigned long long>(startup_cycle_counter),
                   static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
                   i,
                   static_cast<unsigned int>(opt.slave_position[i]),
                   cur_slave_diag.online ? 1u : 0u,
                   cur_slave_diag.operational ? 1u : 0u,
                   al_state_label(cur_slave_diag.al_state),
                   static_cast<unsigned int>(cur_slave_diag.al_state));
              prev_slave_diag[i] = cur_slave_diag;
            }
          }
        }
        const bool startup_process_data_live = domain_state.wc_state != EC_WC_ZERO;
        const bool startup_all_operational = operational_slaves >= opt.num_axes;
        if (startup_process_data_live && !startup_process_data_live_logged) {
          startup_process_data_live_logged = true;
          logf("EtherCAT startup process_data_live elapsed_ms=%llu passive_outputs=%u "
               "queue_suppressed=%u responding=%u/%u online=%u/%u operational=%u/%u "
               "master_al=0x%x domain_wc=%u wkc=%u/%u",
               static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.slaves_responding,
               opt.num_axes,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc);
        }
        startup_ready = master_diag_state.link_up &&
                        master_diag_state.slaves_responding >= opt.num_axes &&
                        online_slaves >= opt.num_axes &&
                        startup_all_operational &&
                        startup_process_data_live;

        const bool have_no_topology_progress =
            master_diag_state.slaves_responding == 0 || online_slaves == 0;

        if (!startup_ready && !startup_reset_issued &&
            have_no_topology_progress &&
            startup_elapsed_ns >= kStartupResetDelayNs) {
          const int rc = ecrt_master_reset(master);
          startup_reset_issued = true;
          if (rc == 0) {
            startup_reset_count++;
          }
          logf("EtherCAT startup reset attempt rc=%d elapsed_ms=%llu passive_outputs=%u "
               "queue_suppressed=%u responding=%u/%u online=%u/%u "
               "operational=%u/%u master_al=0x%x domain_wc=%u wkc=%u/%u",
               rc,
               static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.slaves_responding,
               opt.num_axes,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc);
          log_phase_summary("startup_reset", startup_elapsed_ns, startup_cycle_counter);
        }

        if (!startup_ready &&
            (!startup_wait_logged || startup_elapsed_ns - last_startup_log_ns >= kStartupLogIntervalNs)) {
          startup_wait_logged = true;
          last_startup_log_ns = startup_elapsed_ns;
          logf("EtherCAT startup waiting elapsed_ms=%llu passive_outputs=%u queue_suppressed=%u "
               "link_up=%u responding=%u/%u online=%u/%u "
               "operational=%u/%u master_al=0x%x domain_wc=%u wkc=%u/%u slave0_al=%s",
               static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.link_up ? 1u : 0u,
               master_diag_state.slaves_responding,
               opt.num_axes,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc,
               al_state_label(static_cast<uint8_t>(sc_state[0].al_state)));
        }

        if (startup_ready && !startup_ready_logged) {
          startup_ready_logged = true;
          logf("EtherCAT startup converged elapsed_ms=%llu passive_outputs=%u queue_suppressed=%u "
               "responding=%u/%u online=%u/%u "
               "operational=%u/%u master_al=0x%x domain_wc=%u wkc=%u/%u",
               static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.slaves_responding,
               opt.num_axes,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc);
        }

        if (!startup_ready && !startup_timeout_logged && startup_elapsed_ns >= startup_timeout_ns) {
          startup_timeout_logged = true;
          logf("WARNING: EtherCAT startup did not converge within %llu ms: passive_outputs=%u "
               "queue_suppressed=%u link_up=%u responding=%u/%u "
               "online=%u/%u operational=%u/%u master_al=0x%x domain_wc=%u wkc=%u/%u",
               static_cast<unsigned long long>(startup_timeout_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.link_up ? 1u : 0u,
               master_diag_state.slaves_responding,
               opt.num_axes,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc);
          log_phase_summary("startup_timeout", startup_elapsed_ns, startup_cycle_counter);
          if (master_diag_state.slaves_responding > 0 && !startup_process_data_live) {
            logf("WARNING: EtherCAT link is up and slaves respond, but process-data WKC is zero. "
                 "Likely slave state/PDO configuration mismatch; inspect AL states and live PDO assignments.");
          } else if (startup_process_data_live && !startup_all_operational) {
            logf("WARNING: EtherCAT process-data is alive, but only %u/%u selected slaves are "
                 "operational. Startup is still waiting for full OP.",
                 operational_slaves,
                 opt.num_axes);
          }
        }

        latest_feedback.wkc_actual = static_cast<uint32_t>(domain_state.working_counter);
        latest_feedback.wkc_expected = expected_wkc;
        latest_feedback.master_state =
            master_state_from_al_states(master_diag_state.al_states, master_diag_state.link_up != 0);
        latest_feedback.responding_slaves = master_diag_state.slaves_responding;
        latest_feedback.online_slaves = online_slaves;
        latest_feedback.operational_slaves = operational_slaves;
        // TODO: fill dc_offset_ns from IgH reference clock delta.
        latest_feedback.dc_offset_ns = 0;
        latest_feedback.link_up = master_diag_state.link_up ? 1 : 0;
        latest_feedback.master_al_states = static_cast<uint8_t>(master_diag_state.al_states);
        latest_feedback.domain_wc_state = static_cast<uint8_t>(domain_state.wc_state);
        latest_feedback.startup_ready = startup_ready ? 1 : 0;
        latest_feedback.startup_passive_active = startup_passive_active ? 1 : 0;
        latest_feedback.startup_skip_domain_queue_active = startup_skip_domain_queue_active ? 1 : 0;
        latest_feedback.startup_elapsed_ms = static_cast<uint32_t>(startup_elapsed_ns / 1000000ULL);
        latest_feedback.startup_reset_count = startup_reset_count;
        // Wakeup jitter relative to the requested absolute schedule.
        latest_feedback.cycle_jitter_ns = jitter_ns;
        static uint64_t fb_seq = 1;
        latest_feedback.seq.store(fb_seq++, std::memory_order_release);
      }
#endif  // GRADIENT_HAVE_ECRT

      // Exit after grace window has elapsed (or immediately if no grace requested).
      if (g_stop.load(std::memory_order_relaxed)) {
        if (!shutdown_active || next_ns >= shutdown_until_ns) {
          break;
        }
      }

      rt_cycle_counter.fetch_add(1, std::memory_order_relaxed);

      timespec ts{};
      ts.tv_sec = static_cast<time_t>(next_ns / 1000000000ULL);
      ts.tv_nsec = static_cast<long>(next_ns % 1000000000ULL);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    }

#if GRADIENT_HAVE_ECRT
    // Best-effort cleanup. This is not RT-critical (we're shutting down), and it
    // allows IgH to deactivate the master configuration cleanly.
    if (master) {
      std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
      if (master) {
        ecrt_release_master(master); // deactivates if active
        master = nullptr;
      }
    }
#endif
  });

  // Optional fast-trace writer. Dedicated thread that snapshots the small set
  // of per-axis feedback fields that the RT thread already refreshes every
  // cycle (1 kHz) and writes a compact JSONL line at up to the configured Hz
  // (capped at cycle rate). Designed for J6 seam/whip post-mortem where the
  // coarse 5 Hz metrics.json cadence is inadequate. Disabled by default.
  //
  // Schema (one JSON object per line):
  //   { "t_ns": <monotonic ns>, "seq": <latest_feedback seq>,
  //     "ax": [ { "i": axis_idx,
  //               "p": pos_counts (6064),
  //               "tp": target_pos_counts (last 607A written),
  //               "sw": statusword (6041),
  //               "er": error_code (603F),
  //               "mfr": manufacturer_error_code,
  //               "af": [ { "k": key, "v": value, "ok": valid }, ... ] }, ... ] }
  //
  // The absolute_feedback (e.g. U40.20 / U40.22) stays at whatever rate
  // RTCore's existing SDO poll refreshes (currently ~5 Hz) so consecutive
  // fast-trace samples will typically show the same absolute_feedback value
  // for ~200 ms at a time. Post-processors should treat "af" as a ground-truth
  // anchor and reconstruct per-cycle multi-turn motion from the 1 kHz
  // "p" (6064) stream using shortest-periodic deltas.
  std::thread fast_trace_thread;
  const bool fast_trace_enabled =
      !opt.fast_trace_path.empty() && opt.fast_trace_hz > 0;
  if (fast_trace_enabled) {
    fast_trace_thread = std::thread([&]() {
      pthread_setname_np(pthread_self(), "fasttrace");

      // Axis selection: zero mask = all axes (compat with single-client).
      const uint32_t axis_mask =
          (opt.fast_trace_axis_mask == 0)
              ? ((1u << opt.num_axes) - 1u)
              : (opt.fast_trace_axis_mask & ((1u << opt.num_axes) - 1u));
      if (axis_mask == 0) {
        logf("fast_trace: axis_mask resolved to 0 after clamp; disabling");
        return;
      }

      // Period clamped to cycle rate (writing faster than the RT loop
      // updates feedback would just duplicate samples).
      uint64_t period_ns = 1000000000ULL / opt.fast_trace_hz;
      if (period_ns < opt.cycle_ns) {
        period_ns = opt.cycle_ns;
      }

      FILE* fp = std::fopen(opt.fast_trace_path.c_str(), "w");
      if (!fp) {
        logf("ERROR: fast_trace: could not open %s: %s",
             opt.fast_trace_path.c_str(), std::strerror(errno));
        return;
      }
      // Line-buffered so `tail -f` sees data promptly.
      std::setvbuf(fp, nullptr, _IOLBF, 0);
      logf("fast_trace: writing to %s at %u Hz (period %llu ns) mask=0x%x",
           opt.fast_trace_path.c_str(),
           static_cast<unsigned int>(opt.fast_trace_hz),
           static_cast<unsigned long long>(period_ns),
           static_cast<unsigned int>(axis_mask));

      struct timespec next{};
      clock_gettime(CLOCK_MONOTONIC, &next);

      auto add_ns = [](struct timespec& ts, uint64_t ns) {
        ts.tv_nsec += static_cast<long>(ns % 1000000000ULL);
        ts.tv_sec += static_cast<time_t>(ns / 1000000000ULL);
        if (ts.tv_nsec >= 1000000000L) {
          ts.tv_sec += ts.tv_nsec / 1000000000L;
          ts.tv_nsec = ts.tv_nsec % 1000000000L;
        }
      };

      uint64_t samples_written = 0;
      uint64_t samples_skipped_inconsistent = 0;
      uint64_t samples_skipped_no_fb = 0;

      while (!g_stop.load(std::memory_order_relaxed)) {
        add_ns(next, period_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

        // Consistent snapshot via seq double-read. If the RT thread races
        // through a publish between our reads, skip this sample rather than
        // publish torn state.
        const uint64_t s1 =
            latest_feedback.seq.load(std::memory_order_acquire);
        if (s1 == 0) {
          ++samples_skipped_no_fb;
          continue;
        }
        std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pos{};
        std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> tgt{};
        std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> sw{};
        std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> er{};
        std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> mfr{};
        std::array<AbsoluteFeedbackAxis, gradient::ipc::v1::GRADIENT_MAX_AXES> af{};
        pos = latest_feedback.pos_counts;
        tgt = latest_feedback.target_pos_counts;
        sw = latest_feedback.statusword;
        er = latest_feedback.error_code;
        mfr = latest_feedback.manufacturer_error_code;
        af = latest_feedback.absolute_feedback;
        const uint64_t s2 =
            latest_feedback.seq.load(std::memory_order_acquire);
        if (s1 != s2) {
          ++samples_skipped_inconsistent;
          continue;
        }

        const uint64_t t_ns = now_monotonic_ns();

        std::fprintf(fp, "{\"t_ns\":%llu,\"seq\":%llu,\"ax\":[",
                     static_cast<unsigned long long>(t_ns),
                     static_cast<unsigned long long>(s1));
        bool first_axis = true;
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          if ((axis_mask & (1u << i)) == 0) {
            continue;
          }
          if (!first_axis) {
            std::fputc(',', fp);
          }
          first_axis = false;
          std::fprintf(fp,
                       "{\"i\":%u,\"p\":%d,\"tp\":%d,\"sw\":%u,\"er\":%u,\"mfr\":%u",
                       static_cast<unsigned int>(i),
                       pos[i], tgt[i],
                       static_cast<unsigned int>(sw[i]),
                       static_cast<unsigned int>(er[i]),
                       static_cast<unsigned int>(mfr[i]));
          if (absolute_feedback_cfg.valid &&
              absolute_feedback_cfg.field_count > 0) {
            std::fprintf(fp, ",\"af\":[");
            bool first_field = true;
            for (uint32_t field_i = 0;
                 field_i < absolute_feedback_cfg.field_count; ++field_i) {
              const auto& field_cfg = absolute_feedback_cfg.fields[field_i];
              if (!field_cfg.valid || field_cfg.key.empty()) {
                continue;
              }
              if (!first_field) {
                std::fputc(',', fp);
              }
              first_field = false;
              const auto& field = af[i].fields[field_i];
              std::fprintf(fp,
                           "{\"k\":\"%s\",\"v\":%d,\"ok\":%u}",
                           field_cfg.key.c_str(),
                           field.value,
                           static_cast<unsigned int>(field.valid));
            }
            std::fputc(']', fp);
          }
          std::fputc('}', fp);
        }
        std::fputs("]}\n", fp);
        ++samples_written;
      }

      std::fflush(fp);
      std::fclose(fp);
      logf("fast_trace: closed; written=%llu skipped_inconsistent=%llu skipped_no_fb=%llu",
           static_cast<unsigned long long>(samples_written),
           static_cast<unsigned long long>(samples_skipped_inconsistent),
           static_cast<unsigned long long>(samples_skipped_no_fb));
    });
  }

  // Periodic metrics file for dashboards/monitoring. This intentionally does NOT
  // require an IPC client (RTCore is single-client today).
  const std::filesystem::path sock_parent = std::filesystem::path(opt.socket_path).parent_path();
  const std::filesystem::path metrics_path =
      (sock_parent.empty() ? std::filesystem::path("/tmp")
                           : sock_parent) /
      "metrics.json";
  std::thread metrics_thread([&]() {
    pthread_setname_np(pthread_self(), "metrics");

    uint64_t last_cycles = rt_cycle_counter.load(std::memory_order_relaxed);
    uint64_t last_time_ns = now_monotonic_ns();
    uint64_t last_warn_ns = 0;
    bool startup_readback_complete =
        startup_sdos.empty() || !metrics_startup_readback_enabled;
    bool native_home_offset_refresh_complete =
        !native_home_cfg.valid || !metrics_native_home_refresh_enabled;
    uint64_t startup_readback_ready_since_ns = 0;
    uint64_t native_home_offset_ready_since_ns = 0;
    uint64_t absolute_feedback_ready_since_ns = 0;
    uint64_t absolute_feedback_last_poll_ns = 0;
    bool startup_state_observed = false;
    bool last_startup_ready_flag = false;
    uint32_t last_startup_reset_count = 0;
    constexpr uint64_t kStartupReadbackDelayNs = 500000000ULL; // 500ms after startup_ready
    constexpr uint64_t kAbsoluteFeedbackPollIntervalNs = 200000000ULL; // 200ms

    auto reset_startup_drive_config_feedback = [&](auto& feedback_store) {
      feedback_store.fill(StartupSdoFeedback{});
      for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
        const auto& descriptor = startup_sdos[descriptor_i];
        auto& feedback = feedback_store[descriptor_i];
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          feedback.configured[i] = descriptor.valid ? 1u : 0u;
          feedback.commanded[i] = descriptor.valid ? descriptor.values[i] : 0u;
        }
      }
    };

    auto perform_startup_drive_config_readback = [&](uint64_t now_ns, bool startup_ready_flag) {
#if GRADIENT_HAVE_ECRT
      if (!metrics_startup_readback_enabled) {
        startup_readback_complete = true;
        return;
      }
      if (startup_readback_complete || startup_sdos.empty()) {
        return;
      }
      if (!startup_ready_flag) {
        startup_readback_ready_since_ns = 0;
        return;
      }
      if (startup_readback_ready_since_ns == 0) {
        startup_readback_ready_since_ns = now_ns;
        return;
      }
      if (now_ns - startup_readback_ready_since_ns < kStartupReadbackDelayNs) {
        return;
      }

      startup_readback_complete = true;
      constexpr int kStartupReadbackAttempts = 5;
      for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
        const auto& descriptor = startup_sdos[descriptor_i];
        auto& feedback = latest_feedback.startup_drive_config_feedback[descriptor_i];
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          feedback.configured[i] = descriptor.valid ? 1u : 0u;
          feedback.commanded[i] = descriptor.valid ? descriptor.values[i] : 0u;
        }
        if (descriptor.type != StartupSdoValueType::kU16) {
          logf("WARNING: EtherCAT startup readback skipped for unsupported type key=%s",
               descriptor.key.c_str());
          for (uint32_t i = 0; i < opt.num_axes; ++i) {
            feedback.readback_valid[i] = 0u;
            feedback.readback[i] = 0u;
            feedback.verified[i] = 0u;
          }
          continue;
        }
        logf("EtherCAT startup readback begin key=%s delay_ms=%llu",
             descriptor.key.c_str(),
             static_cast<unsigned long long>(kStartupReadbackDelayNs / 1000000ULL));
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          bool verified = false;
          for (int attempt = 1; attempt <= kStartupReadbackAttempts; ++attempt) {
            uint8_t readback_raw[sizeof(uint16_t)] = {};
            size_t readback_size = 0;
            uint32_t abort_code = 0;
            int rc = -1;
            {
              std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
              if (g_stop.load(std::memory_order_relaxed) || !shared_master) {
                return;
              }
              rc = ecrt_master_sdo_upload(
                  shared_master,
                  opt.slave_position[i],
                  descriptor.index,
                  descriptor.subindex,
                  readback_raw,
                  sizeof(readback_raw),
                  &readback_size,
                  &abort_code);
            }
            if (rc == 0 && readback_size >= sizeof(uint16_t)) {
              const uint16_t readback_value = EC_READ_U16(readback_raw);
              const bool readback_matches =
                  readback_value == static_cast<uint16_t>(descriptor.values[i]);
              feedback.readback_valid[i] = 1u;
              feedback.readback[i] = readback_value;
              feedback.verified[i] = readback_matches ? 1u : 0u;
              logf("EtherCAT startup readback axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x commanded=%u readback=%u verified=%u",
                   i,
                   static_cast<unsigned int>(opt.slave_position[i]),
                   descriptor.key.c_str(),
                   static_cast<unsigned int>(descriptor.index),
                   static_cast<unsigned int>(descriptor.subindex),
                   static_cast<unsigned int>(descriptor.values[i]),
                   static_cast<unsigned int>(readback_value),
                   readback_matches ? 1u : 0u);
              verified = true;
              break;
            }
            if (attempt < kStartupReadbackAttempts) {
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (attempt == kStartupReadbackAttempts) {
              feedback.readback_valid[i] = 0u;
              feedback.readback[i] = 0u;
              feedback.verified[i] = 0u;
              logf("WARNING: EtherCAT startup readback failed for axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x rc=%d abort=0x%08x size=%zu",
                   i,
                   static_cast<unsigned int>(opt.slave_position[i]),
                   descriptor.key.c_str(),
                   static_cast<unsigned int>(descriptor.index),
                   static_cast<unsigned int>(descriptor.subindex),
                   rc,
                   static_cast<unsigned int>(abort_code),
                   readback_size);
            }
          }
          if (!verified) {
            feedback.readback_valid[i] = 0u;
            feedback.readback[i] = 0u;
            feedback.verified[i] = 0u;
          }
        }
      }
#else
      (void)now_ns;
      (void)startup_ready_flag;
      startup_readback_complete = true;
#endif
    };

    auto perform_startup_native_home_offset_refresh = [&](uint64_t now_ns,
                                                          bool startup_ready_flag) {
#if GRADIENT_HAVE_ECRT
      if (!metrics_native_home_refresh_enabled) {
        native_home_offset_refresh_complete = true;
        return;
      }
      if (native_home_offset_refresh_complete || !native_home_cfg.valid) {
        return;
      }
      if (!startup_ready_flag) {
        native_home_offset_ready_since_ns = 0;
        return;
      }
      if (native_home_offset_ready_since_ns == 0) {
        native_home_offset_ready_since_ns = now_ns;
        return;
      }
      if (now_ns - native_home_offset_ready_since_ns < kStartupReadbackDelayNs) {
        return;
      }

      logf("EtherCAT native-home truth refresh begin delay_ms=%llu",
           static_cast<unsigned long long>(kStartupReadbackDelayNs / 1000000ULL));

      constexpr int kNativeHomeOffsetReadAttempts = 5;
      bool all_refreshed = true;
      for (uint32_t i = 0; i < opt.num_axes; ++i) {
        bool refreshed = false;
        for (int attempt = 1; attempt <= kNativeHomeOffsetReadAttempts; ++attempt) {
          int32_t refreshed_offset = 0;
          if (read_native_home_truth_axis(i, &refreshed_offset)) {
            latest_feedback.native_home_position_offset[i] = refreshed_offset;
            logf("EtherCAT native-home truth refresh axis=%u slave_pos=%u value=%d attempt=%d",
                 i,
                 static_cast<unsigned int>(opt.slave_position[i]),
                 refreshed_offset,
                 attempt);
            refreshed = true;
            break;
          }
          if (attempt < kNativeHomeOffsetReadAttempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
        }
        if (!refreshed) {
          all_refreshed = false;
          logf("WARNING: EtherCAT native-home truth refresh unavailable axis=%u slave_pos=%u; keeping in-memory value=%d",
               i,
               static_cast<unsigned int>(opt.slave_position[i]),
               latest_feedback.native_home_position_offset[i]);
        }
      }
      native_home_offset_refresh_complete = all_refreshed;
      if (!all_refreshed) {
        native_home_offset_ready_since_ns = now_ns;
      }
#else
      (void)now_ns;
      (void)startup_ready_flag;
      native_home_offset_refresh_complete = true;
#endif
    };

    auto perform_absolute_feedback_refresh = [&](uint64_t now_ns, bool startup_ready_flag) {
#if GRADIENT_HAVE_ECRT
      if (!metrics_absolute_feedback_poll_enabled) {
        absolute_feedback_ready_since_ns = 0;
        absolute_feedback_last_poll_ns = 0;
        latest_feedback.absolute_feedback.fill(AbsoluteFeedbackAxis{});
        return;
      }
      if (!absolute_feedback_cfg.valid || absolute_feedback_cfg.field_count == 0) {
        return;
      }
      if (!startup_ready_flag) {
        absolute_feedback_ready_since_ns = 0;
        absolute_feedback_last_poll_ns = 0;
        latest_feedback.absolute_feedback.fill(AbsoluteFeedbackAxis{});
        return;
      }
      if (absolute_feedback_ready_since_ns == 0) {
        absolute_feedback_ready_since_ns = now_ns;
        return;
      }
      if (now_ns - absolute_feedback_ready_since_ns < kStartupReadbackDelayNs) {
        return;
      }
      if (absolute_feedback_last_poll_ns != 0 &&
          now_ns - absolute_feedback_last_poll_ns < kAbsoluteFeedbackPollIntervalNs) {
        return;
      }
      absolute_feedback_last_poll_ns = now_ns;

      for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
        AbsoluteFeedbackAxis axis_feedback{};
        for (uint32_t field_i = 0; field_i < absolute_feedback_cfg.field_count; ++field_i) {
          const auto& field_cfg = absolute_feedback_cfg.fields[field_i];
          if (!field_cfg.valid || !field_cfg.object.valid) {
            continue;
          }
          (void)read_absolute_feedback_field_axis(
              i,
              field_cfg.object.index,
              field_cfg.object.subindex,
              field_cfg.object.type,
              &axis_feedback.fields[field_i]);
        }
        latest_feedback.absolute_feedback[i] = axis_feedback;
      }
#else
      (void)now_ns;
      (void)startup_ready_flag;
#endif
    };

    while (!g_stop.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      const uint64_t now_ns = now_monotonic_ns();

      const uint64_t cycles = rt_cycle_counter.load(std::memory_order_relaxed);
      const int64_t last_jitter = rt_last_jitter_ns.load(std::memory_order_relaxed);
      const int64_t max_abs_jitter = rt_max_abs_jitter_ns.load(std::memory_order_relaxed);
      const uint64_t overruns = rt_overrun_count.load(std::memory_order_relaxed);

      double hz = 0.0;
      if (now_ns > last_time_ns) {
        const uint64_t dc = cycles - last_cycles;
        const uint64_t dt = now_ns - last_time_ns;
        hz = (static_cast<double>(dc) * 1e9) / static_cast<double>(dt);
      }
      last_cycles = cycles;
      last_time_ns = now_ns;

      // Snapshot EtherCAT feedback if available (double-read seq).
      uint32_t wkc_actual = 0;
      uint32_t wkc_expected = 2 * opt.num_axes;
      uint32_t master_state = gradient::ipc::v1::MASTER_INIT;
      uint32_t responding_slaves = 0;
      uint32_t online_slaves = 0;
      uint32_t operational_slaves = 0;
      uint32_t startup_elapsed_ms = 0;
      uint32_t startup_reset_count = 0;
      uint8_t link_up = 0;
      uint8_t master_al_states = 0;
      uint8_t domain_wc_state = 0;
      uint8_t startup_ready = 0;
      uint8_t startup_passive_active = 0;
      uint8_t startup_skip_domain_queue_active = 0;
      std::array<StartupSdoFeedback, kMaxStartupSdoDescriptors> startup_drive_config_feedback{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_state{};
      std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_position_offset{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_last_abort_code{};
      std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pos_counts{};
      std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> statusword{};
      std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> error_code{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> manufacturer_error_code{};
      std::array<AbsoluteFeedbackAxis, gradient::ipc::v1::GRADIENT_MAX_AXES> absolute_feedback{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_al_state{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_online{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_operational{};

      bool have_fb = false;
      const uint64_t s1 = latest_feedback.seq.load(std::memory_order_acquire);
      if (s1 != 0) {
        wkc_actual = latest_feedback.wkc_actual;
        wkc_expected = latest_feedback.wkc_expected;
        master_state = latest_feedback.master_state;
        responding_slaves = latest_feedback.responding_slaves;
        online_slaves = latest_feedback.online_slaves;
        operational_slaves = latest_feedback.operational_slaves;
        startup_elapsed_ms = latest_feedback.startup_elapsed_ms;
        startup_reset_count = latest_feedback.startup_reset_count;
        link_up = latest_feedback.link_up;
        master_al_states = latest_feedback.master_al_states;
        domain_wc_state = latest_feedback.domain_wc_state;
        startup_ready = latest_feedback.startup_ready;
        startup_passive_active = latest_feedback.startup_passive_active;
        startup_skip_domain_queue_active = latest_feedback.startup_skip_domain_queue_active;
        startup_drive_config_feedback = latest_feedback.startup_drive_config_feedback;
        native_home_state = latest_feedback.native_home_state;
        native_home_position_offset = latest_feedback.native_home_position_offset;
        native_home_last_abort_code = latest_feedback.native_home_last_abort_code;
        pos_counts = latest_feedback.pos_counts;
        statusword = latest_feedback.statusword;
        error_code = latest_feedback.error_code;
        manufacturer_error_code = latest_feedback.manufacturer_error_code;
        absolute_feedback = latest_feedback.absolute_feedback;
        slave_al_state = latest_feedback.slave_al_state;
        slave_online = latest_feedback.slave_online;
        slave_operational = latest_feedback.slave_operational;
        const uint64_t s2 = latest_feedback.seq.load(std::memory_order_acquire);
        have_fb = (s1 == s2);
      }

      const bool startup_ready_flag = startup_ready != 0;
      if (!startup_state_observed) {
        startup_state_observed = true;
        last_startup_ready_flag = startup_ready_flag;
        last_startup_reset_count = startup_reset_count;
      } else if (startup_reset_count != last_startup_reset_count ||
                 (last_startup_ready_flag && !startup_ready_flag)) {
        startup_readback_complete =
            startup_sdos.empty() || !metrics_startup_readback_enabled;
        native_home_offset_refresh_complete =
            !native_home_cfg.valid || !metrics_native_home_refresh_enabled;
        startup_readback_ready_since_ns = 0;
        native_home_offset_ready_since_ns = 0;
        absolute_feedback_ready_since_ns = 0;
        absolute_feedback_last_poll_ns = 0;
        reset_startup_drive_config_feedback(latest_feedback.startup_drive_config_feedback);
        latest_feedback.native_home_state.fill(
            static_cast<uint8_t>(gradient::ipc::v1::NATIVE_HOME_STATE_IDLE));
        latest_feedback.native_home_last_abort_code.fill(0u);
        latest_feedback.absolute_feedback.fill(AbsoluteFeedbackAxis{});
        reset_startup_drive_config_feedback(startup_drive_config_feedback);
        native_home_state.fill(
            static_cast<uint8_t>(gradient::ipc::v1::NATIVE_HOME_STATE_IDLE));
        native_home_last_abort_code.fill(0u);
        logf("EtherCAT startup epoch changed: ready=%u->%u reset_count=%u->%u; rearming startup readback/offset refresh",
             last_startup_ready_flag ? 1u : 0u,
             startup_ready_flag ? 1u : 0u,
             static_cast<unsigned int>(last_startup_reset_count),
             static_cast<unsigned int>(startup_reset_count));
        last_startup_ready_flag = startup_ready_flag;
        last_startup_reset_count = startup_reset_count;
      } else {
        last_startup_ready_flag = startup_ready_flag;
        last_startup_reset_count = startup_reset_count;
      }

      perform_startup_drive_config_readback(now_ns, startup_ready_flag);
      perform_startup_native_home_offset_refresh(now_ns, startup_ready_flag);
      perform_absolute_feedback_refresh(now_ns, startup_ready_flag);
      startup_drive_config_feedback = latest_feedback.startup_drive_config_feedback;
      native_home_position_offset = latest_feedback.native_home_position_offset;
      absolute_feedback = latest_feedback.absolute_feedback;

      const uint32_t en_mask = axis_enable_mask.load(std::memory_order_relaxed);

      std::ostringstream oss;
      oss << "{";
      oss << "\"time_ns\":" << now_ns << ",";
      oss << "\"cycle_ns\":" << opt.cycle_ns << ",";
      oss << "\"num_axes\":" << opt.num_axes << ",";
      oss << "\"slave_vendor_id\":" << opt.slave_vendor_id << ",";
      oss << "\"slave_product_code\":" << opt.slave_product_code << ",";
      oss << "\"slave_revision_no\":" << opt.slave_revision_no << ",";
      oss << "\"pdo_profile\":\"" << drive_pdo.label << "\",";
      oss << "\"rx_pdo\":" << opt.rx_pdo << ",";
      oss << "\"tx_pdo\":" << opt.tx_pdo << ",";
      oss << "\"dc_enabled\":" << (opt.use_dc ? 1 : 0) << ",";
      oss << "\"output_watchdog_enabled\":" << (opt.disable_output_watchdog ? 0 : 1) << ",";
      oss << "\"split_domains_per_axis\":" << (opt.split_domains_per_axis ? 1 : 0) << ",";
      oss << "\"queue_split_domains_round_robin\":"
          << (opt.queue_split_domains_round_robin ? 1 : 0) << ",";
      oss << "\"explicit_pdo_config\":" << (opt.explicit_pdo_config ? 1 : 0) << ",";
      oss << "\"wait_before_safeop_ms\":" << opt.wait_before_safeop_ms << ",";
      oss << "\"preop_to_safeop_timeout_ms\":" << opt.preop_to_safeop_timeout_ms << ",";
      oss << "\"safeop_to_op_timeout_ms\":" << opt.safeop_to_op_timeout_ms << ",";
      oss << "\"startup_passive_ms\":" << opt.startup_passive_ms << ",";
      oss << "\"startup_passive_active\":" << static_cast<unsigned int>(startup_passive_active) << ",";
      oss << "\"startup_skip_domain_queue_ms\":" << opt.startup_skip_domain_queue_ms << ",";
      oss << "\"startup_skip_domain_queue_active\":"
          << static_cast<unsigned int>(startup_skip_domain_queue_active) << ",";
      oss << "\"metrics_startup_readback_enabled\":"
          << (metrics_startup_readback_enabled ? 1 : 0) << ",";
      oss << "\"metrics_native_home_refresh_enabled\":"
          << (metrics_native_home_refresh_enabled ? 1 : 0) << ",";
      oss << "\"metrics_absolute_feedback_poll_enabled\":"
          << (metrics_absolute_feedback_poll_enabled ? 1 : 0) << ",";
      oss << "\"slave_positions\":[";
      for (uint32_t i = 0; i < opt.num_axes; ++i) {
        if (i != 0) {
          oss << ",";
        }
        oss << static_cast<unsigned int>(opt.slave_position[i]);
      }
      oss << "],";
      oss << "\"rt_cycle_counter\":" << cycles << ",";
      oss << "\"rt_hz\":" << hz << ",";
      oss << "\"rt_last_jitter_ns\":" << last_jitter << ",";
      oss << "\"rt_max_abs_jitter_ns\":" << max_abs_jitter << ",";
      oss << "\"rt_overrun_count\":" << overruns << ",";
      oss << "\"armed\":" << (armed.load(std::memory_order_relaxed) ? 1 : 0) << ",";
      oss << "\"axis_enable_mask\":" << en_mask << ",";
      oss << "\"native_home_active_axis_mask\":"
          << native_home_active_axis_mask.load(std::memory_order_relaxed) << ",";
      oss << "\"have_feedback\":" << (have_fb ? 1 : 0) << ",";
      oss << "\"wkc_actual\":" << (have_fb ? wkc_actual : 0) << ",";
      oss << "\"wkc_expected\":" << wkc_expected << ",";
      oss << "\"master_state\":" << master_state << ",";
      oss << "\"responding_slaves\":" << responding_slaves << ",";
      oss << "\"online_slaves\":" << online_slaves << ",";
      oss << "\"operational_slaves\":" << operational_slaves << ",";
      oss << "\"link_up\":" << static_cast<unsigned int>(link_up) << ",";
      oss << "\"master_al_states\":" << static_cast<unsigned int>(master_al_states) << ",";
      oss << "\"domain_wc_state\":" << static_cast<unsigned int>(domain_wc_state) << ",";
      oss << "\"startup_ready\":" << static_cast<unsigned int>(startup_ready) << ",";
      oss << "\"startup_elapsed_ms\":" << startup_elapsed_ms << ",";
      oss << "\"startup_reset_count\":" << startup_reset_count << ",";
      oss << "\"axes\":[";
      auto append_absolute_feedback_json = [&](const AbsoluteFeedbackAxis& axis_feedback) {
        oss << "\"absolute_feedback\":{";
        bool first = true;
        for (uint32_t field_i = 0; field_i < absolute_feedback_cfg.field_count; ++field_i) {
          const auto& field_cfg = absolute_feedback_cfg.fields[field_i];
          if (!field_cfg.valid || field_cfg.key.empty()) {
            continue;
          }
          if (!first) {
            oss << ",";
          }
          first = false;
          const auto& field = axis_feedback.fields[field_i];
          oss << "\"" << field_cfg.key << "\":{";
          oss << "\"valid\":" << static_cast<unsigned int>(field.valid) << ",";
          oss << "\"value\":" << field.value;
          oss << "}";
        }
        oss << "},";
      };
      auto append_startup_drive_config_json = [&](const StartupSdoConfig& descriptor,
                                                  const StartupSdoFeedback& feedback,
                                                  uint32_t axis_index) {
        if (!descriptor.valid) {
          oss << "null";
          return;
        }
        oss << "{";
        oss << "\"setting_key\":\"" << descriptor.key << "\",";
        oss << "\"type\":\"u16\",";
        oss << "\"index\":" << static_cast<unsigned int>(descriptor.index) << ",";
        oss << "\"subindex\":" << static_cast<unsigned int>(descriptor.subindex) << ",";
        oss << "\"configured\":" << static_cast<unsigned int>(feedback.configured[axis_index]) << ",";
        oss << "\"commanded\":" << feedback.commanded[axis_index] << ",";
        oss << "\"readback_valid\":" << static_cast<unsigned int>(feedback.readback_valid[axis_index]) << ",";
        oss << "\"readback\":" << feedback.readback[axis_index] << ",";
        oss << "\"verified\":" << static_cast<unsigned int>(feedback.verified[axis_index]);
        oss << "}";
      };
      for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
        if (i != 0) {
          oss << ",";
        }
        oss << "{";
        oss << "\"pos_counts\":" << pos_counts[i] << ",";
        oss << "\"statusword\":" << statusword[i] << ",";
        oss << "\"error_code\":" << error_code[i] << ",";
        oss << "\"manufacturer_error_code\":" << manufacturer_error_code[i] << ",";
        oss << "\"startup_drive_config\":";
        if (!startup_sdos.empty()) {
          append_startup_drive_config_json(startup_sdos.front(), startup_drive_config_feedback[0], i);
        } else {
          oss << "null";
        }
        oss << ",";
        oss << "\"startup_drive_configs\":[";
        for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
          if (descriptor_i != 0) {
            oss << ",";
          }
          append_startup_drive_config_json(
              startup_sdos[descriptor_i], startup_drive_config_feedback[descriptor_i], i);
        }
        oss << "],";
        oss << "\"native_home_state\":" << static_cast<unsigned int>(native_home_state[i]) << ",";
        oss << "\"native_home_position_offset\":" << native_home_position_offset[i] << ",";
        oss << "\"native_home_last_abort_code\":" << native_home_last_abort_code[i] << ",";
        append_absolute_feedback_json(absolute_feedback[i]);
        oss << "\"slave_online\":" << static_cast<unsigned int>(slave_online[i]) << ",";
        oss << "\"slave_operational\":" << static_cast<unsigned int>(slave_operational[i]) << ",";
        oss << "\"slave_al_state\":" << static_cast<unsigned int>(slave_al_state[i]);
        oss << "}";
      }
      oss << "]";
      oss << "}\n";

      try {
        const std::filesystem::path tmp = metrics_path.string() + ".tmp";
        std::ofstream f(tmp, std::ios::out | std::ios::trunc);
        f << oss.str();
        f.close();
        std::filesystem::rename(tmp, metrics_path);
        // Keep readable by the controller user/group.
        if (ipc_gid != static_cast<gid_t>(-1)) {
          (void)::chown(metrics_path.c_str(), static_cast<uid_t>(-1), ipc_gid);
        }
        (void)::chmod(metrics_path.c_str(), 0644);
      } catch (const std::exception& e) {
        // Don't spam logs if the filesystem is temporarily unhappy.
        if (now_ns - last_warn_ns > 5000000000ULL) { // 5s
          last_warn_ns = now_ns;
          logf("WARNING: failed to write metrics file %s: %s",
               metrics_path.c_str(), e.what());
        }
      }
    }
  });

  // Connection-scoped state (reset on disconnect).
  int controlling_client_fd = -1;
  ShmRegion cmd_shm;
  ShmRegion status_shm;
  int cmd_eventfd = -1;
  int status_eventfd = -1;
  std::atomic<bool> helper_running{false};
  std::thread helper_thread;

  auto reset_connection = [&]() {
    helper_running.store(false, std::memory_order_relaxed);
    if (helper_thread.joinable()) {
      helper_thread.join();
    }
    if (controlling_client_fd >= 0) {
      close(controlling_client_fd);
      controlling_client_fd = -1;
    }
    if (cmd_eventfd >= 0) {
      close(cmd_eventfd);
      cmd_eventfd = -1;
    }
    if (status_eventfd >= 0) {
      close(status_eventfd);
      status_eventfd = -1;
    }
    cmd_shm.reset();
    status_shm.reset();
  };

  // Main accept loop.
  while (!g_stop.load(std::memory_order_relaxed)) {
    // If a controller was connected previously, detect disconnect and free the slot.
    // Without this, a client exiting cleanly can still leave us thinking a controller
    // is connected (because we don't continuously read from the socket).
    if (controlling_client_fd >= 0) {
      pollfd cfd{};
      cfd.fd = controlling_client_fd;
      cfd.events = POLLIN;
#ifdef POLLRDHUP
      cfd.events |= POLLRDHUP;
#endif
      const int cpr = poll(&cfd, 1, 0);
      if (cpr > 0) {
        short hang_mask = static_cast<short>(POLLHUP | POLLERR);
#ifdef POLLRDHUP
        hang_mask = static_cast<short>(hang_mask | POLLRDHUP);
#endif
        if ((cfd.revents & hang_mask) != 0) {
          logf("Controller disconnected; freeing IPC slot.");
          reset_connection();
        }
      }
    }

    pollfd pfd{};
    pfd.fd = server_fd;
    pfd.events = POLLIN;
    int pr = poll(&pfd, 1, 250);
    if (pr < 0) {
      if (errno == EINTR) {
        continue;
      }
      logf("ERROR: poll(server) failed: %s", std::strerror(errno));
      break;
    }
    if (pr == 0) {
      continue;
    }

    if (!(pfd.revents & POLLIN)) {
      continue;
    }

    int client_fd = accept(server_fd, nullptr, nullptr);
    if (client_fd < 0) {
      if (errno == EINTR) {
        continue;
      }
      logf("ERROR: accept() failed: %s", std::strerror(errno));
      continue;
    }
    set_cloexec(client_fd);

    if (controlling_client_fd >= 0) {
      logf("Rejecting additional client (single-controller policy).");
      close(client_fd);
      continue;
    }

    // Read HELLO.
    gradient::ipc::v1::HelloV1 hello{};
    ssize_t n = recv(client_fd, &hello, sizeof(hello), 0);
    if (n != static_cast<ssize_t>(sizeof(hello))) {
      logf("ERROR: HELLO read failed (got %zd bytes)", n);
      close(client_fd);
      continue;
    }
    if (hello.magic != gradient::ipc::v1::kMagicGipc ||
        hello.ver_major != gradient::ipc::v1::kVerMajor ||
        hello.ver_minor != gradient::ipc::v1::kVerMinor ||
        hello.bytes != sizeof(hello) ||
        hello.role != gradient::ipc::v1::kRoleController) {
      logf("ERROR: HELLO validation failed (magic/ver/bytes/role mismatch)");
      close(client_fd);
      continue;
    }

    // Connection accepted.
    controlling_client_fd = client_fd;
    logf("Controller connected (pid=%llu)", static_cast<unsigned long long>(hello.pid));

    // Create shared memory regions (memfd) and eventfds.
    const uint64_t topology_hash = 0; // TODO(ethercat): compute from live bus
    const uint64_t build_id_hash = 0; // TODO: embed git hash

    const size_t cmd_ring_hdr_aligned = align_up(sizeof(gradient::ipc::v1::RingHeaderV1), 8);
    const size_t cmd_ring_bytes =
        cmd_ring_hdr_aligned +
        static_cast<size_t>(gradient::ipc::v1::GRADIENT_CMD_RING_CAPACITY) *
            gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
    const size_t cmd_shm_bytes =
        align_up(sizeof(gradient::ipc::v1::ShmHeaderV1) + cmd_ring_bytes, 4096);

    const size_t status_ring_hdr_aligned = align_up(sizeof(gradient::ipc::v1::RingHeaderV1), 8);
    const size_t status_ring_bytes =
        status_ring_hdr_aligned +
        static_cast<size_t>(gradient::ipc::v1::GRADIENT_STATUS_RING_CAPACITY) *
            gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
    const size_t status_shm_bytes =
        align_up(sizeof(gradient::ipc::v1::ShmHeaderV1) + status_ring_bytes, 4096);

    cmd_shm = create_memfd_region("gradient_cmd_shm", cmd_shm_bytes);
    status_shm = create_memfd_region("gradient_status_shm", status_shm_bytes);
    if (cmd_shm.fd < 0 || status_shm.fd < 0) {
      reset_connection();
      continue;
    }

    cmd_eventfd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    status_eventfd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (cmd_eventfd < 0 || status_eventfd < 0) {
      logf("ERROR: eventfd() failed: %s", std::strerror(errno));
      reset_connection();
      continue;
    }

    // Initialize cmd_shm.
    {
      std::memset(cmd_shm.base, 0, cmd_shm.bytes);
      auto* hdr = reinterpret_cast<gradient::ipc::v1::ShmHeaderV1*>(cmd_shm.base);
      hdr->magic = gradient::ipc::v1::kMagicGshm;
      hdr->ver_major = gradient::ipc::v1::kVerMajor;
      hdr->ver_minor = gradient::ipc::v1::kVerMinor;
      hdr->bytes = sizeof(*hdr);
      hdr->kind = gradient::ipc::v1::kShmKindCmd;
      hdr->num_axes = opt.num_axes;
      hdr->cycle_ns = opt.cycle_ns;
      hdr->topology_hash = topology_hash;
      hdr->ring_offset = sizeof(*hdr);
      hdr->ring_capacity = gradient::ipc::v1::GRADIENT_CMD_RING_CAPACITY;
      hdr->ring_msg_bytes = gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
      hdr->setpoint_offset = 0;

      auto ring = make_ring_view(cmd_shm.base, hdr);
      ring.header->magic = gradient::ipc::v1::kMagicRing;
      ring.header->capacity = hdr->ring_capacity;
      ring.header->msg_bytes = hdr->ring_msg_bytes;
      ring.header->write_idx = 0;
      ring.header->read_idx = 0;
      ring.header->dropped = 0;

    }

    // Initialize status_shm.
    {
      std::memset(status_shm.base, 0, status_shm.bytes);
      auto* hdr = reinterpret_cast<gradient::ipc::v1::ShmHeaderV1*>(status_shm.base);
      hdr->magic = gradient::ipc::v1::kMagicGshm;
      hdr->ver_major = gradient::ipc::v1::kVerMajor;
      hdr->ver_minor = gradient::ipc::v1::kVerMinor;
      hdr->bytes = sizeof(*hdr);
      hdr->kind = gradient::ipc::v1::kShmKindStatus;
      hdr->num_axes = opt.num_axes;
      hdr->cycle_ns = opt.cycle_ns;
      hdr->topology_hash = topology_hash;
      hdr->ring_offset = sizeof(*hdr);
      hdr->ring_capacity = gradient::ipc::v1::GRADIENT_STATUS_RING_CAPACITY;
      hdr->ring_msg_bytes = gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
      hdr->setpoint_offset = 0;

      auto ring = make_ring_view(status_shm.base, hdr);
      ring.header->magic = gradient::ipc::v1::kMagicRing;
      ring.header->capacity = hdr->ring_capacity;
      ring.header->msg_bytes = hdr->ring_msg_bytes;
      ring.header->write_idx = 0;
      ring.header->read_idx = 0;
      ring.header->dropped = 0;
    }

    // Send WELCOME + SCM_RIGHTS fds.
    gradient::ipc::v1::WelcomeV1 welcome{};
    welcome.magic = gradient::ipc::v1::kMagicGipc;
    welcome.ver_major = gradient::ipc::v1::kVerMajor;
    welcome.ver_minor = gradient::ipc::v1::kVerMinor;
    welcome.bytes = sizeof(welcome);
    welcome.num_axes = opt.num_axes;
    welcome.cycle_ns = opt.cycle_ns;
    welcome.topology_hash = topology_hash;
    welcome.cmd_ring_capacity = gradient::ipc::v1::GRADIENT_CMD_RING_CAPACITY;
    welcome.cmd_msg_bytes = gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
    welcome.status_ring_capacity = gradient::ipc::v1::GRADIENT_STATUS_RING_CAPACITY;
    welcome.status_msg_bytes = gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
    welcome.build_id_hash = build_id_hash;

    int fds[4] = {cmd_shm.fd, status_shm.fd, cmd_eventfd, status_eventfd};
    char cmsg_buf[CMSG_SPACE(sizeof(fds))];
    std::memset(cmsg_buf, 0, sizeof(cmsg_buf));

    iovec iov{};
    iov.iov_base = &welcome;
    iov.iov_len = sizeof(welcome);

    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = cmsg_buf;
    msg.msg_controllen = sizeof(cmsg_buf);

    cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(fds));
    std::memcpy(CMSG_DATA(cmsg), fds, sizeof(fds));

    if (sendmsg(controlling_client_fd, &msg, 0) < 0) {
      logf("ERROR: sendmsg(WELCOME) failed: %s", std::strerror(errno));
      reset_connection();
      continue;
    }

    // Start helper thread for status publishing + command ring draining.
    helper_running.store(true, std::memory_order_relaxed);
    helper_thread = std::thread([&]() {
      pthread_setname_np(pthread_self(), "ipc-helper");

      // Best-effort pin helper to CPU0-CPU1 (housekeeping cores).
      const unsigned int cpu_count = std::thread::hardware_concurrency();
      if (cpu_count >= 2) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(0, &cpuset);
        CPU_SET(1, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
          logf("WARNING: failed to pin ipc-helper thread to CPU0-CPU1: %s", std::strerror(errno));
        }
      }

      auto* status_hdr =
          reinterpret_cast<const gradient::ipc::v1::ShmHeaderV1*>(status_shm.base);
      RingView status_ring = make_ring_view(status_shm.base, status_hdr);
      uint64_t status_seq = 1;

      // Command ring + setpoint slot live in cmd_shm.
      auto* cmd_hdr =
          reinterpret_cast<const gradient::ipc::v1::ShmHeaderV1*>(cmd_shm.base);
      RingView cmd_ring = make_ring_view(cmd_shm.base, cmd_hdr);
      PendingTrajectoryUpload pending_upload{};
      uint64_t committed_traj_seq = 1;

      auto publish_motion_status = [&](uint32_t active_mode,
                                       uint32_t exec_state,
                                       uint64_t traj_id,
                                       uint32_t point_index,
                                       uint32_t queue_depth,
                                       uint32_t last_event_code,
                                       uint32_t stale_flag,
                                       uint32_t motion_done_flag,
                                       uint64_t active_cmd_seq,
                                       uint64_t update_ns) {
        motion_active_mode.store(active_mode, std::memory_order_relaxed);
        motion_exec_state.store(exec_state, std::memory_order_relaxed);
        motion_active_traj_id.store(traj_id, std::memory_order_relaxed);
        motion_current_point_index.store(point_index, std::memory_order_relaxed);
        motion_queue_depth.store(queue_depth, std::memory_order_relaxed);
        motion_last_event_code.store(last_event_code, std::memory_order_relaxed);
        motion_stale_command_flag.store(stale_flag, std::memory_order_relaxed);
        motion_done.store(motion_done_flag, std::memory_order_relaxed);
        motion_active_command_seq.store(active_cmd_seq, std::memory_order_relaxed);
        motion_last_update_ns.store(update_ns, std::memory_order_relaxed);
      };

      auto clear_motion_intent = [&](uint64_t active_cmd_seq,
                                     uint32_t last_event_code,
                                     uint32_t exec_state) {
        const uint64_t clear_now_ns = now_monotonic_ns();
        if (pending_upload.active) {
          logf("FAULT_CLEAR_M1 clear_now_ns=%llu clear_cmd_seq=%llu last_event_code=%u "
               "exec_state=%u pending_traj_id=%llu pending_axis_mask=0x%x "
               "pending_point_count=%u expected_points=%u",
               static_cast<unsigned long long>(clear_now_ns),
               static_cast<unsigned long long>(active_cmd_seq),
               static_cast<unsigned int>(last_event_code),
               static_cast<unsigned int>(exec_state),
               static_cast<unsigned long long>(pending_upload.traj_id),
               static_cast<unsigned int>(pending_upload.axis_mask),
               static_cast<unsigned int>(pending_upload.point_count),
               static_cast<unsigned int>(pending_upload.expected_points));
        }
        pending_upload = PendingTrajectoryUpload{};
        trajectory_abort_request.store(
            std::numeric_limits<uint64_t>::max(),
            std::memory_order_release);
        committed_trajectory.traj_id = 0;
        committed_trajectory.cmd_seq = 0;
        committed_trajectory.axis_mask = 0;
        committed_trajectory.point_count = 0;
        committed_trajectory.seq.store(0, std::memory_order_release);

        latest_jog_command.axis_mask = 0;
        latest_jog_command.flags = gradient::ipc::v1::JOG_FLAG_STOP;
        latest_jog_command.timeout_ns = 0;
        latest_jog_command.cmd_seq = active_cmd_seq;
        for (uint32_t axis_i = 0; axis_i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++axis_i) {
          latest_jog_command.velocity_counts_per_s[axis_i] = 0.0;
        }
        latest_jog_command.seq.store(active_cmd_seq, std::memory_order_release);

        publish_motion_status(
            gradient::ipc::v1::MOTION_MODE_IDLE,
            exec_state,
            0,
            std::numeric_limits<uint32_t>::max(),
            0,
            last_event_code,
            0,
            1,
            active_cmd_seq,
            clear_now_ns);
      };

      auto read_stable_axis_feedback_counts = [&](uint32_t axis, int32_t* pos_counts_out) {
        if (!pos_counts_out || axis >= opt.num_axes) {
          return false;
        }
        for (int attempt = 0; attempt < 20; ++attempt) {
          const uint64_t seq_before = latest_feedback.seq.load(std::memory_order_acquire);
          const int32_t pos_counts = latest_feedback.pos_counts[axis];
          const uint64_t seq_after = latest_feedback.seq.load(std::memory_order_acquire);
          if (seq_before != 0 && seq_before == seq_after) {
            *pos_counts_out = pos_counts;
            return true;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
      };

      auto native_home_axis = [&](uint32_t axis) {
        latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_REQUESTED;
        latest_feedback.native_home_last_abort_code[axis] = 0;

        int32_t pos_counts = latest_feedback.pos_counts[axis];
        (void)read_stable_axis_feedback_counts(axis, &pos_counts);

#if GRADIENT_HAVE_ECRT
        if (!native_home_cfg.valid || !native_home_cfg.truth_source.valid ||
            native_home_cfg.transaction.empty()) {
          latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
          logf("WARNING: native_home requested axis=%u but no native-home descriptor is configured",
               axis);
          return;
        }
        if (!shared_master) {
          latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
          return;
        }
        constexpr auto kServiceStepSleep = std::chrono::milliseconds(50);
        constexpr uint64_t kWaitStatuswordTimeoutNs = 10000000000ULL;
        constexpr uint64_t kWaitSdoTimeoutNs = 5000000000ULL;
        const uint32_t axis_bit = (1u << axis);
        service_controlword_override[axis].store(0, std::memory_order_relaxed);
        service_mode_override[axis].store(
            static_cast<int32_t>(native_home_cfg.commissioning_mode),
            std::memory_order_relaxed);
        service_mode_axis_mask.fetch_or(axis_bit, std::memory_order_relaxed);
        std::this_thread::sleep_for(kServiceStepSleep);

        auto release_service_override = [&](bool restore_steady_mode) {
          service_controlword_override[axis].store(0, std::memory_order_relaxed);
          if (restore_steady_mode) {
            desired_mode_of_operation[axis].store(
                static_cast<int32_t>(native_home_cfg.steady_state_mode),
                std::memory_order_relaxed);
            service_mode_override[axis].store(
                static_cast<int32_t>(native_home_cfg.steady_state_mode),
                std::memory_order_relaxed);
          } else {
            service_mode_override[axis].store(kNoModeOverride, std::memory_order_relaxed);
          }
          std::this_thread::sleep_for(kServiceStepSleep);
          service_mode_axis_mask.fetch_and(~axis_bit, std::memory_order_relaxed);
          service_mode_override[axis].store(kNoModeOverride, std::memory_order_relaxed);
          service_controlword_override[axis].store(0, std::memory_order_relaxed);
        };

        bool succeeded = true;
        bool refreshed_truth = false;
        bool service_override_released = false;
        for (const NativeHomeOp& step : native_home_cfg.transaction) {
          if (!succeeded) {
            break;
          }
          switch (step.kind) {
            case NativeHomeOpKind::kSetMode:
            case NativeHomeOpKind::kRestoreMode:
              service_mode_override[axis].store(step.value_i32, std::memory_order_relaxed);
              std::this_thread::sleep_for(kServiceStepSleep);
              break;
            case NativeHomeOpKind::kWriteSdo: {
              uint32_t abort_code = 0;
              if (!step.object.valid ||
                  !write_sdo_scalar_axis(axis,
                                         step.object.index,
                                         step.object.subindex,
                                         step.object.type,
                                         step.value_i32,
                                         &abort_code)) {
                latest_feedback.native_home_last_abort_code[axis] = abort_code;
                latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
                logf("WARNING: native_home write_sdo failed axis=%u slave_pos=%u index=0x%04x sub=0x%02x abort=0x%08x value=%d",
                     axis,
                     static_cast<unsigned int>(opt.slave_position[axis]),
                     static_cast<unsigned int>(step.object.index),
                     static_cast<unsigned int>(step.object.subindex),
                     static_cast<unsigned int>(abort_code),
                     step.value_i32);
                succeeded = false;
              }
              break;
            }
            case NativeHomeOpKind::kWriteSdoWrapFraction: {
              const uint32_t wrap_period_counts = wrapped_axis_period_counts(opt.axis[axis]);
              uint32_t abort_code = 0;
              if (!step.object.valid || wrap_period_counts == 0 || step.fraction_denominator == 0) {
                latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
                latest_feedback.native_home_last_abort_code[axis] = 0;
                logf("WARNING: native_home write_sdo_wrap_fraction invalid axis=%u slave_pos=%u index=0x%04x sub=0x%02x wrap_period=%u num=%u den=%u",
                     axis,
                     static_cast<unsigned int>(opt.slave_position[axis]),
                     static_cast<unsigned int>(step.object.index),
                     static_cast<unsigned int>(step.object.subindex),
                     static_cast<unsigned int>(wrap_period_counts),
                     static_cast<unsigned int>(step.fraction_numerator),
                     static_cast<unsigned int>(step.fraction_denominator));
                succeeded = false;
                break;
              }
              const double fraction =
                  static_cast<double>(step.fraction_numerator) /
                  static_cast<double>(step.fraction_denominator);
              const int64_t raw_value =
                  static_cast<int64_t>(std::llround(static_cast<double>(wrap_period_counts) * fraction));
              const int32_t wrapped_value = wrap_counts_into_period(raw_value, wrap_period_counts);
              if (!write_sdo_scalar_axis(axis,
                                         step.object.index,
                                         step.object.subindex,
                                         step.object.type,
                                         wrapped_value,
                                         &abort_code)) {
                latest_feedback.native_home_last_abort_code[axis] = abort_code;
                latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
                logf("WARNING: native_home write_sdo_wrap_fraction failed axis=%u slave_pos=%u index=0x%04x sub=0x%02x abort=0x%08x value=%d wrap_period=%u num=%u den=%u",
                     axis,
                     static_cast<unsigned int>(opt.slave_position[axis]),
                     static_cast<unsigned int>(step.object.index),
                     static_cast<unsigned int>(step.object.subindex),
                     static_cast<unsigned int>(abort_code),
                     wrapped_value,
                     static_cast<unsigned int>(wrap_period_counts),
                     static_cast<unsigned int>(step.fraction_numerator),
                     static_cast<unsigned int>(step.fraction_denominator));
                succeeded = false;
              }
              break;
            }
            case NativeHomeOpKind::kControlwordSequence:
              if (step.controlword_count == 0) {
                latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
                succeeded = false;
                break;
              }
              for (uint32_t step_i = 0; step_i < step.controlword_count; ++step_i) {
                service_controlword_override[axis].store(
                    static_cast<uint32_t>(step.controlword_values[step_i]),
                    std::memory_order_relaxed);
                std::this_thread::sleep_for(kServiceStepSleep);
              }
              break;
            case NativeHomeOpKind::kWaitStatusword: {
              bool matched = false;
              const uint64_t deadline_ns = now_monotonic_ns() + kWaitStatuswordTimeoutNs;
              while (now_monotonic_ns() < deadline_ns) {
                const uint16_t statusword = latest_feedback.statusword[axis];
                if ((statusword & step.wait_all_set_mask) == step.wait_all_set_mask &&
                    (statusword & step.wait_all_clear_mask) == 0u) {
                  matched = true;
                  break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
              }
              if (!matched) {
                latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
                logf("WARNING: native_home wait_statusword timed out axis=%u slave_pos=%u all_set=0x%04x all_clear=0x%04x last_statusword=0x%04x",
                     axis,
                     static_cast<unsigned int>(opt.slave_position[axis]),
                     static_cast<unsigned int>(step.wait_all_set_mask),
                     static_cast<unsigned int>(step.wait_all_clear_mask),
                     static_cast<unsigned int>(latest_feedback.statusword[axis]));
                succeeded = false;
              }
              break;
            }
            case NativeHomeOpKind::kWaitSdo: {
              bool matched = false;
              int32_t current_value = 0;
              uint32_t abort_code = 0;
              const uint64_t deadline_ns = now_monotonic_ns() + kWaitSdoTimeoutNs;
              while (now_monotonic_ns() < deadline_ns) {
                abort_code = 0;
                if (step.object.valid &&
                    read_sdo_scalar_axis(axis,
                                         step.object.index,
                                         step.object.subindex,
                                         step.object.type,
                                         &current_value,
                                         &abort_code) &&
                    current_value == step.value_i32) {
                  matched = true;
                  break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
              }
              if (!matched) {
                latest_feedback.native_home_last_abort_code[axis] = abort_code;
                latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
                logf("WARNING: native_home wait_sdo timed out axis=%u slave_pos=%u index=0x%04x sub=0x%02x expected=%d last=%d abort=0x%08x",
                     axis,
                     static_cast<unsigned int>(opt.slave_position[axis]),
                     static_cast<unsigned int>(step.object.index),
                     static_cast<unsigned int>(step.object.subindex),
                     step.value_i32,
                     current_value,
                     static_cast<unsigned int>(abort_code));
                succeeded = false;
              }
              break;
            }
            case NativeHomeOpKind::kReleaseServiceOverride:
              if (!service_override_released) {
                release_service_override(true);
                service_override_released = true;
              }
              break;
            case NativeHomeOpKind::kRefreshTruth: {
              int32_t refreshed_value = 0;
              if (!read_native_home_truth_axis(axis, &refreshed_value)) {
                latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
                succeeded = false;
                break;
              }
              latest_feedback.native_home_position_offset[axis] = refreshed_value;
              refreshed_truth = true;
              break;
            }
            case NativeHomeOpKind::kNone:
            default:
              latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
              succeeded = false;
              break;
          }
        }

        if (succeeded && !refreshed_truth) {
          int32_t refreshed_value = 0;
          if (!read_native_home_truth_axis(axis, &refreshed_value)) {
            latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
            succeeded = false;
          } else {
            latest_feedback.native_home_position_offset[axis] = refreshed_value;
          }
        }

        if (!service_override_released) {
          release_service_override(succeeded);
        }
        if (!succeeded) {
          if (latest_feedback.native_home_state[axis] !=
              gradient::ipc::v1::NATIVE_HOME_STATE_FAILED) {
            latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
          }
          return;
        }

        latest_feedback.native_home_last_abort_code[axis] = 0;
        latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_SUCCEEDED;
        logf("EtherCAT native_home axis=%u slave_pos=%u feedback_counts=%d truth_value=%d commissioning_mode=%u steady_state_mode=%u steps=%zu",
             axis,
             static_cast<unsigned int>(opt.slave_position[axis]),
             pos_counts,
             latest_feedback.native_home_position_offset[axis],
             static_cast<unsigned int>(native_home_cfg.commissioning_mode),
             static_cast<unsigned int>(native_home_cfg.steady_state_mode),
             native_home_cfg.transaction.size());
#else
        (void)pos_counts;
        latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
#endif
      };

      auto service_sdo_write_axes = [&](uint32_t axis_mask,
                                        uint16_t index,
                                        uint8_t subindex,
                                        uint8_t value_type,
                                        uint32_t value_u32) {
#if GRADIENT_HAVE_ECRT
        if (value_type != gradient::ipc::v1::SERVICE_SDO_VALUE_U16) {
          logf("WARNING: service_sdo_write skipped index=0x%04x sub=0x%02x; unsupported value_type=%u",
               static_cast<unsigned int>(index),
               static_cast<unsigned int>(subindex),
               static_cast<unsigned int>(value_type));
          return;
        }

        uint8_t raw[sizeof(uint16_t)] = {};
        EC_WRITE_U16(raw, static_cast<uint16_t>(value_u32 & 0xFFFFu));
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
          if ((axis_mask & (1u << axis)) == 0u) {
            continue;
          }
          uint32_t abort_code = 0;
          int rc = -1;
          {
            std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
            if (g_stop.load(std::memory_order_relaxed) || !shared_master) {
              logf("WARNING: service_sdo_write skipped index=0x%04x sub=0x%02x; master unavailable",
                   static_cast<unsigned int>(index),
                   static_cast<unsigned int>(subindex));
              return;
            }
            rc = ecrt_master_sdo_download(
                shared_master,
                opt.slave_position[axis],
                index,
                subindex,
                raw,
                sizeof(raw),
                &abort_code);
          }
          if (rc != 0) {
            logf("WARNING: service_sdo_write failed axis=%u slave_pos=%u index=0x%04x sub=0x%02x rc=%d abort=0x%08x value=%u type=%u",
                 axis,
                 static_cast<unsigned int>(opt.slave_position[axis]),
                 static_cast<unsigned int>(index),
                 static_cast<unsigned int>(subindex),
                 rc,
                 static_cast<unsigned int>(abort_code),
                 static_cast<unsigned int>(value_u32),
                 static_cast<unsigned int>(value_type));
            continue;
          }
          logf("EtherCAT service_sdo_write axis=%u slave_pos=%u index=0x%04x sub=0x%02x value=%u type=%u",
               axis,
               static_cast<unsigned int>(opt.slave_position[axis]),
               static_cast<unsigned int>(index),
               static_cast<unsigned int>(subindex),
               static_cast<unsigned int>(value_u32),
               static_cast<unsigned int>(value_type));
        }
#else
        (void)axis_mask;
        (void)index;
        (void)subindex;
        (void)value_type;
        (void)value_u32;
#endif
      };

      // Emit STATUS_HELLO once on connect.
      {
        gradient::ipc::v1::StatusHelloV1 sh{};
        sh.build_id_hash = build_id_hash;
        sh.topology_hash = topology_hash;
        sh.cycle_ns = opt.cycle_ns;
        sh.num_axes = opt.num_axes;
        sh.drive_profile_id = opt.drive_profile_id;
        sh.wkc_expected = 2 * opt.num_axes;
        ring_write(status_ring,
                   gradient::ipc::v1::MSG_STATUS_HELLO,
                   &sh,
                   sizeof(sh),
                   &status_seq,
                   now_monotonic_ns());
        eventfd_write_one(status_eventfd);
      }

      // Emit STATUS_AXIS_CONFIG once on connect (bring-up tooling).
      {
        gradient::ipc::v1::StatusAxisConfigV1 cfg{};
        cfg.num_axes = opt.num_axes;
        for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
          cfg.counts_per_rev[i] = opt.axis[i].counts_per_rev;
          cfg.gear_ratio[i] = opt.axis[i].gear_ratio;
          cfg.sign[i] = opt.axis[i].sign;
          cfg.axis_type[i] = opt.axis[i].axis_type;
          cfg.counts_per_unit[i] = opt.axis[i].counts_per_unit;
        }
        ring_write(status_ring,
                   gradient::ipc::v1::MSG_STATUS_AXIS_CONFIG,
                   &cfg,
                   sizeof(cfg),
                   &status_seq,
                   now_monotonic_ns());
        eventfd_write_one(status_eventfd);
      }

      for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
        int32_t current_offset = 0;
        if (read_native_home_truth_axis(axis, &current_offset)) {
          latest_feedback.native_home_position_offset[axis] = current_offset;
        }
      }

      uint64_t next_snapshot_ns = now_monotonic_ns();
      uint64_t next_jog_debug_ns = now_monotonic_ns();

      while (helper_running.load(std::memory_order_relaxed) &&
             !g_stop.load(std::memory_order_relaxed)) {
        pollfd pfds[1]{};
        pfds[0].fd = cmd_eventfd;
        pfds[0].events = POLLIN;

        // Wake periodically for status publishing even if no commands arrive.
        int timeout_ms = 20;
        int pr2 = poll(pfds, 1, timeout_ms);
        if (pr2 > 0 && (pfds[0].revents & POLLIN)) {
          eventfd_drain(cmd_eventfd);

          // Drain command ring entries and update local state.
          if (cmd_ring.header && cmd_ring.header->magic == gradient::ipc::v1::kMagicRing) {
            uint32_t r = cmd_ring.header->read_idx;
            const uint32_t w = cmd_ring.header->write_idx;
            while (r < w) {
              const uint32_t slot = r % cmd_ring.capacity;
              uint8_t* slot_ptr =
                  cmd_ring.entries + static_cast<size_t>(slot) * cmd_ring.msg_bytes;

              auto* mh = reinterpret_cast<const gradient::ipc::v1::MsgHeader*>(slot_ptr);
              const uint8_t* payload = slot_ptr + sizeof(*mh);

              switch (mh->type) {
                case gradient::ipc::v1::MSG_CMD_ARM: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdArmV1)) {
                    auto* cmd = reinterpret_cast<const gradient::ipc::v1::CmdArmV1*>(payload);
                    const bool arm = (cmd->arm != 0);
                    armed.store(arm, std::memory_order_relaxed);
                    if (!arm) {
                      clear_motion_intent(
                          mh->seq,
                          gradient::ipc::v1::EVT_DISARMED,
                          gradient::ipc::v1::EXEC_STATE_ABORTED);
                    }
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_AXIS_ENABLE: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdAxisMaskV1)) {
                    auto* cmd = reinterpret_cast<const gradient::ipc::v1::CmdAxisMaskV1*>(payload);
                    axis_enable_mask.store(cmd->axis_mask, std::memory_order_relaxed);
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_AXIS_DISABLE: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdAxisMaskV1)) {
                    auto* cmd = reinterpret_cast<const gradient::ipc::v1::CmdAxisMaskV1*>(payload);
                    const uint32_t cur = axis_enable_mask.load(std::memory_order_relaxed);
                    const uint32_t next = cur & ~cmd->axis_mask;
                    axis_enable_mask.store(next, std::memory_order_relaxed);
                    if (next == 0u) {
                      clear_motion_intent(
                          mh->seq,
                          gradient::ipc::v1::EVT_DISARMED,
                          gradient::ipc::v1::EXEC_STATE_ABORTED);
                    }
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_SET_MODE: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdSetModeV1)) {
                    auto* cmd = reinterpret_cast<const gradient::ipc::v1::CmdSetModeV1*>(payload);
                    uint32_t mask = cmd->axis_mask;
                    const uint32_t valid =
                        (opt.num_axes >= 32) ? 0xFFFFFFFFu : ((1u << opt.num_axes) - 1u);
                    if (mask == 0u) {
                      mask = valid;
                    }
                    mask &= valid;
                    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
                      if ((mask & (1u << axis)) == 0u) {
                        continue;
                      }
                      desired_mode_of_operation[axis].store(
                          static_cast<int32_t>(cmd->mode),
                          std::memory_order_relaxed);
                    }
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_FAULT_RESET: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdFaultResetV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdFaultResetV1*>(payload);
                    uint32_t mask = cmd->axis_mask;
                    const uint32_t valid =
                        (opt.num_axes >= 32) ? 0xFFFFFFFFu : ((1u << opt.num_axes) - 1u);
                    if (mask == 0) {
                      mask = valid;
                    }
                    mask &= valid;
                    if (mask != 0) {
                      clear_motion_intent(
                          mh->seq,
                          gradient::ipc::v1::EVT_DISARMED,
                          gradient::ipc::v1::EXEC_STATE_ABORTED);
                      fault_reset_request.fetch_or(mask, std::memory_order_relaxed);
                    }
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_NATIVE_HOME: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdNativeHomeV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdNativeHomeV1*>(payload);
                    uint32_t mask = cmd->axis_mask;
                    const uint32_t valid =
                        (opt.num_axes >= 32) ? 0xFFFFFFFFu : ((1u << opt.num_axes) - 1u);
                    mask &= valid;
                    if (mask != 0) {
                      const uint32_t cur = axis_enable_mask.load(std::memory_order_relaxed);
                      const uint32_t next = cur & ~mask;
                      axis_enable_mask.store(next, std::memory_order_relaxed);
                      if (next == 0u) {
                        armed.store(false, std::memory_order_relaxed);
                      }
                      native_home_active_axis_mask.fetch_or(mask, std::memory_order_relaxed);
                      clear_motion_intent(
                          mh->seq,
                          gradient::ipc::v1::EVT_DISARMED,
                          gradient::ipc::v1::EXEC_STATE_ABORTED);
                      // Vendor Q2: "Ensure the motor is stationary and inactive" before HM35.
                      // Wait for each targeted axis to leave OperationEnabled for several
                      // consecutive cycles before starting the HM35 transaction so the
                      // audible disarm + HM35 re-enable pair is replaced by a single
                      // disarm followed by the service-mode-owned HM35 sequence.
                      constexpr uint64_t kDisarmPreconditionTimeoutNs = 500000000ULL; // 500 ms
                      constexpr uint32_t kDisarmPreconditionStableCycles = 3;
                      constexpr auto kDisarmPreconditionPoll =
                          std::chrono::milliseconds(5);
                      uint32_t disarm_confirmed_mask = 0;
                      uint32_t disarm_failed_mask = 0;
                      for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
                        if ((mask & (1u << axis)) == 0u) {
                          continue;
                        }
                        const uint64_t deadline_ns =
                            now_monotonic_ns() + kDisarmPreconditionTimeoutNs;
                        uint32_t stable_cycles = 0;
                        bool confirmed = false;
                        while (now_monotonic_ns() < deadline_ns) {
                          const uint16_t sw = latest_feedback.statusword[axis];
                          const gradient::ds402::State st =
                              gradient::ds402::decode_statusword(sw);
                          if (st != gradient::ds402::State::OperationEnabled &&
                              st != gradient::ds402::State::QuickStopActive) {
                            if (++stable_cycles >= kDisarmPreconditionStableCycles) {
                              confirmed = true;
                              break;
                            }
                          } else {
                            stable_cycles = 0;
                          }
                          std::this_thread::sleep_for(kDisarmPreconditionPoll);
                        }
                        if (confirmed) {
                          disarm_confirmed_mask |= (1u << axis);
                        } else {
                          disarm_failed_mask |= (1u << axis);
                          latest_feedback.native_home_last_abort_code[axis] =
                              gradient::ipc::v1::
                                  NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT;
                          latest_feedback.native_home_state[axis] =
                              gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
                          logf("WARNING: native_home disarm precondition timeout axis=%u last_statusword=0x%04x abort=0x%08x",
                               axis,
                               static_cast<unsigned int>(
                                   latest_feedback.statusword[axis]),
                               static_cast<unsigned int>(
                                   gradient::ipc::v1::
                                       NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT));
                        }
                      }
                      if (disarm_confirmed_mask != 0u) {
                        logf("Native-home disarm precondition satisfied axis_mask=0x%x (timeouts=0x%x)",
                             static_cast<unsigned int>(disarm_confirmed_mask),
                             static_cast<unsigned int>(disarm_failed_mask));
                      }
                      bool all_succeeded = (disarm_failed_mask == 0u);
                      for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
                        if ((disarm_confirmed_mask & (1u << axis)) == 0u) {
                          continue;
                        }
                        native_home_axis(axis);
                        if (latest_feedback.native_home_state[axis] !=
                            gradient::ipc::v1::NATIVE_HOME_STATE_SUCCEEDED) {
                          all_succeeded = false;
                        }
                      }
                      if (all_succeeded) {
                        logf("Native-home success: homed axes remain disabled axis_mask 0x%x -> 0x%x armed=%u",
                             static_cast<unsigned int>(cur),
                             static_cast<unsigned int>(next),
                             static_cast<unsigned int>(
                                 armed.load(std::memory_order_relaxed) ? 1u : 0u));
                      } else {
                        logf("WARNING: native_home left requested axes disarmed after failure axis_mask=0x%x disarm_timeouts=0x%x",
                             static_cast<unsigned int>(mask),
                             static_cast<unsigned int>(disarm_failed_mask));
                      }
                      native_home_active_axis_mask.fetch_and(~mask, std::memory_order_relaxed);
                    }
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_SERVICE_SDO_WRITE: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdServiceSdoWriteV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdServiceSdoWriteV1*>(payload);
                    uint32_t mask = cmd->axis_mask;
                    const uint32_t valid =
                        (opt.num_axes >= 32) ? 0xFFFFFFFFu : ((1u << opt.num_axes) - 1u);
                    mask &= valid;
                    if (mask != 0u) {
                      const uint32_t cur = axis_enable_mask.load(std::memory_order_relaxed);
                      const uint32_t next = cur & ~mask;
                      axis_enable_mask.store(next, std::memory_order_relaxed);
                      if (next == 0u) {
                        armed.store(false, std::memory_order_relaxed);
                      }
                      clear_motion_intent(
                          mh->seq,
                          gradient::ipc::v1::EVT_DISARMED,
                          gradient::ipc::v1::EXEC_STATE_ABORTED);
                      service_sdo_write_axes(
                          mask,
                          cmd->index,
                          cmd->subindex,
                          cmd->value_type,
                          cmd->value_u32);
                    }
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_BEGIN: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdTrajectoryBeginV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdTrajectoryBeginV1*>(payload);
                    pending_upload = PendingTrajectoryUpload{};
                    pending_upload.active = true;
                    pending_upload.traj_id = cmd->traj_id;
                    pending_upload.axis_mask = cmd->axis_mask;
                    pending_upload.expected_points = cmd->expected_points;
                    const uint64_t update_ns = now_monotonic_ns();
                    publish_motion_status(
                        gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                        gradient::ipc::v1::EXEC_STATE_ACCEPTED,
                        cmd->traj_id,
                        std::numeric_limits<uint32_t>::max(),
                        0,
                        gradient::ipc::v1::EVT_TRAJECTORY_ACCEPTED,
                        0,
                        0,
                        mh->seq,
                        update_ns);
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_POINT: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::TrajectoryPointV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::TrajectoryPointV1*>(payload);
                    if (!pending_upload.active) {
                      const uint64_t update_ns = now_monotonic_ns();
                      logf("FAULT_UPLOAD_U1 update_ns=%llu cmd_seq=%llu cmd_traj_id=%llu "
                           "pending_traj_id=%llu pending_active=%u pending_axis_mask=0x%x "
                           "pending_point_count=%u expected_points=%u cmd_point_index=%u "
                           "expected_point_index=%u cmd_t_from_start_ns=%llu axis_mask=0x%x",
                           static_cast<unsigned long long>(update_ns),
                           static_cast<unsigned long long>(mh->seq),
                           static_cast<unsigned long long>(cmd->traj_id),
                           static_cast<unsigned long long>(pending_upload.traj_id),
                           pending_upload.active ? 1u : 0u,
                           static_cast<unsigned int>(pending_upload.axis_mask),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned int>(pending_upload.expected_points),
                           static_cast<unsigned int>(cmd->point_index),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned long long>(cmd->t_from_start_ns),
                           static_cast<unsigned int>(cmd->axis_mask));
                      publish_motion_status(
                          gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                          gradient::ipc::v1::EXEC_STATE_FAULTED,
                          cmd->traj_id,
                          std::numeric_limits<uint32_t>::max(),
                          pending_upload.point_count,
                          gradient::ipc::v1::EVT_TRAJECTORY_FAULTED,
                          0,
                          1,
                          mh->seq,
                          update_ns);
                      pending_upload = PendingTrajectoryUpload{};
                      break;
                    }
                    if (cmd->traj_id != pending_upload.traj_id) {
                      const uint64_t update_ns = now_monotonic_ns();
                      logf("FAULT_UPLOAD_U2 update_ns=%llu cmd_seq=%llu cmd_traj_id=%llu "
                           "pending_traj_id=%llu pending_active=%u pending_axis_mask=0x%x "
                           "pending_point_count=%u expected_points=%u cmd_point_index=%u "
                           "expected_point_index=%u cmd_t_from_start_ns=%llu axis_mask=0x%x",
                           static_cast<unsigned long long>(update_ns),
                           static_cast<unsigned long long>(mh->seq),
                           static_cast<unsigned long long>(cmd->traj_id),
                           static_cast<unsigned long long>(pending_upload.traj_id),
                           pending_upload.active ? 1u : 0u,
                           static_cast<unsigned int>(pending_upload.axis_mask),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned int>(pending_upload.expected_points),
                           static_cast<unsigned int>(cmd->point_index),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned long long>(cmd->t_from_start_ns),
                           static_cast<unsigned int>(cmd->axis_mask));
                      publish_motion_status(
                          gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                          gradient::ipc::v1::EXEC_STATE_FAULTED,
                          cmd->traj_id,
                          std::numeric_limits<uint32_t>::max(),
                          pending_upload.point_count,
                          gradient::ipc::v1::EVT_TRAJECTORY_FAULTED,
                          0,
                          1,
                          mh->seq,
                          update_ns);
                      pending_upload = PendingTrajectoryUpload{};
                      break;
                    }
                    if (pending_upload.point_count >= kMaxTrajectoryPoints) {
                      const uint64_t update_ns = now_monotonic_ns();
                      logf("FAULT_UPLOAD_U3 update_ns=%llu cmd_seq=%llu cmd_traj_id=%llu "
                           "pending_traj_id=%llu pending_active=%u pending_axis_mask=0x%x "
                           "pending_point_count=%u expected_points=%u max_points=%u "
                           "cmd_point_index=%u expected_point_index=%u cmd_t_from_start_ns=%llu "
                           "axis_mask=0x%x",
                           static_cast<unsigned long long>(update_ns),
                           static_cast<unsigned long long>(mh->seq),
                           static_cast<unsigned long long>(cmd->traj_id),
                           static_cast<unsigned long long>(pending_upload.traj_id),
                           pending_upload.active ? 1u : 0u,
                           static_cast<unsigned int>(pending_upload.axis_mask),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned int>(pending_upload.expected_points),
                           static_cast<unsigned int>(kMaxTrajectoryPoints),
                           static_cast<unsigned int>(cmd->point_index),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned long long>(cmd->t_from_start_ns),
                           static_cast<unsigned int>(cmd->axis_mask));
                      publish_motion_status(
                          gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                          gradient::ipc::v1::EXEC_STATE_FAULTED,
                          cmd->traj_id,
                          std::numeric_limits<uint32_t>::max(),
                          pending_upload.point_count,
                          gradient::ipc::v1::EVT_TRAJECTORY_FAULTED,
                          0,
                          1,
                          mh->seq,
                          update_ns);
                      pending_upload = PendingTrajectoryUpload{};
                      break;
                    }
                    if (cmd->point_index != pending_upload.point_count) {
                      const uint64_t update_ns = now_monotonic_ns();
                      logf("FAULT_UPLOAD_U4 update_ns=%llu cmd_seq=%llu cmd_traj_id=%llu "
                           "pending_traj_id=%llu pending_active=%u pending_axis_mask=0x%x "
                           "pending_point_count=%u expected_points=%u cmd_point_index=%u "
                           "expected_point_index=%u cmd_t_from_start_ns=%llu axis_mask=0x%x",
                           static_cast<unsigned long long>(update_ns),
                           static_cast<unsigned long long>(mh->seq),
                           static_cast<unsigned long long>(cmd->traj_id),
                           static_cast<unsigned long long>(pending_upload.traj_id),
                           pending_upload.active ? 1u : 0u,
                           static_cast<unsigned int>(pending_upload.axis_mask),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned int>(pending_upload.expected_points),
                           static_cast<unsigned int>(cmd->point_index),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned long long>(cmd->t_from_start_ns),
                           static_cast<unsigned int>(cmd->axis_mask));
                      publish_motion_status(
                          gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                          gradient::ipc::v1::EXEC_STATE_FAULTED,
                          cmd->traj_id,
                          std::numeric_limits<uint32_t>::max(),
                          pending_upload.point_count,
                          gradient::ipc::v1::EVT_TRAJECTORY_FAULTED,
                          0,
                          1,
                          mh->seq,
                          update_ns);
                      pending_upload = PendingTrajectoryUpload{};
                      break;
                    }

                    if (pending_upload.point_count > 0) {
                      const auto& prev = pending_upload.points[pending_upload.point_count - 1];
                      if (cmd->t_from_start_ns < prev.t_from_start_ns) {
                        const uint64_t update_ns = now_monotonic_ns();
                        logf("FAULT_UPLOAD_U5 update_ns=%llu cmd_seq=%llu cmd_traj_id=%llu "
                             "pending_traj_id=%llu pending_active=%u pending_axis_mask=0x%x "
                             "pending_point_count=%u expected_points=%u cmd_point_index=%u "
                             "expected_point_index=%u cmd_t_from_start_ns=%llu "
                             "prev_t_from_start_ns=%llu prev_point_index=%u axis_mask=0x%x",
                             static_cast<unsigned long long>(update_ns),
                             static_cast<unsigned long long>(mh->seq),
                             static_cast<unsigned long long>(cmd->traj_id),
                             static_cast<unsigned long long>(pending_upload.traj_id),
                             pending_upload.active ? 1u : 0u,
                             static_cast<unsigned int>(pending_upload.axis_mask),
                             static_cast<unsigned int>(pending_upload.point_count),
                             static_cast<unsigned int>(pending_upload.expected_points),
                             static_cast<unsigned int>(cmd->point_index),
                             static_cast<unsigned int>(pending_upload.point_count),
                             static_cast<unsigned long long>(cmd->t_from_start_ns),
                             static_cast<unsigned long long>(prev.t_from_start_ns),
                             static_cast<unsigned int>(pending_upload.point_count - 1),
                             static_cast<unsigned int>(cmd->axis_mask));
                        publish_motion_status(
                            gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                            gradient::ipc::v1::EXEC_STATE_FAULTED,
                            cmd->traj_id,
                            std::numeric_limits<uint32_t>::max(),
                            pending_upload.point_count,
                            gradient::ipc::v1::EVT_TRAJECTORY_FAULTED,
                            0,
                            1,
                            mh->seq,
                            update_ns);
                        pending_upload = PendingTrajectoryUpload{};
                        break;
                      }
                    }

                    TrajectoryPointRuntime runtime_point{};
                    runtime_point.t_from_start_ns = cmd->t_from_start_ns;
                    runtime_point.axis_mask = (cmd->axis_mask != 0u) ? cmd->axis_mask : pending_upload.axis_mask;
                    runtime_point.flags = cmd->flags;
                    for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                      if ((runtime_point.axis_mask & (1u << i)) == 0u) {
                        runtime_point.target_counts[i] = 0.0;
                        runtime_point.velocity_counts_per_s[i] = 0.0;
                        continue;
                      }
                      const double cpu = opt.axis[i].counts_per_unit;
                      const int sgn = opt.axis[i].sign;
                      const double raw_counts = cmd->q[i] * cpu * static_cast<double>(sgn);
                      runtime_point.target_counts[i] = clamp_double_to_i32_range(
                          controller_target_to_csp_wire_counts(
                              raw_counts, latest_feedback.native_home_position_offset[i]));
                      if ((cmd->flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) != 0u) {
                        const double raw_velocity =
                            cmd->qd[i] * cpu * static_cast<double>(sgn);
                        runtime_point.velocity_counts_per_s[i] =
                            clamp_double_to_i32_range(raw_velocity);
                      } else {
                        runtime_point.velocity_counts_per_s[i] = 0.0;
                      }
                    }
                    pending_upload.points[pending_upload.point_count] = runtime_point;
                    pending_upload.point_count += 1;
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_COMMIT: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdTrajectoryControlV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdTrajectoryControlV1*>(payload);
                    if (!pending_upload.active) {
                      const uint64_t update_ns = now_monotonic_ns();
                      logf("FAULT_COMMIT_C1 update_ns=%llu cmd_seq=%llu cmd_traj_id=%llu "
                           "pending_traj_id=%llu pending_active=%u pending_axis_mask=0x%x "
                           "pending_point_count=%u expected_points=%u",
                           static_cast<unsigned long long>(update_ns),
                           static_cast<unsigned long long>(mh->seq),
                           static_cast<unsigned long long>(cmd->traj_id),
                           static_cast<unsigned long long>(pending_upload.traj_id),
                           pending_upload.active ? 1u : 0u,
                           static_cast<unsigned int>(pending_upload.axis_mask),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned int>(pending_upload.expected_points));
                      publish_motion_status(
                          gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                          gradient::ipc::v1::EXEC_STATE_FAULTED,
                          cmd->traj_id,
                          std::numeric_limits<uint32_t>::max(),
                          0,
                          gradient::ipc::v1::EVT_TRAJECTORY_FAULTED,
                          0,
                          1,
                          mh->seq,
                          update_ns);
                      pending_upload = PendingTrajectoryUpload{};
                      break;
                    }
                    if (cmd->traj_id != pending_upload.traj_id) {
                      const uint64_t update_ns = now_monotonic_ns();
                      logf("FAULT_COMMIT_C2 update_ns=%llu cmd_seq=%llu cmd_traj_id=%llu "
                           "pending_traj_id=%llu pending_active=%u pending_axis_mask=0x%x "
                           "pending_point_count=%u expected_points=%u",
                           static_cast<unsigned long long>(update_ns),
                           static_cast<unsigned long long>(mh->seq),
                           static_cast<unsigned long long>(cmd->traj_id),
                           static_cast<unsigned long long>(pending_upload.traj_id),
                           pending_upload.active ? 1u : 0u,
                           static_cast<unsigned int>(pending_upload.axis_mask),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned int>(pending_upload.expected_points));
                      publish_motion_status(
                          gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                          gradient::ipc::v1::EXEC_STATE_FAULTED,
                          cmd->traj_id,
                          std::numeric_limits<uint32_t>::max(),
                          0,
                          gradient::ipc::v1::EVT_TRAJECTORY_FAULTED,
                          0,
                          1,
                          mh->seq,
                          update_ns);
                      pending_upload = PendingTrajectoryUpload{};
                      break;
                    }
                    if (pending_upload.point_count == 0) {
                      const uint64_t update_ns = now_monotonic_ns();
                      logf("FAULT_COMMIT_C3 update_ns=%llu cmd_seq=%llu cmd_traj_id=%llu "
                           "pending_traj_id=%llu pending_active=%u pending_axis_mask=0x%x "
                           "pending_point_count=%u expected_points=%u",
                           static_cast<unsigned long long>(update_ns),
                           static_cast<unsigned long long>(mh->seq),
                           static_cast<unsigned long long>(cmd->traj_id),
                           static_cast<unsigned long long>(pending_upload.traj_id),
                           pending_upload.active ? 1u : 0u,
                           static_cast<unsigned int>(pending_upload.axis_mask),
                           static_cast<unsigned int>(pending_upload.point_count),
                           static_cast<unsigned int>(pending_upload.expected_points));
                      publish_motion_status(
                          gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                          gradient::ipc::v1::EXEC_STATE_FAULTED,
                          cmd->traj_id,
                          std::numeric_limits<uint32_t>::max(),
                          0,
                          gradient::ipc::v1::EVT_TRAJECTORY_FAULTED,
                          0,
                          1,
                          mh->seq,
                          update_ns);
                      pending_upload = PendingTrajectoryUpload{};
                      break;
                    }

                    committed_trajectory.traj_id = pending_upload.traj_id;
                    committed_trajectory.cmd_seq = mh->seq;
                    committed_trajectory.axis_mask = pending_upload.axis_mask;
                    committed_trajectory.point_count = pending_upload.point_count;
                    for (uint32_t i = 0; i < pending_upload.point_count; ++i) {
                      committed_trajectory.points[i] = pending_upload.points[i];
                    }
                    committed_trajectory.seq.store(committed_traj_seq++, std::memory_order_release);
                    const uint64_t update_ns = now_monotonic_ns();
                    publish_motion_status(
                        gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                        gradient::ipc::v1::EXEC_STATE_QUEUED,
                        pending_upload.traj_id,
                        0,
                        pending_upload.point_count,
                        gradient::ipc::v1::EVT_TRAJECTORY_QUEUED,
                        0,
                        0,
                        mh->seq,
                        update_ns);
                    pending_upload = PendingTrajectoryUpload{};
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_ABORT: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdTrajectoryControlV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdTrajectoryControlV1*>(payload);
                    trajectory_abort_request.store(
                        cmd->traj_id == 0 ? std::numeric_limits<uint64_t>::max() : cmd->traj_id,
                        std::memory_order_release);
                    if (pending_upload.active &&
                        (cmd->traj_id == 0 || cmd->traj_id == pending_upload.traj_id)) {
                      pending_upload = PendingTrajectoryUpload{};
                    }
                    const uint64_t update_ns = now_monotonic_ns();
                    publish_motion_status(
                        gradient::ipc::v1::MOTION_MODE_TRAJECTORY,
                        gradient::ipc::v1::EXEC_STATE_ABORTED,
                        cmd->traj_id,
                        std::numeric_limits<uint32_t>::max(),
                        0,
                        gradient::ipc::v1::EVT_TRAJECTORY_ABORTED,
                        0,
                        1,
                        mh->seq,
                        update_ns);
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_JOG: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdJogV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdJogV1*>(payload);
                    latest_jog_command.axis_mask = cmd->axis_mask;
                    latest_jog_command.flags = cmd->flags;
                    latest_jog_command.timeout_ns = cmd->timeout_ns;
                    latest_jog_command.cmd_seq = mh->seq;
                    for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                      if ((cmd->axis_mask & (1u << i)) == 0u) {
                        latest_jog_command.velocity_counts_per_s[i] = 0.0;
                        continue;
                      }
                      const double cpu = opt.axis[i].counts_per_unit;
                      const int sgn = opt.axis[i].sign;
                      latest_jog_command.velocity_counts_per_s[i] =
                          cmd->qd[i] * cpu * static_cast<double>(sgn);
                    }
                    latest_jog_command.seq.store(mh->seq, std::memory_order_release);

                    const uint64_t update_ns = now_monotonic_ns();
                    const bool stop_requested =
                        (cmd->flags & gradient::ipc::v1::JOG_FLAG_STOP) != 0u;
                    publish_motion_status(
                        stop_requested ? gradient::ipc::v1::MOTION_MODE_IDLE
                                       : gradient::ipc::v1::MOTION_MODE_JOG,
                        stop_requested ? gradient::ipc::v1::EXEC_STATE_IDLE
                                       : gradient::ipc::v1::EXEC_STATE_ACCEPTED,
                        0,
                        std::numeric_limits<uint32_t>::max(),
                        0,
                        0,
                        0,
                        stop_requested ? 1 : 0,
                        mh->seq,
                        update_ns);
                  }
                  break;
                }
                default:
                  break;
              }

              r += 1;
            }
            cmd_ring.header->read_idx = r;
          }
        }

        uint64_t now = now_monotonic_ns();
        if (now >= next_snapshot_ns) {
          next_snapshot_ns = now + 100000000; // 100ms (10Hz) for scaffolding

          gradient::ipc::v1::StatusSnapshotV1 snap{};
          snap.num_axes = opt.num_axes;
          snap.wkc_expected = 2 * opt.num_axes;
          snap.wkc_actual = 0;
          snap.master_state = armed.load(std::memory_order_relaxed)
                                  ? gradient::ipc::v1::MASTER_SAFEOP
                                  : gradient::ipc::v1::MASTER_INIT;
          snap.dc_offset_ns = 0;
          snap.cycle_jitter_ns = rt_last_jitter_ns.load(std::memory_order_relaxed);
          snap.topology_hash = topology_hash;

          for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
            snap.axes[i] = AxisStatusV1{};
            snap.axes[i].ds402_state = gradient::ipc::v1::DS402_UNKNOWN;
            snap.axes[i].brake_state = gradient::ipc::v1::BRAKE_UNKNOWN;
          }

          // Prefer real feedback (once EtherCAT is active), else fall back to target counts.
          const uint64_t fb_seq = latest_feedback.seq.load(std::memory_order_acquire);
          if (fb_seq != 0) {
            snap.wkc_actual = latest_feedback.wkc_actual;
            if (latest_feedback.wkc_expected != 0) {
              snap.wkc_expected = latest_feedback.wkc_expected;
            }
            snap.master_state = latest_feedback.master_state;
            snap.dc_offset_ns = latest_feedback.dc_offset_ns;
            snap.cycle_jitter_ns = latest_feedback.cycle_jitter_ns;
            for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
              snap.axes[i].pos_counts = latest_feedback.pos_counts[i];
              snap.axes[i].torque_raw = latest_feedback.torque_raw[i];
              snap.axes[i].statusword = latest_feedback.statusword[i];
              snap.axes[i].error_code = latest_feedback.error_code[i];
              snap.axes[i].manufacturer_error_code = latest_feedback.manufacturer_error_code[i];
              snap.axes[i].mode_display = latest_feedback.mode_display[i];
              snap.axes[i].ds402_state = latest_feedback.ds402_state[i];
              snap.axes[i].di_bits = latest_feedback.di_bits[i];
            }
          } else {
            // No live process data yet; leave snapshot at zero/default values.
          }

          ring_write(status_ring,
                     gradient::ipc::v1::MSG_STATUS_SNAPSHOT,
                     &snap,
                     sizeof(snap),
                     &status_seq,
                     now);
          eventfd_write_one(status_eventfd);

          gradient::ipc::v1::StatusMotionStateV1 motion{};
          motion.active_mode = motion_active_mode.load(std::memory_order_relaxed);
          motion.state = motion_exec_state.load(std::memory_order_relaxed);
          motion.active_traj_id = motion_active_traj_id.load(std::memory_order_relaxed);
          motion.current_point_index = motion_current_point_index.load(std::memory_order_relaxed);
          motion.queue_depth = motion_queue_depth.load(std::memory_order_relaxed);
          motion.queue_capacity = kMaxTrajectoryPoints;
          motion.last_event_code = motion_last_event_code.load(std::memory_order_relaxed);
          motion.underrun_count = motion_underrun_count.load(std::memory_order_relaxed);
          motion.stale_command_flag = motion_stale_command_flag.load(std::memory_order_relaxed);
          motion.motion_done = motion_done.load(std::memory_order_relaxed);
          motion.capability_flags = motion_capability_flags.load(std::memory_order_relaxed);
          motion.active_command_seq = motion_active_command_seq.load(std::memory_order_relaxed);
          motion.last_update_ns = motion_last_update_ns.load(std::memory_order_relaxed);

          ring_write(status_ring,
                     gradient::ipc::v1::MSG_STATUS_MOTION_STATE,
                     &motion,
                     sizeof(motion),
                     &status_seq,
                     now);
          eventfd_write_one(status_eventfd);
        }

        if (now >= next_jog_debug_ns) {
          next_jog_debug_ns = now + 20000000; // 20ms (50Hz) for jog stop instrumentation
          const uint64_t dbg_seq = latest_jog_debug.seq.load(std::memory_order_acquire);
          if (dbg_seq != 0) {
            gradient::ipc::v1::StatusJogDebugV1 jog_debug{};
            jog_debug.num_axes = latest_jog_debug.num_axes;
            jog_debug.active_jog = latest_jog_debug.active_jog;
            jog_debug.active_jog_axis_mask = latest_jog_debug.active_jog_axis_mask;
            jog_debug.command_sp_mask = latest_jog_debug.command_sp_mask;
            jog_debug.have_hold_mask = latest_jog_debug.have_hold_mask;
            jog_debug.have_jog_target_mask = latest_jog_debug.have_jog_target_mask;
            jog_debug.snap_hold_mask = latest_jog_debug.snap_hold_mask;
            jog_debug.stop_arrest_mask = latest_jog_debug.stop_arrest_mask;
            jog_debug.latest_cmd_axis_mask = latest_jog_debug.latest_cmd_axis_mask;
            jog_debug.latest_cmd_flags = latest_jog_debug.latest_cmd_flags;
            jog_debug.last_stop_reason = latest_jog_debug.last_stop_reason;
            jog_debug.last_stop_axis_mask = latest_jog_debug.last_stop_axis_mask;
            jog_debug.sample_time_ns = latest_jog_debug.sample_time_ns;
            jog_debug.active_jog_cmd_seq = latest_jog_debug.active_jog_cmd_seq;
            jog_debug.latest_jog_seq_seen = latest_jog_debug.latest_jog_seq_seen;
            jog_debug.active_jog_deadline_ns = latest_jog_debug.active_jog_deadline_ns;
            jog_debug.latest_cmd_timeout_ns = latest_jog_debug.latest_cmd_timeout_ns;
            jog_debug.last_stop_time_ns = latest_jog_debug.last_stop_time_ns;
            jog_debug.last_stop_cmd_seq = latest_jog_debug.last_stop_cmd_seq;
            std::copy(latest_jog_debug.feedback_pos_counts.begin(),
                      latest_jog_debug.feedback_pos_counts.end(),
                      jog_debug.feedback_pos_counts);
            std::copy(latest_jog_debug.hold_target_counts.begin(),
                      latest_jog_debug.hold_target_counts.end(),
                      jog_debug.hold_target_counts);
            std::copy(latest_jog_debug.output_target_counts.begin(),
                      latest_jog_debug.output_target_counts.end(),
                      jog_debug.output_target_counts);
            std::copy(latest_jog_debug.output_target_velocity_counts_per_s.begin(),
                      latest_jog_debug.output_target_velocity_counts_per_s.end(),
                      jog_debug.output_target_velocity_counts_per_s);

            ring_write(status_ring,
                       gradient::ipc::v1::MSG_STATUS_JOG_DEBUG,
                       &jog_debug,
                       sizeof(jog_debug),
                       &status_seq,
                       now);
            eventfd_write_one(status_eventfd);
          }
        }
      }
    });

    logf("IPC handshake complete (cmd_shm=%zu bytes, status_shm=%zu bytes)",
         cmd_shm.bytes, status_shm.bytes);
  }

  // Shutdown.
  reset_connection();

  g_stop.store(true, std::memory_order_relaxed);
  if (rt_thread.joinable()) {
    rt_thread.join();
  }
  if (metrics_thread.joinable()) {
    metrics_thread.join();
  }
  if (fast_trace_thread.joinable()) {
    fast_trace_thread.join();
  }

  unlink(opt.socket_path.c_str());
  close(server_fd);

  logf("Stopped.");
  return process_exit_code.load(std::memory_order_relaxed);
}

