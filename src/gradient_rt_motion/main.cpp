#include "ipc_v1.hpp"

#include <atomic>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <limits>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <grp.h>
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
#include "a6ec_pdo.hpp"
#include "ds402.hpp"

#ifndef MFD_CLOEXEC
#define MFD_CLOEXEC 0x0001U
#endif

namespace {

std::atomic<bool> g_stop{false};

extern "C" void handle_signal(int) {
  g_stop.store(true, std::memory_order_relaxed);
}

uint64_t now_monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
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
  uint8_t axis_type = gradient::ipc::v1::AXIS_TYPE_ROTARY; // q is radians by default
  double lead_m_per_rev = 0.0; // only used when axis_type==AXIS_TYPE_LINEAR

  // Derived scaling (RTCore uses this for q->counts conversion).
  double counts_per_unit = 0.0; // counts per rad (rotary) or counts per meter (linear)

  // Derived safety clamps from --max-rpm (motor rpm). 0 => disabled.
  int32_t max_step_counts_per_cycle = 0;
  uint32_t max_profile_vel_counts_per_s = 0;
};

struct Options {
  std::string socket_path = "/run/gradient-rt-motion/ipc.sock";
  // Filesystem group that is allowed to connect to the IPC socket (0660).
  // Default matches the appliance user/group.
  std::string ipc_group = "pi";
  uint64_t cycle_ns = 1000000; // 1 kHz
  uint32_t num_axes = 6;       // default arm axes for early scaffolding
  uint32_t drive_profile_id = gradient::ipc::v1::DRIVE_PROFILE_A6EC_DS402;

  // Axis scaling (bring-up defaults; tuned via commissioning).
  // Per-axis lists can be provided via the CLI (comma-separated).
  std::array<AxisConfig, gradient::ipc::v1::GRADIENT_MAX_AXES> axis{};
  std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_position{};

  // Safety: cap commanded motor speed (rpm). 0 disables clamping.
  double max_rpm = 100.0;

  // EtherCAT bring-up policy.
  uint16_t rx_pdo = gradient::a6ec::kRxPdo;
  uint16_t tx_pdo = gradient::a6ec::kTxPdo;
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
};

const char* pdo_profile_label(uint16_t rx_pdo, uint16_t tx_pdo) {
  if (rx_pdo == 0x1701 && tx_pdo == 0x1B01) {
    return "1701/1b01";
  }
  if (rx_pdo == 0x1702 && tx_pdo == 0x1B02) {
    return "1702/1b02";
  }
  return "custom";
}

bool is_supported_pdo_profile(uint16_t rx_pdo, uint16_t tx_pdo) {
  return (rx_pdo == 0x1701 && tx_pdo == 0x1B01) ||
         (rx_pdo == 0x1702 && tx_pdo == 0x1B02);
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
  if (t == "a6ec_ds402" || t == "a6ec" || t == "a6ec-ds402") {
    *out = gradient::ipc::v1::DRIVE_PROFILE_A6EC_DS402;
    return true;
  }
  if (t == "cia402" || t == "cia") {
    *out = gradient::ipc::v1::DRIVE_PROFILE_CIA402;
    return true;
  }
  uint32_t numeric = 0;
  if (parse_u32(t.c_str(), &numeric)) {
    *out = numeric;
    return true;
  }
  return false;
}

std::string trim_ascii_ws(const std::string& s) {
  const size_t first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const size_t last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
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
      "[--axis-type T[,T..]] [--lead-m-per-rev M[,M..]] [--drive-profile ID] [--max-rpm RPM] "
      "[--slave-positions P[,P..]] "
      "[--rx-pdo ID] [--tx-pdo ID] [--no-dc] [--disable-output-watchdog] "
      "[--split-domains-per-axis] [--queue-split-domains-round-robin] [--explicit-pdo-config] "
      "[--wait-before-safeop-ms MS] [--preop-safeop-timeout-ms MS] "
      "[--safeop-op-timeout-ms MS] [--startup-passive-ms MS] "
      "[--startup-skip-domain-queue-ms MS] [--ipc-group NAME]\n\n"
      "Defaults:\n"
      "  --socket-path /run/gradient-rt-motion/ipc.sock\n"
      "  --cycle-ns     1000000\n"
      "  --num-axes     6\n"
      "  --counts-per-rev 131072\n"
      "  --gear-ratio   1.0\n"
      "  --sign         +1\n"
      "  --axis-type    rotary\n"
      "  --drive-profile a6ec_ds402\n"
      "  --max-rpm      100\n"
      "  --slave-positions 0,1,2,...\n"
      "  --rx-pdo       0x1702\n"
      "  --tx-pdo       0x1B02\n"
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
      "  --ipc-group    pi\n",
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
        logf("ERROR: invalid --drive-profile (expected a6ec_ds402, cia402, or numeric id)");
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
  if (!finalize_slave_positions(&opt, slave_positions_spec)) {
    return 2;
  }

  if (!is_supported_pdo_profile(opt.rx_pdo, opt.tx_pdo)) {
    logf("ERROR: unsupported PDO profile rx=0x%04x tx=0x%04x "
         "(supported: 0x1701/0x1b01 or 0x1702/0x1b02)",
         static_cast<unsigned int>(opt.rx_pdo),
         static_cast<unsigned int>(opt.tx_pdo));
    return 2;
  }

#if GRADIENT_HAVE_ECRT
  // A6-EC (CiA402 over EtherCAT) uses DC/SYNC0; the sync cycle must be a multiple
  // of 250 μs (manual ch8; otherwise the drive can raise Er74.0).
  constexpr uint64_t kA6ecDcQuantumNs = 250000; // 250 μs
  if (opt.use_dc && (opt.cycle_ns % kA6ecDcQuantumNs) != 0) {
    logf("ERROR: --cycle-ns (%llu) must be a multiple of %llu ns (250 us) for A6-EC DC/SYNC0",
         static_cast<unsigned long long>(opt.cycle_ns),
         static_cast<unsigned long long>(kA6ecDcQuantumNs));
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

  // Shared state (helper thread produces; RT thread consumes).
  struct LatestSetpoint {
    std::atomic<uint64_t> seq{0};
    uint64_t target_time_ns = 0;
    uint32_t axis_mask = 0;
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> q{};
  };

  struct LatestTargets {
    std::atomic<uint64_t> seq{0};
    uint64_t target_time_ns = 0;
    uint32_t axis_mask = 0;
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> target_counts{};
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
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pos_counts{};
    std::array<int16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> torque_raw{};
    std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> statusword{};
    std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> error_code{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> mode_display{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ds402_state{};
    std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> di_bits{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_al_state{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_online{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_operational{};
  };

  std::atomic<bool> armed{false};
  std::atomic<uint32_t> axis_enable_mask{0};
  std::atomic<int32_t> mode_of_operation{0}; // e.g. 8=CSP
  std::atomic<uint32_t> fault_reset_request{0}; // bitmask; helper thread -> RT thread
  LatestSetpoint latest_setpoint{};
  LatestTargets latest_targets{};
  LatestFeedback latest_feedback{};

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
    uint64_t next_ns = now_monotonic_ns();
    bool shutdown_active = false;
    uint64_t shutdown_until_ns = 0;
    // Grace window to push DS402 disable (controlword=0) before dropping the bus.
    constexpr uint64_t kShutdownGraceNs = 250000000ULL; // 250 ms

#if GRADIENT_HAVE_ECRT
    // -----------------------------------------------------------------------
    // IgH libecrt setup (A6-EC DS402 drives, CSP mode)
    // -----------------------------------------------------------------------
    //
    // NOTE: This block compiles only when IgH headers are present. It is
    // intentionally "init-only" work; the cyclic loop below avoids allocation.
    ec_master_t* master = nullptr;
    ec_domain_t* domain = nullptr;
    std::array<ec_domain_t*, gradient::ipc::v1::GRADIENT_MAX_AXES> axis_domain{};
    uint8_t* domain_pd = nullptr;
    std::array<uint8_t*, gradient::ipc::v1::GRADIENT_MAX_AXES> axis_domain_pd{};
    ec_master_state_t master_diag_state{};
    ec_domain_state_t domain_state{};

    constexpr unsigned int kInvalidOffset = std::numeric_limits<unsigned int>::max();
    constexpr size_t kMaxPdoRegsPerAxis = 16;

    struct AxisOffsets {
      unsigned int cw = kInvalidOffset;
      unsigned int target_pos = kInvalidOffset;
      unsigned int target_vel = kInvalidOffset;
      unsigned int target_torque = kInvalidOffset;
      unsigned int mode = kInvalidOffset;
      unsigned int tp_func = kInvalidOffset;
      unsigned int max_profile_vel = kInvalidOffset;

      unsigned int err = kInvalidOffset;
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

    // Supported A6-EC PDO profiles from the vendor ESI.
    static ec_pdo_entry_info_t a6ec_entries_1701_1b01[] = {
        // RxPDO 0x1701 (4 entries)
        {0x6040, 0x00, 16}, // Controlword
        {0x607A, 0x00, 32}, // Target position
        {0x60B8, 0x00, 16}, // Touch probe function
        {0x60FE, 0x01, 32}, // Physical outputs
        // TxPDO 0x1B01 (9 entries)
        {0x603F, 0x00, 16}, // Error code
        {0x6041, 0x00, 16}, // Statusword
        {0x6064, 0x00, 32}, // Position actual value
        {0x6077, 0x00, 16}, // Torque actual value
        {0x60F4, 0x00, 32}, // Following error actual value
        {0x60B9, 0x00, 16}, // Touch probe status
        {0x60BA, 0x00, 32}, // Touch probe pos1 value
        {0x60BC, 0x00, 32}, // Touch probe pos2 value
        {0x60FD, 0x00, 32}, // Digital inputs
    };

    static ec_pdo_info_t a6ec_pdos_1701_1b01[] = {
        {0x1701, 4, a6ec_entries_1701_1b01 + 0},
        {0x1B01, 9, a6ec_entries_1701_1b01 + 4},
    };

    static ec_sync_info_t a6ec_syncs_1701_1b01[] = {
        {2, EC_DIR_OUTPUT, 1, a6ec_pdos_1701_1b01 + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, a6ec_pdos_1701_1b01 + 1, EC_WD_DISABLE},
        {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE},
    };

    static ec_pdo_entry_info_t a6ec_entries_1702_1b02[] = {
        // RxPDO 0x1702 (7 entries)
        {0x6040, 0x00, 16}, // Controlword
        {0x607A, 0x00, 32}, // Target position
        {0x60FF, 0x00, 32}, // Target velocity
        {0x6071, 0x00, 16}, // Target torque
        {0x6060, 0x00, 8},  // Modes of operation
        {0x60B8, 0x00, 16}, // Touch probe function
        {0x607F, 0x00, 32}, // Max profile velocity
        // TxPDO 0x1B02 (9 entries)
        {0x603F, 0x00, 16}, // Error code
        {0x6041, 0x00, 16}, // Statusword
        {0x6064, 0x00, 32}, // Position actual value
        {0x6077, 0x00, 16}, // Torque actual value
        {0x6061, 0x00, 8},  // Modes of operation display
        {0x60B9, 0x00, 16}, // Touch probe status
        {0x60BA, 0x00, 32}, // Touch probe pos1 value
        {0x60BC, 0x00, 32}, // Touch probe pos2 value
        {0x60FD, 0x00, 32}, // Digital inputs
    };

    static ec_pdo_info_t a6ec_pdos_1702_1b02[] = {
        {0x1702, 7, a6ec_entries_1702_1b02 + 0},
        {0x1B02, 9, a6ec_entries_1702_1b02 + 7},
    };

    static ec_sync_info_t a6ec_syncs_1702_1b02[] = {
        {2, EC_DIR_OUTPUT, 1, a6ec_pdos_1702_1b02 + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, a6ec_pdos_1702_1b02 + 1, EC_WD_DISABLE},
        {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE},
    };

    struct PdoProfileConfig {
      const char* name = "custom";
      uint16_t rx_pdo = 0;
      uint16_t tx_pdo = 0;
      const ec_sync_info_t* syncs = nullptr;
      bool has_target_vel = false;
      bool has_target_torque = false;
      bool has_mode = false;
      bool has_max_profile_vel = false;
      bool has_mode_display = false;
    };

    static const PdoProfileConfig kPdoProfile1701_1b01{
        "1701/1b01",
        0x1701,
        0x1B01,
        a6ec_syncs_1701_1b01,
        false,
        false,
        false,
        false,
        false,
    };

    static const PdoProfileConfig kPdoProfile1702_1b02{
        "1702/1b02",
        0x1702,
        0x1B02,
        a6ec_syncs_1702_1b02,
        true,
        true,
        true,
        true,
        true,
    };

    const PdoProfileConfig* pdo_profile = nullptr;
    if (opt.rx_pdo == kPdoProfile1701_1b01.rx_pdo &&
        opt.tx_pdo == kPdoProfile1701_1b01.tx_pdo) {
      pdo_profile = &kPdoProfile1701_1b01;
    } else if (opt.rx_pdo == kPdoProfile1702_1b02.rx_pdo &&
               opt.tx_pdo == kPdoProfile1702_1b02.tx_pdo) {
      pdo_profile = &kPdoProfile1702_1b02;
    }
    if (!pdo_profile) {
      logf("ERROR: unsupported PDO profile rx=0x%04x tx=0x%04x",
           static_cast<unsigned int>(opt.rx_pdo),
           static_cast<unsigned int>(opt.tx_pdo));
      return;
    }

    std::array<ec_sync_info_t, 3> runtime_syncs{};
    for (size_t i = 0; i < runtime_syncs.size(); ++i) {
      runtime_syncs[i] = pdo_profile->syncs[i];
    }
    if (opt.disable_output_watchdog) {
      runtime_syncs[0].watchdog_mode = EC_WD_DISABLE;
    }

    logf("EtherCAT bring-up config: pdo_profile=%s rx=0x%04x tx=0x%04x dc=%u "
         "output_watchdog=%u split_domains_per_axis=%u queue_split_domains_round_robin=%u "
         "explicit_pdo_config=%u "
         "wait_before_safeop_ms=%u preop_safeop_timeout_ms=%u safeop_op_timeout_ms=%u "
         "startup_passive_ms=%u startup_skip_domain_queue_ms=%u startup_diag_window_ms=%llu",
         pdo_profile->name,
         static_cast<unsigned int>(pdo_profile->rx_pdo),
         static_cast<unsigned int>(pdo_profile->tx_pdo),
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
        logf("EtherCAT phase=pdo_register axis=%u slave_pos=%u offsets cw=%u target_pos=%u sw=%u pos=%u err=%u di=%u",
             i,
             static_cast<unsigned int>(opt.slave_position[i]),
             off[i].cw,
             off[i].target_pos,
             off[i].sw,
             off[i].pos,
             off[i].err,
             off[i].di);
      }
    };

    auto fill_axis_regs = [&](uint32_t axis, auto& regs, size_t& reg_i) {
      const uint16_t slave_pos = opt.slave_position[axis];
      auto add_reg = [&](uint16_t index, uint8_t subindex, unsigned int* offset) {
        regs[reg_i++] = {0,
                         slave_pos,
                         gradient::a6ec::kVendorId,
                         gradient::a6ec::kProductCode,
                         index,
                         subindex,
                         offset,
                         nullptr};
      };

      add_reg(0x6040, 0x00, &off[axis].cw);
      add_reg(0x607A, 0x00, &off[axis].target_pos);
      if (pdo_profile->has_target_vel) {
        add_reg(0x60FF, 0x00, &off[axis].target_vel);
      }
      if (pdo_profile->has_target_torque) {
        add_reg(0x6071, 0x00, &off[axis].target_torque);
      }
      if (pdo_profile->has_mode) {
        add_reg(0x6060, 0x00, &off[axis].mode);
      }
      add_reg(0x60B8, 0x00, &off[axis].tp_func);
      if (pdo_profile->has_max_profile_vel) {
        add_reg(0x607F, 0x00, &off[axis].max_profile_vel);
      }

      add_reg(0x603F, 0x00, &off[axis].err);
      add_reg(0x6041, 0x00, &off[axis].sw);
      add_reg(0x6064, 0x00, &off[axis].pos);
      add_reg(0x6077, 0x00, &off[axis].torque);
      if (pdo_profile->has_mode_display) {
        add_reg(0x6061, 0x00, &off[axis].mode_disp);
      }
      add_reg(0x60B9, 0x00, &off[axis].tp_status);
      add_reg(0x60BA, 0x00, &off[axis].tp_pos1);
      add_reg(0x60BC, 0x00, &off[axis].tp_pos2);
      add_reg(0x60FD, 0x00, &off[axis].di);
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
             pdo_profile->name);
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
           pdo_profile->name);
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

      if (!reg(2, 0, &off[axis].cw, "cw") ||
          !reg(2, 1, &off[axis].target_pos, "target_pos")) {
        return false;
      }

      if (pdo_profile == &kPdoProfile1702_1b02) {
        if (!reg(2, 2, &off[axis].target_vel, "target_vel") ||
            !reg(2, 3, &off[axis].target_torque, "target_torque") ||
            !reg(2, 4, &off[axis].mode, "mode") ||
            !reg(2, 5, &off[axis].tp_func, "tp_func") ||
            !reg(2, 6, &off[axis].max_profile_vel, "max_profile_vel") ||
            !reg(3, 0, &off[axis].err, "err") ||
            !reg(3, 1, &off[axis].sw, "sw") ||
            !reg(3, 2, &off[axis].pos, "pos") ||
            !reg(3, 3, &off[axis].torque, "torque") ||
            !reg(3, 4, &off[axis].mode_disp, "mode_disp") ||
            !reg(3, 5, &off[axis].tp_status, "tp_status") ||
            !reg(3, 6, &off[axis].tp_pos1, "tp_pos1") ||
            !reg(3, 7, &off[axis].tp_pos2, "tp_pos2") ||
            !reg(3, 8, &off[axis].di, "di")) {
          return false;
        }
      } else if (pdo_profile == &kPdoProfile1701_1b01) {
        if (!reg(2, 2, &off[axis].tp_func, "tp_func") ||
            !reg(3, 0, &off[axis].err, "err") ||
            !reg(3, 1, &off[axis].sw, "sw") ||
            !reg(3, 2, &off[axis].pos, "pos") ||
            !reg(3, 3, &off[axis].torque, "torque") ||
            !reg(3, 5, &off[axis].tp_status, "tp_status") ||
            !reg(3, 6, &off[axis].tp_pos1, "tp_pos1") ||
            !reg(3, 7, &off[axis].tp_pos2, "tp_pos2") ||
            !reg(3, 8, &off[axis].di, "di")) {
          return false;
        }
      } else {
        logf("ERROR: unsupported explicit PDO profile registration for axis=%u slave_pos=%u profile=%s",
             axis,
             slave_pos,
             pdo_profile->name);
        return false;
      }

      return true;
    };

    master = ecrt_request_master(0);
    if (!master) {
      logf("ERROR: ecrt_request_master(0) failed");
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
                                           gradient::a6ec::kVendorId,
                                           gradient::a6ec::kProductCode);
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
          mode_of_operation.store(0, std::memory_order_relaxed);
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

        // Read latest targets (double-read seq).
        std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> target_counts{};
        uint32_t sp_mask = 0;
        {
          const uint64_t s1 = latest_targets.seq.load(std::memory_order_acquire);
          target_counts = latest_targets.target_counts;
          sp_mask = latest_targets.axis_mask;
          const uint64_t s2 = latest_targets.seq.load(std::memory_order_acquire);
          if (s1 != s2) {
            // If torn, just hold (keep previous hold_target_counts).
            sp_mask = 0;
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
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          uint8_t* axis_pd = opt.split_domains_per_axis ? axis_domain_pd[i] : domain_pd;
          const uint16_t sw = EC_READ_U16(axis_pd + off[i].sw);
          const uint16_t err = EC_READ_U16(axis_pd + off[i].err);
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

          if (!startup_passive_active) {
            const bool want_enable = is_armed && ((en_mask & (1u << i)) != 0u);
            bool want_fault_reset = (fault_reset_left[i] > 0);
            if (want_fault_reset && st != gradient::ds402::State::Fault) {
              // Stop pulsing once the drive leaves FAULT (or if it never was in FAULT).
              fault_reset_left[i] = 0;
              want_fault_reset = false;
            }

            // Hold-target logic:
            // - While not operation-enabled (or not wanting enable), keep the target aligned to feedback
            //   so we don't present a "big step" when DS402 transitions to OperationEnabled (manual: Er87.*).
            // - Once operation-enabled, latch once, then track commanded targets with optional per-cycle clamp.
            if (want_enable && st == gradient::ds402::State::OperationEnabled) {
              if (!have_hold[i]) {
                hold_target_counts[i] = pos;
                have_hold[i] = true;
              }
              if ((sp_mask & (1u << i)) != 0u) {
                const int32_t desired = target_counts[i];
                const int32_t max_step = opt.axis[i].max_step_counts_per_cycle;
                if (max_step > 0) {
                  const int32_t cur = hold_target_counts[i];
                  const int64_t delta = static_cast<int64_t>(desired) - static_cast<int64_t>(cur);
                  if (delta > max_step) {
                    hold_target_counts[i] = cur + max_step;
                  } else if (delta < -max_step) {
                    hold_target_counts[i] = cur - max_step;
                  } else {
                    hold_target_counts[i] = desired;
                  }
                } else {
                  hold_target_counts[i] = desired;
                }
              }
            } else {
              // Track feedback until OP; keeps 0x607A aligned during state transitions.
              hold_target_counts[i] = pos;
              have_hold[i] = false;
            }

            cw = gradient::ds402::controlword_for_enable(st, want_enable, want_fault_reset);
            if (want_fault_reset && st == gradient::ds402::State::Fault && fault_reset_left[i] > 0) {
              fault_reset_left[i]--;
            }
            target_pos_out = hold_target_counts[i];
            target_vel_out = 0;
            target_torque_out = 0;
            mode_out = want_enable ? 8 : 0;
            tp_func_out = 0;
            max_profile_vel_out = opt.axis[i].max_profile_vel_counts_per_s;
          } else {
            // During passive startup we still exchange cyclic PDO frames, but avoid
            // DS402 state-driving writes so we can distinguish process-data transport
            // problems from higher-level controlword/mode/target sequencing problems.
            hold_target_counts[i] = pos;
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

          // Publish raw feedback (counts + status) for STATUS_SNAPSHOT.
          latest_feedback.pos_counts[i] = pos;
          latest_feedback.torque_raw[i] = torque;
          latest_feedback.statusword[i] = sw;
          latest_feedback.error_code[i] = err;
          latest_feedback.mode_display[i] = mode_disp;
          latest_feedback.ds402_state[i] = static_cast<uint8_t>(st);
          latest_feedback.di_bits[i] = di;
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
      ecrt_release_master(master); // deactivates if active
      master = nullptr;
    }
#endif
  });

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
      std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pos_counts{};
      std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> statusword{};
      std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> error_code{};
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
        pos_counts = latest_feedback.pos_counts;
        statusword = latest_feedback.statusword;
        error_code = latest_feedback.error_code;
        slave_al_state = latest_feedback.slave_al_state;
        slave_online = latest_feedback.slave_online;
        slave_operational = latest_feedback.slave_operational;
        const uint64_t s2 = latest_feedback.seq.load(std::memory_order_acquire);
        have_fb = (s1 == s2);
      }

      const uint32_t en_mask = axis_enable_mask.load(std::memory_order_relaxed);

      std::ostringstream oss;
      oss << "{";
      oss << "\"time_ns\":" << now_ns << ",";
      oss << "\"cycle_ns\":" << opt.cycle_ns << ",";
      oss << "\"num_axes\":" << opt.num_axes << ",";
      oss << "\"pdo_profile\":\"" << pdo_profile_label(opt.rx_pdo, opt.tx_pdo) << "\",";
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
      for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
        if (i != 0) {
          oss << ",";
        }
        oss << "{";
        oss << "\"pos_counts\":" << pos_counts[i] << ",";
        oss << "\"statusword\":" << statusword[i] << ",";
        oss << "\"error_code\":" << error_code[i] << ",";
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
    const size_t cmd_setpoint_offset =
        align_up(sizeof(gradient::ipc::v1::ShmHeaderV1) + cmd_ring_bytes, 64);
    const size_t cmd_shm_bytes =
        align_up(cmd_setpoint_offset + sizeof(gradient::ipc::v1::SetpointSlotV1), 4096);

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
      hdr->setpoint_offset = static_cast<uint32_t>(cmd_setpoint_offset);

      auto ring = make_ring_view(cmd_shm.base, hdr);
      ring.header->magic = gradient::ipc::v1::kMagicRing;
      ring.header->capacity = hdr->ring_capacity;
      ring.header->msg_bytes = hdr->ring_msg_bytes;
      ring.header->write_idx = 0;
      ring.header->read_idx = 0;
      ring.header->dropped = 0;

      auto* slot = reinterpret_cast<gradient::ipc::v1::SetpointSlotV1*>(
          static_cast<uint8_t*>(cmd_shm.base) + hdr->setpoint_offset);
      std::memset(slot, 0, sizeof(*slot));
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
      auto* setpoint_slot = reinterpret_cast<gradient::ipc::v1::SetpointSlotV1*>(
          static_cast<uint8_t*>(cmd_shm.base) + cmd_hdr->setpoint_offset);

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

      uint64_t next_snapshot_ns = now_monotonic_ns();
      uint64_t last_setpoint_seen = 0;

      while (helper_running.load(std::memory_order_relaxed) &&
             !g_stop.load(std::memory_order_relaxed)) {
        pollfd pfds[1]{};
        pfds[0].fd = cmd_eventfd;
        pfds[0].events = POLLIN;

        // Wake periodically for status snapshots even if no commands arrive.
        int timeout_ms = 50;
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
                    axis_enable_mask.store(cur & ~cmd->axis_mask, std::memory_order_relaxed);
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_SET_MODE: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdSetModeV1)) {
                    auto* cmd = reinterpret_cast<const gradient::ipc::v1::CmdSetModeV1*>(payload);
                    mode_of_operation.store(static_cast<int32_t>(cmd->mode),
                                            std::memory_order_relaxed);
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
                      fault_reset_request.fetch_or(mask, std::memory_order_relaxed);
                    }
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

        // Read latest setpoint slot (latest-wins) and publish into local snapshot.
        if (cmd_hdr->setpoint_offset != 0 && setpoint_slot) {
          // Double-read sequence pattern (writer increments seq after writing fields).
          const uint64_t s1 = setpoint_slot->seq;
          const uint64_t target_time = setpoint_slot->target_time_ns;
          const uint32_t axis_mask = setpoint_slot->axis_mask;
          std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> q{};
          for (size_t i = 0; i < q.size(); ++i) {
            q[i] = setpoint_slot->q[i];
          }
          const uint64_t s2 = setpoint_slot->seq;

          if (s1 == s2 && s1 != 0) {
            latest_setpoint.target_time_ns = target_time;
            latest_setpoint.axis_mask = axis_mask;
            latest_setpoint.q = q;
            latest_setpoint.seq.store(s1, std::memory_order_release);

            if (s1 != last_setpoint_seen) {
              // Convert q (axis units) to target counts for the cyclic thread.
              LatestTargets tmp{};
              tmp.target_time_ns = target_time;
              tmp.axis_mask = axis_mask;
              for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                const double cpu = opt.axis[i].counts_per_unit;
                const int sgn = opt.axis[i].sign;
                const double raw = q[i] * cpu;
                const long long rounded = std::llround(raw);
                long long counts = static_cast<long long>(sgn) * rounded;
                if (counts > static_cast<long long>(std::numeric_limits<int32_t>::max())) {
                  counts = static_cast<long long>(std::numeric_limits<int32_t>::max());
                } else if (counts < static_cast<long long>(std::numeric_limits<int32_t>::min())) {
                  counts = static_cast<long long>(std::numeric_limits<int32_t>::min());
                }
                tmp.target_counts[i] = static_cast<int32_t>(counts);
              }
              latest_targets.target_time_ns = tmp.target_time_ns;
              latest_targets.axis_mask = tmp.axis_mask;
              latest_targets.target_counts = tmp.target_counts;
              latest_targets.seq.store(s1, std::memory_order_release);
              last_setpoint_seen = s1;
            }
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
              snap.axes[i].mode_display = latest_feedback.mode_display[i];
              snap.axes[i].ds402_state = latest_feedback.ds402_state[i];
              snap.axes[i].di_bits = latest_feedback.di_bits[i];
            }
          } else {
            // Until EtherCAT is wired up, expose the current target counts as "position" for visibility.
            // This makes it easy to validate the Python->RTCore setpoint path before libecrt is present.
            const uint64_t tc_seq = latest_targets.seq.load(std::memory_order_acquire);
            if (tc_seq != 0) {
              for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                snap.axes[i].pos_counts = latest_targets.target_counts[i];
              }
            }
          }

          ring_write(status_ring,
                     gradient::ipc::v1::MSG_STATUS_SNAPSHOT,
                     &snap,
                     sizeof(snap),
                     &status_seq,
                     now);
          eventfd_write_one(status_eventfd);
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

  unlink(opt.socket_path.c_str());
  close(server_fd);

  logf("Stopped.");
  return 0;
}

