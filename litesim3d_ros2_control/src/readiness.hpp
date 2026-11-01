#pragma once
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>

namespace litesim3d::control {
inline double startup_timeout(const std::string &value) {
  std::size_t consumed = 0; const double timeout = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(timeout) || timeout <= 0 || timeout > 3600)
    throw std::runtime_error("timeout must be finite and in (0, 3600] seconds");
  return timeout;
}
class Readiness {
 public:
  void status(std::map<std::string, std::string> values) {
    fault_ = values["state"] == "fault";
    const bool current = values["ready"] == "true";
    if (current != ready_ || values["run_id"] != run_id_) { initial_.reset(); advancing_ = false; }
    ready_ = current; run_id_ = values["run_id"];
  }
  void clock(int64_t stamp) {
    if (!ready_) return;
    if (!initial_ || stamp < *initial_) advancing_ = false;
    else if (stamp > *initial_) advancing_ = true;
    initial_ = stamp;
  }
  bool ready() const { return ready_ && advancing_ && !fault_; }
  bool fault() const { return fault_; }
 private:
  bool ready_ = false, advancing_ = false, fault_ = false;
  std::string run_id_;
  std::optional<int64_t> initial_;
};
}  // namespace litesim3d::control
