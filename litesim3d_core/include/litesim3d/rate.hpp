#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>

namespace litesim3d {
// Wall-clock samples of committed simulation time. Never use /clock as the
// denominator, never sum robot clocks, and reset on epoch/time discontinuities.
class RealTimeRate {
public:
  std::optional<double> sample(const std::string &epoch, uint64_t stamp, double wall) {
    if (epoch != epoch_ || (!samples_.empty() &&
        (stamp < samples_.back().second || wall <= samples_.back().first))) {
      samples_.clear(); epoch_ = epoch;
    }
    samples_.emplace_back(wall, stamp);
    while (samples_.size() > 2 && samples_[1].first <= wall - 1.0) samples_.pop_front();
    const double span = wall - samples_.front().first;
    if (span < .05) return std::nullopt;
    return double(stamp - samples_.front().second) * 1e-9 / span;
  }
private:
  std::string epoch_;
  std::deque<std::pair<double, uint64_t>> samples_;
};
} // namespace litesim3d
