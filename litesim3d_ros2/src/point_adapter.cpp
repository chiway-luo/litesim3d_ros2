#include "point_adapter.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
namespace {
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
uint32_t word(const std::vector<uint8_t> &b, size_t n, int count = 4) {
  uint32_t v = 0;
  for (int i = 0; i < count; ++i)
    v |= uint32_t(b.at(n + i)) << (8 * i);
  return v;
}
float number(const std::vector<uint8_t> &b, size_t n) {
  auto w = word(b, n);
  float v;
  std::memcpy(&v, &w, 4);
  return v;
}
void put(std::vector<uint8_t> &b, size_t n, uint32_t v, int count = 4) {
  for (int i = 0; i < count; ++i)
    b.at(n + i) = (v >> (8 * i)) & 255;
}
} // namespace
sensor_msgs::msg::PointCloud2
ousterCloud(const sensor_msgs::msg::PointCloud2 &s) {
  check(!s.is_bigendian && s.point_step == 24 &&
            uint64_t(s.row_step) == uint64_t(s.width) * 24 && s.height > 0 &&
            s.width > 0,
        "expected little-endian xyzirt_v1 cloud");
  check(uint64_t(s.height) * s.width <= 4194304 &&
            s.data.size() == uint64_t(s.row_step) * s.height,
        "invalid/budgeted cloud length");
  const char *names[] = {"x", "y", "z", "intensity", "ring", "time"};
  uint32_t offsets[] = {0, 4, 8, 12, 16, 20};
  check(s.fields.size() == 6, "unexpected source fields");
  for (int i = 0; i < 6; ++i) {
    auto f = std::find_if(s.fields.begin(), s.fields.end(),
                          [&](const auto &v) { return v.name == names[i]; });
    check(f != s.fields.end() && f->offset == offsets[i] &&
              f->datatype == (i == 4 ? 4 : 7) && f->count == 1,
          "wrong source field type/offset");
  }
  sensor_msgs::msg::PointCloud2 out;
  out.header = s.header;
  out.height = s.height;
  out.width = s.width;
  out.point_step = 48;
  out.row_step = s.width * 48;
  out.is_dense = true;
  out.is_bigendian = false;
  out.data.resize(size_t(s.height) * out.row_step, 0);
  const char *target[] = {"x",         "y",       "z",
                          "intensity", "t",       "reflectivity",
                          "ring",      "ambient", "range"};
  uint32_t target_offsets[] = {0, 4, 8, 16, 20, 24, 26, 28, 32};
  uint8_t types[] = {7, 7, 7, 7, 6, 4, 4, 4, 6};
  for (int i = 0; i < 9; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = target[i];
    f.offset = target_offsets[i];
    f.datatype = types[i];
    f.count = 1;
    out.fields.push_back(f);
  }
  for (size_t i = 0; i < size_t(s.width) * s.height; ++i) {
    size_t from = i * 24, to = i * 48;
    float x = number(s.data, from), y = number(s.data, from + 4),
          z = number(s.data, from + 8), intensity = number(s.data, from + 12),
          seconds = number(s.data, from + 20);
    check(std::isfinite(seconds) && seconds >= 0 &&
              double(seconds) * 1e9 <= UINT32_MAX,
          "point offset exceeds uint32 nanoseconds");
    check(std::isfinite(intensity) && intensity >= 0 && intensity <= 1,
          "expected synthetic normalized intensity");
    const bool finite =
        std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    out.is_dense &= finite;
    std::copy_n(s.data.begin() + from, 12, out.data.begin() + to);
    put(out.data, to + 12, 0x3f800000);
    put(out.data, to + 16, word(s.data, from + 12));
    put(out.data, to + 20, uint32_t(std::llround(double(seconds) * 1e9)));
    put(out.data, to + 24,
        finite ? uint32_t(std::lround(double(intensity) * 65535)) : 0, 2);
    put(out.data, to + 26, word(s.data, from + 16, 2), 2);
    double mm =
        finite ? std::sqrt(double(x) * x + double(y) * y + double(z) * z) * 1000
               : 0;
    check(mm <= UINT32_MAX, "range exceeds uint32 millimeters");
    put(out.data, to + 32, uint32_t(std::llround(mm)));
  }
  return out;
}
