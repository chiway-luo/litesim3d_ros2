#include "../src/point_adapter.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
using Cloud = sensor_msgs::msg::PointCloud2;
void put(Cloud &cloud, size_t point, size_t offset, uint32_t bits, unsigned bytes = 4) {
  for (unsigned i = 0; i < bytes; ++i)
    cloud.data.at(point * cloud.point_step + offset + i) = uint8_t(bits >> (8 * i));
}
void scalar(Cloud &cloud, size_t point, size_t offset, float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  put(cloud, point, offset, bits);
}
uint32_t word(const Cloud &cloud, size_t point, size_t offset, unsigned bytes = 4) {
  uint32_t result = 0;
  for (unsigned i = 0; i < bytes; ++i)
    result |= uint32_t(cloud.data.at(point * cloud.point_step + offset + i)) << (8 * i);
  return result;
}
Cloud source(unsigned width = 1, unsigned height = 1) {
  Cloud cloud;
  cloud.header.frame_id = "lidar_optical";
  cloud.header.stamp.sec = 12;
  cloud.header.stamp.nanosec = 345;
  cloud.width = width;
  cloud.height = height;
  cloud.point_step = 24;
  cloud.row_step = width * 24;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  const std::array<const char *, 6> names{{"x", "y", "z", "intensity", "ring", "time"}};
  for (unsigned i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i]; field.offset = i * 4;
    field.datatype = i == 4 ? sensor_msgs::msg::PointField::UINT16 : sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1; cloud.fields.push_back(field);
  }
  cloud.data.resize(size_t(cloud.row_step) * height);
  for (size_t i = 0; i < size_t(width) * height; ++i) {
    scalar(cloud, i, 0, 3); scalar(cloud, i, 4, 4);
    scalar(cloud, i, 8, 0); scalar(cloud, i, 12, .5f);
    put(cloud, i, 16, 65535, 2); scalar(cloud, i, 20, .125f);
  }
  return cloud;
}
}

TEST(PointAdapter, FrozenLayoutPreservesOrganizedShapeHeaderAndBitPatterns) {
  auto input = source(2, 2);
  // Source field order is not an ABI requirement; names/types/offsets are.
  std::reverse(input.fields.begin(), input.fields.end());
  const auto output = ousterCloud(input);
  EXPECT_EQ(output.header, input.header);
  EXPECT_EQ(output.width, 2U); EXPECT_EQ(output.height, 2U);
  EXPECT_EQ(output.point_step, 48U); EXPECT_EQ(output.row_step, 96U);
  EXPECT_FALSE(output.is_bigendian); EXPECT_TRUE(output.is_dense);
  ASSERT_EQ(output.data.size(), 192U);
  const std::array<const char *, 9> names{{"x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range"}};
  const std::array<unsigned, 9> offsets{{0, 4, 8, 16, 20, 24, 26, 28, 32}};
  const std::array<unsigned, 9> types{{7, 7, 7, 7, 6, 4, 4, 4, 6}};
  ASSERT_EQ(output.fields.size(), names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    EXPECT_EQ(output.fields[i].name, names[i]);
    EXPECT_EQ(output.fields[i].offset, offsets[i]);
    EXPECT_EQ(output.fields[i].datatype, types[i]);
    EXPECT_EQ(output.fields[i].count, 1U);
  }
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_TRUE(std::equal(input.data.begin() + i * 24, input.data.begin() + i * 24 + 12,
                           output.data.begin() + i * 48));
    EXPECT_EQ(word(output, i, 12), 0x3f800000U);  // homogeneous coordinate
    EXPECT_EQ(word(output, i, 16), word(input, i, 12));
    EXPECT_EQ(word(output, i, 20), 125000000U);
    EXPECT_EQ(word(output, i, 24, 2), 32768U);
    EXPECT_EQ(word(output, i, 26, 2), 65535U);
    EXPECT_EQ(word(output, i, 28, 2), 0U);  // unknown ambient is not synthesized
    EXPECT_EQ(word(output, i, 32), 5000U);
    for (size_t offset : {30U, 31U, 36U, 37U, 38U, 39U, 40U, 41U, 42U, 43U, 44U, 45U, 46U, 47U})
      EXPECT_EQ(output.data[i * 48 + offset], 0U);
  }
}

TEST(PointAdapter, RejectsEachLayoutConstraintBeforeAllocatingOutput) {
  for (unsigned scenario = 0; scenario < 8; ++scenario) {
    auto cloud = source();
    switch (scenario) {
    case 0: cloud.is_bigendian = true; break;
    case 1: cloud.point_step = 23; break;
    case 2: ++cloud.row_step; break;
    case 3: cloud.height = 0; break;
    case 4: cloud.width = 0; cloud.row_step = 0; break;
    case 5: cloud.width = 4194305; cloud.row_step = cloud.width * 24; break;
    case 6: cloud.data.pop_back(); break;
    case 7: cloud.data.push_back(0); break;
    }
    EXPECT_THROW(ousterCloud(cloud), std::runtime_error) << scenario;
  }
  auto cloud = source(); cloud.fields.pop_back();
  EXPECT_THROW(ousterCloud(cloud), std::runtime_error);
  cloud = source(); cloud.fields.push_back(cloud.fields.front());
  EXPECT_THROW(ousterCloud(cloud), std::runtime_error);
}

TEST(PointAdapter, RejectsMissingOrIncorrectFieldsAtEveryPosition) {
  for (size_t index = 0; index < 6; ++index) for (unsigned defect = 0; defect < 4; ++defect) {
    auto cloud = source();
    auto &field = cloud.fields[index];
    if (defect == 0) field.name = "not_a_source_field";
    if (defect == 1) ++field.offset;
    if (defect == 2) field.datatype = field.datatype == 7 ? 4 : 7;
    if (defect == 3) field.count = 2;
    EXPECT_THROW(ousterCloud(cloud), std::runtime_error) << index << ":" << defect;
  }
}

TEST(PointAdapter, RejectsInvalidTimeIntensityAndOverflowingRange) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  for (float seconds : {nan, infinity, -infinity, -.001f, 4.3f}) {
    auto cloud = source(); scalar(cloud, 0, 20, seconds);
    EXPECT_THROW(ousterCloud(cloud), std::runtime_error);
  }
  for (float intensity : {nan, infinity, -infinity, -.001f, 1.001f}) {
    auto cloud = source(); scalar(cloud, 0, 12, intensity);
    EXPECT_THROW(ousterCloud(cloud), std::runtime_error);
  }
  auto cloud = source(); scalar(cloud, 0, 0, 5000000.f);
  EXPECT_THROW(ousterCloud(cloud), std::runtime_error);
}

TEST(PointAdapter, ValidBoundaryValuesAndNonFiniteCoordinatesHaveExplicitSemantics) {
  auto cloud = source(4);
  scalar(cloud, 0, 12, 0); scalar(cloud, 0, 20, 0);
  scalar(cloud, 1, 12, 1); scalar(cloud, 1, 20, 4.294967f);
  scalar(cloud, 2, 0, -0.f); scalar(cloud, 2, 4, 0); scalar(cloud, 2, 8, 0);
  scalar(cloud, 3, 20, 1.5e-9f);
  auto output = ousterCloud(cloud);
  EXPECT_EQ(word(output, 0, 24, 2), 0U); EXPECT_EQ(word(output, 0, 20), 0U);
  EXPECT_EQ(word(output, 1, 24, 2), 65535U);
  EXPECT_GT(word(output, 1, 20), 4294966000U);
  EXPECT_EQ(word(output, 2, 0), 0x80000000U); EXPECT_EQ(word(output, 2, 32), 0U);
  EXPECT_EQ(word(output, 3, 20), 2U);
  for (unsigned axis : {0U, 4U, 8U}) for (float invalid : {
      std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
    auto nonfinite = source(2); scalar(nonfinite, 1, axis, invalid);
    const auto converted = ousterCloud(nonfinite);
    EXPECT_FALSE(converted.is_dense);
    EXPECT_EQ(word(converted, 1, axis), word(nonfinite, 1, axis));
    EXPECT_EQ(word(converted, 1, 24, 2), 0U); EXPECT_EQ(word(converted, 1, 32), 0U);
    EXPECT_EQ(word(converted, 0, 32), 5000U);
  }
}
