#include "cloud_tool_test_boundary.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

// Only I/O/lifecycle types are replaced. The actual production constructors,
// callback registration, validation, conversion and real PCL algorithm execute.
#define LITESIM_CLOUD_TOOL_BOUNDARY cloud_tool_test::Boundary
#define main cloud_probe_unit_entry
#include "../src/sim_cloud_probe.cpp"
#undef main
#define main ouster_adapter_unit_entry
#include "../src/sim_ouster_adapter.cpp"
#undef main
#undef LITESIM_CLOUD_TOOL_BOUNDARY

namespace {
using cloud_tool_test::Boundary;
using cloud_tool_test::Cloud;
using cloud_tool_test::Scope;
using Json = nlohmann::json;

void word(Cloud &cloud, size_t point, size_t offset, uint32_t bits, size_t bytes = 4) {
  for (size_t i = 0; i < bytes; ++i)
    cloud.data.at(point * cloud.point_step + offset + i) = uint8_t(bits >> (8 * i));
}
void scalar(Cloud &cloud, size_t point, size_t offset, float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  word(cloud, point, offset, bits);
}
Cloud source(size_t count = 1) {
  Cloud cloud;
  cloud.header.frame_id = "laser/unit";
  cloud.header.stamp.sec = 5;
  cloud.header.stamp.nanosec = 7;
  cloud.width = uint32_t(count); cloud.height = 1;
  cloud.point_step = 24; cloud.row_step = cloud.width * 24;
  cloud.is_dense = true;
  const std::array<const char *, 6> names = {"x", "y", "z", "intensity", "ring", "time"};
  for (size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i]; field.offset = uint32_t(i * 4);
    field.datatype = i == 4 ? 4 : 7; field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.data.resize(cloud.row_step);
  for (size_t i = 0; i < count; ++i) {
    scalar(cloud, i, 0, 1); scalar(cloud, i, 4, 0); scalar(cloud, i, 8, 0);
    scalar(cloud, i, 12, .5f); word(cloud, i, 16, uint32_t(i + 7), 2);
    scalar(cloud, i, 20, float(i + 1) * .001f);
  }
  return cloud;
}
Json status(const Scope &scope, const std::string &topic) {
  return Json::parse(scope.state->texts.at(topic).back().data);
}
} // namespace

TEST(CloudProbeUnit, FrozenAbiAndRealVoxelGridCentroidsRetainMetadata) {
  EXPECT_EQ(sizeof(ObservedPoint), 48u);
  EXPECT_EQ(alignof(ObservedPoint), 16u);
  EXPECT_EQ(offsetof(ObservedPoint, intensity), 16u);
  EXPECT_EQ(offsetof(ObservedPoint, t), 20u);
  EXPECT_EQ(offsetof(ObservedPoint, reflectivity), 24u);
  EXPECT_EQ(offsetof(ObservedPoint, ring), 26u);
  EXPECT_EQ(offsetof(ObservedPoint, ambient), 28u);
  EXPECT_EQ(offsetof(ObservedPoint, range), 32u);
  Scope scope;
  Probe<Boundary> probe;
  auto raw = source(6);
  for (size_t i = 0; i < 3; ++i) {
    scalar(raw, i, 0, i == 2 ? .21f : .01f + float(i) * .02f);
    scalar(raw, i, 4, .02f + (i == 1 ? .02f : 0));
    scalar(raw, i, 8, .03f + (i == 1 ? .02f : 0));
  }
  for (size_t i = 3; i < 6; ++i) scalar(raw, i, (i - 3) * 4, std::numeric_limits<float>::quiet_NaN());
  auto input = ousterCloud(raw);
  std::reverse(input.fields.begin(), input.fields.end());
  scope.emit(input);
  const auto result = status(scope, "probe_status");
  EXPECT_EQ(result["algorithm"], "PCL VoxelGrid");
  EXPECT_EQ(result["version"], PCL_VERSION_PRETTY);
  EXPECT_EQ(result["input"], 6); EXPECT_EQ(result["finite"], 3);
  EXPECT_EQ(result["output"], 2); EXPECT_EQ(result["frames"], 1);
  EXPECT_EQ(result["fields_ok"], true); EXPECT_EQ(result["frame"], "laser/unit");
  EXPECT_EQ(result["max_ring"], 12); EXPECT_NEAR(result["max_t_ns"].get<double>(), 6000000, 1);
  ASSERT_EQ(result["first_centroids"].size(), 2u);
  EXPECT_NEAR(result["first_centroids"][0][0].get<double>(), .02, 1e-6);
  EXPECT_NEAR(result["first_centroids"][0][1].get<double>(), .03, 1e-6);
  EXPECT_NEAR(result["first_centroids"][0][2].get<double>(), .04, 1e-6);
  EXPECT_NEAR(result["first_centroids"][1][0].get<double>(), .21, 1e-6);
}

TEST(CloudProbeUnit, EveryLayoutAndFieldConstraintProducesAnErrorWithoutCountingAFrame) {
  Scope scope;
  Probe<Boundary> probe;
  const auto valid = ousterCloud(source());
  for (int defect = 0; defect < 8; ++defect) {
    auto cloud = valid;
    if (defect == 0) cloud.is_bigendian = true;
    if (defect == 1) cloud.point_step = 47;
    if (defect == 2) ++cloud.row_step;
    if (defect == 3) { cloud.width = 0; cloud.row_step = 0; }
    if (defect == 4) cloud.height = 0;
    if (defect == 5) { cloud.width = 4194305; cloud.row_step = cloud.width * 48; }
    if (defect == 6) cloud.data.pop_back();
    if (defect == 7) cloud.fields.pop_back();
    scope.emit(cloud);
    const auto result = status(scope, "probe_status");
    EXPECT_EQ(result["fields_ok"], false);
    EXPECT_EQ(result["error"], "invalid original cloud layout/size");
    EXPECT_FALSE(result.contains("frames"));
  }
  for (size_t i = 0; i < valid.fields.size(); ++i) for (int defect = 0; defect < 4; ++defect) {
    auto cloud = valid;
    if (defect == 0) cloud.fields[i].name = "missing";
    if (defect == 1) ++cloud.fields[i].offset;
    if (defect == 2) cloud.fields[i].datatype = 1;
    if (defect == 3) cloud.fields[i].count = 2;
    scope.emit(cloud);
    EXPECT_EQ(status(scope, "probe_status")["error"], "invalid original field");
  }
  scope.emit(valid);
  EXPECT_EQ(status(scope, "probe_status")["frames"], 1);
}

TEST(CloudProbeUnit, SyntheticFieldChecksAndCentroidLimitAreObservable) {
  Scope scope;
  Probe<Boundary> probe;
  const auto valid = ousterCloud(source());
  for (int defect = 0; defect < 3; ++defect) {
    auto cloud = valid;
    if (defect == 0) word(cloud, 0, 32, 1001);
    if (defect == 1) word(cloud, 0, 28, 1, 2);
    if (defect == 2) word(cloud, 0, 24, 0, 2);
    scope.emit(cloud);
    const auto result = status(scope, "probe_status");
    EXPECT_FALSE(result.contains("error")); EXPECT_EQ(result["fields_ok"], false);
  }
  auto raw = source(10);
  for (size_t i = 0; i < 10; ++i) scalar(raw, i, 0, float(i) * .2f);
  scope.emit(ousterCloud(raw));
  EXPECT_EQ(status(scope, "probe_status")["output"], 10);
  EXPECT_EQ(status(scope, "probe_status")["first_centroids"].size(), 8u);
  scalar(raw, 0, 0, std::numeric_limits<float>::quiet_NaN());
  raw.width = 1; raw.row_step = 24; raw.data.resize(24);
  scope.emit(ousterCloud(raw));
  EXPECT_EQ(status(scope, "probe_status")["finite"], 0);
  EXPECT_EQ(status(scope, "probe_status")["output"], 0);
  EXPECT_TRUE(status(scope, "probe_status")["first_centroids"].empty());
}

TEST(CloudProbeUnit, QosAndPublisherFailurePreserveTheDocumentedStatusStream) {
  for (bool reliable : {false, true}) {
    Scope scope;
    scope.state->overrides["sensor_reliable"] = reliable;
    Probe<Boundary> probe;
    EXPECT_EQ(scope.state->publishers.at("probe_status").durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    EXPECT_EQ(scope.state->publishers.at("probe_status").reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(scope.state->publishers.at("probe_status").depth, 10u);
    EXPECT_EQ(scope.state->subscriptions.at("points_ouster").reliability,
              reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    scope.state->fail_publish["probe_status"] = 1;
    scope.emit(ousterCloud(source()));
    EXPECT_EQ(status(scope, "probe_status")["error"], "injected publisher failure");
    scope.emit(ousterCloud(source()));
    EXPECT_EQ(status(scope, "probe_status")["frames"], 2);
  }
}

TEST(OusterAdapterUnit, WiringQosEnvironmentAndPeriodicStatusKeepLivePeerCounts) {
  for (bool reliable : {false, true}) {
    Scope scope;
    scope.state->overrides["sensor_reliable"] = reliable;
    scope.state->overrides["input_topic"] = "unit/raw";
    scope.state->overrides["output_topic"] = "unit/ouster";
    if (reliable) scope.state->run_id = "unit-run-42";
    Adapter<Boundary> adapter;
    EXPECT_EQ(scope.state->environment_names, std::vector<std::string>{"LWS_RUN_ID"});
    EXPECT_EQ(scope.state->timer_period, std::chrono::milliseconds(200));
    EXPECT_EQ(scope.state->publishers.at("adapter_status").depth, 10u);
    EXPECT_EQ(scope.state->subscriptions.at("unit/raw").reliability,
              reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    EXPECT_EQ(scope.state->publishers.at("unit/ouster").reliability,
              scope.state->subscriptions.at("unit/raw").reliability);
    scope.state->input_publishers = 3; scope.state->output_subscribers = 5;
    scope.state->timer->callback();
    const auto result = status(scope, "adapter_status");
    EXPECT_EQ(result["accepted"], 0); EXPECT_EQ(result["rejected"], 0); EXPECT_EQ(result["error"], "");
    EXPECT_EQ(result["input_publishers"], 3); EXPECT_EQ(result["output_subscribers"], 5);
    EXPECT_EQ(result["run_id"], reliable ? "unit-run-42" : "");
  }
}

TEST(OusterAdapterUnit, ActualConversionCountersErrorsAndRecoveryShareOneCallback) {
  Scope scope;
  Adapter<Boundary> adapter;
  const auto input = source(2);
  const auto expected = ousterCloud(input);
  scope.emit(input);
  ASSERT_EQ(scope.state->clouds.at("points_ouster").size(), 1u);
  const auto &converted = scope.state->clouds.at("points_ouster").back();
  EXPECT_EQ(converted.header, expected.header); EXPECT_EQ(converted.fields, expected.fields);
  EXPECT_EQ(converted.data, expected.data); EXPECT_EQ(converted.point_step, 48u);
  EXPECT_EQ(status(scope, "adapter_status")["accepted"], 1);
  auto bad = input; bad.is_bigendian = true;
  scope.emit(bad);
  auto result = status(scope, "adapter_status");
  EXPECT_EQ(result["accepted"], 1); EXPECT_EQ(result["rejected"], 1);
  EXPECT_NE(result["error"].get<std::string>().find("little-endian xyzirt_v1"), std::string::npos);
  EXPECT_EQ(scope.state->clouds.at("points_ouster").size(), 1u);
  scope.emit(input);
  result = status(scope, "adapter_status");
  EXPECT_EQ(result["accepted"], 2); EXPECT_EQ(result["rejected"], 1); EXPECT_EQ(result["error"], "");
  scope.state->input_publishers = 7; scope.state->output_subscribers = 11;
  scope.state->timer->callback();
  EXPECT_EQ(status(scope, "adapter_status")["input_publishers"], 7);
  EXPECT_EQ(status(scope, "adapter_status")["output_subscribers"], 11);
}

TEST(OusterAdapterUnit, FailedPublishIsRejectedAndTheNextCloudClearsItsError) {
  Scope scope;
  Adapter<Boundary> adapter;
  scope.state->fail_publish["points_ouster"] = 1;
  scope.emit(source());
  auto result = status(scope, "adapter_status");
  EXPECT_EQ(result["accepted"], 0); EXPECT_EQ(result["rejected"], 1);
  EXPECT_EQ(result["error"], "injected publisher failure");
  scope.emit(source());
  result = status(scope, "adapter_status");
  EXPECT_EQ(result["accepted"], 1); EXPECT_EQ(result["rejected"], 1); EXPECT_EQ(result["error"], "");
}

TEST(CloudToolEntryUnit, RealEntrypointBodiesForwardArgumentsAndLifecycleWithoutSpinningAPeer) {
  for (auto entry : {cloud_probe_unit_entry, ouster_adapter_unit_entry}) {
    Scope scope;
    char name[] = "cloud-tool", option[] = "--ros-args";
    char *argv[] = {name, option, nullptr};
    EXPECT_EQ(entry(2, argv), 0);
    EXPECT_EQ(scope.state->argv, (std::vector<std::string>{"cloud-tool", "--ros-args"}));
    EXPECT_EQ(scope.state->events.front(), "initialize");
    ASSERT_GE(scope.state->events.size(), 4u);
    EXPECT_EQ(scope.state->events[scope.state->events.size() - 2], "spin");
    EXPECT_EQ(scope.state->events.back(), "shutdown");
  }
}
