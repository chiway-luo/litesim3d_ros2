#include <gtest/gtest.h>
#include "session_protocol.hpp"
#include <limits>
using namespace litesim3d::session;

TEST(SessionProtocol, MemberOwnershipRejectsAmbiguousAndReservedNames) {
  const auto definitions = Json::array({{{"name", "consumer"}, {"argv", {"consumer", "--flag"}}, {"ready_topics", {"/clock", "/points"}}}});
  const auto members = validate_members(definitions, {"/clock", "/points"});
  EXPECT_EQ(members.topics.at("consumer").size(), 2u); EXPECT_EQ(members.owned_names.size(), 4u);
  EXPECT_THROW(validate_members(Json::object(), {}), std::runtime_error);
  auto repeated = definitions; for (int i = 0; i < 33; ++i) repeated.push_back(definitions[0]);
  EXPECT_THROW(validate_members(repeated, {}), std::runtime_error);
  for (const auto &name : {"", "litesim3d", "sim_session", "robot_state_publisher", "../unsafe", "a/b", "1bad"}) {
    auto value = definitions; value[0]["name"] = name;
    EXPECT_THROW(validate_members(value, {"/clock", "/points"}), std::runtime_error);
  }
  for (int failure = 0; failure < 7; ++failure) {
    auto value = definitions;
    if (failure == 0) value[0].erase("name");
    if (failure == 1) value[0]["argv"] = Json::array();
    if (failure == 2) value[0]["argv"] = {1};
    if (failure == 3) value[0]["argv"] = {""};
    if (failure == 4) value[0]["extra"] = 1;
    if (failure == 5) value[0]["ready_topics"] = "/clock";
    if (failure == 6) value[0]["ready_topics"] = {"/missing"};
    EXPECT_THROW(validate_members(value, {"/clock", "/points"}), std::runtime_error);
  }
  auto duplicate = definitions; duplicate.push_back(definitions[0]); EXPECT_THROW(validate_members(duplicate, {"/clock", "/points"}), std::runtime_error);
}
TEST(SessionProtocol, PoseValidationAndIdempotencyPayload) {
  geometry_msgs::msg::Pose pose; pose.position.x = 1; pose.orientation.w = 2;
  const auto values = normalized_pose(pose); EXPECT_EQ(values[0], 1); EXPECT_EQ(values[6], 1);
  EXPECT_EQ(pose_rpy(values), std::vector<double>({0, 0, 0}));
  const auto first = signature("first", false, pose);
  EXPECT_NE(first, signature("second", false, pose)); EXPECT_NE(first, signature("first", true, pose));
  pose.orientation.w = 0; EXPECT_THROW(normalized_pose(pose), std::runtime_error);
  pose.orientation.w = std::numeric_limits<double>::infinity(); EXPECT_THROW(normalized_pose(pose), std::runtime_error);
  pose.orientation.w = 1; pose.position.z = std::numeric_limits<double>::quiet_NaN(); EXPECT_THROW(normalized_pose(pose), std::runtime_error);
  pose.position.z = 0; pose.orientation.x = pose.orientation.y = pose.orientation.z = pose.orientation.w = 1e308;
  EXPECT_THROW(normalized_pose(pose), std::runtime_error);
}
TEST(SessionProtocol, CandidateResourcesAreRelocatedWithoutMutatingSource) {
  TemporaryDirectory directory; write(directory.path() / "robot.urdf", "<robot/>"); write(directory.path() / "pattern.csv", "pattern");
  write(directory.path() / "mesh.obj", "mesh");
  Json data = {{"robot", {{"urdf", "robot.urdf"}}}, {"lidars", {{{"pattern_file", "pattern.csv"}}}}};
  data["entities"] = Json::array({{{"geometry", {{"type", "mesh"}, {"uri", "mesh.obj"}}}, {"visual", {{"type", "box"}}}}});
  auto before = data; const auto result = resolve_resources(data, directory.path()); EXPECT_EQ(data, before);
  EXPECT_EQ(result["robot"]["urdf"], (directory.path() / "robot.urdf").string());
  EXPECT_EQ(result["lidars"][0]["pattern_file"], (directory.path() / "pattern.csv").string());
  EXPECT_EQ(result["entities"][0]["geometry"]["uri"], (directory.path() / "mesh.obj").string());
}
TEST(SessionProtocol, ResetRetriesAreIdempotentAndPreflightIsTransactional) {
  ResetCoordinator state; state.run_id = "old"; state.transitioning = false;
  using Reset = litesim3d_interfaces::srv::Reset;
  Reset::Request request; request.request_id = "reset-1"; request.expected_run_id = "old";
  Reset::Response response; std::vector<std::string> calls;
  ResetActions actions;
  actions.check_participants = [&] { calls.push_back("participants"); };
  actions.validate_pose = [&](const auto &) { calls.push_back("preflight"); return std::vector<double>{1, 2, 3, 0, 0, 0, 1}; };
  actions.remember_topics = [&] { calls.push_back("remember"); };
  actions.restart = [&](const auto &pose) { calls.push_back("restart"); EXPECT_TRUE(pose.empty()); state.run_id = "new"; state.transitioning = false; };
  actions.record = [&](const auto &event, const auto &) { calls.push_back(event); };
  state.reset(request, response, actions); EXPECT_TRUE(response.success); EXPECT_EQ(response.run_id, "new");
  EXPECT_EQ(calls, std::vector<std::string>({"participants", "remember", "reset_begin", "restart"}));
  const auto previous = calls; state.reset(request, response, actions); EXPECT_TRUE(response.success); EXPECT_EQ(calls, previous);
  request.initial_pose.position.x = 1; state.reset(request, response, actions);
  EXPECT_FALSE(response.success); EXPECT_NE(response.message.find("different payload"), std::string::npos);
  request.request_id = "reset-2"; state.reset(request, response, actions); EXPECT_FALSE(response.success); EXPECT_NE(response.message.find("stale"), std::string::npos);
  request.expected_run_id = "new"; request.set_initial_pose = true;
  actions.validate_pose = [&](const auto &) -> std::vector<double> { throw std::runtime_error("preflight rejected"); };
  state.reset(request, response, actions); EXPECT_FALSE(response.success); EXPECT_FALSE(state.failed); EXPECT_FALSE(state.transitioning); EXPECT_EQ(state.run_id, "new");
  actions.validate_pose = [](const auto &) { return std::vector<double>{1, 2, 3, 0, 0, 0, 1}; };
  actions.restart = [](const auto &) { throw std::runtime_error("restart failed"); };
  state.reset(request, response, actions); EXPECT_FALSE(response.success); EXPECT_TRUE(state.failed); EXPECT_TRUE(state.transitioning); EXPECT_EQ(calls.back(), "reset_failed");
  state.reset(request, response, actions); EXPECT_EQ(response.message, "session not ready");
}
TEST(SessionProtocol, ResetValidatesRequestsAndNeverEvictsCommittedResults) {
  using Reset = litesim3d_interfaces::srv::Reset;
  ResetCoordinator state; state.run_id = "run"; state.transitioning = false;
  ResetActions actions; int restart = 0;
  actions.check_participants = [] {}; actions.remember_topics = [] {};
  actions.validate_pose = [](const auto &request) { return normalized_pose(request.initial_pose); };
  actions.restart = [&](const auto &pose) { EXPECT_EQ(pose.size(), 7u); ++restart; state.transitioning = false; };
  actions.record = [](const auto &, const auto &) {};
  Reset::Request request; request.expected_run_id = "run"; Reset::Response response;
  state.reset(request, response, actions); EXPECT_FALSE(response.success);
  request.request_id = std::string(129, 'x'); state.reset(request, response, actions); EXPECT_FALSE(response.success);
  request.set_initial_pose = true; request.initial_pose.orientation.w = 1;
  for (int i = 0; i < 4096; ++i) { request.request_id = std::to_string(i); state.reset(request, response, actions); ASSERT_TRUE(response.success); }
  request.request_id = "full"; state.reset(request, response, actions); EXPECT_FALSE(response.success); EXPECT_EQ(restart, 4096);
  request.request_id = "0"; state.stopping = true; state.reset(request, response, actions); EXPECT_TRUE(response.success); EXPECT_EQ(restart, 4096);
  request.request_id = "new"; state.reset(request, response, actions); EXPECT_EQ(response.message, "session not ready");
}
