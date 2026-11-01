// Isolated queue policy tests. The RCL sender is injected, so no request or
// response is transmitted and timer callbacks are advanced directly.
#include "../src/service_responses.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <gtest/gtest.h>
#include <vector>

namespace litesim3d_ros {
struct ServiceResponsesUnitAccess {
  static size_t size(const ServiceResponses &responses) { return responses.pending_.size(); }
  static unsigned attempts(const ServiceResponses &responses) { return responses.pending_.front().attempts; }
  static rclcpp::TimerBase::SharedPtr timer(const ServiceResponses &responses) { return responses.timer_; }
  static void retry(ServiceResponses &responses) { responses.retry(); }
  static void expire(ServiceResponses &responses, bool alternating = false) {
    for (size_t i = 0; i < responses.pending_.size(); ++i)
      if (!alternating || i % 2 == 0)
        responses.pending_[i].deadline = ServiceResponses::Clock::now() - std::chrono::seconds(1);
    responses.expire();
  }
  static void forgetService(ServiceResponses &responses) { responses.pending_.front().service.reset(); }
};
}
namespace {
using Service = std_srvs::srv::SetBool;
using litesim3d_ros::ServiceResponses;
using Access = litesim3d_ros::ServiceResponsesUnitAccess;
std::deque<rcl_ret_t> outcomes;
std::vector<void *> sent;
std::vector<int64_t> sequences;
rcl_ret_t send(const rcl_service_t *service, rmw_request_id_t *header, void *response) {
  EXPECT_NE(service, nullptr); EXPECT_NE(response, nullptr);
  sent.push_back(response); sequences.push_back(header->sequence_number);
  rcl_ret_t result = outcomes.empty() ? RCL_RET_OK : outcomes.front();
  if (!outcomes.empty()) outcomes.pop_front();
  if (result != RCL_RET_OK) RCL_SET_ERROR_MSG("injected unit sender failure");
  return result;
}
class ResponseUnits : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    outcomes.clear(); sent.clear(); sequences.clear(); rcl_reset_error();
    node = std::make_shared<rclcpp::Node>("response_unit");
    service = node->create_service<Service>("unit_response", [](Service::Request::SharedPtr, Service::Response::SharedPtr) {});
    responses = std::make_shared<ServiceResponses>(node->get_logger(), send);
    responses->start(*node, responses);
  }
  void TearDown() override { responses.reset(); service.reset(); node.reset(); rcl_reset_error(); }
  std::shared_ptr<rmw_request_id_t> header(int64_t sequence) {
    auto result = std::make_shared<rmw_request_id_t>(); result->sequence_number = sequence; return result;
  }
  std::shared_ptr<Service::Response> response() {
    auto result = std::make_shared<Service::Response>(); result->success = true; result->message = "executed exactly once"; return result;
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::Service<Service>::SharedPtr service;
  std::shared_ptr<ServiceResponses> responses;
};
}

TEST_F(ResponseUnits, SuccessfulSendDoesNotQueueAndNonTimeoutErrorsPropagate) {
  auto value = response();
  EXPECT_TRUE(responses->accept("unit_response"));
  EXPECT_TRUE(Access::timer(*responses)->is_canceled());
  responses->send(service, header(17), value);
  EXPECT_EQ(sent, std::vector<void *>{value.get()}); EXPECT_EQ(sequences, std::vector<int64_t>{17});
  EXPECT_EQ(Access::size(*responses), 0U);
  outcomes.push_back(RCL_RET_ERROR);
  EXPECT_THROW(responses->send(service, header(18), value), rclcpp::exceptions::RCLError);
  EXPECT_EQ(Access::size(*responses), 0U);
}

TEST_F(ResponseUnits, TimeoutRetainsExactResponseAndRetriesWithoutReexecutingCommand) {
  outcomes = {RCL_RET_TIMEOUT, RCL_RET_TIMEOUT, RCL_RET_OK};
  auto value = response(); auto *address = value.get(); std::weak_ptr<Service::Response> retained = value;
  responses->send(service, header(42), value); value.reset();
  EXPECT_FALSE(rcl_error_is_set()); EXPECT_FALSE(retained.expired());
  ASSERT_EQ(Access::size(*responses), 1U); EXPECT_EQ(Access::attempts(*responses), 1U);
  EXPECT_FALSE(Access::timer(*responses)->is_canceled());
  Access::timer(*responses)->execute_callback();
  ASSERT_EQ(Access::size(*responses), 1U); EXPECT_EQ(Access::attempts(*responses), 2U);
  Access::timer(*responses)->execute_callback();
  EXPECT_EQ(Access::size(*responses), 0U); EXPECT_TRUE(retained.expired());
  EXPECT_TRUE(Access::timer(*responses)->is_canceled());
  EXPECT_EQ(sent, (std::vector<void *>{address, address, address}));
  EXPECT_EQ(sequences, (std::vector<int64_t>{42, 42, 42}));
}

TEST_F(ResponseUnits, RetryIsRoundRobinAndDoesAtMostOneSendPerTimerCallback) {
  outcomes = {RCL_RET_TIMEOUT, RCL_RET_TIMEOUT, RCL_RET_TIMEOUT, RCL_RET_OK, RCL_RET_OK};
  auto first = response(), second = response();
  responses->send(service, header(1), first); responses->send(service, header(2), second);
  ASSERT_EQ(Access::size(*responses), 2U);
  Access::retry(*responses); EXPECT_EQ(Access::size(*responses), 2U); EXPECT_EQ(sent.size(), 3U);
  Access::retry(*responses); EXPECT_EQ(Access::size(*responses), 1U); EXPECT_EQ(sent.size(), 4U);
  Access::retry(*responses); EXPECT_EQ(Access::size(*responses), 0U); EXPECT_EQ(sent.size(), 5U);
  EXPECT_EQ(sent, (std::vector<void *>{first.get(), second.get(), first.get(), second.get(), first.get()}));
  EXPECT_TRUE(Access::timer(*responses)->is_canceled());
  Access::retry(*responses); EXPECT_EQ(sent.size(), 5U);
}

TEST_F(ResponseUnits, CapacityIsBoundedAndExpiredResponsesReleaseCapacityAndOwnership) {
  std::vector<std::weak_ptr<Service::Response>> retained;
  for (unsigned i = 0; i < 16; ++i) {
    EXPECT_TRUE(responses->accept("unit_response"));
    outcomes.push_back(RCL_RET_TIMEOUT);
    auto value = response(); retained.push_back(value); responses->send(service, header(i), value);
  }
  EXPECT_FALSE(responses->accept("unit_response")); EXPECT_EQ(Access::size(*responses), 16U);
  Access::expire(*responses, true); EXPECT_EQ(Access::size(*responses), 8U);
  EXPECT_TRUE(responses->accept("unit_response"));
  for (size_t i = 0; i < retained.size(); ++i) EXPECT_EQ(retained[i].expired(), i % 2 == 0);
  Access::expire(*responses); EXPECT_EQ(Access::size(*responses), 0U);
  for (const auto &weak : retained) EXPECT_TRUE(weak.expired());
  Access::retry(*responses); EXPECT_TRUE(Access::timer(*responses)->is_canceled());
  EXPECT_EQ(sent.size(), 16U);  // expiration never retransmits or reexecutes a command
}

TEST_F(ResponseUnits, ExpiredServiceAndExpiredOwnerDoNotSendOrKeepCallbacksAlive) {
  outcomes.push_back(RCL_RET_TIMEOUT);
  responses->send(service, header(9), response());
  Access::forgetService(*responses); Access::retry(*responses);
  EXPECT_EQ(Access::size(*responses), 0U); EXPECT_EQ(sent.size(), 1U);
  auto timer = Access::timer(*responses);
  responses.reset();
  EXPECT_NO_THROW(timer->execute_callback()); EXPECT_EQ(sent.size(), 1U);
}

TEST_F(ResponseUnits, FactoryExecutesHandlerOnlyWhenResponseCanBeRetained) {
  unsigned executed = 0;
  auto factory = litesim3d_ros::createResponseService<Service>(*node, "unit_factory",
    [&](Service::Request::SharedPtr request, Service::Response::SharedPtr result) {
      ++executed; result->success = request->data;
    }, send);
  for (int i = 0; i < 20; ++i) {
    if (i < 16) outcomes.push_back(RCL_RET_TIMEOUT);
    auto request = std::make_shared<Service::Request>(); request->data = true;
    factory->handle_request(header(i), request);
  }
  EXPECT_EQ(executed, 16U); EXPECT_EQ(sent.size(), 16U);
}
