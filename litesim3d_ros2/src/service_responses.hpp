#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <utility>

#include <rcl/error_handling.h>
#include <rcl/service.h>
#include <rclcpp/rclcpp.hpp>

namespace litesim3d_ros {

// Used only by services in the node's default mutually exclusive callback group.
// Humble's Service::send_response hides RCL_RET_TIMEOUT. Use the deferred-response
// API and retain the result instead: retrying the command could advance time twice.
class ServiceResponses {
  friend struct ServiceResponsesUnitAccess;
public:
  explicit ServiceResponses(rclcpp::Logger logger,
      decltype(&rcl_send_response) sender = rcl_send_response)
      : logger_(std::move(logger)), sender_(sender) {}

  void start(rclcpp::Node &node, const std::shared_ptr<ServiceResponses> &self) {
    timer_ = node.create_wall_timer(std::chrono::milliseconds(50),
        [weak = std::weak_ptr<ServiceResponses>(self)] {
          if (auto state = weak.lock())
            state->retry();
        });
    timer_->cancel();
  }

  bool accept(const char *service) {
    expire();
    if (pending_.size() < 16)
      return true;
    // Refuse before invoking the handler; never execute a command whose result
    // cannot be retained. The caller still observes a timeout, not a false success.
    RCLCPP_ERROR(logger_, "response queue full for %s; request not executed", service);
    return false;
  }

  void send(const std::shared_ptr<rclcpp::ServiceBase> &service,
            const std::shared_ptr<rmw_request_id_t> &header, std::shared_ptr<void> response) {
    Pending item{service, service->get_service_name(), *header, std::move(response),
                 Clock::now() + std::chrono::seconds(2), 0};
    if (attempt(item))
      return;
    RCLCPP_WARN(logger_, "response timeout for %s; retaining response without re-executing request",
                item.name.c_str());
    const bool was_empty = pending_.empty();
    pending_.push_back(std::move(item));
    if (was_empty)
      timer_->reset();
  }

private:
  using Clock = std::chrono::steady_clock;
  struct Pending {
    std::weak_ptr<rclcpp::ServiceBase> service;
    std::string name;
    rmw_request_id_t header;
    std::shared_ptr<void> response;
    Clock::time_point deadline;
    unsigned attempts;
  };

  bool attempt(Pending &item) {
    auto service = item.service.lock();
    if (!service)
      return true;
    ++item.attempts;
    const auto result = sender_(service->get_service_handle().get(),
                                &item.header, item.response.get());
    if (result == RCL_RET_TIMEOUT) {
      rcl_reset_error();
      return false;
    }
    if (result != RCL_RET_OK)
      rclcpp::exceptions::throw_from_rcl_error(result, "failed to send service response");
    if (item.attempts > 1)
      RCLCPP_INFO(logger_, "response recovered for %s after %u send attempts",
                  item.name.c_str(), item.attempts);
    return true;
  }

  void expire() {
    const auto now = Clock::now();
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (now < it->deadline) {
        ++it;
        continue;
      }
      RCLCPP_ERROR(logger_, "response expired for %s after %u send attempts; request was not re-executed",
                   it->name.c_str(), it->attempts);
      it = pending_.erase(it);
    }
  }

  void retry() {
    expire();
    if (!pending_.empty()) {
      auto item = std::move(pending_.front());
      pending_.pop_front();
      // One send per timer callback bounds executor work even when several
      // clients disappear. Middleware calls can themselves block up to their QoS limit.
      if (!attempt(item))
        pending_.push_back(std::move(item));
    }
    if (pending_.empty())
      timer_->cancel();
  }

  rclcpp::Logger logger_;
  decltype(&rcl_send_response) sender_;
  std::deque<Pending> pending_;
  rclcpp::TimerBase::SharedPtr timer_;
};

template <typename ServiceT, typename Callback>
typename rclcpp::Service<ServiceT>::SharedPtr createResponseService(
    rclcpp::Node &node, const std::string &name, Callback callback,
    decltype(&rcl_send_response) sender = rcl_send_response) {
  auto responses = std::make_shared<ServiceResponses>(node.get_logger(), sender);
  responses->start(node, responses);
  return node.create_service<ServiceT>(name,
      [responses, callback = std::move(callback)](
          std::shared_ptr<rclcpp::Service<ServiceT>> service,
          std::shared_ptr<rmw_request_id_t> header,
          std::shared_ptr<typename ServiceT::Request> request) {
        if (!responses->accept(service->get_service_name()))
          return;
        auto response = std::make_shared<typename ServiceT::Response>();
        callback(std::move(request), response);
        responses->send(service, header, std::move(response));
      });
}

} // namespace litesim3d_ros
