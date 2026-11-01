#pragma once
#include "robot_view.hpp"
#include <filesystem>
#include <type_traits>
#include <utility>

namespace litesim3d_ros {
// The ROS orchestrator owns routing and lifecycle policy, not simulation. Keep
// that boundary explicit so it can consume immutable robot data and an injected
// command/event source without constructing a physics engine or transport peer.
class WorldBackend {
public:
  using EventSink = litesim3d::World::EventSink;
  virtual ~WorldBackend() = default;
  virtual size_t size() const = 0;
  virtual const RobotView &robot(size_t index) const = 0;
  virtual const std::string &runId() const = 0;
  virtual bool command(size_t, const litesim3d::Command &) = 0;
  virtual bool jointCommand(size_t, const litesim3d::JointCommand &) = 0;
  virtual void pause(bool) = 0;
  virtual void fault(const std::string &) = 0;
  virtual void exit() = 0;
  virtual std::vector<std::vector<litesim3d::Event>> tick(const EventSink &) = 0;
  virtual litesim3d::StepResult singleStep(const std::string &, uint32_t,
      std::vector<std::vector<litesim3d::Event>> &) = 0;
  virtual litesim3d::Json manifest() const = 0;
};

template<class Engine> class WorldBackendAdapter final : public WorldBackend {
  using Robot = std::remove_cv_t<std::remove_reference_t<
      decltype(std::declval<const Engine &>().robot(size_t{}))>>;
public:
  explicit WorldBackendAdapter(std::unique_ptr<Engine> engine) : engine_(std::move(engine)) {
    for (size_t i = 0; i < engine_->size(); ++i)
      robots_.push_back(std::make_unique<RobotViewAdapter<Robot>>(engine_->robot(i)));
  }
  size_t size() const override { return engine_->size(); }
  const RobotView &robot(size_t index) const override { return *robots_.at(index); }
  const std::string &runId() const override { return engine_->run_id; }
  bool command(size_t i, const litesim3d::Command &value) override { return engine_->command(i, value); }
  bool jointCommand(size_t i, const litesim3d::JointCommand &value) override { return engine_->jointCommand(i, value); }
  void pause(bool value) override { engine_->pause(value); }
  void fault(const std::string &value) override { engine_->fault(value); }
  void exit() override { engine_->exit(); }
  std::vector<std::vector<litesim3d::Event>> tick(const EventSink &sink) override { return engine_->tick(sink); }
  litesim3d::StepResult singleStep(const std::string &id, uint32_t count,
      std::vector<std::vector<litesim3d::Event>> &events) override {
    return engine_->singleStep(id, count, events);
  }
  litesim3d::Json manifest() const override { return engine_->manifest(); }
private:
  std::unique_ptr<Engine> engine_;
  std::vector<std::unique_ptr<RobotViewAdapter<Robot>>> robots_;
};

template<class Engine = litesim3d::World>
std::unique_ptr<WorldBackend> loadWorldBackend(const std::filesystem::path &path) {
  return std::make_unique<WorldBackendAdapter<Engine>>(Engine::load(path));
}
} // namespace litesim3d_ros
