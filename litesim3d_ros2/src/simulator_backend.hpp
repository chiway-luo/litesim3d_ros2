#pragma once
#include "robot_view.hpp"

namespace litesim3d_ros {
// Physics is a dependency of the ROS bridge, not part of its unit boundary.
class SimulatorBackend : public virtual RobotView {
public:
  virtual bool command(const litesim3d::Command &) = 0;
  virtual bool jointCommand(const litesim3d::JointCommand &) = 0;
  virtual void pause(bool) = 0;
  virtual std::vector<litesim3d::Event> tick() = 0;
  virtual litesim3d::StepResult singleStep(const std::string &, uint32_t, std::vector<litesim3d::Event> &) = 0;
  virtual void setPose(const litesim3d::Pose &) = 0;
  virtual void fault(const std::string &) = 0;
  virtual void exit() = 0;
  virtual litesim3d::Json manifest(const std::string &) const = 0;
  virtual void processCamera(size_t, litesim3d::plugins::ImageFrame &) = 0;
};

template<class Engine>
class SimulatorBackendAdapter final : public SimulatorBackend, public RobotViewAdapter<Engine> {
public:
  explicit SimulatorBackendAdapter(std::shared_ptr<Engine> engine)
      : RobotViewAdapter<Engine>(*engine), engine_(std::move(engine)) {}
  bool command(const litesim3d::Command &value) override { return engine_->command(value); }
  bool jointCommand(const litesim3d::JointCommand &value) override { return engine_->jointCommand(value); }
  void pause(bool value) override { engine_->pause(value); }
  std::vector<litesim3d::Event> tick() override { return engine_->tick(); }
  litesim3d::StepResult singleStep(const std::string &id, uint32_t count, std::vector<litesim3d::Event> &events) override {
    return engine_->singleStep(id, count, events);
  }
  void setPose(const litesim3d::Pose &value) override { engine_->setPose(value); }
  void fault(const std::string &reason) override { engine_->fault(reason); }
  void exit() override { engine_->exit(); }
  litesim3d::Json manifest(const std::string &reason) const override { return engine_->manifest(reason); }
  void processCamera(size_t index, litesim3d::plugins::ImageFrame &image) override { engine_->processCamera(index, image); }
private:
  std::shared_ptr<Engine> engine_;
};
}
