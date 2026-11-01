#pragma once
#include <litesim3d/sim.hpp>

namespace litesim3d_ros {
// Read-only bridge input boundary. A view never advances or owns physics.
class RobotView {
public:
  virtual ~RobotView() = default;
  virtual const litesim3d::Config &config() const = 0;
  virtual const litesim3d::Model &model() const = 0;
  virtual const litesim3d::Scene &scene() const = 0;
  virtual const litesim3d::Snapshot &snapshot() const = 0;
  virtual litesim3d::State state() const = 0;
  virtual litesim3d::Json diagnostics() const = 0;
  virtual const std::vector<litesim3d::Contact> &contacts() const = 0;
  virtual uint64_t contactCount() const = 0;
  virtual const std::string &runId() const = 0;
};

template<class Engine>
class RobotViewAdapter : public virtual RobotView {
public:
  explicit RobotViewAdapter(const Engine &engine) : engine_(engine) {}
  const litesim3d::Config &config() const override { return engine_.config(); }
  const litesim3d::Model &model() const override { return engine_.model(); }
  const litesim3d::Scene &scene() const override { return engine_.scene(); }
  const litesim3d::Snapshot &snapshot() const override { return engine_.snapshot(); }
  litesim3d::State state() const override { return engine_.state(); }
  litesim3d::Json diagnostics() const override { return engine_.diagnostics(); }
  const std::vector<litesim3d::Contact> &contacts() const override { return engine_.contacts(); }
  uint64_t contactCount() const override { return engine_.contactCount(); }
  const std::string &runId() const override { return engine_.run_id; }
private:
  const Engine &engine_;
};
}
