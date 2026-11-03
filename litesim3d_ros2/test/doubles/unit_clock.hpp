#pragma once
#include <litesim3d_runtime/runtime.hpp>
#include <functional>
namespace litesim3d::runtime {
struct UnitClock {
  using duration=std::chrono::steady_clock::duration;
  using time_point=std::chrono::steady_clock::time_point;
  inline static time_point value{};
  inline static duration step=std::chrono::milliseconds(10);
  inline static std::function<void()> on_now;
  static time_point now(){if(on_now)on_now();auto out=value;value+=step;return out;}
  static void reset(){value=time_point{};on_now={};step=std::chrono::milliseconds(10);}
};
}
