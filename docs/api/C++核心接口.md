# C++ 核心接口

头文件：[litesim3d/sim.hpp](../../litesim3d_core/include/litesim3d/sim.hpp)。命名空间 `litesim3d`，使用 C++17。插件 API 另见 [插件接口](插件接口.md)。

## 链接与最小调用

安装后使用导出的 CMake target，头文件与依赖随 target 传递：

```cmake
find_package(litesim3d_core REQUIRED)
add_executable(my_sim main.cpp)
target_link_libraries(my_sim PRIVATE litesim3d_core::litesim3d_core)
```

```cpp
#include <litesim3d/sim.hpp>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  litesim3d::Scene scene;
  auto config = litesim3d::Config::load(argv[1], scene);
  auto model = litesim3d::Model::load(config.urdf, config.model_options);
  litesim3d::Simulator sim(config, model, scene);
  sim.pause(false);
  for (int i = 0; i < 20; ++i) {
    sim.command({0.2, 0.0});
    for (const auto &event : sim.tick()) {
      // 消费 event.snapshot / clouds / imus；相机需要独立渲染宿主。
      (void)event;
    }
  }
  sim.exit();
  return 0;
}
```

输入必须是已准备好的配置和 URDF；先用模型准备器展开 Xacro。插件包必须已安装且可发现。上例无 ROS 发布与渲染，使用无相机或全部相机停用的配置；生产宿主需捕获加载/运行异常。仓库有 [安装后消费示例](../../scripts/fixtures/core_consumer/main.cpp) 与对应 [CMake](../../scripts/fixtures/core_consumer/CMakeLists.txt)。

## 加载与资源

| 接口 | 输入 / 输出与限制 |
| --- | --- |
| `Config::load(file, scene, control={})` | 解析 YAML、资源及插件声明，填充 Scene；失败抛异常 |
| `Model::load(file, options={})` | 已展开 URDF，返回关节树、几何与导入报告 |
| `Model::mechanicalGeometry(base_link)` | 从已加载模型计算 base 下的中立关节位置、轴向、限位及可确定的圆柱轮径；返回 `plugins::ModelGeometry`，不接受第二份尺寸配置 |
| `World::load(file)` | 世界 YAML，返回 unique_ptr，加载 1–4 台同构机器人 |
| `resolveResource(uri, base)` | 按声明目录/包索引定位资源 |
| `LoadControl` | byte_budget、triangle_budget、cancelled；checkpoint 检查预算/取消；fail_allocation_after 用于故障注入 |

不要把失败加载后的部分输出当成可运行世界；宿主先构造候选对象，成功后再替换当前实例。预算并非通用进程 RSS 上限。

安装后的 `bin/model_geometry expanded.urdf base_link` 提供同一读取逻辑的 JSON 输出。带 `--resolve` 时从标准输入接受请求数组，每项为 `{"kind":"differential|ackermann|omni","parameters":{...关节绑定...}}`，返回 `resolved` 参数数组。启动适配器使用该工具填充控制器参数；不修改 URDF 或源 YAML。轮子缺少可确定的圆柱 collision、模型无效或旧尺寸冲突时退出码非零。

## Simulator

| 方法 | 语义 |
| --- | --- |
| `Simulator(Config, Model, Scene)` | 建立实例与插件，初态 Ready，时间 1 ns |
| `config/model/scene/snapshot()` | 只读引用，不能超出所属实例生命周期 |
| `state()` / `stepIndex()` | 当前状态与物理步数 |
| `command(const Command&) -> bool` | 校验并接收目标速度；false 表示拒绝，不刷新有效命令超时 |
| `pause(bool)` | 暂停/恢复；终止状态不能恢复 |
| `tick() -> vector<Event>` | 运行时推进一个物理步，返回到期事件；暂停不推进 |
| `singleStep(id, count, output) -> StepResult` | 暂停下执行多步；ID 幂等；返回码见消息与服务 |
| `setPose(const Pose&)` | 仅 Ready 初始化，不是运行中 reset |
| `fault(reason)` / `exit()` | 进入终止状态 |
| `manifest(reason)` / `diagnostics()` | JSON 清单与诊断 |
| `contacts()` / `contactCount()` | 有界保留接触列表 / 总计数，两者可能不同 |
| `processCamera(index, ImageFrame&)` | 已渲染图像的插件后处理，不负责渲染 |

Simulator::singleStep 将事件追加到传入 output；每次调用前使用空容器，尤其重试命中缓存时不会替调用方清空旧内容。World::singleStep 会先清空它的分组输出。

Command 包含 v/w、lateral/vertical、roll_rate/pitch_rate、可选 stamp（纳秒）与 frame。不同模型拒绝不支持的分量。Snapshot 同时提供 world_base 与 odom_base/odom_root，真实速度与测量速度分开。

Event 含采样快照、odom 标志、Cloud/ImuSample 列表、需要渲染的相机索引。Cloud 的 stamp 为扫描起点，不应改写成接收时间；事件只交付已经完成采样的帧。相机宿主须使用该事件的真实位姿完成渲染，再处理和发布。

## World

`World` 不可复制或移动，其他机器人轨迹回调依赖其地址稳定。`size()`、`robot(index)` 提供只读实例视图；`command(index, command)` 控制指定机器人；`pause/fault/exit` 作用于整个世界。

`tick(const EventSink& sink={})` 返回按机器人索引分组的事件；EventSink 签名是 `void(size_t, const vector<Event>&)`，回调数据仅在调用期间有效。`singleStep(id,count,events)` 按同一时间轴推进所有机器人，`manifest()` 汇总结果。不要另外推进 robot() 所指的单机对象。

## 状态与调用边界

公开 State 枚举为 Ready、Running、Paused、Fault、Exited；加载中和重置中是宿主/会话状态，不是这组枚举。内核不提供通用 reset()，新实验重建 Simulator/World；插件 reset 只处理自身状态。

接口未声明多线程并发调用保证，宿主应串行执行修改操作。world 与单机的单步输出预算、幂等缓存和错误码见 [消息与服务](消息与服务.md#step)。此 SDK 不启动 ROS executor、RSP、GUI 或相机服务，也不替宿主协调下游算法历史。
