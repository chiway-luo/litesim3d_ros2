# Fast DDS 旧帧与虚拟机时钟诊断

日期：2026-10-19（Asia/Singapore）。基线提交：`cb2b05f`；目标版本：0.4.0。

## 当前结论

已定位这轮 ARM64 旧帧故障的触发链：Lima 与客体 Ubuntu 的两个校时服务反复改变系统时间，回拨触发 Fast DDS 2.6.12 的接收历史处理分支，备用缓存被耗尽，之后新样本被拒收。仿真时间和生产端仍正常前进，慢消费者却逐渐落后。

经用户授权，已仅在 `humble-arm64` 停用客体 `systemd-timesyncd`，保留 Lima 宿主机校时。独立 DDS 三轮、加强后的慢消费者五轮和完整 ROS 78 项均通过；约 507 秒完整回归窗口内未发现超过 100 ms 的时钟跳变。**当前 Lima/Humble 环境的旧帧发布阻塞已关闭。** Fast DDS 的上游回拨缺陷仍然存在，系统时钟再次大幅回拨属于已知限制。

宿主机时间正确不等于客体时间始终连续：Fast DDS 读取客体系统时间，Ubuntu 的外部 NTP 校正与 Lima 的宿主机校正分别作用于客体。即使 Lima 把客体纠正到准确时间，这次回拨本身也足以触发缺陷。外部 NTP 偏差的具体网络原因未确定。

## 证据链

1. 无仿真器的独立 Python 发布、订阅进程：50 Hz 发布约 3 KiB 点云，`KEEP_LAST(5)`，每 0.2 秒取一个样本。默认 Fast DDS 连续三轮最大帧龄约 1.02 秒；预期五帧缓存对应约 0.1 秒。
2. 相同最小程序改用 UDP-only 后三轮仍出现约 1.02 秒旧帧。绕过 executor 直接取样、改用 reliable 也复现。因此关闭 SHM、调整 executor 或改可靠性都不足以解决本次问题。
3. DDS 的 source/reception 时间差仅约 0.1 ms，而取样时二者都已约 1 秒前，说明主要滞留在接收缓存。
4. 仅供诊断的进程级探针抓到：发布序号 210 时系统时间回拨约 1.322 秒；DDS 序号 211 返回接收成功却未保留在历史队列，随后出现 200 次缓存分配失败。探针未替换系统库，未用于通过验收。
5. Fast DDS 同时报告 `Maximum number of allowed reserved caches reached` 与拒收新样本。Lima host agent 日志每 10 秒记录约 1–2 秒的时钟修正；客体 `systemd-timesyncd` 同时处于 active/enabled 状态。
6. 只读 15 秒时钟检查检出四次跳变，最大回拨约 2.433 秒。amd64 `local_humble` 同一检查无跳变，最大差值约 0.082 ms；其独立 DDS 对照三轮最大帧龄为 0.105–0.108 秒。

amd64 使用 Fast DDS 2.6.10 / rclpy 3.3.19，ARM64 使用 2.6.12 / 3.3.22。两侧版本和时钟环境都不同，不能把跨主机对照单独解释为版本回归或 ARM 指令集缺陷；直接的回拨、缓存分支与拒收日志才是本次归因依据。

上游 [Fast DDS #5019](https://github.com/eProsima/Fast-DDS/issues/5019) 记录了系统时钟回拨与该历史比较分支的问题。本次还通过实际运行定位到接收成功但未入队后耗尽备用缓存的路径，未仅凭相似 issue 归因。相关固定版本源码为 [DataReaderHistory.cpp](https://github.com/eProsima/Fast-DDS/blob/v2.6.12/src/cpp/fastdds/subscriber/history/DataReaderHistory.cpp)。Lima 2.2.1 的 [host 校时循环](https://github.com/lima-vm/lima/blob/v2.2.1/pkg/hostagent/timesync.go) 每 10 秒检查一次，客体通过 [Settimeofday](https://github.com/lima-vm/lima/blob/v2.2.1/pkg/guestagent/timesync/timesync_linux.go) 调整系统时间。

## 回归改动

`test_ROS07_slow_consumer_queue_and_thirty_second_pause` 保留原有跨度、32 MiB 内存增长、30 秒暂停和历史回放断言，新增每次取样的帧龄上限：五帧缓存加两个扫描周期容差，10 Hz 时为 0.7 秒。同时保存系统时钟相对单调时钟的跳变，不跳过或自动重试失败。

修复前实际运行中，旧跨度断言通过，末两次取样分别落后 0.8 和 1.7 秒，新断言正确失败。这说明以前的“全套通过”确实可能漏检逐渐积压，不能作为本问题消失的证据。

新增的只读检查器无需 ROS，不修改时钟或配置，发现跳变返回非零：

```bash
python3 scripts/checks/check_clock_stability.py --duration 15 --output /tmp/litesim-clock.json
```

短窗口通过只表示该窗口未观测到超阈值跳变，不保证之后的系统时间永不调整，也不是 `/clock` 的仿真时间正确性检查。

## 环境修复与复验

仅在已确认的 `lima-humble-arm64` 客体执行以下变更，保留 Lima 的宿主机校时。原服务状态为 active/enabled，操作后为 inactive/disabled：

```bash
sudo systemctl disable --now systemd-timesyncd.service
systemctl show systemd-timesyncd.service -p ActiveState -p UnitFileState
```

回退命令为：

```bash
sudo systemctl enable --now systemd-timesyncd.service
```

这项配置用于本轮确认存在双重校时的 Lima 客体，不是其他 ROS 主机的通用配置要求。macOS、`local_humble`、Lyrical、Fast DDS 默认传输、系统库和产品 QoS 均未改变。证据目录中的执行脚本还检查客体主机名，避免作用于其他主机。

复验使用新建的 DDS 参与者，避免复用已经耗尽缓存的旧进程；全部使用默认 Fast DDS，移除了进程级传输配置和诊断库注入。修复后的 60 秒时钟检查最大相邻偏移差为 0.071 ms；独立 DDS 三轮最大帧龄分别为 0.119676、0.117356、0.119077 秒。

| 复验 | 实际结果 |
| --- | --- |
| 默认 Fast DDS 独立程序 | 3/3 正常完成；最大帧龄 0.117–0.120 秒，修复前约 1.02 秒 |
| 加强后的慢消费者及 30 秒暂停 | 连续 5/5 通过；每轮最大滞后 0.4 秒，上限 0.7 秒 |
| ROS 桥接 pytest | 74/74 通过，0 失败、0 跳过；包含 100 次完整会话重置和第六次慢消费者测试 |
| 单机 / 多机 launch_testing | 4/4 通过，0 失败、0 跳过，进程正常退出 |
| 完整回归期间的系统时钟 | 506.75 秒内无超阈值跳变；最大相邻偏移差 0.130 ms |

重复运行用于检查间歇性故障，不计为额外的不同用例。完整回归中的慢消费者最大滞后也为 0.4 秒。修复前失败与修复后通过使用同一新增帧龄上限，没有通过放宽阈值、跳过或自动重试来消除失败。

本轮修改了环境配置、测试和诊断文档，仿真核心、ROS 生产实现、系统 DDS 库与已安装程序保持原样。已有 12 个 ARM64 Debian 包没有重新生成；本轮实际运行的是前轮独立构建的安装前缀，执行文件摘要随证据保存。前轮的 306 项全套与二进制安装结果仍按前轮源码记录，本轮新证据范围为上述 ROS 回归及故障诊断，不等于重新执行全部 306 项、长稳或真实桌面验收。

## 原始证据

仓库外 `artifacts/evidence/fastdds-old-frame-20261019/` 保存基线、独立复现脚本、进程诊断探针、修复与回退脚本、原环境日志及上游固定版本源文件。`arm64-before/fastdds-fix/` 下包含 `minimal-default`、`minimal-udp`、`direct-default`、`direct-reliable`、`cache-trace` 和 `regression-before`。

修复后原始日志、JUnit、时钟结果与安装程序摘要位于 `arm64-after/fastdds-fix/`；其中 `full-after/` 保存 78 项完整 ROS 回归及全程时钟记录，`verification.json` 汇总结果。虚拟机已停止以释放内存，停用客体 NTP 的配置保留，后续启动仍由 Lima 校时。

此前提交 `cb2b05f` 的 GitHub Actions 两个任务 `m1_m3` 与 `release_packages` 均已通过：[对应 CI](https://github.com/chiway-luo/litesim3d_ros2/actions/runs/39217495499)。该 CI 不含本报告中的新断言，不能替代修复后的回归。
