# RM65 200 Hz 高跟随 + O7 固定目标纯写入 HIL（1 Hz）

## 阶段收尾：保留的 1 Hz 真机证据

当前固定目标A=[255,255,220,255,255,255,255]，B=[255,255,255,255,255,255,255]。保留 [20261103_165032 summary](../concurrency_logs/o7_write_only_1hz_20261103_165032/summary.json)，事件实际开始16:50:59.911517、结束16:51:09.912172（+08:00），10次写均返回1，平均220.216325ms、最大288.217957ms。注入期间CANFD2000/2000成功、最大间隔5.219386ms，feedback age最大5.334555ms，停发/锁存/发送失败均0。操作者确认食指运动；原日志没有设备侧执行确认，不能把返回1转换为成功。

245↔255旧轮和初始读取失败旧轮已外部归档，当前模式不读取O7、不依赖读取准入、不按三次非零停发、不追加恢复。此页的1Hz模式与另一个自动目标10Hz/实际写约4Hz模式独立。真机窗口与mock覆盖的区别见 [交接文档](PICO_O7_TELEOP_HANDOFF.md)。


独立的自动三角波 HIL 见 [O7_AUTO_STREAM_HIL.md](O7_AUTO_STREAM_HIL.md)。本文的 1 Hz 手动 Trigger 模式保留，三种 HIL 共用一次接受机会。


## 本次测试的含义

本轮仅验证通过已有唯一 API2 handle 发出的固定 O7 目标是否在现场产生可见动作，并采集对 RM65 主线的影响。

Stream HIL **不读取任何 O7 寄存器**：无 IR0～6、IR14～20 或其他初始化反馈读取，也不设置 RS485 模式或速度。已移除旧的初始实际位置读取、相对位置计算、三次非零返回停发和结束恢复写入。原速度单次 HIL 保留，与 Stream HIL 仍共用本进程一次接受机会。

单一 RoboticArm、API2 handle、TCP Client 以及连接/销毁机制保持原样；原 200 Hz CANFD、Remote IK、UDP realtime、所有主线安全检查、FAULT_LATCHED 和 stop 策略保持原样。阻塞式 Modbus 只在已有独立 worker 中执行，不新增全局 SDK 大锁。

## 固定目标与操作前确认

```python
POSITION_A = [255, 255, 220, 255, 255, 255, 255]
POSITION_B = [255, 255, 255, 255, 255, 255, 255]
```

通过 `rm_write_registers()` 写入 port=1、device=39、address=0、num=7，使用原 uint16 大端编码。每次同时写七个关节目标，仅 index_pitch（joint[2]）的目标在 220～255 之间交替；**其他六指的目标固定为 255，并非保持未知初始位置不变。在任意初始状态下，不能保证其他六指不会运动。**

测试前操作者必须现场确认：

- O7 已处于安全张开姿态。
- 手指没有接触物体。
- 七个固定目标不会造成其他关节意外运动风险。
- 既有 Tool RS485 配置正确，且没有第二个 API2 Client 工具运行。

完成上述现场确认后，才可显式设置 `o7_stream_open_pose_confirmed:=true`。该参数是操作者声明，不是程序对姿态的传感器验证；未确认则拒绝开始测试。原 `allow_motion` / `motion_confirmation` 授权机制仍必须满足。

## 参数和接口

| 参数 | 默认值 | 本轮限制 |
|---|---:|---|
| enable_o7_stream_hil | false | 显式 true 才允许启动 |
| o7_stream_open_pose_confirmed | false | 操作者完成上述确认后显式 true |
| o7_stream_hz | 1.0 | 仅 1.0，拒绝 5/10 Hz |
| o7_stream_duration_sec | 10.0 | 2～10 秒 |
| o7_stream_joint | 2 | 仅 2 |

旧 `o7_stream_amplitude` 已移除，本轮只有上述固定目标，不支持动态幅度。

- 启动：`/rm65/o7_stream_hil/start`，`std_srvs/srv/Trigger`。
- 事件：`/rm65/o7_modbus_hil/event`，`std_msgs/msg/String` JSON。
- 参数在节点启动时读取；改变测试配置需要安全退出、重启并重新授权。
- 服务不执行 SDK 调用，也不等待测试完成。`success=true` 仅表示请求被接受，不能解释为 O7 运动成功。
- 与原单次速度 HIL 共用一次接受机会，接受后即消耗，运行中/完成后均拒绝重复触发。不自动重试、重启或升级测试阶段。

继续使用原准入和持续检查：API2 connected、Remote IK 初始化、motion_permitted、CANFD ACTIVE 且持续发送、UDP fresh、无故障锁存/控制器异常、非关闭状态。启动前仍要求约 1 秒持续 200 Hz 发送记录；运行中任意退出 ACTIVE/停发周期会锁存取消，等待期间也检查安全状态。主线安全检查没有关闭或放宽。

## 调度和结束

1 Hz 表示每秒发送一个目标，不是每秒完成一次往返。正常调度：

```text
t=0s  A（食指220）
t=1s  B（食指255）
t=2s  A
t=3s  B
...
t=9s  B
t=10s 停止后续下发
```

按 STREAM_START 对应的 monotonic 时间计算绝对 slot；SDK 调用或调度太迟时跳过已过去的 slot，选择当前最新目标，不排 FIFO、不补发历史目标、不会并发或重入 Modbus。

非零返回码只记录，不触发提前停止、不触发重发。下一次下发是下一调度周期的目标。最多 10 秒、正常最多 10 个写调用，不无限忽略错误或延长测试。

主线断线、UDP stale、FAULT_LATCHED、其他安全状态不满足或 SDK 异常时停止后续写入。测试完成后没有恢复写入，也没有自动循环重启；最终事件明确记录最后一次实际进入写调用的 `last_target_positions` 和 `last_return_code`。若发生跳周期或提前停止，不能假定最后一个目标是 B。

停止下发不代表 O7 电机已经停止运动或回到初始姿态，已收到的目标可能继续执行。无法安全强杀一个已进入的原生 SDK 调用；10 秒限制后续调用启动，若已有调用长期阻塞，其返回和 STREAM_FINISH 可能晚于 10 秒。原有关闭流程继续保留 handle 等待 worker，不强制销毁仍在使用的资源。

## 返回码和结果解释

`POSITION_WRITE_FINISH` 表示调用返回或抛出异常，不代表 SDK 确认成功：

- `return_code=0`：SDK acknowledged success。
- `return_code!=0`：SDK unconfirmed command，保留原值，包括连续十次返回 1。
- SDK 异常：return_code=null，保留异常和耗时，并终止后续下发。

最终事件、summary.json、summary.csv 分别报告：

```text
sdk_acknowledged_success_count
sdk_unconfirmed_command_count
sdk_exception_count
actual_motion = NOT AUTOMATICALLY VERIFIED
```

SDK 异常不混入“非零返回命令”计数。`consecutive_failures` 仅用于记录，不再用于停止判断。`STREAM_FINISH.schedule_completed` 只表示调度预算结束，不表示全部 SDK 确认，更不表示实际运动已验证；Stream FINISH 不再提供容易误解的 success 字段。

monitor 在正常调度结束后即使存在未确认返回也会完成采集并正常退出；必须查看独立 SDK 计数，不能把进程退出码 0 当作运动或并发安全通过。

## 真机命令（由操作者执行）

开发期间没有执行以下真机命令。先完成上述现场确认；正常 ROS system time 下运行，不在采集时调整系统时钟，也不使用 use_sim_time。

终端 A：

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  enable_o7_single_session_hil:=false \
  enable_o7_stream_hil:=true \
  o7_stream_open_pose_confirmed:=true \
  o7_stream_hz:=1.0 \
  o7_stream_duration_sec:=10.0 \
  o7_stream_joint:=2 \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED
```

按既有 PICO 流程手动进入并保持 ACTIVE，程序不会自动开始 RM65 运动。

终端 B：

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
HIL_RUN_DIR="$PWD/concurrency_logs/o7_write_only_1hz_$(date +%Y%m%d_%H%M%S)"
ros2 run rm_teleop single_session_hil_monitor \
  --execute-stream --duration 60 --output "$HIL_RUN_DIR"
cat "$HIL_RUN_DIR/summary.json"
```

先采集连续 10 秒健康 ACTIVE 基线，再请求一次已有 Trigger 服务；节点独立复核准入。收到 STREAM_FINISH 后继续采集至少 10 秒 POST，再输出日志和 SDK 分项计数。操作者同时观察/录像食指和其他关节的实际动作，并将观察结果与日志一起提供。

若只需被动监测，省略 `--execute-stream`。也可先启动被动 monitor，在至少 10 秒基线后从另一个已 source 的终端手动调用（不要与自动触发同时使用）：

```bash
ros2 service call /rm65/o7_stream_hil/start std_srvs/srv/Trigger '{}'
```

## CSV 和事件

输出目录为命令指定的 `$HIL_RUN_DIR`：

- `samples.csv`：所有主线控制周期（包括停发/故障）、原诊断 JSON 和 HIL 事件，保留 CANFD command_interval_sec、command_duration_ms、feedback_age_sec、state、fault_latched、ROS control_rx_interval_sec 等。
- `events.csv`：事件和完整 JSON，展开 sequence、target_positions、start_timestamp、end_timestamp、duration_ms、return_code；保留兼容字段 command_sequence、modbus_return_code、modbus_duration_ms、perf_counter 起止和调用 wall-clock 起止。
- `summary.json`：原主线阶段统计，加 stream SDK 分项计数、原始返回码分布、异常、最后目标和最终事件。
- `summary.csv`：原主线阶段统计；`stream_total_sdk_*` 明确表示整轮总计，在各 phase 行重复展示，不能相加当作分阶段计数。保留 actual_motion 和 stream_last_target_positions。

事件包括 STREAM_START、POSITION_WRITE_START、POSITION_WRITE_FINISH、STREAM_STOP、STREAM_FINISH；SDK 异常另有 STREAM_EXCEPTION，START 发布后安全条件改变则记录 POSITION_WRITE_CANCELLED，不进入 SDK。START 的结果/结束字段为 null；FINISH 带真实调用起止、耗时和结果。结束事件的最后目标不包括尚未进入 SDK 就取消的命令。

所有对齐使用真实 wall-clock 纳秒；FINISH 的 start_timestamp / end_timestamp 对应实际包装调用前后，耗时用 perf_counter。START 事件在进入调用前发布，可能包含少量日志/调度间隔，不能将其发布时刻当作精确 C 调用入口。不得混减 wall-clock 与 perf_counter 时间。

按真实 STREAM_START / STREAM_FINISH 标记 BASELINE 前 10 秒、INJECTION 完整测试窗口、POST 后 10 秒。没有假定每次 Modbus 都耗时 66 ms 或 205 ms。服务拒绝、缺事件、超时或 Ctrl+C 保存已有数据，不伪造完整测试；正常退出采集器才能保证 CSV 写出。

## 人工判断和失败处理

重点核对现场实际动作、逐条返回码/耗时/目标、CANFD 5 ms 发送间隔分布和所有 >10 ms 间隔、停发原因、UDP freshness、robot_feedback_stale、FAULT_LATCHED 和 stop。ROS 接收间隔与节点发送间隔分别统计，不能混同。

非零返回不证明未执行，全部返回 0 也不替代实际动作观察。不要仅凭平均值、单个最大值或一次完成判断长期并发安全。发生主线安全故障时按既有安全流程处置，保留原始 CSV 和节点日志，不自动清错、复位、再连接或追加恢复动作。

恢复默认关闭状态：安全退出 monitor 和 teleop，下次启动显式锁定 HIL 和机械臂运动：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  enable_o7_stream_hil:=false \
  o7_stream_open_pose_confirmed:=false \
  enable_o7_single_session_hil:=false \
  allow_motion:=false
```

不自动恢复 O7 姿态或速度配置。需要恢复时由操作者按现场安全流程另行处理。本阶段不实现 5/10 Hz，也不开发完整 O7 遥操作。

## 离线验证

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/rm_teleop:$PYTHONPATH python3 -m pytest src/rm_teleop/test -q
python3 -m compileall -q src/rm_teleop/rm_teleop src/rm_teleop/launch
git diff --check
colcon build --packages-select rm_teleop --symlink-install
colcon test --packages-select rm_teleop --event-handlers console_direct+
colcon test-result --test-result-base build/rm_teleop --verbose
```

本轮 pytest 为 132 passed，保留原 90 项测试，替换与旧读取/恢复逻辑绑定的 Stream 测试。全部 SDK 测试使用 mock；没有构造真实 RoboticArm、连接机器人或注入。主线控制、安全、stop、连接/销毁相关 13 个方法的源码与本轮修改前一致。
