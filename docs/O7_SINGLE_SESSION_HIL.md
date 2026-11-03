# RM65 单 API2 Session + O7 单次 Modbus 并发 HIL

## Legacy 状态与证据

保留此功能用于单次通信/并发回归，不是下一阶段手指控制入口；它仍被高跟随节点、Adapter和测试引用，不删除。三种HIL共用一次接受机会。历史20261103_154058速度写入耗时205.623263ms、返回1，主线在该窗口持续发送；这是速度配置写，不是位置动作证据。原数据已按 [清理清单](WORKSPACE_CLEANUP_INVENTORY.md) 归档至工作空间外，目标路径见 [归档校验](WORKSPACE_ARCHIVE_VERIFICATION.json)。

最新两轮位置真机证据见 [交接](PICO_O7_TELEOP_HANDOFF.md)：返回1表示SDK未确认，但操作者观察到位置运动；不能解释为一定未收到，也不能转成成功。自动目标10Hz、实际写约4Hz，尚无无限时长/全部恢复路径验证。本轮整理不运行此legacy服务，不重配RS485。


新增的 1 Hz 食指运动 HIL 见 [O7_STREAM_HIL.md](O7_STREAM_HIL.md)。三种 HIL 共用一次性生命周期，任一种接受后本进程均拒绝再次注入；本页描述原单次速度测试。

## 测试目的与边界

验证现有 `RmApi2MotionAdapter` 的同一个 `RoboticArm` / API2 handle，在主控制线程执行 Remote IK + 200 Hz CANFD 时，能否与独立线程的一次 O7 Modbus 写共存。自动测试仅使用 mock，开发期间没有连接机器人或执行真机注入。

本功能不是正式 O7 控制器。没有第二个 TCP Client、第二个 handle、持续 Modbus 调度、位置写入、RS485 重新配置或额外 UDP callback。一次成功也不能证明长期并发安全；后续位置HIL已另行开发；该单次模式本身不验证持续吞吐量。

**本测试是实际设备配置写入：Tool port=1，slave=39，HR14~20=25，num=7，大端数据 `[0,25]*7`。它不写位置目标，但速度设置可能持续保留，退出不会自动恢复。** 请事先确认 O7 的既有位置目标与设备状态适合执行该配置写入。

## 源码检查与架构

- `rm_api2_adapter.py:connect()` 创建 `RoboticArm(RM_TRIPLE_MODE_E)` 并调用一次 `rm_create_robot_arm()`；对象和 handle 保存在 Adapter。原有初始化失败会 disconnect 并等待重试；连接线程成功后返回，运行中反馈失效走原有故障策略，不会自动重连。
- `on_timer()` 仍在锁外运行 FK、`_run_control_cycle()` 和发布；Remote IK、安全检查、`follow=True`、trajectory mode、命令发送和原有严重故障快速停止/锁存路径保持原实现。
- `_adapter_lock` 仍只保护 Adapter 引用、连接状态和配置；HIL 不持此锁调用 SDK。新增的小锁只保护一次性状态和准入历史，主循环不等待 Modbus。
- Service 与 timer 使用同一默认 mutually-exclusive ROS callback group；Service 验证后启动独立非 daemon Python worker，立即返回，不 join、不执行 Modbus。
- 本地 `/home/ai/RM_API2/Python/Robotic_Arm/rm_robot_interface.py` 的 `rm_write_registers()` 使用 `self.handle` 调用底层函数。所查包装源码未发现明确禁止该并发方式的声明；这不是并发安全保证。SDK 内部互斥、GIL、响应串扰和 UDP 影响仍需 HIL 数据验证。

```text
teleop process / 一个 RmApi2MotionAdapter / 一个 RoboticArm + handle
  主 ROS 控制线程 → Remote IK → rm_movej_canfd (200 Hz)
  一次性 HIL worker → rm_write_registers (仅一次，HR14~20)
                     ↓ RM65 Tool RS485 → O7
独立 monitor process → 只订阅 ROS；显式选项才调用 Trigger 服务
```

## 参数、服务和事件

参数 `enable_o7_single_session_hil: false` 默认关闭，可通过同名 launch 参数显式打开。它是启动时读取的配置，修改后需重启。`allow_motion` 和 `motion_confirmation` 原有双重授权机制不变。

- 服务：`/rm65/o7_modbus_hil/trigger_once`，`std_srvs/srv/Trigger`。
- 事件：`/rm65/o7_modbus_hil/event`，`std_msgs/msg/String`，JSON。
- `success=true` 仅说明接受了一次请求。真实结果必须看 `MODBUS_FINISH.return_code`，0 才是 SDK 报告成功；`MODBUS_EXCEPTION` 是异常。
- 接受后无论写入成功、失败、异常或线程启动失败，都消耗本进程的一次机会，拒绝再次注入，避免不确定结果后的自动重写。条件不满足而拒绝的请求不消耗机会。

准入同时检查：API2 对象和 handle 存在、connected、Remote IK 初始化、现有 `_critical_feedback_reason()` 无异常（包括反馈时效/关节使能/关节错误/控制器状态）、motion_permitted、未锁存故障、当前 ACTIVE 并成功发送、非关闭状态、无已接受的 HIL。控制频率必须是 200 Hz。最近约 1 秒每条控制记录均须 ACTIVE 且 command_sent，至少 180 条，覆盖至少 0.99 秒，记录间隔及最后记录距当前时间都不超过 10 ms。该准入仅用于 HIL，不改变主控制器的抖动/停机策略。

事件使用主机 wall-clock `timestamp_ns` 与 monitor 对齐，`time.perf_counter()` 记录 `modbus_start_timestamp`、`modbus_end_timestamp`、`modbus_duration_ms`；结果也提供 `duration_ms`、`return_code`、`modbus_return_code`。perf_counter 时间不能与 wall-clock 时间相减。START 在调用前发布，FINISH/EXCEPTION 在调用返回后发布；事件窗口包含少量发布/参数包装开销，耗时测量排除 START 发布和日志开销。没有假设固定 66 ms。所有事件同时写入节点日志。

## 构建和离线测试

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

历史初版执行：pytest 90 tests passed；colcon test-result 为 90 tests, 0 errors, 0 failures, 0 skipped；compileall 和 diff --check 通过；colcon 构建成功（1 package finished）。`ros2 run rm_teleop single_session_hil_monitor --help` 通过，`ros2 launch ... --show-args` 通过；后者首次因默认 `~/.ros/log` 只读失败，授权重新解析后通过，没有启动节点。没有实际执行以下真机命令。

## 真机操作流程（由操作者手动执行）

确认已按既有流程完成 O7 RS485 模式设置，并关闭先前会创建第二个 API2 Client 的工具；不要运行旧 `--auto-o7`。使用正常 ROS system time（不要 use_sim_time，也不要在采集时调整系统时钟）；所有节点和 monitor 在同一主机时钟下采集。

终端 A：启动原 teleop 并显式开启 HIL，保留实际运动授权 token：

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  enable_o7_single_session_hil:=true \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED
```

由操作者按原 PICO 启停流程使主线持续 ACTIVE，确认环境和姿态安全，并在完整采集窗口持续保持合法目标输入。程序不会自动启动机械臂遥操作会话。

终端 B：显式授权本次真实写入，monitor 收到连续 10 秒健康 ACTIVE/成功发送记录后请求一次 Trigger；服务还会独立执行上述准入检查。

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
HIL_RUN_DIR="$PWD/concurrency_logs/single_session_$(date +%Y%m%d_%H%M%S)"
ros2 run rm_teleop single_session_hil_monitor \
  --execute-injection --duration 60 --output "$HIL_RUN_DIR"
cat "$HIL_RUN_DIR/summary.json"
```

`--duration 60` 是等待基线/接受后等待事件的最大秒数。收到 FINISH 或 EXCEPTION 后继续采集至少 10 秒再保存退出。服务拒绝不重试；等待超时、Ctrl+C 或丢失事件时保存不完整原始数据。终端断电或 kill -9 无法保证保存，请正常退出采集程序。控制故障时安全处理优先，不必为了完整 POST 继续保持运动授权。

默认被动监测，无任何写入请求：

```bash
ros2 run rm_teleop single_session_hil_monitor \
  --duration 30 --output "$PWD/concurrency_logs/single_session_passive"
```

可选手动触发：先启动上述被动 monitor，在其运行中确保至少 10 秒基线和所有准入条件，再在另一已 source 的终端调用（不要同时使用自动模式）：

```bash
ros2 service call /rm65/o7_modbus_hil/trigger_once std_srvs/srv/Trigger '{}'
```

## CSV 输出与统计

monitor 入口位于 Python 包中，由 setup.py 安装为 `ros2 run rm_teleop single_session_hil_monitor`，不加载 API2。

- `samples.csv`：所有控制记录（含 command_sent=false）、诊断记录和事件记录；`record_type` 区分类型。含源 `timestamp_ns`、接收 `rx_timestamp_ns`、phase、外部 `control_rx_interval_sec`、state/reason、command_sent/attempted、节点 command_interval_sec、command_duration_ms、movej_canfd_return_code、feedback_age_sec、ik_duration_ms、fault_latched、stop_command_sent 等。
- 诊断记录的 `diagnostics_json` 保留 `/rm65/status` 所有 status、level、message、hardware_id 和实际 KeyValue 数据。兼容 Humble 的 bytes level。
- `events.csv`：精确 START/FINISH/EXCEPTION 及完整 JSON；自动模式另存 TRIGGER_RESPONSE（含 accepted 和原因）。
- `summary.csv` / `summary.json`：各阶段的 avg/p95/p99/max 和数量、停发原因、失败发送、fault_latched 样本数与进入次数、stop 次数、反馈失效样本/原因及 Modbus 结果。`complete_window` 表示源时间是否覆盖前后完整 10 秒，不代表数据连续完整或测试通过。

最终按真实事件源时间回填：BASELINE=`[START-10s, START)`，INJECTION=`[START, END]`，POST=`(END, END+10s]`；其余为 OUTSIDE，没有 START 则 UNTRIGGERED。缺少结束事件时不能声称完整 POST 或测试成功。控制记录按其 header 时间归类，诊断按诊断 header 时间归类，事件按事件 timestamp_ns。跨边界的控制周期/调用可覆盖两个阶段，分析时应结合相邻原始行和 duration，不能只看所属阶段。

CANFD 间隔只统计真正 `command_sent=true` 的节点报告值；首次/无效负值不参与分布。CANFD 调用耗时统计所有 attempted 调用（含失败）。停发周期仍完整保存并统计原因。外部 RX 间隔独立统计，包括停发周期；它受 ROS 排队、订阅回调、DDS 丢失和调度影响，不是 SDK 发送间隔。没有样本的阶段分布为 null/count=0，不能当作零延迟成功。故障锁存的“样本数”与“进入次数”分别报告，避免把每周期重复的故障当成多次新故障。

## 验证指标与通过条件

人工结合原始 CSV、节点本地 CANFD 日志和事件判断，不自动判定并发安全：

1. CANFD 目标周期 5 ms，BASELINE/INJECTION/POST 均持续发送。检查 avg/p95/p99/max、所有 >10 ms 次数及其对应时间、停发数量/原因。期望没有注入相关 >10 ms 异常间隔；少量系统调度抖动须结合基线和原始数据解释。
2. UDP realtime 反馈不中断，feedback_age 不超过原有 50 ms freshness；没有 robot_feedback_stale / robot_feedback_disconnected、FAULT_LATCHED、异常 stop 或 CANFD 返回失败。
3. 一次 START 对应一次 FINISH，Modbus 返回 0，有真实 perf_counter 耗时；EXCEPTION、非零返回、缺事件或缺采集窗口均不可判通过。
4. 比较三个阶段的 CANFD 调用耗时、外部 ROS RX 间隔和 feedback_age 分布。几十毫秒的注入窗口可能只有很少样本，必须核对窗口边缘，不能只用平均值或单个最大值下结论。
5. 确认完整基线和 POST、ROS 接收缺口、诊断内容与节点原始日志一致。一次成功仅说明本次条件下未观察到干扰，不是长期并发安全证明。

## 失败处理与生命周期

发现反馈中断、明显发送间隙、返回失败或锁存故障，按既有安全流程停止 PICO 遥操作，保存全部日志，由操作者处理机器人安全；不自动清错、复位、重新连接或再次注入。SDK 并发若导致 stop 本身阻塞，应使用既有物理安全手段，不能依赖软件线程强制中止原生调用。

关闭时先禁止新 HIL，摘除节点连接状态，保留原有 shutdown quick-stop；之后在关闭流程等待 HIL worker，完成后才恢复 UDP 配置/删除 handle。不会在 200 Hz 控制回调中 join。若 worker 2 秒仍未返回，明确记录 blocked 警告，保留 Adapter、handle、callback 并继续等待；Python 无法安全强杀该线程，因此退出可能长期等待。连接初始化尚未完成时，也等待其返回并由连接线程清理，不在关闭后发布新 Adapter。这是保守保留资源，不是假装超时取消。不要强制调用 delete 或另开 API2 Client 来“解堵”。

## 恢复原始配置

正常退出 monitor 和 teleop。下一次以 `enable_o7_single_session_hil:=false`（或省略此参数）启动，直接运行节点/YAML 的 allow_motion=false；launch 默认 true 且空确认会拒绝，恢复关闭须显式 allow_motion=false：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py allow_motion:=false
```

关闭 HIL 时不记录准入历史、不启动 worker，原控制路径保持。O7 速度寄存器不会自动恢复；如需恢复，先退出本测试并依照已验证的设备配置流程由操作者手动恢复已记录的原值，不能假定原值或在本测试中追加恢复写入。不要自动解除 FAULT_LATCHED，也不要未经后续授权扩展正式 O7 模块。
