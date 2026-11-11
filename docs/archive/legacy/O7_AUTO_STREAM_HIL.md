# RM65 200 Hz + O7 自动 10 Hz 目标生成 HIL

## 阶段收尾：当前真机证据

保留 [最新 summary](../concurrency_logs/o7_auto_10hz_20261103_172402/summary.json) 和 [复核证据](RETAINED_HIL_EVIDENCE.json)。2026-11-03 17:24:36～17:24:51 +08:00，实际参数 timeout_sec=15.0（源码默认60，上限60，本轮没有修改）。目标150个，生成10.001457757Hz，SDK写61次、4.031874359Hz，覆盖88个、停止丢弃pending 1个。平均250.165413ms，最大436.590997ms，61次均返回1；操作者观察到运动，日志仍为 NOT AUTOMATICALLY VERIFIED。注入窗口CANFD3064/3064成功，>10ms=0，最大5.155292ms，feedback age最大4.344360ms，锁存/异常stop=0。BASELINE中1791个waiting_for_enable，只有209个发送，不能称为10秒ACTIVE基线。

停止原因为 timeout_limit，FINAL晚于STOP约0.322秒（在途调用自然返回）。主动退出ACTIVE、故障等取消路径有 mock 覆盖，不能据此宣称均已真机验证。无限时长、所有状态和异常恢复未验证。本轮仅整理，无真机执行。完整交接见 [PICO_O7_TELEOP_HANDOFF.md](PICO_O7_TELEOP_HANDOFF.md)。


## 范围和架构

本模式为一次性、纯写入 HIL，不是正式灵巧手控制，不接收 PICO 手指输入。原 1 Hz Trigger HIL 和单次速度 HIL 保留。

- 复用现有 `RmApi2MotionAdapter.arm` 和唯一 handle；没有第二个 RoboticArm、TCP Client 或连接。
- 原 Remote IK、CANFD 200 Hz、UDP realtime、主线安全检查、FAULT_LATCHED、stop 和 API2 连接/销毁方法不变。
- 原 HIL owner 统一互斥及一次性接受机会。自动模式内有一个目标生成线程和一个串行 Modbus writer；生成线程不调用 SDK，writer 不并行/重入。
- 小锁只保护 owner 状态和单值 mailbox；不在 SDK 调用或日志发布期间持 mailbox 锁，没有阻塞 CANFD 的全局 SDK 锁。

```text
原主线：Remote IK → CANFD 200 Hz → 原唯一 API2 Session
独立生成线程：monotonic 三角波 → 10 Hz 最新目标 → 单值 mailbox
独立串行 writer：取最新值 → 同一 adapter.write_o7_hil_positions → 同一 API2 Session
```

不读取 O7 寄存器，不重新配置 RS485 或速度。Tool port=1、slave=39，写 HR0～6、num=7，沿用 uint16 大端编码。

## 授权、启动与一次性约束

| 启动参数 | 默认值 | 条件 |
|---|---:|---|
| enable_o7_auto_stream_hil | false | 必须显式 true |
| o7_auto_stream_hz | 10.0 | 本模式只接受 10.0 |
| o7_auto_stream_timeout_sec | 60.0 | 0.1～60 秒，绝不超过 60 秒 |
| o7_auto_stream_open_pose_confirmed | false | 操作者完成现场确认后显式 true |

参数启动时读取；运行中修改 ROS 参数不会重新授权或重启本轮。

操作者必须确认 O7 已安全张开、无物体接触、食指 220～255 行程和其他六指目标 255 均安全。程序没有位置反馈验证；不能宣称其他六指在未知初始状态下不会运动。

原 allow_motion / motion_confirmation 必须满足。独立 20 Hz ROS 准入 timer 只检查状态和启动线程，不执行 SDK、等待线程或生成轨迹。默认关闭时不创建此自动准入 timer。

自动启动复用现有 HIL 准入：API2 connected、Remote IK 初始化、motion_permitted、200 Hz 配置、ACTIVE 且 command_sent、UDP fresh、无故障/控制器异常、非关闭状态；最近约 1 秒至少 180 个 ACTIVE 成功周期，跨度至少 0.99 秒，控制记录间隔及最新记录年龄不超过 10 ms。

自动模式与两种旧 HIL **共用本进程一次接受机会**。若旧 HIL 先被接受，自动模式不再启动；自动模式一旦接受，旧服务也拒绝。自动运行退出 ACTIVE 后不会因重入 ACTIVE 再启动。重新测试须安全退出、重启并重新授权。没有自动清错、恢复连接或升级频率。

## 三角波与 latest-value mailbox

joint[2] 从 255 开始，按 monotonic 经过时间：

- 0～1 秒：255 线性减到 220。
- 1～2 秒：220 线性增到 255。
- 之后重复 2 秒周期，位置取整；其他六个目标始终 255。

10 Hz 表示每 100 ms 生成一个时间对应的目标，不是每 100 ms 在端点间跳变。首次 255 样本保留到 writer 选中，随后生成线程独立运行；SDK 阻塞时继续覆盖 pending 目标，只保留一个最新值。

SDK 调用返回后取当前最新目标，跳过已被覆盖的历史值，不补发、不重发。生成线程调度太迟时也跳过过期生成 slot，直接根据当前 monotonic 时间计算三角波，避免累计漂移。

过去的 SDK 调用约 220 ms，因此实际写入可能只有约 4～5 Hz；这只是预期量级，必须用本轮数据测量。程序不能保证 SDK 实际 10 Hz，也不创建多个调用线程提高频率。

分别记录：

- `configured_target_generation_hz`：配置值 10。
- `target_generation_hz`：实际生成时间序列的 `(N-1)/(最后生成时刻-首次生成时刻)`。
- `actual_modbus_write_hz`：实际提交时间序列的相同计算方式，不是目标生成频率。
- 样本不足两个时频率为 null，不伪造 10 Hz。
- `overwritten_target_count`：mailbox 待发值被新值覆盖数量。
- `skipped_generation_slots`：生成线程调度错过的绝对 slot 数量。
- `discarded_pending_target_count` / `cancelled_selected_target_count`：停止时丢弃/选中后取消的目标数量。

若 C SDK 持有 GIL 导致 Python 生成线程也不能按期执行，实际生成频率和跳 slot 计数会反映这一点；使用两个线程不构成 SDK 并发安全保证。

## 停止、返回码与生命周期

以下情况禁止后续 O7 提交：退出 ACTIVE/command_sent=false、CANFD 记录超时或超过 10 ms 间隙、UDP stale、FAULT_LATCHED、连接/授权异常、节点关闭、SDK/worker 异常或达到 timeout。上述更严格的 CANFD 连续性检查只取消自动 O7 HIL，不改变原机械臂故障策略。

控制周期的异常会锁存取消；即使很快恢复 ACTIVE，也不继续本轮。生成和发送线程均复查安全条件；等待期间检查间隔至多约 20 ms，具体执行受系统调度/GIL影响。提交时再次检查安全条件、mailbox 关闭状态与 monotonic deadline。

STOP 先关闭 mailbox、禁止新提交，并记录准确停止原因和时间。已经提交的原生调用不能安全强杀，可自然返回；不会在返回后再下发新指令。60 秒是**新提交的硬截止时间**，不是强制取消在途调用的承诺；阻塞调用可能使 AUTO_STREAM_FINISH 和节点退出晚于 60 秒。

owner 的原关闭流程继续先禁新 HIL/设置取消、保留原 stop 策略，再等待 writer。writer 在最终退出前还会等待生成线程，因此原 handle 释放只发生在两个线程结束之后。SDK 长期阻塞时保留 handle 并按原逻辑记录警告等待，不强制销毁、不假装线程已终止。

返回 1 或其他非零码如实记为 SDK unconfirmed，不重发、不因返回码提前停止。SDK 抛出异常则立即禁止后续提交。最终仍分开报告 acknowledged / unconfirmed / exception 数量，`actual_motion=NOT AUTOMATICALLY VERIFIED`；本轮实际动作由操作者观察。

停止时不追加恢复写入，记录最后提交的完整目标。停止发送不代表 O7 已经停止运动或恢复张开；已收到目标可能继续执行。现场有风险时按既有物理安全流程处理，不能依赖 Python 线程中断原生调用。

## 复现实测参数的真机步骤（本轮整理不执行）

先启动 monitor，再启动 teleop。为保存完整的前 10 秒源数据，可在 teleop 启动后保持 PICO 失能至少 10 秒再手动启用。自动模式将在 ACTIVE 稳定约 1 秒后开始，因此这 10 秒基线可能包含 READY 等非发送周期，不能称为“10 秒健康 ACTIVE 基线”。原始记录会完整保留这些周期。

终端 B，先启动被动监测：

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
set -o pipefail
HIL_RUN_DIR="$PWD/concurrency_logs/o7_auto_10hz_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$HIL_RUN_DIR"
printf '%s\n' "$HIL_RUN_DIR" > /tmp/rm65_o7_auto_hil_run_dir
ros2 run rm_teleop single_session_hil_monitor \
  --observe-auto-stream --duration 120 --output "$HIL_RUN_DIR" \
  2>&1 | tee "$HIL_RUN_DIR/monitor_console.log"
```

`--observe-auto-stream` 不创建 Trigger client，不发送请求，与旧的 --execute-stream / --execute-injection 互斥。等待 AUTO_STREAM_FINISH 后继续采集至少 10 秒 POST，缺事件/超时则保存不完整数据。正常停止采集器才能写出 CSV，勿 kill -9。

终端 A，完成现场确认后：

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
set -o pipefail
HIL_RUN_DIR="$(cat /tmp/rm65_o7_auto_hil_run_dir)"
ros2 launch rm_teleop teleop_high_follow.launch.py \
  enable_o7_single_session_hil:=false \
  enable_o7_stream_hil:=false \
  enable_o7_auto_stream_hil:=true \
  o7_auto_stream_hz:=10.0 \
  o7_auto_stream_timeout_sec:=15.0 \
  o7_auto_stream_open_pose_confirmed:=true \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  2>&1 | tee "$HIL_RUN_DIR/teleop_console.log"
```

按原 PICO 流程手动进入 ACTIVE，不调用任何 Trigger。记录食指是否连续运动、其他关节是否保持安全，并将观察/视频与日志一起提供。不要运行会创建第二个 API2 Client 的旧工具。

## 事件、CSV 与统计

沿用 `/rm65/o7_modbus_hil/event` JSON topic：

- AUTO_STREAM_START：启动时间、配置、monotonic epoch/deadline、确认和轨迹范围。
- TARGET_GENERATED：每次实际生成的序号、monotonic 时间、目标和当前频率/覆盖计数。
- POSITION_WRITE_START / FINISH：目标序号与生成序号、目标年龄、完整向量、真实调用起止、perf_counter 耗时、返回码或异常。
- POSITION_WRITE_CANCELLED：选中后安全检查失败，尚未提交 SDK。
- AUTO_STREAM_STOP：mailbox 已关闭；此时仍可能有一个在途调用。
- AUTO_STREAM_EXCEPTION：SDK、生成线程或 worker 异常。
- AUTO_STREAM_FINISH：在途调用返回、生成线程退出后的最终结果及实际频率、覆盖/跳过数量、最后目标。

输出：

- `samples.csv`：原 CANFD 周期（含停发/故障）、UDP age、诊断 JSON 和全部 HIL 事件。
- `events.csv`：事件精确时间、位置、返回码、耗时、generation_sequence、目标年龄和频率/覆盖字段。
- `summary.json`：原主线阶段统计；stream 分项 SDK 计数；新增 auto_stream 启动/停止/结束事件、节点报告频率和 monitor 收到事件推算的频率，两者分开保存。
- `summary.csv`：保留主线各阶段统计，并增加 `auto_stream_total_*` 整轮指标。重复在各 phase 行中的整轮指标不能相加。

INJECTION 按真实 AUTO_STREAM_START 至 AUTO_STREAM_FINISH 划分，包含停止后尚未返回的在途调用；AUTO_STREAM_STOP 时间单独保存，不把在途返回误解为停止后新增指令。BASELINE/POST 为相邻前后各 10 秒。缺 START/FINISH 或不足窗口不能声称完整测试。

比较 CANFD 5 ms 目标周期、avg/p95/p99/max、所有 >10 ms 间隔、停发原因、CANFD API 耗时、独立 ROS RX 间隔、feedback_age、robot_feedback_stale、FAULT_LATCHED 和 stop。不要将目标更新 10 Hz 当作实际 Modbus 10 Hz，也不要用一次完成或均值认定长期并发安全。

## 安全停止、打包与恢复关闭

正常按原 PICO 失能流程退出高跟随 ACTIVE，即取消本轮 O7 后续下发，不会因重入 ACTIVE 重启。必要时 Ctrl+C 终端 A 关闭节点；当前 SDK 调用可能需要等待自然返回。不要为结束线程强制删除 handle。现场风险优先采用既有物理安全措施。

保留 monitor 等待结束事件和 POST；若发生长期阻塞、未能收到最终事件，可 Ctrl+C monitor 保存已有数据，结果必须标为不完整。终端 A/B 均正常结束后打包：

```bash
cd ~/teleop_ws
HIL_RUN_DIR="$(cat /tmp/rm65_o7_auto_hil_run_dir)"
cat "$HIL_RUN_DIR/summary.json"
tar -czf "${HIL_RUN_DIR}.tar.gz" \
  -C "$(dirname "$HIL_RUN_DIR")" "$(basename "$HIL_RUN_DIR")"
```

归档包含本轮全部 HIL CSV/JSON 和两个控制台日志；原节点自己的 CANFD CSV 仍在既有 `canfd_logs` 配置目录，其路径也保留在诊断数据中，必要时一并提供。

下次恢复关闭，并显式锁定运动：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  enable_o7_auto_stream_hil:=false \
  o7_auto_stream_open_pose_confirmed:=false \
  enable_o7_stream_hil:=false \
  enable_o7_single_session_hil:=false \
  allow_motion:=false
```

不自动恢复 O7 位置、速度或清除故障；正式遥操作和后续实验不在本轮范围。

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

所有 SDK 自动测试均为 mock，未构造真实 RoboticArm、未连接机器人、未进行 HIL 注入。测试涵盖稳定准入、授权、三角波、10 Hz 生成、220 ms SDK 单写线程、mailbox 覆盖、返回 1、故障/超时取消、互斥/一次性、线程结束后释放 handle 和 CSV。

本轮离线结果：pytest 164 passed（原 132 项全部通过）；colcon build 成功；colcon test-result 为 164 tests、0 errors、0 failures、0 skipped；compileall、git diff --check 和 monitor --help 通过。核对的 13 个原主线控制、安全、stop、连接/销毁方法源码未改变。
