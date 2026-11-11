# PICO → O7 Right：参数启动与 RM65 联动现场流程

更新：2026-11-04。手部执行由启动参数直接选择；不再需要 enable/disable 服务。本文替代前一版服务门控流程。
所有当前代码/配置/模型/求解环境已迁入 teleop_ws；旧参考目录不再参与运行。本轮离线完成追踪恢复修复，尚未做新版真机验收。

## 两条执行路径独立选择

在 ROS 工作空间准备环境，不激活手部求解用的 Python3.11 .venv：

```bash
cd /home/ai/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/home/ai/teleop_ws/log/pico_o7
```

使用增强版 launch，而不是同时启动原机械臂 launch：

```bash
# 仅手部：ARM 锁定，手部在安全连接就绪和收到新鲜帧后自动跟随。
ros2 launch pico_o7 teleop.launch.py \
  allow_motion:=false enable_o7_retargeting:=true
```

| 启动选择 | allow_motion | enable_o7_retargeting | motion_confirmation |
|---|---|---|---|
| 都不执行 | false | false | 不需要 |
| 仅手部 | false | true | 不需要 |
| 仅机械臂 | true | false | 原 ARM token |
| 臂手联动 | true | true | 原 ARM token |

ARM=true 时保留 `motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED` 和原 PICO 启用门控。
O7=true 就是本次手部执行选择，不再有服务二次确认。O7=false 不创建手部控制器/求解器/写入 Worker。
参数在启动时加载。切换执行选择需退出旧实例并重新启动，不支持用 `ros2 param set` 热切换这些缓存值。
如需运行中单独停用 O7，请按原流程停臂并退出本实例，然后以 O7=false 重启；旧手部开关服务已删除。

## 保留的配置与安全行为

- 实际设备配置：`/home/ai/teleop_ws/src/pico_o7/config/o7_device_linear.json`，write_hz=4、七轴 max_step=16、max_age=0.25s、max_call=750ms、initial_hold 全 255。本轮未改。
- 调手部参数可传 `o7_device_mapping_path:=/绝对路径/副本.json`，重启生效；本轮以 4Hz/16 为基线，不要求先优化到 10Hz。
- initial_hold 只是首笔限步命令参考，不是位置读回或启动写入。现场先确认已知保持姿态与参考一致；未知实物状态下，限步16不能证明只运动16。
- ARM YAML 经安装链接仍解析为 teleop_ws/src/rm_teleop/config/high_follow.yaml。ARM 原位置比例0.75、速度0.08m/s/12deg/s、launch motion_stop_hold_sec=0.30均保留，可用原 launch 参数选择。
- 保持单 UDP5005 接收器、单 RoboticArm(RM_TRIPLE_MODE_E)/handle、原 Adapter、latest-value mailbox 和独立串行 Worker。无 O7 读、无速度/力矩写入、无端口重配。
- ARM 高频回调、Remote IK、200Hz CANFD、5ms UDP、FAULT_LATCHED、stop 策略、连接/销毁代码均未修改；没有新增 SDK 大锁。

手部启动时 `startup_pending=true`。锁臂模式等待原连接就绪；联动还要求原 ARM 安全检查通过、ACTIVE/command_sent=true 和最近成功 CANFD 发送不超过10ms。
首次条件满足后记录 `O7_HAND_PARAMETER_START`，清空旧缓存，只处理随后新鲜帧，不自动发送全255。
追踪无效/超时清空缓存、停止新提交；锁臂模式恢复后可用新鲜帧继续。
ARM普通退出ACTIVE/短暂CANFD停发会暂停；ARM按原流程恢复健康ACTIVE后，O7自动恢复，但只接受恢复后的新帧。UDP反馈失效/故障锁存/真实断线、SDK异常和超耗时仍停写，不自动清错。
返回码1保留原值并可继续新目标，不推断动作成功、不自动重发。

## 0. 启动前结束旧实例

```bash
pgrep -af 'shared_session_node|pico_hand_receiver_node|o7_shared_session_node|rm65_canfd_high_follow_node|pico_receiver_node|o7_standalone_write|o7_rm485'
ss -lunp '( sport = :5005 )'
ss -tnp '( dport = :8080 )'
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION
```

已有实例先按原流程停臂、Ctrl+C、等待完整退出后再复查。不要同时运行 o7_field.sh 与 launch，不与独立 B 客户端并行。
项目文件锁不能拦住不遵守锁的外部 SDK 程序，不能省略此检查。各终端沿用同一个 ROS domain/RMW，不猜设新值。
现场确认 O7 无接触、无夹持风险、已知安全姿态；ARM工作区和硬件急停可用。

## 1. 每轮先开启被动记录

终端 B：下面以锁臂轮为例，后续 ARM基线/联动轮分别将目录前缀改为 pico_o7_arm_only / pico_o7_combined，不覆盖上轮。

```bash
source /opt/ros/humble/setup.bash
source /home/ai/teleop_ws/install/setup.bash
export O7_RUN="/home/ai/teleop_ws/concurrency_logs/pico_o7_locked_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$O7_RUN"
printf '%s\n' "$O7_RUN" > /tmp/pico_o7_run_path
cp /home/ai/teleop_ws/src/pico_o7/config/o7_device_linear.json "$O7_RUN/device_config.json"
cp /home/ai/teleop_ws/src/pico_o7/config/o7_right_vectors.yaml "$O7_RUN/solver_config.yaml"
set -o pipefail
ros2 run rm_teleop single_session_hil_monitor --duration 900 \
  --output "$O7_RUN/capture" 2>&1 | tee "$O7_RUN/monitor_console.log"
```

不传任何 --execute-*，监测器不调用服务/SDK。本版不是旧 HIL，不靠 --observe-auto-stream 自动分段。
900s仅为采集上限，当前手部跟随没有旧HIL的60s自动结束上限；现场自行短时测试后退出。

## 2. 锁臂手部复核

终端 A，在完成前述现场确认后：

```bash
cd /home/ai/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/home/ai/teleop_ws/log/pico_o7
export O7_RUN="$(cat /tmp/pico_o7_run_path)"
set -o pipefail
ros2 launch pico_o7 teleop.launch.py \
  allow_motion:=false enable_o7_retargeting:=true \
  2>&1 | tee "$O7_RUN/teleop_console.log"
```

终端 C source 同一 ROS 环境后，只读检查：

```bash
ros2 node list
ros2 param get /rm65_canfd_high_follow_node allow_motion
ros2 param get /rm65_canfd_high_follow_node enable_o7_retargeting
ros2 param get /rm65_canfd_high_follow_node o7_device_mapping_path
ros2 topic echo /rm65/canfd_control_state --once
ros2 service call /rm65/o7_hand/status std_srvs/srv/Trigger '{}'
```

状态服务仅用于诊断，可不调用。预期 ARM MOTION_LOCKED、motion_permitted=false、command_sent=false；O7连接就绪后自动 enabled=true，不再期待 WRITES DISABLED。
缓慢少量手势，分别记录跟随、顿挫和意外关节动作。主动遮挡/停止手部追踪至少3秒，记录追踪失效时间和新提交是否停止；恢复时会接受新鲜帧，必须在安全条件下恢复。
SDK返回码不是实物动作验证。
正常结束锁臂实例：终端A Ctrl+C，等待在途SDK返回、Worker和求解器退出；终端B继续记录约10秒再Ctrl+C。
完成第5节收尾，确认旧实例完全退出后才进入下一轮。

## 3. 先验收 ARM 单独基线

重新建立独立 RUN 和被动采集。终端 A 仅将启动选择改为：

```bash
ros2 launch pico_o7 teleop.launch.py \
  allow_motion:=true motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  enable_o7_retargeting:=false
```

按原 PICO 流程先释放，再产生新的 enable 上升沿；本文不猜测 APK 具体物理按键。
原报文 hand_teleop_enabled 要求 deadman=true、teleop_armed=true、input_mode=hand_teleop_active、stop_reason为空、left_controller_connected=false、right_hand_tracked=true、palm.valid。
确认 ACTIVE、command_sent=true、ik_initialized=true、motion_permitted=true、fault_latched=false、反馈<=50ms，维持约10秒。
此轮手部不加载，也不可能通过服务打开。
按原 PICO 流程停止 ARM，确认停发后退出并保存日志。只有这轮现场确认后才选择联动。

## 4. 臂手联动

结束旧实例、重新建立 RUN/监测后启动：

```bash
ros2 launch pico_o7 teleop.launch.py \
  allow_motion:=true motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  enable_o7_retargeting:=true
```

O7在 ARM 未ACTIVE时等待，不要求第二次确认。按原 PICO 流程使 ARM进入健康ACTIVE后，O7自动使用新帧开始。
先短时缓慢小范围联动，不同时大幅动臂和快速全手动作。若安全拒绝/停写，保留原因，不放宽门槛或自动重试。
现场重点复现：手移出PICO画面至少3秒，确认不再有新O7提交；手移回并按原流程恢复ARM ACTIVE后，O7应记录 O7_HAND_RESUME 并用新帧继续跟随，不需要重启或服务。
联动丢追踪可能导致ARM退出ACTIVE；两条路径都应恢复，但需分别核对跟踪/ARM门控。发生FAULT_LATCHED时不得以此流程自动清错。

正常结束：先走原 PICO 停臂流程确认 command_sent=false（O7同时安全停写），再Ctrl+C等待退出；采集延后约10秒结束。
已经进入SDK的调用不能撤回，FINISH可晚于停写。停止提交不代表电机立刻停止或恢复全张开。
现有包装退出先等待手部，因此Ctrl+C不是ARM急停；危险时使用硬件急停，不等服务/日志/SDK。

## 5. 保存、分析和打包

节点和采集结束后：

```bash
export O7_RUN="$(cat /tmp/pico_o7_run_path)"
mkdir -p "$O7_RUN/arm_control"
find /home/ai/teleop_ws/canfd_logs -maxdepth 1 -type f \
  -name '*.csv' -newer "$O7_RUN/device_config.json" -exec cp -t "$O7_RUN/arm_control" -- {} +
python3 /home/ai/teleop_ws/tools/pico_o7_field_review.py report "$O7_RUN" \
  | tee "$O7_RUN/review_console.log"
# 先补填 operator_observations.txt，再打包。
 tar -czf "${O7_RUN}.tar.gz" -C "$(dirname "$O7_RUN")" "$(basename "$O7_RUN")"
```

现场可用本地marker记录阶段（不执行任何控制）：

```bash
python3 /home/ai/teleop_ws/tools/pico_o7_field_review.py mark "$O7_RUN" TRACKING_LOSS_OBSERVATION --note '现场填写遮挡起止和实际动作'
```

原始ROS日志位于 teleop_ws/log/pico_o7（按上面的ROS_LOG_DIR设置），ARM CSV位于 teleop_ws/canfd_logs。
RUN保存capture/{samples.csv,events.csv,summary.json,summary.csv}、配置/console/ARM CSV以及pico_o7_review.json。
原监测器摘要只识别旧HIL，不要用UNTRIGGERED或写入0判断本版跟手；补充报告从O7_HAND_WRITE_*原始事件重算。
若需精确调查追踪输入，在另一终端被动录包，不开额外UDP/SDK：

```bash
source /opt/ros/humble/setup.bash
source /home/ai/teleop_ws/install/setup.bash
export O7_RUN="$(cat /tmp/pico_o7_run_path)"
ros2 bag record -o "$O7_RUN/rosbag" /pico/right_hand_raw /pico/teleop_state \
  /rm65/canfd_control_state /rm65/status /rm65/o7_modbus_hil/event
```

录包结束等待保存再打包。记录是否录包，因为额外采集也有调度负载。

## 分项验收

- ARM：5ms目标；sent周期的interval avg/p95/p99/max与>10ms次数；尝试失败数、停发原因、API耗时。故障周期不丢弃。
- UDP/安全：feedback_age分布、失效、FAULT_LATCHED、stop。没有ACTIVE发送不能算并发通过。
- O7：配置4Hz与实际调用START间隔/Hz分开；Adapter调用耗时不含求解/排队；原始返回码、异常、最后目标及覆盖计数保留。
- 停写：跟踪失效、退出ACTIVE、节点退出分别记录。比较新的实际调用起始时间；在途FINISH不算新提交。START事件可能取消，精确调用时间在FINISH内；缺FINISH保持未验证。
- 实物动作：另记操作者、起始姿态、视频、跟随/顿挫/意外轴、停写后的动作。SDK码1既不证明未执行也不转为成功。
- 历史660笔约224ms、独立B单笔204.389ms和旧共享HIL均不能代替本版验收；正式A/B、独立10Hz复现、长时间联动仍待验证。

本轮正式模块位于 src/pico_o7，使用原 rm_teleop Adapter/Worker 所有权及生命周期。迁移清单、根因、离线结果见 docs/PICO_O7_WORKSPACE_HANDOFF.md 和 docs/pico_o7_migration_20261104/。旧目录只保留参考，没有继续修改；机械臂控制源码不变，没有执行真机测试。
