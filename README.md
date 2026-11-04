# RM65 + PICO + LinkerHand O7 遥操作系统

ROS2 Humble 工作空间：PICO 位姿驱动 RM65-B 六轴机械臂，Remote IK + CANFD 200 Hz 高跟随；O7 右手经同一个 RealMan API2 Session、RM65 Tool RS485 和阻塞 Modbus 写入控制。PICO右手21点已接入somehand重定向和O7七轴绝对位置映射，手部模块已统一迁入本工作空间。锁臂跟手及一次本版迁移前联动有现场记录；新版追踪恢复修复仍需现场验收。

当前入口见 [PICO-O7 工作空间交接](docs/PICO_O7_WORKSPACE_HANDOFF.md) 与 [现场流程](docs/PICO_O7_COMBINED_FIELD_GUIDE.md)。旧 HIL 阶段交接保留作历史资料，不再作为开发起点。

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
# 仅手部；ARM=false/true 与 O7=false/true 可独立选择。
ros2 launch pico_o7 teleop.launch.py allow_motion:=false enable_o7_retargeting:=true
# 联动需结束旧实例，再使用：
# ros2 launch pico_o7 teleop.launch.py allow_motion:=true motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED enable_o7_retargeting:=true
```

`enable_o7_retargeting=true` 直接选择手部执行，无 enable/disable 服务步骤。追踪暂失/普通退出ACTIVE只暂停，健康恢复后清缓存并用新帧继续；故障锁存和SDK异常仍停写。

## 真实架构

```text
PICO UDP JSON v1 → 单一增强接收器 → /pico/teleop_state
                       └→ /pico/right_hand_raw → O7求解/映射
                                      ↓
                            target_generator_node
                                      ↓ /rm_teleop/flange_target
                    rm65_canfd_high_follow_node（同一进程）
                                      │
                     RmApi2MotionAdapter / 单 RoboticArm
                     单 API2 handle / RM_TRIPLE_MODE_E
                         ┌────────────┴────────────┐
                    主 ROS 回调              O7 Hand owner
                   Remote IK                   ↓
                CANFD 200 Hz          PICO 21点 → somehand求解
                         │             latest-value mailbox
                         │              串行 Modbus worker
                         └────────────┬────────────┘
                              RM65 Controller
                                   ↓ Tool RS485
                              LinkerHand O7 右手
```

UDP realtime 5 ms 反馈由原高跟随节点接收并发布。不能另开 O7 API2 Client，不能在 200 Hz 回调执行阻塞 Modbus，不能用全局 SDK 大锁让 CANFD 等待 O7。

## 当前状态与证据

| 能力 | 状态 / 证据边界 |
|---|---|
| RM65 位姿高跟随、Remote IK、CANFD 200 Hz、UDP、故障锁存 | 历史真机已验证；当前实现保留 |
| O7 独立控制 10/12 Hz、速度 25 | 操作者历史结果；不代表共享 Session 吞吐量 |
| 同 Session 1 Hz 位置写入 | 220↔255，10 次返回 1；操作者观察到运动，主线无故障 |
| 自动 HIL 10 Hz 目标生成 | 最新日志实际配置 **15 秒**；10.001 Hz 生成、4.032 Hz 实际写入，150 个目标 / 61 次写调用 / 88 次覆盖 |
| SDK 返回码 | 61 次均为 1：SDK 未确认，设备仍可能执行；不能改成成功，也不能认定未执行 |
| 长时运行、所有控制器状态和异常恢复、完整七轴同步 | 尚未验证 |
| PICO 手指数据 / 映射 | 已在 pico_o7 包接入21点、按名七轴映射；0.25s新鲜度、4Hz/限步16，逐轴标定仍待验证 |
| 追踪移出/移回 | 已定位旧永久禁用问题；新版可恢复暂停已通过mock，待现场复验 |

保留原始数据：

- [1 Hz 220↔255](concurrency_logs/o7_write_only_1hz_20261103_165032/summary.json)：2026-11-03 16:50:59～16:51:09 +08:00。
- [自动 HIL](concurrency_logs/o7_auto_10hz_20261103_172402/summary.json)：17:24:36～17:24:51，停止原因 `timeout_limit`；不能替代主动退出 ACTIVE 的真机停止验证。
- [核算证据](docs/RETAINED_HIL_EVIDENCE.json)、[保留文件哈希](docs/RETAINED_HIL_SHA256.json)。原日志未重写，现场运动确认来自操作者，monitor 仍正确记录 `NOT AUTOMATICALLY VERIFIED`。

## 目录与环境

```text
src/
  rm_teleop/
    rm_teleop/                 四个入口及控制、HIL、采集核心
    config/high_follow.yaml  三节点参数
    launch/teleop_high_follow.launch.py
    test/                    164 项自动测试基线
  pico_o7/                   手部正式包：增强接收器/共享节点、重定向、模型、vendor、测试
    launch/teleop.launch.py   当前臂手联合入口
    config/ models/ vendor/  自包含运行资源
  pico_rm65_teleop_interfaces/msg/
    PicoTeleopState.msg / RmTeleopTarget.msg / RmCanfdControlState.msg
docs/                        HIL、整理清单/报告、开发交接
concurrency_logs/            原两轮HIL、当前跟手/追踪恢复证据和后续采集
canfd_logs/                  后续运行时原节点 CSV 输出目录（已清理历史内容）
build/ install/ log/         colcon 生成，Git 忽略
一、参数生效关系.md           参数优先级说明（已校正）
开发交接.md                  legacy：旧 RM65 输入稳定实验笔记
```

运行launch/config/test位于包内；根tools提供字段分析、环境重建和启动便捷脚本。求解Python3.11环境位于工作空间 .o7_solver_venv（Git忽略），不使用旧参考目录环境。

本机核对：Ubuntu 22.04.5 LTS、Python 3.10.12、ROS_DISTRO=humble，RMW 为 `rmw_cyclonedds_cpp`；ROS_DOMAIN_ID 环境未设置，实际部署 domain 待确认。SDK 自动搜索可用路径 `/home/ai/RM_API2/Python`，本地 SDK 仓库 tag 为 `1.1.5`；本次未查询真机固件/运行库版本。需安装 ROS 包 rclpy、diagnostic_msgs、geometry_msgs、sensor_msgs、std_msgs、std_srvs、launch/launch_ros、ament_index_python，以及 colcon、pytest、setuptools 和 ROSIDL 构建依赖。详见两个 package.xml。

## 构建与离线检查

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pico_rm65_teleop_interfaces rm_teleop pico_o7 --symlink-install
source install/setup.bash
python3 -m pytest src/rm_teleop/test src/pico_o7/test tools -q
python3 -m compileall -q src/rm_teleop/rm_teleop
git diff --check
colcon test --packages-select rm_teleop --event-handlers console_direct+
colcon test-result --test-result-base build/rm_teleop --verbose
ros2 pkg executables rm_teleop
ros2 launch rm_teleop teleop_high_follow.launch.py --show-args
```

这些检查不需要连接机器人。不要为例行构建清空 build/install，也不要运行旧双 Client 脚本。

## 常规运行：HIL 全关闭

启动前确认设备和环境安全；按原 PICO 流程启用。仅锁定输出验证时将 allow_motion 显式设为 false（节点仍可能连接机械臂，故本轮整理不会运行此命令）。

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  enable_o7_single_session_hil:=false \
  enable_o7_stream_hil:=false \
  enable_o7_auto_stream_hil:=false
```

**不要混淆默认值**：节点/YAML 的 allow_motion=false，但现有 launch 默认 true、确认字符串默认空；省略 token 时节点会拒绝启动。此差异本轮只记录，不改已验证默认参数。三种 HIL 开关和两种开姿确认始终默认 false。

## HIL 模式

所有 HIL 共用进程一次接受机会，互斥且不会自动重启。以下都是由操作者手动执行的真机命令，不属于离线检查。

自动 HIL：先启动被动 monitor，再启动 teleop；确认 O7 安全张开、无接触且其他六指目标 255 安全。最新已验证运行时间为 15 秒，源码允许上限 60 秒；不能宣称 60 秒已实测。

```bash
ros2 run rm_teleop single_session_hil_monitor \
  --observe-auto-stream --duration 120 \
  --output "$PWD/concurrency_logs/o7_auto_10hz_$(date +%Y%m%d_%H%M%S)"
```

另一个已 source 的终端：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  enable_o7_single_session_hil:=false \
  enable_o7_stream_hil:=false \
  enable_o7_auto_stream_hil:=true \
  o7_auto_stream_hz:=10.0 \
  o7_auto_stream_timeout_sec:=15.0 \
  o7_auto_stream_open_pose_confirmed:=true
```

保持 PICO 失能约 10 秒可记录前置窗口，再手动进入 ACTIVE，约 1 秒后自动开始。前置窗口含 READY，不能当成 10 秒 ACTIVE 基线。无需 Trigger。按原失能流程退出 ACTIVE 或 Ctrl+C 关闭，禁止后续 O7 提交；在途阻塞调用自然返回后才释放 handle，停止下发不等于手指电机已停止或恢复姿态。

手动 1 Hz 使用 `enable_o7_stream_hil`、`o7_stream_open_pose_confirmed`，经 `/rm65/o7_stream_hil/start` 或 monitor `--execute-stream` 启动。legacy 速度单次模式写 HR14～20=25，仅用于回归。详见 [1 Hz HIL](docs/O7_STREAM_HIL.md)、[自动 HIL](docs/O7_AUTO_STREAM_HIL.md)、[legacy 速度 HIL](docs/O7_SINGLE_SESSION_HIL.md)。

## ROS2 接口与参数

| Topic | 消息类型 |
|---|---|
| /pico/teleop_state | pico_rm65_teleop_interfaces/msg/PicoTeleopState |
| /pico/status | diagnostic_msgs/msg/DiagnosticArray |
| /rm_teleop/flange_target | pico_rm65_teleop_interfaces/msg/RmTeleopTarget |
| /rm65/current_flange_pose、/rm65/controller_waypoint_pose | geometry_msgs/msg/PoseStamped |
| /rm65/joint_states | sensor_msgs/msg/JointState |
| /rm65/feedback_valid | std_msgs/msg/Bool |
| /rm65/arm_current_status | std_msgs/msg/Int32 |
| /rm65/status | diagnostic_msgs/msg/DiagnosticArray |
| /rm65/configuration | std_msgs/msg/String（JSON，transient local） |
| /rm65/canfd_control_state | pico_rm65_teleop_interfaces/msg/RmCanfdControlState |
| /rm65/o7_modbus_hil/event | std_msgs/msg/String（JSON） |

服务仅 `/rm65/o7_modbus_hil/trigger_once` 和 `/rm65/o7_stream_hil/start`，均为 std_srvs/srv/Trigger；success=true 是接受请求，不是设备动作确认。没有项目自定义 Action。

关键参数：publish_hz=200、ik_dt_sec=0.005、udp_period_ms=5、feedback_freshness_timeout_sec=0.05、feedback_disconnect_timeout_sec=0.20、target_freshness_timeout_sec=0.25、j3_hard_min_deg=10、j3_activation_min_deg=15。通过 launch 的 position_scale=0.75、translation_speed_mps=0.08、rotation_speed_deg_s=12；motion_stop_hold_sec 的 launch=0.30 会覆盖 YAML=0.50。其余以 [参数说明](一、参数生效关系.md) 和 --show-args 为准；不是所有节点参数都暴露为 launch 参数。

## 安全与后续开发

严重反馈/控制器/发送异常走原快速 stop 和 FAULT_LATCHED，不自动清错；普通失能停止流。SDK handle 由单一 Adapter 拥有，关闭等待所有使用者，不能强杀线程后立即 delete。

已验证的是日志覆盖的具体运行；未验证无限时长、全部控制器状态或全部故障恢复。当前接受约 4 Hz 实际写入，不在本阶段优化 Modbus。下一阶段先调查 PICO 发送端能提供什么手指数据，再设计/离线验证映射，不复制 HIL 循环作为正式控制器。

整理审计见 [清点清单](docs/WORKSPACE_CLEANUP_INVENTORY.md)、[整理报告](docs/WORKSPACE_CLEANUP_REPORT.md)。历史数据归档到 `/home/ai/teleop_ws_legacy_archive/20261103_174140/`，不是永久删除；当前未自动提交 Git。
