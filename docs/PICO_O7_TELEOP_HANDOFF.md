# RM65 + O7 单 Session 并发控制阶段总结与 PICO 手指遥操作开发交接

> 本文为2026-11-03单次/自动HIL阶段历史交接。2026-11-04已完成PICO手部接入并统一迁入teleop_ws；当前请从 [工作空间交接](PICO_O7_WORKSPACE_HANDOFF.md) 开始。以下“待开发/待调查”和旧 NEW_SESSION 内容只代表当时状态。

核查日期：2026-11-03（Asia/Shanghai）。工作空间 `/home/ai/teleop_ws`。本轮仅源码/原始日志审计、外部归档、文档和离线验证；不连真机、不改控制代码、不做 Modbus 优化。

## 1. 项目背景与当前边界

RM65-B 六轴机械臂以 PICO 掌心位姿遥操作，ROS2 Humble / Python，Remote IK 输出 CANFD 200 Hz。LinkerHand O7 右手经 RM65 Tool RS485 接收位置目标，已完成同 API2 Session 并发 HIL；当前 O7 输入仍为固定往返/三角波，**没有 PICO 手指关节解析或七自由度映射**。

已实测、已 mock 覆盖、未验证必须分开：源码保留原安全机制；真机结论只覆盖下面两轮窗口；长时运行、所有控制器状态/恢复路径不能由单次成功推出。下一阶段接受约 4 Hz 实际 Modbus 写能力，先调查输入数据，不重新研究或更换已确定的单 Session 架构。

## 2. 当前环境（实际核查）

| 项目 | 当前证据 |
|---|---|
| OS | `/etc/os-release`：Ubuntu 22.04.5 LTS |
| ROS2 | ROS_DISTRO=humble，`/opt/ros/humble` 可用 |
| Python | `python3 --version`：3.10.12 |
| SDK | `/home/ai/RM_API2/Python/Robotic_Arm` 存在，本地 SDK 仓库 `git describe` 为 1.1.5；历史 C API v1.1.5。实际加载二进制/控制器固件版本本轮未查询，待运行时确认 |
| SDK 搜索 | YAML api2_python_path 空，环境 RM_API2_PYTHON_PATH 未设置；Adapter 自动候选含 `~/RM_API2/Python`；不打包厂商 SDK 到本仓库 |
| Robot | YAML 与保留日志：192.168.125.19:8080；日志 arm_model=RM_65、arm_dof=6、force_type=B、controller_version=3 |
| UDP | YAML/控制台：目标PC 192.168.125.100:8089，周期5ms；当前网卡配置/可达性待确认，没有 ping/连接 |
| ROS domain | ROS_DOMAIN_ID 环境未设置；本轮不创建 ROS node，实际部署 domain 待确认，不用历史上下文臆造 |
| RMW | 环境与离线 `get_rmw_implementation_identifier()` 均为 rmw_cyclonedds_cpp |
| 工作分支 | cleanup/remove-obsolete-code，起点 df980a2；阶段功能/文档尚未自动提交，先查 git status，勿 git clean |

ROS 包只有 `rm_teleop`（ament_python）和 `pico_rm65_teleop_interfaces`（ament_cmake）。前者依赖 rclpy、geometry_msgs、sensor_msgs、diagnostic_msgs、std_msgs、std_srvs、launch/launch_ros、ament_index_python 和自定义接口；后者由 rosidl_default_generators 生成三种消息。没有项目自定义 Action 或自定义 srv。

## 3. 当前系统架构

```text
PICO（发送端不在当前 src 中）
  ↓ UDP JSON v1 / 5005
PicoReceiverNode → PicoTeleopState（掌心/腕位姿、跟踪与启停状态）
  ↓
TargetGeneratorNode / TargetGeneratorCore（坐标映射、轻量滤波、STILL/MOVING/SETTLING、限速）
  ↓ RmTeleopTarget / flange_target
Rm65CanfdHighFollowNode：Remote IK → 位置/速度/J3/自碰撞检查 → rm_movej_canfd(follow=True)
  ↓ 200 Hz / 原唯一 RmApi2MotionAdapter / RM_TRIPLE_MODE_E
RM65
```

```text
同一个 rm65_canfd_high_follow_node 进程内部
  SingleSessionHil owner（准入、互斥、一次接受机会、取消和生命周期）
    ↓ 自动 HIL
  AutoStreamRunner.generate（10 Hz，monotonic 时间三角波）
    ↓
  LatestTargetMailbox（只有一个 pending 值，覆盖过期值）
    ↓
  AutoStreamRunner.run/write（唯一串行 Modbus writer）
    ↓ adapter.write_o7_hil_positions → 同一 adapter.arm / API2 handle
  RM65 Controller → Tool RS485 port=1 → Modbus slave=39 → O7 右手
```

原主控制回调不等待 O7。生成与发送线程分开，生成线程不调用 SDK；SDK 220～400ms 阻塞期间 mailbox 继续更新，实际发送只取最新目标。Python 两线程不能保证厂商 SDK 内部无互斥/GIL/响应串扰；约 4 Hz 是已观测吞吐而非接口承诺。

## 4. 真实源码索引

路径均相对工作空间。所有运行入口由 `src/rm_teleop/setup.py` 安装，不是根 scripts 目录。

| 文件 | 类 / 核心函数 | 功能及依赖 | 下一阶段建议 |
|---|---|---|---|
| src/rm_teleop/rm_teleop/pico_receiver_node.py | PicoReceiverNode.on_poll_timer / publish_packet / publish_safe_state | UDP非阻塞接收、ReceiverCore、发布PicoTeleopState；超时安全失能 | Stage1先调查，必要时最小扩展手指协议输出 |
| src/rm_teleop/rm_teleop/protocol.py | TeleopPacket / parse_datagram | JSON v1、有限值/类型/四元数/跟踪门控 | 先确定真实发送数据，再设计扩展和兼容 |
| src/rm_teleop/rm_teleop/receiver_core.py | ReceiverCore / SequenceTracker | 序号、去重、重启/回退和统计 | 优先保留 |
| src/rm_teleop/rm_teleop/target_generator_node.py | TargetGeneratorNode.on_pico_state / on_timer | 把 palm_pose 与当前法兰输入 core，发布RmTeleopTarget | ARM路径冻结，不混入阻塞手指通信 |
| src/rm_teleop/rm_teleop/target_generator_core.py | TargetGeneratorCore.step / _active_invalid_reason | 相对锚定、位姿滤波、运动门控/追赶、目标时效 | 保留，不为手指重构 |
| src/rm_teleop/rm_teleop/target_math.py | Pose3D、坐标/四元数及限速工具 | 纯数学，单元测试覆盖 | ARM坐标映射保持 |
| src/rm_teleop/rm_teleop/rm65_canfd_high_follow_node.py | Rm65CanfdHighFollowNode._connection_worker / on_timer / _run_control_cycle / _critical_fault / destroy_node | 持有唯一Adapter；FK/IK、CANFD、UDP与安全状态；HIL owner | 将来只在明确设计后接入手指目标/启停，不能破坏主线 |
| src/rm_teleop/rm_teleop/rm_api2_adapter.py | RmApi2ReadOnlyAdapter.connect / initialize_remote_ik / solve_remote_ik / disconnect | SDK模块、RoboticArm/handle、UDP callback、算法初始化与释放 | 会话所有权不得分散 |
| src/rm_teleop/rm_teleop/rm_api2_motion_adapter.py | RmApi2MotionAdapter.send_joint_canfd / write_o7_hil_positions / write_o7_hil_speed_once / hard_stop | 原运动及Modbus封装，同一self.arm；位置7值校验与大端编码 | 复用现有写入，不新建Client |
| src/rm_teleop/rm_teleop/rm_feedback_core.py | FeedbackStore / snapshot_from_api2_state | UDP快照、回调校验/统计 | 保留原freshness语义 |
| src/rm_teleop/rm_teleop/o7_single_session_hil.py | SingleSessionHil.trigger / record / begin_close / wait_closed；run_position_stream | 三种模式共享一次接受机会与取消；手动1Hz仍有回归价值 | 保留HIL；正式控制不能直接复制其一次性逻辑 |
| src/rm_teleop/rm_teleop/o7_auto_stream_hil.py | AutoStreamConfig / triangle_positions / LatestTargetMailbox / AutoStreamRunner | 生成/最新值/串行发送、超时/取消；调用MotionAdapter | 可参考mailbox与生命周期；轨迹、授权和上限是HIL专用 |
| src/rm_teleop/rm_teleop/single_session_hil_monitor.py | main / Monitor.control / status / event | 独立进程，订阅采集；显式execute选项才调用Trigger | 持续用于集成验证 |
| src/rm_teleop/rm_teleop/hil_monitor_data.py | Capture / distribution / diagnostic_level | 原始事件/CSV、真实时间分期、分别统计节点发送与ROS接收 | 保留真实返回码和数据完整性 |
| src/rm_teleop/rm_teleop/canfd_csv_logger.py | CanfdCsvLogger | 原主节点逐周期CSV | 不为手指控制修改其原发送路径 |
| src/rm_teleop/rm_teleop/canfd_control_core.py | motion_permission / tracking_error 等 | 授权和跟踪诊断；部分限步/时序工具供测试/旧实验 | 不因未在主循环调用就删掉整个模块 |
| src/pico_rm65_teleop_interfaces/msg/PicoTeleopState.msg | 消息 | 版本/序号/有效性/启停、palm/wrist；没有手指数组 | 新字段需Stage1/2设计，不能假设已存在 |
| src/pico_rm65_teleop_interfaces/msg/RmTeleopTarget.msg | 消息 | 原始/限速目标、门控/追赶状态 | ARM接口保留 |
| src/pico_rm65_teleop_interfaces/msg/RmCanfdControlState.msg | 消息 | state/reason、发送/IK耗时、反馈age、返回码、stop/fault、关节诊断 | 后续继续采集 |

补充入口：pico_receiver_node、target_generator_node、rm65_canfd_high_follow_node、single_session_hil_monitor。唯一 launch 是 `src/rm_teleop/launch/teleop_high_follow.launch.py`，加载包 share/config/high_follow.yaml；参数字典覆盖 YAML，即使命令行没有显式给值。

## 5. 单 API2 Session 所有权与关闭顺序

1. 主节点 `_connection_worker()` 创建 **一个 RmApi2MotionAdapter**，调用其 connect()，继承实现创建 `RoboticArm(RM_TRIPLE_MODE_E)`，调用 rm_create_robot_arm，保存 `self.arm` 和 `self.handle`。初始化失败会释放并延时重试；这里是顺序重试，不是并行第二客户端。
2. 初始化唯一 UDP callback、5ms推送、Remote IK 后，将 Adapter 引用放入节点 `_adapter`。`_adapter_lock` 只保护引用和连接状态，不覆盖 SDK 调用。
3. 主回调取同一引用，在锁外调用 Remote IK、安全检查和 send_joint_canfd。O7 owner 接受该 Adapter 引用，worker 调用其 write_o7_hil_positions，最终使用同一 arm。没有第二次 create、独立 Client 或额外UDP callback。
4. 初始连接成功后连接线程返回。运行时反馈失效走原 stop/FAULT_LATCHED，不会自动重连、清错或复位。
5. 关闭：设置 `_stop_event`、hil.begin_close取消并拒绝新工作，摘除节点引用；按原策略在仍运动时请求 hard_stop；hil.wait_closed等待writer。自动writer退出前join目标生成线程。然后 Adapter.disconnect恢复原UDP配置（若配置允许）、rm_delete_robot_arm并清引用/callback/IK状态。初始化尚未结束时由连接线程检查关闭状态并清理，再join。
6. 等待2秒仍阻塞会明确记录并继续保留handle等待；Python不能安全强杀原生调用。主控制周期不join worker；关闭可能延长，这是避免use-after-free的必要限制。严重stop本身仍可能受SDK并发/阻塞影响，现场不能只依赖软件。

禁止从独立O7 ROS节点再创建RoboticArm。若未来分离进程，Python对象/handle不能直接共享，必须重新设计会话所有者和进程间目标传递；本轮不做这一变更。

## 6. 保留真机结果与可追溯证据

所有时间均为 Asia/Shanghai。原始文件未改写，原诊断中的旧绝对机械臂CSV路径保留；当前集中保存路径见下表，映射与哈希见 [归档校验](WORKSPACE_ARCHIVE_VERIFICATION.json)。[RETAINED_HIL_EVIDENCE.json](RETAINED_HIL_EVIDENCE.json) 从samples重新核算各阶段计数/间隔/feedback age，核对events与summary一致；没有用人工叙述覆盖原日志。

### 6.1 1 Hz：220↔255 / 10秒

目录：[o7_write_only_1hz_20261103_165032](../concurrency_logs/o7_write_only_1hz_20261103_165032/summary.json)。事件开始16:50:59.911517，结束16:51:09.912172。目录时间16:50:32是采集器启动，不是注入起点。

可确认参数来自STREAM_START：stream_hz=1、duration_sec=10、joint=2、A=[255,255,220,255,255,255,255]、B全255、open_pose_confirmed=true。诊断motion_output_enabled=true、connected/feedback_valid/ik_initialized=true。没有保存逐字历史shell CLI，不能臆造其他未记录参数；下文提供当前代码等效复现命令。

- 10次SDK调用，10次返回1，确认成功0，异常0；原始实际运动字段为NOT AUTOMATICALLY VERIFIED，操作者在历史任务中确认食指运动。
- Modbus avg=220.216325ms、p95=259.443414ms、p99=282.463048ms、max=288.217957ms。
- INJECTION：CANFD2000/2000发送，失败/停发/锁存/异常stop/反馈失效样本均0；间隔avg=5.000001ms，p99=5.031792ms，max=5.219386ms，>10ms=0。feedback age max=5.334555ms。
- BASELINE/POST各2000个成功发送；全三阶段无>10ms，基线max=8.238700ms，不能只看注入均值。
- 正常duration_complete，最后目标全255；没有额外恢复写入，也不能由最后目标推断实际已恢复。

实际文件：samples.csv、events.csv、summary.json、summary.csv、`arm_control/rm65_canfd_high_follow_20261103_165029.csv`。**monitor_console.log和teleop_console.log不存在，没有补造**。

### 6.2 自动：10 Hz目标 / 实际约4 Hz

目录：[o7_auto_10hz_20261103_172402](../concurrency_logs/o7_auto_10hz_20261103_172402/summary.json)。17:24:36.563190 START，17:24:51.563277 STOP，17:24:51.885066 FINISH。actual timeout_sec=**15.0**，源码default/max=60.0；不能称60秒真机测试已完成。

事件参数：configured_target_generation_hz=10、triangle_period_sec=2、范围220～255、joint2、其他六目标255、open_pose_confirmed=true；控制台确认RM_TRIPLE_MODE_E路径初始化的RM65、主线200Hz、follow=True、trajectory_mode=0、motion_permitted=True。

| 指标 | 原日志值 |
|---|---:|
| 实际目标生成频率 | 10.001457757 Hz |
| 实际Modbus写调用频率 | 4.031874359 Hz |
| 生成目标 / SDK调用 | 150 / 61 |
| 覆盖 / 停止丢弃pending / 生成跳slot | 88 / 1 / 0 |
| Modbus avg / p95 / p99 / max | 250.165413 / 342.020733 / 397.117763 / 436.590997 ms |
| SDK 0 / SDK 1 / SDK异常 | 0 / 61 / 0 |
| 注入期间CANFD发送 / 失败 / 停发 | 3064 / 0 / 0 |
| CANFD interval avg / p99 / max | 4.999989 / 5.033439 / 5.155292 ms |
| CANFD >10ms / 锁存样本 / stop样本 | 0 / 0 / 0 |
| feedback age max（INJECTION） | 4.344360 ms |
| 最后目标joint2 | 227（不是255） |
| 停止原因 | timeout_limit |

计数满足150=61写+88覆盖+1停止丢弃。BASELINE为2000条控制记录，其中只有209条发送、1791条waiting_for_enable，不是10秒持续ACTIVE基线；POST 2000条均发送，feedback age max=5.766755ms。采集器更早的OUTSIDE启动阶段有connected=false/feedback_valid=false，不能把它误读为注入期间UDP中断，也不能从摘要抹掉启动事实。

STOP后约0.322秒FINAL是允许在途调用返回。此轮没有通过退出ACTIVE结束，**自动超时结束不等于主动失能/所有故障停止路径已经真机验证**。操作者确认实际O7运动，设备侧执行仍未可靠确认。

实际文件：原六项samples/events/summary.json/summary.csv/monitor_console.log/teleop_console.log，以及 `arm_control/rm65_canfd_high_follow_20261103_172406.csv`。重复tar的六个文件已逐一哈希匹配，归档到外部，不在当前目录重复保存。

### 6.3 历史资料与验证层次

独立O7的10/12Hz和速度25较平滑属于操作者历史记录，当前保留两轮不能重新证明独立客户端吞吐；严禁把它写成共享Session实际10～12Hz。双Client曾观察UDP反馈问题，不是唯一根因已被严格证明。

旧速度单次、读取失败、245↔255实验、双Client并发CSV、R1～R5和旧IK/ARM日志，已完整归档 `/home/ai/teleop_ws_legacy_archive/20261103_174140/`。参见 [清点](WORKSPACE_CLEANUP_INVENTORY.md)、[操作清单](WORKSPACE_CLEANUP_MANIFEST.json)、[实际校验](WORKSPACE_ARCHIVE_VERIFICATION.json)、[保留哈希](RETAINED_HIL_SHA256.json)。不存在永久销毁。

## 7. O7 接口约定与现有限制

硬件约定：Tool RS485 port=1，右手slave=39(0x27)，115200，已配置Modbus mode=1。这些是已验证历史设备配置，本轮没有读设备核对或重设。位置0～255是O7设备单位，不是机械臂角度。

| O7索引 | 关节 |
|---:|---|
|0|thumb_pitch|
|1|thumb_yaw|
|2|index_pitch|
|3|middle_pitch|
|4|ring_pitch|
|5|little_pitch|
|6|thumb_roll|

| 地址 | 设备约定 | 当前代码使用 |
|---|---|---|
| HR0～6 | 目标位置 | write_o7_hil_positions每次写完整7向量 |
| HR7～13 | 目标力矩 | 未实现本轮控制 |
| HR14～20 | 目标速度 | legacy write_o7_hil_speed_once写25；设置可能持久保留 |
| IR0～6 | 实际位置 | 当前HIL不读 |
| IR14～20 | 实际速度 | 当前HIL不读 |

位置调用参数固定port=1/device=39/address=0/num=7，校验7个int且0～255；编码 `[hi0,lo0,...,hi6,lo6]`，例如25→[0,25]。SDK返回值原样保留；1是SDK未确认，不是动作必然失败，也不是成功别名。

自动目标三角波按monotonic时间：255→220耗1秒，220→255耗1秒，四舍五入整数。最新值mailbox只保留一个pending，串行writer不会并发调用，也不重发失败目标。target_generation_hz和actual_modbus_write_hz分别用实际时间序列计算，不足两个样本为null。自动HIL当前仅接受10Hz、timeout0.1～60秒；手动模式仅1Hz、2～10秒。正式控制必须另行设计启停/故障语义，不能把HIL的定时轨迹或“非零一直容忍到超时”直接当生产策略。

## 8. 下一阶段目标和首要调查

目标仅为 **PICO手指→O7位置遥操作**，保留ARM200Hz、单Session、串行写和latest-value，接受约4Hz实际写入，不开展Modbus优化。

```text
PICO手部数据 → ROS2接收 → 手指关节解析 → PICO/O7映射
  → 输出范围/安全限制 → latest-value mailbox → 串行O7 worker
  → 原共享API2 handle → RM65 Tool RS485 → O7
```

现状调查结果：parse_datagram读取JSON version/seq/timestamp/target_hz、deadman/input_mode/teleop_armed/stop_reason、reset_origin/record_toggle、left_controller_connected/right_hand_tracked、right_palm/right_wrist。TeleopPacket与PicoTeleopState没有finger/bone/joint数组，publish_packet只转发掌心/腕和状态；TargetGeneratorNode.on_pico_state只使用palm_pose。不能因为发送额外字段就认为它被解析/发布，也不能据此断言PICO SDK拿不到手指数据。

第一阶段还需调查发送端工程（当前src没有PICO SDK/Unity发送端）：SDK/API是否能提供手指骨骼、真实字段/排列、单位/坐标系、更新率/时戳、左右手标识、跟踪置信度/失效行为、版本兼容和数据包大小。当前约24Hz只是既有注释/历史位姿观测，不是手指数据实测频率。

映射设计必须先确定每个O7自由度对应哪些输入、弯曲方向、开/闭范围、标定/比例、滤波/死区、拇指pitch/yaw/roll三自由度的可观测性和耦合。无真实数据不得硬猜索引或用掌心姿态替代手指关节。

安全决策待设计：手指启用是否单独deadman；PICO跟踪丢失/数据过期如何处理；退出ARM ACTIVE是否同步停发；在途目标和最后位置如何处置；是否需要受控归零/张开；SDK未确认、设备执行无反馈时如何降级。停止发送不是电机急停，自动归零不是默认安全行为。

## 9. 推荐开发顺序（逐阶段人工决定，不自动升级）

1. **Stage1 数据调查**：只读检查现接收协议/消息与PICO发送端，输出带真实样本、单位、排列、时戳/频率、失效标志的数据结构说明。禁止先写完整映射。
2. **Stage2 映射设计**：定义七自由度输入/输出、标定、范围、方向、拇指耦合和启停表；不动真实机械臂。
3. **Stage3 离线验证**：用模拟或录制数据验证映射、极值/NaN/过期/丢跟踪、限幅和滤波，不使用API2真机连接。
4. **Stage4 单Session集成**：在统一会话owner内接入映射最新目标，生命周期统一；避免第二Client，主控制不等Modbus，HIL与正式模式互斥语义须明确。
5. **Stage5 真机HIL**：人工授权先单指、再多指、最后七自由度；每步分别记录主线时序、反馈、故障、SDK结果和实际动作，不自动跨阶段。
6. **Stage6 联合遥操作**：RM65高跟随+O7输入同时运行，验证长时、主动失能、通信异常与状态关联；保留原fault/stop，逐项补足证据。

## 10. 已知风险与尚未验证

1. SDK返回码与设备执行可能不一致，不能转成成功或认定未执行。
2. 共享Session实际写约4Hz，不能承诺目标频率等于吞吐量。
3. 当前保留自动轮只有15秒，未验证长时运行；源码60秒上限有mock覆盖但不等同真机60秒验证。
4. PICO手指数据尚未解析/转发；发送端获取能力待调查。
5. 尚未验证O7七自由度同步遥操作和拇指映射。
6. 尚未验证所有通信异常/控制器状态/恢复路径；不新增自动复位。
7. ARM主控制不能等待Modbus，也不能用全局SDK锁掩盖并发问题。
8. handle生命周期必须统一；SDK长期阻塞无法安全强杀，退出可长期等待。
9. 缺少设备侧可靠执行确认，速度设置和最后目标可能持续生效。
10. timeout正常结束不证明主动退出ACTIVE停止路径已真机验证；mock不是设备证据。
11. 其他六指固定255只对事先确认安全张开姿态有意义，不保证任意初始状态不运动。
12. launch allow_motion=true与YAML=false、motion_stop_hold_sec 0.30与0.50存在覆盖差异。本轮只记录，不改默认值。当前PICO来源IP未限制、workspace_limit_enabled=false也按源码保留，不擅自修改。

## 11. 下一阶段命令入口

全部命令在 `/home/ai/teleop_ws`。本轮整理只执行离线检查，以下真机运行命令交由操作者。

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pico_rm65_teleop_interfaces rm_teleop --symlink-install
source install/setup.bash
python3 -m pytest src/rm_teleop/test -q
python3 -m compileall -q src/rm_teleop/rm_teleop
git diff --check
colcon test --packages-select rm_teleop --event-handlers console_direct+
colcon test-result --test-result-base build/rm_teleop --verbose
ros2 pkg executables rm_teleop
ros2 launch rm_teleop teleop_high_follow.launch.py --show-args
```

正常高跟随，HIL全部显式关闭：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  enable_o7_single_session_hil:=false \
  enable_o7_stream_hil:=false \
  enable_o7_auto_stream_hil:=false
```

只锁定输出时用allow_motion:=false，但节点仍会建立机器人连接，不能当作无硬件离线测试。

当前自动HIL等效复现（不是历史逐字CLI），须先确认O7开姿/无接触且七目标安全。先在终端B被动监测：

```bash
ros2 run rm_teleop single_session_hil_monitor \
  --observe-auto-stream --duration 120 \
  --output "$PWD/concurrency_logs/o7_auto_10hz_$(date +%Y%m%d_%H%M%S)"
```

再在已source的终端A：

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

手动1Hz改为enable_o7_auto_stream_hil:=false、enable_o7_stream_hil:=true、o7_stream_hz:=1.0、o7_stream_duration_sec:=10.0、o7_stream_joint:=2、o7_stream_open_pose_confirmed:=true，再使用monitor --execute-stream或Trigger `/rm65/o7_stream_hil/start`，不能省略前述运动授权。不存在旧o7_stream_amplitude或旧--auto-o7有效入口。

现场状态观察（这些是已有运行系统的只读ROS订阅，不在本轮启动）：

```bash
ros2 topic echo /rm65/canfd_control_state
ros2 topic echo /rm65/status
ros2 topic echo /rm65/o7_modbus_hil/event
ros2 topic hz /pico/teleop_state
```

root README列出全部主要topic类型；Trigger success只代表接受请求。已有state/reason、command_sent/attempted、fault_latched、feedback_age、stop标志和返回码不能为方便测试而关闭。退出原PICO ACTIVE取消后续O7提交，不追加恢复；异常现场按既有物理安全流程处置。

## 12. 本轮整理和测试状态

数据只保留两轮；96个历史文件复制到外部并逐一SHA-256/数量/大小核对后才移除原路径。两份关联ARM CSV集中到各测试arm_control子目录。源码、测试、接口、launch和YAML保持字节不变，未删HIL功能。旧技术笔记标为legacy，保留历史分析，不作为下一阶段调参指令。

详细结果以 [WORKSPACE_CLEANUP_REPORT.md](WORKSPACE_CLEANUP_REPORT.md) 为准。Git没有自动add/commit/push，新增HIL文件和文档仍需之后审阅纳入版本管理。不可git clean删除尚未跟踪的工程成果。

## LEGACY_SESSION_START_20261103

当前项目是RM65-B六轴机械臂、PICO和LinkerHand O7右手遥操作系统，工作空间为~/teleop_ws。已完成机械臂位姿遥操作、Remote IK、CANFD二百赫兹、五毫秒UDP反馈、安全检查与故障锁存，以及同一API2会话下的O7并发位置HIL。下一阶段只开发PICO手指到O7位置遥操作，不再优化Modbus吞吐，不重新设计机械臂控制。

本机为Ubuntu二十二点零四点五、ROS2 Humble、Python三点十点十二；SDK目录是~/RM_API2/Python，本地SDK仓库标签为一点一点五。配置机器人地址192.168.125.19:8080，PC反馈目标192.168.125.100:8089。RMW为rmw_cyclonedds_cpp，ROS_DOMAIN_ID环境未设置，实际部署域待确认；控制器实时固件和加载库版本未在整理时查询。

最终架构必须保留：一个RoboticArm、一个API2 handle、一个TCP Client，线程模式RM_TRIPLE_MODE_E。高跟随节点统一拥有RmApi2MotionAdapter，机械臂主回调做Remote IK和CANFD；O7独立串行worker复用同一adapter.arm，不能另建客户端，不能在主循环调用阻塞Modbus，也不能引入全局SDK大锁。若拆分进程，不能直接共享Python对象，必须另行设计会话所有权和IPC。

两轮保留原始日志在concurrency_logs：o7_write_only_1hz_20261103_165032和o7_auto_10hz_20261103_172402。前者食指二百二十到二百五十五、每秒一次十秒，十次SDK返回一，操作者观察到运动。后者实际配置十五秒，并非六十秒；生成约十赫兹、实际写约四点零三二赫兹，一百五十个目标、六十一次写、八十八次覆盖、一个停止丢弃，平均调用约二百五十毫秒。SDK均返回一，主线注入期间发送失败、故障锁存和反馈失效均为零。

返回一的准确含义是SDK未确认，不能转成成功，也不能断定设备未执行。实际动作来自操作者观察，monitor仍标NOT AUTOMATICALLY VERIFIED。自动轮因超时结束，不能把它当作主动退出ACTIVE或所有故障恢复都已真机验证；长时运行、七自由度同步控制和可靠设备执行确认仍缺证据。其他六指固定二百五十五仅在操作者确认安全开姿时适用，停止下发不代表电机立即停止或恢复初始姿态。

重要源码在src/rm_teleop/rm_teleop：pico_receiver_node、protocol、target_generator_node/core、rm65_canfd_high_follow_node、rm_api2_adapter、rm_api2_motion_adapter、o7_single_session_hil、o7_auto_stream_hil。消息在src/pico_rm65_teleop_interfaces/msg。三种HIL默认关闭并共用进程一次接受机会；自动模式有上限且不重启，正式控制不能照搬其轨迹和一次性策略。latest-value mailbox只保留最新目标，不排队，接受当前约四赫兹实际写入。

第一项任务是调查真实PICO手指数据。当前JSON协议和PicoTeleopState只解析、转发右掌/腕位姿与跟踪启停状态，没有手指关节数组；目标生成器只消费掌心。发送端工程不在当前src内，先找发送端SDK能力、字段排列、单位坐标、左右手、置信度、时戳和频率，输出真实数据结构及样本，再设计拇指三自由度和其余手指映射。不要先猜索引、写完整映射或连接真机。

推荐按数据调查、映射设计、录制或模拟数据离线验证、同会话集成、人工授权单指到多指HIL、完整联合遥操作分阶段推进，不能自动跳级。必须设计PICO失效、O7启停和ARM退出ACTIVE时最后目标的处理；保留原故障锁存与stop语义。主线程不能等worker；退出时等待在途调用和生成线程，禁止提前销毁handle。源码与配置本轮未改，历史数据已外部校验归档，未知文件未删；当前Git有未提交成果，先查状态，不能git clean。以本交接和根README为准，旧开发交接笔记标为legacy。

O7约定是Tool端口一、从站三十九、波特率一一五二零零；七索引顺序为拇指俯仰、拇指侧摆、食指、中指、无名指、小指、拇指滚转。当前位置写接口每次写七个大端十六位寄存器，位置范围零至二百五十五；速度二十五的独立实验只是历史效果，不代表正式映射或共享吞吐。当前测试不读任何O7寄存器，速度设置可能持久保留，不能假定退出会复原。

正常运行须明确关闭三种HIL，机械臂运动仍须allow_motion与精确motion_confirmation共同授权。注意现有launch默认允许运动但确认字符串为空，直接节点默认锁定，两者不可混淆。先运行离线测试和参数解析，不把锁定运动的节点启动当成不连接硬件的离线操作。整理清单、外部归档校验和保留日志哈希均在docs中，可据此核对后续文件来源。
