# V1.0.0 对外接口

名称为当前配置值，实际注册以节点源码/参数覆盖及ROS remap为准。以下不包含ROS2自动参数服务、/rosout和/parameter_events；无自定义Action、HTTP或WebSocket服务。

## 话题

节点简称：R=pico_hand_receiver_node（ROS名称pico_receiver_node），T=target_generator_node，S=shared_session_node（ROS名称rm65_canfd_high_follow_node）。原rm_teleop接收器和控制器提供基础接口，但不应与扩展节点并行启动。

| 名称 | 类型 | 发布 → 订阅/用途 |
|---|---|---|
| /pico/teleop_state | pico_rm65_teleop_interfaces/msg/PicoTeleopState | R → T；使能、seq、掌心/手腕姿态 |
| /pico/status | diagnostic_msgs/msg/DiagnosticArray | R → 观察者；接收诊断 |
| /pico/right_hand_raw | std_msgs/msg/String（JSON） | R → S；接收单调时间、有效性、seq/time及21点 |
| /rm_teleop/flange_target | pico_rm65_teleop_interfaces/msg/RmTeleopTarget | T → S；原始/限速目标与输入稳定状态 |
| /rm65/joint_states | sensor_msgs/msg/JointState | S → 观察者；关节弧度 |
| /rm65/current_flange_pose | geometry_msgs/msg/PoseStamped | S → T；法兰反馈 |
| /rm65/controller_waypoint_pose | geometry_msgs/msg/PoseStamped | S → 观察者 |
| /rm65/feedback_valid | std_msgs/msg/Bool | S → 观察者 |
| /rm65/arm_current_status | std_msgs/msg/Int32 | S → 观察者 |
| /rm65/status | diagnostic_msgs/msg/DiagnosticArray | S → 观察者/旧HIL monitor |
| /rm65/configuration | std_msgs/msg/String（JSON） | S → 观察者；初始化配置 |
| /rm65/canfd_control_state | pico_rm65_teleop_interfaces/msg/RmCanfdControlState | S → 观察者/monitor；周期、返回码及安全状态 |
| /rm65/o7_modbus_hil/event | std_msgs/msg/String（JSON） | S → monitor/录包；初始化、写入及恢复事件 |
| /rm65/o7_stage/result | std_msgs/msg/String（JSON） | S → o7_session_tools；事件副本 |

PICO/手部原始/关节和姿态反馈使用Best Effort；目标、控制状态和HIL事件Reliable。configuration使用Reliable+Transient Local depth1，stage/result使用Reliable+Transient Local depth20；新订阅者须防止把保留事件误认为本次请求结果。自定义消息权威字段见src/pico_rm65_teleop_interfaces/msg，明确带_deg的字段为度。

## 服务

全部为std_srvs/srv/Trigger（空请求，bool success + string message）。异步任务的success可能仅表示接受，最终结果须按request_id/事件类型检查，不等于实际运动成功。

| 服务 | 行为/条件 |
|---|---|
| /rm65/o7_hand/status | 本地状态JSON，不读取设备配置 |
| /rm65/o7_hand/enable | 可选维护：请求初始化；不绕过故障锁存，不是启动必需步骤 |
| /rm65/o7_hand/disable | 可选维护：停止新提交，已在途调用需结束；不会自动张开 |
| /rm65/o7_hand/latency_once | 锁臂环境、计划验证、单次真实写入；正常跟随时拒绝 |
| /rm65/o7_diagnostics/read_positions_once | 读全部位置；须enable_o7_read_diag、allow_motion=false |
| /rm65/o7_diagnostics/read_index_once | 单轴位置诊断；同上 |
| /rm65/o7_calibration/index2/dry_run | 返回计划，无设备指令 |
| /rm65/o7_calibration/index2/write_once | 须校准开关和有效计划、锁臂；一次真实写入 |
| /rm65/o7_modbus_hil/trigger_once | 原节点兼容旧HIL，需要相应开关和安全门槛 |
| /rm65/o7_stream_hil/start | 原节点兼容旧流式HIL，需要相应开关和安全门槛 |

共享节点要求三个旧HIL开关关闭；服务名称存在不代表当前模式允许执行。实时/读诊断/校准互斥。诊断读写不应在正式运行期间试调用。

```bash
ros2 service call /rm65/o7_hand/status std_srvs/srv/Trigger '{}'
# 以下为可选维护操作，会改变O7提交状态：
ros2 service call /rm65/o7_hand/disable std_srvs/srv/Trigger '{}'
ros2 service call /rm65/o7_hand/enable std_srvs/srv/Trigger '{}'
```

初始化日志格式示例（数值仅说明字段，不是新增实验）：
```json
{"event":"O7_MODBUS_INIT_START","timestamp_ns":0,"reason":"session_start_or_resume"}
{"event":"O7_MODBUS_INIT_SUCCESS","reason":"communication_recovery","success":true,"return_code":0,"config":{"return_code":0,"mode":1,"baudrate":115200,"timeout":1},"duration_ms":393.82}
{"event":"O7_MODBUS_INIT_FAILED","reason":"session_start_or_resume","success":false,"return_code":-2,"config":null,"duration_ms":518.94}
```
实际SUCCESS/FAILED还带mode=pico_o7_realtime和timestamp_ns，异常时含error。写入FINISH包含目标、seq、epoch、调用墙钟/单调时钟、duration_ms、return_code、error及status。

## CLI与Launch

安装入口：rm_teleop提供pico_receiver_node、target_generator_node、rm65_canfd_high_follow_node、single_session_hil_monitor；pico_o7提供pico_hand_receiver_node、shared_session_node、o7_session_tools。

正常入口为`ros2 launch pico_o7 teleop.launch.py`，参数见README。原`rm_teleop teleop_high_follow.launch.py`保留兼容：原launch allow_motion默认true但仍要求确认字符串；pico_o7 launch默认allow_motion=false、enable_o7_retargeting=false。

`o7_session_tools`支持inspect/dry-run/read/read-index/node-dry-run/write-once/latency-once/hand-status。write-once须--execute-index2，latency-once须--execute-latency。enable/disable没有CLI快捷子命令，请用上面的服务。inspect中旧返回码说明已滞后于实时控制：实时非零会暂停恢复；命令行仅作为诊断参考，不能用其说明覆盖源码。

`tools/o7_field.sh`保留hand-start、combined-start、hand-status、dry-run、tests；启动子命令会连接设备。standalone_write模块显式执行会创建独立Handle，禁止与正式会话并发。其余离线模块：retarget_dry_run、write_latency_report、pico_hand_skeleton_viewer；现场报告工具为tools/pico_o7_field_review.py。

## 网络、配置与环境

PICO监听UDP 0.0.0.0:5005；allowed_source_ip为空表示无来源IP过滤。协议version=1、uint32 seq、非负timestamp、target_hz 1..1000以及deadman/input_mode/teleop_armed/stop_reason/reset_origin/record_toggle/left_controller_connected/right_hand_tracked/right_palm/right_wrist。姿态为valid、position{x,y,z}、rotation{x,y,z,w}；手部另需21个有效right_keypoints。record_toggle仅转发，不等于自动录包。

RM65 SDK连接192.168.125.19:8080；反馈推送PC 192.168.125.100:8089，5ms。网卡enp5s0为验收环境记录，不是代码硬绑定。SDK通过CANFD API控制，没有另开PC SocketCAN控制器。

全部节点参数见config/high_follow.yaml及declare_parameter；launch会覆盖部分YAML。例：motion_stop_hold_sec YAML为0.50，launch默认0.30；不能仅根据YAML推断最终值。未发现动态参数更新回调，不保证ros2 param set立即影响既存控制对象。

O7 JSON字段write_hz/max_age_sec/max_call_ms/initial_hold/axes不是ROS动态参数；通过o7_device_mapping_path选路径、启动加载。PICO_O7_SHARE可覆盖资源路径；o7_solver_python指定隔离解释器，默认.o7_solver_venv/bin/python。ROS_LOG_DIR控制ROS日志位置。

## O7解耦修复后的控制语义

O7启用、初始化与恢复只依赖共享SDK连接、O7本地状态和新鲜手部输入，不读取ARM ACTIVE、CANFD发送间隔、IK/反馈或ARM故障标志。正常launch仍自动初始化，不需要enable服务。ARM停止并不等于手停止；需要停手可使用disable或关闭节点，ARM故障不会自动清除O7状态。ARM本身的安全逻辑未改变。详见O7_DECOUPLING.md。
