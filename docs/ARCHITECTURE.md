# RM65 + O7 Teleoperation V1.0.0 架构

本文以当前磁盘源码为准；`archive/legacy/` 是历史资料，不是当前运行指南。

## 包与依赖

| 包 | 职责与正式入口 | 依赖 |
|---|---|---|
| pico_rm65_teleop_interfaces | PicoTeleopState、RmTeleopTarget、RmCanfdControlState三个消息 | ament_cmake、rosidl、std_msgs、geometry_msgs |
| rm_teleop | 接收器基类、目标生成、Remote IK、CANFD、反馈及安全状态机 | rclpy、interfaces、geometry/sensor/diagnostic/std消息、std_srvs、launch |
| pico_o7 | 单socket接收器扩展、共享控制节点、隔离求解器和O7 Worker | rm_teleop、rclpy、launch、std_msgs/std_srvs、ament_index_python；求解依赖见requirements.lock |

权威依赖声明是各包package.xml；Python入口是各包setup.py。不改变导入路径或安装规则。

## 正式调用链

```mermaid
flowchart TD
 P[PICO UDP JSON :5005] --> R[pico_hand_receiver_node\n继承原Receiver，一个UDP socket]
 R --> S[/pico/teleop_state]
 S --> T[target_generator_node\n坐标映射、滤波、限速、输入状态机]
 F[/rm65/current_flange_pose] --> T
 T --> X[/rm_teleop/flange_target]
 X --> A[shared_session_node\n继承RM65高跟随节点]
 A --> I[Remote IK + 限位/碰撞检查]
 I --> C[rm_movej_canfd\n200Hz高跟随]
 C --> RM[RM65]
 RM --> U[SDK UDP反馈 :8089]
 U --> F
 R --> H[/pico/right_hand_raw]
 H --> M[LatestHand原始值槽]
 M --> B[o7_solver_bridge线程]
 B --> Q[Python3.11 solver_process\nsomehand RetargetingEngine]
 Q --> V[按名七通道Mapper]
 V --> L[Latest-value待发槽 / epoch]
 L --> W[唯一o7_shared_hand_worker]
 W --> MB[rm_write_registers\n起点间隔至少100ms]
 MB --> RS[Tool RS485 → RS485-CAN → O7]
 A -.唯一Adapter/SDK Handle.-> W
```

launch实际启动三个节点：`pico_hand_receiver_node`（ROS名称pico_receiver_node）、`shared_session_node`（ROS名称rm65_canfd_high_follow_node）、`target_generator_node`。不启动原接收器/原控制节点的第二份实例。

## 线程与生命周期

- 原连接线程创建唯一RoboticArm/Handle，配置UDP并初始化Remote IK。硬件限制同一控制器只能一个Handle。
- ROS控制定时器按200Hz执行原有反馈、目标检查、IK及CANFD逻辑。SDK反馈回调更新FeedbackStore。
- somehand在隔离Python3.11子进程运行；桥接线程以逐行JSON stdin/stdout通信，ROS侧使用系统Python3.10。
- StageHil复用现有生命周期管理，仅一个O7 Worker；写入同步且不重叠。不能用第二SDK客户端规避阻塞。
- Worker与控制循环通常并发使用Handle；没有覆盖所有SDK调用的全局锁。SDK内部资源行为仍依赖厂商实现，不能断言完全无竞争。
- 关闭先使O7失能、停止求解器并等待Worker退出，再交还父节点释放SDK。阻塞native调用无法安全强制撤回。

## 初始化和自动恢复

启动参数enable_o7_retargeting=true直接request_startup，无需Service。初始化位于O7 Worker，通过SharedO7Node._initialize_hand核对共享连接后调用Adapter.initialize_o7_modbus。

设置port1/115200/timeout1，只有设置返回0、查询返回0且mode1/115200/timeout1匹配才承认初始化。新的会话和Handle变化使旧结果失效；Handle检测包含Adapter、arm、handle对象身份和handle.id。

初始化不再读取ARM ACTIVE、CANFD发送间隔、IK或ARM故障状态，也不持有控制定时器调度门。SharedO7Node直接继承原on_timer，不因Modbus初始化跳过ARM周期。O7所有Modbus调用仍在唯一Worker串行执行；与CANFD可并发进入SDK，厂商native内部是否竞争需真机确认，离线测试不能保证其时序性能。初始化不在200Hz定时器或输入回调内执行。

```text
启动/会话恢复/Handle变化 → 初始化待处理 → 共享SDK已连接 → 设置并查询
  ├ 成功 → 清空历史/推进epoch → 等待NEW PICO帧 → 写入
  └ 失败 → 恢复退避 → 连接可用后重试
写入非零/异常/超过750ms → 暂停、清空目标 → 恢复退避
恢复预算耗尽 → O7故障锁存（不再自动写入）
```

恢复延迟按窗口内已用次数为1、2、4秒（代码封顶8秒）；60秒滚动窗口最多三次实际恢复初始化，成功也消耗预算。共享连接不可用时返回None不消耗次数。首次正常初始化不计恢复预算，失败后进入该预算。成功不会立即重发失败目标；输入需在重新enable的accept_after之后。

O7自身恢复预算耗尽、不可恢复本地拒绝、求解器关闭会阻止恢复。ARM运动状态与故障锁存不再控制O7；不修改或解除ARM本身的故障。O7锁存不因Handle变化或enable Service解除；排查后需要重启节点。显式disable保留人工停止意图，不能因重连重新启用。

## 数据与安全

LatestHand仅保留最新原始输入和最新解；epoch阻止旧解回流。max_age_sec=0.25，max_call_ms=750；七轴max_step=16，保留最后提交目标作为限步锚点。10Hz是上限，重复目标、输入不足或SDK阻塞可以降低频率，不补发追赶。

普通无效关键点/过期输入清空槽，不直接触发Modbus恢复。ARM退出ACTIVE、短时CANFD间隙和ARM故障不会暂停O7或失效Modbus配置。O7原始关键点校验并不独立检查deadman/teleop_armed；联动和锁臂模式均按手部自身输入独立跟手；ARM的deadman/teleop_armed不再作为O7准入条件。

ARM的Remote IK、J3/关节限位、速度检查、碰撞检查、反馈新鲜度及故障锁存沿用现有代码。停止提交并不等于物理电机立即停止或自动张开。软件CANFD周期不是物理总线测量；SDK返回0不是实际姿态精度证明。

## V1.0.0后解耦修复

最新实机发现旧10ms CANFD门槛导致O7等待非ACTIVE窗口，见[解耦说明](O7_DECOUPLING.md)。该修复改变了初始化与CANFD并发条件，旧HIL报告不是此修复的真机验收。
