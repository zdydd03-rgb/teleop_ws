# rm_teleop — V1.0.0

提供PICO协议与Receiver基类、目标生成/坐标映射/滤波/限速、RM65 Adapter、Remote IK、200Hz CANFD与原机械臂安全状态机。interfaces依赖pico_rm65_teleop_interfaces；正式臂手联动由pico_o7扩展节点复用这些实现，不要同时启动两份SDK控制节点。

保留原入口：pico_receiver_node、target_generator_node、rm65_canfd_high_follow_node、single_session_hil_monitor，以及teleop_high_follow.launch.py。原launch的allow_motion默认true但必须通过确认字符串；pico_o7 launch默认false。完整正式联动命令见工作空间README.md。

配置config/high_follow.yaml；launch可覆盖部分YAML，实际参数与消息见工作空间docs/INTERFACES.md。CANFD软件日志在~/teleop_ws/canfd_logs，物理总线时序不由该日志直接证明。Modbus自动初始化封装在rm_api2_motion_adapter.py，仅由独立O7 Worker调用，不使用ARM状态门槛或控制定时器调度锁，不改变Remote IK/CANFD算法。

旧HIL模块同时承担SingleSessionHil生命周期和诊断兼容接口，仍有当前测试引用，保留；不是正式控制目标生成路径。详细架构与验收见docs/ARCHITECTURE.md和docs/validation/v1.0.0/。原包README已归档。
