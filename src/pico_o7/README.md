# pico_o7 — V1.0.0

正式PICO接收扩展与O7重定向包，复用rm_teleop唯一SDK Handle。配置write_hz=10.0、max_age_sec=0.25、max_call_ms=750、七通道max_step=16。somehand在Python3.11隔离进程运行，ROS在系统Python3.10运行。

正式入口为`ros2 launch pico_o7 teleop.launch.py allow_motion:=true motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED enable_o7_retargeting:=true`。启动自动初始化Modbus并查询验证，无需enable服务。通信失败有退避和60秒/三次恢复预算；不再受ARM ACTIVE/CANFD间隔/ARM故障约束，初始化与CANFD可并发，恢复后必须接收新PICO帧。严重故障/预算耗尽锁存。

完整编译、环境、操作与安全限制见工作空间README.md及docs/ARCHITECTURE.md、INTERFACES.md、O7_MODBUS.md；真实验收见docs/validation/v1.0.0/HIL_REPORT.md。旧README已归档，不以旧4Hz/返回码不控制准入的说明作为正式行为。

setup.py保留pico_hand_receiver_node、shared_session_node、o7_session_tools。所有历史单次/读诊断工具和兼容服务仍受安全门槛限制；独立客户端禁止与正式会话并发。模型、vendor许可、标定和requirements.lock均保留。
