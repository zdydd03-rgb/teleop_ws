# O7 Modbus — V1.0.0

## 链路与数据

RM65 Tool RS485 → RS485-CAN转接器 → LinkerHand O7。转接器厂商、型号、固件和内部应答机制未确认，不将其他型号手的协议当成O7依据。

`rm_api2_motion_adapter.py::write_o7_hil_positions`校验七个0～255整数，保留输入顺序：

- port=1，baudrate=115200，timeout=1（单位100ms）。
- device=39（0x27），address=0，num=7，多保持寄存器写入（项目FC16）。
- 每个数展开为`[value >> 8, value & 255]`，即14个字节的uint16大端数据；高字节在当前范围内为0。
- 通道依次为thumb_pitch、thumb_yaw、index_pitch、middle_pitch、ring_pitch、little_pitch、thumb_roll。
- 映射/方向/步长由config/o7_device_linear.json提供；这不是已经测量完成的全轴标定。

## 调度与初始化

单个o7_shared_hand_worker同步调用现有Adapter，不创建SDK Handle，不读回O7、不自动补发失败命令。Latest-value Mailbox覆盖旧目标。write_hz=10.0是提交起点限频，max_age_sec=0.25，max_call_ms=750，max_step=16。

自动初始化复用唯一Handle：`rm_set_modbus_mode(1,115200,1)`；返回0后调用`rm_get_tool_rs485_mode()`，要求return_code=0、mode=1、baudrate=115200、timeout=1。配置可能持久化；本入口的正式工作配置固定为该值，不执行诊断A/B切换。

启动、重新进入控制会话、Handle变化自动初始化，不依赖enable Service。Worker只检查共享SDK连接与节点退出，不再依据ARM ACTIVE、反馈、故障或CANFD间隔决定初始化/写入。删除了原控制定时器调度门；不新增全局SDK锁，不改变ARM定时器。成功后等待新鲜PICO帧，旧epoch目标不可重放。

## 恢复与锁存

写入返回任何非零码、native异常或耗时超过750ms，暂停O7并进入恢复。返回1不是成功。初始化失败同样进入恢复。

恢复按1、2、4秒退避，60秒窗口最多三次实际恢复初始化（成功也计预算）。首次正常初始化不占预算；共享连接不可用而未实际初始化时不计尝试。预算耗尽且仍需恢复则fault_latched=true、reason=modbus_recovery_limit_latched，不自动解锁。

ARM故障既不被O7恢复解除，也不触发O7暂停/锁存；显式disable不会因Handle变化复活。普通关键点暂时缺失只清空数据，不等于Modbus故障。ACTIVE不再阻止O7恢复；系统不为O7擅自停臂。SDK native内部并发性能尚待本修复后的真机验证。

## 状态与证据

status服务和事件包含modbus_initialized、init_count、init_reason、init_result、recovering、fault_latched、next_init_delay_sec、recovery_attempts_in_window、last_code、last_duration_ms。recent_write_hz和recent_acknowledged_hz按最近5秒计数/5统计，启动不足5秒会偏低；返回0只代表SDK确认。

事件：O7_MODBUS_INIT_START/SUCCESS/FAILED、O7_HAND_WRITE_START/FINISH/CANCELLED、O7_HAND_RESUME/PAUSE/SAFETY_STOP及周期STATUS。

[本次HIL](validation/v1.0.0/HIL_REPORT.md)复核477/477返回0，平均64.632ms、P95 76.342ms、最大103.037ms，连续起点频率9.443Hz。首次初始化返回-2，自动重试成功；首次失败根因未知，不能声称所有故障均可恢复。

本文件描述当前解耦实现，旧V1验收证据仍保留历史语义，详见[修复说明](O7_DECOUPLING.md)。
