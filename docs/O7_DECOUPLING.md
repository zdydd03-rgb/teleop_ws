# O7与ARM控制状态解耦修复

## 本次问题与范围

2026-11-11最新17:56:52运行日志：Modbus初始化成功，随后71次写入全部返回0。17:57:17.333附近ARM软件发送间隔为25.212ms，O7因CANFD_not_continuously_sending取消一笔写入；此后ARM仍ACTIVE而O7要求非ACTIVE初始化，导致持续停手，直到17:57:35退出ACTIVE。证据为~/.ros/log/python3_32464_1794391012015.log第183行和canfd_logs/rm65_canfd_high_follow_20261111_175652.csv第5007行。

根因是旧O7门槛把ARM调度间隙当作会话失效，并要求重新初始化；初始化又禁止ACTIVE。这不是该次写入返回非零导致，也不能由此认定机械臂运动异常。目录整理前后的运行文件哈希当时一致。

本修复替代旧“非ACTIVE初始化、ACTIVE持续发送才能跟手”的逻辑。docs/validation/v1.0.0保留原始历史验收证据，不重写为新行为的验收。

## 当前行为

- 正式launch参数不变，仍自动初始化；只等待共享SDK可用，不等待ARM ACTIVE。
- _hand_rejection只检查节点退出和共享Adapter连接；不读取ARM ACTIVE/READY、CANFD发送间隔、IK、反馈状态或ARM故障标志。
- _initialize_hand只检查共享连接与退出，并在唯一O7 Worker调用现有设置/查询封装；移除非ACTIVE门槛。
- 删除SharedO7Node的on_timer覆写和_o7_window锁，直接继承原ARM定时器。Modbus操作不持有使其等待或跳过周期的应用层调度锁。
- O7只因自身通信返回非零、调用异常/过长、输入无效/过期、显式停止、求解器关闭或共享连接不可用而停止相应提交。
- O7通信恢复仍为1/2/4秒退避、60秒最多三次恢复初始化；预算耗尽仍锁存。ARM故障不触发/解除O7锁存，也不由O7清除ARM故障。
- 每次重新就绪推进epoch、清空旧目标并等待新PICO输入。单Handle、单O7 Worker、10Hz上限、步长/过期保护和映射均不变。

**操作语义改变：机械臂停止、退出ACTIVE或发生ARM故障，不再自动停手。** 手按新鲜关键点独立运行；要停手使用原O7 disable维护接口或关闭整个节点。共享SDK不可用和节点关闭仍阻止手部通信。

## 并发边界

ARM定时器、Remote IK、CANFD算法、参数、安全保护没有修改；只删除了O7扩展节点外加的调度耦合。初始化及写入均在O7单Worker，不在ROS回调或ARM定时器中阻塞执行；不创建第二Handle或TCP客户端。

初始化现在可以与CANFD并发进入native SDK。没有厂商内部锁实现证据证明其一定无影响；离线测试只能验证应用层不等待/跳过ARM定时器，不能保证native资源竞争、控制器重置行为或真机周期性能。后续真机应检查初始化/恢复期间的CANFD间隔、返回码及手部表现，不能把旧约50.5秒HIL结果作为这条新并发路径的验证。

## 离线验证

完整包内与tools回归：253通过。新增测试确认ARM状态变化不再暂停/重新初始化O7，实际SharedNode在ACTIVE及ARM故障标志下仍可初始化，mock native初始化阻塞时继承的ARM定时器仍可调用。连接丢失、恢复预算、旧epoch丢弃、10Hz限制和有序关闭继续覆盖。

详细结果在validation/o7_decoupling/。与V1整理快照比较，运行文件仅shared_session_node.py和o7_stage/realtime.py改变；ARM源码、设备配置、模型与Launch均一致。没有启动真机或执行真实SDK通信。
