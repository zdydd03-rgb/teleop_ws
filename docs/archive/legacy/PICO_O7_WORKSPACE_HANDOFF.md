# PICO → O7 Right：teleop_ws 统一工程交接

更新：2026-11-04。所有后续开发在 `/home/ai/teleop_ws`。原参考目录保留历史记录，但不再修改，也不作为运行依赖。

## 本轮根因与修复

操作者报告：手离开PICO画面后，追踪恢复、机械臂可重新运动，但O7不再跟随。
原始日志 `python3_26594_1793782470682.log` 与机械臂CSV确认：

- O7已完成476次调用，全部原始码1，SDK异常0；平均246.861ms、最大448.472ms。
- ARM `1793782625.080616149` 进入READY/teleop_disabled，在 `1793782626.960910757` 恢复ACTIVE，相隔约1.88s。
- O7却保持 enabled=false、startup_pending=false、reason=control_not_ACTIVE_sending，新输入持续到达但不再求解/写入。
- 该轮ARM故障锁存样本为0。不能把这个普通追踪/启用中断当作需要重启的严重故障。

旧实现把普通退出ACTIVE永久禁用了。本轮只在手部Worker修复：

| 情况 | 处理 |
|---|---|
| 21点无效/过期/遮挡 | 清空输入与目标，停止新提交；恢复时使用新鲜帧 |
| 联动普通退出ACTIVE、短暂CANFD停发 | 可恢复暂停，清缓存/epoch；ARM原状态机恢复健康ACTIVE后O7自动恢复 |
| 恢复边界 | 只接受恢复时刻之后的帧；旧求解结果作废，保留最后提交目标作限步参考，不发回位命令 |
| FAULT_LATCHED、UDP反馈失效、SDK断线、异常/超750ms、关闭 | 停写，不自动清错；故障恢复仍按原安全流程 |
| Modbus非零返回 | 原值记录；不据此判断动作成功/失败，不自动重发 |

暂停/恢复/安全停止事件分别为 O7_HAND_PAUSE、O7_HAND_RESUME、O7_HAND_SAFETY_STOP。
本轮未连接机器人，自动恢复仍需现场用相同遮挡操作复验。
来源哈希和关键事件见 [根因证据](pico_o7_migration_20261104/recovery_log_evidence.json)，原控制台日志副本保留在 `concurrency_logs/pico_o7_tracking_loss_20261104/teleop_console.log`。

## 统一目录与源码索引

| 路径 | 职责 |
|---|---|
| `src/rm_teleop/rm_teleop/rm65_canfd_high_follow_node.py` | 原连接线程、Remote IK、200Hz控制、安全/FAULT/stop和handle释放，未修改 |
| `src/rm_teleop/rm_teleop/rm_api2_adapter.py` / motion adapter | 唯一RoboticArm/handle和已有O7位置编码/写入，未修改 |
| `src/pico_o7/pico_o7/shared_session_node.py` | 继承原节点；参数、手部门控、共享Worker和生命周期包装 |
| `src/pico_o7/pico_o7/pico_hand_receiver_node.py` | 在原UDP接收路径转发21点，保留ARM消息；不新增接收器 |
| `src/pico_o7/pico_o7/o7_stage/hand_input.py` | 21点有效性与原报文/序号检查接入 |
| `src/pico_o7/pico_o7/o7_stage/realtime.py` | LatestHand状态、epoch、latest mailbox、求解子进程、串行写入和暂停恢复 |
| `src/pico_o7/pico_o7/o7_stage/mapping.py` | 按模型名字排序、线性映射、范围和限步16 |
| `src/pico_o7/pico_o7/solver_process.py` / capture_io | 原somehand求解与经验坐标预处理，Python3.11隔离进程 |
| `src/pico_o7/config/` | 设备JSON和求解YAML，原参数字节保留 |
| `src/pico_o7/models/` / vendor | 原MJCF、有效限位、厂商URDF、somehand核心源码与许可证 |
| `src/pico_o7/launch/teleop.launch.py` | 当前工作空间标准ROS launch入口 |
| `src/pico_o7/test/` | 迁移的42项mock、恢复与四模式节点生命周期测试；fixture来自原始实录 |
| `tools/o7_field.sh` | 工作空间便捷入口，与ros2 launch等价，不创建额外客户端 |
| `tools/pico_o7_field_review.py` | 只读分析原monitor的手部事件/控制CSV |
| `tools/setup_o7_solver_env.py` | 工作空间Python3.11环境重建/离线复制工具 |

历史单次标定、读取、独立A/B解析/入口作为显式回归模块保留，正常跟随不调用它们。不得另开独立客户端与本launch并行。

## 架构与所有权

```text
唯一PICO UDP5005 → 增强Receiver → 原ARM消息 → 原Target generator
                          │                         ↓
                          │              原Remote IK → CANFD 200Hz
                          ↓                         │
                  /pico/right_hand_raw              │
                          ↓                         │
                  Python3.11 somehand               │
                          ↓                         │
                  latest-value mailbox              │
                          ↓                         │
                  单一串行Modbus Worker              │
                          └────────────┬────────────┘
                              同一Adapter/handle
                                    ↓
                            RM65 Tool RS485 → O7 Right
```

原 rm_teleop 连接线程拥有会话，初始化 RM_TRIPLE_MODE_E/Remote IK/UDP。手部借用同一 Adapter；Worker槽由原 SingleSessionHil 的派生 owner 管理。
不在ARM回调求解/Modbus、不加全局SDK锁、不创建第二个客户端。关闭时先停止提交并等待在途Worker，再由原节点释放handle。
无法安全强制取消原生SDK调用；Ctrl+C并非即时急停。正常先按PICO流程停臂再退出，危险用硬件急停。

## 环境与实际基线

Ubuntu22.04/ROS2 Humble，ROS侧Python3.10。现有RealMan SDK仍由原Adapter搜索系统依赖位置；没有迁移或修改厂商SDK。
求解侧Python3.11及已安装依赖离线复制到 `teleop_ws/.o7_solver_venv`，无指向原参考项目的运行链接。
CPython基础安装、ROS和SDK属于系统依赖；项目源码、模型、配置、vendor和已复制的solver第三方包均位于本工作空间。
环境受Git忽略，不把465MB依赖纳入源码提交。可用 `tools/setup_o7_solver_env.py --python /path/to/python3.11 --install` 按固定requirements重建；本轮没有下载/升级依赖。

配置基线不变：write_hz=4、max_step全16、max_age=0.25s、max_call=750ms、initial_hold全255。
initial_hold是命令基准，不是读回；现场初始保持姿态应已知一致。模型限位线性映射并非实测角度标定，thumb_yaw/roll仍待专门确认。
Tool port1、slave39、FC16 HR0..6、7个uint16大端；正常路径只写不读，不设置速度/力矩或重配端口。

## 编译、启动与采集

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pico_rm65_teleop_interfaces rm_teleop pico_o7 --symlink-install
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/pico_o7"

# 仅手部；启动即选择允许跟随，安全会话就绪后接受新帧。
ros2 launch pico_o7 teleop.launch.py allow_motion:=false enable_o7_retargeting:=true
```

结束旧实例后选择联动：

```bash
ros2 launch pico_o7 teleop.launch.py \
  allow_motion:=true motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  enable_o7_retargeting:=true
```

两参数可独立组合，默认false/false；ARM=true必须保留原token及PICO启用状态机。
没有手部enable/disable服务二次门控。可选只读查询 `ros2 run pico_o7 o7_session_tools hand-status`。
`o7_device_mapping_path` / `o7_solver_python`可显式覆盖；启动参数在启动时读取，不支持假定动态param set会改变缓存配置。
正常原ARM-only launch仍可使用，但不能与上述launch同时启动。

原被动monitor继续可用：

```bash
ros2 run rm_teleop single_session_hil_monitor --duration 180 \
  --output concurrency_logs/pico_o7_recovery_check
```

只订阅，不传execute选项。原摘要识别旧HIL，手部统计应使用原事件与补充只读分析器；现场详细分阶段命令见 [操作指南](PICO_O7_COMBINED_FIELD_GUIDE.md)。
ROS日志在log/pico_o7，ARM CSV默认canfd_logs；保留原始数据和操作者动作/遮挡起止观察。
重点检查移出时停写、移回及ARM ACTIVE后新帧恢复、没有旧目标回放、CANFD失败和反馈失效/FAULT是否出现。

## 验证与剩余风险

原机械臂源码/配置/消息哈希不变。238项离线/mock通过：原ARM164、迁移原测试42、工具/门控21、新恢复7、四模式节点生命周期4。
三份录制共2430帧，2260有效帧全部求解；与迁移前逐帧关节角/目标比较最大差0，SDK调用0。
新包安装入口和四种launch参数解析通过；详细构建/colcon和完整验证结果见 [迁移报告](PICO_O7_MIGRATION_REPORT.md)。

尚未验证：本次恢复修复的真机效果、完整七轴标定、长期运行、所有通信故障恢复。SDK返回1与动作不一致、实际约4Hz及共享SDK内部并发仍是已知边界。
不要为解决追踪恢复而放宽ARM安全门控或清除FAULT；不要以平均调用耗时、mock或日志无错误替代现场动作验收。

## NEW_SESSION_START_HERE

项目位于~/teleop_ws，包含rm_teleop、pico_rm65_teleop_interfaces、pico_o7三个ROS2包。后续只在此工作空间开发；原外部PICO-O7目录已冻结，仅作历史参考。
RM65-B原200Hz高跟随、Remote IK、UDP反馈、FAULT_LATCHED和stop保持原源码。pico_o7包装接收器/高跟随节点，唯一UDP5005同时转发原机械臂消息与右手21点。独立Python3.11求解进程使用同工作空间vendor somehand与原模型，经按名七轴映射、latest mailbox、单串行Modbus Worker复用ARM唯一API2 Adapter/handle。正常路径不读O7、不重配RS485，不新增客户端/全局SDK锁。
当前基线write_hz=4、七轴限步16、新鲜度0.25s、调用保护750ms。SDK返回码只记录，1不代表实际未执行也不转换成成功；设备动作由现场观察。锁臂跟手已经现场跑通，明显顿挫仍在，暂不优化10Hz。
刚定位一轮真实故障：476次写入后手移出画面，ARM READY约1.88s后恢复ACTIVE，但旧手部逻辑因control_not_ACTIVE_sending永久禁用。本轮修复为可恢复暂停：清空旧缓存/epoch，ARM原状态机恢复健康ACTIVE后只消费恢复后新帧，保留最后命令作为限步参考。故障锁存、反馈失效、真实断线及SDK异常仍停写，不自动清错。新版恢复需操作者复验，不能宣称已经真机通过。
运行入口是ros2 launch pico_o7 teleop.launch.py。allow_motion与enable_o7_retargeting独立选择ARM/O7，默认false；ARM=true保留原motion_confirmation token和PICO门控。手部true直接选择运行，不再调用enable/disable服务。求解环境为工作空间.o7_solver_venv，不要激活它运行ROS。源码索引、环境、命令、证据和风险见本文及PICO_O7_COMBINED_FIELD_GUIDE.md。
开始前先查Git、实际代码和迁移报告，不修改旧目录。下一步由用户先锁臂复核，再联动移出/移回追踪，返回日志与动作观察；核对新提交、旧目标丢弃、ARM发送时序、UDP和故障状态。不要自动连接硬件或绕过安全门控。
