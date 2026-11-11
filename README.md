# RM65 + O7 Teleoperation V1.0.0

基于PICO的RM65-B机械臂与LinkerHand O7七通道灵巧手一体化ROS2遥操作工程。机械臂与灵巧手共享**唯一睿尔曼SDK Handle**。

## 功能与验证范围

已实现：单socket PICO接收、机械臂坐标映射/滤波/限速、Remote IK、200Hz CANFD高跟随与原安全状态机；21关键点、somehand重定向、七通道映射、Latest-value Mailbox、单O7 Worker、10Hz写入上限、自动Modbus初始化/配置验证、退避恢复和故障锁存。

离线覆盖见[测试记录](docs/validation/v1.0.0/OFFLINE_TESTS.txt)。2026-11-11真实联动约50.5秒：O7 477次全部返回0、平均64.63ms、约9.44Hz；CANFD 10,112次和Remote IK均返回0，软件平均周期5ms。首次初始化失败后自动恢复成功。详细数据、校验和与限制见[HIL报告](docs/validation/v1.0.0/HIL_REPORT.md)。未将未覆盖故障或物理精度宣称为已验收。

## 架构与目录

```text
PICO UDP :5005 → pico_hand_receiver_node
  ├ /pico/teleop_state → target_generator_node → Remote IK → CANFD 200Hz → RM65
  └ /pico/right_hand_raw → somehand隔离求解器 → O7 Mapper → Latest-value Mailbox
                                                   → 单Worker → Modbus ≤10Hz → O7
                                机械臂和O7复用shared_session_node唯一SDK Handle
```

```text
src/pico_rm65_teleop_interfaces/  三种ROS消息
src/rm_teleop/                   ARM基础节点、Adapter、控制算法及测试
src/pico_o7/                     共享节点、O7求解/映射、配置、模型及测试
tools/                          现场入口、离线检查、环境准备及测试
docs/                           当前架构、接口、Modbus和V1验收文档
docs/archive/legacy/            历史说明和迁移记录（非当前操作依据）
concurrency_logs/               本次原始rosbag及运行锁（不入Git）
canfd_logs/                     本次原始CANFD CSV（不入Git）
archive/                       其他历史原始日志（不入Git，不删除标定/示教数据）
build/ install/ log/             可重建产物和构建日志（不入Git）
.o7_solver_venv/                本地求解环境（不入Git）
```

[开发架构](docs/ARCHITECTURE.md) · [接口](docs/INTERFACES.md) · [O7 Modbus](docs/O7_MODBUS.md) · [证据清单](docs/validation/v1.0.0/TEST_MANIFEST.md) · [整理审计](docs/validation/v1.0.0/RELEASE_AUDIT.md)

## 环境与依赖

验收环境Ubuntu22.04、ROS2 Humble、ROS系统Python3.10、隔离求解Python3.11；API2 C API v1.1.5为现场确认版本。本机SDK位于/home/ai/RM_API2/Python，为外部依赖，不复制二进制进仓库。TRIPLE线程模式不变。控制器192.168.125.19:8080，PC反馈192.168.125.100:8089，PICO UDP监听5005；实际网络环境应匹配配置。

ROS依赖以各包package.xml为准；三个项目包及ROS基础消息、launch/rclpy、rosidl等需预先安装。somehand是vendor中的0.3.0标识源码子集，不是完整上游安装包；来源和本地修改见src/pico_o7/VENDOR_NOTES.md及sources_manifest.json。模型和标定资料保留原状。

求解依赖精确列表见src/pico_o7/requirements.lock。本机现有环境保留；全新环境可使用现有脚本（会创建环境/联网安装，不是启动步骤）：

```bash
python3 tools/setup_o7_solver_env.py --python /path/to/python3.11 --install
```

脚本拒绝覆盖现有环境。SDK供应商安装步骤、固件版本和硬件接线的完整可复现安装包不包含在仓库中，需现场保留。

## 编译与离线验证

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pico_rm65_teleop_interfaces rm_teleop pico_o7 --symlink-install
source install/setup.bash
/usr/bin/python3 -m pytest src/rm_teleop/test src/pico_o7/test tools -q
```

求解器离线重放需要已有PICO JSONL/ZIP数据：
```bash
env -u PYTHONPATH -u PYTHONHOME PYTHONPATH="$PWD/src/pico_o7" \
  .o7_solver_venv/bin/python -m pico_o7.retarget_dry_run \
  --input /path/to/capture.jsonl --output concurrency_logs/offline_check
```

## 正式启动与使用

确认机械臂和灵巧手安全空间、PICO设备就绪，且没有其他SDK客户端/旧节点占用控制器。启动时先不使能PICO运动。

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pico_o7 teleop.launch.py \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  enable_o7_retargeting:=true
```

**Modbus初始化及可恢复通信故障由系统自动处理，无需手动初始化或调用enable Service。** 等待O7_MODBUS_INIT_SUCCESS，按现有PICO使能/跟踪流程操作机械臂；O7独立接收就绪后的新鲜手部数据，不等待ARM ACTIVE。10Hz是上限，不强制补发旧目标。

正常结束：通过PICO原有操作解除遥操作使能，观察机械臂控制退出ACTIVE，再在launch终端Ctrl+C（O7仍可独立跟手直到节点退出或显式disable），等待各进程清理退出。停止目标提交不保证手指立即停止/自动张开；硬件急停沿用现场规范。

可选锁臂跟手：`bash tools/o7_field.sh hand-start`；联动快捷入口：`bash tools/o7_field.sh combined-start`。两者也会连接设备，不可并发启动。

## 状态与记录

```bash
ros2 service call /rm65/o7_hand/status std_srvs/srv/Trigger '{}'
ros2 topic echo /rm65/o7_modbus_hil/event
ros2 topic echo /rm65/canfd_control_state
ros2 topic echo /rm65/status
```

状态包含初始化次数/原因/结果、恢复/锁存、最近调用耗时/返回码、最近5秒调用和成功确认频率。enable/disable服务仅为可选维护，见接口文档；锁存不会被enable绕过。

ARM CSV默认在~/teleop_ws/canfd_logs；ROS默认日志在~/.ros/log，o7_field.sh启动时设置到工作区log/pico_o7。可在另一已source终端录包（不创建SDK连接）：

```bash
ros2 bag record -o "concurrency_logs/o7_combined_10hz_$(date +%Y%m%d_%H%M%S)" \
  /pico/right_hand_raw /rm65/o7_modbus_hil/event /rm65/canfd_control_state /rm65/status
```

## 已知限制与版本

- 尚未实机覆盖：运动期间写入异常恢复、停止后再次进入ACTIVE完整闭环、长时稳定性、O7实际关节反馈和运动误差。
- 首次初始化可能返回-2，当前自动恢复有效但首次失败根因未知。
- O7已与ARM ACTIVE/CANFD间隔/故障状态解耦；预算耗尽或O7本地严重故障仍锁存。Modbus初始化可与CANFD并发，native内部资源影响需修复后真机验收。
- native调用可能阻塞，无法强制撤回；软件CANFD周期不等于物理CAN周期；SDK返回0不等于测量的物理运动成功。
- 保留诊断兼容入口；旧HIL开关在正式共享模式必须关闭，独立SDK工具禁止与正式会话并行。

软件版本字段均为1.0.0。发布提交/标签是否已冻结以[整理审计](docs/validation/v1.0.0/RELEASE_AUDIT.md)及Git实际状态为准，不将文档版本名称等同于已打标签。

当前含[V1.0.0后O7解耦修复](docs/O7_DECOUPLING.md)，旧HIL报告不代表此修复已经真机验收。
