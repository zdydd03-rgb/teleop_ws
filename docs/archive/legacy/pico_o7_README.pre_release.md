# PICO → O7 Right（teleop_ws 正式入口）

本包把原参考项目已验证的接收、somehand 重定向、按名七轴映射和串行 Worker 迁入 teleop_ws。
正常运行不导入或启动旧参考目录中的代码、配置、模型或 Python 环境。

机械臂继续由 rm_teleop 原 Remote IK、200Hz CANFD 和 UDP 反馈实现；本包只增加接收器/节点包装，不修改机械臂主线。
O7 复用该节点唯一 Adapter/RoboticArm/API2 handle，4Hz/每轴限步16，latest-value mailbox，不读 O7、不自动重发。

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pico_rm65_teleop_interfaces rm_teleop pico_o7 --symlink-install
source install/setup.bash

# 仅手部；启动参数直接允许执行，不再需要启用服务。
ros2 launch pico_o7 teleop.launch.py allow_motion:=false enable_o7_retargeting:=true
```

臂手联动：结束旧实例后执行，保持原 ARM 运动确认与 PICO 门控。

```bash
ros2 launch pico_o7 teleop.launch.py \
  allow_motion:=true motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  enable_o7_retargeting:=true
```

`allow_motion=false/true` 和 `enable_o7_retargeting=false/true` 独立选择仅手、仅臂、联动或都不执行。
本 launch 两项默认 false。原 rm_teleop launch 的默认值和控制行为未改变。不得同时启动两个 launch 或额外 SDK 客户端。

## 追踪丢失后的恢复

手移出 PICO 画面会清空旧输入和待发解。联动中普通退出 ACTIVE 或短暂 CANFD 停发使 O7 暂停，
当 ARM 原状态机恢复健康 ACTIVE 后，手部重新接受新鲜帧，不需要重启或服务确认。
旧求解结果被 epoch 丢弃，最后提交的目标保留为限步参考，不回零、不回放丢失前目标。

FAULT_LATCHED、UDP反馈失效、SDK异常/超750ms、真实断线仍停写，不自动清错。
返回码1只保留诊断值，不证明动作成功或失败，也不触发重发。
恢复正常追踪不能撤回已进入SDK的阻塞调用。停止下发不等于电机立即停止或自动张开。

## 配置、环境与日志

- `config/o7_device_linear.json`：4Hz、限步16、0.25s新鲜度、750ms调用保护、initial_hold全255。
- `config/o7_right_vectors.yaml`、`models/`、`vendor/somehand/`：迁移的原模型与求解参数，未调重定向。
- `o7_device_mapping_path`：可选配置副本绝对路径，重启生效。
- `o7_solver_python`：默认工作空间 `.o7_solver_venv/bin/python`。ROS使用系统Python3.10，求解隔离为Python3.11。
- `tools/setup_o7_solver_env.py --python /path/to/python3.11 --install`：按 requirements.lock 重建依赖；联网安装由用户显式执行。
- ARM CSV 默认 `~/teleop_ws/canfd_logs/`；设置 `ROS_LOG_DIR="$PWD/log/pico_o7"` 可统一 ROS 日志。
- 原被动监测器继续使用；`tools/pico_o7_field_review.py` 分析手部事件，保留原始 CSV/JSON。
- 可选只读查询：`ros2 run pico_o7 o7_session_tools hand-status`。

所有项目源代码、模型、vendor、配置与测试都在本工作空间，Python/ROS/RealMan SDK 仍是系统依赖。
第一次迁移离线复制了已安装的 solver 依赖到本地 venv；运行不依赖复制来源。环境不纳入Git。

历史单次标定/读取/独立A/B模块保留用于显式回归，不属于正常跟随路径。独立客户端只能在正常会话完全退出后使用。

## 离线检查

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/rm_teleop/test src/pico_o7/test tools -q
env -u PYTHONPATH PYTHONPATH="$PWD/src/pico_o7" .o7_solver_venv/bin/python \
  -m pico_o7.retarget_dry_run --input /path/to/capture.jsonl --output concurrency_logs/offline_check
```

详见工作空间 `docs/PICO_O7_COMBINED_FIELD_GUIDE.md` 和迁移报告。新版本尚待现场验证追踪移出/移回的自动恢复，不把 mock 或历史真机记录当作本版通过。
