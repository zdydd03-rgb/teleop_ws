# PICO → RM65 CANFD 高跟随（单链路版）

工作空间总入口及最新 O7 证据见 [根 README](../../README.md)，下一阶段见 [PICO-O7 交接](../../docs/PICO_O7_TELEOP_HANDOFF.md)。本页保留机械臂坐标映射和运行说明，不代表 PICO 手指映射已经实现。

O7 使用高跟随节点内的同一 Adapter/handle。三种 HIL 默认关闭、共用进程一次接受机会；自动模式 10 Hz 是目标生成，最新共享 Session 实际写入约 4.032 Hz。位置非零返回表示 SDK 未确认，不能等同于未执行或成功。不要另开 O7 API2 Client。

本包只保留一条真机控制链路：

```text
PICO UDP
  → /pico/teleop_state
  → 目标生成器
  → /rm_teleop/flange_target
  → Remote IK
  → rm_movej_canfd(follow=true, trajectory_mode=0)
```

已移除慢速CANFD、独立IK dry-run、独立状态节点、重复launch和重复配置。RM65状态、法兰位姿和诊断由高跟随节点直接发布。

## 固定坐标关系

平移：

```text
PICO +X（右）  → RM65 Base +Z（现实向右）
PICO +Y（上）  → RM65 Base -Y（现实向上）
PICO +Z（前）  → RM65 Base -X（现实向前）
```

旋转采用相同的现实轴含义，但通过相对四元数的向量部分进行直接轴映射：

```text
PICO +Rx → RM65 Base -Rz
PICO +Ry → RM65 Base +Ry
PICO +Rz → RM65 Base +Rx
```

代码不会再对姿态做第二次X/Z交换，也不会使用 `B * R * B^T` 造成旋转方向反转。

## 唯一启动文件

使用已验证的参数启动真机：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED
```

launch 默认允许运动，但仍需要上述确认字符串；不提供确认字符串时，控制节点会拒绝启动。需要保持运动输出锁定时，显式运行：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py allow_motion:=false
```

## 速度参数

高跟随控制频率固定为200Hz。以下参数只控制目标变化速度和手部位移比例：

```text
position_scale             手位移到机械臂位移的比例，默认0.75
translation_speed_mps      法兰最大平移速度，默认0.08 m/s
rotation_speed_deg_s       法兰最大旋转速度，默认12 deg/s
```

纯平移测试：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  translation_speed_mps:=0.03 \
  rotation_speed_deg_s:=0.0
```

纯旋转测试：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  translation_speed_mps:=0.0 \
  rotation_speed_deg_s:=3.0
```

完整低速测试：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  position_scale:=0.25 \
  translation_speed_mps:=0.05 \
  rotation_speed_deg_s:=5.0
```

## 主要话题

```text
/pico/teleop_state
/rm_teleop/flange_target
/rm65/current_flange_pose
/rm65/controller_waypoint_pose
/rm65/joint_states
/rm65/canfd_control_state
/rm65/status
```

## 构建

例行增量构建不需要删除 build/install：

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pico_rm65_teleop_interfaces rm_teleop
source install/setup.bash
python3 -m pytest src/rm_teleop/test -q
```

## 启动前检查

```bash
ros2 pkg prefix rm_teleop
ros2 launch rm_teleop teleop_high_follow.launch.py --show-args
```

启动时手部遥操作应保持失能。等待RM65反馈正常后，再产生一次新的启用上升沿。
