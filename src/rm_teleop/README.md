# PICO → RM65 CANFD 高跟随（单链路版）

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

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py
```

默认运动输出锁定。真机解锁：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED
```

## 速度参数

高跟随控制频率固定为200Hz。以下参数只控制目标变化速度和手部位移比例：

```text
position_scale             手位移到机械臂位移的比例，默认0.25
translation_speed_mps      法兰最大平移速度，默认0.05 m/s
rotation_speed_deg_s       法兰最大旋转速度，默认12 deg/s
```

纯平移测试：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  translation_speed_mps:=0.03 \
  rotation_speed_deg_s:=0.0
```

纯旋转测试：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  translation_speed_mps:=0.0 \
  rotation_speed_deg_s:=3.0
```

完整低速测试：

```bash
ros2 launch rm_teleop teleop_high_follow.launch.py \
  allow_motion:=true \
  motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED \
  position_scale:=0.25 \
  translation_speed_mps:=0.05 \
  rotation_speed_deg_s:=5.0

ros2 launch rm_teleop teleop_high_follow.launch.py   allow_motion:=true   motion_confirmation:=RM65_CANFD_HIGH_FOLLOW_CONFIRMED   position_scale:=0.75   translation_speed_mps:=0.08   rotation_speed_deg_s:=10.0

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

完整替换源码后清理旧构建结果：

```bash
source /opt/ros/humble/setup.bash
cd ~/teleop_ws

rm -rf \
  build/rm_teleop \
  build/pico_rm65_teleop_interfaces \
  install/rm_teleop \
  install/pico_rm65_teleop_interfaces

colcon build \
  --symlink-install \
  --packages-select \
  pico_rm65_teleop_interfaces \
  rm_teleop

source ~/teleop_ws/install/setup.bash
```

## 启动前检查

```bash
ros2 pkg prefix rm_teleop
ros2 launch rm_teleop teleop_high_follow.launch.py --show-args
```

启动时手部遥操作应保持失能。等待RM65反馈正常后，再产生一次新的启用上升沿。
