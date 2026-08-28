# 替换说明

压缩包内只有两个ROS 2源码包，目录结构为：

```text
src/
  pico_rm65_teleop_interfaces/
  rm_teleop/
```

建议先备份旧源码：

```bash
cd ~/teleop_ws/src
mv rm_teleop rm_teleop_old
mv pico_rm65_teleop_interfaces pico_rm65_teleop_interfaces_old
```

然后把本压缩包中的 `src/rm_teleop` 和 `src/pico_rm65_teleop_interfaces` 放入 `~/teleop_ws/src/`。

旧版的慢速节点、dry-run节点、独立状态节点及重复launch不会保留。唯一真机启动命令见 `src/rm_teleop/README.md`。
