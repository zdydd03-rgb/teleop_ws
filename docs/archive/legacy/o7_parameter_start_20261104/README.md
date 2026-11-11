# 手部参数直接启动：离线回归

2026-11-04。未连接机器人、未执行硬件写入。

当前语义：`enable_o7_retargeting=false` 不启动手部控制器；`true` 自动等待安全连接/联动 ACTIVE，随后仅消费新鲜帧。删除手部 enable/disable 服务、脚本子命令和 CLI 执行确认标志。保留可选的只读状态查询。
追踪失效、ARM 安全异常、SDK 异常/超耗时、退出与 handle 生命周期保护保留；首次启动后的安全禁用需重启恢复。

结果：

- `hand_tests.log`：原手部 42 项离线/mock 回归通过。
- `parameter_tests.log`：21 项通过，其中新增 9 项参数启动测试，加上原门控/Worker/报告 12 项。
- `arm_tests.log`：原 rm_teleop 164 项 pytest 通过。
- `build.log`：rm_teleop colcon 构建通过。
- `launch_args.log`：只解析 launch 参数，没有启动节点。
- `integrity.json`：9 个手部源码/CLI/文档文件的前后哈希和备份位置；原 ARM 来源哈希校验通过，设备配置不变。

同时完成 `bash -n`、修改后 Python 内存 compile、ROS 类导入及 `git diff --check`，均通过。
原始手部文件备份在 `/home/ai/pico_o7_retargeting/outputs/parameter_start_20261104/source_before/`。
当前 ARM 200Hz 控制、Remote IK、UDP、故障锁存、安全停止及 API2 创建销毁源码/配置未修改。

现场命令见 [参数启动与联动指南](../PICO_O7_COMBINED_FIELD_GUIDE.md)。真机效果、停止路径和本版联动仍待操作者逐项验收。
