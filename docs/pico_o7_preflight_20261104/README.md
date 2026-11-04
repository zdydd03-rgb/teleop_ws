# 2026-11-04 联动前离线证据

本目录不是现场联动验收结果。没有连接机器人或触发真实硬件写入。

- `validation.json`：本轮检查结果汇总。
- `integrity.json`：原 ARM/SDK 来源哈希仍一致；两处手部改动的前后哈希及外部备份路径。
- `device_config.json`：未改动的 4 Hz、七轴限步 16 配置快照。
- `hand_tests.log`：原有 42 项手部离线/mock 测试通过。首次受沙箱限制无法打开项目锁文件，申请文件系统权限后重跑通过；没有真实 SDK 连接。
- `combined_tests.log`：新增门控/Worker 9 项、只读报告 3 项，共 12 项通过。
- `launch_args.log`、`launch_matrix.log`：只解析参数；四种 ARM/O7 开关组合与旧 HIL 默认关闭均通过，没有运行节点。
- `build.log`、`colcon_test.log`、`colcon_result.log`：rm_teleop 构建成功；164 tests、0 errors、0 failures、0 skipped。另直接运行 pytest 也为 164 passed。
- `offline_result.json`、`offline_dry_run/`：2430 帧中 2260 有效，全部求解，0 异常、0 SDK 调用。dry-run 原脚本备注仍含旧 2 Hz 字样，实际使用当前 4 Hz 配置；模拟提交数量不等于真机写入吞吐。

modified Python 通过内存 compile 静态检查，`bash -n o7_field.sh`、依赖 pip check、ROS 类导入、原始日志哈希校验和 `git diff --check` 均通过。没有修改 ARM 控制代码、连接/销毁逻辑或配置默认值。

现场流程见 [操作指南](../PICO_O7_COMBINED_FIELD_GUIDE.md)。锁臂复核、ARM 基线、真实臂手联动和各停止路径仍由操作者逐步执行并提供现场观察。
