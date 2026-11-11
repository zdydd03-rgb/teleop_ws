# V1.0.0 证据与复核清单

## 原始证据（必须保留，不提交大型数据）

- `concurrency_logs/o7_combined_10hz_20261111_172614/metadata.yaml`
- `concurrency_logs/o7_combined_10hz_20261111_172614/o7_combined_10hz_20261111_172614_0.db3`
- `canfd_logs/rm65_canfd_high_follow_20261111_172612.csv`
- `docs/validation/v1.0.0/raw_logs/python3_29026_1794389172660.log`：控制进程，含首次初始化-2和自动恢复。
- raw_logs中同次PID29024接收器、PID29028目标生成、PID29245录包及launch.log。

raw_logs从本次~/.ros/log复制，外部原文件没有更改；launch.log来自2026-11-11-17-26-12-398316-ai-Legion-29023。源码范围不依赖外部诊断项目。

**所有上述文件大小与SHA256逐项见HIL_METRICS.json的files字段。** bag时间范围1794389175256105013～1794389255498358653 ns。测试环境和统计口径见HIL_REPORT.md。

这些原始文件本地保留且被.gitignore排除；Git clone不会自带原始证据。移交版本时必须另行备份并按SHA256核对，不能以本报告替代原始记录。

## 离线证据

- AUDIT_BEFORE.json：整理前分支、HEAD、git状态、文件分类及大小。
- RUNTIME_SHA256.json：整理前运行源码/配置/模型候选快照；整理结束逐文件确认未改变。
- CLEANUP_MANIFEST.json / DELETED_FILES.md：逐项归档和删除路径与原因。
- PRE_CLEANUP_TESTS.txt：完整测试初跑8失败、244通过、2线程告警；原231包内测试未出现这些旧工具夹具问题。
- OFFLINE_TESTS.txt：修复夹具后的全量包内+tools测试结果，未删除失败测试。
- BUILD.txt：三个包colcon构建。
- LAUNCH_ARGS.txt：两份launch通过--show-args仅解析，不启动节点。
- STATIC_CHECKS.json：Python AST、包依赖声明及运行源码完整性检查。
- SOLVER_PIP_CHECK.txt / SOLVER_REPLAY.txt：现有隔离环境和录制fixture的离线求解验证。

原始运行没有源码hash/发布tag，不能伪称HIL运行于已提交v1.0.0。分支/冻结状态见RELEASE_AUDIT.md。
