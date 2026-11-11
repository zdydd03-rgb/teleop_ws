# teleop_ws 阶段整理报告

日期：2026-11-03T17:59:14.568039+08:00。任务仅整理/核查/归档/文档交接，无真机连接和注入。

## 1. 整理前情况

分支 cleanup/remove-obsolete-code，HEAD df980a2。开始时8个已跟踪文件有历史修改，HIL源码/测试/docs和原始日志尚未跟踪；暂存区为空。本轮保留此前开发成果，不重置、不自动提交。根README原先只有项目名和一句简介，包README是原机械臂入口，三份HIL文档尚缺最终原始证据。

实际源码只有两个ROS包；根launch/config/test/scripts不存在，包内有launch/config/test，monitor由console_scripts安装。原始产物包括concurrency_logs、40份canfd CSV、5份IK CSV和34份R1～R5相关文件。build/install/log和工具/凭证目录不是本轮历史实验删除对象。

## 2. 保留模块与代码核查

保留原RM65链路、Remote IK、CANFD200Hz、UDP、FAULT_LATCHED、原stop、单RoboticArm/handle和全部接口/Mock测试。SingleSessionHil仍是共用owner和回归工具，不是废弃文件；速度单次与手动1Hz标为legacy/回归用途，自动HIL和mailbox保留。canfd_control_core同时被主线和测试依赖，未按“部分函数未走主线”删除整模块。没有废弃入口能在不改已验证行为的前提下证明必须移除，因此未删功能代码或安装入口。

本轮比较41个非Markdown源/配置/接口/测试文件SHA-256，全部与开始时一致。Python、launch、YAML、setup.py、package.xml、CMake和消息定义没有修改；不新增依赖、Client、控制器、全局锁、Modbus优化或PICO映射。

## 3. 保留的两轮数据

| 测试目录 | 实际原件 | 关键核对 |
|---|---|---|
| concurrency_logs/o7_write_only_1hz_20261103_165032/ | samples.csv、events.csv、summary.json、summary.csv + arm_control/原节点CSV | 220↔255，1Hz10秒，10次返回1，avg220.216ms/max288.218ms；两份console日志原本不存在 |
| concurrency_logs/o7_auto_10hz_20261103_172402/ | 原六项CSV/JSON/console + arm_control/原节点CSV | 实际15秒timeout；生成10.001Hz/实际4.032Hz，150生成/61写/88覆盖/1丢弃，avg250.165ms/max436.591ms |

当前仅这两个测试目录，共12个原始文件，82121130 bytes。未伪造缺失文件，也未修改原summary的NOT AUTOMATICALLY VERIFIED。现场运动来源是操作者历史确认。

两份原机械臂CSV由samples诊断中的csv_log_path精确定位并集中保留：

- `canfd_logs/rm65_canfd_high_follow_20261103_165029.csv` → `concurrency_logs/o7_write_only_1hz_20261103_165032/arm_control/rm65_canfd_high_follow_20261103_165029.csv`；复制/哈希一致后移除原路径，保留原mtime。
- `canfd_logs/rm65_canfd_high_follow_20261103_172406.csv` → `concurrency_logs/o7_auto_10hz_20261103_172402/arm_control/rm65_canfd_high_follow_20261103_172406.csv`；复制/哈希一致后移除原路径，保留原mtime。

原始诊断中的旧绝对路径刻意不重写，以上是当前路径映射。核查证据见 [RETAINED_HIL_EVIDENCE.json](RETAINED_HIL_EVIDENCE.json)，保留文件完整性见 [RETAINED_HIL_SHA256.json](RETAINED_HIL_SHA256.json)。

自动轮BASELINE含1791个waiting_for_enable，仅209次发送；不能宣称10秒ACTIVE基线。INJECTION两轮发送失败/锁存/反馈失效/异常stop均0。自动停止原因为timeout_limit，不能冒充ACTIVE失能或故障停止路径真机验证。

## 4. 外部归档与原路径清理

归档根：`/home/ai/teleop_ws_legacy_archive/20261103_174140/`。

先生成 [清点清单](WORKSPACE_CLEANUP_INVENTORY.md) 和 [操作计划](WORKSPACE_CLEANUP_MANIFEST.json)，获得沙箱外目录写入权限后复制；全部96份源文件及副本校验大小与SHA-256，再完整复核一次，之后逐项unlink对应源路径。仅对确认空的历史子目录rmdir，没有rm -rf或git clean。没有删除/移动已跟踪源码。

| 原数据类别 | 归档文件数 | 字节数 |
|---|---:|---:|
| canfd_logs/ | 38 | 1352463592 |
| concurrency_logs/ | 19 | 43056541 |
| ik_logs/ | 5 | 215957180 |
| test_results/ | 34 | 268454769 |
| 合计 | 96 | 1879932082 |

完整逐文件路径、归档目标和哈希见 [WORKSPACE_ARCHIVE_VERIFICATION.json](WORKSPACE_ARCHIVE_VERIFICATION.json)。外部另有manifest.json作为复制阶段校验快照，最终工作区移除状态以docs中的校验记录为准。

自动HIL重复tar的六个原文件与保留目录逐一SHA-256一致；将该原tar归档到外部，只保留当前目录作为规范展开数据。R1～R5旧zip和展开资料均外部保留，不做未经核对的永久去重。归档后未永久销毁任何实验数据。

## 5. 清理状态分类

- **已清理原路径**：96份已校验外部归档产物；另外2份原节点CSV搬到两轮arm_control/。ik_logs和test_results空目录移除；canfd_logs保留空根供未来原配置输出。
- **已归档**：旧双Client并发CSV、旧速度单次/读取失败/245↔255实验及重复tar、非保留轮的机械臂/IK日志、R1～R5历史结果。归档不是永久删除。
- **待确认**：`__pycache__/ik_acceptance_test.cpython-310.pyc`，14964 bytes，缺对应源码；UNKNOWN保留，未运行、未反编译、未删除。
- **未改动**：所有控制代码、测试、配置默认值、ROS接口、SDK、Git元数据、.aws/.agents/.codex、当前构建产物和忽略规则。受保护目录未读取内部内容。
- **REMOVE_CANDIDATE**：没有另行直接删除项；能够确认的历史产物均已采用可恢复归档。

## 6. 文档和配置

新增根入口README内容、WORKSPACE_CLEANUP_INVENTORY、PICO_O7_TELEOP_HANDOFF、本报告以及四份JSON审计证据（清理计划、归档校验、保留哈希、HIL复核）。更新包README、三份HIL文档、参数关系说明；旧开发交接笔记保留为legacy（文本换行规范化），没有照旧建议改IK或阈值。

修正了launch默认allow_motion=true/YAML=false、motion_stop_hold_sec launch=0.30/YAML=0.50、23个launch参数、当前position_scale=0.75/平移0.08、单次/手动/自动HIL共用一次机会，以及自动实测15秒而非60秒。HIL默认关闭的源码值未变。原“重建前rm -rf build/install”指令改为例行增量构建说明。

当前有效CLI从源码及--show-args核对；已删除的o7_stream_amplitude/旧--auto-o7只作为失效项说明，不作为启动命令。没有从entry_points删除任何入口。

## 7. 实际离线验证

```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/rm_teleop/test -q
python3 -m compileall -q src/rm_teleop/rm_teleop
git diff --check
colcon build --packages-select rm_teleop --symlink-install
colcon test --packages-select rm_teleop --event-handlers console_direct+
colcon test-result --test-result-base build/rm_teleop --verbose
```

- pytest：164 passed，测试数量未变；没有因清理删除任何测试。
- compileall、git diff --check：通过。
- colcon build：1 package finished；colcon test-result：164 tests, 0 errors, 0 failures, 0 skipped。
- 四个已安装console entry_points均可导入且callable；三个已生成接口导入通过。
- `ros2 pkg executables rm_teleop`：四个有效入口，无已删除脚本残留。
- `ROS_LOG_DIR="$PWD/log/workspace_cleanup_validation" ros2 launch rm_teleop teleop_high_follow.launch.py --show-args`：23项参数解析通过；使用工作区日志目录，仅解析不启动节点。
- monitor --help通过；HIL互斥/默认关闭由原mock测试和源码核对覆盖。
- 关联接口包CMake和msg未变，生成接口导入已核对，故本轮未重新生成接口包；根README给出全工作空间两包构建命令。
- raw samples/events与summary统计重新核对一致；保留及外部归档哈希核验通过。
- 最终复核：全部文档文件链接存在，NEW_SESSION_START_HERE为1176个中文字；外部96份与保留12份哈希均一致；安装YAML/launch与源码一致。未连接机器人、未执行真实HIL。

## 8. 当前 Git 状态（报告生成时）

下面含此前未提交改动，不表示这些Python修改由本轮整理产生；41文件哈希证明控制/配置/测试未变。没有add/commit/push或fetch，远端最新状态未查询。

```text
## cleanup/remove-obsolete-code...origin/cleanup/remove-obsolete-code
 M README.md
 M src/rm_teleop/README.md
 M src/rm_teleop/config/high_follow.yaml
 M src/rm_teleop/launch/teleop_high_follow.launch.py
 M src/rm_teleop/package.xml
 M src/rm_teleop/rm_teleop/rm65_canfd_high_follow_node.py
 M src/rm_teleop/rm_teleop/rm_api2_motion_adapter.py
 M src/rm_teleop/setup.py
 M src/rm_teleop/test/test_single_high_follow_profile.py
 M 开发交接.md
?? concurrency_logs/
?? docs/
?? src/rm_teleop/rm_teleop/hil_monitor_data.py
?? src/rm_teleop/rm_teleop/o7_auto_stream_hil.py
?? src/rm_teleop/rm_teleop/o7_single_session_hil.py
?? src/rm_teleop/rm_teleop/single_session_hil_monitor.py
?? src/rm_teleop/test/test_o7_auto_stream_hil.py
?? src/rm_teleop/test/test_o7_single_session_hil.py
?? src/rm_teleop/test/test_o7_stream_hil.py
?? 一、参数生效关系.md
```

新增HIL源码、测试、文档和保留原始日志尚未全部纳入Git。本轮不替用户自动提交，避免把大原始产物与代码混合提交。数据有本地哈希和外部历史归档，但Git忽略/未跟踪不等于已远端备份。

## 9. 下一阶段入口

从 [PICO_O7_TELEOP_HANDOFF.md](PICO_O7_TELEOP_HANDOFF.md) 开始，末尾NEW_SESSION_START_HERE可独立复制。第一项任务是调查PICO发送端SDK与现有协议是否能提供真实手指骨骼数据；当前ROS协议只有掌心/腕。先输出真实字段、单位、坐标、左右手、排列、频率与失效标志，再做映射设计和离线验证。不要新增API2 Client或把HIL循环直接当正式控制器。
