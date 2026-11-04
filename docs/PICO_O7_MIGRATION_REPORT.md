# PICO-O7 统一工作空间与追踪恢复修复报告

日期：2026-11-04。所有实现工作位于 teleop_ws；未继续修改旧参考项目，未连接真机或触发硬件写入。

## 整理结果

新增正式 ROS2 包 `src/pico_o7`，安装在同一工作空间。已迁入：增强接收器、共享会话包装节点、手部状态/邮箱/Worker、原somehand求解进程、经验坐标解析、具名七轴映射、配置、模型、vendor源码及许可证、必要诊断和42项原mock测试。
新增标准 launch `ros2 launch pico_o7 teleop.launch.py`；三个入口均安装可解析，不再从外部目录加载脚本。
求解依赖离线复制到工作空间 `.o7_solver_venv`（Git忽略）；其Python与NumPy/SciPy/MuJoCo/YAML实查路径已记录，不从旧venv导入。
旧参考目录保留，只读使用了来源、已有日志和录制输入；没有移动/删除其文件。

新增/更新主要文件：

- `src/pico_o7/package.xml`、setup.py/setup.cfg、pytest.ini：ROS安装、入口、资源和Python3.10测试范围。
- `src/pico_o7/launch/teleop.launch.py`：四种ARM/O7参数组合，默认全部关闭。
- `src/pico_o7/pico_o7/{shared_session_node,pico_hand_receiver_node,solver_process,paths}.py`：包内入口和资源/解释器定位。
- `src/pico_o7/pico_o7/o7_stage/`：原映射、共享Worker、计时，修复可恢复暂停语义。
- `src/pico_o7/config/`、models/、vendor/、calibration/：迁移原资源，保留来源证据和历史标定未知项。
- `src/pico_o7/test/`：原42项与新增7项恢复、4种模式实际节点生命周期mock。
- `tools/o7_field.sh`、setup_o7_solver_env.py：工作空间便捷入口和环境重建。
- `tools/test*o7*.py`：原21项检查改为引用工作空间包，并调整暂停/恢复事件预期。
- `.gitignore`：忽略本地solver环境，未忽略源码/vendor/model。
- 根README、包README、当前现场指南与新工作空间交接；旧HIL交接明确标记为历史。

## 恢复问题

根因不是SDK返回码1。操作者移手丢追踪后，原ARM正常退出ACTIVE，约1.88s后恢复；旧手部代码却把 `control_not_ACTIVE_sending` 处理成永久禁用。
真实记录包含476次SDK返回1、0 SDK异常、0故障锁存，ARM成功发送31098次、失败0，sent周期最大8.996ms、>10ms为0。只证明这轮窗口，不能推广无限并发安全。

修复将普通退出ACTIVE/短暂停发变成恢复等待。原ARM门控重新满足时，O7清空旧缓存并接受恢复后新鲜帧；旧epoch求解结果失效，最后提交命令保留作限步参考。
FAULT_LATCHED、UDP反馈失效、真实断线、SDK异常/超750ms、关闭仍停写，不自动清错。
参数true直接选择执行，无服务二次开关；没有恢复写入/回位/重试队列。

根因证据见 [原始日志核算](pico_o7_migration_20261104/recovery_log_evidence.json)。该轮原控制台和ARM CSV副本及哈希都保留在 `concurrency_logs/pico_o7_tracking_loss_20261104/`，没有重写原数据。

## 主线与单会话核对

`git diff --quiet -- src/rm_teleop/rm_teleop src/rm_teleop/config src/rm_teleop/launch src/pico_rm65_teleop_interfaces/msg` 返回0。
原ARM/SDK来源哈希校验通过，见 [arm_integrity.json](pico_o7_migration_20261104/arm_integrity.json)。只改rm_teleop的README说明，没有修改其运行源码/配置/消息。

保持单RoboticArm/handle、RM_TRIPLE_MODE_E，原连接线程拥有会话。手部独立串行Worker借用原Adapter，生命周期等待后再disconnect。
没有第二个正常API2客户端/UDP接收器、没有全局SDK大锁、没有ARM回调内Modbus。
保留4Hz、限步16、原求解/模型参数。没有优化Modbus吞吐或开发其他运动功能。

## 实际验证结果

| 检查 | 结果 |
|---|---|
| 全部直接pytest（两包+tools） | 238 passed，4.79s |
| 测试组成 | ARM164 + 迁移原42 + tools21 + 恢复7 + 节点四模式4 |
| colcon test rm_teleop | 164 tests，0 errors/failures/skipped |
| colcon test pico_o7 | 53 tests，0 errors/failures/skipped |
| 三包 colcon build --symlink-install | 成功，接口包/rm_teleop/pico_o7均完成 |
| 实录回放 | 2430帧、2260有效，全部求解，0异常、0 SDK调用 |
| 迁移前后逐帧比对 | 三份录制的关节角/目标最大差均为0 |
| 四模式launch及三个entrypoint | 解析/导入通过，没有启动节点或SDK |
| 静态/环境 | compileall、bash -n、pip check、git diff --check通过 |

具体命令和日志在 [验证目录](pico_o7_migration_20261104/validation.json)。

构建过程遇到并已解决的两项问题，原失败日志保留：

1. 旧接口包build中的普通Python生成目录阻挡symlink-install。精确移动该生成目录到 `build/pico_o7_migration_backup_20261104/interface_python_directory` 后重建成功；没有删除源码或改变消息。
2. colcon默认从新包根遍历vendor，被ROS launch-testing插件提前以Python3.10导入求解模块。新增pytest.ini限定test/后53项通过；MuJoCo仍只在隔离Python3.11执行，不向ROS环境混装依赖。

## 当前入口与后续现场确认

先读 [工作空间交接](PICO_O7_WORKSPACE_HANDOFF.md) 与 [现场步骤](PICO_O7_COMBINED_FIELD_GUIDE.md)。正常source工作空间后，使用包名launch，不再使用旧绝对路径。
两项执行参数为 allow_motion / enable_o7_retargeting；ARM=true仍需原motion_confirmation和PICO启用状态机。
启动前确保旧参考实例/SDK客户端已完全退出，不能两套并行。

下一次现场重点：先锁臂复核，再联动，移手至少3秒；核对无新提交，移回并恢复ARM ACTIVE后出现O7_HAND_RESUME且仅新帧恢复运动。
保留原始CANFD/反馈、O7调用间隔/耗时/返回码、遮挡起止及实际动作观察。现场未验证项不得用mock或返回码补结论。

本轮未提交或推送Git；原始实验数据和新包均在当前工作区。旧参考目录被冻结保留，未做无差别清理。
