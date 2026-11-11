> 发布更新：用户已明确授权将当前工作树提交为本地正式版本，包含现有功能修改、O7解耦修复、整理归档和当前文件删除。计划提交说明为 `release: RM65 O7 teleoperation v1.0.0 baseline`，附注标签为 `v1.0.0`，标签说明为 `RM65 + O7 Teleoperation V1.0.0`。下文“尚未冻结/需人工确认”是整理时的历史状态，不再是本次提交的阻塞项；最终提交以Git标签指向为准。不执行远程push。
> 最新解耦修复回归为253项通过、三包构建成功；原V1.0.0 HIL仅代表修复前运行，native并发初始化仍待新实机验证。见../o7_decoupling/及../../O7_DECOUPLING.md。

# V1.0.0 整理与发布状态

## 结果

工程内容已按V1.0.0整理；**本地发布提交和v1.0.0标签尚未冻结**。分支`cleanup/remove-obsolete-code`，当前HEAD `578f4e0536974af0451c249fabef5549c7bc83d0`，没有创建提交、标签或push；工作树不干净。

原因：整理开始即存在多处未提交功能修改（含机械臂旋转/输入稳定参数及O7恢复）和用户既有删除`一、参数生效关系.md`。原README还带用户追加启动命令。虽然保留的功能均通过当前离线回归，不能把未确认归属的历史改动和文档删除擅自纳入发布提交。初始路径列表和Git状态完整保存在AUDIT_BEFORE.json；用户应确认哪些历史改动纳入版本后再冻结，不要求重新初始化Git或回退修改。

本轮改动是根/包README、VENDOR_NOTES说明、docs现行规范、历史资料归档、.gitignore及两份tools测试夹具。运行源码、配置、模型、标定参数和Launch未改。92个运行/资源文件逐项SHA256一致（排除文档、测试、Python/pytest缓存），见STATIC_CHECKS.json及RUNTIME_SHA256.json。

## 文件审计与处置

- FILE_CLASSIFICATION.md逐项列出src和tools实际文件及保留原因。
- CLEANUP_MANIFEST.json记录49项归档移动、120个缓存文件和本轮一次性脚本删除；DELETED_FILES.md逐文件解释删除原因。没有删除测试或原始HIL日志。
- 历史docs、旧README、开发交接移动到docs/archive/legacy；历史canfd/concurrency实验移到archive/对应目录。全部仍在工作空间；旧绝对路径的迁移映射在manifest中。
- 明确保留tools/o7_field.sh、setup_o7_solver_env.py、pico_o7_field_review.py及所有tools测试；standalone_write/write_latency_report/旧HIL/可视化仍有测试或独立诊断用途，不满足删除的五项条件，不武断删除。
- build/install、现有求解venv和原构建日志保留；未为整理清空可工作安装。
- 模型、calibration、vendor许可证及PICO录制fixture均保留，不把标定或训练资料作为日志。

## 验证

整理前完整测试：244通过、8失败、2线程告警（工具测试用旧构造夹具、旧返回码/服务断言）。未通过删除测试解决。修复两份tools测试夹具以使用真实HandController构造+mock初始化，并断言当前ARM故障锁存和成功确认语义；运行源码未改。

最终：**252通过，0失败，0跳过，无pytest告警**。3包构建成功，无构建告警；两份launch --show-args解析通过，不启动节点。Python AST共84个src/tools文件通过；包manifest的版本/依赖已解析并由构建验证。solver pip check通过，1帧真实录制fixture离线求解成功（hardware_sdk_calls=0，不是新增真机试验）。证据见同目录TXT/JSON。

HIL核算保留477次O7全部确认成功、平均64.632ms和9.443Hz；CANFD发送10112次，平均4.999997ms。首次初始化-2后恢复成功的原日志已保存并记录哈希，见HIL_REPORT.md/HIL_METRICS.json。物理总线周期、O7实际精度和全部恢复路径未验收。

## 冻结前人工检查

确认AUDIT_BEFORE中的历史变更归属及中文文档删除意图，复核当前diff并备份被忽略的验收原始数据。确认后才能选择性stage与提交`release: RM65 O7 teleoperation v1.0.0 baseline`、创建本地v1.0.0标签；不要git add全部原始日志，不自动push。此文档不声称旧HEAD已经包含本次验收功能。
