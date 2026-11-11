# 工作空间清理前清点清单

清点时间：2026-11-03T17:41:41.856879+08:00。先清点后迁移；计划归档根：`/home/ai/teleop_ws_legacy_archive/20261103_174140`。

分类：KEEP 保留；ARCHIVE 校验后外部归档再移除原路径；REMOVE_CANDIDATE 仅候选不得直接删；UNKNOWN 不处理。
本次没有永久删除项，也没有删除/移动 Git 已跟踪源码；历史压缩包和展开数据均按原件保留到外部，不做未经验证的去重。
目录 metadata 行不代表递归读取了凭证或工具内部文件；build/install/log 为当前验证保留。清单为修改前快照，后续新增整理文档见报告。

普通文件 157；计划外部归档 96 个文件；关联机械臂 CSV 集中保留 2 个。

完整 SHA-256、绝对目标路径和逐文件操作见 [JSON 清单](WORKSPACE_CLEANUP_MANIFEST.json)。

| 路径 | 类型 | 大小 bytes | Git 跟踪 | 修改时间 (+08:00) | 归属 | 分类 / 操作 | 原因 |
|---|---|---:|---|---|---|---|---|
| .gitignore | text/config | 336 | 是 | 2026-08-28T15:18:39.752537+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| README.md | .md | 48 | 是 | 2026-08-28T15:20:04.649995+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| canfd_logs/rm65_canfd_high_follow_20260716_173756.csv | .csv | 16439128 | 否 | 2026-07-16T17:39:45.571073+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260716_182303.csv | .csv | 15479682 | 否 | 2026-07-16T18:24:13.281299+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260716_183812.csv | .csv | 200176 | 否 | 2026-07-16T18:38:13.701560+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260716_183819.csv | .csv | 24586679 | 否 | 2026-07-16T18:40:11.464776+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260716_184025.csv | .csv | 31916577 | 否 | 2026-07-16T18:42:44.860164+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260716_184427.csv | .csv | 18658965 | 否 | 2026-07-16T18:45:51.499730+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260716_184652.csv | .csv | 19238505 | 否 | 2026-07-16T18:48:17.664215+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260722_172334.csv | .csv | 256559 | 否 | 2026-07-22T17:23:35.648510+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260722_172449.csv | .csv | 34727305 | 否 | 2026-07-22T17:27:27.469118+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260722_172757.csv | .csv | 25145570 | 否 | 2026-07-22T17:29:47.852268+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260722_173357.csv | .csv | 19980033 | 否 | 2026-07-22T17:35:26.585871+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260722_173710.csv | .csv | 13225753 | 否 | 2026-07-22T17:38:13.593620+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260722_173836.csv | .csv | 5908660 | 否 | 2026-07-22T17:39:04.407760+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260722_173921.csv | .csv | 23801144 | 否 | 2026-07-22T17:41:05.190471+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260722_174346.csv | .csv | 22122848 | 否 | 2026-07-22T17:45:23.238263+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260722_174640.csv | .csv | 41785268 | 否 | 2026-07-22T17:49:47.663198+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_154628.csv | .csv | 19937984 | 否 | 2026-09-18T15:48:03.548496+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_155136.csv | .csv | 21581932 | 否 | 2026-09-18T15:53:14.634113+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_155933.csv | .csv | 33775383 | 否 | 2026-09-18T16:02:05.511812+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_160817.csv | .csv | 33079419 | 否 | 2026-09-18T16:10:52.499920+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_161308.csv | .csv | 15009275 | 否 | 2026-09-18T16:14:18.962435+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_161438.csv | .csv | 14105324 | 否 | 2026-09-18T16:15:44.339123+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_161546.csv | .csv | 49817053 | 否 | 2026-09-18T16:19:36.756253+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_162519.csv | .csv | 37287625 | 否 | 2026-09-18T16:28:04.857436+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_162933.csv | .csv | 36773383 | 否 | 2026-09-18T16:32:21.525902+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_170449.csv | .csv | 85072207 | 否 | 2026-09-18T17:11:31.435946+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_171230.csv | .csv | 15466603 | 否 | 2026-09-18T17:13:43.623398+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_171348.csv | .csv | 33075665 | 否 | 2026-09-18T17:16:21.689743+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20260918_172616.csv | .csv | 26789025 | 否 | 2026-09-18T17:28:17.409772+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_151030.csv | .csv | 45063557 | 否 | 2026-11-03T15:14:04.953700+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_152104.csv | .csv | 86072014 | 否 | 2026-11-03T15:27:49.977144+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_152759.csv | .csv | 33180069 | 否 | 2026-11-03T15:30:37.404705+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_153757.csv | .csv | 30587981 | 否 | 2026-11-03T15:40:20.857720+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_154024.csv | .csv | 282668244 | 否 | 2026-11-03T16:02:42.694723+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_161945.csv | .csv | 40908867 | 否 | 2026-11-03T16:22:56.121008+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_162259.csv | .csv | 10995970 | 否 | 2026-11-03T16:23:49.554489+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_164203.csv | .csv | 84869860 | 否 | 2026-11-03T16:48:40.665871+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_164947.csv | .csv | 2873300 | 否 | 2026-11-03T16:50:01.101042+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| canfd_logs/rm65_canfd_high_follow_20261103_165029.csv | .csv | 14245278 | 否 | 2026-11-03T16:51:33.025042+08:00 | o7_write_only_1hz_20261103_165032 | KEEP / RELOCATE_KEEP | 由保留测试 diagnostics_json.csv_log_path 精确定位；集中保留其原始机械臂 CSV |
| canfd_logs/rm65_canfd_high_follow_20261103_172406.csv | .csv | 19012734 | 否 | 2026-11-03T17:25:30.486228+08:00 | o7_auto_10hz_20261103_172402 | KEEP / RELOCATE_KEEP | 由保留测试 diagnostics_json.csv_log_path 精确定位；集中保留其原始机械臂 CSV |
| concurrency_logs/o7_auto_10hz_20261103_172402/events.csv | .csv | 256884 | 否 | 2026-11-03T17:25:02.265433+08:00 | o7_auto_10hz_20261103_172402 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/o7_auto_10hz_20261103_172402/monitor_console.log | .log | 198999 | 否 | 2026-11-03T17:25:02.487432+08:00 | o7_auto_10hz_20261103_172402 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/o7_auto_10hz_20261103_172402/samples.csv | .csv | 26000785 | 否 | 2026-11-03T17:25:02.260433+08:00 | o7_auto_10hz_20261103_172402 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/o7_auto_10hz_20261103_172402/summary.csv | .csv | 2941 | 否 | 2026-11-03T17:25:02.279433+08:00 | o7_auto_10hz_20261103_172402 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/o7_auto_10hz_20261103_172402/summary.json | .json | 241149 | 否 | 2026-11-03T17:25:02.283433+08:00 | o7_auto_10hz_20261103_172402 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/o7_auto_10hz_20261103_172402/teleop_console.log | .log | 223653 | 否 | 2026-11-03T17:25:28.036245+08:00 | o7_auto_10hz_20261103_172402 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/o7_auto_10hz_20261103_172402.tar.gz | .gz | 1259567 | 否 | 2026-11-03T17:25:47.883100+08:00 | o7_auto_10hz_20261103_172402.tar.gz | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/o7_stream_1hz_20261103_162055/events.csv | .csv | 3365 | 否 | 2026-11-03T16:21:17.409247+08:00 | o7_stream_1hz_20261103_162055 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/o7_stream_1hz_20261103_162055/samples.csv | .csv | 9730034 | 否 | 2026-11-03T16:21:17.409247+08:00 | o7_stream_1hz_20261103_162055 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/o7_stream_1hz_20261103_162055/summary.csv | .csv | 1832 | 否 | 2026-11-03T16:21:17.415247+08:00 | o7_stream_1hz_20261103_162055 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/o7_stream_1hz_20261103_162055/summary.json | .json | 7757 | 否 | 2026-11-03T16:21:17.415247+08:00 | o7_stream_1hz_20261103_162055 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/o7_write_only_1hz_20261103_164300/events.csv | .csv | 20087 | 否 | 2026-11-03T16:43:30.672496+08:00 | o7_write_only_1hz_20261103_164300 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/o7_write_only_1hz_20261103_164300/samples.csv | .csv | 13937336 | 否 | 2026-11-03T16:43:30.671496+08:00 | o7_write_only_1hz_20261103_164300 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/o7_write_only_1hz_20261103_164300/summary.csv | .csv | 2294 | 否 | 2026-11-03T16:43:30.684496+08:00 | o7_write_only_1hz_20261103_164300 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/o7_write_only_1hz_20261103_164300/summary.json | .json | 24525 | 否 | 2026-11-03T16:43:30.684496+08:00 | o7_write_only_1hz_20261103_164300 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/o7_write_only_1hz_20261103_165032/events.csv | .csv | 20083 | 否 | 2026-11-03T16:51:20.259184+08:00 | o7_write_only_1hz_20261103_165032 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/o7_write_only_1hz_20261103_165032/samples.csv | .csv | 21891848 | 否 | 2026-11-03T16:51:20.259184+08:00 | o7_write_only_1hz_20261103_165032 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/o7_write_only_1hz_20261103_165032/summary.csv | .csv | 2275 | 否 | 2026-11-03T16:51:20.270184+08:00 | o7_write_only_1hz_20261103_165032 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/o7_write_only_1hz_20261103_165032/summary.json | .json | 24501 | 否 | 2026-11-03T16:51:20.271184+08:00 | o7_write_only_1hz_20261103_165032 | KEEP / KEEP | 最近两轮关键 HIL 原始数据，内容已核对 |
| concurrency_logs/rm65_o7_concurrency_20260918_170533.csv | .csv | 5028 | 否 | 2026-09-18T17:05:33.202426+08:00 | rm65_o7_concurrency_20260918_170533.csv | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/rm65_o7_concurrency_20260918_171436.csv | .csv | 4267351 | 否 | 2026-09-18T17:15:01.296076+08:00 | rm65_o7_concurrency_20260918_171436.csv | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/single_session_20261103_153842/events.csv | .csv | 220 | 否 | 2026-11-03T15:38:52.804875+08:00 | single_session_20261103_153842 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/single_session_20261103_153842/samples.csv | .csv | 4562975 | 否 | 2026-11-03T15:38:52.804875+08:00 | single_session_20261103_153842 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/single_session_20261103_153842/summary.csv | .csv | 1217 | 否 | 2026-11-03T15:38:52.807875+08:00 | single_session_20261103_153842 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/single_session_20261103_153842/summary.json | .json | 4124 | 否 | 2026-11-03T15:38:52.807875+08:00 | single_session_20261103_153842 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/single_session_20261103_154058/events.csv | .csv | 827 | 否 | 2026-11-03T15:41:19.586133+08:00 | single_session_20261103_154058 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/single_session_20261103_154058/samples.csv | .csv | 9220960 | 否 | 2026-11-03T15:41:19.586133+08:00 | single_session_20261103_154058 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/single_session_20261103_154058/summary.csv | .csv | 1826 | 否 | 2026-11-03T15:41:19.593133+08:00 | single_session_20261103_154058 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| concurrency_logs/single_session_20261103_154058/summary.json | .json | 5216 | 否 | 2026-11-03T15:41:19.594133+08:00 | single_session_20261103_154058 | ARCHIVE / ARCHIVE_MOVE | 已识别历史实验产物；最新自动 HIL tar 的六个文件已逐一与保留目录 SHA-256 一致 |
| docs/O7_AUTO_STREAM_HIL.md | .md | 12839 | 否 | 2026-11-03T17:13:37.646250+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| docs/O7_SINGLE_SESSION_HIL.md | .md | 12818 | 否 | 2026-11-03T16:10:32.487842+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| docs/O7_STREAM_HIL.md | .md | 11327 | 否 | 2026-11-03T17:14:07.625155+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| ik_logs/rm65_ik_dry_run_20260712_175026.csv | .csv | 106837546 | 否 | 2026-07-12T18:20:09.155729+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| ik_logs/rm65_ik_dry_run_20260712_182025.csv | .csv | 39304032 | 否 | 2026-07-12T18:31:15.492729+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| ik_logs/rm65_ik_dry_run_20260712_183122.csv | .csv | 32228918 | 否 | 2026-07-12T18:40:14.968019+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| ik_logs/rm65_ik_dry_run_20260712_184205.csv | .csv | 4426827 | 否 | 2026-07-12T18:43:17.839196+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| ik_logs/rm65_ik_dry_run_20260712_184325.csv | .csv | 33159857 | 否 | 2026-07-12T18:52:33.048590+08:00 | 历史机械臂控制/IK实验 | ARCHIVE / ARCHIVE_MOVE | 已核对 CSV 表头；非最近两轮关联数据，保留到外部历史归档 |
| src/pico_rm65_teleop_interfaces/CMakeLists.txt | .txt | 592 | 是 | 2026-07-13T14:46:28+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/pico_rm65_teleop_interfaces/msg/PicoTeleopState.msg | .msg | 449 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/pico_rm65_teleop_interfaces/msg/RmCanfdControlState.msg | .msg | 1120 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/pico_rm65_teleop_interfaces/msg/RmTeleopTarget.msg | .msg | 577 | 是 | 2026-07-16T19:04:39+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/pico_rm65_teleop_interfaces/package.xml | .xml | 785 | 是 | 2026-07-13T14:46:28+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/README.md | .md | 3183 | 是 | 2026-11-03T15:26:58.836135+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/config/high_follow.yaml | .yaml | 3659 | 是 | 2026-11-03T17:02:17.323981+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/launch/teleop_high_follow.launch.py | .py | 7620 | 是 | 2026-11-03T17:02:17.323981+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/package.xml | .xml | 833 | 是 | 2026-11-03T15:07:21.977933+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/resource/rm_teleop | text/config | 0 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/__init__.py | .py | 0 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/canfd_control_core.py | .py | 4786 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/canfd_csv_logger.py | .py | 3265 | 是 | 2026-07-16T19:04:39+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/hil_monitor_data.py | .py | 10511 | 否 | 2026-11-03T17:03:28.303809+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/o7_auto_stream_hil.py | .py | 12051 | 否 | 2026-11-03T17:09:15.092566+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/o7_single_session_hil.py | .py | 11433 | 否 | 2026-11-03T17:09:15.091566+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/pico_receiver_node.py | .py | 17675 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/protocol.py | .py | 10098 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/receiver_core.py | .py | 6729 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/rm65_canfd_high_follow_node.py | .py | 65659 | 是 | 2026-11-03T17:02:17.322981+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/rm_api2_adapter.py | .py | 17285 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/rm_api2_motion_adapter.py | .py | 4313 | 是 | 2026-11-03T16:31:47.539107+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/rm_feedback_core.py | .py | 7236 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/single_session_hil_monitor.py | .py | 7765 | 否 | 2026-11-03T17:03:28.303809+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/target_generator_core.py | .py | 29711 | 是 | 2026-07-16T19:06:56+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/target_generator_node.py | .py | 12980 | 是 | 2026-07-16T19:04:39+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/rm_teleop/target_math.py | .py | 13664 | 是 | 2026-07-13T14:58:51+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/setup.cfg | .cfg | 88 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/setup.py | .py | 1202 | 是 | 2026-11-03T15:07:21.978934+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_canfd_control_core.py | .py | 2792 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_canfd_static_safety.py | .py | 1393 | 是 | 2026-07-13T14:47:17+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_o7_auto_stream_hil.py | .py | 17923 | 否 | 2026-11-03T17:09:15.095566+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_o7_single_session_hil.py | .py | 10812 | 否 | 2026-11-03T15:10:28.306704+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_o7_stream_hil.py | .py | 18237 | 否 | 2026-11-03T16:47:43.789426+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_protocol.py | .py | 3476 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_receiver_core.py | .py | 3302 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_remote_ik_matrix.py | .py | 936 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_rm_feedback_core.py | .py | 3545 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_sequence.py | .py | 1402 | 是 | 2026-07-13T14:46:04+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_single_high_follow_profile.py | .py | 3789 | 是 | 2026-11-03T15:26:58.802131+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_target_generator_core.py | .py | 11992 | 是 | 2026-07-16T19:05:57+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| src/rm_teleop/test/test_target_math.py | .py | 5619 | 是 | 2026-07-13T14:58:51+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| test_results/R1/T01_static.csv | .csv | 10766517 | 否 | 2026-07-16T15:48:38.974885+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R1/T01_static_launch.log | .log | 7088 | 否 | 2026-07-16T15:48:29.404631+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R1/T02_translation.csv | .csv | 14912078 | 否 | 2026-07-16T15:52:44.955824+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R1/T02_translation_launch.log | .log | 9282 | 否 | 2026-07-16T15:52:32.715941+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R1/T03_wrist_roll.csv | .csv | 19903167 | 否 | 2026-07-16T15:56:13.487942+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R1/T03_wrist_roll_launch.log | .log | 6600 | 否 | 2026-07-16T15:55:56.398917+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R1/T04_pitch_yaw.csv | .csv | 18138815 | 否 | 2026-07-16T15:59:10.560650+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R1/T04_pitch_yaw_launch.log | .log | 6586 | 否 | 2026-07-16T15:58:51.042053+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R1.zip | .zip | 11954629 | 否 | 2026-07-16T16:05:58.913976+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R2/T05_raw_static/T05_raw_static_0.db3 | .db3 | 5390336 | 否 | 2026-07-16T16:54:10.485748+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R2/T05_raw_static/metadata.yaml | .yaml | 2566 | 否 | 2026-07-16T16:54:10.487748+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R2(靠住).zip | .zip | 955366 | 否 | 2026-07-16T16:55:03.664619+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R2（悬空）.zip | .zip | 1359183 | 否 | 2026-07-16T16:53:20.262970+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R3/T07_dynamic/T07_dynamic_0.db3 | .db3 | 22077440 | 否 | 2026-07-16T17:39:42.981954+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R3/T07_dynamic/metadata.yaml | .yaml | 2566 | 否 | 2026-07-16T17:39:42.982954+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R3/T07_dynamic_launch.log | .log | 5657 | 否 | 2026-07-16T17:39:38.658755+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R3.zip | .zip | 4087819 | 否 | 2026-07-16T17:40:15.738483+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R4/T02_rotation.csv | .csv | 18658965 | 否 | 2026-07-16T18:45:57.863924+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R4/T02_rotation_launch.log | .log | 10748 | 否 | 2026-07-16T18:45:46.640583+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R4/T03_translation.csv | .csv | 19238505 | 否 | 2026-07-16T18:48:36.301794+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R4/T03_translation_launch.log | .log | 6407 | 否 | 2026-07-16T18:48:08.348927+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R4.zip | .zip | 5092691 | 否 | 2026-07-16T18:48:45.904092+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T01_after_motion_static.csv | .csv | 19980033 | 否 | 2026-07-22T17:35:51.547431+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T01_after_motion_static_launch.log | .log | 11929 | 否 | 2026-07-22T17:35:18.370687+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T01_after_motion_static_state.log | .log | 2922306 | 否 | 2026-07-22T17:35:25.529847+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T02_rotation.csv | .csv | 23801144 | 否 | 2026-07-22T17:41:26.696954+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T02_rotation_launch.log | .log | 10868 | 否 | 2026-07-22T17:40:58.906330+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T02_rotation_state.log | .log | 3531463 | 否 | 2026-07-22T17:41:05.192471+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T03_interrupt_settling.csv | .csv | 22122848 | 否 | 2026-07-22T17:45:56.418007+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T03_interrupt_settling_launch.log | .log | 11729 | 否 | 2026-07-22T17:45:17.503134+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T03_interrupt_settling_state.log | .log | 3264520 | 否 | 2026-07-22T17:45:23.238263+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T04_translation.csv | .csv | 28958065 | 否 | 2026-07-22T17:48:47.086838+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5/T04_translation_launch.log | .log | 20644 | 否 | 2026-07-22T17:48:29.668447+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| test_results/R5.zip | .zip | 11226209 | 否 | 2026-07-22T17:50:21.504957+08:00 | 历史 R1-R5 机械臂调试 | ARCHIVE / ARCHIVE_MOVE | 既有历史实验目录及记录/压缩包，外部完整保留，不永久销毁 |
| 一、参数生效关系.md | .md | 13410 | 否 | 2026-11-03T15:02:18.561766+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| 开发交接.md | .md | 11788 | 是 | 2026-08-28T14:27:47.723676+08:00 | 工程 | KEEP / KEEP | 工程源码/接口/测试/配置/技术文档，保留 |
| build/ | directory | 4096 | 否 | 2026-07-22T17:16:02.143777+08:00 | 构建/工具/本地环境 | KEEP / KEEP | 构建/工具/凭证状态不属于本轮历史实验清理；不读取受保护目录内容、不删除 |
| install/ | directory | 4096 | 否 | 2026-07-22T17:16:02.699790+08:00 | 构建/工具/本地环境 | KEEP / KEEP | 构建/工具/凭证状态不属于本轮历史实验清理；不读取受保护目录内容、不删除 |
| log/ | directory | 4096 | 否 | 2026-11-03T17:38:27.810447+08:00 | 构建/工具/本地环境 | KEEP / KEEP | 构建/工具/凭证状态不属于本轮历史实验清理；不读取受保护目录内容、不删除 |
| __pycache__/ | directory | 4096 | 否 | 2026-08-28T14:56:35.107137+08:00 | 构建/工具/本地环境 | UNKNOWN / KEEP | 构建/工具/凭证状态不属于本轮历史实验清理；不读取受保护目录内容、不删除 |
| .git/ | directory | 4096 | 否 | 2026-09-18T13:52:51.207764+08:00 | 构建/工具/本地环境 | KEEP / KEEP | 构建/工具/凭证状态不属于本轮历史实验清理；不读取受保护目录内容、不删除 |
| .aws/ | directory | 40 | 否 | 2026-11-03T17:41:40.199550+08:00 | 构建/工具/本地环境 | KEEP / KEEP | 构建/工具/凭证状态不属于本轮历史实验清理；不读取受保护目录内容、不删除 |
| .agents/ | directory | 40 | 否 | 2026-11-03T17:41:40.199550+08:00 | 构建/工具/本地环境 | KEEP / KEEP | 构建/工具/凭证状态不属于本轮历史实验清理；不读取受保护目录内容、不删除 |
| .codex/ | directory | 40 | 否 | 2026-11-03T17:41:40.199550+08:00 | 构建/工具/本地环境 | KEEP / KEEP | 构建/工具/凭证状态不属于本轮历史实验清理；不读取受保护目录内容、不删除 |

## 未处理缓存补充清点

| 路径 | 类型 | 大小 bytes | Git跟踪 | 修改时间 | 分类/操作 | 原因 |
|---|---|---:|---|---|---|---|
| __pycache__/ik_acceptance_test.cpython-310.pyc | .pyc | 14964 | 否（忽略） | 2026-08-28T14:56:35.107137+08:00 | UNKNOWN / KEEP | 仅能识别为历史字节码，未确认可删除；保留待确认 |
