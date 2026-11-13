# VIO 验证记录（2026-11-11，持续更新）

## 2026-11-12 最新验证索引

以下历史表格保留原始失败和当时状态，不以本轮成功覆盖。当前证据：

| 验证 | 结果/证据 |
| --- | --- |
| 已部署fail_closed库 | SafeExp故障闭锁、原生注入、旧Atlas不变、1727帧完整回放、3200帧压力回放均有记录；`results/vio/safe_exp_fail_closed_20261112` |
| 时序重复对照 | 10对有效受控运行、8对完整可测几何结果；失败/未完成保留；`phase_confirmation_20261112`及`_additional` |
| 跨录制相位/启动验证 | 新+34/旧+99ms独立旋转核验；实际LiveVio两会话同PID复用词典；自动工作快照原始文件不变；`results/vio/session_phase_20261112` |
| 地图包/GUI | 2975占据、25490空闲体素，GUI实际导入；同源272/300回载，旧独立录制559/974有效且处于加载地图0/epoch9；不作为连续实机定位或绝对精度通过 |
| 当前Python回归 | 已部署链路497通过、1跳过、2条依赖告警，显式排除原有屏幕/DPI测试；`logs/vio_scan_readiness_final_regression_20261112.log` |
| 正深度v3候选 | 同DLL只切开关，10对/20次均1727帧完成、退出0，源码/输入核对通过；开启的全部提交检查点无效深度0；原始有效帧均1715，门控中位数两组均1497；`cheirality_v3_confirmation_20261112` |
| v3最终发布验证 | 3200帧完成；Atlas建图330/340有效、回载100/100有效且原文件不变；两会话同PID词典复用；全量Python498通过、1跳过、2条依赖告警。结果 `results/vio/cheirality_v3_release_20261112/release_validation.json` success=true |
| 默认部署 | live/Atlas选择cheirality_v3 SHA0a5b、正深度开关1；上一版SHA91及清单备份保留。部署后配置/GUI地图等25项回归通过；重启程序生效 |

本轮准确命令在每个结果目录的 `inputs.json`、`commands.json`/`summary.json` 和发布验证JSON内。原生真实断言测试、17项相关Python回归及被替代候选原因见 `VIO_正深度观测修复_20261112.md`。未通过/未实施的实机30分钟、独立真值ATE/RPE、定距往返、真实遮挡与家具变化仍明确保留；用户已选择稍后配合实机，当前继续离线检查。

3200帧回放有838帧未初始化，不能称全程准确定位；128个资源样本RSS约532→848MiB，尚不能证明长期内存上界。当前扫描Atlas进程关闭回环，以上导出回归不覆盖启用回环时的独立后台GBA线程关闭/保存同步，该通用路径仍在专项核查。不把本批局部修复的通过扩大成所有路径或实机精度均已通过。

## 输入与冻结基线

新录制：`data/imu/session_20261111_105436_609132400`。基线备份、源码和 DLL 哈希位于 `results/vio/stability_baseline/source_manifest.json` 与 `baseline_files/`，原始录制不改写。完整性检查为 1736 对索引双目和 27776 条 IMU；实际完整解码及前导 IMU 覆盖筛选后本轮回放 1727 帧。

每组确切命令与输入哈希由 `inputs.json`、`gauge_matrix/matrix.json` 记录。回放与物理设备结果分开，本报告尚无连续 30 分钟实机验收。没有独立真值，ATE/RPE 不可测。CPU 构建/测试同时运行过，现有延迟只作诊断，不能据此声称修复提速。

## 已执行

```powershell
.\.venv\Scripts\python.exe scripts/audit_vio_recording_inputs.py data/imu/session_20261111_105436_609132400 --output results/vio/stability_baseline/recording_inputs_rechecked.json
.\.venv\Scripts\python.exe scripts/run_vio_stability_experiment.py --recording data/imu/session_20261111_105436_609132400 --output results/vio/stability_baseline/new_v9_baseline --library src/egohand/vio/native/bin/egohand_vio_recovery_bridge_v9.dll
cmd /c scripts\build_vio_stability.cmd
.\.venv\Scripts\python.exe scripts/run_vio_stability_experiment.py --recording data/imu/session_20261111_105436_609132400 --output results/vio/stability_baseline/new_diag_A --library src/egohand/vio/native/bin/egohand_vio_stability_diagnostic.dll --diagnostics
.\.venv\Scripts\python.exe scripts/run_vio_stability_matrix.py --recording data/imu/session_20261111_105436_609132400 --library src/egohand/vio/native/bin/egohand_vio_stability_diagnostic.dll --output results/vio/stability_baseline/gauge_matrix
.\.venv\Scripts\python.exe scripts/extract_vio_jump_cases.py --recording data/imu/session_20261111_105436_609132400 --poses results/vio/stability_baseline/new_v9_baseline/poses.jsonl --output results/vio/jump_cases/new_v9 --cases 6
.\.venv\Scripts\python.exe scripts/summarize_vio_stability_experiments.py results/vio/stability_baseline
.\.venv\Scripts\python.exe -m pytest tests -q --disable-warnings --maxfail=3
```

全量回归此次为 **432 passed, 1 skipped**，日志 `logs/vio_stability_regression.log`；后续原子快照原生改动需单独再验，不将这次结果移用于后续改动。

## 已完成实验结果

| 组 | 帧数 | 原生有效 | 门控有效 | 同 epoch 相邻记录 >0.5m | 同 epoch 最大步长 | 运行结果 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| v9 基线 | 1727 | 1519 | 456 | 9 | 24.612m | 完成、退出0 |
| 新诊断 A，规范修复关闭 | 1727 | 1608 | 694 | 16 | 22.102m | 完成、退出0 |
| 同 DLL，规范修复 B 打开 | 760 | 709 | 392 | 0 | 0.230m | 单次Track超时，失败 |
| 同诊断库重复 A | 1727 | 1630 | 1115 | 1 | 1.031m | 完成、退出0 |
| 关闭实验恢复协调器 | 1727 | 1369 | 477 | 10 | 1.138m | 完成、退出0 |
| packet 相位 +77ms | 1727 | 1660 | 1016 | 0 | 0.380m | 完成、退出0 |
| packet 相位 +117ms | 1727 | 1431 | 324 | 5 | 1.037m | 完成、退出0 |

A 与 v9 数据不同还包含异步调度和诊断开销影响，不能当作算法精度提升。B 仅在独立实验进程由环境变量开启，**未部署为 GUI 或扫描器默认行为**。当前没有证据宣布规范修复候选已通过。

同诊断库同输入重复 A 的指标明显不同，表明异步建图与优化调度会影响回放结果；因此不能仅凭一轮 +77ms 的表现将其选为真实物理相位，默认仍 +97ms。关闭实验协调器仍保留既有原生恢复分支，不等于完全未修改上游 ORB。

原子快照最终 DLL `egohand_vio_stability_snapshot.dll` 已成功构建，SHA256 `b7e99ba9aad1f4c2add5d1551a6724c91bcef0c68fa41e25c8ef6f0045bdad16`。全量新增快照代码回归为 **442 passed, 1 skipped**（`logs/vio_stability_snapshot_regression.log`），原生回放另行执行。未将此 DLL 部署为默认。

## 产物

- 原始位姿、worker日志、输入哈希、间隔、图像诊断与RSS采样：各实验目录。
- 带事件的轨迹、速度、角速度、初始化与版本时序图：各 `analysis/timeline.png`。
- 相同 ID 关键帧/地图点、求解残差与刚体/相似变换拟合：`native_diagnostics/fit_report.json`。
- 六个最大突变案例的前后原始双目图、状态、独立OpenCV特征对应：`results/vio/jump_cases/new_v9`。其特征匹配不是原生ORB内点，明确区分。

## 仍需完成的验收

原子快照版本、纯双目/惯性、固定版本重复、恢复关闭、相位与旧标定对照结果需要逐组补齐。3000帧以上长录制、旧NaN黑帧恢复、Atlas非零地图ID和保存加载、GUI/扫描器完整回归及实机30分钟尚不能依据上述结果视为已通过。物理曝光同步、外参方向、scale-misalignment与真值精度需要可验证的厂商资料或独立测量。
