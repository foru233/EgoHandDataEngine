# VS030 v4 实时恢复 GUI 交付状态

更新时间：2026-11-13

当前状态：软件接线完成，候选模式已可由两个 GUI 显式选择；实机验收待设备。

已完成：

- 客户 GUI 增加独立 `--live-recovery-candidate` 入口和 `run_live_recovery_candidate.bat`。
- 候选 Profile 校验标定、ORB 设置、时序、质量门控、恢复配置和 DLL SHA-256，并检查短时恢复 ABI 标记。
- 候选只传入隔离 VIO 子进程；正式 `active_bridges.json`、正式 DLL 和默认模式未改动。
- 研发 GUI 的实时 VIO 控制区域增加相同的 Profile 选择。
- 实时子进程身份输出 PID、DLL SHA-256、恢复配置 SHA-256，并按配置内容变化重建持久进程。
- 初始化、失跟、IMU 预测、视觉恢复和独立新地图仍沿用既有状态机与世界坐标隔离逻辑。

当前边界：未连接 VS030；本目录中的 GUI 截图和启动日志是软件烟测，不是设备运行证据。
