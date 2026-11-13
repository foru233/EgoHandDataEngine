# VS030 v4 实时恢复实验入口

客户 GUI：运行 `userrr/run_live_recovery_candidate.bat`，或执行：

```text
.venv\Scripts\python.exe userrr\run_client_gui.py --live-recovery-candidate
```

窗口会固定显示 `VS030 设备` 和 `双目＋IMU · v4 实时恢复实验版`。点击“连接 / 启动”后，程序会先检查相机、IMU、标定、ORB 设置、恢复配置和候选 DLL；任一文件缺失、哈希不匹配或 ABI 标记不满足都会停止实验并报告错误。

研发 GUI 运行 `run_gui.bat`，在 VIO 控制区把 Profile 选为 `v4 实时恢复实验版`，再启动 VS030 数据源。两个 GUI 共用 `LiveRecoveryProfile` 和同一个实时 VIO 子进程配置，不会修改正式 `active_bridges.json` 或正式 DLL。

启动后的实时日志和 GUI 状态会显示子进程 PID、DLL SHA-256、恢复配置 SHA-256、初始化状态、预测/视觉恢复状态和独立新地图事件。v4 保持实验标签；未完成 VS030 实机和精度验收前，不切换正式默认模式。
