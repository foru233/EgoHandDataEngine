# VS030 v4 实时恢复实验入口

客户 GUI：运行 `userrr/run_live_recovery_candidate.bat`，或执行：

```text
.venv\Scripts\python.exe userrr\run_client_gui.py --live-recovery-candidate
```

普通客户 GUI 现已默认选择 `VS030 设备` 和 `双目＋IMU · v4 实时恢复实验版`；导入录制时自动切换为 v4 离线恢复候选。点击“连接 / 启动”后，候选文件缺失、哈希不匹配或 ABI 标记不满足都会报告错误。

研发 GUI 运行 `run_gui.bat`，VIO 控制区默认选择 `v4 实时恢复实验版`；录制面板的离线 VIO 回放也使用该选择。可选择正式稳定版回退。两个 GUI 不会修改正式 `active_bridges.json` 或正式 DLL。

启动后的实时日志和 GUI 状态会显示子进程 PID、DLL SHA-256、恢复配置 SHA-256、初始化状态、预测/视觉恢复状态和独立新地图事件。v4 保持实验标签；未完成 VS030 实机和精度验收前，不切换正式默认模式。
