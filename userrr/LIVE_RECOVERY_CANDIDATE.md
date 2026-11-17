# VS030 v4 交付 R1（20261116）

客户 GUI：运行 `E:\EgoHandDataEngine\userrr\run_client_gui.bat`。关闭原来已打开的 GUI 后重启，才能加载本次更新。也可执行：

```text
.venv\Scripts\python.exe userrr\run_client_gui.py
```

默认选择 `VS030 设备` 和 `双目＋IMU · v4 实时恢复 · 交付 R1`；导入录制时自动切换为 v4 离线恢复 R1。三份 runtime_configs 清单的 release_id 均为 `v4-r1-20261116`，启动校验 DLL、配置和标定哈希。手部仍使用原模型与双目三角化，已整合同帧位姿绑定和最新有效配对显示修正。

研发 GUI 运行 `E:\EgoHandDataEngine\run_gui.bat`，VIO 区默认 v4 使用相同 R1 Profile。原正式 DLL 入口仍保留；旧 DLL、冻结候选、stable_offline_v2、原录制与标定未覆盖。

已完成三段真实录制回放、EuRoC 真值回归和录制采集节奏下的可见 GUI 检查。184714 仍可能约 4 秒定位中断，未验收 VS030 实机物理精度与稳定 30 FPS。临时预测与可信定位分开，不同 segment 不强行拼接。

交付证据、边界、冻结与完整回退步骤见 [交付说明](../handoff/delivery_v4_r1_20261116/交付说明.md)。完整回退默认仅核验：`.venv\Scripts\python.exe handoff\delivery_v4_r1_20261116\rollback.py`；明确加 `--apply` 才恢复更新前源文件和清单。冻结后停止继续更改。

## 轨迹显示补丁（20261116）

已修复参考更新/地图重启时历史轨迹从界面丢失：旧段独立保留，通过三维视图工具栏的“历史轨迹（N）”回看，当前视图继续使用当前参考。不会连接不同参考坐标，不将历史位姿用于当前手部。历史为当前 GUI 会话内显示缓存；新采集会话、明确“清除”或关闭 GUI 会清除该缓存，原始位姿日志仍保留。单参考显示仍沿用最多 10000 点限制。

补丁验收、封存与回退见 [轨迹显示修复说明](../handoff/delivery_v4_r1_history_fix_20261116/修复说明.md)。本补丁不改变 VIO DLL、准入策略、模型或标定。若要恢复更新前完整 v4，先关闭 GUI 并执行该补丁的回退，再执行 R1 的回退。
