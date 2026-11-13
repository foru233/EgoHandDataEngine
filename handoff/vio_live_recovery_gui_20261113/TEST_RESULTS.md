# 测试结果

## 软件烟测

- 客户 GUI：`run_client_gui.py --live-recovery-candidate --quit-after 1`，进程退出码 0。
- 客户 GUI 截图：[customer_gui_v4_candidate_windows.png](customer_gui_v4_candidate_windows.png)。
- 研发 GUI 的 v4 选择截图：[research_gui_v4_profile_visible.png](research_gui_v4_profile_visible.png)。
- 客户 GUI 日志：[gui_smoke.log](gui_smoke.log)。
- Profile 证据：[profile_provenance.json](profile_provenance.json)。
- Profile 几何、ABI 标记和哈希校验：4 项定向测试通过。
- 实时运行时、两个 GUI、传感器模式、离线候选回归：53 项定向测试通过。
- 全项目：`935 passed, 1 skipped, 2 warnings`。

## 现有回归

本轮完整回归已覆盖新增接线。离线 111807、111833、184714 的历史证据保持引用，不因本轮 GUI 接线重复运行完整矩阵。

## 证据边界

软件烟测只证明入口、Profile 绑定和窗口创建，不证明相机连接、真实 30 FPS、定位精度或恢复错误率。
