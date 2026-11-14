# 测试结果

使用 `E:\EgoHandDataEngine\.venv\Scripts\python.exe`。

专项显示和面板测试：

```text
tests/test_best_effort_display_pose.py
tests/test_vio_panel.py
20 passed
```

实时、回放和 GUI 回归：

```text
tests/test_best_effort_display_pose.py tests/test_vio_panel.py
tests/test_replay_gui.py tests/test_offline_recovery_gui.py
tests/test_gui_smoke.py tests/test_vio_live_runtime.py
49 passed
```

回放链路：`tests/test_vio_replay.py`，`28 passed`。

全量回归（当前代码）：`961 passed, 1 skipped, 2 warnings`。

覆盖内容包括外参旋转/偏移、预测轴移动、独立轨迹、视觉候选、冻结、恢复跳变、地图身份隔离、非法矩阵、暂停期间保留视图，以及回放和实时字段兼容。

截图：`client_gui_imu_prediction_simulated.png`。该截图是软件模拟失跟，不是真实 VS030 遮挡验收。
