# 测试结果

专项显示、冻结和坐标系稳定修复：`23 passed`（`tests/test_best_effort_display_pose.py`、`tests/test_vio_panel.py`）；离线候选配置回归合计 `31 passed`。

门控/实时/回放相关回归在上轮代码基础上通过；当前全量回归为 `964 passed, 1 skipped, 2 warnings`。

分析命令：

```text
.venv/Scripts/python.exe scripts/audit_gate_decisions.py <poses.jsonl> <report.json> --decisions <decisions.jsonl>
```

输入回放哈希、DLL、配置和标定哈希见 `BASELINE_METRICS.json`。所有原始录制只读，未覆盖正式 DLL 或历史结果。
