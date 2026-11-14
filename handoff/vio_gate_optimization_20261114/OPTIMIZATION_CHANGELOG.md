# 实际修改

| 文件 | 修改 | 风险边界 |
|---|---|---|
| `src/egohand/vio/display_pose.py` | 冻结状态保留最后真实估计时间戳，连续冻结年龄正确累计 | 仅 GUI 显示 |
| `tests/test_best_effort_display_pose.py` | 增加连续冻结年龄测试 | 不改变运行时门控 |
| `scripts/audit_gate_decisions.py` | 新增真实回放逐帧门控、事件时序和 shadow 输出 | 只读分析 |

没有修改正式门控数值、RecoveryCoordinator 准入语义、VIO DLL、地图拼接规则或世界手部准入。
