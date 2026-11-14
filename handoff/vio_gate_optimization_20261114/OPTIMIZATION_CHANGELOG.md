# 实际修改

| 文件 | 修改 | 风险边界 |
|---|---|---|
| `src/egohand/vio/display_pose.py` | 冻结状态保留最后真实估计时间戳，连续冻结年龄正确累计 | 仅 GUI 显示 |
| `tests/test_best_effort_display_pose.py` | 增加连续冻结年龄测试 | 不改变运行时门控 |
| `scripts/audit_gate_decisions.py` | 新增真实回放逐帧门控、事件时序和 shadow 输出 | 只读分析 |
| `configs/vio/live_quality_fast_experimental.yaml` | 新增 0.5 秒应用层稳定窗口候选 | 仅显式快速离线候选 |
| `handoff/vio_recovery_bridge_v4_20261113/recovery_candidate_fast.yaml` | 新增 0.5 秒 RecoveryCoordinator 候选 | 仅显式快速离线候选 |
| `userrr/runtime_configs/offline_recovery_fast_candidate.json` | 绑定快速候选的质量、恢复配置和 DLL 哈希 | 不替换原候选 |
| `userrr/run_client_gui.py` | 增加 `--offline-recovery-fast-candidate` 入口 | 不改变默认启动 |

没有修改正式稳定版门控数值、RecoveryCoordinator 准入语义、VIO DLL、地图拼接规则或世界手部准入。快速候选只在显式入口使用。
