# 后续任务

全量数据与公开 GT 联合验证已纳入主任务。当前检查点、盘点覆盖范围和下一条工作见 `CROSS_DATASET_CHECKPOINT.md`；以下原有任务仍需执行。

1. 使用独立的 VS030 外部定位或标定场景采集至少多组完整失跟/恢复事件，按完整录制划分开发、验证和测试集。
2. 对稳定窗口候选（至少 0.5、1.0、1.5 s）分别重新运行原生 DLL、RecoveryCoordinator、地图和手部链路，比较错误恢复与同图 RPE。
3. 对 184714 的 141–492 帧按原生事件日志进一步拆分初始化、参考切换和失跟原因，避免用单一窗口参数处理不同根因。
4. 若闭环 A/B 证明安全，再以独立配置提交数值变更；否则继续保留 1.5 s 基线。
5. 为短恢复 ABI 实现真正的原生立体/地图几何求解器；当前 `egohand_vio_short_recovery_report` 只返回结构化 unavailable，任何 candidate 运行都必须继续隔离，不能把这次 room1 结果当作恢复成功。
6. 在 solver 完成后，先用 room1/room2 开发集做重复事件和 GT 关联验证，再用未参与调参的 room3 holdout 做冻结后的最终评测；在此之前不更新正式 DLL、默认 1.5 s 参数或发布结论。

当前可直接复现快速候选：

```text
.venv/Scripts/python.exe userrr/run_client_gui.py --offline-recovery-fast-candidate --recording E:\\EgoHandDataEngine\\data\\session_20261112_184714_636910800 --start
```
