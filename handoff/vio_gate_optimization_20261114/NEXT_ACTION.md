# 后续任务

1. 使用独立的 VS030 外部定位或标定场景采集至少多组完整失跟/恢复事件，按完整录制划分开发、验证和测试集。
2. 对稳定窗口候选（至少 0.5、1.0、1.5 s）分别重新运行原生 DLL、RecoveryCoordinator、地图和手部链路，比较错误恢复与同图 RPE。
3. 对 184714 的 141–492 帧按原生事件日志进一步拆分初始化、参考切换和失跟原因，避免用单一窗口参数处理不同根因。
4. 若闭环 A/B 证明安全，再以独立配置提交数值变更；否则继续保留 1.5 s 基线。

当前可直接复现快速候选：

```text
.venv/Scripts/python.exe userrr/run_client_gui.py --offline-recovery-fast-candidate --recording E:\\EgoHandDataEngine\\data\\session_20261112_184714_636910800 --start
```
