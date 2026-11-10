# 真实双目验证增强：修改前基线

基线建立于2026-11-03 11:48（Asia/Shanghai）。当前目录没有Git仓库，因此使用可恢复的ZIP和逐文件SHA256清单，未覆盖、重置或清理任何已有改动。

- 源码/配置/测试/文档快照：`logs/validation_20261103_114805/baseline/source_before.zip`（105文件）。
- 文件摘要：同目录 `source_manifest.json`；Git查询原始结果见 `git_status.txt`。
- 环境：同目录 `environment.json`；Python3.12.15，现有项目`.venv`；MediaPipe0.10.21 / OpenCV contrib4.11.0.86 / NumPy1.26.4 / PySide6 6.8.3。
- 本次实际完整测试：**85 passed，0 failed，0 skipped，8.42s**；`baseline/tests.xml`。包括官方模型真实加载/图片推理和最近的三维拖动修复；不是历史73项结果。
- 原生GUI：`baseline/gui_evidence.json`，Windows平台、有效GL上下文、42个有效合成关节、errors=[]；深浅截图在 `gui_dark.png`、`gui_light.png`。
- 原有JSON/CSV/NPZ结构参照 `docs/DATA_SCHEMA.md`、`storage/exporter.py` 和原始快照；schema_version=1.0.0。Joint3D尚无协方差字段。

审查发现：quality_score仅表达极线/重投影一致性；关联代价中disparity和geometry重复使用有效点损失比例；硬件同步与数据集配对尚无独立评测入口。这些结论不是改进精度的证据，后续先以回归/实验量化。

已阅读现有README、架构、坐标、schema、测试报告、UI更新及指定核心源文件/测试。GUI/visualization与Skill文件列为本轮保护范围，结束时对照本基线校验摘要。
公开数据使用用户指定路径 `%PROJECT_ROOT%\samples\date`，不使用任务书中的建议datasets目录。下载数据不加入源码快照或对外打包。
