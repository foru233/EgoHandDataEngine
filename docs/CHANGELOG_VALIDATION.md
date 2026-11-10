# M01–M05 验证增强变更

本轮保留原接口/界面，源码快照和完整基线见BENCHMARK_BASELINE.md。主目录无Git，使用ZIP+摘要+独立差异文件审计，不重置用户修改。

## 软件修改

- `stereo/uncertainty.py`：中心数值Jacobian、步长一致性、退化/PSD检查及可选拒绝；默认诊断；原quality_score公式保持。
- `core/types.py`：1.1结果schema，追加四个可选协方差字段，1.0 JSON兼容；`storage/exporter.py`追加CSV/NPZ字段；`storage/recorder.py`保持raw容器1.0并记录result_schema_version。
- `stereo/hand_matching.py`：保留SciPy Hungarian，删除disparity对几何覆盖损失的重复加权（原配置键保留、诊断声明忽略）；保存候选、成本分量、拒绝状态、替代解margin。长间隔重识别增加历史运动/姿态条件，不能只靠最近邻恢复ID。
- `core/pipeline.py`：增加可选dataset_pairing上下文，原调用兼容；同步分层、关联/不确定性分开诊断。普通采集原同步门控保留。
- `benchmarks/`：严格STB及HOT3D适配、官方FISHEYE624映射、原生灰度正向推理/逆映射、覆盖率与N/A指标、同步实验、独立CLI/HTML。
- `scripts/prepare_hot3d.py`和转换脚本：完整tar/哈希/逐帧检查、只复制标注；官方UmeTrack FK在独立既有Torch环境执行。没有把Torch或MANO加入主环境。
- `.gitignore`只追加数据/结果目录排除，避免整个数据集进入未来源码提交。

## 先失败，再修复的证据

`logs/uncertainty_before.xml`：复现右像素系统偏移造成大三维误差、低重投影；新增协方差字段测试在实现前失败。协方差不是解决系统偏差的万能算法，最终仍保留此分类与限制。

`logs/association_before.xml`：3 failed/5 passed，其中两项具体行为失败：重复覆盖损失使错误候选被接受；只有一次历史也会在250ms后强行恢复旧track。第三项是新增诊断未实现。修改后相关原有/新增关联与几何测试21 passed。

新代价下前一个案例变为明确歧义拒绝；后一个案例新建track。收益是这两个条件下的错误更可靠地拒绝/隔离，未证明真实数据错误率已降低。

## 验证入口

```powershell
.venv\Scripts\python.exe -m pytest -q --junitxml=logs/validation_20261103_114805/final_tests.xml
.venv\Scripts\python.exe scripts\validate_gui_unchanged.py
.venv\Scripts\python.exe scripts\run_synchronization_experiment.py
.venv\Scripts\python.exe scripts\run_stereo_benchmark.py --help
```

原有测试文件摘要均保持。四窗口×两主题原生Qt/真实GL截图在 `logs/validation_20261103_114805/gui_final`；每次均42有效合成关节、errors=[]。保护范围GUI/visualization/主题/布局及design-system摘要与修改前一致。

最终完整测试186 passed、1 skipped（真实STB资产不可用）、0 failed；原85项全部保留。真实数据结果和性能数字见BENCHMARK_REPORT.md；测试XML、保护摘要、完整源差异位于本次logs目录。官方数据无真值或无可验证资产时标skip/blocked，不计入“精度通过”。
