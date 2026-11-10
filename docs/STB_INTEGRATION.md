# STB接入与当前门禁

官方来源：<https://github.com/zhjwustc/icip17_stereo_hand_pose_dataset>，参考仓库本地提交 `9d849ff7edbc2a0982fcfbd3f8e105c6cc156c88`。
官方论文：<https://jianbojiao.com/pdfs/icip.pdf>。

当前数据路径 `samples/date/stb`。已检查官方README和历史：当前OneDrive链接返回404；百度分享页面提示链接不存在/失效；仓库历史中的Dropbox链接跳转至公开文件夹，但当前页面未能列出样例文件。访问记录在 `samples/date/access_probe.json` 与该目录下的response/access文件。
截至本阶段，未获取可验证的BB图像对，状态 **BLOCKED_BY_DATA**；没有真实STB MPJPE，也没有用Oracle分数替代模型评测。

## 适配器

`benchmarks/stb.py:STBBenchmarkAdapter`只配同序列、同编号`BB_left_*`和`BB_right_*`，只读取`*_BB.mat`的`handPara[3,21,N]`并把毫米转米。SK_color绝不混入立体对。
排序保持时间顺序，可指定sequence/固定frame清单；不制造左右传感器时间戳，sync_quality=DATASET_DOCUMENTED，时间精度未验收。

官方README提供的BB基线和焦距不足以独立确认外参方向、图像是否已校正和GT坐标系。因此默认 calibration=None / CALIBRATION_MAPPING_UNVERIFIED。
要启用米制评测，须提供`benchmark_calibration.json`、实际标定文件、明确GT原始左光学坐标和raw几何，并通过至少5组独立观测的左右投影核验。Oracle投影再回算不算独立证据。其他原生已校正投影需先明确适配，不能仅改标签。

## 保守关节映射

官方列表的掌心点0排除；四指的MCP/PIP/DIP/TIP共16点可比较；拇指仅明确的tip纳入。STB拇指mcp/pip/dip与MediaPipe CMC/MCP/IP不能仅凭序号认定同义，因此排除17/18/19。
最终17点，完整映射/原因由 `stb_joint_mapping()` 和 `preparation_report.json` 输出。没有把STB21点顺序原样喂给MediaPipe骨架。

## 本地数据准备

```text
samples/date/stb/
  images/B1Counting/BB_left_0.png
  images/B1Counting/BB_right_0.png
  labels/B1Counting_BB.mat
  benchmark_calibration.json  # 独立校验后的几何契约，非猜测示例
```

`.venv\Scripts\python.exe scripts\prepare_stb.py --dataset-root samples/date/stb --max-frames 200`。
本轮格式/映射测试使用明确的人工格式fixture，不是真实数据成绩；实际资产测试在缺失官方数据时明确skip。
公开仓库代码为MIT；外部下载数据的具体再分发许可未在现有README中单独明确，不将代码许可自动推定为图像无限制再分发许可。
