# EgoHand Data Engine / EgoHand Studio

Windows 上的**双目手部几何重建与数据采集**工程：用左右两路相机检测手部 21 个关键点，
完成极线校正、双手关联、三角化与质量门控，输出**左相机光学坐标系下的米制三维关节点**，
并支持录制、回放、重处理与 JSON/CSV/NPZ 导出。

> 本项目是研究/工程工具，不是已通过计量验收的产品。真实设备的同步精度与米制测量精度
> 尚未完成硬件级验收，详见 [注意事项](#注意事项) 与 [docs/TEST_REPORT.md](docs/TEST_REPORT.md)。

## 项目解决的问题

- 单目 MediaPipe 只能给二维关键点；本项目把**立体视觉**接上：标定 → 校正 → 左右匹配 →
  三角化 → 几何有效性检查，得到带坐标系、单位、有效性标记的三维关节点。
- 手部数据采集通常缺少**可追溯性**：本项目为每帧结果保留来源、标定 ID、时间戳、
  同步等级、质量分数与不确定性字段，明确区分合成/数据集/真实设备，不虚构精度。
- 提供从**采集 → 录制 → 离线重处理 → 导出**的完整链路，并保留 GUI 与无 GUI 两种入口。

## 主要功能

**重建链路**
- 12 类合成几何场景（交叉、远近、遮挡、错配、无效标定、零视差、时间异常等），用于算法验收。
- 支持 pinhole 与 fisheye 畸变模型；立体校正、极线误差/重投影误差门控。
- 双手检测与左右视图关联（Hungarian 匹配 + 歧义代价 + 几何硬门控）。
- 三维三角化，带骨骼长度、速度/时序、跨帧 ID 跟踪与重捕获检查。
- 可选三维修正不确定性诊断（数值 Jacobian 传播，`reject` 模式下拒绝高不确定点）。

**检测**
- MediaPipe Tasks Hand Landmarker（IMAGE / VIDEO 两种模式，左右视图独立实例）。
- 漏检救援：全图完全漏检时对四个固定重叠区域重试。
- 可选 RTMPose/RTMDet 检测器与手动 ROI（需要额外下载 onnxruntime 与 ONNX 权重）。

**数据源**
- 几何模拟（synthetic）、录制回放、外部图像清单 / 双视频 + 时间戳 CSV。
- 真实单图（单目 2D only）、HOT3D Clips、SHOW3D v3 双目、DexYCB 多相机 RGB。
- VS030 厂商相机双通道预览、双目 H.265 视频 + IMU 原始载荷录制（实验性）。

**GUI（PySide6）**
- 双视图显示 + 可旋转/缩放/平移/选择的 OpenGL 3D 骨架，深浅主题。
- 播放控制（暂停/继续/单帧/定位/循环）、数据源切换、标定面板、检测参数面板、ROI 面板。
- 录制面板（独立「录制」页）、VIO 离线回放面板、调试页、当前帧与批量导出。
- 有界队列后台 worker：实时模式丢最旧待处理帧并计数，离线模式等待不丢帧。

**存储与导出**
- 录制分层：`raw/` 原始图像/视频/IMU、`results/` 处理结果、`manifest.json`、
  `calibration.yaml`、`rectification.npz`、`events.jsonl`。
- 导出 JSON（完整结果）、CSV（一行一个关节）、NPZ（数组，无 pickle）。
- 新旧 schema 兼容读取，非有限值在 JSON 中为 `null`、在 NPZ 中为 `NaN`。

**VIO（实验性）**
- `src/egohand/vio`：IMU 格点重排、位姿质量门控、轨迹、hand_world 变换、native bridge。
- 离线 VIO 回放：对同一份录制反复回放不同 IMU 对齐参数，无需重新采集。
- 依赖外部 ORB-SLAM3（GPL-3.0）与厂商 SDK，均**不随仓库分发**，未完成实时验收。

**仍待验收或未实现**：真实硬件米制精度验收、实时 IMU 融合验收、物体/机器人协议实现和训练标注生成。MANO 拟合与世界坐标手部绑定已提供实验性实现，效果仍需按实际录制和设备验证。

## 技术栈

| 层 | 组件 |
|---|---|
| 语言 | Python 3.12（`>=3.12,<3.13`） |
| GUI | PySide6 6.8.3、pyqtgraph 0.13.7、PyOpenGL 3.1.9 |
| 视觉 | opencv-contrib-python 4.11.0.86、MediaPipe 0.10.21、NumPy 1.26.4、SciPy 1.14.1 |
| 可选 | jax/jaxlib 0.4.35、onnxruntime 1.31.1、rtmlib 0.0.16、av 19.0.2 |
| 工程 | setuptools（src 布局）、pytest 8.3.5 + pytest-qt 4.4.0 |
| 原生 | C++（VS030 SDK 探针、ORB-SLAM3 native bridge），仅 Windows |

## 目录结构

```text
configs/                  应用、检测器、双目几何、标定示例、ROI、MANO、VIO 配置
models/                   模型说明（模型本体不随仓库提交，见 models/README.md）
docs/                     架构、坐标系、数据格式、接入、使用、验收与性能/基准报告
design-system/            GUI 设计规范记录
samples/synthetic/        12 类已生成合成场景（真值 + 二维投影 + 标定）
samples/public/           公开 CC0 真实照片的来源记录（图片由脚本下载）
src/egohand/
  main.py                 GUI 入口
  core/                   类型、状态、无 GUI 引擎、协议
  sources/                合成 / 录制 / 图像 / HOT3D / SHOW3D / DexYCB / VS030 数据源
  calibration/            标定读写、校验、pinhole/fisheye 校正
  detection/              MediaPipe 双手检测、21 点定义、RTMPose、ROI
  stereo/                 歧义门控、Hungarian 关联、三角化、质量与时序验证
  visualization/          二维叠加与可交互 GL 骨架
  gui/                    面板、工作区布局、有界队列后台 workers、主题
  storage/                原始/结果分层、异步录制、恢复、回放与导出
  benchmarks/             基准测试运行器、HOT3D/STB 指标
  vio/                    离线 VIO 回放、IMU 格点、位姿质量、native bridge
  future/                 IMU、位姿、物体/机器人等协议占位
scripts/                  安装验证、模型下载、样例、CLI、基准与验收脚本
tests/                    pytest 用例（缺少模型/数据资产时自动 skip）
```

不随仓库分发（已在 `.gitignore` 排除）：`third_party/`（GPL-3.0 上游代码）、
`tools/`（vendored 依赖与 CC BY-NC 工具）、`SDK/`（厂商交付包）、
`models/mano/`、`models/*.onnx`、`*.task`、`samples/date/`（第三方数据集）、
`recordings/`、`exports/`、`logs/`、`data/`、`results/`、`build/`、`handoff/`、`标定结果/`。

## 安装

要求：Windows 10/11 x64、Python 3.12（建议用 `uv` 或指定解释器创建隔离环境）。

```powershell
git clone https://github.com/foru233/EgoHandDataEngine.git
cd EgoHandDataEngine

# 方式 A：已有 uv
.\setup_windows.ps1 -DownloadAssets

# 方式 B：指定 Python 3.12 解释器
.\setup_windows.ps1 -PythonExe C:\path\to\python312\python.exe -DownloadAssets
```

`setup_windows.ps1` 会：创建 `.venv` → 按 `requirements-lock.txt` 安装 → `pip install --no-deps -e .`
→ 运行 `scripts/verify_environment.py` →（`-DownloadAssets`）下载官方 Hand Landmarker 模型与公开图片。

手动安装等价于：

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
.venv\Scripts\python.exe -m pip install --no-deps -e .
.venv\Scripts\python.exe scripts\verify_environment.py
.venv\Scripts\python.exe scripts\download_models.py      # models/hand_landmarker.task
.venv\Scripts\python.exe scripts\download_sample.py      # samples/public/hand.jpg
```

依赖文件说明：`requirements.txt`（`-e .`）、`requirements-dev.txt`（pytest）、
`requirements-lock.txt`（完整锁定）、`requirements-rtmpose.txt`（可选 RTMPose）、
`requirements-vendor.txt`（可选 VS030 视频解码 `av`）。

## 环境配置

项目**不需要 `.env`**，也不读取任何密钥。可选环境变量均有默认值：

| 变量 | 默认 | 用途 |
|---|---|---|
| `EGOHAND_DECODE_HW` | `cuda` | VS030 视频解码后端（`cuda`/`cpu`） |
| `EGOHAND_VIO_CONFIG` | `configs/vio/vs030_rectified_preliminary.yaml` | VIO 子进程配置路径 |
| `QT_QPA_PLATFORM` | 未设置 | 无头测试时可设为 `offscreen` |

运行配置以 `configs/` 下的 YAML 为准（`app.yaml`、`detector.yaml`、`stereo.yaml`、
`calibration_example.yaml`、`roi.yaml`、`mano.yaml`、`configs/vio/*`）。

## 启动

```powershell
.\run_gui.bat                 # 或：.venv\Scripts\python.exe -m egohand.main
.venv\Scripts\python.exe -m egohand.main --demo --quit-after 5
egohand-studio                # 安装后提供的控制台脚本（等价于 -m egohand.main）
.\run_tests.bat               # pytest，结果写入 logs\pytest.xml
```

GUI 启动后：左侧选数据源 → 选「几何模拟」与场景 → 点「开始」即可离线跑通整条链路；
点「录制」按钮可把原始层与结果层落盘。

## 使用方法

### GUI 工作流

1. 数据源：几何模拟 / 录制或外部清单 / 真实单图（2D only）/ VS030 双通道预览 /
   HOT3D Clips / SHOW3D v3 双目 / DexYCB 多相机 RGB。
2. 需要立体重建时必须有有效双目标定与同步依据；缺失时只输出二维结果。
3. 「录制」页可设置保存位置，开始/暂停/停止录制；VS030 录制停止时自动生成 `replay_check`。
4. 处理结果可在界面查看，或用「查看已有结果」加载历史录制，或对原始数据做重新处理。
5. 导出：当前帧或批量导出 JSON / CSV / NPZ（GUI 还可导出叠加截图与 GL 骨架截图）。

### 无 GUI / CLI

```powershell
.venv\Scripts\python.exe scripts\api_example.py
.venv\Scripts\python.exe scripts\generate_synthetic_data.py --frames 12
.venv\Scripts\python.exe scripts\run_pipeline.py --scenario crossing --frames 120 --record recordings\my_session --output exports\my_session
.venv\Scripts\python.exe scripts\run_pipeline.py --recorded recordings\my_session --reuse-detections --frames 120 --output exports\my_reprocessing
.venv\Scripts\python.exe scripts\inspect_real_image.py
.venv\Scripts\python.exe scripts\benchmark.py
```

`scripts/run_pipeline.py` 参数：`--scenario`（12 选 1，默认 `two_static`）、`--frames`、
`--recorded`、`--reuse-detections`、`--output`、`--record`。
输出目录采用新建策略，不覆盖已存在的原始/导出目录。

### Python API

```python
from egohand import EgoHandEngine
from egohand.sources.synthetic import SyntheticStereoSource

engine = EgoHandEngine()
try:
    engine.start(SyntheticStereoSource("two_static", frames=1))
    result = engine.process_next()
    for hand in result.hands:
        wrist = hand.joints[0]
        if wrist.valid:
            print(hand.track_id, wrist.xyz_m)   # camera_left_optical, m
finally:
    engine.close()
```

外部程序可对左右图像、标定、原始时间戳与可选预计算二维点调用 `engine.process_stereo()`。
图像本身不足以证明同步：真实模式需要显式 `real_stereo` / `synchronization_evidence` 依据，
缺少依据或缺少有效标定时只输出二维结果。字段定义见 [docs/DATA_SCHEMA.md](docs/DATA_SCHEMA.md)。

## 输入输出示例

**输入：外部图像清单**（见 `docs/DATA_SCHEMA.md`）

```json
{
  "metadata": {
    "source_type": "recorded_real",
    "real_stereo": true,
    "synchronized": true,
    "real_calibration": true,
    "metric_capable": true,
    "synthetic": false,
    "synchronization_evidence": "USER_ASSERTED: ...",
    "timestamp_kind": "sensor"
  },
  "calibration": "camera.yaml",
  "frames": [
    {"frame_id": 0, "timestamp_ns": 1000000000,
     "left_timestamp_ns": 1000000000, "right_timestamp_ns": 1000000000,
     "left": "left/000.png", "right": "right/000.png"}
  ]
}
```

**输出：`export_all()` 生成 `results.json` + `results.csv` + `results.npz`**

- JSON：完整结果数组，字段含 `schema_version / frame_id / timestamp_ns / session_id /
  source_type / calibration_id / coordinate_frame / unit / reconstruction_status`，
  每个 `Hand3D` 固定 21 个 `Joint3D`；未匹配手 `track_id = -1`；非有限值为 `null`。
- CSV：一行一个关节，含 `x_m / y_m / z_m / valid / status / epipolar_error_px /
  reprojection_error_px / derived_quality_score / depth_std_m / position_std_m`。
- NPZ：`xyz_m [N,H,21,3]`、`valid [N,H,21]`、`status`、`track_id [N,H]`、
  误差/质量/不确定度数组，`coordinate_frame="camera_left_optical"`、`unit="m"`。
  `H` 是帧内槽位，不是跨帧身份；跨帧按 `session_id + track_id` 对齐。

**录制目录**：`manifest.json`（来源/算法/标定/状态）、`raw/`（无损图像或视频+IMU 原始流）、
`results/frames.jsonl`、`calibration.yaml`、`rectification.npz`、`events.jsonl`。

## 注意事项

- **真实硬件精度未验收**：真实 VS030 设备已完成部分采集链路（双目录制、时间戳与 IMU 载荷分析），
  但硬件同步精度、传输完整性与米制测量精度**没有**完成验收；软件测试通过不能推导设备精度通过。
- **不是完整训练数据**：输出是左相机光学的三维关节点，不自动包含世界坐标轨迹、物体状态、
  机器人动作、相机世界位姿或完整标注。
- **合成数据不是真实测量**：合成场景的 3D 真值只用于生成二维投影和测试比较，重建不读取真值。
- **模型与数据需自行准备**：见 `models/README.md`；MANO 模型、RTMDet/RTMPose 权重、
  ORB-SLAM3、SHOW3D 工具、厂商 SDK 均不在本仓库，且各有独立许可（见 LICENSE 末节）。
- **CPU 推理是已验证路径**，GPU 推理未实现或未验收。
- **仅 Windows**：GUI、录制与 native bridge 依赖 Windows 环境。
- 测试套件在缺少官方模型/公开图片/数据集资产时会 `pytest.skip`，属预期行为。

更多文档：[使用与 API](docs/USER_GUIDE.md) · [架构](docs/ARCHITECTURE.md) ·
[坐标系与标定](docs/COORDINATES.md) · [数据格式](docs/DATA_SCHEMA.md) ·
[录制与离线回放](docs/RECORDING_REPLAY.md) · [相机 SDK 接入](docs/CAMERA_INTEGRATION.md) ·
[验收报告](docs/TEST_REPORT.md) · [性能报告](docs/PERFORMANCE_REPORT.md) ·
[模型说明](models/README.md)。

## License

本项目自有代码以 **MIT License** 发布，见 [LICENSE](LICENSE)。

运行/构建所依赖的第三方组件不随仓库分发，各自遵循其原始许可：
ORB-SLAM3 与 manopth 为 GPL-3.0；SHOW3D 工具为 CC BY-NC 4.0（仅非商业）；
MANO 模型为 Max Planck 专有许可、禁止再分发；xdSDK 为厂商专有交付物。
若你分发包含这些组件的产物，需自行遵守对应许可证。

## 客户端与三维回放

客户交付界面通过 `userrr/run_client_gui.py` 启动，支持 VS030 录制、离线计算、回放、录制截取，以及 SMPL 人体和 MANO 手部网格显示。录制与计算默认分别保存到 `data/ori` 和 `data/count`。人体姿态为根据头部与手部观测估计的结果，并非全身实测；实机精度仍需验收。

另有 [ACE 联合重建离线实验入口](scripts/ace_joint/README.md)，可从原始 VS030 录制直接运行一次人体与手部联合拟合，生成可独立加载的回放缓存；无需先算旧算法。

SMPL 模型文件不随本仓库分发。请在 [SMPL 官方网站](https://smpl.is.tue.mpg.de/)注册并按照官方流程接受许可、下载模型，将中性模型放到本地 `models/smpl/SMPL_NEUTRAL.pkl`；该目录已被 Git 忽略。模型使用受 [SMPL 官方许可](https://smpl.is.tue.mpg.de/modellicense.html)约束。MANO、SMPL、厂商 SDK 和 ORB-SLAM3 等依赖各自适用其原有许可，商业使用须自行确认授权。
