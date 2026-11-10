# SHOW3D v3 数据源（EgoHand Studio）

## 打开 GUI

在项目根目录 `%PROJECT_ROOT%` 使用 `run_gui.bat` 启动。

1. 顶部「数据源」选择 **SHOW3D · v3 双目**。
2. 默认路径是 `%PROJECT_ROOT%\samples\date\SHOW3D`。点击「选择 SHOW3D 目录」可选择整个数据集根目录，或 `scenes\<subject>\<scene>` 中的**单个已验收场景**。
3. 选择 **官方真值（GT）** 或 **算法预测（MediaPipe）**，点击「开始回放」；可暂停、单步、定位和循环。
4. 对于数据集根目录，只枚举已具备 `verification/<subject>/<scene>/validation.json` 与 `file_verification.json`、官方 v3 和 PinholePlane 标定完整、文件大小吻合的场景。正在下载/未验收场景不会加载；若下载新增场景，请停止并重新开始刷新列表。

## 数据和坐标约定

- 使用现有两路 `headset0.mp4`、`headset1.mp4`。官方头显图为 1024×1280 单色，OpenCV 解码为三个相同的 BGR 通道；这不是原生彩色图。
- 使用固定 SHOW3D 数据 revision `81403d3e4254cef959027118bc668bb2f26c86f6`，**显式**指定 `hand_pose_version="v3"`。
- 官方发布图已经去畸变，为 `PinholePlane`（无需 HOT3D FISHEYE624 变换），逐帧 `T_world_from_camera`；世界 XYZ 及其平移分量是 mm。
- 每帧由两组 camera-to-world 外参计算 right-from-left，再校正为 OpenCV 双目视图。三角化 XYZ 返回 **原始 headset0 左光学坐标系**、单位 **m**；叠加的 2D 坐标基于**校正后图像**。
- UmeTrack 21 个点含独立的 palm center（索引 20），MediaPipe 21 个点的拓扑不同：按已核对的 HOT3D canonical20 → MediaPipe 对应关系映射 **20** 个关节，不伪造缺失的 MediaPipe THUMB_CMC。
- 官方帧时间戳记录于数据源 metadata；处理 API 用单调回放 tick。官方公开帧索引只证明数据集配对，**不代表已验证两台设备的硬件同步误差**。
- 动态逐帧相机标定目前会重建校正器及跟踪状态：适合单帧真实尺度 3D 验证，不应把跨帧 ID 连续性视为经过验证。

## 模式和评测

**GT 模式**直接展示官方 v3 UmeTrack 世界 3D 标注变换后的原始左相机 m 坐标及两路重投影。**它不是模型预测，也不是三角化得到的测量**，因此不应标记重建精度。

**MediaPipe 预测模式**先对已去畸变的单色针孔帧进行极线校正，再在校正图上独立检测，并运行左右关联和三角化。GT 从未作为检测观测输入。

预测结果可选做 **GT MPJPE (mm)** 后验比较：只有左右手标签唯一、无身份歧义、存在有效预测 3D 和对应官方 GT 的关节才计算；不做尺度对齐、刚体对齐或补全。标签匹配并不能证明身份匹配正确。无有效比较时为 **N/A**，不能记为 0 mm。评测详情在 `diagnostics.show3d_gt_comparison`。

**局限：** 几何投影在画面内不代表无遮挡；有 GT 不代表 MediaPipe 一定检出。检测/关联不成功时 3D 可能为 0 有效关节，须调优算法与成像参数，不能反过来注入 GT。逐帧校准/映射有一定计算开销。

## 工程文件

- `src/egohand/sources/show3d.py`：数据源和 GT / 预测处理分离
- `src/egohand/gui/source_panel.py`：SHOW3D 数据源选择
- `src/egohand/gui/main_window.py`、`workspace.py`、`workers.py`：GUI 和工作线程接入
- `tests/test_show3d_gui.py`：实际本地场景、GUI、GT 与预测数据隔离
- `scripts/smoke_show3d_prediction.py`：真实 MediaPipe smoke
- `scripts/smoke_show3d_gui_e2e.py`：Qt GUI 实际帧展示 smoke
- `logs/show3d_gui_backup/`：改动前原文件备份

当前修改仅增加数据源接口和在原有管线中登记 SHOW3D 作为可明确配对的公开离线数据集；无模型训练、无下载任务改动。


