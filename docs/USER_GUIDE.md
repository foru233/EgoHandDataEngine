# EgoHand Studio 使用说明

## 安装与启动

在 PowerShell 中进入工程根目录。

```powershell
.\setup_windows.ps1 -PythonExe C:\path\to\Python312\python.exe -DownloadAssets
.\run_gui.bat
```

本机已使用 `<path-to-python312>\python.exe` 创建 `.venv`，没有修改该 Conda 环境的包。
可直接双击 run_gui.bat。若项目目录移动或 Conda 基础解释器被移除，需要重建 `.venv`。
新版使用自适应工作台：四个指标显示为紧凑单行；按窗口比例自动安排双目和三维区域，保留完整图像。顶部“配置”可收起右栏；三维区域的布局菜单提供自动、双目在左、双目在上。工作配置分为采集、录制、检测、区域、标定、坐标、诊断七个页签；录制面板独立成页，避免和数据源设置挤在一起。
依赖见 requirements-lock.txt；只有 opencv-contrib-python 提供 cv2，避免混装 opencv-python/headless。
GUI 默认深色，可在顶部切换浅色；Qt6 支持 Windows 高 DPI。拖动分隔条调整图像/骨架/属性区域。

## 几何模拟

数据源选择“几何模拟”，选择 scenario 后点击开始。默认 crossing，有两只不同深度的手，右检测列表顺序会变化。
画面始终有 SYNTHETIC 标记，状态为 Synthetic Ground Truth；算法实际经过匹配和三角化，没有直接复制真值。
可使用编号和极线开关，红圈标出几何无效二维观测。
暂停后可以单帧或定位，停止后可以重新开始。修改场景后停止并重新开始。
3D 左键旋转，滚轮缩放，中键拖动平移；重置视角恢复默认。点击有效关节显示原始左相机坐标。
GL 中方向轴为平移后的方向示意，网格间距 0.10m；导出不受 GL 显示变换影响。

## 真实照片二维检测

选择“真实单图 · 2D”，浏览图片，或留空使用 samples/public/hand.jpg，点击开始。
模型在后台加载；右视图为空，未把单图复制为双目。状态必须为 2D Only。
模型检测置信阈值和 num_hands 可在属性区调整，重启流水线后生效，也可改 configs/detector.yaml。
加载模型按钮设置路径，实际初始化发生在后台。缺失/损坏模型会显示 Model Not Loaded 或明确错误。
公开照片仅证明二维推理接口运行成功，不证明每个关节点都准确，尤其存在遮挡时。

## 录制与回放

【VS030 双目视频 + IMU 录制】数据源选“新 SDK / VS030”并开始采集后，右栏“录制”页的“录制会话”面板可以：
- 选择保存位置（默认 configs/app.yaml 的 recording_root，缺省 recordings）；新会话写到 `<位置>/session_时间_纳秒`，不覆盖旧会话；
- 开始/暂停/继续/停止录制；录制中显示帧、视频包、IMU 包、耗时、写入量；
- 停止后自动生成并展示 `replay_check`：可回放/不可回放、帧数、IMU 条数/速率；
- 点击“离线 VIO 回放…”对同一份录制反复回放（点“参数”展开对齐方式/时移/步长），**不需要再手动转动相机**。详见 [录制与离线回放](RECORDING_REPLAY.md)。

【通用录制】非 VS030 数据源点击录制保存到 recordings/session_时间（原始帧为无损 PNG，见 DATA_SCHEMA）。右侧面板可独立暂停/继续/停止录制。
推理队列丢帧会计数，原始录制独立。写盘过载或失败会停止录制并明确显示 incomplete。
暂停录制表示用户主动跳过该区间，应在使用数据时保留此操作语义；VS030 的暂停会在原始记录里留下时间缺口，做 VIO 回放时建议连续录制。
查看已有结果：点击“已有结果”选择原生录制目录，用定位或单帧查看保存的结果和对应图像，不重新推理。
重新处理：选择“录制 / 外部清单”，浏览 manifest.json，勾选离线完整处理，点击开始。
合成记录要复用保存二维投影点；真实图像可取消复用，运行新的 MediaPipe 检测。
重新处理使用新的 session_id；若需要持久保存，另行开始录制到新目录。原始记录不会被覆盖。
两个视频/外部序列通过 DATA_SCHEMA.md 的清单输入；FPS 一致不会自动开启米制重建。

## 导出

“导出当前帧”输出当前帧 JSON/CSV/NPZ 和视图截图。
“导出”在运行时输出最多 600 帧有界缓存；完整会话应先录制，然后“已有结果”加载完整结果后导出。
导出对话框显示实际帧数。输出为新建的带时间目录，已有目录不覆盖。
关节有效性必须结合 valid/status 判断；JSON null、NPZ NaN 都不是零坐标。

## 标定与错误

加载标定选择 YAML/JSON。格式见 configs/calibration_example.yaml，它明确为 synthetic，严禁用作实际相机参数。
真实双目需提供真实标定、同步依据和图像来源声明。只有加载一个标定文件不能使不明来源图像成为可靠双目。
初始深度 0.15–2m、同步容差 5ms、身份失联 0.5s 只为可配置初始门控；修改 configs/stereo.yaml 后重启生效。
缺少标定、同步依据不足、尺寸不符时保留二维功能，禁止米制重建。非法标定不会保留旧的有效标定。
原始传感器时间、时钟映射依据与估计时间必须分别记录。
源码日志位于 logs/studio.log；测试/性能/截图证据见 TEST_REPORT.md、PERFORMANCE_REPORT.md。

## 无 GUI API

运行 `.venv\Scripts\python.exe scripts\api_example.py` 可查看最简合成例子。

```python
from egohand import EgoHandEngine
from egohand.sources.synthetic import SyntheticStereoSource

engine = EgoHandEngine()
try:
    engine.start(SyntheticStereoSource("two_static", frames=1))
    result = engine.process_next()
    # result.hands[*].joints[*].xyz_m, valid, status
finally:
    engine.close()
```

外部图像使用 `engine.process_stereo(left_image, right_image, frame_id=..., timestamp_ns=..., ...)`。
传入已知 calibration、左右原始时间、synchronized、source_type、metadata，以及可选 left_detections/right_detections。
metadata 的真实模式必须显式包含 real_stereo=true、synthetic=false、synchronization_evidence；合成模式必须明确 synthetic=true 且使用合成标定。
没有预计算二维点时须提供两个独立 MediaPipeHandDetector 实例；不提供检测器时返回二维/未检测状态，不生成虚假关节。
逐帧调用中沿用同一个标定对象或在构造器设置，避免无意重置时序。
