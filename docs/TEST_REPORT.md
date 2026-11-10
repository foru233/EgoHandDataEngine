# M01–M05 验收报告

日期：2026-11-03（Asia/Shanghai）。环境：Windows 11 x64，i7-13700KF，31.84 GiB RAM，RTX3050 6GiB；CPU 推理。
后续界面更新已完成独立验收：73项完整回归通过，含多尺寸自适应图像占用率/比例测试；先前多窗口GL警告未再出现。最新证据 `logs/ui_redesign_full.xml` 和 `docs/UI_REDESIGN.md`，以下保留初次交付记录。
Python 3.12.15，MediaPipe 0.10.21，OpenCV contrib 4.11.0.86，PySide6 6.8.3。完整机器信息见 `logs/environment.json`。

## 自动化结果

已实际执行：`.venv\Scripts\python.exe -m pytest -q --junitxml=logs/pytest.xml`。
最终完整运行：**72 passed，0 failed，0 skipped**，5.84 秒。XML 见 `logs/pytest.xml`。
停止桌面操作后的最终命令行复核：**65 passed，0 failed，0 skipped**，3.13 秒；涵盖全部算法、数据源、导出、API 与真实模型测试，排除会创建窗口/GUI worker 的用例。证据 `logs/final_headless.xml`。未用该次无窗口复核冒充共享 GL 上下文的界面复验。
已执行 compileall；`uv pip check` 检查 41 个已安装包，无依赖冲突。
模型及真实图片均在本机存在，IMAGE/VIDEO 真实推理测试实际执行，没有以 skipped 冒充 PASS。

覆盖：标定维度/非有限值/单位/焦距/旋转/基线/尺寸/来源；pinhole 和 fisheye；不同左右内参与非平行相机；垂直双目；校正幂等；无噪声与含噪几何；零视差/负深度/远距离/非有限点/极线和重投影拒绝；双手顺序/交叉/缺失/标签错误/匹配歧义/身份歧义/过期 ID/时序关节跳变；原始录制回放/截断尾部恢复/视频 CSV/时钟偏移/无时间视频禁用同步；实时丢帧与独立原始录制/离线不丢帧/写盘过载与失败/暂停录制事件；JSON/CSV/NPZ 与 NaN/null；无 GUI API；GUI 启停/重启/源切换/已有结果图像/定位重置；真实模型、颜色、21 点索引、独立实例、VIDEO 时间单调和无手场景。

端到端用例真实调用 SyntheticStereoSource → 校正 → Hungarian 关联 → OpenCV 三角化 → 验证 → 录制 → 重读 → 与 Ground Truth 比较，没有复制真值作为输出。

## 原生 GUI 证据

脚本 `scripts/gui_evidence.py` 在 `qt_platform=windows` 下实际打开 GUI，不使用 offscreen 平台。
结果 `logs/gui_evidence.json`：

- 左右视图均有图像，合成场景 42 个有效关节，48 个动态 GL 图元。
- GL 上下文有效；暂停、停止、重新开始成功；采集线程退出。
- 公共真实照片检测到 1 只手，状态 2D Only，无任何米制双目关节。
- 合成预览约 21.92 FPS，单次软件采集到处理完成延迟约 3.91ms，队列 0、推理丢帧 0。
- 本次原生证据 errors=[]。

截图：`logs/gui_synthetic.png`、`logs/skeleton.png`、`logs/gui_real_image.png`、`logs/gui_light.png`。
另通过 computer-use 检查实际窗口树和画面，发现并修正状态条高度及三维默认视角问题。
用户按物理 Escape 停止 Computer Use 后，未继续桌面操控。窗口旋转/缩放/平移/拾取代码已接入，但本次没有完成所有鼠标交互的人工逐项验收，不能把代码存在作为这些手工项目全部通过的证据。

## 按阶段验收

| 阶段 | 软件状态 | 实际证据 | 独立硬件条件 |
|---|---|---|---|
| M01 | PASS | Windows GUI 实际启动，双图/GL 显示、线程启停/重启自动测试和原生截图 | 设备预览 BLOCKED_BY_HARDWARE |
| M02 | PASS | 三类源接口、12 场景、PNG 录制回放、双视频/CSV 测试；SDK 占位明确报错 | SDK/硬件帧/IMU 采集 BLOCKED_BY_HARDWARE |
| M03 | PASS | pinhole/fisheye/不同内参/非平行及垂直双目测试；标定校验与来源门控 | 厂家标定与实测极线 BLOCKED_BY_HARDWARE |
| M04 | PASS | 官方模型真实加载；公开照片实际输出 21 点；IMAGE/VIDEO/双实例测试 | 设备双视图实际检测 BLOCKED_BY_HARDWARE |
| M05 | PASS | 双手关联、退化拒绝、身份歧义、三角化、真值误差比较、导出回读均实际执行 | 真实设备米制精度 BLOCKED_BY_HARDWARE |

这些 PASS 表示目前可验证的软件路径通过；不表示未知相机已接入，也不代表所有人工交互都做完。

## 模型与真实资产

官方 Hand Landmarker float16/version1，SHA256：`fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1`。
首次 canonical 域名 TLS 超时，改用同一 Google Cloud Storage 桶的官方子域名成功下载；文件为真实 Tasks 包。
来源：<https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task>。
照片：<https://pxhere.com/en/photo/990733>，来源页面声明 CC0；摘要、下载时间见 `samples/public/provenance.json`。
实际二维检测结果在 `logs/real_image_detection.json`，叠加图在 `logs/real_image_detection.jpg`。
图片存在遮挡，模型输出不是每个关节准确性的独立证明。没有将该单图作为一对双目输入生成米制结果。

## CLI 与持久数据证据

`recordings/cli_acceptance`：24 帧双手交叉原始数据及处理结果。
`exports/cli_acceptance`：24 帧、1008 个有效关节的 JSON/CSV/NPZ。
`exports/cli_reprocessed`：实际从上述原始记录重新处理得到 24 帧、1008 个有效关节；独立会话，未覆盖原始数据。
12 类生成场景见 samples/synthetic，每类 12 帧，包含 Ground Truth 和二维投影。

## 问题修复与剩余复核

1. 初始非对称畸变测试暴露模拟器右视图误用了左畸变，已修复为逐视图参数并加入 fisheye 投影；后续相关测试通过。
2. 预置录制命令在采集后才执行会漏掉开头帧，已改为启动采集前处理命令；25 帧高频输入的原始录制完整、推理丢帧测试通过。
3. 同一可靠旧身份的竞争或近等价关联会显式标记 IDENTITY_AMBIGUOUS，停止发布受影响的有效关节，保留旧轨迹至超时；测试通过。
4. MediaPipe/protobuf 有两条 Python 3.14 相关弃用警告；本项目限定 Python 3.12。
5. 最终完整测试出现一条 PyQtGraph 多窗口 shader/context `glUseProgram` 警告，测试仍 72/72 通过；此前单窗口原生 GL 证据有效。随后在应用、GUI 验收脚本和 pytest 初始化前加 `AA_ShareOpenGLContexts`，该修改尚未再次打开桌面复核。应在下次允许桌面操作时运行 `run_tests.bat` / `gui_evidence.py` 复查。
6. 手工旋转/平移/拾取、不同 DPI/缩放、长时间写盘/断电恢复及真实设备压力未在本次完整人工验收。已提供对应实现和可复核入口，不能以本报告代替实际设备验收。

未实现功能：真实 Vendor SDK、IMU 融合、VIO/SLAM、世界位姿、WiLoR、MANO、物体跟踪、机器人映射、训练集标注与 MCAP 写入；仅提供接口。
