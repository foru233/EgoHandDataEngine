# 架构与实施边界

`StereoDataSource → StereoFrame → CalibrationProvider → StereoRectifier → 两个独立 HandDetector → StereoHandAssociator → JointTriangulator → ReconstructionValidator → StereoHandResult`。

算法位于 `core/calibration/detection/stereo`，不依赖 Qt。GUI 只选择数据源、发出控制命令与绘制结果。
MediaPipe 使用官方 Tasks API；三角化使用 OpenCV；分配使用 SciPy Hungarian，包含显式未匹配代价。
逐一禁用最优匹配边再计算全局替代解，若代价差小于阈值则拒绝争议配对。handedness 仅为软约束。
几何候选检查极线、有效视差、左右正深度、重投影、深度范围、宽松骨长；时序关联检查运动范围与歧义。
不能凭低重投影误差断言匹配正确。视差与从视差得到的深度仅作相关几何门控，不作为独立可信证据重复相乘。

## 线程和数据所有权

主线程：Qt 交互和绘图。PipelineWorker：模型初始化、推理、几何处理。独立 Capture 线程：读取数据源。
AsyncRecorder：独立写盘线程。主线程用 33ms 定时器读取容量为 1 的最新预览邮箱，避免 Qt 信号积压。
处理队列默认 2 帧：实时移除最旧待处理帧并计数；离线有界等待，不主动丢弃。
原始帧在进入推理队列前提交录制，因此推理丢帧不会自动丢失原始帧。
录制队列默认 32 项，实时过载显式停止并标记 incomplete；离线等待。录制暂停不会暂停数据采集。
未来 SDK 的 read() 必须有有限超时，stop/close 必须可释放阻塞，禁止只依赖 daemon 线程退出。
数据源交付的图像与 landmarks 应在交付后保持不变；厂商复用缓冲时适配器必须复制或完成所有权转移。

源切换、定位、循环回到开头重置算法会话与两个 VIDEO 检测器。track ID 只在 session_id 内唯一。
Qt 入口在 QApplication 创建前设置 AA_ShareOpenGLContexts，适配 PyQtGraph 跨窗口 shader 缓存；此次最终小修复的桌面复核尚待进行，见 TEST_REPORT。
默认轨迹缓存保留 0.5 秒，恢复需唯一关联，身份不确定时新建 track，禁止静默交换旧 ID。
暂停后处理队列可能继续排空；单帧命令只放行一对新采集图像。

## 数据与扩展

原始 PNG/JSONL 与结果 JSONL 分层保存，重新处理生成新会话和新输出目录。写盘每行 flush，启动时恢复完整 JSONL 行并报告截断尾部。
这提供进程中断后的完整行恢复，不承诺断电级事务原子性。不可解析的中间行拒绝静默跳过。
未来 DatasetWriter 可增加 MCAP 实现；算法核心不引用文件格式。
IMUProvider、CameraPoseProvider、TemporalHandOptimizer、MANOHandReconstructor、ObjectTracker、RobotRetargetingAdapter 仅定义协议。
本阶段无 VIO、SLAM、世界轨迹、MANO 拟合或机器人控制实现。

## 参考与许可证

- MediaPipe 官方：<https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python>
- OpenCV：<https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html>
- PyQtGraph：<https://pyqtgraph.readthedocs.io/>
- MCAP：<https://mcap.dev/>
- StereoHand：<https://github.com/NavehBrenner/stereohand>，已核查 master/LICENSE 为 MIT；没有复制其代码，使用官方 OpenCV 三角化实现独立双手匹配逻辑。

公开照片来源与 CC0 声明见 `samples/public/provenance.json` 及 <https://pxhere.com/en/photo/990733>。
