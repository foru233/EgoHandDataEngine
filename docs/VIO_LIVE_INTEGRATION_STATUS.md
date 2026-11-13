# VS030 实时 VIO 接入状态（2026-11-09，更新）

## 本次进展

- 已按现有 ORB-SLAM3 Release / MSVC 工具链构建独立 C ABI 桥 `src/egohand/vio/native/bin/egohand_vio_bridge.dll`。入口 `scripts/build_vio_core_patch.cmd` 重建本次必要的 ORB 核心补丁和桥；`scripts/build_vio_native_bridge.cmd` 单独构建桥。DBoW2 使用现有静态库。GUI 的 `.venv` 在加载 NumPy 后遇到 Windows DLL 符号冲突，不能直接在 GUI 进程调用；独立 `egohand-vio` Python 可加载，后续实时部署必须使用隔离进程。
- 修复启动期的关键帧指针初始化和 IMU 状态判断：ORB-SLAM3 的 `GetTimeFromIMUInit()` 在未初始化时返回 0，不能用 `>=0` 判成功；查询前增加未初始化门控。Reset 重置桥内相对时间 epoch。
- 独立 `egohand-vio` 进程以 EuRoC MH_01_easy 的 160 帧双目图像和原始 IMU 调用 bridge，156 帧跟踪状态 OK 且位姿有限；首帧 IMU 未初始化，108 帧报告初始化已发生，VIBA 1 日志出现。结果见 `results/vio/native_bridge_euroc_smoke.json`，测试使用 EuRoC 原始图像及官方配置，由 ORB 内部校正，只验证桥和离线数据，不代表 VS030 已运行或定位精度达标。脚本 `scripts/smoke_vio_bridge_euroc.py` 不依赖该环境内损坏的 Python cv2。
- 对 VS030 原始 cam0/cam1 标定计算发现，按当前 GUI 的 cam0 左、cam1 右顺序校正，P2 的水平平移为正；ORB-SLAM3 的校正双目匹配需要正视差。`egohand.vio.vs030_geometry` 为 ORB 单独使用 cam1/channel 5 为左、cam0/channel 4 为右，并计算与 GUI 左相机及 IMU 的刚体转换；GUI 原双目顺序不变。该适配经过投影和外参一致性测试，尚未接入实时估计。
- `egohand.vio.device_timing` 新增设备采样时间门控：要求明确曝光时间语义、IMU 采样来源、同一时钟域、严格递增与完整帧间区间；缓冲溢出、丢样、混时钟均显式失败，记录首帧前丢弃样本。它不猜测时钟映射和时间偏移，目前未绑定未经验证的 VS030 视频流。
- IIO 采集启动前实际读取并检查设备尺度、时间戳时钟与扫描布局；两路时钟不一致、布局异常或缓冲区被其他程序占用时拒绝运行，不静默套用旧参数或打断已有采集。设备实测短采样陀螺 1101 条、加速度计 1118 条，0 非递增和队列丢弃；其时间语义仍只能标记为设备 IIO 时钟、来源待验证。

**实时 VS030 VIO、长时间相机—IMU 同步、GUI 位姿和手部世界坐标尚未完成验收。** 更新后的 SDK 提供设备时间戳，但其是否为曝光时间尚未确认；不能把 IIO 设备时间与主机解码完成时间直接相减，也不能将 Kalibr 时间偏移直接套到主机时间。请取得包含曝光时间字段、IMU 采样时间及其时钟域定义的厂商 SDK/源码或协议文档，或明确设备上带原始时间戳的实时输出接口，再继续接入。

## 反复初始化的时序排查（2026-11-09）

- 现场运行的 667 个状态帧涉及 38 个地图 epoch，日志多次出现局部地图跟踪失败与运动不足初始化重置，确认不只是界面显示问题。当前 Kalibr 的相机重投影误差均值约 2.1 px，标定采集时所报相机到 IMU 偏移约 -224 ms，不能直接假定它适用于新固件。
- 采集 35 秒转动数据，仅保存数值。校正图光流估计角速度与 IMU 候选角速度经标定旋转后的最佳相关位置约 -85 ms，相关系数 0.991；0 ms 为 0.935，-225 ms 为 0.864。单应旋转受平移和视差干扰，仅是初步证据，不是精确标定或对固件曝光语义的证明。结果 `results/vio/vs030_gyro_diagnostic.json`。
- `configs/vio/live_timing.yaml` 暂按观测使用 `t_imu=t_camera-85ms`，将 IMU 时刻变换到相机时钟后再分配帧间区间；设备时间戳仍作为采样基准，主机解码时间未参与融合。偏移支持调整与设回 0，归一化方向有单元测试。该值需要独立复测和厂商说明。
- 真实现场启用该暂定偏移的 50 秒运动试验有 400 帧视觉跟踪 OK、287 帧报告 IMU 初始化、5 个地图 epoch。与先前不同动作的试验相比不能单独归因。
- 用相同 162 张内存图（实际公共起始区间减少若干张）和相同 IMU 数据，分别以 0 / -85 ms 离线重放：两组均 125 帧 OK、2 个地图 epoch；IMU 初始化标志分别 80 / 71 帧。此对照未证明 -85 ms 改善建图稳定性；初始化帧数也不是精度指标。重放为 JPEG 内存压缩且非实时。结果 `results/vio/vs030_offset_same_input_ab.json`。
- 新固件静态场景条件下，使用 LK 正反向一致且正视差的 2872 个匹配，校正图纵向残差中位数约 0.33 px、P90 约 2.60 px；这是有筛选偏差的快速诊断，不证明标定或全部视场合格。
- 结论：已确认时间对齐存在明显待检偏差，已实现可调整的初步补偿；仍有地图重置与跟踪不稳定，尚不能声称根因已全部查明或完全修复。需要验证 IMU 载荷字段/单位、精确曝光时刻、标定质量，以及实时处理丢帧和运动激励。

## 静止时位姿跳变排查（2026-11-09）

- 用户反馈静止时轨迹跳变。对应运行日志包含 155 次“运动不足，重置初始化”和 27 次“局部地图跟踪失败”，说明定位初始化/跟踪反复不稳定；这不能单独证明每一次跳变的唯一原因。IMU 载荷含义、时间偏移和标定精度仍需验证。
- 新增 `configs/vio/live_quality.yaml` 和 `pose_quality.py`：仅当跟踪 OK 且 IMU 已初始化并连续稳定一段时间后，位姿才进入 GUI；时间倒退、地图切换、跟踪失效或相邻位姿的线速度/角速度异常时拒绝绘制并断开显示轨迹。这是异常门控，不修复算法本身的慢漂移，不证明定位精度，也不对场景深度或设备绝对位置设置限制。初始阈值为实验参数。
- 设备和独立 ORB 实时测试约 40 秒、收到 1115 帧，ORB 240 个状态均未完成初始化，0 个有效原始位姿、0 个显示位姿，无传输/进程错误。该测试没有自然出现有效原始跳变，因此不能证明门控拦截了真实跳变；单元测试以构造数据验证突变拒绝、恢复分段和时间/地图切换，GUI 测试验证原因提示。
- 每个会话新增 `logs/vio_vs030_live_*_poses.jsonl`，同时保留原始姿态、初始化/跟踪状态和门控结论，供后续对真实跳变进行分析；日志不包含图像。

## 实验性实时 ORB-SLAM3 与 GUI（2026-11-09）

- 用户确认相机与 IMU 共用采样基准。SDK 相机时间戳实测按微秒量级转换为纳秒；候选 IMU 数据按 `<QQ6f` 解析，假定第一时间字段对应前三个加速度值、第二字段对应后三个角速度值，分别按时间戳插值对齐。上述单位和字段语义尚无厂商协议核对，不能视为精度验收。未直接使用旧 Kalibr 时间偏移，避免叠加可能已处理过的偏移；旧标定是否仍适用需重新验证。
- `live_runtime.py` 将原始双目按 ORB 所需 cam1/cam0 顺序校正，使用独立 `egohand-vio` 进程通过 C ABI 执行 Stereo-Inertial；连续 IMU 包独立有界队列，时间非单调、缺口过大或溢出会停止 VIO。预览采集继续；暂停手部处理时定位继续接收实时数据。
- 轨迹页面新增“实时 VIO（实验）”开关，采集前设置；页面显示启动、跟踪状态、IMU 初始化状态和当前地图轨迹。ORB 跟踪地图重置/切换时使用新增地图 epoch 清除当前显示轨迹，不把不同坐标系拼成一条线。变换到原始 GUI 左相机使用显式外参，尚未接入手部世界坐标。
- 实机运动测试 `results/vio/vs030_live_motion_smoke.json`：约 45 秒采集、1273 图像帧，ORB 输出 195 帧状态 OK、19 帧近期丢失、2 帧丢失；其中 9 帧报告 IMU 初始化标志。SDK 统计 66418 包、0 坏包、0 CRC 错、0 图像/传输轮次丢失；日志仍显示多次地图重置，不能宣称稳定 VIO 或精度合格。一次先前运动测试检测到约 401 ms 候选 IMU 缺口并停止，缓冲改进后此次未复现，不代表长期不会出现。
- 重建 native bridge 加入 epoch 后，EuRoC 160 帧回归 156 帧有效；Qt 离屏真实相机工作线程测试收到 83 个状态帧、无 GUI 线程错误。没有真值轨迹、ATE/RPE 或跨运动条件的精度验收。

## 新版 SDK 的 GUI 双目输入（后续进展）

- GUI 的 VS030 数据源改用 `XdSdkStereoSource`：SDK 在独立 `xdsdk_stream_worker.exe` 中读取数据，GUI Python 使用现有 `av` 解码；自动选择唯一 USB link-local 设备。设备分辨率和镜像配置与原标定输入不一致时拒绝启动。
- 实机连续读取 12 组 1600×1200 双目图成功；另一次确认现有标定可生成两张校正图。用户确认的双目同步也用于新 SDK 的同序号、同设备时间戳配对；没有将这当成定量曝光同步测量。
- SDK 子进程临时启用 IMU 输出并在正常退出时恢复原有预览配置；实机收到通道 7 原始包。当前原始 IMU 随预览帧传递，预览队列可能丢旧帧，**不能用于 VIO 积分**。设备原始时间戳随帧保留，但 GUI 帧主时间戳仍为主机解码完成时间，不能作为曝光时间。
- 已增加可选的独立有界 IMU 包队列：溢出会显式失败，不静默丢包。候选 40 字节布局解析器仅供诊断；实机抽取 48 条候选记录，两路候选时间在本次样本中递增，仍未确认字段语义。独立 ORB-SLAM3 实时处理和 GUI 位姿已接入，参见上方实机测试；稳定性和准确度尚未验收。新版固件下标定的实际有效性和 VIO 精度均未验收。

## 更新固件后的 SDK 复测（2026-11-09）

用户确认相机已更新后，之前“SDK 不兼容”的结论只适用于更新前的数据流，不能用于当前固件。通过官方 `ControlClient::probe` 发现设备 `<device-ip>`，设备回复型号 VC020、固件标识 1.0.0.0；请以硬件厂商型号说明为准。当前 USB 网络可达，ADB 不在线，原依赖 ADB 和旧协议的 `VendorStereoSource` 不能假定仍可用。

- 官方 SDK 读取预览配置为 UDP、H.265、端口 45679；图像配置 1600×1200、镜像标志 0/0。尺寸一致不等于旧标定经固件更新后已验证有效。
- 读取配置并使用设备指定端口后，SDK 成功接收双目及设备原始时间戳。仅双目测试 217 组完整传输、434 幅图像，0 坏包、0 CRC 错、0 图像丢弃；通道 4/5 同序号时间戳相同。原始结果 `results/camera/vs030_xdsdk_updated_probe.log`。
- 随后临时增加 IMU 预览位并在停止时恢复原始预览配置：收到了通道 7 的 640 字节载荷，双目与 IMU 数据可同时经 SDK 输出；第二次复测日志 `results/camera/vs030_xdsdk_updated_imu_probe.log` 记录 407 个完整传输轮次、611 条交付，0 坏包、0 CRC 错、0 图像丢弃。日志不保存视频内容。测试末尾恢复设备配置返回成功。
- 观测视频时间戳每 30 Hz 帧约增加 33333，数值符合微秒增量的假设；部分 IMU 载荷按 40 字节分组可观察到数值合理的候选字段与纳秒量级时间，但这些只是诊断推测。SDK 声明时间单位由固件定义，未在交付资料定义通道 7 载荷、各字段坐标和单位、相机时间戳是否代表曝光起点/中点或其与 IMU 时钟的精确定义；不得将推测字段直接送入 VIO，也不得默认套用旧 Kalibr 时间偏移。
- 新固件下的 GUI 相机适配、SDK 实时解码、IMU 载荷和时钟语义、相机–IMU 联合标定、真实实时 VIO 与轨迹精度尚未验收。下一步先取得对应协议或固件源码明确以上语义，并实现 SDK 图像流适配与独立 VIO 子进程；之后对设备时钟连续性、同步、校正图像及 VIO 进行实测。

## 厂商 SDK 实机验证（2026-11-09）

- 用户提供的 `SDK/delivery` 为 xdsdk 0.2.0；随包说明针对 VC020 V2 协议。该协议的 `device_ts` 单位由固件定义，文档也未确认其代表曝光时刻或与 IIO 的共同时间域。
- 使用 `scripts/probe_xdsdk_vs030.py` 将当前 VS030 原始 UDP 数据包镜像给 SDK 的被动解析器，未存储图像、未更改固件：收到 2332 个数据包，2332 个判为无效，0 个完整图像帧。原始包的签名和帧头布局与 SDK 所描述的 V2 格式不一致。结果保存在 `results/camera/vs030_xdsdk_compatibility.json`。这只说明所给 SDK 无法解析当前实际数据流，不排除其他版本或其他接口可用。
- 因此不能从 V2 结构中套取 VS030 的 `device_ts`，也不能声称已取得曝光时间。下一步需要当前 VS030 固件对应的协议/SDK，明确图像曝光时间和 IMU 采样时间、单位、时钟域与重置行为。用户已确认双目同步，图像与 IMU 的时钟关系仍需独立确认。
- `scripts/prepare_vs030_vio_config.py` 已生成初步 ORB 校正配置；在独立环境中能初始化并正常关闭，尚未输入真实 VS030 双目和 IMU。该配置及标定噪声参数须经过真实运动和精度验证。
- SDK 验证后相关回归测试 29 项通过（VIO 几何、时间门控、IMU、相机、轨迹及位姿面板）。这不是全工程通过，也不是实时 VIO 验收。

## 当前结论

- 实时双目图像已可由 Windows 本机通过 ADB USB 网络直连获取，不依赖 E:\camera_2_up。
- 设备启动脚本 run_all.sh 会调用 cam_fsync.sh 30；现场 sysfs 实测 PWM period=33,333,333 ns、duty=16,666,666 ns、enable=1、polarity=normal。说明设备确实配置了约30Hz硬件触发。用户已确认双目同步；PWM 观测本身不提供曝光时间差的定量测量，图像与 IMU 的时间关系仍待验证。
- 已新增 src/egohand/sources/vendor_imu.py：直接读取相机 Android/Linux IIO 的 LSM6DSV16X 陀螺仪和加速度计，原始时间戳与 clock-domain 显式保留。
- 该 IMU 适配器已在真实相机上完成约 2 秒短时采集：陀螺仪 1045 条、加速度计 1062 条；两路数据均严格递增，0 丢弃、0 非单调。设备报告 current_timestamp_clock=realtime。已用持续 ADB shell 做80次时钟探测：预热后 host perf_counter 与设备 realtime 的估计偏移中位数约 -1,793,009,597,269,294 ns，往返延迟中位数3.181 ms、最低0.537 ms、偏移观测范围4.17 ms。它是初步时钟映射，不是相机曝光同步验收。设备自身 realtime 约为 Unix epoch 后20.9天（系统日期并未校准到现实日期），因此不能直接使用设备 wall-clock 日期。
- 适配器启动时保存当前 IIO ODR、buffer length、buffer enable 和所有 scan-element enable；停止时恢复。实测停止后恢复到原来的 15 Hz、buffer disabled、scan elements disabled。
- 新增 tests/test_vendor_imu.py，与现有相机测试一起：12 passed。

## 重要的时间戳说明

IIO 每条记录为 16 字节：3 个 little-endian int16 轴值、2 字节对齐填充、1 个 little-endian int64 时间戳。按该布局解析已用真实数据验证。陀螺仪和加速度计的时间戳应按时间排序/合并，不能按两个独立读取线程的数组下标配对。两路首个读到的样本相差约 39.16 ms，只反映本次两个独立缓冲区读取起点不同，不能单独解释为传感器间实际延迟。

当前设备 ODR 请求 480 Hz，IIO 时间戳增量约 2.17536 ms（约 460 Hz），与 Kalibr 报告中的 IMU update rate 459 Hz 接近；需要在连续静置/运动数据上复核实际频率与丢样情况。SI scale 从设备 sysfs 读取记录已确认：gyro 0.000610865 rad/s/LSB，accel 0.002392822 m/s²/LSB。

## 尚未接入 ORB-SLAM3 的原因（必须先解决）

1. 当前 VendorStereoSource 为完整 UDP bundle 解码完成时间 time.perf_counter_ns()，不是相机曝光时间戳。UDP bundle header 目前只证实包含包长度/分片信息，未发现可用曝光时间戳。
2. IIO 时间戳虽单调，但设备时间到 Windows perf_counter_ns 的偏移/漂移映射尚未实测。
3. Kalibr 标定的 timeshift_cam_imu 约 -0.22378 s，不能直接套用到“解码完成时间”；传输/编码延迟与抖动必须单独测量。
4. 当前 Kalibr camera reprojection residual median 约 1.1 px、mean 约 2.1 px；需要确认图像通道、左右顺序、图像方向、分辨率与标定采集时完全一致。
5. ORB-SLAM3 原静态库已经通过独立 C ABI bridge 完成 EuRoC 离线调用；实时处理进程和 GUI 通信尚未完成。原目录中的 ORB_SLAM3.dll 为 0 字节，不能当作可加载 DLL。
6. ORB-SLAM3 的 Stereo-Inertial 输入必须是同一时间轴的同步立体帧、正确的 rectified mono8 图像和按帧区间排序的 IMU measurements。不能将 host decode time 当成 exposure time 伪造 VIO。
7. egohand-vio 环境的 cv2 Python import 当前出现 OpenCV 5.0.0.93 gapi_wip_gst_GStreamerPipeline AttributeError；不要在此环境继续假定 Python OpenCV 可用，也不要破坏工程 .venv。C++ ORB-SLAM3 构建使用的是 Conda C++ OpenCV 4.11。

## 标定信息

标定结果/1/data3-camchain-imucam.yaml：
- 1600x1200
- Kalibr pinhole + equidistant（4 个畸变系数），当前 StereoRectifier 使用 OpenCV fisheye API。
- cam0 为用户确认的左路，channel 4；cam1/channel 5。
- baseline norm 约 0.06555 m。
- timeshift_cam_imu: cam0 -0.22378133675 s；cam1 -0.22379060798 s。
- T_cam_imu 是 IMU 到各 camera 的 Kalibr 变换；ORB 配置需按官方 IMU.T_b_c1 的坐标约定核对后再转换，不能仅按字段名称猜矩阵方向。
- Kalibr 报告重投影误差 cam0 mean 2.149 px / median 1.127 px；cam1 mean 2.097 px / median 1.091 px。
- IMU update rate 459 Hz；噪声参数来自 Kalibr YAML/报告，仍需确认单位转换到 ORB-SLAM3 要求的连续时间噪声/随机游走定义。

## 下一步顺序

1. 不启动 VIO，先完成设备 IIO clock 与 Windows monotonic clock 的往返映射测量，报告 offset、漂移、P50/P95 jitter。
2. 查找相机编码/SDK/固件是否能提供 sensor exposure timestamp；若无，则通过受控运动/闪光实验估计 transport latency 和 jitter，若 jitter 不达标，不进入惯性融合。
3. 用户已确认双目同步；如需精度验收，再定量测量曝光差和队列行为。
4. 复核 Kalibr 标定与当前视频输入方向、通道和分辨率；生成 ORB Rectified 配置，图像由现有 StereoRectifier 输出 mono8，避免 ORB 误用四参数 equidistant 畸变。
5. 已建立独立 C ABI bridge，并通过 EuRoC 离线冒烟测试；待设备时钟语义确认后进行实时数据试验。
6. 只有时间同步通过后，才将 IMU 按每个相机帧时间区间插值/合并为 ORB IMU::Point，调用 TrackStereo。
7. 最后将实时 T_world_from_body / T_world_from_camera_left 输出接入 EgoHandEngine，在相同 session、clock domain、calibration_id 下生成 world hands。丢帧、重定位、初始化和 tracking lost 状态必须显式输出。

## 本次测试记录

- tests/test_vendor_imu.py + tests/test_vendor_camera.py: 12 passed。
- 全量测试：267 passed、13 skipped、7 failed。失败集中于已有 GUI/offscreen OpenGL、Vendor 旧占位测试预期、HOT3D GUI 原图恢复、Windows 临时目录 PermissionError 和一个 triangulation far-point 断言。完整日志由本机本次 pytest 输出提供；这些失败尚未逐项修复，不得宣称全工程 PASS。

## 录制与离线回放（2026-11-10 增补）

- 实机 IMU 对齐 A/B 不再需要每次手动转动相机：GUI 录制面板（右栏“采集”页）可选保存位置，一键录制**双目 H.265 视频 + 原始 IMU**；停止时自动生成 `replay_check`（可解码帧、配对时间戳、IMU 行数/速率），见 `docs/RECORDING_REPLAY.md`。
- 离线回放复用与实时相同的 `egohand-vio` 子进程、native bridge、`prepare_vs030_vio_geometry`、`PoseQualityGate` 与 `CandidateImuBuffer`/`LatticeImuBuffer`，按录制顺序确定性逐帧送桥；输出 `poses.jsonl`/`replay.json`，GUI 与 `scripts/replay_recording_vio.py` 均可触发。
- 实机 31.6 s 录制样本（834 对双目、IMU 13281 行/479.1 Hz）已通过 replay_check、视频导出与 8 帧回放冒烟；静止数据 `ok=0` 属预期。`device_ts`/IMU 载荷语义仍未获厂商确认，回放结论只能作相对比较。

## 双目 VIO 地图保留与短时跟踪恢复（2026-11-10）

- 双目已有视觉地图时，以 IMU 原点的关键帧位移检查是否达到 5 cm，再尝试惯性初始化。初始化后，短时小位移保留地图。惯性优化预热阶段丢跟踪时，给当前地图最多 1 秒视觉重定位窗口，之后按原流程处理持续丢失。
- 修复校正双目无特征帧缺少 IMU 处理所需状态，以及没有有效双目匹配时访问空结果的问题。11:06 录制第 99–103 帧双目置黑，IMU 和时间轴照常回放。旧版在第 100 帧报错退出，新版第 106 帧找回原地图并跑完 260 帧。
- 使用 GUI 的 800×600 配置和包头格点 +97 ms，三段录制逐帧对比。图像帧序号、相机时间戳及每帧 IMU 样本数一致，位姿使用相同质量门控。

| 录制 | 处理帧 | 地图切换（旧 → 新） | 可用位姿（旧 → 新） |
| --- | ---: | ---: | ---: |
| 11:06 | 1841 | 8 → 0 | 831 → 1649 |
| 11:07 | 957 | 6 → 0 | 210 → 835 |
| 11:20 | 974 | 9 → 0 | 56 → 844 |

- `scripts/build_vio_core_patch.cmd` 应用原生补丁后重建 GUI 和回放共用的桥。回放明细见 `results/vio/tracking_recovery/summary.json`。EuRoC 160 帧回归前后均为 156 帧跟踪 OK、108 帧已初始化。VS030 结果衡量录制回放的稳定性，实机运行时的处理帧率和绝对定位精度仍待验证。

### IMU 初始化后的重定位崩溃

- 13:40 和 13:41 两次实时日志均在重定位后记录了原生空指针访问。关闭定位进程时又抛出 C++ 异常，GUI 因此显示 `0xe06d7363`。清理失败现在作为附注写入子进程日志，GUI 显示最初的错误。
- GUI 配置分支补上了重定位后的视觉跟踪窗口。恢复惯性优化时，上一帧缺少惯性约束就从最近关键帧重建；缺少有效预积分时继续用视觉优化。移除了未被使用的重定位帧副本。
- 11:20 录制在初始化后置黑 5 帧，旧版复现了同样的空指针访问。连续遮挡还抓到后台 `KeyFrameCulling → SetBadFlag → ChangeParent → AddChild` 的原生调用栈。局部建图现在保留尚无上级节点的关键帧，直到它具备安全裁剪所需的连接。
- 修复后，单次遮挡、两次连续遮挡的两轮重跑，以及抽帧到 15 Hz 的遮挡回放，均完成 340 帧并保留原地图。原始录制完整回放 974 帧，964 帧跟踪 OK，零地图切换。EuRoC 回归 156/160 帧跟踪 OK。实机复测待进行。

### 实时 VIO 词典常驻

- 同一个 GUI 窗口在首次启动实时 VIO 时加载词典。停止采集后保留 ORB 进程，重新采集时复用词典并重置地图、时间原点和 IMU 预积分。关闭 GUI 时结束进程，ORB 配置文件变更时重建进程。
- 11:20 录制通过实时链路连续启停三轮，每轮处理 140 帧。三轮使用同一个 PID，词典只加载一次，每轮首帧均未初始化且地图标识不同。首次就绪 18.447 秒，之后分别为 0.299 秒和 0.312 秒。
- ORB 原生日志写入 `logs/vio_vs030_worker_*.log`，其中标记了每个会话。每次采集仍各自保留位姿日志和原始 IMU 文件。
