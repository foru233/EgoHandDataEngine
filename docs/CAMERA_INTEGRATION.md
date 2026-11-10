# 未来真实相机接入与架构审查

目前真实相机型号、SDK 和厂商标定未知；VendorStereoSource.open() 明确报“真实相机适配器尚未配置”。
没有伪造 SDK、DLL、设备帧、IMU 或厂商标定。硬件验收为 BLOCKED_BY_HARDWARE。

## 收到设备资料时先查什么

1. 精确型号/固件、Windows SDK 版本/位数、许可证、官方 Python/C API、DLL 依赖与可用枚举示例。
2. 双目是否是同时曝光的两路独立相机；硬件同步/帧号含义、丢帧语义、曝光起止与 rolling shutter。
3. 左右传感器时钟、SDK 到达时钟、IMU 时钟之间的关系，时间戳单位/回绕/复位/偏移校准依据。
4. 分辨率、裁剪、缩放、旋转、镜像、像素格式、stride、Bayer/YUV 转换规则、缓冲生命周期。
5. 原始或已校正图像；pinhole/fisheye 模型、畸变系数顺序、内参对应分辨率、外参方向和长度单位。
6. SDK 是否返回 R1/R2/P1/P2/Q，已校正图像是否与本工程参数完全一致。不能重复校正。
7. IMU 轴向、单位、频率、标定与左右相机的外参；本阶段只采集和保留，不融合。

## 修改范围

主要实现 `sources/vendor_placeholder.py` 的 VendorStereoSource 或在该包新增真实厂商适配器；
提供 CalibrationProvider（可由 source.get_calibration 实现），可选独立 IMUProvider。
在 GUI 数据源工厂注册适配器。标定解析新增专属转换器，输出现有 StereoCalibration 契约。
MediaPipeHandDetector、StereoHandAssociator、JointTriangulator、ReconstructionValidator、StereoHandResult、显示组件和 Exporter 可复用。
若 SDK 输出本工程尚不支持的投影模型或不可还原的专用校正格式，需要新增几何适配，不能承诺所有未知 SDK 都零修改。

## 适配器实现步骤

初始化 SDK → 枚举/选择设备 → 开左右流 → 独立开 IMU 流 → 有超时 read → 验证同步并按硬件配对 → 转 uint8 BGR → 返回 StereoFrame。
source metadata 必须明确 real_stereo/synchronized/real_calibration/metric_capable/contains_imu，禁止按名称猜测。
frame.metadata 必须携带 real_stereo=true、synthetic=false、synchronization_evidence、timestamp_kind。
原始硬件 frame ID/同步编号、各路 sensor exposure timestamp、SDK arrival timestamp 和 clock domain 分别保存，不能覆盖 timestamp_ns 代替原始字段。
timestamp_ns 作为处理时间轴映射须有可追溯的转换说明。IMU 独立流保存 IMUSample，不强行一帧配一个样本。
厂商复用图像缓冲时复制后再交付；错误恢复保留 discontinuity 标记并重置时序实例。
stop 应停止 read 等待并在有界时间内退出；close 释放视频流、IMU 流、设备句柄与 SDK。

## 硬件验收

检查序号连续性与左右同步误差分布，不能仅凭相同 FPS；初始 5ms 软件门控不代表硬件同步质量已验证。
用真实匹配点核查极线及畸变，用已知距离/尺寸参考物多深度测误差和偏差。
测试双手交叉、遮挡、标签错误、无手、单视图漏检、快速运动、拔插与重连。
验证录制原始传感器时间、回放重处理、原始文件完整性和长时写盘压力。
记录真实分辨率、曝光、CPU/模型、读流/检测/几何 FPS、延迟、队列与丢帧。
只有这些实测通过后，才能在验收报告标记真实设备米制坐标能力。
