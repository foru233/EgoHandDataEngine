# VIO 阶段记录（进行中）

- 主机：本地 Windows；未使用 WSL、Docker 或其他主机。原有 `.venv` 未修改。
- 两份 bag 的 SHA-256 已与 `docs/VIO_DOWNLOAD_HANDOFF.md` 记录核对一致。输入保持只读。
- 官方 ORB-SLAM3 源码位于 `third_party/ORB_SLAM3`，提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`；GPLv3。已在 Windows 构建并运行，补丁及评测见 `VIO_BENCHMARK_REPORT.md`。
- 独立 Conda 环境 `egohand-vio`：Python 3.11，rosbags 0.11.9、evo 1.38.1、OpenCV Python headless 5.0.0.93、CMake 3.29.6、Ninja 1.13.0。Python wheel 不用于 C++ 链接。
- Visual Studio 2022 Build Tools 17.14、MSVC 19.44，CMake 3.29.6、Ninja 1.13.0 已使用；C++ OpenCV 4.11 和 Eigen、Pangolin 等来自独立 Conda 环境。vcpkg 构建受生成文件解析错误阻塞，最终未用于链接。
- `logs/vio_bag_initial_inspection.log` 为真实消息头与 topic 检查。`scripts/convert_vio_bag.py` 将真实图像和 IMU 写入 `samples/date/VIO/converted/`，不使用 GT 辅助处理。
- EuRoC：3682 对 752×480 mono8 图像、36820 条 IMU；TUM-VI：2228 对 512×512 mono16 图像、22212 条 IMU。两侧图像 header 时间戳完全一致，所有传感器时间戳严格递增。首、中、末各两路图像与 bag 原像素完全一致。
- TUM-VI 保留 16 位 PNG；官方示例 `IMREAD_GRAYSCALE` 以 8 位读取，首中末帧动态范围与标准差保存在 conversion_report。图像质量尚未通过 SLAM 特征跟踪评估。
- TUM-VI 第一帧早于首个 IMU；`timestamps_inertial.txt` 单独跳过该帧以避免官方示例索引越界，原始完整图像和 `timestamps.txt` 未删改。两数据集完整的帧间 IMU 计数见 `imu_interval_audit.json`。
- 转换目录包含从固定提交复制的官方 Stereo-Inertial YAML。后续已从 TUM-VI 校正 bag 提取官方动捕话题，按论文核对其时间及 IMU 坐标系；EuRoC 使用 OpenVINS 真值副本，来源及时间误差仍需独立核对。精度结果和限制见 `VIO_BENCHMARK_REPORT.md`。
- 原项目回归 `logs/vio_existing_regression.log`：236 通过、1 跳过；数据检查 `tests/test_vio_converted_data.py` 通过。
- 当前仍未完成：GUI 位姿绑定及真实同一会话的手部与 VIO 验收、完整的实时状态和性能统计。引擎已支持显式绑定和独立世界手部 JSON 导出；现有 GUI 位姿页只查看离线惯性轨迹，不声称与手部同步。

注意官方 `System::SaveTrajectoryEuRoC` 的输出并非始终为相机轨迹：视觉惯性模式输出 `T_world_from_body`，纯视觉模式输出 `T_world_from_camera`，时间戳以纳秒写出。评测及手部坐标转换必须分别处理 IMU–左相机外参，不能将两种文件直接当作同一种位姿。
