# VIO 数据下载与 Windows 构建交接（2026-11-04）
项目：%PROJECT_ROOT%，唯一执行主机：<build-host> / Windows 11。
**仅下载原始数据并执行文件级校验，尚未运行 ORB-SLAM3，未执行 ROS bag 的逐消息解码。**

## 真实已完成下载
1. `samples/date/VIO/EuRoC/MH_01_easy/MH_01_easy.bag`
   - 2,673,818,914 bytes；SHA256 `57f440ccd68ec8dc8f9461269f5909656b86198bac3adfd677b1fcc7a1428fa9`
   - 来源：公开单序列镜像 https://huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01 （原数据为 ETH Zurich EuRoC）
   - 本机原 ETH 逐序列下载链接 TLS 出错，ETH 新入口提供约 12 GB 的 Machine Hall 合集；未下载合集。
   - 已另下载 GT: `MH_01_easy_gt_openvins.csv`、`MH_01_easy_gt_openvins.tum`（来源 rpng/open_vins/ov_data/euroc_mav；逐文件哈希见 `gt_sidecar_sources.json`）。
2. `samples/date/VIO/TUM-VI/room4/dataset-room4_512_16.bag`
   - 2,346,890,121 bytes；SHA256 `8027451a7390efd51098a10949b91dae7c183d9daf1a09ab058e8169c55ecf9e`
   - 官方 https://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room4_512_16.bag
   - 另下载 `room4_gt_imu_frame.txt`（公开镜像）和 `room4_gt_openvins.csv`（rpng/open_vins），均仅作后验评估。
   - 官方链路断续，Range 续传成功；与公开镜像对比 0、335544320、1073741824、1572864000 字节处的 1MB 内容逐字节一致。
**总 bag 容量：5,020,709,035 bytes ≈5.02 GB（不含轻量 GT）。**
文件级校验：字节数匹配、均是 `#ROSBAG V2.0` 头、SHA256 已记录。
在 `download_transfer_validation.json` 中，两个 bag 均找到 `/cam0/image_raw`,`/cam1/image_raw`,`/imu0` 与类型标识；TUM-VI bag 发现 `vrpn_client` 字符串。这是字符串预扫描，**不是逐消息完整性/时间同步验收**。
TUM-VI 真值 sidecar 13,075 非注释行，首行列为秒级时间戳、XYZ、四元数。GT 坐标系和相机标定请按官方定义校对，不可直接视为 left optical。

## 本机软件环境事实
已有 Conda，原 `.venv` 为 Python 3.12、PySide6/OpenCV/MediaPipe，保持不动。
`git`、`cmake`、`cl`、`ninja`、`vcpkg` 未在 PATH/常见安装位置找到；未找到 VS Installer/vswhere。WSL 可执行文件存在但本机尚无 Linux 发行版，**本期不使用 WSL/其他主机**。
建议新建独立 Conda `egohand-vio`（Python 3.11）作 `rosbags` 数据转换与 `evo` 评测；C++ ORB-SLAM3 用原生 VS2022/MSVC、CMake、Git 及固定版本依赖，在 `third_party/orbslam3` 单独编译。官方 ORB-SLAM3 Windows 原生非主要支持平台，若编译需补丁，必须保留 diff 和上游 SHA，并确保不会破坏工程。

## Codex 待做（未完成，不可宣称通过）
1. 在隔离环境解析 bag 的全部连接/主题、消息数、时间戳、编码，随机解码首中末双目帧与 IMU；检查 GT 与传感器时钟域。
2. 仅转换所选两段序列到 ORB-SLAM3 示例实际要求的目录；保留原始 bag 只读，不双重去畸变。TUM-VI 为 512x512/16bit 鱼眼，参考 ORB-SLAM3 的 `TUM-VI.yaml` / KannalaBrandt8；EuRoC 参考官方 `EuRoC.yaml`。
3. 构建并在 EuRoC 先跑 stereo 再 stereo-inertial，TUM-VI 再跑 stereo-inertial；输出带时间戳轨迹、跟踪状态、ATE/RPE（仅正确坐标与时间对齐后），不任意尺度对齐。
4. 用独立 CameraPoseProvider / PoseBuffer 接入 EgoHand Studio；保留现有 M01-M05，新增轨迹不覆盖原有 left optical 3D。
5. 实施结果放 `results/vio`、`logs/vio`，新增测试。真实个人摄像头+IMU 仍待硬件验收。

相关脚本：
`scripts/download_vio_baselines.py`（可断点重试且不重复完整文件），`scripts/download_vio_gt_sidecars.py`、`scripts/validate_vio_archives.py`，`scripts/check_vio_build_env.py`。
