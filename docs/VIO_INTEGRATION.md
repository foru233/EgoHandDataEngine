# 离线 ORB-SLAM3 使用和接口边界

当前实现用于离线研究验证，不提供实时相机–IMU 硬件接口，也未把手部帧自动绑定到 VIO 轨迹。

运行现有 `run_gui.bat`，先在“标定 → 相机—IMU 标定”加载 YAML，再在三维视图切换到“相机位姿”，导入 **Stereo-Inertial** 运行产生的 `CameraTrajectory.txt`。例如 EuRoC 可使用 `results/vio/euroc_inertial_full_20261104_144429/CameraTrajectory.txt` 和 `samples/date/VIO/converted/EuRoC/MH_01_easy/calibration/EuRoC.yaml`。调用方必须选择正确的惯性轨迹及标定；纯双目轨迹不可当成惯性轨迹导入。

轨迹中的世界系是 SLAM 局部世界，不是机器人的基座或真实场景全局坐标；GUI 中跟踪状态未知，且不自动将轨迹与采集界面正在显示的图像或手关联。

`egohand.vio.trajectory.load_euroc_trajectory` 显式区分纯视觉相机位姿和视觉惯性 IMU 位姿：前者必须提供导出相机系到原始左相机的变换，后者必须提供 `T_body_from_camera`。`OfflinePoseProvider` 按纳秒最近邻匹配，默认容差 1000 ns；查询不到位姿即返回不可用，不跨缺帧补位姿。由于官方示例以 double 秒处理时间并将其输出为纳秒，在约 10^18 ns 时间戳上存在约百纳秒的浮点量化。

`world_hands_for_result` 独立转换有效关节，不更改手部相机系结果或 Track ID。调用必须显式提供左图时间戳、会话和双方时钟域，标定 ID、坐标系和时间匹配均不满足时返回不可用。`EgoHandEngine.bind_offline_pose_provider` 可在当前会话显式绑定提供器，切换会话会自动解绑；此后输出增加 `world_hands`，`export_all` 会单独保存 `world_hands.json`。该绑定接口及世界坐标导出已经用合成手部帧进行集成测试，GUI 尚无可用的绑定入口，真实同一会话的手部 + VIO 数据尚未验收。需先建立可靠的时间域、标定、位姿来源和轨迹版本，不能把 EuRoC 或 TUM-VI 位姿直接套到 HOT3D、SHOW3D 或设备图像上。

复现实验可在独立 Conda 环境运行 `scripts/run_vio_sequence.py`，选择 `euroc_stereo`、`euroc_inertial`、`tum_stereo` 或 `tum_inertial`。每次输出新目录并保存命令、运行日志、超时和退出状态。评测需激活 Conda 环境使其 `Library/bin` 在 PATH 中；脚本 `scripts/evaluate_euroc_inertial.py` 支持 `--dataset tumvi` 和 `--stereo`，以及 `scripts/extract_tumvi_gt.py`。真实数据输出和结果口径见 `VIO_BENCHMARK_REPORT.md`。
# 三维视图中的离线轨迹

在“标定”页的“相机—IMU 标定”区域先加载对应 YAML，再在主工作台三维视图右上选择“相机位姿”，可分别导入和勾选拟合 / 估计轨迹与真值轨迹；选择“手部重建”可切回。轨迹文件选择默认打开 `samples/date/VIO`。清除轨迹会保留标定；已有轨迹时需要先清除才能切换标定。此入口只适用于以 IMU / body 为输出坐标的双目惯性轨迹；纯双目相机系轨迹不能当成体坐标导入。

估计支持 ORB-SLAM3 纳秒时间戳文本 `*.txt` 和 TUM 秒时间戳 `*.tum`；真值使用 TUM 格式（秒、位置 XYZ、四元数 XYZW），并假定是与标定相符的 IMU / body 坐标。两者均使用 `IMU.T_b_c1` 转换为左相机位姿。两条轨迹有不少于三个非退化、时间相差不超过 5 ms 的匹配点时，显示使用 SE(3) 刚体对齐、不调整尺度；否则会显示未对齐，不能将两条原始曲线的相对位置当作误差。滑块以估计时间轴为准，真值只在 5 ms 内有匹配时标出当前点。导入的轨迹为离线结果，没有与当前手部视频建立同步。

