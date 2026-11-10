# ORB-SLAM3 原生 Windows 验证记录

官方源码 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`，GPLv3；未来发布、分发或商业集成应评估许可证义务。实际执行和日志均在本机，未使用 GT 辅助定位。

## 方法与范围

四组完整轨迹均正常退出。通过 evo 1.38.1 的 API 按 5 ms 最近时间匹配真值，在匹配位姿上做 SE(3) 刚体对齐；尺度固定为 1，未搜索时间偏移。RPE 是相邻匹配帧相对位姿误差，不能与其他时间间隔的 RPE 直接比较。对齐使用全部匹配位姿，因此这些数字不代表未对齐的绝对全局定位误差。

EuRoC 真值是 OpenVINS 发布的参考副本，仍需独立核对发布来源及时间误差；纯双目导出位姿为校正左相机系，先使用官方 `stereoRectify` 的 R1 和 `IMU.T_b_c1` 转为 IMU 系。TUM-VI 真值从本机已校正 bag 的 `/vrpn_client/raw_transform` 提取；[官方论文](https://cvg.cit.tum.de/_media/spezial/bib/schubert2018vidataset.pdf)说明校正 bag 的动捕位姿已转到 IMU 坐标系并同步至 IMU 时间，room 序列有全程动捕覆盖。TUM 纯双目用 `IMU.T_b_c1` 转换；惯性模式由官方导出 IMU 位姿。

| 数据 / 模式 | 输入帧 | 导出位姿 | 真值匹配 | 有效轨迹秒 | ATE RMSE m | RPE 平移 RMSE m | RPE 旋转 RMSE ° |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| EuRoC Stereo | 3682 | 3682 | 3638 | 184.05 | 0.05478 | 0.00551 | 0.05915 |
| EuRoC Stereo-Inertial | 3682 | 3678 | 3638 | 183.85 | 0.04432 | 0.00361 | 0.02322 |
| TUM-VI Stereo | 2228 | 2228 | 2177 | 111.35 | 0.02337 | 0.00689 | 0.41188 |
| TUM-VI Stereo-Inertial | 2227 | 2171 | 2121 | 108.50 | 0.00824 | 0.00364 | 0.37750 |

TUM-VI 惯性输入跳过早于首个 IMU 的首张图；日志记录初始化重置、51 帧被标记丢失，之后完成 VIBA 1 和 VIBA 2，并发生回环检测。导出位姿数不等于严格的实时跟踪成功帧数，尤其 ORB 可能以相对参考关键帧恢复历史轨迹。各运行输出及 `evo_metrics.json` 位于 `results/vio/*full_20261104_*`；失败与重试目录也原样保留。

TUM 示例的跟踪耗时中位数：Stereo 38.7 ms，惯性 35.0 ms；示例报告均值分别 42.2 ms、38.0 ms。各任务有并发且程序含回放节流，上述不是独占机器基准；P95、逐帧实时跟踪状态、完整丢失/重定位次数、进程 CPU 和内存峰值没有经过采集，暂不报告。

## 构建与兼容性

- Visual Studio Build Tools 17.14 / MSVC 19.44，CMake 3.29.6，Ninja 1.13.0，OpenCV C++ 4.11，Eigen 3.4，Pangolin 0.9.6；辅助环境 Conda `egohand-vio` / Python 3.11，原 `.venv` 未修改。
- 构建入口 `scripts/build_orbslam3_vio_msvc.cmd`；后续增量构建 `cmake --build third_party/ORB_SLAM3/build_msvc_initial --target stereo_euroc stereo_inertial_euroc stereo_tum_vi stereo_inertial_tum_vi -j 2`，需要先运行 `vcvars64.bat`。
- 项目采用 ORB-SLAM3、g2o、DBoW2 静态链接；核心建图、IMU、回环未删除。Windows 补丁在 `logs/vio_orbslam3_windows.patch`，新增头单独备份为 `logs/vio_windows_compat.h`。新增无 Viewer 运行修正 Qt 平台插件弹窗；GUI 本身仍使用原有 PySide6。
- TUM 鱼眼建图和关键帧剔除中，两处上游代码对仅按左图分配的数组使用了右图索引，已限制为适用的左图数据。上游 TUM Stereo 配置的 `Camera.overlapping*` 字段名在项目副本中改为程序实际读取的 `Camera1.overlapping*`。

## 交付与缺口

GUI 的“位姿”选项卡可手动加载真实 ORB-SLAM3 双目惯性轨迹及对应标定，展示离线位置、旋转矩阵和 XY 轨迹；用真实 EuRoC 3678 条位姿加载成功。当前标注跟踪状态未知、手部同步不可用；未将不同数据源强行拼接。`src/egohand/vio/hand_world.py` 和 `EgoHandEngine.bind_offline_pose_provider` 已实现显式绑定、坐标转换和独立 JSON 导出，并通过合成手部集成测试；GUI 尚未提供绑定入口，真实同一会话的手部 + VIO 数据也尚未验收，因此不能宣称世界手部链路验收完成。离线姿态文件也不能证明逐帧实时状态、重定位或回环版本。

Windows Qt 全量回归 `logs/vio_final_regression_windows.log`：243 通过、1 跳过。仍需验证真实同一采集会话绑定、GUI 同步状态、连续性和失联、P95/资源监控，之后才能完成任务书全部验收。
