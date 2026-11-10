# HOT3D-Clips 实测接入

本轮只适配 Clips tar，不装完整 VRS/VIO/SLAM 工具链，不实现 M06–M12，不修改 GUI。使用用户实际提供的 `samples/date/hot3d`；本机没有所述 `datasets/hot3d` 目录。下载与原数据不进入源码提交。

## 文件与官方身份

官方仓库：<https://huggingface.co/datasets/bop-benchmark/hot3d/tree/main>，固定数据提交 `30fe9674782f32e1e5edba98476b6ff4300132c5`。

|片段|官方 split|字节|hands.json|
|---|---|---:|---|
|001849|train_aria|104069120|150 个逐帧文件|
|001851|train_aria|103127040|150 个逐帧文件|
|001894|train_aria|103956480|150 个逐帧文件；本轮补下载|
|003365|test_aria|103444480|未公开，包内没有|

四包本地 SHA256 均等于官方 LFS oid；详见 `samples/date/hot3d/official_file_metadata.json` 与 `prepared/inspection.json`。003365 尽管本地位于 train_aria，不能当训练 GT 使用；按哈希识别出的官方 split 优先于文件夹名。

检查完整 tar 成员负载、重复/不安全路径、JSON 可解析性、150 帧配对、两路灰度 JPEG 全部解码与原生尺寸、时间单调性。训练包1201成员，测试包901成员。只另存标注副本到 `prepared/clip-XXXXXX/annotations/`，不重写 tar 或既有解包目录。标注名为 `000000.hands.json` 等，不是单个总表。

## 相机与同步

来源：[官方格式](https://github.com/facebookresearch/hot3d/blob/main/hot3d/clips/README.md)、[官方相机实现](https://github.com/facebookresearch/hand_tracking_toolkit/blob/9064a2fe7fda51bb1f1451a0d3d0da7f3ef5e64e/hand_tracking_toolkit/camera.py)。

只配 `1201-1` 与 `1201-2`。`214-1` 为 RGB，绝不参与双目。Aria 原生两路均为640×480单通道。以三通道复制适配 MediaPipe，记录 `grayscale_replicated_to_rgb`，没有恢复颜色。

采用原生 FISHEYE624（6径向、2切向、4薄棱镜），直接调用官方 forward projection，不替换成 OpenCV 四参数 fisheye。15参数形式的 Aria 单焦距解析由官方 from_json 处理。

`T_right_from_left = inverse(T_device_from_right) @ T_device_from_left`；同时用 `inverse(T_world_from_right) @ T_world_from_left` 独立核对。第一样本最大矩阵差约3.3e-16、基线约0.13889m。标定可能在片段中变化：001849 第62帧起出现更新，适配器按每帧完整标定重建映射；不能冻结第0帧外参。

由实际基线方向和两路平均光轴建立固定规则的公共虚拟平行双目，640×480、f=160px；原始 FISHEYE624 投影生成反向采样图，限制原生有效角度和像域，黑区保留，所有参数/有效视域比例写入 provenance。此重采样会损失原生视域及手部采样密度，因此另测原生图检测路径。

原生图推理遵循[官方 vis_clips.py](https://github.com/facebookresearch/hot3d/blob/main/hot3d/clips/vis_clips.py)的 `np.rot90(k=3)` 正向处理。检测后恢复原生像素：`u_native=v_upright, v_native=H-1-u_upright`。对官方畸变函数做数值 Newton 逆解，必须通过原生 forward residual <1e-5px、有效角度与正深度核验，才把射线映入公共虚拟视图。没有用 GT 框裁剪、GT 点补检或 GT 身份修改关联。

三角化仍调用既有 Engine/Hungarian/JointTriangulator。内部坐标为虚拟左光学坐标；正式导出通过 `X_native_left = X_virtual_left @ R_virtual_from_native_left` 回原生1201-1左光学坐标，协方差同步旋转，重新计算 native Z 标准差；米制 XYZ 不来自 MediaPipe world landmarks。

逐路时间来自 `info.image_timestamps_ns`，保留 ref_timestamp_ns（GT参考时刻）与各相机偏移。配对来源 DATASET_DOCUMENTED；例如001849首帧灰度左右相差12ns。这个记录值不是本机测得的硬件同步精度，GT与图像参考时刻也非完全相同。本轮 hardware_sync_precision_verified=false。

## 官方三维真值

[官方 UmeTrack FK](https://github.com/facebookresearch/hand_tracking_toolkit/blob/9064a2fe7fda51bb1f1451a0d3d0da7f3ef5e64e/hand_tracking_toolkit/hand_models/umetrack_hand_model.py) 固定提交 `9064a2fe7fda51bb1f1451a0d3d0da7f3ef5e64e`。

`convert_hot3d_umetrack.py` 在已有 `<path-to-python-with-torch>/python.exe` 中导入官方实现。读取手形 UserProfile、逐帧22维 joint_angles、T_world_from_wrist。由官方 from_json 处理静态毫米→米，由官方 FK 处理右手镜像、关节蒙皮；requires_mesh=False。保留模型代码摘要、源tar摘要、解释器/Torch版本和 world 米制输出。主`.venv`没有安装Torch、MANO或smplx；typing_extensions4.15.0仅位于数据目录的独立runtime下。

不重新实现 UmeTrack 蒙皮。MANO 参数保留原文，但没有额外 MANO 资产授权/模型文件，**MANO 评测未实现**。采用官方允许的 UmeTrack 路径建立 GT，不能说同时验证了 MANO。

每帧用逆 T_world_from_camera 转到真实左相机，再旋转到虚拟左相机供几何比较。没有根对齐、尺度拟合或手形拟合以美化误差。

## 显式关节映射

官方 UmeTrack FK 输出20个 canonical landmarks；MANO_TO_CANONICAL_LANDMARK_MAPPING给出其共享语义顺序。对应MediaPipe索引：

|Canonical索引|模型语义|MediaPipe索引|
|---|---|---|
|0–4|拇/食/中/环/小指尖|4,8,12,16,20|
|5|wrist|0|
|6–7|thumb MCP/IP|2,3|
|8–10|index MCP/PIP/DIP|5,6,7|
|11–13|middle MCP/PIP/DIP|9,10,11|
|14–16|ring MCP/PIP/DIP|13,14,15|
|17–19|pinky MCP/PIP/DIP|17,18,19|

20点模型对应关系，排除MediaPipe THUMB_CMC=1。UserProfile原有第21个额外点不属于官方 canonical20 输出，不能补为 CMC。映射仍是两种模型地标之间的比较，不是证明各模型定位在完全相同的人体组织中心；剩余语义/形状差异是误差组成。

## 复现

```powershell
.venv\Scripts\python.exe scripts\prepare_hot3d.py --dataset-root samples/date/hot3d --torch-python <path-to-python-with-torch>/python.exe --toolkit-root samples/date/hot3d/official-hand-toolkit
.venv\Scripts\python.exe scripts\run_stereo_benchmark.py --dataset hot3d --dataset-root samples/date/hot3d --clips clip-001849,clip-001851,clip-001894 --max-frames 450 --mode oracle-2d --output benchmark-results/my_oracle
.venv\Scripts\python.exe scripts\run_stereo_benchmark.py --dataset hot3d --dataset-root samples/date/hot3d --clips clip-001849,clip-001851,clip-001894 --max-frames 450 --mode mediapipe-stereo --detector-input native --native-detection --output benchmark-results/my_real_images
```

工具仓库不在主依赖中，首次准备可用git clone官方仓库并checkout上述提交；独立runtime只需typing_extensions，官方相机还使用主环境已有NumPy/SciPy。数据许可证入口为 <https://www.projectaria.com/datasets/hot3d/license/> 和 HF 根目录 `hot3d_dataset_license_agreement.pdf`；Clips沿用HOT3D数据许可，不能把工具代码的Apache2.0视为所有数据/第三方模型的许可。

结果见 BENCHMARK_REPORT.md：真实 MediaPipe 训练片段未产出有效双目三维，绝对误差尚不能验收。Oracle仅验证模型投影后的几何运算。003365无GT，保持N/A。
