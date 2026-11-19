# ACE 人体与手部联合重建（Windows 离线实验）

此入口直接读取 VS030 原始录制，提取相机轨迹和双目观测，以中立 SMPL 和左右目 ACE 手姿初始化，只执行一次 SMPL / MANO 联合优化。计算完成会生成独立的离线回放缓存；无需先运行旧算法。录制输入和已有输出不会被覆盖。

先按仓库根目录说明准备项目 Python 环境、VS030 SDK、ORB-SLAM3 运行时、检测器和模型。此实验另外使用自己准备的 ACE-Ego-Hand 安装与其推理权重，包括 ACE checkpoint、Wan 主干及 VAE。适配器使用官方 tiled 预测接口；它启动独立编码和推理子进程。第三方代码、权重、厂商 SDK 和标定文件不在此目录分发，各自适用原许可。当前版本只验证了相容的 Windows / CUDA 环境，不保证所有 ACE 上游版本均兼容。

在 PowerShell 中按自己的安装位置配置，再从仓库根目录运行：

~~~powershell
$env:EGOHAND_ACE_ROOT = 'C:\path\to\ACE-Ego-Hand'
$env:EGOHAND_JOINT_ACE_PYTHON = 'C:\path\to\ace-env\python.exe'
$env:EGOHAND_JOINT_BODY_PYTHON = 'C:\path\to\body-env\python.exe'
.\.venv\Scripts\python.exe scripts/ace_joint/joint_client_gui.py
~~~

- ACE Python 环境需要可用的 CUDA PyTorch、OpenCV、SciPy、SMPL-X、Accelerate 及匹配的上游 ACE 依赖。ACE 根目录下需有 `options/ace_ego_hand_k.yml`、`checkpoints/ace_ego_hand_k.pt`，以及上游配置所需的其他权重。
- 人体环境需要 Python 3.11+、PyTorch、SMPL-X 及项目依赖，并从仓库的 `models/smpl/SMPL_NEUTRAL.pkl` 读取自备模型。MANO 模型默认在 `models/mano/`，可用 `EGOHAND_JOINT_MANO_DIR` 指定。未单独指定解释器时，子进程使用当前 Python。
- 工作目录默认是 `data/.joint_work`，可用 `EGOHAND_JOINT_WORK_DIR` 指定有足够空间的位置。工作目录、录制、快载缓存和结果都只保存在本机。若本地没有交付版定位配置和二进制，定位使用录制的配置快照与本地已配置的 VIO 运行时。

在界面点击「打开录制」选择原始目录，再点「计算」。新算法默认选中；额头偏移默认 7.5 mm，也可调整初始离地高度。生成的结果可用「查看计算结果」打开。倍速和「最快」档都逐帧播放。

也可用同一套环境变量走命令行，输出路径必须是尚未创建的独立目录：

~~~powershell
.\.venv\Scripts\python.exe scripts/ace_joint/joint_offline_pipeline.py --recording 'C:\path\to\recording' --output 'data\joint_result'
.\.venv\Scripts\python.exe scripts/ace_joint/joint_client_gui.py --open-results 'data\joint_result'
~~~

完成标志是输出目录的 `pipeline.json`，其中记录重建次数和快载状态。存在 `display_cache.sqlite3`、`depth_results.sqlite3` 和 `joint_pose_index.npz` 时，本入口会直接回放预计算数据。网格只在通过定位和几何检查的帧发布。重投影误差是对输入像素的一致性指标，不能替代三维真值。
