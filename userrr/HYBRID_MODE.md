# 单套 VIO 的视觉先行模式

用户最终要求：只运行一套 VIO，惯性不可用时允许视觉定位，惯性可用后优先 VIO。此前独立纯视觉进程方案已撤销，源码不再构造 StereoFallback。

## 实现

- 一个原生 `IMU_STEREO` 实例、一份地图和一次双目特征跟踪；不双开 SLAM。
- ORB 原有 `IMU.fastInit` 只跳过首次双目建图前的加速度变化检查，不设置 IMU 已初始化。本次补齐 `Tracking` 版本化配置加载路径对该选项的读取；旧式参数加载原本已支持它。
- 仅混合模式生成 `userrr/runtime_configs/hybrid_*.yaml` 配置副本，并启用 fastInit。录制快照及普通 VIO 配置不变。
- 独立桥接库 `egohand_vio_visual_bootstrap.dll` 包含同一套现有修复，仅混合模式选择。`active_bridges.json` 未切换，普通 VIO 和纯视觉仍用原库。
- 两套轻量门控消费同一原始位姿，视觉引导阶段保留稳定时长、丢失、突变、世界身份和恢复隔离检查；**VIO 正式接管显示还要求真实 IMU 初始化、原生 BA2 完成和 `inertial_scale_finalized=True`**。只有这三个原生条件与质量门控均满足时才标成 `vio`；否则使用同一实例中合格的视觉引导位姿或暂停显示。此阶段不是第二套独立 STEREO 追踪，原生惯性优化仍可能改变原地图坐标。两者均无效时不显示位姿。
- `pose_source` 为 `visual_fallback` 或 `vio`。来源切换不触发原生地图重置：同一 `map_epoch` 和 `world_frame_revision` 内保留已有位姿，只在 `display_segment` 变化处断开绘制线段；跟踪临时无效也只打断线段。地图/世界修订变化及原生 `vio_reset` 才清除不兼容的旧世界轨迹。这里的视觉阶段不等同于另一个独立 STEREO 算法实例。

## 运行

重启客户 GUI，停止状态选择“混合 · VIO优先 / 视觉回退”，再连接或导入原生录制。默认模式仍是正常“双目＋IMU”，未强制更改用户默认选择。

构建命令：`scripts\build_vio_world_frame.cmd egohand_vio_visual_bootstrap`。构建日志 `single-vio-build.log`，库旁 build.json 记录SHA256和源树信息。未重建或替换原默认DLL。

## 历史录制的 IMU 时序保护（2026-11-12）

客户 GUI 在混合模式导入旧 VS030 SDK 录制时，如果快照仍为固定、未经核验的 packet lattice 相位，会在**新回放输出目录**中运行双目视觉旋转/陀螺仪独立核验，生成 `session_phase_preparation.json` 和 `verified_config/`，只用核验后的相位送入 IMU_STEREO。原录制、标定和其 `vio_timing.yaml` 均不修改。核验失败则明确报错，不拿旧固定相位冒充已核验。已有有效会话模型保持原样；显式 `ReplayVio(timing=...)` 的可复现实验仍尊重参数覆盖。不同录制不能共用相位。

示例 `data/imu/session_20261111_105436_609132400`：旧快照 +97 ms，独立核验得到 +34 ms（训练/留出最优分别 34/35 ms）。同原生 DLL、同 900 帧对照，+97 ms 跟踪 OK 871、门控通过 474、突变 14、BA2 完成帧 0；+34 ms 分别为 898、684、0、180。证据在 `results/vio/hybrid_105436_phase_audit_20261112.json` 和 `results/vio/hybrid_105436_900_ab_20261112/`。这是录制内诊断，不是设备曝光真值或全设备普适偏移。

## 限制

这是对首次建图门控的明确调整，不是宣称惯性初始化可忽略运动条件。后续惯性优化及数值安全检查保持有效。IMU数据缺失、逐会话时序准备未通过、视觉纹理不足或跟踪丢失，仍可能无有效位姿；本次没有伪造传感器数据或无效位姿。

没有第二套 SLAM 的重复计算开销，但原先未建图时几乎不执行的后续跟踪现在会真正运行，不能据此承诺整体FPS不变。更改前后的总时长若包含词典加载，也不能直接当稳态FPS比较。

`hand_hybrid_independent_160` 等早期目录属于已撤销的双实例试验，不是最终单实例验收；最终证据以 `hand_single_vio_fixed_250` 为准。

## 本次验证结果

- 完整 Python/GUI 回归：522通过、1跳过（`single-vio-final-tests.log`）。
- 第一段 `data/hand/session_20261110_111807_489019700` 前250帧：248帧跟踪OK，202帧门控后可显示，IMU初始化0帧；视觉阶段成功输出，没有伪称惯性就绪。
- 第二段 `data/hand/session_20261110_111833_589265500` 前250帧：248帧跟踪OK，203帧门控后可显示，IMU初始化0帧。
- 两次均完成、无重置、地图epoch保持0。原生日志均显示 `IMU_STEREO`、`Visual bootstrap enabled` 及实际创建地图，不含独立纯视觉实例。
- 原默认 `egohand_vio_cheirality_v3.dll` SHA256仍为 `0a5bbadd98c2eff77edd78ce364119de5ada6471e674cf406908571d1536cf95`。
- VIO优先、初始化等待、视觉回退、失效不补位姿和源切换分段有自动化测试覆盖；这两段小运动录制未完成IMU初始化，因此不能把它们作为真实惯性接管成功或精度提高的证据。
