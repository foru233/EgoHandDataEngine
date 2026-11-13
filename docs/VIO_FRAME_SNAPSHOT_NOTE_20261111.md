# VIO 帧结果一致性扩展

## 修复依据与范围

旧链路在 `TrackStereo()` 返回后分别读取相机位姿、tracking state、地图 epoch、局部建图线程初始化时间、BA2 标志，然后由 Python 再读取惯性体状态。各读取之间后台建图可能提交优化，因此各字段不一定来自同一地图修订。原子快照修复消除这条可确认的接口风险；它不等于证明此前 280 米跳变全由读取竞态引起。

新 `TrackingFrameSnapshot` 在 `Tracking::Track()` 持有 `Map::mMutexMapUpdate` 时复制结果。作用域清理器在地图锁释放前执行，覆盖该锁内的正常结束与提前返回。进入地图锁之前返回、地图切换或时间不匹配时，结果不宣称一致。快照只做内存复制，不向正常热路径无条件写诊断文件。

System 在输入帧开始时清除上一帧快照并传入当前 reset revision。发布时复制：frame ID、算法时间、map ID、大范围修改版本、普通 `map_change_index`、reset revision、tracking state、IMU 初始化、BA1/BA2、相机/惯性体位姿、速度、偏置以及特征/有效深度/内点计数。初始化直接读取锁内 map flag，不再通过异步 LocalMapping 时间差是否为正来猜测。

地图锁内不调用 `System::GetMapEpoch()`，不反向取得 reset mutex。原有重定位候选回调也改为使用输入帧 generation 与当前锁内地图版本编码 epoch，减少旧 map-lock → reset-lock 的锁顺序风险。该改动没有重新引入 `ResetFrameIMU` 清空预积分。

## ABI 与兼容

- `EgohandVioResult` 字段及大小不变。
- 新增独立的 `EgohandVioFrameSnapshot` v1，以及 `egohand_vio_snapshot_size`、`egohand_vio_get_frame_snapshot`。读取要求调用方大小与 DLL 大小一致，并核对 schema/struct_size。
- 兼容 `egohand_vio_get_body_state` 在新 DLL 中也只消费同一帧缓存。
- Python `VioFrameResult.frame_snapshot` 是可选扩展。支持新接口时，不再从异步 body getter 拼接响应；核对快照 timestamp 和 world frame ID 是否等于当前帧结果。旧 DLL 保留原逻辑，并明确输出 `snapshot_consistent:null`，不虚构一致性。
- 元数据的普通 `map_change_index` 与大范围 `map_revision` 分开。旧 `map_epoch` 不变不代表没有 Local BA 修正，因此不能把同 epoch 所有跳变直接归为视觉前端错误。
- 保留旧 `inertial_scale_finalized` 名称以保持 ABI，注释更正为 BA2 完成标志。双目在 BA2 以前已有基线提供的米制尺度，BA2 完成也不是测量准确的证据。
- `world_frame_id` 仍沿用旧 map ID/reset revision/big-change 编码；对普通 Local BA 变化应同时检查 `map_change_index`。

## 诊断对照开关

- `EGOHAND_VIO_STEREO_ONLY=1` 只在显式诊断时使普通 create 使用 ORB `STEREO`，并打印 `Egohand sensor: STEREO (diagnostic)`。默认仍为 `IMU_STEREO`。Atlas create 路径不受此开关影响。
- `EGOHAND_VIO_DISABLE_RECOVERY=1` 禁用 Python 实验性 RecoveryCoordinator 与回调，默认保留。此开关不等于撤销所有本地 ORB 恢复补丁，报告必须区分。
- `EGOHAND_VIO_LIBRARY` 支持显式选择对照 DLL。没有部署清单时由公共 `library_manifest.select_library` 固定 live=v9、atlas=旧 Atlas 库，不再探测一串历史 DLL 名称。
- 新库以独立 `egohand_vio_stability_snapshot.dll` 构建，未更改默认部署，不覆盖已有 DLL。

## 自动检查

`tests/test_vio_frame_snapshot.py` 验证无初始化状态可返回元数据、同帧 body、ABI 拒绝、不一致快照不供给 body、旧能力未知，以及 socket 响应不再二次读取 body。

`scripts/audit_vio_stability.py` 输出 JSON/CSV/图表，分别统计连续记录、跨无效记录的相邻可用矩阵、短时间间隔、同地图 ID、同精确 epoch 与跨 epoch。长缺口不算相邻瞬移；图像时间覆盖率不当作处理墙钟吞吐。原生日志无帧时钟或退出码时保留缺失，不编造崩溃计数。

实际测试和构建结果以 `logs/vio_stability_snapshot_regression.log`、`logs/vio_stability_snapshot_build_final.log` 及独立 DLL 的 `.build.json` 为准。Python 检查不替代同输入原生回放、Atlas 与设备验收；也没有加入显示平滑、放宽门限或修改原始矩阵。

本次实际执行：

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_vio_frame_snapshot.py tests/test_vio_stability_audit.py tests/test_vio_diagnostic_flags.py tests/test_vio_body_state.py tests/test_vio_live_runtime.py tests/test_vio_recovery_callback_flow.py -q
.\.venv\Scripts\python.exe -m pytest tests/ -q --disable-warnings
cmd /c scripts\build_vio_stability.cmd egohand_vio_stability_snapshot
```

针对性检查 32 通过；全量 Python 测试 442 通过、1 跳过、2 warnings，运行约 92.46 秒。最终构建成功，DLL SHA256 `b7e99ba9aad1f4c2add5d1551a6724c91bcef0c68fa41e25c8ef6f0045bdad16`；未部署到默认角色。这些检查尚未证明新原生快照在真实回放和 Atlas 导入中的行为，需由后续同输入实验验证。

## 新录制的原生快照回放检查

`results/vio/stability_baseline/snapshot_matrix/snapshot_default` 使用新录制内的标定/ORB 快照及本独立 DLL，默认 gauge 开关关闭。本次运行 1727 帧完成，worker exit 0，未报告错误。用 `scripts/audit_vio_frame_snapshots.py` 实际核对：

- 1727 条均 `snapshot_available=true`、`snapshot_consistent=true`，两类拒绝计数均为零；没有通过拒绝这些快照降低跳变统计。
- 原生有效 1646，门控后有效 1022；没有发现 frame/timestamp/world-frame/body 身份矛盾。
- 1572 个 body/相机外参闭合样本最大矩阵元素差约 1.905e-6；这只是同帧代数闭合，不是独立精度验证。
- IMU 初始化首次在第 89 行（2.933 s），BA1 第 600 行（19.967 s），BA2 第 962 行（32.033 s）。
- 同精确 epoch、间隔不超过 0.2 秒的相邻可用原始位姿中，超过 0.5 米的样本为零；步长最大 0.428 m、P95 0.043 m。相同 epoch 仍可含普通 Local BA 修订，这不证明前端完全稳定。
- **跨 epoch 仍有巨幅跳变**：第 600→601 行约 7.337 m，第 961→963 行约 31.378 m。后者跨一条无有效矩阵的记录，实际间隔 0.066667 s；不能误称连续两条矩阵记录。两次地图 ID 和 reset generation 保持不变，big revision 变化。

因此本轮验证了快照内部一致性与可观测性，但没有消除全局优化发布时的地图跳变。此 DLL 同时包含重力边界保护，单次回放及 ORB 多线程运行差异不能证明全部变化由快照修复导致；仍需重复运行和独立消融。原始快照检查见 `snapshot_audit.json`，完整步长/时序见同目录 `audit/poses/report.json`。

同 DLL 的 `snapshot_matrix/stereo_only` 也完成 1727 帧，所有快照一致/可用，未发现身份矛盾；原生有效 1727，门控有效 1682，无地图 epoch 改变。同 epoch 步长最大约 0.04138 m、P95 0.02385 m。原生日志明确打印 `STEREO (diagnostic)`，IMU 初始化与 body 均未产生，属于预期。两次实验图像时间戳序列完全一致，输入配置及 DLL 哈希一致。

但该版 runner 对 stereo-only 实际强制关闭了实验性恢复协调器，最初 `inputs.json` 却仅按显式 `--disable-recovery` 记录，误写为 enabled。原始文件保持原样，本处记录更正；不能把这对实验称为只改变 sensor 的严格单变量对照。后续应使用明确关闭恢复的惯性快照回放与 stereo-only 比较。即使两模式差异明显，也不能只凭有效帧数选定标定、时序或某一数值模块为唯一根因。

同输入 `snapshot_repeat` 再次完成 1727 帧且 exit 0，全部快照仍一致/可用，身份矛盾为零。但原生有效变为 1531、门控有效 697，同 epoch 超过 0.5 m 的步长出现 3 次，最大 1.4296 m；这说明首轮零次不具备重复保证。三次都伴随普通 Local BA 修订号增加：

| 日志行（从 1 开始） | 位移 | 普通 map_change_index | 前后内点 |
| --- | --- | --- | --- |
| 1027→1028 | 0.941 m | 394→395 | 315→377 |
| 1622→1623 | 1.430 m | 633→634 | 359→361 |
| 1624→1625 | 0.519 m | 635→636 | 406→403 |

这些帧约有 1200 个特征，不能简单归为没有视觉特征；也不能把修订号同时变化当作“位移完全由 Local BA 引起”的证明。需要同步的同 ID KF 与地图点前后快照才能分解。此次跨 epoch 还出现 2.823 m 位移，以及一次 reset 后的 2.999 m 位移；后者 generation 改变，不能把跨 reset 世界坐标差当相机真实运动。重复运行数据进一步表明原子接口修复本身尚不足以消除 SLAM 几何与重复性问题。
