# VIO 配置和地图接口重构记录

## 2026-11-12 更新

- 统一配置快照已经用于实时GUI、扫描、历史回放及Atlas；历史完整快照保留原行为。自动模式录制额外保存实际会话相位模型；缺少模型时在新工作目录独立核验，不改原始录制，不默默套用初始相位。
- 原生一致性快照及输入帧身份保护已落地，`world_frame_revision` 与epoch共同使用，恢复预测锚点和扫描几何不能跨坐标版本/工作进程重启复用。
- 扫描准入与原始跟踪、可信度判断分开：本次时序核验、IMU初始化、BA2提交、稳定跟踪和有效深度全部满足才启用开始，避免初始化过程中提前锁定粗点云世界版本。
- live/Atlas统一由 `active_bridges.json` 选择并验证库、哈希、能力和角色开关，旧DLL保留回退；普通GUI生命周期内复用词典工作进程。测试注入DLL禁止进入普通采集。
- 新增正深度观测模块 `VioCheiralityGuard.h` 与显式开关，真实几何检查/提交清理、非有限状态闭锁和诊断输出分离；只在提交后持地图更新锁执行，保留点同步刷新关联缓存。新增计数器为可选符号，旧库缺失返回None，不扩展旧结构布局或Atlas序列化字段。
- v3的10对受控回放、Atlas/3200帧/词典复用检查均完成，已切为live/Atlas默认库并启用正深度开关。上一版fail_closed SHA91和清单备份完整保留；部署后配置/GUI地图等25项回归通过。每版候选保留失败或替代原因，此处不是实机精度验收。

最新状态见 `VIO_故障闭锁与时序复核_20261112.md` 和 `VIO_正深度观测修复_20261112.md`。下方保留首轮重构历史，原“快照/部署未完成”等描述只对应当时状态。

日期：2026-11-11。本文件记录已落地的配置链路与 Atlas 身份整理，不作为 SLAM 数值稳定性或实机精度验收结论。当前代码仍存在未完成的原生快照、优化和部署工作，结果应结合根因与验收报告阅读。

## 改动前的问题

- 实时 GUI 从当前标定和 `live_timing.yaml` 取配置；录制虽然有快照，但通用回放仍用模块级标定、当前质量参数和默认 ORB 设置。
- 通用回放忽略 `image_scale`，可能将完整 1600×1200 校正图送给采用不同内参或分辨率的配置。
- 扫描器和 Atlas 回放硬编码标定结果 1，同时读取共享 timing 指向的标定 2 ORB 配置。
- Atlas 身份判定使用 `map_id << 32`，而当前原生 epoch 编码为 `map_id << 40 | reset_id << 20 | big_change_revision`。非零地图 ID 或非零初始修订容易误判。
- 位姿矩阵、质量判断与显示已有分层，不能通过显示连续性修正来取代原始 SLAM 修复。本轮继续保留关闭显示重锚的默认行为。

## 目前配置链路

```text
设备 SDK 原始记录 / 标定源文件（只读）
        │
        ├─ 实时 resolve_live_config()
        │       └─ GUI VIO / 扫描准备 / 新录制配置快照
        │
        └─ 历史 resolve_recording_config()
                ├─ 通用 ReplayVio
                └─ Atlas 回放 → 粗点云 → 最终位姿融合

VioConfigSnapshot
  四个文件路径 + 有效 timing + quality + schema/hash/fingerprint
        │
        ├─ validate_geometry() → 共用几何适配
        └─ 显式 ORB settings → 原生桥 / 独立 worker

原始结果 → 质量门控 → Atlas 地图身份检查 → 展示 / 差异 / 更新准入
```

`src/egohand/vio/config_snapshot.py` 不重写采集、IMU 格点或优化算法，只统一配置解析与可追溯性。几何依然由 `vs030_geometry.py` 完成，IMU 区间依然复用 `imu_lattice.py`。文件哈希在会话准备和保存阶段计算，不在逐帧热路径执行。

## 文件变化

| 文件 | 变化及理由 |
| --- | --- |
| `src/egohand/vio/config_snapshot.py` | 新增完整配置对象、实时与历史入口、几何一致性检查、版本与哈希描述、快照写入。 |
| `src/egohand/vio/replay_runtime.py` | 回放采用录制四文件快照；显式传递 ORB 设置到子进程；校正后按快照缩放；配置失败仍写出失败报告。 |
| `scripts/replay_recording_vio.py` | 增加 `--config-snapshot`，允许用户为缺快照的旧录制明确选择完整覆盖配置。 |
| `src/egohand/vio/live_runtime.py` | 主线程工作接入共用实时配置、几何校验与每会话配置描述；本文件的其他 VIO 变化应分别审查。 |
| `src/egohand/storage/recorder.py` | 新录制保存完整快照及哈希描述。原始录制不批量改写。 |
| `map_creat/scanner.py` | 扫描准备用实时共用配置，粗点云缩放由配置决定；离线融合从录制读取配置。 |
| `map_creat/atlas_replay.py` | 取消标定 1 混用；采用录制快照及显式完整覆盖；读取地图包时先验证包校验值和几何兼容。 |
| `map_creat/scan_pipeline.py` | 建图、粗点云、最终融合采用同一快照，处理信息记录配置身份。 |
| `src/egohand/vio/map_identity.py` | 新增 epoch 解码及只读地图身份固定；允许非零地图 ID / 初始修订，拒绝后续未知世界坐标修订。 |
| `src/egohand/vio/map_localization.py` | 用地图 ID 和固定完整 epoch 判定所选旧地图，保留原始估计供门控审计。 |
| `scripts/audit_vio_recording_inputs.py` | 独立只读输入取证脚本，检查全部记录头和载荷、两路配对、时钟、IMU 格点、配置一致性及原始哈希。 |
| `tests/test_vio_config_snapshot.py` | 缺失快照、显式覆盖、哈希篡改、几何缩放不匹配和旧四文件快照兼容测试。 |
| `tests/test_vio_map_identity.py` | 非零 ID / 修订、暂时无效、跨版本或 reset 拒绝测试。 |
| `tests/test_vio_replay.py` | 假桥测试显式隔离假几何配置；合成真实桥用独立完整配置快照。 |

## 配置和地图兼容性

- 历史录制默认要求四个文件：`kalibr_camchain.yaml`、`vio_settings.yaml`、`vio_timing.yaml`、`vio_quality.yaml`。缺少时失败并说明需要完整显式覆盖，不退回当前配置。
- 旧四文件录制没有 `vio_config_snapshot.json` 仍可读取，并执行几何一致性检查；其来源哈希不是录制时签名，不能由事后哈希证明原始配置真实性。
- 新描述文件有 schema、四文件实际哈希及有效配置 fingerprint，并附来源快照。描述文件存在但哈希不符时拒绝默认回放。显式覆盖保留来源信息。
- `vio_timing.yaml` 中的旧 `orb_config` 路径仅作为来源信息保留。历史实际 ORB 设置固定采用同目录 `vio_settings.yaml`，不会因仓库路径变化改用当前文件。
- lattice 下仍只应用 packet phase；保留的 timestamp lag 不与 phase 相加。
- Atlas `map.json` 既有 schema 和文件列表未改变。新增处理元数据不会改写原 Atlas。几何不兼容的地图/录制明确拒绝；版本不同的世界位姿不会拼接到旧点云。
- 只读定位锁定第一次有效定位的完整 epoch。后续版本变化暂停地图叠加，需要明确重新载入；这比自动假定“同 map ID 就同坐标”更保守。

## ABI、DLL 与上游补丁范围

本轮配置和身份模块不改变 C ABI、Atlas 序列化、原始矩阵、`ResetFrameIMU` 或 g2o 参数。没有为了测试通过放宽门控，没有平滑轨迹或重新定义原点。

`library_manifest.py` 和构建 manifest 脚本在当前工作树已存在，但是否已接入实际 DLL 选择、是否已部署新的角色清单、原生快照 ABI 是否已发布，必须按最终源码与构建记录核对，不能因辅助文件存在就声明完成。历史 DLL 保留供回退。规范自由度修复和原子结果接口不属于本文已经验收的配置改动。

## 实际验证及限制

- 首轮 `test_vio_config_snapshot.py + test_vio_replay.py`：17 passed，包含 20 帧合成 HEVC + 真实桥回放。该结果早于最后的哈希描述增强。
- 哈希增强后执行 config、map identity、map package、replay、live runtime 五组相关测试：33 passed、1 deselected。包含真实配置快照选择与 1600×1200 → 800×600 输入检查、缺快照时失败报告落盘；主动排除合成真实桥，避免与主线程原生 A/B 和编译争用资源。
- 新录制只读审计：5208 条 SDK 记录、1736 对双目、27776 条 IMU 行；两条原始二进制 SHA 与 manifest 一致。快照缩放内参误差为 0，IMU 外参矩阵差约 2.4e-8；这些仅证明内部一致性，不证明物理标定或曝光同步。
- 完整命令与更广泛回归结果由 `VIO_ACCEPTANCE_REPORT.md` 汇总。最终整套测试需覆盖这些最后变化，不能套用此前全量成绩。

## 尚未收敛的工作

1. 原生一致性结果快照及优化前后同 ID 元素诊断已在独立 snapshot DLL 实现并开始原生回放。规范 A/B 已取证，B 超时，仍默认关闭；跨优化的地图畸变尚未修复。细节见 `VIO_FRAME_SNAPSHOT_NOTE_20261111.md` 和根因报告。
2. 已增加显式 live/Atlas 角色选择、哈希/能力清单与构建 provenance，独立 snapshot DLL 有完整构建清单；默认角色尚未切换，历史库仍保留回退。
3. 严格快照模式下旧录制的 GUI 完整覆盖选择界面。目前默认明确报错，CLI 可显式提供完整目录。
4. Atlas 几何兼容、保存/加载和真实非零地图身份的原生端到端回归；当前新增身份测试为合成结果层测试。
5. 当前 IMU scale-misalignment、噪声定义、曝光时刻及硬件同步的外部证据；未擅自替换。
6. 至少 3000 帧压力测试、30 分钟实机、同一输入修复前后原生结果对比。低抖动不是准确性证明。
