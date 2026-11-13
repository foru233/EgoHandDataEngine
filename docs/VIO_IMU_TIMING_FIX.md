# VS030 IMU–相机时间对齐：问题、修复与验证（2026-11-09 晚）

## 1. 问题

- 实时 VIO 用固定 `camera_to_imu_lag_ms: -85` 对齐，35 s 数据上视觉旋转与陀螺相关 ≈0.991，但 0/−85 ms A/B 都不改善地图稳定性，ORB 仍反复初始化/重置。
- 现场 667 状态帧会话出现 38 个 map epoch；另有 255 次“运动不足”原地清图但 epoch 只记 4 次。

## 2. 根因（2026-11-10 修正）

### 2.1 关键修正：物理采样没有漂移，漂移的是时间戳

用 8 分钟真实采集（`results/vio/vs030_lag_capture_20261109_183614.npz`，228,583 条 IMU、3,084 条视觉旋转）做**格点速率扫描**，每个速率下看 10 s 窗的最佳时延：

| 格点速率 | 各 10 s 窗最佳时延 | 斜率 |
| --- | --- | --- |
| 479.0 Hz | −25 → +95 ms | +1.89 ms/s |
| 479.5 Hz | −35 → +25 ms | +1.05 ms/s |
| **480.0 Hz** | **−40 … −35 ms（全程恒定）** | **+0.02 ms/s** |
| 480.5 Hz | −50 → −95 ms | −0.89 ms/s |
| 480.9 Hz | −55 → −180 ms | −2.05 ms/s |
| 481.2 Hz | −60 → −215 ms | −2.59 ms/s |

**结论**：

- **IMU 物理采样与相机严格 16:1（480 Hz / 30 Hz）**，与厂商“第一帧对视觉、其后按采样计数铺时间”的做法一致；
- 驱动时间戳由 IMU 内部 RC 计数器按标称 480 Hz 合成，相对真实时间有 **~0.2–0.3% 误差**；**用时间戳对齐**才会看到“随时间增长的漂移”（−3 ms/s），这是**时间戳的假漂移**，不是物理采样漂移，也不是延迟在累积；
- 正确模型是 **固定延迟 + 480 Hz 帧格点**；之前用 `479.68 Hz`（657 ppm）重采样等于每秒少消费约 0.067% 的样本（0.32 条/s），90 s 累积约 60 ms 相位滑移；它能解释部分实机差异，但不能单独解释 26/118/200 epoch 的全部差距；
- 35 s 诊断得到的 −85 ms 只是“时间戳漂移在窗口内的平均值”，因此固定 −85 ms 不可能稳定。

### 2.2 关于 459 Hz

当前 UDP 通道 7 数据的中位 Δt 为 **2.0847 ms（479.68 Hz）**、均值 **2.0794 ms（480.91 Hz）**，全部落在 2.0621/2.0847/2.1074 ms 三档（22.66 µs 的 RC 量化步长），**没有 459 Hz**。459 Hz 出现在旧 IIO/Kalibr 链路（`data3-results-imucam.txt` 的 459 Hz、旧文档的 2.17536 ms），不适用于当前 UDP 数据。

### 2.3 厂商固件的自述

- IIO driver 时间戳由 IMU 内部 RC tick 合成：“**NOT the true sampling instant, drifts ~0.2% vs the frame clock. Used only for drop detection, never for alignment**”；
- MCAP 录制路径用 `exposure_start + i*T/N` 做帧格点重定时；UDP 实时路径没有做（日志：`udp imu: no frame anchor, header timestamp stays on driver clock`）。

## 3. 修复

### 3.1 新增 `src/egohand/vio/imu_lattice.py`

`LatticeImuBuffer`：按相机帧间隔做**计数锁定重采样**。

- 每个帧间隔消费 `round(Δt_cam × rate_cam)` 个样本（含小数进位，长期不累积误差）；
- 样本均匀铺在 `(prev, now)`：`t_i = prev + (i+0.5)·Δt/n`；
- 完全不用 driver 时间戳做对齐，只用于顺序、缺口与会话起点裁剪；
- 样本不足返回 `None`（上层显式处理），不猜测、不伪造。

默认 `rate_cam = imu_rate_hz / (1 + drift_ppm·1e-6)`；修正后取 **480.0 Hz、drift 0**（见 2.1 的扫描）。

### 3.2 接入 `src/egohand/vio/live_runtime.py`

- `live_timing.yaml` 新增 `imu_alignment: lattice | timestamp`，默认 lattice；
- 统一 `pull_imu()` 收包/归档/运动检测；两种模式共用错误处理；
- 格点首帧**不按时间戳裁剪**：清空缓冲、从首帧之后的样本开始计数对齐（对相机/IMU 标签域相差数秒免疫）；
- 审计字段新增 `imu_alignment`、`imu_lattice{pending,consumed,gap_events,rate_hz,drift_ppm}`；
- `LiveVio` 支持注入 `timing`，便于 A/B 与测试。

### 3.3 健壮性（本轮实测暴露并修复）

| 问题 | 处理 |
| --- | --- |
| 收包超时固定 0.6 s，帧间隔 700 ms 时需要 336 样本却只等到 334 → VIO 终止 | 超时按 `max(0.6, 帧间隔+0.5)` 动态放宽（上限 8 s） |
| 缓冲上限 2 万条，长帧间隔下溢出 | 上限 6 万条；溢出/样本不足时**显式重建对齐并继续**，不再杀死 VIO |
| 运动检测阈值过高 | 0.30 → 0.15 rad/s（8.6°/s），连续 8 个样本触发 |

### 3.4 配置

```yaml
imu_alignment: lattice
imu_rate_hz: 480.0
camera_to_imu_drift_ppm: 0.0     # 物理采样 480 Hz，无漂移
camera_to_imu_lag_ms: -85.0      # 仅 timestamp 回退模式
```

## 4. 验证

### 4.1 离线（真实 VS030 数据）

- 按时间戳对齐：逐窗时延从 −50 漂到 −240 ms（假漂移）；
- 按 **480.0 Hz 格点**重采样：逐窗时延恒为 −40…−35 ms（斜率 +0.02 ms/s，见 2.1 扫描）；
- 单元测试 `tests/test_imu_lattice.py` 7 项，覆盖计数、进位、缺口、非递增、缓冲不足。

### 4.2 EuRoC 端到端（真实图像 + 真实 IMU + 真实 ORB-SLAM3）

`scripts/compare_lattice_euroc.py`（160 帧）：

| 模式 | 跟踪 OK | NOT_INIT | IMU 初始化帧 |
| --- | --- | --- | --- |
| 原始时间戳 | 156 | 4 | 109 |
| 格点重采样 | 156 | 4 | 108 |

结论：格点路径在标准数据上**没有引入回归**（轨迹世界系不同导致绝对位置不可直接比较，未做 ATE）。

### 4.3 实机静态浸泡（20 分钟）

格点模式连续运行 2,958 个状态帧，**无样本不足/溢出错误**，pending 稳定；因相机始终静止（陀螺 max 0.008 rad/s），IMU 初始化 0 次——这符合 ORB 对运动激励的要求，但不能作为初始化验证。

### 4.4 实机运动验证（2026-11-09 晚 – 11-10 上午，共 6 次）

| # | 模式 | 时长 | 运动样本 | 首次初始化 | 初始化帧 | OK 帧 | epoch | 门控接受 | 错误 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | lattice | 90 s | 40,627 | 第 27 帧（≈3 s） | 488 | 822 | 26 | 407 | 0 |
| 2 | timestamp | 90 s | 16 | — | 0 | 1 | 1 | 0 | 样本不足 |
| 3 | lattice | 90 s | 42,778 | 第 313 帧 | 1 | 619 | 118 | 0 | 0 |
| 4 | timestamp | 90 s | 42,497 | 第 196 帧 | 72 | 496 | 85 | 49 | 0 |
| 5 | lattice | 120 s | 55,513 | 第 384 帧 | 14 | 743 | 200 | 0 | 0 |
| 6 | timestamp | 120 s | 32 | — | 0 | 1 | 1 | 0 | 样本不足 |

另有“同一段 60 s 数据重放”实验（4 种参数）：

| 参数 | OK 帧 | 初始化帧 | epoch |
| --- | --- | --- | --- |
| lattice 480.0 | 409 | 3 | 76 |
| lattice 479.68 | 406 | 3 | 78 |
| lattice 479.2 | 403 | 4 | 79 |
| timestamp −85 ms | 410 | 4 | 76 |

**重要说明**：以上 6 次实机运行使用的配置是 **`drift_ppm=657`（479.68 Hz）**，即 2.1 节判定的**错误速率**，因此不能用来评价修正后的方案；另外重放实验中 480.0 与 479.68 只差 0.07%，在弱激励数据上区分不出来。

**解读**：

1. 格点模式能工作：第 1 次实机第 27 帧（约 3 秒）完成 IMU 初始化、822 帧 OK、最长连续 324 帧。
2. 逐次差异极大（epoch 26/118/200、初始化帧 488/1/14）；657 ppm 的速率误差在 90 s 内只累积约 60 ms 相位滑移，只能解释一部分，其余更可能来自运动激励与跟踪质量。
3. 旧 timestamp 模式在实机运动中 6 次里 3 次直接报“样本不足”终止；按时间戳对齐在原理上无法跟随 RC 时间戳的漂移。
4. **修正速率到 480.0 Hz 后的实机 A/B 尚未重跑**——这是当前唯一缺的验证。

### 4.5 录制数据回放（2026-11-10，31 s 强运动，同一份数据）

录制：`recordings/session_20261110_103856_125237400`（929 对双目、14,859 条 IMU，陀螺峰值 5.8 rad/s）。

| 模式 | OK 帧 | IMU 初始化帧 | map epoch | 错误 |
| --- | --- | --- | --- | --- |
| **lattice 480 Hz** | **912 / 929** | **407** | **7** | 无 |
| timestamp −85 ms | 765 / 929 | 29 | 13 | 末尾样本不足 |

**关键发现**：这段录制里相机 `device_ts` 与 IMU 时间戳的标签域相差 **≈4.62 s**（两者时长都是 30.9 s、样本数 14,859 ≈ 929×16，说明内容 1:1 对齐，只是标签不同）。旧实现在首帧 `trim_before(首帧时间)` 会误删前 4.6 s 的 IMU，导致后段样本不足、初始化很晚。

修复：**格点首帧不再按时间戳裁剪，而是清空旧缓冲、从首帧之后到达的样本按计数对齐**；这样对任意标签偏移免疫，也不需要知道两路时钟的差。

修复后同一份数据：lattice 912/929 OK、407 帧初始化、7 个 epoch；timestamp 仍受 4.6 s 标签偏移影响（765/929、29 帧初始化、13 epoch，末尾报样本不足）。

### 4.6 用 UDP 包头定位时间差，复测相位（2026-11-10）

检查 `data/imu` 和 `data/hand` 的五份原始录制。6,006 个 sink 7 IMU 包中，5,998 个包头 `device_ts` 可与 sink 4 相机标签精确匹配。在匹配包内，首条 IMU 的 `ts_a − device_ts×1000` 随时间变化，斜率为 **−3.00 到 −3.04 ms/s**。五份录制首个匹配包的差为 −1.23 到 −1.84 s。旧报告的约 4.62 s 来自另一次录制的包内时间与相机时间比较。该原始会话不在目前的 `data/imu`，这里复核的是产生秒级标签差的机制。

先按包头时间和 480 Hz 格点为每条 IMU 编排时间。用校正右相机的光流和单应旋转估计两帧间角速度，并将陀螺转到相机坐标后对同一帧间隔积分。扫描相位时只比较两种时间轴都覆盖的区间。

| `data/imu` 会话 | 视觉间隔 | 无补偿相关系数 | 包格点 +97 ms 相关系数 |
| --- | ---: | ---: | ---: |
| `110623_140565800` | 611 | 0.8991 | 0.9960 |
| `110729_776644600` | 317 | 0.8550 | 0.9988 |
| `112020_689246900` | 312 | 0.8365 | 0.9991 |

三份录制的全段最优值均为 +97 ms，九个分窗的最优值为 +95 到 +99 ms。正号表示把编排好的 IMU 样本时间后移。旧的约 −40 ms 使用首帧计数起点，原数值抓取没有保存包头，无法把它解释为固定的硬件延迟。+97 ms 是当前 VS030 和录制条件下的相对时序标定，现有证据不能将它分解为曝光、读出、FIFO 和固件分包各自的延迟。

现在 GUI 实时 VIO 和录制回放默认采用 `lattice_reference: packet` 与 `lattice_phase_ms: 97.0`。每包独立锚定，短包保留真实样本数，驱动时间只作包内丢样检查。实时不再按到达顺序猜第一包的位置，回放不再用驱动时间裁首帧。选 `count` 可复现旧格点，`timestamp` 仍保留为原始驱动时钟对照。CLI 可用 `--lattice-phase-ms 0` 做零相位对照。

`112020_689246900` 的 ORB-SLAM3 对照使用相同的 300 帧和相同的每帧样本数（`--stride 2`）。从零补偿改到 +97 ms，跟踪 OK 从 282 到 292，已初始化帧从 17 到 218，通过位姿门控从 0 到 171，地图段数从 8 到 2。`110729_776644600` 的全速回放完成 957 帧，其中 952 帧跟踪 OK、577 帧通过位姿门控，无 IMU 覆盖提前结束。还用录制数据喂入实时线程，验证 32 帧通过实际的半分辨率桥接链路。实机重新采集和轨迹精度仍待验证。

### 4.6 data/imu 三段录制验证（2026-11-10）

同一段录制、同一套 ORB（800×600 半分辨率配置），分别用 lattice(480 Hz) 与 timestamp(−85 ms) 重放：

| 录制 | 双目对数 | 模式 | OK 帧 | IMU 初始化帧 | map epoch | 单帧耗时 | 错误 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 11:06（62 s） | 1851 | lattice | **1807（97.6%）** | 503 | 18 | 55 ms | 无 |
| 11:06 | | timestamp | 1390（75.1%） | 138 | 204 | 106 ms | 末尾样本不足 |
| 11:07（33 s） | 977 | lattice | **921（94.3%）** | 187 | 11 | 57 ms | 无 |
| 11:07 | | timestamp | 624（63.8%） | 3 | 130 | 104 ms | 末尾样本不足 |
| 11:20（33 s） | 987 | lattice | **930（94.2%）** | 92 | 12 | 55 ms | 无 |
| 11:20 | | timestamp | 614（62.2%） | 1 | 151 | 135 ms | 末尾样本不足 |

结论：三段数据上 lattice 的跟踪 OK 率 94–98%、epoch 11–18、无错误；timestamp 只有 62–75%、epoch 130–204，且全部在末尾报错。格点对齐的优势在这批真实数据上是可重复的。

剩余问题：lattice 仍有 11–18 个 epoch（约 0.3–0.5 次/秒），来自 `Fail to track local map`/`RECENTLY_LOST`，属于 ORB 跟踪本身而非时间对齐；10 分钟实机日志里 459 次重置中 466 条是“Not enough motion”，说明现场运动以旋转/小位移为主，触发了 ORB 的 IMU 初始化重置判据。

## 5. 复现

```powershell
# 实时 A/B（每段等待运动后计时）
.\.venv\Scripts\python.exe scripts\compare_vs030_alignment_live.py --seconds 120

# 同一段数据重放（抓 60 s，再离线跑 4 种参数）
.\.venv\Scripts\python.exe scripts\compare_vs030_alignment_replay.py --seconds 60
```

## 6. 改动文件

- 新增：`src/egohand/vio/imu_lattice.py`、`tests/test_imu_lattice.py`
- 修改：`src/egohand/vio/live_runtime.py`、`configs/vio/live_timing.yaml`
- 工具：`scripts/run_vs030_live_session.py`、`scripts/compare_vs030_alignment_live.py`、`scripts/compare_lattice_euroc.py`、`scripts/validate_lattice_alignment.py`、`scripts/capture_vs030_lag_dataset.py`、`scripts/analyze_vs030_lag_capture.py`
- 证据：`results/vio/vs030_lag_capture_*`、`vs030_lag_capture_analysis.json`、`vs030_lattice_validation.json`、`vs030_lattice_euroc_compare.json`、`logs/vio_vs030_live_*`

## 7. 限制与未决问题

- **包格点 +97 ms 已做离线相对标定**：需要接回相机验证实时链路，并用独立的硬件时序实验区分各级物理延迟。换固件后应重新扫描。
- 480 Hz 格点的实机 A/B 仍可在修正后的配置下重跑；不过 4.5 的录制回放已经在同一份真实数据上给出明确对比（912 vs 765 OK、407 vs 29 初始化、7 vs 13 epoch）。
- 包头时标与相机可直接匹配，包内驱动时间相对包头累积偏差约 3 ms/s。旧的 4.62 s 数值尚缺原会话文件复核。
- 格点假设“帧间隔内样本数 = Δt × rate”，丢样会造成相位滑移；当前靠缺口检测 + 重建对齐兜底，但没有逐样本的丢样补偿。
- **实机验证受运动激励支配**：同参数 3 次运行 epoch 从 26 到 200，初始化帧从 1 到 488；人手转动无法重复，因此“相对基线改善”尚未获得统计证据。需要转台/机械臂做标准化激励。
- 旧 `timestamp` 模式在实机运动中 6 次里 3 次直接报“样本不足”终止，已不适合作为稳定基线；其固定 −85 ms 无法跟上漂移是直接原因之一。
- 未完成轨迹精度（ATE/RPE）与长时稳定性验收；不能宣称定位精度。

## 8. 同图内位置跳变（2026-11-10 下午）

### 8.1 现象与两个独立成因

`map_epoch` 不变（同一张地图），轨迹中间出现 1.9 m / 61 m / 274 m 的位姿跳变，之后恢复稳定。原因有两条，互不相同：

1. **IMU 初始化/惯性 BA 期间尺度仍在收敛**：VIBA 1 完成后地图尺度还会被 VIBA 2 修正，同一物理位置在不同时刻被写成不同数值。
2. **跟踪丢失后的重定位落错位置**：14:18 的补丁把丢失处理改成同图内 `Relocalization()`（不再 Reset 地图）。重定位是纯视觉全局 PnP，不强制与丢失前连续、也不用 IMU 预测，因此可能解出一个几何自洽但物理上很远的位置。实测跳变全部发生在 `RECENTLY_LOST → OK` 的恢复点。

### 8.2 已实现：重定位的 IMU 一致性校验（C++ / ORB-SLAM3）

`Tracking::Relocalization()` 在接受候选位姿前增加校验（`src/Tracking.cc`）：

- 用 `mvImuFromLastFrame` 的陀螺增量（扣 `mLastBias`）积分出机体旋转 `Rinc`；
- 按 `mTcb`（T_cb，`mTcw = mTcb * Tbw`）换算成相机旋转增量 `dRpred = Rcb^T · Rinc · Rcb`；
- 与重定位结果相对上一可信帧的旋转/位移比较：
  - 平移上限 `0.30 m + 2.0 m/s × 丢失时长`
  - 旋转上限 `25° + 90°/s × 丢失时长`
  - 丢失超过 2 s 不做预测（跳过校验）
- 超限则丢弃该候选（打日志 `Relocalization rejected by IMU check`），保持 LOST，而不是让相机瞬移。

### 8.3 已实现：显示侧的刚体重锚（Python）

`src/egohand/vio/pose_continuity.py`：`D = A · R`，检测到物理上不可能的跳变时令 `A = D_prev · R_new⁻¹`，使新位姿落在上一显示位姿处；之后 `A` 不变，相对运动完整保留。接入 `live_runtime.py`（`quality.check` 之后），配置在 `live_quality.yaml`。

### 8.4 验证（data/imu 三段录制，同一套 ORB）

| 录制 | OK 帧 | epoch | 原始最大跳变 | 重锚后最大跳变 | 重锚次数 | 重定位接受/拒绝 |
| --- | --- | --- | --- | --- | --- | --- |
| 11:06 | 1692 | 1 | 23.27 m | **1.38 m** | 9 | 13 / 0 |
| 11:07 | 613 | 2 | 16.28 m | **1.88 m** | 40 | 10 / 0 |
| 11:20 | 678 | 2 | 10.46 m | **1.34 m** | 37 | 19 / 1 |

单元测试：`tests/test_pose_continuity.py` 5 项；连同 VIO 回归 25 passed。

### 8.5 已知限制

- 校验只拒绝“物理不可能”的重定位；**物理上可能但实际错误**的重定位仍会被接受（三段数据共 42 次接受、仅 1 次拒绝），这类跳变靠 8.3 的重锚消除显示断点，但**不保证位姿真值正确**。
- 本树 `Tracking::ResetFrameIMU()` 是空实现（`{ // TODO }`），所以日志里的 `RESETING FRAME` 不会真正改变位姿。
- 仍未完成：长时稳定性、真实轨迹精度（ATE/RPE）验收。


## 9. 保留重定位的地图坐标（历史记录，已被后续实现更新）

用户要求恢复后使用视觉在原地图中的定位，不应把位置强行贴回丢失前的点。实时默认关闭 `pose_continuity`；若显式启用实验性显示重锚，界面显示“非原地图坐标”。丢失后同地图恢复重新经过稳定窗口，显示轨迹断开；接受后使用 ORB 给出的地图坐标。

注意：当前 C++ 的重定位检查只以 `mvImuFromLastFrame` 预测上一帧到当前帧的旋转、比较上一帧和候选位置距离；不是从最后可信状态覆盖整个丢失时段的 IMU 位置外推，且某些条件下跳过检查。尚未实现完整外推与不确定性评估，也没有重定位真值验收。

修复离线回放将显示重锚配置误传给 PoseQualityGate 导致的启动异常。指定测试 29 passed、1 skipped。真实录制回放在 104 个处理帧后因 `[Errno 12] Cannot allocate memory` 终止：97 帧 ORB OK、60 帧报告 IMU 初始化、37 帧通过质量门控；结果仅为部分回放，不能作为恢复正确或长时稳定的证明。


## 丢失区间预测原型（历史记录，已接入后续实验流程）

`imu_loss_prediction.py` 新增可信状态快照、完整区间 IMU 覆盖及单调/缺口检查、体坐标候选位姿比较。位置/角度界限是随时间增长的经验包络，不是协方差或统计置信区间。单元测试包括静止、匀速、旋转、偏置、缺样、地图变化和不一致候选。**该模块尚未连接 ORB-SLAM3 跟踪状态或实际重定位候选；现有实时行为未改变，尚不能声称完整恢复已实现或验收。**

## 2026-11-10 丢失恢复实验实现与验收状态

- 基于 BA2 后通过稳定门控的机体状态、完整 IMU 区间进行外推，并在 ORB 重定位候选接受前检查；门限为经验包络，不是已标定协方差。
- 直接从短暂丢失恢复而未经过重定位回调时，只能进行事后检查；不一致或缺少证据时隔离旧锚点，不能宣称 ORB 内部候选已被事前拒绝。
- 地图版本包含 ORB 的重大地图修正计数；发生已报告的闭环等重大修正时不再使用旧锚点。该计数并不保证覆盖所有局部优化。
- VIO 子进程优先搜索其指定 Conda 环境的运行库。一次真实录制回放在强制拒绝测试中触发 8 次原生回调并拒绝，验证回调接线，不验证 IMU 预测精度。
- 仍需验证：存在可信锚点时的真实丢失恢复、地图修正、长期静止及运动对照、带独立真值的轨迹精度；VS030 IMU 字段、时间与标定仍需厂商确认。无可用 IMU 证据时，ORB 的视觉候选仍可能被接受并标记为未验证；不应把实验输出当作精确定位。

## 2026-11-10 后续：实际锚点与地图修正

- 在 VS030 录制 110623 上，BA2 完成后建立锚点并注入 8 帧遮挡；8 帧输出纯 IMU 外推，恢复时视觉/IMU 位置残差 0.01549 m。无独立真值，不能解释为真实定位误差。结果：`results/vio/recovery_real_anchor_occlusion_v4`。
- 在 EuRoC MH_01_easy 片段中，30 帧人为遮挡后直接恢复，残差 0.00613 m；同一地图版本内 188 个非隔离机体位置经过无尺度刚体对齐，与本地 OpenVINS 格式真值位置 RMSE 0.00473 m。数据短且经过离线拟合，不代表 VS030 或长期精度。结果：`results/vio/euroc_same_map_recovery_v3`。
- 发现 ORB 的 BA2 标志原本在优化前设置，已调整为成功完成之后发布。惯性全局优化现在会更新地图版本。不同地图版本的坐标不可直接相减：切断显示轨迹、废弃旧外推锚点并重新等待稳定；同版本发生异常跳变时才隔离。该机制不证明全局优化结果准确。
- 运行中的 GUI 进程仍使用启动时加载的旧 DLL，新进程选择新 DLL。纯 IMU 外推有最长时长和样本覆盖检查，并单独标记，不作为视觉轨迹。

## 2026-11-10 防止恢复隔离卡住

- 未验证位姿进入隔离后，默认等待 3 秒；仍无法验证时通过 ORB-SLAM3 的同步活动地图重置流程启动新的地图坐标系。旧的失败活动地图会被清除，跨地图轨迹不连接。阈值可在 `configs/vio/recovery_experimental.yaml` 的 `quarantine_timeout_s` 调整。
- 完全丢失后最多约 8 秒尝试原地图恢复，然后允许启动新地图。IMU 超出外推时限时不得标记为惯性验证通过。重置后仍需纹理和运动满足 ORB 初始化条件。
- 真实 VS030 录制的故障注入回放中，隔离约 3 秒后触发一次重置；后续 325 帧中 227 帧通过位姿门控，回放 450 帧正常完成。记录：`results/vio/auto_new_map_real_recording_v6_long`。触发隔离为人为注入，非现场实机长期验收。
- 当前运行中的 GUI / VIO 子进程不会自动替换已加载 DLL；需重启 GUI 启动新的 VIO 进程后加载最新修复。
- EuRoC 注入约 6.5 秒遮挡时，原地图恢复未成功，系统在约 8 秒窗口结束后新建地图；该次不能计为原地图重定位成功。记录：`results/vio/euroc_lost_original_map_v6`。

## EuRoC IMU 外推误差的初步量化

对本地 EuRoC MH_01_easy 的单个短片段，使用已完成 BA2 的 ORB 体速度与偏置、原始 IMU 以及 OpenVINS 格式真值，在每个锚点用真值姿态和位置作局部刚体对齐，排除跟踪丢失及地图版本变化，量化相对运动预测误差：1 秒位置误差中位数约 1.24 cm、P95 约 2.93 cm（20 个相关样本）；2 秒中位数约 4.73 cm、P95 约 10.14 cm（12 个相关样本）。这是带真值锚点对齐的离线短片段评估，不是系统在线绝对位置误差，也不能推广到 VS030。脚本 `scripts/evaluate_euroc_imu_prediction.py`，结果 `results/vio/euroc_imu_prediction_error.json`。

现有候选门限在 2 秒时为位置 4.3 m、旋转 205°，属于很宽的经验包络，不能解释为预测在厘米级准确；尚未用多数据集独立样本标定统计置信度。界面将通过状态标为”未超限”。

## 2026-11-10 末：ResetFrameIMU 实现与预测器自愈（本次修复）

### 修复 1：Tracking::ResetFrameIMU()

`third_party/ORB_SLAM3/src/Tracking.cc` 中的 `ResetFrameIMU()` 是空实现（TODO），导致重定位后 `mnFramesToResetIMU` 帧时打印 `RESETING FRAME!!!` 但不改变任何 IMU 状态。

实现要点：
- 优先使用 `mpLastKeyFrame->GetImuBias()`（最近一次惯性 BA 给出的偏置）；无关键帧时退回到 `mCurrentFrame.mImuBias`。
- 对 `mCurrentFrame.mpImuPreintegratedFrame`（最近帧到当前帧的预积分）和 `mpImuPreintegratedFromLastKF`（最近关键帧到当前帧的累积预积分，供 `PredictStateIMU` 使用）均调用 `Initialize(useBias)` 原位重置，避免 delete/new 引起所有权问题。
- 重置后，重定位后第一个完整帧周期的 IMU 预积分从正确的位姿状态重新起算，不再携带重定位前的陈旧积分。

影响范围：仅影响 `mSensor != System::STEREO`（即 Stereo-IMU 或 Mono-IMU）且地图已完成 IMU 初始化的会话；VS030 Stereo-IMU 配置满足此条件。

### 修复 2：recovery_coordinator.py — push 失败后清空污染样本

`begin_frame` 中当 `predictor.push(s)` 抛 `ValueError/OverflowError`（IMU 缺口或时间非递增）时，仅设 `invalid_reason` 而不清理 `predictor.samples`，导致后续每帧 push 都继续失败。样本列表持续累积脏数据，`invalid_reason` 只能等到地图 epoch 变化或建立新锚点才能清除。

修复：在 except 块补加 `self.predictor.samples = []`：
- 锚点**保留**：仍可用于重定位候选的一致性比较；
- 样本**清空**：下一帧的合法样本可正常 push 进去；
- **预测仍不可用**：新样本不覆盖锚点到当前时刻的完整区间，`predict()` 仍返回 'unavailable'，直到通过质量门控建立新锚点（此时 `set_trusted_state` 同时清零 `invalid_reason`）；
- 正常跟踪场景（无丢失、无重定位）：一旦下一帧 body 有 BA2 且未隔离，锚点被更新，`invalid_reason` 随之清除，系统自愈无需等待 epoch 变化。

### 测试验证

修改后运行 41 项 VIO 相关单元测试（含 `test_vio_recovery_coordinator`、`test_vio_recovery_monitor`、`test_imu_lattice`、`test_imu_loss_prediction`、`test_vio_recovery_callback_flow`、`test_pose_continuity`、`test_vio_pose_quality`、`test_vio_body_state`），**全部通过，无回归**。

### 仍需验证

- `ResetFrameIMU` 在实机重定位场景的表现：需在重定位后观察随后的 `PredictStateIMU` 轨迹是否更稳定（当前无独立真值）。
- 污染样本清空后，重定位候选的 IMU 一致性检查回落到锚点但样本不足的情形：此时返回 'unavailable'，不触发隔离（除非确认 'inconsistent'）——符合”数据不足时不可判断”的设计意图。
- 实机长期稳定性与轨迹精度验收尚未完成，上述修复不改变已有的”实验性”标注。

## 2026-11-11 背景导入后的继续修复

### 背景导入

- `claude-desktop-zh-cn` 中文补丁项目已存在于 `E:\EgoHandDataEngine\claude-desktop-zh-cn`（非 git 克隆，是解压后的源码树）。Windows 安装方式：退出 Claude Desktop → 双击 `install-windows.bat` → UAC 授权 → 选操作 `1`（安装中文补丁，第三方 API 模式）→ 选语言 `1`（简体中文）。脚本会备份被修改的文件并在卸载时恢复。注意：脚本会改写 `app.asar` / `ion-dist.asar` 并更新 `Claude.exe` 的 ASAR 头哈希，原有 Authenticode 签名会失效（脚本会修补 `cowork-svc.exe` 的签名校验分支）。当前沙箱是 Linux，无法直接执行 Windows 安装脚本，需在 Windows 机器上操作。

### 修复 3：地图尺度未定时不再把视觉位姿当作米制绝对坐标

**问题**：`inertial_scale_finalized` 字段早已在 `EgohandVioResult` 里存在，但 Python 侧此前完全没读它。VIBA2（惯性第二阶段优化）完成前，ORB 的世界坐标仍是纯视觉的任意尺度，1 个单位不等于 1 米。此时把 XYZ 显示为 "m"、并用它建立 IMU 外推锚点，得到的数值没有物理意义。

**修复**：
- `src/egohand/vio/recovery_coordinator.py`：只有尺度已定标（`inertial_scale_finalized is not False`）的位姿才能建立 IMU 外推可信锚点；未定时不建立锚点，候选检查返回 'unavailable'。
- `src/egohand/gui/vio_panel.py`：读数追加 "尺度未定标（非米制）" 标注。
- `src/egohand/vio/pose_quality.py`：**刻意不改**。早期曾试过在尺度未定时把 `pose_valid` 置 False，会让 VIBA2 之前的约 200 帧跟踪全部不显示，反而掩盖了跟踪连续性。位姿照常输出并参与跳变检测，但上层不会拿它建立米制锚点，GUI 用文字标注其单位不成立。

**验证**（Linux 沙箱，Python 3.10 无法运行完整 pytest 套件，因 `egohand.core.status` 需要 Python 3.11+ 的 `StrEnum`；测试改为脱离 conftest 单独执行）：
- 尺度未定时 `RecoveryCoordinator` 不建立锚点（`predictor.anchor is None`），候选检查返回 'unavailable'；尺度已定时建立锚点。
- `PoseQualityGate` 在尺度未定时仍输出位姿（`pose_valid=True`）并保留 `raw_pose_valid`，供跳变检测与连续性观察使用。
- 新增用例：`tests/test_vio_recovery_coordinator.py`、`tests/test_vio_body_state.py`。

### 修复 4：原生桥接库 ABI 校验

`EgohandVioResult` 的字段顺序决定 Python 侧 ctypes 的偏移。若 DLL 与 Python 结构体布局不一致，`inertial_scale_finalized` 之后的所有字段（两个 double、`map_epoch`、16 个 float）都会被错位读取，地图尺度状态会变成垃圾值，进而误判"尺度已定标"。

**修复**：新增导出 `egohand_vio_result_size()`，Python 侧在 `_bind_api()` 之后用 `ctypes.sizeof(_CResult)` 比对，不一致直接抛错并提示重新编译。旧 DLL 没有该导出时跳过校验（保持对已有 v6 DLL 的兼容），不阻断启动。

### 修复 5：撤销重复的地图版本通知

`LoopClosing::MergeLocal2()` 里新增的 `InformNewBigChange()` 是多余的：`Map::MergeLocal2()` 自身已经对两张地图调用了 `InformNewBigChange()`，而所有调用 `ApplyScaledRotation` 的路径（`LoopClosing.cc:1849`、`LocalMapping.cc:1309`、`LocalMapping.cc:1508`）都已覆盖。保留重复调用不会更安全，只会让地图版本号虚增、导致锚点被更频繁地废弃。已撤销。

### 定位结论：82 m "跳变"不是跟踪跳变

在 `logs/vio_vs030_live_1794307699783036000_poses.jsonl` 中，帧 484 出现从 `[-48.08, 46.06, 2.99]` 到 `[-24.69, -16.04, -45.65]` 的巨大位置变化。但逐帧追踪 `map_epoch` 显示：

- 帧 440：`map_epoch=4`，显示段 51
- 帧 452：`map_epoch=5`
- 帧 456–490：`map_epoch=6`
- 帧 484 的跳变发生在 `map_epoch=6` 内部

对照 worker 日志（`logs/vio_vs030_worker_1794307478980447700.log`），帧 300 前后有 `start VIBA 1/2`、`RESETING FRAME!!!`、`*Merge detected` / `Merge finished!` 与随后的 `*Loop detected` / `BAD LOOP!!!` 序列。这说明该段跨越了惯性初始化、地图合并与闭环，全局/惯性优化改变了世界坐标系，属于**坐标系统变化**而非单帧跟踪跳变。旧坐标与新坐标不可直接相减，这正是 `map_epoch` 机制要处理的情况。

**结论**：帧 484 的位置差不是缺陷，也不能用作精度指标；它反映的是地图坐标系被重建。原日志中"位姿突变已拒绝：平移 16.688 m"等条目属于同一类坐标系变化，需要结合 `map_epoch` 解读。

### 仍需完成

- 上述原生改动（`vio_bridge.cpp` / `vio_bridge.h`）**必须在 Windows 上用 MSVC 重新编译**，本沙箱无法执行；未编译前 `egohand_vio_result_size` 导出不存在，ABI 校验会被跳过。
- 完整 pytest 套件需在 Windows + Python 3.11+ 环境运行（本沙箱为 Python 3.10）。
- 恢复门限仍是宽的经验包络（2 秒时位置 4.3 m、旋转 205°），未经统计标定，界面措辞保持"未超限"。

### claude-desktop-zh-cn 安装说明

`E:\EgoHandDataEngine\claude-desktop-zh-cn` 与上游 `javaht/claude-desktop-zh-cn` 的 `main` 分支逐文件一致（diff 无差异）。

在 Windows 上安装：
1. 完全退出 Claude Desktop。
2. 双击 `claude-desktop-zh-cn\install-windows.bat`（脚本会自行请求 UAC 管理员权限）。
3. 选操作 `1`（安装中文补丁，第三方 API 模式），再选语言 `1`（简体中文）。
4. 脚本会备份被改动的文件、改写 `app.asar` / `ion-dist.asar`、更新 `Claude.exe` 内的 ASAR 头哈希，并修补 `cowork-svc.exe` 的签名校验分支。
5. 安装完成后 Claude 会自动重启；如未切换，左下角账号菜单 → Language → 中文（简体）。

也可直接走非交互入口（需管理员权限的终端）：
`scripts\install_claude_zh_cn.bat install zh-CN`
对应上游参数：`scripts\install_windows.ps1 -Action install -Language zh-CN`。
卸载/还原：`scripts\install_claude_zh_cn.bat uninstall`。

注意事项：
- 补丁会使 Claude Desktop 的 Authenticode 签名失效；上游已同时修补 `cowork-svc.exe` 的签名校验以避免 "RPC pipe closed"。
- 商店版（MSIX）若在 `WindowsApps` 下，脚本可能需要接管目录所有权（`-AllowTakeOwnership`），卸载后不会自动恢复 ACL。
- 官方账号登录页本身已支持中文，脚本不改写它；汉化针对的是第三方 API 模式的本地界面。
# 2026-11-11：实时进程 NaN 退出及重启保护

- `vio_vs030_worker_1794364067862063400.log` 在重定位、`RESETING FRAME!!!` 后
  报 Sophus `SO3::exp failed`，角速度增量为 NaN，随后 GUI 收到 WinError 10054。
  日志能确定进程异常退出，但不能仅凭日志证明 NaN 的唯一根因。
- 本地 `ResetFrameIMU()` 原实现对仍可能被帧/关键帧引用的预积分调用 `Initialize`，
  将累计时长、测量和协方差清零。此行为并非上游实现，存在破坏惯性约束的风险；
  已恢复上游未实现该函数的保守行为，编译独立 v9 桥，保留 v8。
- 实时工作进程连接异常时最多自动重新启动两次，每次清空 IMU 对齐与旧地图显示轨迹，
  重新初始化并建图；恢复期间无可信位姿。连续失败后停止并给出工作进程日志路径。
- 首次 995 帧回放受内存分配失败中断，未复现该次重定位路径；该结果不是完整验收。
- 随后串行对 `112020` 真实录制做 340 帧回放，对第 145–157 帧双图注入黑帧，
  日志实际出现 `Relocalized!!` 和 `RESETING FRAME!!!`，程序完成且未出现 Sophus NaN。
  该单次试验不证明 NaN 的唯一来源，也不能替代实机长时稳定性验收。

