# VS030 设备侧时序与 IMU 载荷证据（2026-11-09 18:00–18:20）

> 目的：用设备本体的代码/日志/IIO 状态，回答交接说明里“`<QQ6f` 假设、双时间戳、−85 ms、曝光时刻”这些此前只能靠相关性推断的问题。
> 边界：只做只读探测；除 SDK 正常启停预览（设备自行恢复）外，未修改设备配置、未写 sysfs、未重启服务。

## 0. 访问方式

- 设备 `<device-ip>:22`，root 登录，凭据为厂商 SDK 公开默认值（`SDK/delivery/include/xdsdk/xdsdk_sftp.h:31-32`：user `root` / password `<vendor-default-password>`）。
- 设备系统：`Linux rv1126b-buildroot 6.1.141 #75 SMP aarch64`；主程序 `/root/vision_service`（1.3 MB，**保留符号表**），另有 `/root/fsync_lock`、`/root/ip_discover`、`/root/scripts/run_all.sh`、`/root/xdyn.json`。
- 设备时钟未对时：`date` 显示 2021-01-08（NTP 未生效）；数据时基不用 wall clock，见下。

## 1. 结论摘要

1. **UDP 通道 7 载荷 = N×40 字节的原始 `ImuSample` 数组**，不是带包头的自定义帧。`SendImuGroupLocked` 用 `size/40` 求样本数，再以 sink=7、`n*40` 字节整包发送。
2. **40 字节布局（实测确认）**：`uint64 ts_a` + `uint64 ts_b` + `float accel[3]` + `float gyro[3]`。
   - 静止实测：`accel` 模长约 **9.70 m/s²**（Z 轴 −9.65），`gyro` 模长约 **0.005 rad/s**。→ 三轴顺序为加速度在前、角速度在后，单位是 **m/s² / rad/s**（SI），与 GUI 现有 `values[:3]`=accel、`values[3:]`=gyro 的假设一致。
   - 采样间隔中位数 **2.08472 ms ≈ 479.7 Hz**，与 IIO `sampling_frequency=480` 一致；每包 640 字节 = 16 条记录（约 33.4 ms 数据）。
3. **两个时间戳都在同一时基**：设备 IIO 的 `current_timestamp_clock = monotonic`（CLOCK_MONOTONIC，约等于 boottime），数值约 `1.02e13 ns`（开机 10 200 s 量级）。
4. **`ts_b − ts_a` 是固定常数 4 146 780 ns（≈ 2 个采样周期），量化步长 22 660 ns**，不是抖动。说明两条时间戳来自同一批 FIFO 的固定相位关系，而不是两个自由运行的时钟。
5. **相机和 IMU 的 UDP 包头 `device_ts` 可以共用帧标签**：2026-11-10 对五份录制复核，6,006 个 IMU 包中有 5,998 个包头标签与相机精确匹配。包内 `ts_a`、`ts_b` 则由驱动计时产生，和包头时标之间实测每秒累积约 3 ms 偏差。两者数值同量级不足以证明它们能直接对齐。详见 [包头和相位复测](VIO_IMU_TIMING_FIX.md#46-用-udp-包头定位时间差复测相位2026-11-10)。
6. **固件自己承认原始 IMU 时间戳不能用于对齐**。`vision_service` 内嵌的 `xdyn.Imu` JSON schema 写明：`accel_driver_timestamp_ns` / `gyro_driver_timestamp_ns` 是内核根据 IMU 内部 RC tick 合成的，“**NOT the true sampling instant, drifts ~0.2% vs the frame clock. Used only for drop detection, never for alignment**”。
7. **固件有“帧格点”重定时机制**：MCAP 路径把 IMU 样本放到 `exposure_start + i*T/N` 的格点上（`timestamp_retimed`、`sample_index`），并丢弃格点外样本（pre-roll/post-roll/gap 日志）。固件日志有 `udp imu: no frame anchor, header timestamp stays on driver clock`。对实际收到的 UDP 记录，应区分 SDK `FrameDesc.device_ts` 标签与载荷内每条样本的驱动时间。即使包头标签能匹配相机，它也需要经相位标定才能用于安排包内样本。
8. **曝光时刻确实存在且被固件校正**：`fsync` 上升沿 = 曝光开始，双目共用同一触发沿；`ExposureClock` 统计 `frames=51161 lock=96.7% subtract=3.3% raw=0.0% reject=1671`，即 96.7% 的帧时间戳已“锁定”在曝光基准上，3.3% 需要修正。
9. **最重要的新结论：IMU 驱动时钟相对相机时钟以约 0.3% 的速率漂移**。8 分钟实测拟合：`lag(t) = −2.995 ms/s · t − 17.7 ms`（10 s 窗、5 s 步长、12 个有效窗），残差 RMS 12.3 ms；若用单一常数时延，残差 60 ms。对全段做仿射补偿（`t_cam = t_imu + 17.7 ms + 0.003·(t−t0)`）后，视觉旋转与陀螺的全段相关从 **0.42 提升到 0.89**。这直接解释了：35 s 数据上“−85 ms 相关 0.991”只是窗口平均；0/−85 ms A/B 都不能改善；ORB 反复初始化。
10. **每帧 IMU 样本数 = 16.0**（480 Hz / 30 fps，实测 64 条 / 133.3 ms，中位数）。这为“按帧格点重排 IMU”提供了可行性依据（与厂商 MCAP 的 lattice 一致）。

## 2. 证据明细

### 2.1 IIO 与驱动

```text
iio:device1 name=lsm6dsv16x_gyro   sf=480.000000  clock=monotonic
iio:device2 name=lsm6dsv16x_accel  sf=480.000000  clock=monotonic
dmesg: ODR-triggered mode: 8 samples per INT2 reference period (readback 0x08)
dmesg: ASYNC HW timestamp: disabled
```

- 陀螺/加速度各是一个 IIO 设备，FIFO 水位 8（≈16.7 ms 一次中断），硬件异步时间戳被禁用 → 内核用 RC tick 合成时间戳，于是有 schema 里说的漂移与固定相位。
- 相机触发：`run_all.sh` 调用 `cam_fsync.sh 30`；此前现场已测得 PWM period 33,333,333 ns、duty 16,666,666 ns、enable=1。

### 2.2 载荷实测（静止，20 帧 / 19 包 / 304 条记录）

```text
sizes [640]
ts_a dt median 2.08472 ms
ts_b-ts_a: min 4,146,780 ns  median 4,146,780 ns  max 4,169,440 ns  stdev ≈10.7 µs
triplet1(accel) mean [0.047, -0.070, -9.697]  norm 9.698
triplet2(gyro)  mean [-0.0038, 0.0024, -0.0027] norm 0.0052
```

原始行示例：

```text
(10222462911106, 10222467080546, 0.0455, -0.0718, -9.6455, -0.00305, 0.00183, -0.00244)
```

→ 时间戳单位为 ns、数值连续；加速度受重力；陀螺静止接近零。完整输出见 `results/camera/vs030_imu_payload_probe.txt`。

### 2.3 相机 vs IMU 时钟域

```text
cam_device_ts_raw 10232594667 → ×1000 = 10,232,594,667,000 ns
imu ts_a 最近值             ≈ 10,232,660,232,533 ns
差值 −65.6 ms（等于当前帧解码完成时 IMU 已多走的量，不是时钟偏差）
```

同一时刻两者都是 `1.02e13` 量级、以 2.08 ms/33.3 ms 的节奏推进 → 同一 boottime 时基。完整输出见 `results/camera/vs030_clock_domain_probe.txt`。

### 2.4 固件自述（二进制内嵌 schema，原文摘录）

```text
"mechanism":"hardware trigger sync: fsync rising edge = exposure start, both cameras share the same trigger edge"
"clock_source":"CLOCK_BOOTTIME (kernel boot clock; NTP only sets the wall clock, not the data timebase)"
"timestamp_point":"exposure_start"
"accel_driver_timestamp_ns":"... synthesized by the kernel from the IMU internal RC tick counter - NOT the true sampling instant, drifts ~0.2% vs the frame clock. Used only for drop detection, never for alignment."
"timestamp_retimed":"True when placed on the frame lattice: exposure_start + i*T/N"
"sample_index":"Index within the frame lattice (0..N-1); -1 = not on lattice"
```

### 2.5 运行日志（`/root/log/vs/vs_2.log`）

```text
[ExposureClock] frames=51161 lock=96.7% subtract=3.3% raw=0.0% reject=1671
udp preview: video=h265 pcm=0 imu=1
HdlImu devId=0 PthCollect exit (frames=958)   # ≈2 s，479 Hz
udp imu: no frame anchor, header timestamp stays on driver clock
IMU lattice: re-anchor %+.3f ms at frame %u    # MCAP 路径
IMU pre-roll/post-roll dropped ...             # MCAP 路径
```

### 2.6 代码路径（符号级）

- `XD::HdlSinkUdp::SendImuGroupLocked(void const*, size_t, size_t)`：`size/40` → 记录数，`SendStart(..., 1, n*40)` + `SendSignEt(buf, 7, n*40)`。
- `XD::HdlSinkUdp::SendImu(const ImuSample&)`：把 40 字节样本原样入队，聚合后整包发送。
- `XD::HdlMcapWriter::writeImuMessage(...)` / `flushImuIntervals(...)`：做格点重定时。
- `XD::HdlImuCollector::ReadFrame(short*, short*, ulong&, ulong&)`：读 IIO FIFO 的 accel/gyro 原始值与两个时间戳；`ToPhysical` 应用 scale（gyro 0.000610865、accel 0.002392822）。

### 2.7 8 分钟漂移实测（2026-11-09 18:29–18:36）

- 采集：`scripts/capture_vs030_lag_dataset.py 480`，期间手动缓慢转动相机（先动后静）。
- 数据：IMU 228,583 条（479.68 Hz）、视觉旋转 3,084 条（约 6.4 Hz）、SDK 0 坏包/CRC/丢帧。
- 方法：以 10 s 窗、5 s 步长，逐窗在 ±300 ms 内搜索视觉旋转与陀螺（已按 `T_imu_from_orb_left` 转到相机系）的最佳时延；只保留 `corr>0.85` 且 `motion_std>0.2` 的窗。
- 结果（12 个有效窗，覆盖前 70 s）：

```text
lag(t) = -2.995 ms/s · t - 17.7 ms        # t 为采集内时间
RMS 残差 12.3 ms；单常数时延残差 60.0 ms
滞后从 t=0 的 -50 ms 单调变到 t=65 s 的 -240 ms
```

- 仿射补偿验证（全段 3084 个视觉样本）：

```text
t_cam = t_imu + 17.7 ms + 0.003·(t_imu - t0)
零时延相关：0.4205（原始） → 0.8873（仿射补偿后）
```

- 每帧样本数：133.3 ms 间隔内 64 条（中位数），折算 16.0 条 / 33.3 ms，与 480 Hz ÷ 30 fps 一致。

**解释**：`dt` 每 10 s 增加约 30 ms，折合 **0.3%（3000 ppm）**，与厂商 schema 的 “drifts ~0.2% vs the frame clock” 同量级。因为偏移随时间线性增长，任何固定 `camera_to_imu_lag_ms` 都只能在一小段内对齐；35 s 诊断得到的 −85 ms 正是这段的平均值。这就是“相关系数很高但 A/B 不改善、ORB 反复初始化”的直接原因。

数据文件：

- `results/vio/vs030_lag_capture_20261109_183614.npz`（数值，不含图像）
- `results/vio/vs030_lag_capture_analysis.json`（逐窗时延与漂移拟合）
- `scripts/capture_vs030_lag_dataset.py`、`scripts/analyze_vs030_lag_capture.py`

## 3. 对交接说明 6 个问题的影响

| 问题 | 设备证据带来的修正 |
| --- | --- |
| 1. 反复初始化/重置/静止跳变根因 | 新增两个候选：**有效 VIO 仅约 6 Hz**（≈4/5 帧未进入 ORB）与**IMU 两时间戳固定相差 4.15 ms**；再加上 ORB 对“运动不足”的 10 s 判据。设备侧不认为原始 IMU 时间可用于对齐。 |
| 2. `<QQ6f`、陀螺插值、时差方向、帧间选取 | 布局已由设备二进制与实测确认（accel、gyro、SI 单位、40 B/条、640 B/包）。但“两个时间戳=加速度/角速度采样时刻”仍是推断；厂商 schema 明确说这两个 driver 时间戳只可用于丢样检测，**不可用于对齐**。 |
| 3. 双目互换、校正矩阵、`T_b_c1`、输出变换 | 设备确认 fsync 上升沿=曝光开始、双目同一触发沿；`device_ts` 与 IMU 同域，因此几何外参之外不再需要“跨时钟”转换。`T_b_c1` 方向仍应独立验证。 |
| 4. −85 ms 相关高但 A/B 无改善 | 已定量：IMU 时钟以约 −3 ms/s（0.3%）相对相机漂移；−85 ms 是 35 s 窗口的平均值。单常数时延的残差 60 ms，仿射补偿把全段相关从 0.42 提到 0.89。**A/B 无改善是必然的，不是实验噪声。** |
| 5. 合法静止 vs 激励不足 vs IMU 语义 vs 跟踪失效 | 静止时 0 初始化与 ORB 的激励判据一致；但 6 Hz 会放大“运动不足”。设备侧能提供 `sample_index`/`timestamp_retimed`，可用于构造“格点一致性”指标区分 IMU 语义问题。 |
| 6. 丢帧、同步队列、门控、epoch、线程关闭 | 新增：epoch 漏计 255 次原地重置（上一轮结论）；UDP IMU 无帧锚点，只能拿 driver clock；固件已实现格点重定时但只用于 MCAP。 |

## 4. 建议的下一步（按优先级）

1. **先修时间映射（最高优先级）**：把“固定 lag”换成随时间线性变化的模型，二选一：
   - **仿射**：`t_cam = t_imu − (lag0 + drift_ppm·(t_imu − t0))`。本次实测 `drift ≈ −3000 ppm`，采集起点 `lag0 ≈ −17.7 ms`；但 `lag0` 每次上电/开流都可能不同，需要每次会话用一段运动重新估计（或在启动时做 10 s 自校准）。
   - **格点（推荐，与厂商 MCAP 一致）**：完全忽略 driver 时间戳做对齐——把每帧间隔内实际到达的 16×N 个样本按 `exposure_start + i·T/N` 均匀铺开，driver 时间只用于丢样/缺口检测。这样 0.3% 漂移被结构性消除，也不需要每次估 `lag0`。
2. **MCAP 录制交叉验证**：录 30–60 s 带 IMU 的 MCAP，解析 `timestamp_retimed`/`sample_index` 与 `rec.meta`，验证格点实现与厂商一致，并量化 `device_ts` 相对 `exposure_start` 的偏移。
3. **修正后重跑实时 A/B**：指标改成“首次 IMU 初始化耗时、10 min 内原地重置次数、轨迹是否连续、ATE（若有真值）”，不再用 epoch 数；同一输入跑 3 次看方差。
4. **配套小修**：陀螺插值统一用 `ts_b` 相位；在 `LiveVio._run` 记录逐帧耗时，确认 6 Hz 瓶颈（若 ORB 单帧 ~160 ms，考虑 10 Hz 输入或降特征数）。

## 5. 复现

```powershell
# 主机侧（只打印数值）
.\.venv\Scripts\python.exe scripts\probe_vs030_imu_payload.py
.\.venv\Scripts\python.exe scripts\probe_vs030_clock_domain.py

# 设备侧（只读）
plink -ssh -pw <vendor-default-password> root@<device-ip> "cat /sys/bus/iio/devices/iio:device1/current_timestamp_clock; ..."
```

产出文件：

- `results/camera/vs030_imu_payload_probe.txt`
- `results/camera/vs030_clock_domain_probe.txt`
- `results/camera/vs030_device_imu_disasm.txt`（`vision_service` 相关函数反汇编，供复核）
