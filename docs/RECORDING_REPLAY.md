# 录制（视频 + IMU）与离线 VIO 回放

更新：2026-11-10。适用于 VS030（VC020，xdsdk 0.2.0）数据源。本文说明 GUI 录制链路、录制目录格式、自动校验、视频导出和**不接相机**的离线 VIO 回放。

## 1. 为什么需要这一层

上一阶段（见 `handoff/VS030_VIO_背景信息导入_20261109.md`、`docs/VIO_IMU_TIMING_FIX.md`）在实机上做 IMU 对齐 A/B 时，每次都要重新手动转动相机采集；同参数多次运行的地图 epoch/初始化帧差异被运动激励支配，无法统计。正确做法是**录一次原始数据（双目 H.265 视频 + 原始 IMU 包），之后对同一份数据反复离线回放**不同参数。本层提供：

- GUI 里选择保存位置、点一下录制，同时落盘双目视频与 IMU；
- 停止录制自动生成 `replay_check.json`，回答“这段录制能不能回放”；
- 离线回放（GUI 按钮 + CLI），复用与实时完全相同的 ORB-SLAM3 桥、校正、IMU 对齐与位姿门控；
- 可选把录制导出成左右 MP4 + `imu.csv`，便于人工查看和第三方工具使用。

原始录制仍然是**唯一无损真源**：H.265 视频码流与 IMU 载荷按 SDK 原样保存，不转码、不重采样。导出 MP4 只是有损预览。

## 2. GUI 操作

1. 数据源选“新 SDK / VS030”，点“开始采集”（右栏“配置”需展开）。
2. 右栏“录制”页是**录制会话**面板（独立页签，避免与采集配置挤在窄栏里）：
   - **保存位置**：默认 `configs/app.yaml` 的 `recording_root`（相对工程根解析，缺省 `recordings`）。点“选择…”或直接改目录；该位置决定新会话写到哪里。
   - **开始录制**：立即在 `<保存位置>/session_YYYYmmdd_HHMMSS_<纳秒>` 新建会话；不会覆盖旧会话。
   - 录制中面板按 ≤2 Hz 刷新：**帧 / 视频包（左+右）/ IMU 包 / 耗时 / 已写 MB**。工具栏“录制”按钮与面板状态同步，也可用菜单暂停/停止。
   - **暂停/继续**：暂停期间不写原始数据，`events.jsonl` 显式记录用户主动跳过的区间。
   - **停止录制**：写盘线程排空后，后台自动执行录制校验，面板显示“可回放/不可回放 + 原因 + 帧数/IMU 条数/速率”。
3. 停止后出现**离线 VIO 回放…** 按钮（仅 VS030 且 `replay_ready=true` 可用）。点“参数”可展开对齐方式（`lattice` 默认 / `timestamp`）、相机—IMU 偏移 `lag_ms`、步长 `stride`（1 = 每帧都送 ORB；>1 隔帧送）；参数默认收起，保证面板在右栏一屏内放得下。点击回放后在后台线程执行，界面不卡；完成后显示 `replay.json` 摘要并提供“打开结果”。

> 相机采集与录制分属两个线程：预览/手部推理丢帧不影响原始录制；写盘队列溢出会显式停止录制并标记 `incomplete`，不会静默丢数据。

## 3. 录制目录结构（VS030）

```text
session_YYYYmmdd_HHMMSS_<ns>/
  manifest.json                 # 计数、SHA-256、recording_status、software_versions、replay_check
  replay_check.json             # 停止录制时自动生成的“能否回放”校验
  events.jsonl                  # 开始/暂停/继续/停止操作
  calibration.yaml              # 本次使用的设备标定快照
  kalibr_camchain.yaml          # Kalibr 原始 camchain（相机—IMU 外参与时移）
  kalibr_results.txt
  vio_timing.yaml               # 录制时的 IMU 对齐参数快照（回放默认取它）
  vio_quality.yaml              # 位姿门控参数快照
  vio_settings.yaml             # ORB rectified 配置快照
  rectification.npz             # R1/R2/P1/P2/Q 等校正矩阵
  raw/sdk_records.bin           # 【视频+IMU 无损真源】IPC v2 记录流
  raw/imu_records.bin           # 仅通道 7（IMU）记录的便捷副本，头+载荷
  raw/sdk_index.jsonl           # 每条 SDK 记录的 offset/sink/sequence/device_ts/…
  raw/camera_frames.jsonl       # 每个成功配对的双目预览帧的元数据与时间证据
  raw/frames.jsonl              # 非 VS030 数据源的 PNG 索引（VS030 为空）
  results/frames.jsonl          # 手部重建结果层（实时可能少于原始帧）
```

`raw/sdk_records.bin` 每条记录：头 `<IHHIIQQQQQQ>`（magic `0x58444652`、version 2、header_size、sink、payload_size、sequence、device_ts、capture_ts_ns、arrival_ts_ns、host_receive_monotonic_ns、host_receive_unix_ns）+ 原始载荷。

- sink 4 = 左相机 H.265，sink 5 = 右相机 H.265，sink 7 = IMU（每 40 B 一条 `<QQ6f`：两个 ns 时间 + accel xyz + gyro xyz，实测约 480 Hz，单包 11–16 条）。
- 双目成帧条件与实时一致：两通道同 `sequence` 且 `device_ts` 相等。

## 4. 自动校验 `replay_check.json`

停止录制时由 `egohand.storage.sdk_recording.build_replay_check` 生成（`manifest.json["replay_check"]` 同时保存），只读快检，约 0.3–0.5 s：

| 字段 | 含义 |
| --- | --- |
| `replay_ready` / `reason` / `error` | 结论与原因；校验异常只写 `error`，不改录制状态 |
| `recording_status` | `transport_unverified`（VS030 正常结束）/ `incomplete`（写盘过载等） |
| `frames.index_pairs` | 索引中成功配对的双目帧数 |
| `frames.index_unpaired_packets` / `index_device_ts_mismatch` | 未配对包 / device_ts 不一致数（应为 0） |
| `frames.paired_device_ts_first/last` | 配对帧设备时间范围（严格递增才通过） |
| `frames.beginning_sample` / `middle_sample` | 开头/中部抽样解码结果（对 `sdk_records.bin` 实际解码几帧，证明视频可解） |
| `imu.packets` / `rows` / `rate_hz` | IMU 包数、行数、实测平均速率 |
| `checks[]` | 逐项中文结论（录制状态/记录文件/索引/配对/偏移/IMU/抽样解码/总体） |
| `warnings[]` / `errors[]` | 不阻塞的告警与错误 |

限制：只抽样解码开头与中部，不逐帧全解；全量结构核查仍可用 `scripts/validate_vs030_recording.py`。若录制起点恰在 GOP 中间且开头 4–8 s 内没有 IDR，开头抽样可能失败——这是编码边界，不是数据损坏。

## 5. 导出视频 / IMU（可选，有损预览）

```powershell
.\.venv\Scripts\python.exe scripts\export_recording_videos.py `
  --recording recordings\session_xxx --output recordings\session_xxx_export --scale 0.5
```

输出 `left.mp4`、`right.mp4`（OpenCV mp4v，或回退 PyAV libx264）、`imu.csv`（`time_a_ns,time_b_ns,ax,ay,az,gx,gy,gz`）、`frames.csv`、`export.json`（帧数、编码器、耗时）。默认输出 `<recording>/export/`，已存在则拒绝覆盖（exit 2）。回放本身**不依赖**导出文件。

## 6. 离线 VIO 回放

### GUI

录制面板“离线 VIO 回放…”：后台线程执行，完成后面板显示 `frames.processed/ok/imu_initialized/valid_poses`、`map_epochs`、输出目录；可在面板“打开回放结果”。

### CLI

```powershell
.\.venv\Scripts\python.exe scripts\replay_recording_vio.py `
  --recording recordings\session_xxx `
  --alignment lattice --lag-ms -85 --stride 1 --max-frames 200
```

参数：

- `--alignment lattice|timestamp`：默认取录制目录的 `vio_timing.yaml`。`lattice` 按“相机帧间隔 × 配置采样率”做计数锁定重采样，不依赖会漂移的 IMU driver 时间戳；`timestamp` 用 `camera_to_imu_lag_ms` 做固定时移后再按时间戳选样本。
- `--lag-ms`：仅 `timestamp` 语义下有意义（相机时间 = IMU 时间 − lag）；`lattice` 下保留在报告里便于对照。
- `--stride N`：每 N 帧送一帧给 ORB（lattice 下会按跳过的时长补偿相位），用于快速参数扫。
- `--max-frames N`：只回放前 N 帧（冒烟验证）。
- `--start-stamp-ns NS` / `--end-stamp-ns NS`：按相机绝对纳秒时间选闭区间（录制 `device_ts × 1000`，可从已有 `poses.jsonl` 取 `timestamp_ns`）。估计器从区间内首个选中帧重新开始，该帧不带 IMU；此前帧仅用于顺序解码和推进传感器缓冲。保留原时间戳、录制帧索引及整段录制的 stride 基准。`replay.json.image_range` 记录请求边界、实际首末处理时间及起点前跳过帧数。空区间报告失败。区间选择用于诊断，不应按输出结果倒选起点宣称精度改善。
- `--output DIR`：默认 `<recording>/replay_vio/replay_<unixns>/`。

输出：

```text
replay_<unixns>/
  run.log        # egohand-vio 子进程 stdout（ORB-SLAM3 日志）
  poses.jsonl    # 每帧一条：tracking_state/imu_initialized/pose_valid/map_epoch/quality_status/
                 #   matrix（T_world_from_gui_left，已乘 T_orb_left_from_gui_left）/
                 #   sdk_timestamp_audit / replay 元信息
  replay.json    # 报告：status、timing、frames{decoded,processed,skipped,ok,imu_initialized,valid_poses}、
                 #   map_epochs、reset_events{system_reset,motion_insufficient,map_epoch_changes}、imu、elapsed_s、error、output
```

回放按录制顺序逐帧送桥；显式 stride、时间范围、帧数上限和 IMU 覆盖不足会影响处理范围。输入顺序可复现，ORB-SLAM3 的并发调度仍可能造成轨迹和初始化结果差异，不能保证逐字节确定性。实时链路 `frames` 队列 `maxsize=1` 会丢帧，这一点与离线回放不同。回放使用 `egohand-vio` 独立进程，DLL 由显式实验路径或角色清单选择，Python 路径由 `egohand.vio.live_runtime.PYTHON` 提供。

## 7. 实测验证（2026-11-10，VS030 实机）

- 录制：`recordings/session_20261110_095033_463299800`，31.6 s，834 对双目帧（`sdk_count_by_sink` 4/5/7 = 834/834/832），`sdk_records.bin` 20.6 MB，`replay_check`：**replay_ready=true**、0.38 s、IMU 832 包 / 13281 行 / 479.1 Hz、开头/中部抽样解码均通过。
- 导出：`--scale 0.5` → 左右 MP4 各 834 帧 800×600@30fps，`imu.csv` 13281 行。
- 离线回放（同一录制、lattice 默认参数、`--stride 5` 全段）：`status=completed`，decoded 832 / processed 167、**ok 159、imu_initialized 122、valid_poses 102**、`map_epochs=[0,1]`、`motion_insufficient` 文本 38 次、耗时 59 s；lattice 每帧 16 样本、相位无漂移（`lattice_consumed=2656=166×16`）。
- 确定性：同参数两次 30 帧回放的逐帧时间戳与每帧 IMU 样本数完全一致（实时链路会因 `frames` 队列丢帧而不同）。
- GUI 离屏真机全链路（`logs/gui_recording_validation.json` + 截图 `logs/gui_recording_inprogress.png`、`logs/gui_recording_panel.png`）：VS030 1600×1200 → 选目录 → 开始录制 → 暂停/继续 → 停止；会话 290 对双目、IMU 4620 行，`replay_check` 可回放，面板显示摘要且回放按钮可用。
- UI 布局（2026-11-10 下午，用户反馈拥挤后调整）：录制面板独立成右栏“录制”页签，参数默认收起。1500×950 窗口下右栏宽 320、面板 286×319（收起）/451（展开），均小于页签可视高度 508；280/240 px 宽度下关键控件无横向截断（`logs/ui_layout_validation.json`、截图 `logs/ui_layout_stopped.png`、`logs/ui_layout_panel_widget.png`、`logs/ui_layout_panel_replay.png`）。
- 真实数据特性（来自上述回放）：
  - **IMU 比视频早结束约 49 ms**（13281 行 / 27.72 s vs 相机 27.77 s）；全段 lattice 到最后一帧时样本不足。回放按实时“样本不足跳过该帧”语义处理：不插值不外推，提前结束但 `status=completed`，`imu_coverage_exhausted=true` 并写入 `warnings`。
  - 即使相机静止，该场景 ORB 也能短暂初始化/跟踪（159/167 OK、epoch 0→1）；这不构成精度验收，只说明回放链路真实工作。
  - `--alignment timestamp` 的默认 `-85 ms` 对这份录制**不适用**：本样本 IMU `time_a` ≈ `device_ts×1000 + 1.1 ms`，减 `-85 ms` 后样本整体晚于相机时间约 85 ms，第 2 帧即报样本不足（实时链路同样会失败）。报告会提示改用 `--lag-ms 0` 或 `lattice`；`lattice` 是默认且不依赖该偏移。

## 8. 限制与未验证项（必须保留的口径）

1. `device_ts` 单位（按 µs×1000 转 ns）、是否曝光时刻、通道 7 IMU 载荷轴/单位/两时间字段语义，仍缺厂商协议确认；回放只是复用实时链路同样的假设，不代表时延标定真值。
2. 回放画面帧序确定，但不等于实时行为：实时 `LiveVio.frames` 队列 `maxsize=1` 会丢帧（约 6 Hz 有效输入），回放不丢帧；两者结论不能直接互换。
3. `--alignment timestamp` 对 `lag_ms` 敏感，且不同录制/固件的 IMU-相机关系可能不同；先用 `lattice`，需要 timestamp 时用 `--lag-ms` 做小范围对照并记录。
4. 实机 `map_epoch` 仍漏计 `LocalMapping` 原地清图；`reset_events.motion_insufficient` 从 `run.log` 文本统计，粒度有限。
5. 长录制回放耗时：桥启动 15–28 s（ORB 词典），之后按 ORB 单帧耗时线性增长；先用 `--max-frames`/`--stride` 冒烟。
6. 导出 MP4 有损（mp4v/libx264），回放本身永远读原始 H.265+IMU。录制暂停会留下时间缺口（`raw` 里没有该区间数据）；VIO 回放建议连续录制，`replay_check` 会给出缺口提示。
7. GUI 录制依赖数据源已启动（采集线程运行中），当前不提供“只录制不推理”的独立模式。

## 9. 相关文件与测试

- 解析/校验/解码：`src/egohand/storage/sdk_recording.py`
- 录制写盘：`src/egohand/storage/recorder.py`
- 离线回放：`src/egohand/vio/replay_runtime.py`、`scripts/replay_recording_vio.py`
- 视频导出：`scripts/export_recording_videos.py`
- GUI：`src/egohand/gui/recording_panel.py`、`main_window.py`、`workspace.py`
- 测试：`tests/test_sdk_recording.py`、`tests/test_vio_replay.py`、`tests/test_recording_panel.py`、`tests/test_replay_gui.py`
