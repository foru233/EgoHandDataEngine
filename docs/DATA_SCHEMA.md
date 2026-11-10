# 结果数据格式 1.1.0（兼容读取 1.0.0）

2026-11-03 增量：Joint3D 追加可选 `depth_std_m`、`position_std_m`、`position_covariance_m2`、`uncertainty_status`。旧 JSON 缺少这些字段时加载为 None，不补 0。旧结果的 schema_version 保留原值。原有 quality_score 公式与含义不变。

depth_std_m = sqrt(Σ[2,2])；position_std_m = sqrt(协方差最大特征值)，不是平均位置误差。协方差是对应输出相机坐标系下的 3×3 米²矩阵；仅传播假定的二维像素随机噪声，不覆盖系统偏差、错误关联、标定误差或时间不同步。默认诊断模式；显式拒绝模式新增 HIGH_UNCERTAINTY。详见 UNCERTAINTY_MODEL.md。

CSV 原列顺序保留，末尾追加以上四列，协方差以严格 JSON 矩阵字符串保存。NPZ 追加 depth_std_m / position_std_m / uncertainty_status [N,H,21] 与 position_covariance_m2 [N,H,21,3,3]，以及逐帧 result_schema_version；原数组键和单位保持。无效/不可用字段 JSON/CSV 为 null/空，NPZ 为 NaN，不能读取为零误差。

录制容器/raw 索引结构仍为 1.0.0；新 manifest 另写 result_schema_version=1.1.0，结果 JSONL 每行自带 schema_version。旧录制无需迁移。

评测 dataset/sequence/frame、processing_run_id、joint_mapping_version、calibration_fingerprint、metric_valid、gt_available、uncertainty_assumptions 放在结果诊断及独立 provenance/manifest 中。HOT3D 内部虚拟双目结果在失败案例中标注 virtual_left_optical，正式 results.json/CSV/NPZ 已旋转回原始 1201-1 左光学坐标，单位米。二维 UV 仍属于 virtual_rectified_pixels，诊断中保存变换矩阵。

同步分层为 HARDWARE_VERIFIED / DATASET_DOCUMENTED / USER_ASSERTED / SOFTWARE_ESTIMATED / UNKNOWN。离线数据集配对需显式 benchmark context，元数据单独写 DATASET_DOCUMENTED 不能绕过原采集门控。数据集纳秒时间戳和硬件同步精度验收分开；本轮 hardware_sync_precision_verified 始终为 false。

所有结果包括 schema_version/frame_id/timestamp_ns/session_id/source_type/calibration_id/coordinate_frame/unit/reconstruction_status/processing_time_ms/diagnostics。
每个 Hand3D 固定 21 Joint3D，未匹配 track_id=-1。一个视图单独出现的手保持无效三维结构及该视图二维观测。
normalized_landmarks 是 MediaPipe 归一化坐标（其 z 若存在保留），pixel_landmarks 为 21×2 像素；world_landmarks 为模型局部 21×3，与双目三维分开。
关节索引：0 WRIST；1–4 THUMB_CMC/MCP/IP/TIP；5–8 INDEX_MCP/PIP/DIP/TIP；9–12 MIDDLE；13–16 RING；17–20 PINKY。
精确名称和骨架连接位于 detection/landmarks.py。

## 原始录制

目录 manifest.json 保存 source metadata、算法配置和模型摘要，calibration.yaml 保存标定快照。
raw/left、raw/right 为无损 PNG；raw/frames.jsonl 每行保存路径、全部时间、源、图像几何、元数据和二维输入。
results/frames.jsonl 保存实际处理结果，可含少于原始层的帧数（实时推理丢帧）。通过 frame_id+timestamp_ns 关联。
manifest recording_status 为 recording/complete/incomplete，出现过载与写入失败保存原因。中断后 reading 状态可恢复完整行，但不能冒称完整录制。
录制暂停/继续操作另存 events.jsonl，包含操作系统时间和最后已提交原始帧，显式记录用户主动跳过的采集区间。
有效标定同时保存 rectification.npz（R1/R2/P1/P2/Q、新 K 和 ROI），manifest 保存算法/NumPy/OpenCV/MediaPipe 版本；鱼眼 API 未提供的 ROI 保存为空数组。
合成录制 metadata 明确 ground_truth/synthetic；Ground Truth 在 raw metadata 中，算法不读取其 xyz。
重新处理另写目录，不修改 raw。实时/离线处理标识位于 diagnostics.processing_mode。

## VS030 原始 SDK 录制（视频 + IMU）

数据源为 VS030 时，原始层保存 SDK 原始载荷（不转码）：

- `raw/sdk_records.bin`：IPC v2 记录流，每条头 `<IHHIIQQQQQQ>`（magic 0x58444652、version 2、header_size、sink、payload_size、sequence、device_ts、capture_ts_ns、arrival_ts_ns、host_receive_monotonic_ns、host_receive_unix_ns）+ 载荷；sink 4/5 为左右 HEVC 视频，sink 7 为 IMU（40 B/条 `<QQ6f`）。
- `raw/imu_records.bin`：仅 sink 7 的便捷副本（同头 + 载荷）。
- `raw/sdk_index.jsonl`：每条记录 offset/record_bytes/sink/sequence/device_timestamp_raw/…
- `raw/camera_frames.jsonl`：成功配对的双目预览帧元数据（时间证据、IMU 包计数）。
- `vio_timing.yaml`/`vio_quality.yaml`/`vio_settings.yaml`/`kalibr_camchain.yaml`/`kalibr_results.txt`/`rectification.npz`：录制时使用的 VIO 参数与标定快照；离线回放默认读取这些快照。
- `events.jsonl`：录制开始/暂停/继续/停止；暂停区间不写 SDK 记录（时间缺口）。
- `replay_check.json`（停止录制时自动生成，约 0.3–0.5 s 只读快检）：`replay_ready`/`reason`/`error`、`recording_status`、`frames.index_pairs`/`index_unpaired_packets`/`index_device_ts_mismatch`/`paired_device_ts_first|last`/`beginning_sample|middle_sample`（抽样解码结果）、`imu.packets|rows|rate_hz`、`checks[]`（逐项中文结论）、`warnings[]`、`errors[]`。同时写入 `manifest["replay_check"]`；校验异常只记 `error`，不改变 `recording_status`。
- `manifest recording_status`：VS030 正常结束为 `transport_unverified`（传输完整性需看 SDK 统计），异常为 `incomplete`。

离线回放输出在 `<recording>/replay_vio/replay_<unixns>/`：`run.log`、`poses.jsonl`（每帧位姿/跟踪状态/门控/时间证据 + `replay` 元信息）、`replay.json`（status、timing、frames、map_epochs、reset_events、imu、elapsed_s、error、output）。格式与限制详见 [RECORDING_REPLAY.md](RECORDING_REPLAY.md)。

## 导出

JSON 为完整结果数组，使用标准 JSON null 表示非有限值，禁止 NaN 字面值。
CSV 每行一个关节，包括 x_m/y_m/z_m、valid/status、极线/重投影误差、derived_quality_score 及帧/源/标定/坐标系。
NPZ 不使用 pickle：xyz_m [N,H,21,3]、valid/status/epipolar_error_px/reprojection_error_px/derived_quality_score [N,H,21]、track_id [N,H]。
H 至少为 2，左右均有未匹配检测时可能更大；槽位是帧内列表位置，不是跨帧身份。跨帧须按 session_id+track_id 对齐。
另存 frame_id/timestamp_ns/source_type/calibration_id/session_id 和 schema_version/coordinate_frame/unit。无效 xyz 为 NaN，必须读取 valid。
GUI 导出提供左右显示叠加截图及 GL 骨架截图。图像几何状态见相应帧诊断。

## 外部图像清单

```json
{
  "metadata": {
    "source_type": "recorded_real",
    "real_stereo": true,
    "synchronized": true,
    "real_calibration": true,
    "metric_capable": true,
    "synthetic": false,
    "synchronization_evidence": "USER_ASSERTED: hardware sequence mapping supplied by operator",
    "timestamp_kind": "sensor"
  },
  "calibration": "camera.yaml",
  "frames": [
    {"frame_id": 0, "timestamp_ns": 1000000000, "left_timestamp_ns": 1000000000, "right_timestamp_ns": 1000000000, "left": "left/000.png", "right": "right/000.png"}
  ]
}
```

这些字段必须由真实数据依据填写，示例不是同步证明。声明来源永久保留，USER_ASSERTED 不等于硬件验收。
无同步依据的图像序列设置 synchronized=false，仍能运行二维检测。
视频模式用 left_video/right_video 替换 frames，可提供 timestamps_csv，其列含 timestamp_ns/left_timestamp_ns/right_timestamp_ns；每行对应一对指定视频帧。
此输入不执行任意异步视频自动配对，需先提供已配对清单。可选 right_timestamp_offset_ns 明确记录右时钟映射；原始左右时间不会丢失。
没有 CSV 时帧时间只从 FPS 估计并强制禁用同步声明。单帧图像序列必须显式提供时间戳。
