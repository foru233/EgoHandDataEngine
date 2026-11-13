# VIO 对比调试窗口（纯视觉 / 视觉+IMU × 门控前 / 门控后）

> 目的：一眼看出"大跳变到底是不是 IMU 引起的"。
> 脚本：`scripts/vio_debug_window.py`　测试：`tests/test_vio_debug_window.py`

## 1. 四张图是什么

| | 左列：门控前（原生 ORB 输出） | 右列：门控后（GUI 实际展示） |
| --- | --- | --- |
| **上行** | ① 纯视觉：`raw_matrix`，无 IMU 约束 | ② 纯视觉：`matrix`（过 `PoseQualityGate`） |
| **下行** | ③ 视觉+IMU：`raw_matrix` | ④ 视觉+IMU：`matrix` |

- 门控前 = 原生桥返回后、未经质量门控的估计；门控后 = GUI 里真正画出来的位姿。
- 两张门控前后并排，可以直接看出"门控丢掉了哪些帧/哪些段"。
- 上排 vs 下排，看是不是"只有加了 IMU 才跳"。

图上标记：

- **红色点**：同一段内相邻两点位移 > 0.5 m（异常步长候选）。
- **黄色点**：地图版本（`map_epoch`）切换处——不同地图坐标系不能连线，所以线在这里断开。
- 线在无效帧处也会断开（不插值、不补点）。

每个图上方有统计：帧数 / 原生有效 / 门控后 / 时长 / 同地图最大步长与 >0.5 m 次数 / 跨地图最大跳变与次数 / 轨迹跨度。

## 2. 启动

```powershell
# 用 results/vio 下最新的一组纯视觉 + 一组视觉+IMU
.\.venv\Scripts\python.exe scripts\vio_debug_window.py --auto

# 指定两组的输出目录（各自含 poses.jsonl）
.\.venv\Scripts\python.exe scripts\vio_debug_window.py `
    --stereo-only     results\vio\stability_baseline\snapshot_matrix\stereo_only `
    --stereo-inertial results\vio\stability_baseline\snapshot_matrix\snapshot_default

# 直接在窗口里"重新跑两组"（后台串行跑，跑完自动载入）
.\.venv\Scripts\python.exe scripts\vio_debug_window.py --auto `
    --recording data/imu/session_20261111_105436_609132400 `
    --library   src/egohand/vio/native/bin/egohand_vio_stability_gauge_constraint.dll
```

窗口控件：

- **统一坐标尺度（四图同比例）**：四张图用同一个相机距离，便于直接比较跨度；关掉则各自自适应。
- **同步视角**：在任意一张图上旋转/平移/缩放，其余三张跟随（四图同角度才方便对比）。
- **重新载入** / **保存截图**（存到 `results/vio/debug_window/`）。

### 纯视觉必须用支持 `stereo_diagnostic` 的桥接库

`--stereo-only` 通过 `EGOHAND_VIO_STEREO_ONLY=1` 让原生桥跑纯双目。**v9 恢复库不支持该开关**（用 v9 跑 `--stereo-only` 会得到 `raw_valid=0`）。请用：

- `egohand_vio_stability_snapshot.dll`
- `egohand_vio_stability_gauge_constraint.dll`

两组必须用**同一个库**、同一份录制、同一套标定/时序，否则不可比。

## 3. 怎么判读（这是本窗口的真正用途）

1. **① ② 连续、③ ④ 才跳** → 问题在惯性链路（初始化 / 预积分 / BA2 / 外参 / 时序），而不是视觉前端或标定。
2. **①② 也跳** → 视觉前端或标定/几何有问题，先别查 IMU。
3. **③④ 的黄色点（epoch 切换）附近出现大段位移** → 对应 `FullInertialBA` 求解释放全局规范自由度（详见 `docs/VIO_ROOT_CAUSE_REPORT.md`）。
4. **② vs ④ 的有效帧数差很多** → 门控把惯性分支的大量帧拒了，说明惯性分支的位姿质量更差（或门控阈值对惯性分支不合适）。
5. **① 与 ③ 同帧位置差很多** → 惯性输入改变了整条轨迹，属于结构性问题，不是单帧抖动。

已知的定量参照（同录制 `session_20261111_105436_609132400`、1727 帧、snapshot DLL）：

| 组 | 原生有效 | 门控后 | 同段最大步长 |
| --- | ---: | ---: | ---: |
| 纯视觉 | 1727 / 1727 | — | ≈0.041 m |
| 视觉+IMU | 1519 | 456 | 24.6 m（跨地图） |

## 4. 复用了哪些现有实现（没有另写一套）

| 用途 | 复用的代码 |
| --- | --- |
| 三维视图、配色、断线绘制、自动取景 | `src/egohand/gui/vio_panel.py` 的 `TrajectoryView`（GUI 主界面同一控件） |
| 统计口径（0.5 m / 0.2 s、同段与跨段分类） | `scripts/audit_vio_stability.py` 的 `audit_rows` / `raw_matrix` / `map_identity`（与验收报告同一套） |
| 数据生成 | `scripts/run_vio_stability_experiment.py`（`--stereo-only` 切纯视觉） |

窗口只是把这三处拼成"四联图 + 重跑按钮"，不重新实现几何、门控、回放或统计。

## 5. 限制

- 没有独立真值，四张图只能做**相对比较**，不能作为精度验收。
- 单次回放的运行间方差很大（同一配置重复跑，同段 >0.5 m 次数可从 5 变到 49），单张图不足以定论；要判优劣必须重复多次看分布。
- 跨地图（`map_epoch` 变化）的位移不是同一坐标系下的运动，不能被当作跟踪误差。
- GL 需要可用的 OpenGL 上下文：远程桌面/无显卡驱动环境请加 `--no-gl`（退化为二维顶视图 X-Y，Z 用明暗表示）。
