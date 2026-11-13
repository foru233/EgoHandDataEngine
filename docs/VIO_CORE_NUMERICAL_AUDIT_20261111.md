# ORB 核心数值审查与诊断开关

## 已确认的源码行为

- `LocalMapping::InitializeIMU` 在 VIBA2 调用 `FullInertialBA`，使用 `bFixLocal=false`。
- `Optimizer::FullInertialBA` 的原实现只在 `bFixLocal` 分支固定部分位姿。全局路径没有世界位置、绝对航向先验，存在三维平移和绕重力方向旋转的规范自由度。该路径原先与仓库内上游版本一致，不能归咎于本地历史补丁。
- 双目惯性优化中的尺度顶点由 `VS->setFixed(!bMono)` 固定。双目已有基线提供的米制尺度，BA2 是惯性优化阶段而不是米制开关。
- 普通局部 BA 的 `IncreaseChangeIndex` 与大修改的 `InformNewBigChange` 不同；现有 `map_epoch` 未包含普通地图修订。同 epoch 跳变不能直接证明视觉前端自身跳错，必须同时检查局部优化提交。
- `TrackStereo` 返回矩阵后，原生桥分次读取 tracking、epoch、BA2，Python 随后还单独请求 body state。每个读取有局部锁不代表整条结果同版本。此处存在跨版本拼接机会；具体发生次数需一致性快照取证。

## 诊断 A 与实验 B

新增 `VioOptimizationDiagnostics.h`，默认不开诊断，不做文件写入。

- 设 `EGOHAND_VIO_DIAGNOSTICS` 为已经创建的目录，记录地图初始化开始、惯性优化后、全局优化前、地图提交前后快照。
- Full BA 另记录求解器顶点在 `fullba_before`、`fullba_solved` 时的同 ID 位置、旋转、速度、偏置，以及地图点坐标。
- 输出观测关联、边类型/维数/chi-square/顶点 ID、普通及 robust chi-square、迭代返回值及非有限边数量。未暴露的条件数和线性求解器内部状态明确输出 `unavailable`。
- 设 `EGOHAND_VIO_GAUGE_FIX=1` 开启实验 B。默认关闭，尚不代表已通过实际回归。

B 在全局 Full BA 求解后、发布前，为全部优化关键帧、地图点、速度选择同一套三维平移与重力轴 yaw 规范，使锚关键帧原点与水平航向保持原值。不固定首帧六自由度，不阻碍 roll/pitch 优化，不改变尺度或偏置，不做单帧显示平滑。视觉与惯性残差对该统一变换不变；输出 `fullba_normalized` 及其重新计算的残差，供实际检查。

B 只解决不可观测全局坐标的选择，不能解决优化数值退化、错误视觉匹配、标定/时间误差或同版本不一致。其内部图仍有规范自由度，因此不能单凭它宣布优化器条件改善。

比较命令：

```powershell
.\.venv\Scripts\python.exe scripts\compare_vio_optimization_snapshots.py <诊断目录>
```

同 ID 拟合同时给出 SE(3)、Sim(3)、非刚体剩余误差、确定性抽样的两两距离变化。低残差说明可由整体变换解释，不说明定位绝对准确。关键帧分布共线或很小时，旋转拟合会退化，报告保留秩和奇异值。

自动几何测试 `tests/test_vio_optimization_snapshots.py` 覆盖整体 280 米平移+yaw、尺度变化、单点畸变、缺失/非有限 ID，已运行 4 项通过。原生 A/B 和实机验收以主报告为准。

## 另发现的可复现数值风险

`LocalMapping::InitializeIMU` 原重力对齐手写 `cross`、`acos`、除以旋转轴长度。重力同向时为 `0*0/0`，反向时轴也未定义；零重力累计向量归一化同样产生 NaN。独立 `VioGravityAlignment.h` 使用 `Quaternion::FromTwoVectors` 处理同向/反向，零或非有限方向返回失败。该发现不能证明历史 Sophus NaN 只有这一来源。

`EdgeInertial`、`EdgeInertialGS` 及偏置随机游走边直接对预积分协方差取逆，缺少奇异性/有限性检查。零协方差可传播 NaN，需从异常数据中取得实际协方差与时长再决定改法，不应随意加噪声或抬高特征值掩盖问题。已撤销的 `ResetFrameIMU` 清空补丁未重新引入。

## 实验 C：在求解图中消除四维规范自由度

B 只是解后统一坐标，不改善求解图的秩亏。因此新增独立实验 C，由 `EGOHAND_VIO_GAUGE_CONSTRAINT=1` 启用，默认关闭，只用于 `FullInertialBA` 的 `bFixLocal=false` 路径。B、C 开关彼此独立，受控比较时只开其中一个。

新增 `EdgeWorldGauge` 只连接一个惯性关键帧的 `VertexPose`，四维残差为 IMU 世界位置与初值的差（3 维），以及一个稳定水平 body 轴的世界航向与初值的角差（1 维）。不固定首帧六自由度，也不固定 roll/pitch、尺度、速度或偏置。选择初始最水平的 body 轴，避免原姿态的 yaw 图表奇异。

按 ORB `ImuCamPose::Update` 的局部更新约定，位姿扰动为 `R'=R Exp(dtheta)`、`p'=p+R dt`。位置 Jacobian 为 `[0,R]`；航向 Jacobian 为 `grad_atan2 * (-R skew(e_axis))`，对局部平移的导数为零。这四行 rank 为 4，在全局三维平移加重力轴 yaw 四个群方向上的作用同样 rank 4。对于任何可观测形状，都可沿此四维群把残差置零，所以它选取规范而不额外限制可观测几何。

信息矩阵为单位阵，不借此改变视觉/惯性权重。不把 soft gauge prior 当作独立外部测量。完整图仍可能因弱视差、偏置/速度冲突、错误输入或其他退化而病态，C 是否改善真实回放必须实际验证。

共用数学头 `VioGaugeMath.h` 与独立 MSVC 测试 `scripts/test_vio_gauge_constraint.cpp` 已验证：解析 Jacobian 对中央差分误差小于 1e-8，两个 rank 检查均为 4，沿群置零和视觉/惯性残差不变量误差小于 1e-12。航向图表在所选轴竖直时仍可退化；代码检测此情形并写出失败线索，不能把未遇到奇异当作全局保证。

新原生库使用独立名称 `egohand_vio_stability_gauge_constraint.dll`，不会自动替换现有 GUI 或 Atlas DLL；编译日志为 `logs/vio_stability_gauge_constraint_build.log`。构建和回放结果以验收报告为准。
