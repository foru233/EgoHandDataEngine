# 条件三维不确定性

实现 `stereo/uncertainty.py`，输出原始左光学坐标下米²协方差。默认诊断模式；不更改原 XYZ、quality_score 公式及几何门控。quality_score继续表示极线/重投影一致性，不能解释为真实三维精度或成功概率。

## 方法

观测向量 q=(uL,vL,uR,vR)，三角化函数 X(q) 含 cv2.triangulatePoints 和从校正坐标回原左相机的旋转。固定标定和对应关系，中心差分得到3×4 Jacobian：`J[:,i]=(X(q+h e_i)-X(q-h e_i))/(2h)`。

`ΣX ≈ J Σuv Jᵀ`。默认 `Σuv=1.5² I4 px²`，假定四维独立同分布；接口也接受显式4×4半正定协方差。1.5px是未校准的测试假设，没有来自MediaPipe每关节的误差概率。0.5/1/2/4px敏感性结果保存在sync实验输出中。

默认步长h=.01px，同时计算h/2，比较相对Jacobian变化，检查有限齐次分母、正深度、奇异值秩/条件数、对称半正定协方差。不可用返回None和INVALID_INPUT/DEGENERATE_GEOMETRY/UNSTABLE_JACOBIAN/INVALID_COVARIANCE等状态，绝不填0。已经被几何拒绝的候选不浪费时间传播协方差，标记GEOMETRY_REJECTED；禁用时DISABLED。

`depth_std_m = sqrt(ΣX[2,2])`，表示输出光学Z分量标准差。`position_std_m = sqrt(λmax(ΣX))`，表示最大主轴的一倍标准差，不是三维平均误差或置信半径。HOT3D虚拟输出转换回原生左相机时同时旋转协方差，位置主轴标准差不变，深度标准差重新计算。

标准平行双目的对照简式 `σZ ≈ f B σd / d²`，独立左右同噪声时σd=√2σpixel。仅在简式假设下用来核验一般方法，不能用于所有旋转/垂直/鱼眼几何。已自动验证水平、垂直、旋转、OpenCV fisheye校正后的像素模型。FISHEYE624走官方像素/射线适配，不套四系数公式。

## 假设边界

- 固定标定、左右时刻、对应手/关节、选择的模型语义。
- 固定**校正后像素**噪声。原生鱼眼像素转换后噪声并非各向同性；本版没有估计MediaPipe原生噪声再经过FISHEYE624变换传播的完整链。native benchmark也明确沿用虚拟像素假设，不能把它说成实际传感器不确定性。
- 不含内外参误差、系统性关键点偏移、错误匹配、时间不同步、模型形状/语义差异、遮挡的多峰误差。
- 不校准概率；多个证据不平均为成功概率。

右图整体水平偏移的回归保留 `systematic_bias` 分类：真实误差可超过5cm，而重投影误差和旧quality_score近乎完美；固定随机像素噪声的协方差也可能低估该偏差。60组物理时间偏移实验进一步复现：1m/s水平运动、5ms偏移，平均误差55.967mm、重投影4.87e-14px、深度标准差27.479mm，全部点仍能通过5ms同步容差。软件容差不能充当精度承诺。

## 可选拒绝

`configs/stereo.yaml`默认 `enabled:true, mode:diagnostic`。仅显式 `mode:reject` 才按max_depth_std_m/max_position_std_m（目前各.03m测试值）或reject_unavailable拒绝，状态HIGH_UNCERTAINTY、坐标NaN、valid=false。

阈值只是实验起点，尚无真实端到端误差分布支持，不默认开启。coverage_error.csv/png采用阈值2/5/10/20/30/50/100mm及不限的**事后筛选**，同时列出全部GT分母覆盖率，不能冒充重新运行整个关联流程后的改善。真实MediaPipe无三维输出时曲线明确N/A。默认诊断增加计算成本，实际性能对照见报告。
