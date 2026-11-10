# 坐标与标定契约

输出 `coordinate_frame=camera_left_optical`，单位 `m`；X 右、Y 下、Z 前。
外参严格采用列向量关系 `p_right = R_right_left @ p_left + t_right_left`，平移单位必须显式为米。
右相机在左相机右侧时，标准平行双目示例 t 的 X 为负；不能把 t 直接理解为右相机中心位置。
右相机中心在左坐标系为 `-R.T @ t`。

OpenCV `stereoRectify` 得到 R1/R2/P1/P2/Q 和 ROI；鱼眼走 `cv2.fisheye` 分支。
鱼眼 API 不提供的有效 ROI 显式为 None。P1/P2 仅用于对应校正图像/像素。
三角化返回 `camera_left_optical_rectified` 中的点。用列向量 `p_left=R1.T @ p_rectified` 转回原始左相机。
代码数组按行存储，等价写为 `xyz_left=xyz_rectified @ R1`。
GL 显示单独变换 `(x,y,z)→(x,-y,-z)`；导出始终为原始 optical 坐标。

标准水平双目极线误差 `abs(v_left-v_right)`；若 pinhole 标定经校正为垂直双目，自动沿对应轴计算。
图像标为 rectified 时必须携带匹配的 rectification_id，幂等返回，不能再次 remap。
本契约只接受由该标定和本程序相同校正参数生成的已校正图像。其他 SDK 校正投影应先适配/验证，不能只填一个标签。

无效点内部 NaN+valid mask；JSON 导出坐标 null。quality_score 为自定义几何量：
`exp(-0.5*((epipolar_error/max_epipolar)^2+(reprojection_error/max_reprojection)^2))`。
它不是校准概率，不是 MediaPipe 原生单关节置信度。MediaPipe world_landmarks 另存，不表示左相机坐标。

MediaPipe handedness 保留模型原始标签，不自动等同于已验证的解剖学左右手。镜像输入/不同视角可影响标签，匹配只作软约束。
GUI 默认不镜像。厂商适配时必须核对图像方向与左右视图顺序。
