# 显示契约

`BestEffortPoseSelector` 位于 `src/egohand/vio/display_pose.py`，客户 GUI 与研发 GUI 都通过 `VioPanel` 使用它。

实时和回放链路增加两个仅显示字段：

- `display_pose_contract.T_body_from_camera`：由 `T_imu_from_orb_left @ T_orb_left_from_gui_left` 得到，明确机体到 GUI 左相机的外参。
- `display_visual_candidate`：带时间戳、帧号、参考身份和 4×4 矩阵的轻量原生候选，不改变正式位姿字段。

预测使用 `T_world_from_body @ T_body_from_camera`。若存在同一局部段的 `T_segment_from_map`，再转换到该段；地图/段身份变化时丢弃旧状态，不能跨图拼接。

`VioPanel.TrajectoryView` 保留可信轨迹接口，并新增独立的橙色 IMU 预测轨迹和黄色视觉候选轨迹。预测和候选不会进入 `poses`。冻结状态保留轴，不追加轨迹点。
