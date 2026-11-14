# 刷新性能

本轮未有同一提交前的可复现 GUI 基线，因此不伪造 before 数字。改动后的显示路径只保留有限对象：每次刷新复用 `TrajectoryView` 的已有列表，轨迹点上限 10000，预测和候选各自独立，不创建无限 OpenGL 对象。

可复现实验：

```text
capture_demo.py                         # 生成客户 GUI 模拟截图
tests/test_vio_panel.py                 # 含 Qt 轴/轨迹刷新回归
```

性能风险主要仍来自 Qt/OpenGL 和图像处理；最佳位姿选择本身只做小型 4×4 矩阵校验和有限轨迹更新。
