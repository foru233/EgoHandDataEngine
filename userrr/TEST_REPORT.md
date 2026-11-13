# 客户 GUI 验证报告（2026-11-12）

## 最终回归

执行：`.venv/Scripts/python.exe -m pytest tests userrr/test_client_gui.py -q --tb=short`

结果：**512 passed, 1 skipped, 2 warnings，86.03 秒**。完整日志：`final-regression.log`。警告来自既有 protobuf Python 3.14 弃用提示，不影响当前 Python 3.12。

首轮全量曾因旧测试 SlowVio 模拟对象不接受新增 mode 参数失败2项；已更新模拟对象并断言默认模式，针对12项通过后重新运行全量，得到上述最终结果。

## 启动和界面

- 原入口 `run_gui.py --demo --quit-after 3` 启动并正常退出，日志 `debug-startup.log`。
- 新入口 `userrr/run_client_gui.py --demo --quit-after 4` 启动并正常退出。
- 客户生命周期测试覆盖三列面板、左右图像、有效骨架显示、运行时模式锁定、停止后模式切换、重启和缺失录制友好提示。
- 真正 Windows Qt/OpenGL 1600×950 与1000×700截图：`screenshots/client-preview.png`、`client-compact.png`。`preview-evidence.json` 记录 `has_result=true`、`gl_valid=true`。截图输入是明确标注的合成几何，不是设备实拍。

## 真实录制

输入：`recordings/_gui_validate3/session_20261110_100232_934797400`。

通过 `verify_recording.py` 运行客户 GUI：纯视觉与双目＋IMU各处理至少8帧，左右图像存在、同帧定位消息存在、worker无错误、停止成功。证据：`recording-stereo.json`、`recording-stereo_inertial.json`、对应真实日志与截图。

这里的“定位消息存在”不等于通过质量门控。短片段尤其惯性模式尚未完成初始化；本验证不声称精度、长期稳定性或完整有效轨迹已经验收。

最后一次纯视觉停止时后台已处理9帧，`sessions/replay_1794447486771785300/results/frames.jsonl` 经 `load_recorded_results` 读取为9帧、9份对应位姿消息、无尾部截断。`reprocessing.json` 正确记录 cancelled（主动停止）、原始录制引用和模式。没有把主动停止写成完整回放。

算法层另有真实 DLL 两模式各4帧完成报告，位于 `validation/native_modes_valid_1794447262696504900`；纯视觉4/4跟踪OK，惯性短段未初始化，两者都未满足显示稳定门控时长。

SDK原始源首3帧解码为1600×1200并运行 MediaPipe；同步依据保持 USER_CONFIRMED_SAME_BUNDLE，硬件曝光精度未宣称已验证。该段首3帧没有有效手部匹配；受控几何测试单独验证同一适配链能输出米制关节。

## 明确保留的限制

- `recordings/map_scan_*` 五份旧录制快照中缩放与 ORB 内参不一致，严格拒绝。没有修改源录制或用现场配置静默替代。
- 已有结果回看显示手部和已保存位姿轨迹，目前不自动解码原图；图像与深度清空，完整图像可选择重新计算。
- 未连接硬件验证实时设备连接与长时间录制；实际硬件长期稳定性、精度、快转和遮挡优势待实机验收。
