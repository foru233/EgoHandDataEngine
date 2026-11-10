# 可复现双目评测协议

所有操作保持 M01–M05 范围。数据读取、官方手部模型转换、相机几何、几何Oracle、真实图像模型推理分开验收。硬件/厂家SDK未在本轮验收。

## 输入与选择

CLI `scripts/run_stereo_benchmark.py --help` 支持 dataset/stb|hot3d、dataset-root、max-frames、mode/oracle-2d|mediapipe-stereo、config、output、seed、manifest、clips、toolkit-root、detector-input/native|virtual、native-detection。

使用固定序列及数值帧顺序，不打乱；seed=0。manifest记录dataset/sequence/frame、图像或tar摘要，重跑 `--manifest 上次/sample_manifest.json`，按原始数据时序选择，已有摘要不符明确阻断。processing_run_id和时间等审计标识每次不同，不要求文件逐字节相同。

每次输出保存配置、环境、算法诊断、calibration_fingerprint、官方来源/数据提交、相机变换、关节映射、原始时间戳依据。输入只读。没有STB传感器时间戳时，用单独标注的处理顺序值满足API单调性，并禁用跨帧三维跟踪，不伪造硬件时间。HOT3D采用逐路实际传感器记录值。

## 两条通道

Oracle-2D Geometry Benchmark：可信官方模型GT经已验证相机几何投影成二维，再把二维点交给被测关联/三角化/门控。三角化接口不接收GT XYZ。Oracle不可用于报告真实图片检测精度，表内检测率实际表示“提供了Oracle观测”。

Real images → MediaPipe → stereo：执行官方模型CPU IMAGE推理，灰度图明确三通道复制。HOT3D可分别测试官方完整FISHEYE624重采样图、原生正向灰度图检测再做射线适配。两者是分开参数化的输入管线，不能合并挑选每帧较好结果。没有GT裁剪、插补或用GT修正关联。

STB几何默认受CALIBRATION_MAPPING_UNVERIFIED门禁；不得猜外参。HOT3D按官方模型与当前帧标定，GT通过官方UmeTrack FK生成，主引擎保持原接口。

## 误差、覆盖与身份

绝对MPJPE/median/P90/P95/max在**有效且可比较**的关节上计算，单位mm；同时报告全体已选帧、全体语义可比GT（含视域外/漏检/被拒绝）分母覆盖率、无重建比例、检测失败率。全部GT分母通常比双目重叠区大，另保存both_views_valid，不能只挑可见点掩盖失败。

HOT3D使用canonical20，STB使用保守17点。不做root alignment/scale alignment。2D评测身份仅由左/右各自预测像素与投影GT的独立Hungarian比较建立：至少4个共同像素点、median距离<80px，替代解需相差≥10px。这个匹配仅用于评测，不影响被测关联、跟踪或重建。不能根据3D误差选GT身份。

若预测左右对应不同GT手，仍按左图识别出的GT手计算输出3D误差，错误立体对应不会被删掉。无法唯一定位到GT的预测单独计入unknown_gt_pairs/未匹配；有GT却无有效预测的关节仍进入覆盖率分母。身份/像素分配的80px和10px本身也会影响可评测子集，未校准，须与覆盖一起审阅。

Stereo Match Accuracy只在两视图身份均可独立识别的已接受对上计算，其他对另外计数。ID switch只对相邻帧同GT手连续可识别输出统计；间断和标定改变重置不当成证明连续身份。没有可评测对时输出N/A而非0次错误。原生像素2D GT不可用，2D Landmark Error=N/A；投影模型代理不得冒充人工像素误差。

遮挡分层使用HOT3D modeled per-hand visibility，缺失视图为0；它不等同逐关节测得可见性，亦不覆盖所有未建模物体遮挡。深度按原始左光学Z分层，视差按虚拟校正像素分层。

## 失败与性能

failure_cases保存原始tar路径/hash、二维输出、匹配过程/候选代价、GT模型三维/原生姿态、拒绝原因、标定及输出。HTML/PNG可视化绿色GT、橙色检测，重现失败和长尾。输入图为虚拟重采样视图时明确1201-1/1201-2来源，未加入RGB配对。

processing_fps是完整样本迭代吞吐：含读tar、官方重采样、推理、评测与失败样本捕获；不含下载/模型初始化/最终报告序列化。native_detection辅助重复推理若启用也包含在实际耗时。另列engine_plus_evaluation_fps，不能混为设备实时帧率。输出含失败捕获I/O，机器负载会影响值。

固定噪声协方差仅是条件风险诊断，默认不拒绝；Coverage–Error曲线为事后标准差筛选，报告零覆盖区域N/A。

## 证据等级

SOFTWARE_VERIFIED：指定实现和回归测试已运行。
BENCHMARK_VALIDATED：某数据/参数/关节子集上有有效量化结果；Oracle只授予几何通道，不授予检测精度。
BLOCKED_BY_DATA：缺数据、GT、许可资产或可靠相机映射。
BLOCKED_BY_HARDWARE：无本机相机、SDK、硬件时间戳/标定验收。
NOT_IMPLEMENTED：未实现的模型/算法（例如MANO资产路径、像素概率标定、完整时间同步、M06–M12）。

保持旧测试断言，不用跳过真实资产的测试计为通过。不以测试数量增长推断精度提高。
