> 最新检查点（2026-11-15，继承后续明确授权与完整实验）：v24夹具修复及原生单位已通过；184714/111807/111833六臂全分辨率VIO+原手部融合、EuRoC两臂完整GT、真实客户GUI、全项目996 passed/1 skipped均已完成。v24因111833跳变拒绝1→10及EuRoC初始化延迟90→134帧，不晋升默认。手部相机系数值两臂一致、世界坐标发布审计错误0；P0/P2仍NOT ACCEPTED。详细最新证据见 `continuation_native_geometry_20261114/root_cause_convergence_20261115/{FULL_FUSION_FINDINGS,EUROC_FINDINGS,GUI_AND_FINAL_CHECKS}.md`。失跟转移帧预测断链补丁已在候选外准备，未应用/未构建；新的运行DLL实验等待用户明确授权。v11外部干预证据保留，正式/stable_offline_v2/v23/v24不变。以下历史检查点完整保留，不再将旧“v24 unit尚未通过”视为当前状态。

## 最新检查点：v11 外部干预已登记；v23完成；v24单位未通过

整体 IN PROGRESS / NOT ACCEPTED，goal active，正式默认和 stable_offline_v2 保留。

- 用户报告 candidate_v11_application_ack_shadow 曾被未经授权修改 CMake、创建静态构建脚本/日志并重生成 Tracking/System对象与ORB静态库。已标记 external_intervention，不再视为原始冻结候选。六项文件哈希/时间、275项当前清单及文本副本已保存于 continuation_native_geometry_20261114/external_intervention_v11_20261115/；保留原始证据，无回滚/删除/重编译/候选覆盖，无commit/push。
- 本次审计正式保护160项、v23既有清单294项全部匹配；审计前后v23/v24共595文件变化0。不能据当前哈希证明干预前原始身份，也不能证明历史上不存在目录外写入。退出码0来源明确为用户报告。
- v23两次完整111833各1433帧exit0。同DLL单变量prior0/1，cached与实际公式一致，after_cache_error均0；本次没有stale cache。prior1最新KF边占初始cost>99%有49次，prior0为0。jump9/9，视觉cost上涨4/30；不能据准入1270/1287判改善。v21继续不晋升，v23_post_two_recording_integrity_v1.json原975项errors[]。
- v24在本事件指令前已修复默认Frame复制夹具并启动构建session57573，现已结束exit0。之后仅执行既有单位程序、证据输出至候选外：v24_native_unit_exit_v3.json在0.1秒连续支持断言失败(returncode3221226505)，尚未通过单位、封存或真实A/B。保留原v1空日志和v2访问异常记录，不将构建成功冒充测试通过。
- 依最新指令不再自行修改/重新编译/覆盖任何候选。v24 prefix A/B driver已准备但未运行；必须先解决原生单位失败，不能跳过断言直接验收。v11事件审计仅只读现有候选，后续开发不得依赖受干预v11身份。

详情：continuation_native_geometry_20261114/external_intervention_v11_20261115/事件记录.md。以下保留历史检查点，旧“v23运行中”已被本检查点取代。

## 最新检查点：111833分项代价已闭环，v23对照诊断进行中

整体 IN PROGRESS / NOT ACCEPTED，正式默认、stable_offline_v2继续保留。详细见 continuation_native_geometry_20261114/局部惯性BA分项代价诊断_20261115.md。

- v22观察器Sophus<double>转换导致1085/1433帧后原生终止，故障证据保留，不作验收。独立v22b只修只读矩阵观察，float旋转回归/六图native重放通过并封存。
- v22b完整111833 1433帧exit0，88对solver→88commit核验errors[]，五类缓存代价加和、状态有限性、提交矩阵一致；总代价88次均下降、视觉代价66次上升，最大KF中心更新0.750749m。15次jump全部追溯到提交BA，且其视觉代价均上升。不是放宽原生代价上涨例外导致；没有外部GT，不把大幅更新判为真运动或假跳变。
- 准入1287/jump15/隔离91，仅独立异步poses-only诊断，不与历史融合计数作因果改善比较。v22b_111833_local_ba_cost_audit_v1.json / v22b_111833_cost_jump_links_v1.json / 图v22b_111833_local_ba_costs_v1.png。
- v23隔离观察器增加逐惯性边cached/实际状态公式error、information、C、dt/KF时间跨度，不改edge缓存/顶点/bias/solver。实际原生回归验证与computeError一致且无修改；两单位+六图重放全部通过，29对象/227源封存。
- 当前session6473按序运行同DLL/同配置111833 control(prior=0)→candidate(prior=1)两次完整回放。冻结v23/DLL/driver/runtime，不重启；当前文件v23_111833_inertial_edge_{control,diagnostic}_v1/。旧v22b session78150和build80241已结束。
- 正常Windows Qt全项目996 passed、1 skipped、2 warnings(89.71s)；先前offscreen唯一OpenGL失败保留，未改断言。正式160/v22原283/v22b293保护全部匹配。未知PID12348未触碰；无commit/push。

下一步：收齐v23两次全段，独立核对惯性区间、实际残差、信息矩阵与误差缓存，区分共享原生问题与已拒绝v21约束实验影响。不能仅据缓存总代价下降或准入数交付。恢复首次提速/精度非劣和最终验收仍未通过。

## 最新检查点：v22观察器故障已隔离修复，v22b真实回放进行中

整体 IN PROGRESS / NOT ACCEPTED，目标保持 active。正式默认与 stable_offline_v2 保留，无 commit/push。

- 原 v22/session43679 已失败结束：1085/1433帧，WinError10053；实际原因是新增观察器将 float 来源旋转送入 Sophus<double> 严格构造检查，原生 ensure 终止。0条 local_inertial_solver_cost，journal尾行截断；不得用作完整回放或恢复验收。原源/DLL/故障日志保留：v22_failed_diagnostic_assessment_v1.json。
- 独立 candidate_v22b_local_ba_cost_diagnostics 只修观察器，直接记录 Rcw转置和相机中心，不投影旋转、不改 solver/gate/提交条件。新增实际 float 旋转原生回归证明可输出且顶点不变。两原生单位、六图独立重放通过，位姿/速度/bias/外点差异0。29对象、227源封存，native_validation_v2.json / source_provenance_v1.json。封存脚本v1仅Windows路径比较失败，单位已通过；保留原log，v2路径标准化后封存成功。
- v22b 111833全段poses-only诊断回放正在 session78150；冻结driver/runtime/DLL，不重启运行任务。日志 v22b_111833_local_ba_cost_diagnostic_v1.log。前述旧session30811/43679/94984已结束，不再等待。
- 新全项目测试：offscreen仅因不支持OpenGL造成1 failed/995 passed；正常Windows Qt后端996 passed、1 skipped、2 warnings(89.71s)。pytest_v22_windows_qt_v2.log；未修改GUI测试或产品来绕过失败。
- 正式160保护项、v22原283项和v22b293项全部匹配：v22b_pre_recording_integrity_v1.json errors[]。未知PID12348未触碰。

下一步：收齐完整111833；用 scripts/audit_local_inertial_ba_costs.py 核对五类缓存代价、KF变化与实际commit。缓存可能是LM回滚前被拒绝试探残差，不能冒充最终状态的精确重算代价。根据证据决定下一处最小原生修复；当前没有恢复改善或安全非劣结论，v21不晋升结论维持。

## 最新检查点：v21完整融合/GUI核验结束，候选不晋升；继续BA诊断

整体 IN PROGRESS / NOT ACCEPTED，总目标保持active。正式默认、stable_offline_v2和原始输入保留。详见 continuation_native_geometry_20261114/v21关键帧不确定性公开回归_20261115.md。

- v21三段6次全分辨率融合、6次世界手部审计全部exit0；三对同输入/配置/DLL/host，相机系手部数值身份一致。对照/候选准入1847142946/3052、111807687/688、1118331354/1287；jump0/0、0/0、1/9。111833两臂均237相机系同帧双手，局部融合236/162帧，VIO拒绝增多造成可用性退化；不推广。vs030_v21_full_fusion_summary_v1.json errors[]。
- 184714首次140→288新segment恢复两臂均4.933334秒，几何273→准入288均0.50秒。IMU60/60成功传播/最长1.966668秒，旧图60帧全部不够原几何证据，原图未找回。不是靠解除应用隔离能修复的窗口。
- 111833候选9次jump均追溯到已提交local inertial BA→参考KF→UpdateLastFrame重基准，参考KF改动0.0507–1.0703m，本帧优化仅再改0.0031–0.0170m。v4审计已校准cam0↔ORB cam1，BA链路最大平移残差1.605e-7m；原稿跨相机字段错误保留并标无效。无外部GT不称为真实运动/正确优化，不放宽门限/伪拼接。
- EuRoC1710帧两臂及GT评分exit0，共同1543时间戳5参考交集段；初始化阶段误差增大，成熟阶段略改善。实际162针孔图全部原生重放通过、外点不一致0，捕获12次marginal。公开TUM4对完成且恢复未提速/corridor4尾部退化；均非独立最终留出集。
- 客户Qt111807/184714完整新回放完成。184714显示3283/3283，可信3112/视觉候选106/预测52/冻结8/原点5，预测最长2s；非可信状态无世界手部，29341世界关节审计[]。CUA初始化失败，无人工交互/实时/物理精度认证；独立异步GUI不与CLI作提速因果比较。
- 正式160保护项、v21源/对象/DLL/依赖/证据/host快照全部再次匹配。最新全测996 passed、1 skipped、2 warnings，新增独立工具实际执行+py_compile。v21_post_full_fusion_and_gui_integrity_v1.json。

当前：v21全部回放/评分/图审计/GUI/session4476、78579、62675、97767等已结束。隔离candidate_v22_local_ba_cost_diagnostics正补充局部惯性BA前后分项代价/KF状态观察，不改求解/门限/提交，构建v1缺少SparseOptimizer显式include已修复，v2构建session30811。保留全部失败日志；未知PID12348不触碰。下一步完成v22诊断构建/单元验证与111833真实原生回放，追查BA大幅KF更新，不能把新增诊断当恢复改善。无commit/push。

## 最新检查点：v21公开四组完整对照完成，VS030融合继续中

整体 IN PROGRESS / NOT ACCEPTED。正式默认与 stable_offline_v2 保留。详细结果：continuation_native_geometry_20261114/v21关键帧不确定性公开回归_20261115.md。

- v21原生六图单元/独立重放通过，源/DLL/29对象封存；全项目996 passed、1 skipped、2 warnings。公开4对8次回放32评分、共同时间戳对比全部exit0；候选实际723次KF marginal约束，地图/KF/prior生命周期审计无错误。未放宽门控，不搬旧prior，仍非全地图联合后验。
- room1/room2恢复时间两臂分别0.500160/0.549753s；room3两臂同为4.150568s，对照新参考/候选原参考，候选首次真值误差1.928cm/0.591deg。room3 jump11/1，但候选原生失跟更多、之后仍800jump/860新图。corridor4 jump183/209、原生最大GT误差6.397/10.297cm；不能推广或宣称安全非劣。
- room3全部148精确图通过native重放，外点不一致0；对照原始OpenCV数字解析失败及探针保留，独立副本仅等值数字词法转换、保留NaN并逐字段验证后通过，未改原图/算法。正式160保护项和candidate全部源/对象/DLL/依赖/host快照匹配。
- 三段VS030全分辨率VIO+双手同变量6次回放正在session4476：continuation_native_geometry_20261114/vs030_v21_keyframe_uncertainty_full_fusion_AB_v2.log。首次脚本复制编码失败在worker前，修复UTF8路径且标定哈希与v17一致；不重启正在运行回放、不改封存runtime/driver/DLL。未知PID12348不触碰。

下一步：完成六次融合及手部相机系逐项相等/世界坐标审计、184714逐帧恢复时间/segment证据；补全公开全时段步长GT分析，区分快速真实运动与错误优化。没有证据不提高速度门限、不自动解除隔离；GUI/独立最终验收仍待完成。总目标active。

## 最新检查点：v20 精确图诊断已完成，恢复修复尚未验收

整体 IN PROGRESS / NOT ACCEPTED；正式默认与 stable_offline_v2 保留。详见 continuation_native_geometry_20261114/v20精确图诊断_20261115.md。

- v20 为基于 v17 的隔离可选图导出，算法/门控不变；四种原生图单元与独立重放完全匹配。完整 room3 2821 帧和四项评分全部 exit0；640–720 区间 75 对实际图（46 frame IMU / 8 KF IMU / 21 visual）全部原生重放通过，外点不一致 0，最大位姿元素误差1.662e-6。首版纯视觉工具失败保留，独立 tool_v2 修复后全通过，未动封存 DLL。
- room3 本次原生657、首几何659、应用667（距解遮挡3.85056s），原参考恢复；首次误差1.584cm/0.995deg，之后jump682/786/867。导出会影响异步调度，不能对历史运行宣称提速。
- 同一682图的原生/纯视觉结果位置差3.234cm，局部相对真值平移误差2.903/1.297cm；纯视觉仅诊断，无可用VIO速度。相同预积分上的KF局部BA状态替换，位姿/速度/bias分别贡献1.005/2.863/0.100cm传播变化；不能因此断言新速度错误或复用旧prior。
- 全项目996 passed、1 skipped、2 warnings（117.53s）；正式160保护项与冻结候选/依赖303项全匹配。v20_post_replay_integrity_v1.json。v17六次VS030/手部结果、111833退化、v18/v19拒绝结论全部保留；本轮无新VS030/GUI验收。

下一步：针对局部BA后固定KF状态/不确定性与视觉图冲突，设计同图可验证的最小原生处理，再进行完整闭环与独立数据/VS030手部/客户GUI验收。不盲增补丁，不传输非刚性更新前prior，不改正式默认。所有本轮构建/回放/audit/pytest session均已结束；未知PID12348不触碰。总目标保持active。

## 最新封存结果：本轮全部回放结束，恢复候选未验收

整体 IN PROGRESS / NOT ACCEPTED。正式默认与stable_offline_v2保持，不覆盖DLL/live/标定/录制，无commit/push。完整说明：continuation_native_geometry_20261114/恢复专项续执行_20261115.md。

- v17公开4对8回放32评分、4共同时间戳对比全部完成；VS030三段六次全分辨率VIO+双手回放和六次手部审计全部exit0。三对相机手数值/身份一致，同输入/配置/根源码/同DLL核对通过。vs030_v17_full_fusion_summary_v1.json。
- VS030对照/候选准入：1847143068/3097、111807687/687、1118331344/1273。跳变：0/0、0/0、2/14。111833候选退化现象保留，不宣称稳定性非劣。全部公开与VS030回放都未观察完成的MergeInertialBA图，合并修复实际闭环路径仍未覆盖。
- 184714两臂失跟141/142，均60次成功原生IMU传播/最长1.966668秒，但旧图未恢复；最终独立新segment原生272/243、应用288/259，几何至准入均0.50秒。不把预测成功当定位成功。
- v18完整room3两回放8评分，283次实际搜索种子审计[]，但686仍被后续关键帧惯性优化拉动0.045063m并跳变；REJECTED。
- v19完整room3两回放8评分，5尝试/1提交(898)、实际双帧速度公式及事务审计[]；候选独立新图准入6.60035秒、对照旧图4.00056秒，3/2次jump；REJECTED。之前恢复差异不能归因于898才发生的成功提交。
- 新全项目测试996 passed、1 skipped、2 warnings(137.71s)。保护160全匹配，三隔离候选275/279/280项源/链接对象/DLL哈希全一致。pytest_v19_isolated_experiments_v1.log；candidate_v17_v18_v19_post_replay_integrity_v1.json。
- 所有本轮session已结束：54921/73620/67495/8137/33956/39710/41787/99229/60751；不要重复等待。未知用户PID12348未触碰。

下一步：对关键帧惯性优化导出精确可重放图快照(相机/像素/金字塔/地图点/状态/预积分C/priorH/关键帧更新)，以同一内部图隔离普通localBA重基准与速度/bias不一致；不凭异步运行差异继续盲加补丁，不放宽门控或搬运旧prior。后续安全候选仍需独立GT和真实客户GUI，不重复旧GUI冒充新版本。总目标仍active。

## 最新检查点：v19闭环未改善，稳定默认保留

整体 IN PROGRESS / NOT ACCEPTED，不能把已完成的实验当成项目验收。

- v17公开4对完整8回放/32评分/4共同时间戳对比已完成，未证明恢复提速，未记录完成合并图；合并边界真实针孔/鱼眼IMU图单元通过。v17 VS030六次完整双手融合还剩111833_control(session73620)，其余5次已完成。184714/111807两组成对相机手数值与身份保持，世界手部审计[]。全套结束前保持根runtime、融合driver、手部config与v17候选冻结。
- 本轮184714两臂失跟141/142，均60次成功原生IMU传播，失跟预测最大1.966668秒；旧图未找回。对照经历3个loss episode，最终新图原生272/应用288；候选2个episode，原生243/应用259；两者有效几何至准入均0.50秒。均无jump标记但无外部GT，不能宣称精度/正确性通过。历史179帧事件不再作为本轮事件起点。vs030_v17_184714_recovery_comparison_v1.json。
- v18实际283次搜索初值审计全部通过；686帧先采用正确的一帧IMU搜索初值，随后关键帧惯性优化仍拉动0.045063m并导致跳变。仅搜索初值方案REJECTED。完整2回放8评分与证据保留。
- v19保存惯性优化前搜索匹配，再做双帧同地图视觉重试；真实针孔/鱼眼优化单元通过，完整room3两回放8评分完成，实际5次尝试/1次提交(898帧)，事务与重估双帧速度公式审计[]。候选首次重新准入6.60035秒独立新图，对照4.00056秒旧图；候选3次jump/对照2次。未达到恢复改进标准，REJECTED，不扩散或推广。成功提交898发生在恢复事件之后，之前差异不能归因于成功重试。
- 最近保护160项再次全匹配：protected_files_v19_before_AB_v1.json。原全测996/1skip保留；新一轮全项目pytest正在运行，日志pytest_v19_isolated_experiments_v1.log。无正式DLL/live/标定/录制覆盖，无默认切换/commit/push。

下一步：完成73620与全项目pytest；生成第三手部对照及六次融合总汇，实际合并图触发统计；记录完整新旧segment与准入时间，不能只报告数量。继续诊断关键帧惯性约束与视觉/普通localBA重基准冲突，不能盲目传输旧prior或扩大门限。公开room3已是开发数据，未来安全候选仍需独立GT和实际客户GUI。

## 最新续执行检查点：v17公开回归完成，v18仅搜索初值不足

整体 IN PROGRESS / NOT ACCEPTED，正式默认与 stable_offline_v2 保留。

- v17 合并观察索引/权重隔离修复：真实针孔/鱼眼有无IMU共4次优化图测试通过；固定公开4对8回放、32评分及4共同时间戳比较完成。全部公开日志未观察到完成的 MergeInertialBA 图，不能声称真实合并路径已覆盖。room3 对照/候选重新准入3.90056/3.95064秒，后续7/5次跳变；没有证明恢复提速。merge_observation_v17_development_summary_v1.json / assessment_v1.json。
- v17 三段完整 VIO+双手六次对照 session73620 仍在运行，184714 两臂已完成回放，后续111807/111833进行中；冻结 v17 DLL、根runtime、融合driver与手部审计/config。worker不等于客户GUI。
- v18 已构建并通过原生传播/杆臂/身份/时间/共享状态测试，完整room3单变量2回放8评分完成。实际283次搜索初值审计全部通过；候选仍在686跳变，搜索初值正确采用后被关键帧惯性优化拉动约4.5cm。仅搜索初值不足，不能交付。frame_search_seed_v18_room3_summary_v1.json / assessment_v1.json。
- v19 隔离候选正在构建：仅同参考、上一帧几何有效、普通map更新时保存优化前搜索匹配；关键帧惯性结果几何不合格时，尝试在同一地图版本重新视觉求解相邻两帧。仅双帧都几何通过才提交视觉位置/速度，并退役被拒绝结果的本帧prior；失败保留原生原结果。尚未证明有效。
- 最近全项目996 passed、1 skipped；保护160文件最近全匹配。无正式DLL/live/标定/录制覆盖，无commit/push/默认切换、无实机或实时通过结论。当前动态见 continuation_native_geometry_20261114/STATUS.md。

# 最新续执行检查点：v15原生测试通过，公开单变量对照进行中

整体 IN PROGRESS / NOT ACCEPTED，正式默认未切换。已重新读取官方会话“继续 VS030 VIO 离线验证（官方）”最新记录，旧会话最后因额度中断，不能视为完成。

- v12三段完整双手融合对照全部完成：6/6 worker、6/6世界手部审计通过；同输入/配置/原生DLL/源码核对通过，相机系手部数值与身份字段保持一致。这是worker融合回放，不是新候选实际客户GUI。184714早期失跟两臂均原生242新图、应用259准入，延迟0.566666s，未证明该事件提速。
- v13固定三次room3成对重复全部完成，6/6回放、24/24评分通过；候选54次实际速度初始化审计通过。两个同图恢复control出现19–47cm错误准入；三个候选对应post-entry最大误差约1.7–2.0cm，但随后仍跳变隔离，不能交付。重复同事件不算独立泛化证据。
- v14补齐普通localBA mapChange版本围栏，原生构建/测试通过；4组公开对照8/8回放、32/32评分完成。room3候选实际18次速度初始化审计通过，但仍跳变686/766，未通过验收。
- 同次原生日志发现后续惯性结果未通过已有双目几何，原生仍可能凭15内点返回OK。隔离v15使用真实视觉优化重新求解当前帧与上一帧副本，仅两帧都满足原双目几何、连续时间/身份/地图版本时提交新位姿与body速度，并退役被拒绝惯性结果的新prior；不改bias、共享预积分、地图或host门槛。
- v15真实针孔/鱼眼优化、速度与失败事务测试已通过。16源/二进制、29链接对象哈希与reviewable patch已封存。四组同DLL仅GEOMETRIC_RETRY开关对照运行中，session31114；不得修改其冻结源码/DLL/driver/scorer。
- 最近全项目自动测试996 passed、1 skipped、2 warnings。160保护项最近全匹配。正式DLL/live/标定/录制/stable_offline_v2保留，无新commit/push，无实机或绝对精度/实时通过结论。

最新动态与完整证据以 `continuation_native_geometry_20261114/STATUS.md` 为准。下面历史记录保留；旧的building/pending不代表当前状态。

# 最新续执行检查点：v12已封存，v13进行中

新增原生状态衔接缺陷：视觉重定位窗口SetPose不设置velocity；空ResetFrameIMU未重建metric速度；KeyFrame无速度时零值会被后续惯性优化当状态使用。隔离v13只为同geometry/reference代际、连续真正视觉合格帧重建body velocity，不改变pose/bias/preintegration/准入门槛，尚未验收。失跟禁止插入KF和relocalization orientation bounds仅在隔离v12。

C = `continuation_native_geometry_20261114/`。历史记录与失败保留，以下旧pending以本段和C/STATUS.md为准。

# 最新门控证据修正：恢复几何尚不充分

以下旧表是v4检查点，不得据其把J/I认定已验收。新增v8 room3有实际错误同图准入，0.299/0.280m超预声明0.15m；现有mutable-map双目投影可自洽，初始化参考版本不覆盖全部局部BA形变。J/I/H须补充与失跟前可信坐标绑定的独立证据。v9冻结shadow对应点0，不能算有效性通过；v10在查覆盖率。v7/v8具体右目优化器bug已有修复与A/B，但隔离保留，不等同所有门控通过。全测996通过与160保护哈希一致不取代GT正确性验收。

# 全部门控状态（2026-11-14，实验候选，未正式验收）

本表取代之前的“原生几何 unavailable / v3 待编译”检查点。正式 DLL、active registry、stable_offline_v2、live 配置及正式 1.5 秒窗口均未替换。没有依据的阈值一律保留。单元负例与真实数据是不同证据，不将二者合计为独立失跟样本。

证据目录：`continuation_native_geometry_20261114/`（下称 C）。全量工作树回归 `C/pytest_final_guarded_expiry.log`：996 passed、1 skipped、2 warnings。

| 门控 | 原始/最终参数 | 已执行的数据与根因证据 | 最终状态与仍缺证据 |
|---|---|---|---|
| A 时间戳/相机 IMU 同步 | 单调性、packet lattice、逐会话独立转动核验；不推广固定偏移 | 9 个非空独立 SDK 来源盘点；既有 7 段回放；map_scan 早期静止不足、1800 帧独立视觉/陀螺核验相位100ms，短GUI录制 insufficient_changing_rotation 拒绝 | 保留；会话相位不是曝光时刻真值；硬件同步/物理时间语义未认证 |
| B 初始化 | 原生合法初始化，不强迫完成；anchor 仍要求 frame-bound 有效 body/velocity/bias | v4 解决 pre-BA2 分支内部又要求 BA2 的不可达逻辑；111807/111833/184714 全段原生回放；参考初始化 commit 原子版本 | 候选；有界预测可使用已初始化状态，未初始化不能预测；初始化重力/bias绝对正确性尚缺GT认证 |
| C Tracking | OK/RECENTLY_LOST/LOST/新图分开记录 | C/audit_184714_v5_{candidate,control}_v2 导出参考关键帧、motion、local map、BoW返回值；171–180附近确有原生预测但局部几何不足 | 已诊断；原生短暂OK不等于持续恢复或应用准入 |
| D BA2/尺度 | 不降低 BA2 触发要求，不调整尺度 | 原生已初始化的pre-BA2预测、经过几何复核的双目bootstrap可单独标记；未冒充尺度最终确认 | 显式实验路径；正式默认保留；BA2布尔值不证明精度；尺度非劣未认证 |
| E SE(3)/变换链 | 有限值、正交性/行列式、相机/体/世界原有约束 | C++ ABI字段偏移、native unit；test_vs030_vio_geometry、test_vio_body_state、snapshot tests；公开GT使用camera/body外参且SE3不拟合尺度/时间 | 不变；数值合法仍不能证明定位正确 |
| F 平移跳变 | 1.5m/s + 0.02m，不变 | room3完整长段多次拒绝并独立切图；VS030同输入v4对照0跳变不能证明GT正确 | 保守保留；历史8.41m/280m生成原因及Stereo39.77s仍需对应原生优化过程证据，不能写已修复 |
| F 旋转跳变 | 360deg/s + 3deg，不变 | 真实184714快速转动陀螺证据与图像模糊；独立GT事件标签及负例 | 保留；同源IMU一致不是独立真值，不提高阈值 |
| G 稳定窗口 | 正式1.5s；显式实验0.5/1.0/1.5s | public_v4_development_suite 24/24，同DLL/输入/策略；host_expiry_window_holdout_v1 14/14；典型准入0.50–0.55/1.05/1.55s | 矩阵完成但仅2个开发事件位置；不可据帧数或总case数宣称统计非劣 |
| H 地图身份/参考版本 | map/reset/revision/world/reference/atlas/geometry snapshot fencing不变 | 原生初始化 commit journal；新图null token崩溃已在隔离v4修复；stale/aborted/inconsistent negatives | 候选通过有限不变量测试；不跨图拼接，没有变换的新segment独立 |
| I 单一恢复owner | RecoveryCoordinator + application feedback两阶段，不新增状态机 | 同图原生几何证据绑定最终帧；隔离超时仅请求独立新图；expiry bug严格host-only A/B | 已修复实验分支；pose_jump、sensor invalid、identity保护不清除；长期room3不稳未验收 |
| J 双目几何 | 内点≥50，双目≥30，覆盖≥6/12，比例≥50%，P95≤3，不变 | v4 pinhole PnP/fisheye投影与左右sigma、Tlr；公开真值恢复gap评分；v5详细前端计数 | 隔离候选；已有同图恢复通过预声明0.15m/5deg事件包络，仍缺更多独立错误恢复场景 |
| K IMU 缺口 | 20ms，不变；不补造样本 | test_imu_lattice/test_imu_loss_prediction；expiry后gap/非单调故障保留负例；短GUI断裂录制列为负例 | 保留；未绕过不合格时序 |
| L IMU预测时限 | host/GUI 2s，不变；pre-BA2 native搜索2s；已有BA2原生RECENTLY_LOST策略另有5s上限 | C/host_expiry_AB 与严格room3 host-only配对；过期不再永久污染invalid_reason，仅短恢复开关启用；公开预测误差按时长分桶 | 有界且显示/搜索/真实输出分开；原生搜索duration与host预测上限不能混为同一指标，不延长GUI预测 |
| L IMU残差 | 平移0.30+1.0*t²；旋转25+90*t，不变 | audit_recovery_prediction_gt.py逐时长GT误差；inconsistent residual仍拒绝几何候选 | 经验包络，非协方差；未获统计标定，不调整 |
| M 新地图超时 | 3s，不变；独立segment | room3两组均无法原图恢复；matched host-only基线新图4.50s、候选4.80s；完整段分别9/6次请求 | 没有证明普遍更快；重建次数变少不等于漂移改善，不改默认 |
| N 世界/segment手部 | 精确同帧、已准入相机姿态、身份一致；不使用冻结/IMU估计 | 3段全融合6/6；world_hand_audit errors=[]；camera-space数值/mask/状态完全一致；111833候选双手segment 138帧 | 软件融合不变量通过；无外部手部世界GT；实际GUI回放单列验收，不用worker代替 |
| O GUI显示 | trusted/IMU临时/视觉candidate/frozen/origin独立，年龄从最后真实估计累计 | GUI冻结年龄回归；真实客户GUI 184714完整回放与display_audit、截图独立输出 | 实际GUI结果见C/gui_184714_v4_expiry_guarded；截图不代替逐帧world-hand审计；无实机/实时FPS认证 |

架构审计：保持单一恢复准入责任者；跨进程反馈事务与最终快照绑定；过期锚点退休避免无界预测；没有新增融合缓存或状态机。未声称完成线程竞争/长期内存泄漏的穷尽证明。离线并发回放性能只能记录观测，不能证明30FPS或性能非劣。

公开GT评分使用固定时间关联、单位米、SE(3)无尺度对齐；同图恢复使用遮挡前锚点，不能用恢复后拟合掩盖错误。room3长段失败、room1某次2s跳变失败、旧v1/v2崩溃均保留；不按首次成功删除后半程失败。
