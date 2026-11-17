> 产品接续 v6：三段完整实际客户Qt对照及111807暂停/恢复完成；全项目回归通过，完整性17230项/5670文件errors=[]。有效修复为精确采集绑定、即时GUI世界手部、segment和暂停/年龄保护。手部选帧+短队列虽增显示，但184714最大可信位姿采样间隔7.833→10.033s，拒绝晋升；OpenCV单线程也不采用。额外融合等待观测不足以解释250ms失效，不新增线程。无自有进程，PID12348未动。正式/默认/旧候选保留，整体ACTIVE / NOT ACCEPTED。下一步检查实时丢帧下原生初始化/参考变更，而非降低门控。报告 handoff/p0_reference_stability_20261115/live_delivery_product_review_20261116_v1/交付状态与下一步.md；检查点 product_continuation_checkpoint_20261116_v6.json。以下为历史。

> 产品接续 v5：实际 Qt 已修复同帧世界手部显示；111833 默认队列/提交采样159、144帧，短队列+手部选帧200、207帧，但仍约15 FPS手部/13 FPS VIO及秒级恢复，未验收、不切默认。单独手部跟随提交和OpenCV单线程未改善，不采用。111807暂停实测显示固定、后台VIO持续；候选session31019运行勿重启。接着184714完整GUI A/B、全测和身份审计。详情 product_continuation_checkpoint_20261116_v5.json。以下为历史。

> 最新产品接续 v4（2026-11-16）：整体 ACTIVE / NOT ACCEPTED，正式/默认不变。完整184714共享传图同DLL AB完成：端到端6.54→8.55fps，VIO调用79.7→45.6ms；最长可信中断2.93→3.03s，恢复未提速。实际30Hz双手录制驱动LiveVio：VIO9.5→15.4fps，但世界手部265/403帧均未到Segment/GUI。独立隔离修复host/sensor显式采集绑定和异步预览，367帧世界→367帧Segment与GUI绑定成功；实际Qt仍0/439显示，查明有序写盘等待阻塞GUI。新隔离副本仅增加同一精确绑定的即时Qt通知，录制顺序保持，显示限250ms且同segment。实际Qt session46686运行，不重启。后续复核暂停/停止，再全测及身份审计。检查点：handoff/p0_reference_stability_20261115/product_continuation_checkpoint_20261116_v4.json。以下为历史。

> 最新产品接续 v3（2026-11-16）：整体 ACTIVE / NOT ACCEPTED；正式/默认不变。temporal initializer 整段最长中断仅4.033→3.933s，未采用。共享内存真实原生360帧ABBA已完成：VIO调用中位71–73→40–42ms，但快速两臂140/141帧提前失跟，稳定性尚未通过。完整184714原手部+VIO同DLL传图AB session87872正在运行，不重启、不与其他性能回放并行。实际LiveVio+原手部模型的111833双手录制节奏实验驱动已准备；后续检查延迟/丢帧/精确帧融合，不以测试数交付。检查点：handoff/p0_reference_stability_20261115/product_continuation_checkpoint_20261116_v3.json。以下为历史。

> 最新产品接续（2026-11-16）：目标是稳定实时双目手部系统，测试数量不作为成果。整体 ACTIVE / NOT ACCEPTED；正式/默认保持。首次恢复精确图已核实：126图有效、2图排除，无可直接纯视觉挽救机会；原生短时预测确已工作。新默认关闭 temporal initializer v2 已构建并通过真实图像功能检查；前360帧 ABBA 对照288/288、候选262/259帧持续输出，但仅一臂实际延后初始化，因果未证明。完整184714原PipelineWorker+手部+VIO双臂 session49533 进行中，不重启；之后检查最长无输出窗口/手部世界坐标恢复。此前整段仅约4fps、VIO调用中位127ms，实时目标尚未达到。通信Nagle/缓冲区两假设无收益，不改生产。完整A/B后测真实全分辨率Frame成本，不再堆叠门控变体。检查点：handoff/p0_reference_stability_20261115/product_continuation_checkpoint_20261116_v1.json。以下为历史。

> 最新续执行（原生逐帧优化诊断）：上一轮geometry_restart应用修正、两轮公开/三段融合/实际GUI已完成；源审计12053项/4782文件errors=[]，仍未验收。当前复用冻结DLL完整184714只跑VIO，采集3020–3275精确优化图session33867；独立图重放工具session86768在新目录编译，不重建DLL。旧候选与正式环境保留，未知PID12348未触碰。检查点：handoff/p0_reference_stability_20261115/real_frame_optimization_checkpoint_v1.json。以下为历史。

> 最新完成检查点（geometry restart v2）：默认关闭的应用隔离修正已完成两轮完整EuRoC、三段完整原手部融合AB、实际Qt及全测。公开几何→独立新图准入约0.65s（一次1.20s），隔离244→46/183→58；旧位姿不释放、不拼图。真实184714原生恢复提速仍未证明；整体ACTIVE / NOT ACCEPTED，稳定/默认不切换。无待授权/本轮运行任务。详情：handoff/p0_reference_stability_20261115/GEOMETRY_RESTART_COMPLETION_REVIEW_v2.md。以下为历史。

> 最新续执行（geometry restart v1）：默认关闭的应用隔离副本已实际实现；37项新测试、996 passed/1 skipped、167项隔离默认回归通过。首轮完整EuRoC三次合格几何→独立新图准入约0.65s，对照2.8–2.9s；隔离244→46帧，旧位姿/世界手部仍禁止输出，未拼图。反序完整公开AB session89177、184714完整融合AB session57825进行中，勿重启。冷重建组合完整录制净减少65准入帧，已拒绝、不扩展。整体ACTIVE / NOT ACCEPTED，正式/默认不切换，无待授权。检查点：handoff/p0_reference_stability_20261115/geometry_restart_progress_checkpoint_v1.json。以下为历史。

> 最新续执行（v2）：三段完整融合AB、两轮完整EuRoC、8次TUM-VI遮挡、真实Qt GUI及完整测试已收齐。公开初始化精度重复改善，但184714未证明持续恢复提速；ACTIVE / NOT ACCEPTED，稳定/默认不切换，无待授权。新旧地图研究证据与完整结果：handoff/p0_reference_stability_20261115/PREINIT_KF_RECOVERY_REVIEW_v2.md。以下为历史。

> 最新续执行检查点（2026-11-15）：整体 ACTIVE / NOT ACCEPTED，无待授权阻塞。冷启动地图0.2s重试在完整184714仅提前1帧、准入减少28帧，保留隔离证据但不采用。新根因证据：首次局部视觉BA的20条高残差边全部已被跟踪器判为离群，却按原设计进入关键帧。全新 isolated_preinit_kf_inliers_v1 仅重编译Tracking、默认关闭，只在IMU_STEREO初始化前不继承这些时间关联。真实EuRoC180帧ABBA完成，两候选首次BA后KF2误差均约2.1cm；首次对照37.9cm，另一次对照4.8cm，调度差异保留。完整1710帧首轮共同1429个0.1s窗口RPE平移5.84→1.09cm、旋转0.875→0.163deg，reset17→4，但仍244帧隔离，未验收。996 passed/1 skipped及隔离恢复测试通过；6232项/3485文件身份核验errors=[]。反序完整EuRoC session81658、完整184714手部融合AB session56395运行中，勿重启/终止。正式DLL/默认配置/原始数据/v11/v23/v24不变。详情：handoff/p0_reference_stability_20261115/preinit_kf_stability_checkpoint_v1.json。以下为历史。

> 最新完整验证检查点（2026-11-15，startup geometry v3）：本轮隔离实验/三段完整VIO+手部/16次公开回放及评分/实际Qt/全测均已结束；整体 ACTIVE / NOT ACCEPTED，不切默认。前360帧仅一次可追溯提前0.867s；完整184714失跟→持续准入control3.900002s、candidate4.933334s，六次完整回放均未触发startup最终释放，未证明稳定收益。实际seed203/237的63次几何查询仅238合格，勿把239–269等待误称为应用拒绝已有合格几何。room3/corridor4长等待及EuRoC原生OK后大量拒绝仍未解决。手部相机输出两臂一致、世界发布审计通过；996passed/1skipped、隔离167passed。最终4862项期望/2320唯一文件匹配，v11仅按外部干预后基线核验；正式/旧候选/stable_offline_v2保持。用户长期隔离授权有效，无待确认、无本轮运行任务，不重复已完成回放。下一步查实际初始化帧几何与原生OK拒绝区间，不放宽门控。完整报告：handoff/p0_reference_stability_20261115/startup_geometry_completion_review_v1/REPORT.md；综合summary.json及final_integrity.json同目录。以下均为历史。

> 最新执行（2026-11-15）：用户“以后都允许就”已解除逐项授权阻塞，不再重复确认。首个应用ABBA已完成：无因果持续改善；240帧临时接受后最终内点比例53/118不合格，隔离又挡住245/246候选。保留全部结果；在全新v3应用副本只修本阶段独立新图的视觉复核自锁，既有数值门槛/pose_jump/身份保护保持，28项新回归、13项工具回归通过。session98062运行固定前360帧ABBA，不重启。首轮准备工具Windows相对路径比较bug导致0帧失败，副本与日志保留，v2/v3均已新增真正补丁SHA及两个解释器参数核验。正式DLL/配置未改，无原生重编译。整体IN_PROGRESS / NOT ACCEPTED；历史BLOCKED已解除。检查点：handoff/p0_reference_stability_20261115/startup_application_actual_execution_checkpoint_v13.json。

> 最新授权（2026-11-15）：用户明确回复“以后都允许就”及“允许新应用隔离副本及离线 A/B”。此前应用行为授权阻塞已解除；今后当前目标所需的隔离修改、构建及离线验证直接执行，不再逐项重复确认。正式环境与冻结证据保留，不代表允许覆盖正式部署。现在从startup_geometry_execution_tools_v1执行全新应用副本及固定360帧ABBA；历史blocked条目仅为历史。授权记录：handoff/p0_reference_stability_20261115/standing_isolated_execution_authorization_20261115_v1.json。

> 最新状态（2026-11-15）：新应用恢复行为的授权仍未答复；同一边界已连续三轮复核，当前阻塞审计通过，整体 BLOCKED / NOT ACCEPTED。查询范围构建/ABBA已完成且未提速；候选外应用提案与执行工具准备完毕，21项提案/134项既有默认策略/12项工具测试已通过。封存提案15文件、工具15文件匹配，新应用副本/新回放/授权记录均不存在，无自有VIO进程；PID12348未触碰。没有重复实验或提问，未将自动目标续执行当作新行为授权。此前“不要擅自……覆盖任何候选”的边界仍有效，最新明确答复只覆盖查询范围补丁。收到已发具体授权答复后，从startup_geometry_execution_tools_v1直接继续；不重新开始。完整产品目标保留且未验收，正式环境不变。证据：handoff/p0_reference_stability_20261115/startup_geometry_authorization_blocked_audit_v1.json。以下ACTIVE为历史。

> 最新检查点（2026-11-15，应用修正工具准备）：整体 ACTIVE / NOT ACCEPTED。新地图首次几何恢复的具体授权仍未答复，不重复询问。候选外执行工具已完成：准备副本、固定360帧ABBA、父/子Python源码路径探测、临时候选与最终局部地图恢复分开统计。12项工具回归通过，两项预览及三项拒绝路径通过；原四臂持续恢复仍288帧，未将历史日志冒充新实验。封存提案保持一致；新应用副本与新回放目录均不存在，未启动VIO、未重编译或覆盖任何候选。未知PID12348保留。下一步只在收到实际人类答复后执行工具，不视自动目标续执行为新增授权。检查点：handoff/p0_reference_stability_20261115/startup_execution_tools_checkpoint_v11.json。以下为历史。

> 最新检查点（2026-11-15）：查询范围修正隔离构建与三项原生验证已完成；184714前360帧同输入ABBA共1440帧、零跳帧。四臂均288帧持续准入，最终绑定几何273帧→准入0.50s；本补丁未提升持续恢复，不晋升、不扩展其完整回归。候选两臂均240/245/246帧找到合格中间几何，但新reset=1地图无已准入视觉身份，预接受回调以no_trusted_anchor/rejected_visual_verifier_unavailable拒绝，原生最终仍失跟。原统计将被拒中间候选误作绑定错误，首次失败保留；独立v2分析只对显式拒绝且未准入候选分类，4项回归通过。正式与旧候选2293保护核验通过，运行冻结亦匹配。候选外startup_geometry_recovery_proposal_v1已准备默认关闭的应用修正，21项新测试/134项既有默认策略测试通过，真实原生持续改善尚未验证；未应用任何候选，待该新行为单独授权。整体ACTIVE / NOT ACCEPTED。检查点：handoff/p0_reference_stability_20261115/map_scope_prefix_complete_checkpoint_v10.json。以下均历史。

> 最新授权执行（2026-11-15）：用户已答复“允许新查询范围副本构建及离线 A/B”，此前BLOCKED解除；整体 ACTIVE / NOT ACCEPTED。新 isolated_map_scope_v1 已应用封存补丁，只重编译KeyFrameDatabase，28个ORB对象复用，build session18627进行中。准备阶段遇继承的旧授权文件同名冲突，已保留旧文件并新增本次专属授权，全部复制文件与2293项保护身份复核通过；未开始编译前的故障已记录。后续原生单位→184714前360帧ABBA，按已批准计划决定是否扩展完整回归。现有计时/v11/v23/v24/观测副本、正式环境保持不变；不再重复询问同一授权。检查点：handoff/p0_reference_stability_20261115/map_scope_authorized_execution_checkpoint_v8.json。以下blocked为历史。

> 最新状态（2026-11-15，查询范围修正等待授权）：整体将按已通过的三轮阻塞审计标记 BLOCKED / NOT ACCEPTED，完整目标保留。新 map_scope.patch 的隔离构建与离线A_B具体确认已发出但未答复；最近授权仅覆盖已完成的计时修正，自动续执行不扩大权限。只读图像/原生几何诊断、封存补丁、执行工具及9项针对性回归已完成；本轮无自有实验进程，新副本/原生单位/ABBA目录均不存在，两份封存清单220/16文件全部匹配。未重编译或覆盖任何候选，正式环境不变。不要重复提问或重启完成实验；收到明确答复后直接执行既有准备工具。证据：handoff/p0_reference_stability_20261115/map_scope_authorization_blocked_audit_v1.json。以下active检查点为历史。

> 最新检查点（2026-11-15）：map_scope 新补丁确认已发出，尚未收到答复，不重复询问，也不视自动续执行为授权。已补齐新副本准备/构建、三项原生单元及360帧ABBA执行工具；9项针对性测试通过，统计复核历史184714持续恢复2.633334s/4.900001s。未启动新候选构建或原生回放；现有候选、正式环境和封存诊断保持不变。整体 ACTIVE / NOT ACCEPTED，计时实验仍不晋升。详见 handoff/p0_reference_stability_20261115/map_scope_execution_tools_checkpoint_v7.json。以下为历史。

> 最新检查点（2026-11-15）：计时实验及客户GUI均已完成，不晋升，整体 ACTIVE / NOT ACCEPTED。GUI 3283帧/零跳帧、2740显示连接审计 errors=[]；全测996passed/1skipped。继续只读诊断184714发现数据库先跨地图排名后按地图过滤，可压制新图候选。100张真实图像特征数完全吻合；62次查询47次被此机制排空；238种子在240/245/246帧可通过原生几何复核，但完整持续恢复尚未验证。新map_scope.patch/构建与A_B计划均在候选外准备，未建新候选、未应用或重编译。LocalMapping源码进程视图不同；Python与封存哈希一致，未证实损坏。正式/v11/v23/v24/观察/计时副本保护复核全部通过；v11仍为外部干预后身份。详见 handoff/p0_reference_stability_20261115/relocalization_map_scope_diagnostic_v1/诊断与下一步.md 及 clock_and_map_scope_checkpoint_v6.json。以下正在运行/待收尾条目均为历史。

> 最新完成检查点（2026-11-15，三段计时修正完整融合已完成）：整体 ACTIVE / NOT ACCEPTED，不晋升。session61176 exit0：184714/111807/111833各两臂全分辨率零跳帧；六次世界手部审计通过，三对相机系数值一致。准入control→candidate分别3126→3054、686→689、1255→1319；jump1→0、0→0、2→0。184714候选原生失跟→最终准入4.90s，对照2.63s；候选含两次新图短暂OK后立即失跟，最终合格几何→准入均0.50s，无恢复优势。111833双手相机237/237、双手segment118→180，但候选未BA2。全测996passed/1skipped/2warnings(96.55s)。独立客户GUI session5771正在完整184714回放，实际DLL哈希68688899...已从其自有进程映射核验；不重启、不影响其他GUI。原始图像抽查与轨迹图已生成，frame171角步主要来自IMU预测，不得误称BA物理跳变。候选/default/v11/v23/v24/原观测副本保护保持；v11外部干预后身份不当原冻结。最终GUI连接审计/保护复核/报告待收尾。以下为历史。

> 最新执行检查点（2026-11-15，计时修正实际回归继续）：整体 ACTIVE / NOT ACCEPTED。此前授权已执行，不再重复请求。room1八次0.2/0.5/1/2s遮挡全部完成，148遮挡帧绑定原生预测成功；0.5s候选原生几何恢复慢0.30s，应用仍正常0.50s窗口。完整184714对照3283帧零跳帧、准入3126、手部世界审计exit0；主session61176正在候选臂，随后111807/111833成对原手部融合及全项目测试，勿重启。对照180失跟，60次预测成功最长1.966667s，新地图244合格→259准入0.50s；不是旧图恢复。保护核验1747项匹配（含28复用对象及6新构建身份），v11仍外部干预后基线。已准备独立客户GUI回归，成对回放结束后运行；正式/default/v11/v23/v24/原观测副本未改，无commit/push。证据：handoff/p0_reference_stability_20261115/clock_execution_checkpoint_v3.json。以下为历史。

> 最新执行检查点（2026-11-15，计时修正已授权实际回归）：整体 active / NOT ACCEPTED，旧blocked失效。新副本仅LocalMapping改动，28ORB对象复用，构建62994exit0；两个原生单位及真实初始化失败/重置45672exit0。111833四遍全1433帧零跳帧ABBA96795完成：control jump3/6，candidate0/0；候选最大KF更新0.0277/0.0326m，但未进入BA2。候选61次IMU预测最长2s，原生旧地图几何仍失败，2.666668s后新segment准入，合格新几何→准入0.500002s；不是原图恢复提速证明。EuRoC79403两臂1710帧及GT共同1541对已完成，候选22.7s完成BA2；成熟段平移RMSE8.42→9.15cm、姿态0.449→0.307deg，精度结果混合，不能晋升。room1八次遮挡session19626运行中；已完成0.2/0.5s，候选0.5s遮挡恢复0.8004s vscontrol0.5001s，继续检查剩余并保留不利结果。接下来优先完整184714，再111807/111833两臂原手部融合、完整测试；正式/v11/v23/v24/观测副本不变，无commit/push。当前检查点：handoff/p0_reference_stability_20261115/clock_execution_checkpoint_v2.json。以下为历史状态。

> 最新授权检查点（2026-11-15，计时修正新副本）：用户已明确答复“允许新计时修正副本构建及离线 A/B”，此前等待授权阻塞解除，整体仍未验收。新的 isolated_refinement_clock_v1 从冻结观测副本复制389文件，应用已审阅唯一计时补丁；28个ORB对象冻结复用，仅重新编译LocalMapping，build session62994进行中。原v11/v23/v24/观察副本/正式环境不变。接续执行原生单位后111833同输入全分辨率ABBA重复回放，两臂bootstrap/prior/transition seed均0、phase106ms，大图同步输出关闭；检查实际初始化/BA1/BA2、恢复延迟、地图形变及边界进入大窗口。新授权不代表物理精度或验收通过；默认不切换。当前执行记录：handoff/p0_reference_stability_20261115/isolated_refinement_clock_v1/authorization_execution_checkpoint.json。以下blocked与pending仅为历史。

> 最新状态（2026-11-15，等待具体行为构建授权）：整体目标未完成，将标记 BLOCKED；范围不缩减。连续三轮存在同一明确阻塞：计时修正 refinement_clock.patch 的全新隔离副本构建及离线A/B尚未获得答复。此前具体授权问题保持待答复，不重复提交；自动goal续执行不能视为批准。当前授权内的观察构建、精确图重放、独立调度/语法检查、负实验与边界提升诊断已完成。现场确认无本轮运行任务，仅未知Python12348未触碰；26项proposal封存文件全部一致、基线LocalMapping未改、新行为候选目录不存在。正式/v11/v23/v24/已有观测副本未修改或重编译。下一步唯一依赖：取得已发出具体问题的明确答复后执行新副本构建与真实A/B。细节：handoff/p0_reference_stability_20261115/behavior_build_authorization_blocked_v1.json。以下历史active检查点保留。

> 最新检查点（2026-11-15，固定边界提升只读诊断）：整体 active / NOT ACCEPTED。待计时补丁新隔离行为DLL授权，未应用/未构建；本轮仅分析既有日志和精确异常图。定点111833 KF239为10条惯性边、KF240扩大25条；158→160从旧固定边界重新进入内部，KF200后与KF240前残差最大分量差1.129e-7，信息矩阵倍率100（比例误差1.531e-16），不再有边界Huber，稳健cost1227.976→2289840.884，占KF240惯性cost74.872%；捕获图独立计算与journal吻合，实际最大body更新1.528515m。此为既有边界策略与历史状态不一致共同造成的目标突变，不能称为新物理运动或证明计时修正已有效。新增A/B必要判据要求覆盖真正BA2后与10→25窗口转换，不能仅靠延长初始化期验收。两遍调度不同的回放统计仅为观察，不作恢复速度因果比较。现有候选/正式环境/手部/原始数据不变；只发现未知Python12348，未触碰。证据：handoff/p0_reference_stability_20261115/window_boundary_promotion_audit_v1/{findings.json,诊断说明.md,后续A_B必要判据.md}。以下为历史检查点。

> 最新检查点（2026-11-15，延迟初始化阶段计时 proposal）：整体 active / NOT ACCEPTED。继承完整局部图精确重放证据，未重启既有实验。111833两次完整记录均在首次观察IMU初始化后0.10s完成BA2；LocalMapping首次把初始化前地图年龄赋给mTinit，提前满足5/15s阶段条件。候选外 refinement_clock.patch 仅修正IMU_STEREO首次阶段计时，不改阈值/权重/准入/单目。git apply --check、独立C++阶段调度回归及完整源/Zs均exit0（原syntax session22184已结束）；速度/bias warm-start仍1.645364m更新，负结果保留。1391项既有保护文件身份核验errors=[]。尚未应用到任何候选，未构建行为DLL；已通过request_user_input_async请求仅此新隔离副本构建及离线A/B，等待答复，不能扩大观测授权。现有观测/v11/v23/v24/正式/stable_offline_v2/标定/录制/手部不变，无commit/push。预BA2已有有界预测，但LOST等策略仍依赖BA2，须真实回归检验延后阶段的影响。证据与明确构建A/B计划：handoff/p0_reference_stability_20261115/proposed_refinement_clock_fix_v1/证据与验证计划.md 和 build_ab_plan.json。无本轮长任务运行；未知PID12348不触碰。以下为历史检查点。

> 最新完成检查点（2026-11-15，已授权完整局部图观测）：整体active / NOT ACCEPTED，默认不切换。新隔离观察DLL构建和两个原生单位通过；111833两次完整1433帧/零跳帧，首遍0jump/最大KF更新0.028m，缩窄214–246窗口固定106ms第二遍复现8jump/最大KF更新2.497662m。21+33=54对图全部独立原生精确重放，位姿/点/速度/bias误差0，残差缓存匹配；KF240已捕获1.528515m非刚性更新，视觉cost6347→35622而IMU cost3058326→2567。独立同权重20/40迭代均15次收敛，更新增至2.009817m，不能靠增加迭代修复。345条惯性边information与C逆匹配、Jacobian差0；3条长区间14.9–17.1µs时间差超过10µs筛查，失败原证据保留，不据此认定物理故障。再次完整测试996 passed/1 skipped/2 warnings；正式/v23/v24/v11干预后共1020项及实验547项身份匹配，另外原v23副本304文件匹配。新增观察授权已执行，无等待确认；无本轮长任务运行。保护版本/标定/原始录制/手部不变，无commit/push。详细报告：handoff/p0_reference_stability_20261115/完整局部图诊断交接_20261115.md；汇总observer_diagnosis_summary_v1.json。下一步基于精确异常图核验物理噪声/标定/时序与旧地图状态可观性，并准备可审阅的独立新segment恢复方案，不放宽门控/不直接解隔离。以下进度均为历史。

> 最新授权检查点（2026-11-15，完整局部BA图观测）：用户已明确允许新观测副本构建和离线诊断，不再等待此授权。新 isolated_local_graph_observer_v1 两次构建exit0、两个原生单位通过；schema2补齐g2o internal_id；独立reader 14项受控/异常输入检查通过。111833完整1433帧/零跳帧exit0，21对局部图全部精确重放；本次0jump、最大KF更新0.028m，未捕获历史大跳变，不是修复/提速证据。原始夹具失败及混合journal保留，已按12行单位前缀/12502行回放边界生成可追溯副本。全项目测试正在运行，随后使用相同DLL缩窄214–246窗口、固定106ms继续捕获异常；不重编译DLL。正式/stable_offline_v2/v11/v23/v24/手部保持；整体active / NOT ACCEPTED。证据：handoff/p0_reference_stability_20261115/observer_authorization_and_execution_v1.json 和 recorded_local_graph_replay_v2/findings.json。以下旧“待观测授权”已失效。

> 最新续执行（2026-11-15，P0同输入传感器对照与相位诊断均已结束）：整体 active / NOT ACCEPTED。冻结v23的111833 Stereo/IMU对照1312/1250准入、0/4跳变；4次均归因到已提交LocalInertialBA参考KF更新。新增固定phase106/100两次完整1433帧、零跳帧、同输入/源码/DLL/噪声/门控审计通过；跳变12/11、最大KF改动0.289/0.452m，23/23跳变均对应惯性BA参考更新，100ms未修复、不采用。两臂均91帧隔离，部分几何合格仍因pose_jump/旧锚点失效拒绝，最终独立新segment准入；新参考几何至准入均0.533332s。独立中段缺激励、后段holdout全网格平坦，不能宣称时间漂移；运行噪声来源及sqrt频率换算已核查，未改权重。正式160/v23 294/v24 291/v11干预后275项哈希匹配。此前transition seed v2完整融合/公开/GUI/996测试证据继续有效，但新增分支未在真实candidate触发，未证明恢复提速。完整局部BA图观测补丁仍仅proposal：已提出新隔离副本构建请求，待用户明确答复；不得扩大seed授权，不应用或构建该新DLL。当前无本轮长任务运行，未知PID12348不触碰。正式/stable_offline_v2/v11/v23/v24及手部不变、无commit/push。详见 `handoff/p0_reference_stability_20261115/续执行与相位对照_20261115.md` 和 `fixed_phase_111833_v1/{findings,quarantine_findings}.json`。以下历史记录保留，旧building/pending/completed以本条为准。

> 当前续执行（2026-11-15，预测断链 v2 完整实验已收齐）：用户授权的新隔离副本 `handoff/transition_seed_20261115/` 构建及原生几何/预测链/完整 Track 两臂受控测试通过；184714/111807/111833 六次全分辨率原手部融合、room1四长度八次遮挡、EuRoC两臂1710帧、真实客户GUI184714共3283帧均完成。全项目996 passed/1 skipped/2 warnings。保护审计和1484项实验身份核对 errors=[]。直接记录到control3039转移帧HasPose丢失导致3040预测断链；完整Track测试证明修复机制，但真实candidate未执行新增分支，不能把准入2946→3068视为提速证明。111833跳变拒绝5→22、双手segment236→137，P0仍不稳定，不晋升。正式/stable_offline_v2/v11/v23/v24保持；整体goal active / NOT ACCEPTED。报告：`../transition_seed_20261115/验证交付报告.md`，机器汇总delivery_summary.json。旧授权阻塞已解除；以下全部为历史记录，不代表新实验仍在运行。

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

整体仍 IN PROGRESS。v12重定位右目索引崩溃修复及失跟禁止建点已完成4组同DLL开发AB：8/8回放、32/32评分执行通过，所有worker exit0。room3仍有恢复后684–688的18–45cm错误准入，不能交付或切默认。已定位新KF速度为零的状态衔接，隔离v13正在构建。三段v12全融合尚在进行。

C = `continuation_native_geometry_20261114/`。历史记录与失败保留，以下旧pending以本段和C/STATUS.md为准。

# 最新状态：原生右目约束修复已做开发对照，恢复安全仍未通过

整体 IN PROGRESS，未切默认。v6逐帧证据确认room3局部BA存在非刚性地图形变；v7修复两个全局BA右目combined-index边界错误；v8修复局部惯性BA右目octave/相机uncertainty错误，均只在隔离候选。v7 8/8回放、7/8原生事务审计通过（一个关闭后日志重启失败保留）；v8 8/8回放、32/32评分通过，所有worker exit0。

关键失败：v8 room3两臂在恢复+0.550249s同图准入，但遮挡前锚定GT误差0.299m/0.280m超过预声明0.15m包络，事件窗口有2/4帧错误准入。几何投影与Tracking OK不足以保证失跟前坐标连续，不能因跳变数减少推广。

v9冻结地图纯诊断完成2821帧，未复现相同错误准入，已有pose_jump保护触发；冻结点对应数为0，目前不能解释为成功检测形变。v10正在补充点ID覆盖率诊断；没有生产冻结准入实现。源码/输入冻结、失败产物均保留。

本次全项目回归996 passed、1 skipped、2 warnings；160保护项哈希一致。详见C/STATUS.md、right_weight_v8_development_summary_v1.json、room3_v9_frozen_shadow_audit_v2.json。下面保留上一阶段结果，若有冲突以本段为准。

# VIO 门控优化任务状态：整体进行中

2026-11-14，继续官方会话 `01a1d55a-1bf2-79e2-b141-91bbc390b891`、当前main/HEAD d6e5d54和继承dirty工作树，无新分支、无reset/clean、无commit/push。

本阶段已实际完成：v4原生几何/初始化参考证据和pre-BA2有界搜索；隔离DLL构建/ABI/unit；host预测到期不再永久污染传感器invalid的实验开关修复；开发矩阵24/24、补充窗口/留出14/14、严格expiry A/B8/8和room3 host-only2/2；3段全融合6/6；184714真实客户GUI双臂各3283帧并逐帧手部档案审计；新增Stereo双臂各3283帧；map_scan全段7295处理及独立中后时序窗口；短GUI录制时序不可观负例；corridor4双臂各1927帧；EuRoC新v4全段1710帧评分。

最终回归996 passed、1 skipped、2 warnings；正式/快照/盘点录制160项保护哈希一致。

未完成正式验收：room3与corridor4长段多跳变/新图、EuRoC精度非劣、历史大跳变原生优化根因、长期时序/RSS/实时性能、VS030外部GT和实机。不得标记所有门控PASS。默认1.5s、正式DLL/live/registry和stable_offline_v2保留。显式实验入口 `C/run_184714_geometry_v4_experimental.cmd`。

证据根目录：`continuation_native_geometry_20261114/`（下称 C）。主报告 `C/阶段交付与未决问题.md`；可复核总索引 `C/CROSS_DATASET_FINAL_GUARDED_EXPIRY_V2.json/.md`；正式默认未切换。旧主文件备份于 `C/master_state_before_final_guarded_expiry/`，历史失败仍原样保留。


## 2026-11-16 产品续执行 v10（最新，整体未验收）

检查点：`E:/EgoHandDataEngine/handoff/p0_reference_stability_20261115/product_continuation_checkpoint_20261116_v10.json`。原生ORB提取和GUI显示开销已取证并在新隔离副本实际优化；4线程方案因供帧积压与手部下降拒绝，2线程及同帧提交仍未解决184714，显示副本缩放保留为默认关闭的局部性能改进。完整Qt三段及重复/暂停退出已结束。184714仍约7.7秒位姿采样中断，世界手部显示改善有限，正式默认不变，目标ACTIVE / NOT ACCEPTED。全项目996/1skip、新隔离118、23920期望/8026文件审计无错，不能当作产品验收。所有自有进程结束，不等待旧session；PID12348不动。

继续优先查184714实时采样原采集56–294帧的IMU初始化前连续失跟/反复建图，以及HEVC解码/Qt渲染CPU预算；不新增无依据门控、不增大250ms显示上限、不放宽地图身份。具体新报告：`handoff/p0_reference_stability_20261115/real_time_product_review_20261116_v2/阶段结果与未决产品问题.md`。不要启动已拒绝4线程方案的预备公开回放脚本。


## 2026-11-16 产品续执行 v12（最新，ACTIVE / NOT ACCEPTED）

入口：`E:/EgoHandDataEngine/handoff/p0_reference_stability_20261115/product_continuation_checkpoint_20261116_v12.json`。本轮实际构建两个新 Tracking-only 隔离副本，并完成两组完整184714实际Qt ABBA。BA2前预测后视觉优先条件会影响正常OK帧，已拒绝；成熟RECENTLY_LOST复用短时视觉搜索分支在重复臂实际执行15帧，但未证明GUI世界手部可重复改善，继续默认关闭。两组均已结束，不重复等待/重启。正式DLL、默认配置、原数据、stable_offline_v2不变，PID12348不动。

下一步聚焦源3079/3099：有114/139几何内点但关联383/399，比例不足。先检查视觉修正位姿后的真实重新投影和描述子重新匹配；不删除outlier美化比例、不降低门槛。更详细证据：`handoff/p0_reference_stability_20261115/real_time_product_review_20261116_v4/阶段结果与未决产品问题.md`。扩展v8审计28672期望/8830文件无错，但不是产品验收。未经收益证明不启动更多公开/三段回归，仅保留已完成根全测结果。


## Product continuation v13 (20261116)

ACTIVE / NOT ACCEPTED. Bounded rematch full Qt ABBA and preinit rotation startup Qt ABBA completed: neither enables a default change. Exact172-frame mode diagnostic confirms repeated native losses in both modes. No product acceptance by PASS count. Checkpoint: E:\EgoHandDataEngine\handoff\p0_reference_stability_20261115\product_continuation_checkpoint_20261116_v13.json
Report: E:\EgoHandDataEngine\handoff\p0_reference_stability_20261115\real_time_product_review_20261116_v5\阶段结果与未决产品问题.md
Next: E:\EgoHandDataEngine\handoff\p0_reference_stability_20261115\NEXT_ACTION_v13.md
