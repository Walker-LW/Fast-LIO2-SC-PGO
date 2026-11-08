# FAST-LIO2 当前状态与会话交接

最后更新：2026-11-08（Asia/Shanghai）。保留原文件名，方便沿用已有链接。

## 1. 文档边界

本文件只保存当前状态、关键路径和下一步。详细实验与排障历史见 [问题处理与改进复盘](FAST-LIO2问题处理与改进复盘.md)，目标与阶段进度见 [研究路线与进度](FAST-LIO2研究路线与进度.md)。已过期的准备命令不作为当前任务入口。

新会话从本文件开始：先确认第2节的已完成清单、第4节的实际配置与下一项；没有完整结果的工作不计入实验结论。执行入口需按当前环境核对，不因已有脚本就视为整段运行已验收。

## 2. 当前实现与验证状态

**当前主线：仅用LiDAR＋IMU运行FAST-LIO2＋SC-PGO，定位并改善Corridor01点云重影。** 相机仅曾用于离线场景旁证，没有输入前端估计、回环因子或地图优化。RGB-D/标记板位姿参考支线现暂停；此前官方标定使用问题不再是主线的待答阻塞项。

| 已执行工作 | 实际结果 | 不能据此声称 |
|---|---|---|
| FAST-LIO2基线 | Corridor01/02 blind=1完整运行；Corridor01 blind=4严重发散 | 已证明全部重影来自几何退化或缺少回环 |
| SC-PGO Corridor01旧001 | 完整运行，264关键帧；最大位置修正约2.26cm，目视无明显改善 | 后端所有回环正确 |
| SC-PGO Riverside02 | 完整运行，5951关键帧；地图有明显定性收拢 | 已获GT精度提升或参数适合室内 |
| SC-PGO Corridor01配置002 | 完整运行，383关键帧；最大位置修正约7.18cm，末段8组配对均失败 | 半径/关键帧变化已解决重影 |
| 后端几何验证与修正 | 确认子地图拼接及测量坐标/方向错误；实际源码已改，独立编译，六组已知几何测试通过 | 代码正确性修正自动保证地图改善 |
| 几何修正整图对照001 | 用户完成old/fixed；4034组同输入、383帧相同关键帧，每图2102827点；最大位置改动6.89→1.79cm，统一尺度总览/局部图未见可辨认收拢 | 已获得GT精度提升，或接受减少就等于排除错误回环 |
| Corridor02几何修正版08-001 | 已恢复关闭、导图及审查；2930组原生前端输出与旧基线完全一致，304关键帧每图1527256点；最大位置改动3.89cm，未见可辨认收拢 | 晚停使所有历史状态完全不变、ICP通过等于真实回环、已确认走廊退化 |
| 几何修正真实配对对照 | 103组×2种位姿×旧/新＝412次ICP；37组通过→拒绝、58组仍通过、末段8组仍拒绝 | 37组全部是假回环、58组全部是真回环 |
| 初值诊断 | 12组×4初值＝48次；补入SC yaw未让末段通过 | yaw已接入实际后端（仍未接入） |
| 对应距离对照 | 12组×150/5/2/1m＝48次；较小门限改善末段局部对齐，原全点规则仍拒绝末段 | 已选出生产最优门限或修复整图 |
| 验收审查 | 复用上述48次，新增16张双相机帧及28条跨帧检查；低分/多内点不足以验明回环 | 已部署联合接受器 |
| RGB-D支线 | 九帧图像/深度解码、标记对应检查完成；无CameraInfo，未估计米制位姿 | 相机已融合进SLAM或独立真值已建立 |

几何修正版的完整Corridor01同输入对照现已完成。实现行为及保存地图发生变化，但未显示可辨认的重影改善；末段回访仍未通过ICP。证据见[效果审查](tools/scpgo_geometry_effect_audit_20261107/README.md)。两组仍有退出异常及PCL警告，不能称完全正常退出。

证据入口：[参数化](tools/scpgo_parameterization_20261106/README.md)、[几何修正](tools/scpgo_geometry_fix_20261106/README.md)、[初值诊断](tools/scpgo_terminal_diagnosis_20261106/README.md)、[距离对照](tools/scpgo_distance_ablation_20261107/README.md)、[验收审查](tools/scpgo_acceptance_audit_20261107/README.md)、[暂停的RGB-D输入检查](tools/scpgo_rgbd_reference_20261107/README.md)。历史经过统一见主复盘。

## 3. 当前Git与环境身份

- 实际仓库：WSL /home/lw/slam-research/fast_lio_ws/src/M3DGR。
- 远端：https://github.com/Walker-LW/Fast-LIO2-SC-PGO ，只保存当前工程、SC-PGO及部署记录。
- main与baseline/fastlio2-before-scpgo：4cd7b8a151be7fec7260935213042636a271cc03。
- integration/scpgo已拆分归档：参数化与运行工具提交`a2ae9e1`；几何修正提交`e96b9f7`，与实际已验证源码字节一致。研究文档及复现工具随后独立提交，完整HEAD以Git日志为准。SC-PGO位于该分支，main中没有它。
- SC-PGO路径：baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO；包aloam_velodyne，程序alaserPGO。
- 部署记录：仓库deployment-scpgo；/home/lw/slam-research/deployment-scpgo为其符号链接。
- 原镜像slam-research-fastlio:noetic及baseline-pre-scpgo-20261105：sha256:fcfce4546a744e9e76b02e79485ac60f40cc54522362cc8ad7654871c98c2f7f。
- 派生镜像slam-research-fastlio-scpgo:noetic-gtsam403-v1：sha256:f306a0ec73450db1aa499376f84c0aa56d1046069ceb435087bd46b54e7e6db5。
- 原程序：/ws/devel/lib/fast_lio/fastlio_mapping；后端：/ws/devel_scpgo/lib/aloam_velodyne/alaserPGO。

原M3DGR提交e0cf7d5仅为上游来源。旧历史仍在本地upstream引用与独立备份，禁止用push --all/--tags将其全部推到新远端。编译产物、镜像、数据和实验结果不纳入Git。当前未创建PR或合并main。

Windows D:/projects/fast-lio2-learning保存文档、参考源码和辅助工具，不是容器运行的算法工作空间。WSL /home/lw/slam-research/下fast_lio_ws对应/ws，datasets对应只读输入，results保存结果。

## 4. 当前结论、下一任务与边界

### 实际代码与配置

- 前端仍为LiDAR＋IMU的FAST-LIO2，当前基线blind=1。原前端、镜像及实验目录保留。
- SC-PGO已完成38项参数暴露及用户批准的几何修正；`local2global`固定16线程遗漏已记录，尚未修。
- 五项批准值：SC半径40m、关键帧1m/10°、SC距离阈值0.2、地图显示体素0.2m。
- 实际SC/ICP体素仍0.4m、最大对应距离仍150m、全点接受门限仍0.3m²。5/2/1m仅为离线对照值；SC yaw仍未进入ICP初值；没有新范围过滤、多尺度、当前侧多帧或新接受器。
- 几何修正版程序：`/ws/devel_scpgo_geometry/lib/aloam_velodyne/alaserPGO`，SHA256 `03d2e316a408a1906a9613757794af4888512b34b2aae5683d7aab17d4b3c9c6`。需要该devel环境；旧程序仍在`devel_scpgo`、`devel_scpgo_params`，不能混用。

### 回到两项核心问题

1. 前端为什么产生误差：累计漂移、局部错配/退化、输入或运动补偿等原因尚未完整分清，不能把“走廊”直接等同已证明退化。
2. 后端为什么未纠正：已确认实现几何错误并修正；150m对本批近场配准过宽、默认评分范围与受限优化对应不同；候选真实性仍存在问题。

### 已完成整图对照与下一项建议

用户已手动完成`scpgo-corridor01-geometry-ab-20261107-001`的old/fixed；助手只读审查及离线补图。两组关键帧、时间、原始位姿、配置及优化前地图相同；异步候选集合不同。ICP通过108→70次（按FIFO日志推导94→60组不同配对），共有226组中36组通过→拒绝、2组拒绝→通过。末段(0,382)各34次完整结果均拒绝，fitness约830→620m²仍远超0.3m²。

最大位置改变量old/fixed为6.89/1.79cm，最大旋转改变量0.03757/0.01368°；局部距离≤40m点最大移动7.79/1.96cm。总览和四处局部图未见可辨认收拢；没有GT精度结论或完整优化器因子清单。两组保存文件冻结检查通过，但均保留44次PCL警告和退出异常。详细结果、逐帧CSV及统一尺度图位于Windows `tools/scpgo_geometry_effect_audit_20261107/verified/`。

下一建议调整：冻结Corridor01现有失败结果，将Corridor02受控跨序列验证列为近期任务，同时准备只记录的前端几何方向诊断。不要继续把全部问题限定在Corridor01后端调参，也不要将场景名直接当退化证明。Corridor02已只读核实2930组世界系点云/位姿精确配对，但无已录原生body点云；转换适用性、实际回访与共同几何先核实，未启动实验。前端误差成因与后端拒绝/约束作用分别研究，方案见[下一步审查](tools/scpgo_geometry_effect_audit_20261107/NEXT_STEPS.md)。新增记录模块、输入转换、生产规则及数值均尚未实施。

用户随后明确本轮从原始Corridor02完整重跑，并确认沿用40m、1m/10°、SC0.2、显示0.2m及ICP150m/0.3m²。已准备独立手动入口[完整流程](tools/scpgo_corridor02_geometry_20261107/README.md)：新前端采集原生body/odom与几何修正版后端，增加完整性/事件/位姿及对应地图点改变量、轨迹和同尺度总览/四象限图。已通过真实环境静态解析及合成录包到报告的检查，正式目录`scpgo-corridor02-geometry-20261107-001`尚未由助手创建，未播放真实数据。无新雅可比模块，仍无ATE/RPE/退化因果结论；前端诊断保留后续独立任务。本轮不转换旧世界点云。

2026-11-08执行入口纠正：工具版本目录仍为`tools/scpgo_corridor02_geometry_20261107`，本次实验ID改由`--id scpgo-corridor02-geometry-20261108-001`明确指定。准备后的脚本读取自身manifest，不能将工具目录日期也手动改为08。07/08结果目录均在本次只读检查时未创建，实际08预检通过；不代表正式运行完成。以完整流程当前命令为准。

随后实际运行状态：08-001已完整接收2933 LiDAR/59655 IMU、2930组body/odom，前端已正常退出、保存304关键帧；finish因rosnode info对unknown node返回0被误判而中断。后端/录包/keep-alive在诊断时仍运行。工具已改用注册名称检查，专用recover_finish.py复用原检查记录并保留冻结副本，已交用户从B终端手动恢复；正式关闭、导图和效果分析待核实，不能标记完成或声称改善。

最新完成状态：用户已恢复关闭并导图，助手完成[只读晚停/效果核查](tools/scpgo_corridor02_effect_audit_20261108/README.md)。额外等待388.48s，尾段399次ICP均拒绝、无新增通过；3877份已发布303节点路径完全相同，40份发布地图XYZI也相同。最后第303号节点没有早期完整快照，不能证明其全部历史。前后完整304关键帧图未见可辨认收拢，最大位置改动3.89cm，近40m点最大移动4.63cm；无GT精度或退化结论。原日志交错导致的解析缺口已在独立派生证据恢复，不覆盖原结果。下一步不因晚停自动重跑，应分开前端约束诊断、可信回访/接受与图作用证据。

### 暂缓事项

相机位姿参考暂停，不扩展视觉融合；Corridor02跨序列验证不再要求先把Corridor01调好；intersection_s/tunnel_s仅元数据检查完成，尚未跑SLAM。多尺度、多帧、0.2m体素和优化噪声不同时加入。近期交付是可解释、可复现的前端/后端问题案例及适用边界。

## 5. 历史证据入口

以下相对WSL /home/lw/slam-research/results/，均为已占用目录，禁止覆盖：

| 路径 | 用途 |
|---|---|
| baseline/corridor02-full-b1-20261102-001 | Corridor02完整blind=1，2930条位姿/点云，最终PCD15442992点 |
| baseline/corridor01-full-b1-20261103-001 | Corridor01完整blind=1，4034条位姿/点云，原生与导出XYZI一致 |
| baseline/corridor01-full-b4-20261105-001 | Corridor01完整blind=4，严重发散，保留失败结果 |
| baseline/corridor01-blind-ab-20261105-001 | blind参数对照审计 |
| validation/scpgo-baseline-import-20261105-001 | Git基线、源码导入与旧Git元数据备份 |
| validation/scpgo-image-20261105-001/verification-002 | 派生依赖镜像验收 |
| validation/scpgo-build-20261105-001 | 后端独立编译与原产物保护验收 |
| validation/scpgo-runtime-20261105-001 | 第5步状态核对与四种候选配置静态检查 |
| validation/scpgo-runtime-20261105-002 | Corridor01专用环境准备后原产物保护复核 |
| scpgo/scpgo-corridor01-20261105-001 | 接入后完整运行、录包、264关键帧及双地图；后端退出异常与PCL警告保留 |
| scpgo/scpgo-corridor01-20261106-002 | 批准五项配置的完整运行，383关键帧及双地图/主体预览；几何修正之前版本 |
| scpgo/scpgo-corridor01-geometry-ab-20261107-001 | 用户完成同输入old/fixed整图对照；383相同关键帧，未见可辨认重影改善；退出/PCL异常保留 |
| scpgo/scpgo-riverside02-20261106-001 | 整段运行、23分卷、5951关键帧及双地图/并排预览；正式RESULT.md和checks完整核查 |
| validation/scpgo-geometry-fix-20261106-001 | 几何修正前源码/程序身份备份及独立构建、离线结果审计 |
| validation/scpgo-run-review-20261105-001 | 完整运行后只读核验：录包、配对、点云数值、地图哈希、位姿变化及退出异常 |

## 6. 执行与维护约定

- 先核对实际环境；未知关键条件询问用户，不补造状态。
- 新实验使用独立容器和未占用结果目录，不复用旧Dynamic01输出挂载。
- 播放控制按当次用户授权；Riverside02已授权助手完成。不得把计划、编译通过或结构检查写成真实回环有效。
- 用户2026-11-06要求：后续所有测试交付优化前后并排二维预览及路径。同一可比帧集/点集、共同坐标系、相同XY范围/比例，叠加轨迹与起点，不额外对齐；记录来源/校验值、点数及抽样元数据。变化小时仍出总览，可补局部放大。失败或缺可比输出须记录缺图原因与待补状态，不伪造另一侧或标记完成。详细规则见序列记录与路线第7节；本轮未增加自动出图代码。
- 源码/算法改动先说明证据、必要性及最小范围；已批准几何修正保留，其余变更单独审查。
- 交接只更新当前摘要；复盘只记录实际处理与关键纠正；路线只同步阶段状态，不逐轮堆叠聊天记录。
