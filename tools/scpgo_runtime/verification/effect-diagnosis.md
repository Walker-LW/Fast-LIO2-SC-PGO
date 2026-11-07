# SC-PGO无明显改善：证据、上游资料与诊断顺序

2026-11-05复核。修正前轮“先形成源码修正方案”的过早排序；本轮仅核对来源、检索资料和整理记录，未改算法、参数或重新运行实验。

**2026-11-06 最新状态：**Riverside02 完整播放、前后端运行、审计和双地图预览已完成，主要功能链接通，二维预览有明显校正；全部回环正确与 GT 精度提升尚未验收，退出 -6/PCL 警告保留。实际结果见[序列记录](../../mulran_riverside02/README.md)。本文以下保留 Corridor01 的局部证据与条件分支，不能据另一序列成功确定本序列无改善的根因；未开展新的 Corridor01 测试或参数/源码改动。

**回访与图像纠正：**多圈不是有效回环的必要条件，估计轨迹首尾未相交不能否定实际回访。Riverside02 参考首尾相距约 561.072 米，有内部回访；Corridor01 原始图像支持返回起始区域，但 Avia 共同覆盖及可唯一匹配性仍未验证。“只走一圈所以无效”不是已证实原因。Corridor01 已有同组 264 关键帧双 PCD，各 1391336 点；当时未生成 Riverside02 式并排 PNG，是展示步骤遗漏，本轮未补图。用户现要求后续所有测试交付同尺度并排二维预览，具体规则见[测试要求](../../mulran_riverside02/README.md)。两次运行均保留 `LIDAR_HEIGHT=2.0`，其含义及未改动边界见[有效参数](../effective-parameters.md)。

## 已确认与尚未确认

- **输入适用性优先于后端诊断：**官方论文将该类LiDAR退化采集描述为绕行返回起点，退化测试与物理回访并不互斥。现已实际读取本地原包两路相机的固定时刻画面：开始与约395/403秒可见同一标定板、立柱及周围布置，支持返回起始区域；不能再说本包是否回访完全未知。但这只确认地点回访，不自动证明Avia三维重叠足够、每个SC候选正确或完整包版本已对齐。[论文采集说明](https://arxiv.org/html/2507.08364v1#S3.SS4)
- **数据身份待核实：**用户确认OneDrive来源；重新计算本地SHA-256为c6200a18…b8e1566，当前[官方checksums](https://raw.githubusercontent.com/sjtuyinjie/M3DGR/main/checksums.txt)列172f63a8…43da3a，二者不同。官方[#32](https://github.com/sjtuyinjie/M3DGR/issues/32)确认过阿里云盘旧版尾段不全，但不能套用到本OneDrive文件；首尾画面实际回到标定区域。哈希差异仅确认字节版本不同，不直接证明缺尾/损坏，也不擅自重新下载替换。此前“完整回放”仅对本地文件内部计数成立，不能称官方版本完整性已通过。
- **回访附近的实际几何信息有限：**首尾关键帧0/263保存840/1259点，仅占55/48个0.4m body网格；该占用不等于实际PCL输出或独立约束数。末尾已记录ICP源为31点。固定首尾原始Avia抽样的非零点主要在约2～3m、未出现1m内非零点，不支持本次blind=1把该样本近点删掉的解释。走廊中段控制样本有效覆盖更多；不能用几个样本代表全包。
- **走廊ICP误匹配是待验证分支：**重复墙面/门洞可能使错误位置的局部几何残差也小；收敛和fitness只表明当前配对的数值拟合，不能替代物理地点与重叠验证。[PCL配准说明](https://pointclouds.org/documentation/group__registration.html)、[重复几何研究](https://www.roboticsproceedings.org/rss18/p015.html)。输入画面与原始点统计见input-inspection.json、overlap-input-statistics.json，原始证据目录results/validation/corridor01-input-suitability-20261105-001。
- 001全序列运行、录包和导出完成；用户目视无明显改善。4034组位姿/body时间配对，264关键帧，两图各1391336点，源/输出哈希通过。关键帧原始XYZI与body消息逐点相同，没有额外体素降采样；稀疏来自选帧。最大关键帧位置变化约2.26cm，无GT精度改善结论。
- 118次ICP通过对应58组不同配对。三样本离线复算复现得分，但原位姿不是GT，与前端估计冲突不等于约束错误；正确闭环也可能纠正漂移里程计。5000是单项基础信息权重比，离线Cauchy估计不是在线求解器权重，不能据此直接确定主因。
- 子图公共root与ICP逆变换仍是几何约定需要验证的实现风险；没有修正后对照，不能声称它们已被证明是本次无改善的唯一或主要原因。后端退出-6和26条PCL索引溢出警告另行保留。

## 是否为克隆问题

实际主cpp与[上游固定提交10eab538](https://github.com/gisbi-kim/FAST_LIO_SLAM/blob/10eab538db70678f41aaa24247d16831ca921374/SC-PGO/src/laserPosegraphOptimization.cpp)全文LF归一后完全一致，SHA-256同为606c2fb6c7d06ab839d04b6aa7d1e3af6f0910c40865c759558f0193363c28bf；源目录相对原样导入无差异。公共root及poseFrom.between(poseTo)均来自原版，没有证据指向克隆损坏。证据：[在线核查摘要](upstream-web-review.json)。

上游相同不能证明实现必然正确；公开issue同样不能证明本项目根因或提供可直接照搬的修复：

| 原始资料 | 能支持什么 |
|---|---|
| FAST_LIO_SLAM [#19](https://github.com/gisbi-kim/FAST_LIO_SLAM/issues/19) | 用户报告Avia室内走廊多次检测回环但未对齐，说明相似现象并非本地独有；不是解决方案 |
| FAST_LIO_SLAM [#40](https://github.com/gisbi-kim/FAST_LIO_SLAM/issues/40)、SC-A-LOAM [#12](https://github.com/gisbi-kim/SC-A-LOAM/issues/12) | 对公共root子图变换提出质疑；维护者确认及本场景因果仍不能由此推定 |
| FAST_LIO_SLAM [#31](https://github.com/gisbi-kim/FAST_LIO_SLAM/issues/31) | 相对约束方向已有公开讨论，不把“取逆”本身当成错误证明 |
| SC-A-LOAM [#7](https://github.com/gisbi-kim/SC-A-LOAM/issues/7) | 旧版讨论的是keyNear索引，不能因issue已关闭就推断当前公共root问题已解决 |
| SC-A-LOAM [#28](https://github.com/gisbi-kim/SC-A-LOAM/issues/28) | 用户报告噪声模型影响效果；不采纳其去鲁棒核、极强权重作为已验证方案 |

访问限制：GitHub API评论403限流，以上按可读取公开页面记录，未核实完整评论，也未找到可据本次资料直接确认的维护者修复结论。WSL直连重置后由Windows读取固定原文件并成功全文比对，没有改网络/镜像。

## 走廊与参数：需要核查，但不一起改

[原作者README](https://github.com/gisbi-kim/SC-A-LOAM#indoor)明确建议室内使用较小sc_max_radius；[上游launch](https://github.com/gisbi-kim/FAST_LIO_SLAM/blob/main/SC-PGO/launch/fastlio_ouster64.launch)注释列20/40m为室内参考、80m为室外示例。它们是候选依据，不是Avia走廊最优值。

[Avia官方规格](https://www.livoxtech.com/avia/specs)表明有限FoV；[SOLiD作者论文页面](https://sparolab.github.io/research/solid/)说明有限FoV会影响地点识别。走廊结构相似、视向/共同可见区域差异可能影响描述子和ICP，但实际覆盖与真假配对尚未核验，不能仅凭“走廊”认定退化或必须调到某值。

| 项目 | 当前值/性质 | 核查次序 |
|---|---|---|
| sc_max_radius | 80m，ROS有效参数 | 先看有效点/描述子占用及可信回访分数；需要时仅离线比较80/40/20 |
| sc_dist_thres | 0.2，ROS有效参数 | 待真回访/相似非回访分数有区分依据再选，不因已有118次通过就排除漏检 |
| 关键帧间隔 | 2m/10°，ROS有效参数 | 与源码排除最近30帧联动，不能只解释成地图稀疏；先固定 |
| ICP/体素/图权重 | 门限0.3、内部体素0.4m、噪声及鲁棒核均含源码常量 | 不填无效YAML；几何/质量未验证前不放宽门限或放大权重 |
| 前端blind | 1m | 维持已验证设置，不能把先前blind的因果结论迁移到后端参数 |

与blind对照相同的方法是“先统计并定位失效环节，再单因素验证”，而不是“场景是走廊，所以同步修改一组参数”。

## Corridor01 后续条件分支（本轮未执行）

1. 优先核对OneDrive本地文件与官方版本差异，不覆盖现有包和结果；不能仅因哈希不同就判缺尾。以实际首尾图像已确认的回访区域为起点，选择Avia共同视域/几何覆盖充分的帧对及相似非回访对；不要只挑对着标定板的静止帧，不能用漂移里程计距离或轮速轨迹代替真值。
2. 对固定配对表一次性核对SC分数/有效覆盖、ICP输出、相对测量与残差，定位失败层。停止无关扩展核查。
3. 真回访漏检才走半径单因素离线分支；可信配对的几何测量失败才走已知变换单测及最小修改审核分支；约束可信而优化仍弱才研究实际图权重。一次只选一个分支，不同时改代码和参数。

退出/PCL问题单列，影响配对诊断或保存时提高优先级；不把“图变动更大”当更正确。不新回放、不动原源码/镜像/编译产物，001不复用。

原始证据：WSL results/validation/scpgo-run-review-20261105-001和scpgo-effect-diagnosis-20261105-001；逐事件/三样本数据仍保留在本目录effect-inputs.json、effect-icp-probe.json、effect-loop-events.csv。图3完整文件路径待用户确认。
