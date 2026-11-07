# Corridor01 后端参数化（2026-11-06）

> 当前状态补充（2026-11-07）：下文说明参数化当时的范围；之后另有几何修正提交e96b9f7及已完成的old/fixed整图对照，未见可辨认收拢。生产数值仍与批准配置一致。


用户授权的五项配置保存在 [corridor01-indoor.yaml](corridor01-indoor.yaml)：半径40m、关键帧1m/10°、SC阈值0.2、显示体素0.2m。其它38项保持导入版本默认值，列于同一YAML及parameter-defaults.json。参数仅在启动时读取，使用`/scpgo`命名空间；修改YAML后需重新启动新后端，不支持运行中热更新。

实际源码位于WSL `/home/lw/slam-research/fast_lio_ws/src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO`。新增`include/aloam_velodyne/BackendParameters.h`；修改主CPP与Scancontext.h/.cpp。新增参数覆盖独立SC/ICP体素、子图帧范围、ICP对应距离/迭代/停止/RANSAC/接受值、SC高度/维度/排除/候选/重建/检索、图噪声及鲁棒核尺度、iSAM、频率、队列与线程。分环角度与宽度随维度和半径重新计算。

参数化没有修正子图root、相对变换方向、配对线程或退出实现；原版风险继续保留。未增加最大输入量程滤波，因为这会改变行为而非仅暴露现有常量。`map_skip_frames=2`也只影响发布图的选帧，不是保存关键帧或完整离线地图的选帧。

新构建目录为`/ws/build_scpgo_params`与`/ws/devel_scpgo_params`。原前端及`devel_scpgo`程序不替换。WSL deployment-scpgo/runtime 中新增：

- `corridor01-indoor.yaml`：本轮批准配置；全部其它值显式固定。
- `backend-parameterized.launch`：明确选择新程序，GPS重映射继续隔离，参数与节点同属`/scpgo`。
- `build-parameterized.sh`：固定已有镜像ID、离线独立构建。

旧`backend-defaults.yaml`、`backend.launch`和旧prepare入口保留原版配置/程序校验规则，**不使用旧入口启动本轮新配置**。历史实验目录内的脚本和配置不修改。新配置完整回放尚未准备或执行；需使用新独立输出目录及新后端入口。默认启动逻辑仍包含上游Scans清理，因此输出目录必须全新且不含历史文件。

`validate_config_only=true`会加载并校验参数、打印有效值后退出，发生在输出文件打开/目录清理及工作线程启动之前。check_config.py使用此路径检验批准配置、所有原默认值、参数覆盖、非法值拒绝以及实际launch程序解析；不发布LiDAR/IMU，不测试地图精度。

## 其它参数的审查建议（尚未实施）

1. **ICP体素0.4→0.2m：建议下一轮优先做离线单因素试验。**末段源点仅31个；末帧body网格占用0.4/0.2m为48/121，近场结构可能被压缩。它不证明0.2m必然修复；需要检查实际PCL输出、覆盖、误配和运算量。SC体素先保留0.4，避免一起变化。
2. **最大对应距离150m：建议收紧，但具体值暂不定。**末段多为近场几何，150m允许很远点对；应根据可信配对初始错位/对应距离提出米数后再审核。太小可能找不到足够对应；评分范围与迭代门限独立。
3. **子图±25、排除最近30、SC重建30次：先保留并实测。**关键帧从2m改1m后对应实际时间/空间范围变化；先报告实际范围与可信候选可见性，再提新值。
4. **高度2m、图噪声和Cauchy尺度、SC/ICP接受阈值、迭代停止条件：保持。**当前没有足够依据选替代值。高度须核对body原点和地面分布，权重须先核实约束正确。

上述均为建议，不代表已批准更改。用户批准五项同时组成一个新场景配置，后续与旧配置比较可评价组合效果，不能把差异单独归因于半径或关键帧间隔。

原源码备份、构建日志、配置校验记录和受保护程序哈希位于WSL `results/validation/scpgo-parameterization-20261106-001`；本目录保留修改前source、修改后modified及差异便于审阅。
