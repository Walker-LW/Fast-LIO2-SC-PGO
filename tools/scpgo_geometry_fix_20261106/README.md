# SC-PGO 几何修正与验证产物

用户批准后已修正实际 WSL 源码：按各历史帧相对位姿拼接到历史根帧坐标，当前帧保持自身局部坐标，ICP 输出直接作为历史→当前相对位姿测量。子地图构建先在同一锁内复制位姿快照；调试发布使用对应根位姿变换出的世界坐标副本。数值参数、SC 检索、ICP 初值、图噪声和退出逻辑未改。

## 构建与入口

- 源码：`/home/lw/slam-research/fast_lio_ws/src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO/src/laserPosegraphOptimization.cpp`。
- 新程序：`/home/lw/slam-research/fast_lio_ws/devel_scpgo_geometry/lib/aloam_velodyne/alaserPGO`。
- SHA256：`03d2e316a408a1906a9613757794af4888512b34b2aae5683d7aab17d4b3c9c6`。
- 镜像及批准的 `corridor01-indoor.yaml` 不变。需加载 `/ws/devel_scpgo_geometry/setup.bash`；旧环境仍会解析到旧程序。现有 `backend-parameterized.launch` 在新环境下的配置校验已通过。
- 原前端、原后端、参数化后端均未覆盖；备份在 WSL `results/validation/scpgo-geometry-fix-20261106-001`。旧运行目录不可用作新启动输出目录。
- 调试主题仍叫 `/loop_scan_local`、`/loop_submap_local`，现在发布正确标注为 `camera_init` 的世界坐标显示副本；内部 ICP 输入仍分别位于当前/历史机体坐标。旧脚本不能直接将这两份显示副本当作原始 ICP 输入复算得分。

## 验证结果

六个原定几何用例全部通过：最大平移误差 5.20e-7 米、旋转误差 1.20e-5 度，三帧子地图 RMS 2.88e-7 米。真实程序编译及批准配置、38项默认参数、launch 解析检查通过；无完整 SLAM 回放。

真实离线对照固定选取全部 95 组曾通过候选及末段 8 组，共 103 组；分别用保存的原始里程计和旧最终优化位姿，执行旧版/修正版四种组合，共 412 次 ICP。两种位姿快照的判定一致：37 组通过→拒绝，58 组通过→通过，8 组末段拒绝→拒绝。

以原始里程计快照为主对照：首个通过候选 `(0,32)` 得分 0.2653→0.9440；大里程计间距候选 `(147,310)` 0.0432→0.4054；最终 `(0,382)` 829.8936→619.9608。阈值仍为 0.3 平方米，三者修正后均拒绝。

仍接受的58组与保存里程计相对位姿存在差异：平移差中位数46.88米，旋转差中位数89.40度。里程计不是真值，不能据此直接给错误回环标签，但不能将58次通过当作可信闭环验收。

本轮证明了几何实现修正，不证明 Corridor01 重影已解决。保存的位姿有文本舍入且不是各次在线位姿快照；最终优化位姿含旧后端反馈，故只作敏感性检查。下一步应核实实际回访重叠和初值，再决定后续单因素测试及完整回放。

## 文件

- `geometry-fix.patch`、`laserPosegraphOptimization.cpp`：最小变更及已安装源码副本。
- `synthetic/`：原六用例、提取函数、测试程序、日志及结果。
- `real/manifest.json`：固定候选选择、输入哈希、限制；其余 `real/` 文件为四组原始输出与可执行提取程序。
- `results-summary.json`：全部逐配对结果；`final-report.json`：编译、参数、输入不变性及最终审计。
- `build.sh`、`check_backend.sh`：独立构建及配置核查入口。`install.py` 仅接受旧源码哈希，不能在已安装后重复执行。

仍保留 `local2global` 固定16线程的参数化遗漏；本轮只修正获批准的几何范围。持续复盘见项目主复盘文档。
