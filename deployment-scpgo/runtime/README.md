# SC-PGO 第五步：运行准备

> 当前状态补充（2026-11-07）：此处原版001步骤为历史。参数化、几何修正及完整整图对照已经完成；最新总结见项目根目录交接文档，不能用本页旧入口直接启动新配置。


2026-11-05。保留当前 FAST-LIO2 与 ROS1 原版 SC-PGO。本目录是 Windows 可查看副本；实际部署位置为 WSL `/home/lw/slam-research/deployment-scpgo/runtime`。算法源码没有修改。

## 当前状态与使用边界

001已由用户完成Corridor01全序列1倍速运行、录包及双地图导出，容器停止；4034组输入配对、264关键帧和两图各1391336点核验通过。用户目视无明显改善；最大位置变化约2.26cm，无GT收益结论。后端退出-6和26次PCL索引溢出警告保留。

主cpp与在线固定上游10eab538全文LF归一一致，公共root/取逆不是克隆引入。实现风险、走廊/有限FoV参数适配及权重主因未定；当前先用已有数据核对可信回访及失败层，再选择单因素半径验证或几何单测，不默认改源码。资料、证据和优先级见[诊断记录](verification/effect-diagnosis.md)（Windows副本）；实际部署的当前交接见项目文档。

首次前端节点查找失败已通过按进程devel顺序解决，无须重编译；失败日志及标记保留，细节见统一复盘和001/checks/node-lookup-20261105-001。原镜像、源码和编译产物未改。

**以下命令保留为已执行的001操作说明，不是当前待办。不要再运行001的start、retry、算法或player，也不要删除单次标记。新实验须先明确验证问题、独立ID和配置。**

## 用户已确定的配置

1. Corridor01，单独准备。
2. 沿用最近一次 blind=1 完整实验留档配置，前端原生累计PCD开启、interval=-1。
3. 先使用后端源码默认值完成链路验证，随后按证据调参。数值及来源见 [有效参数与接口](effective-parameters.md)，不是 Avia 走廊调优结论。

正式结果目录：`/home/lw/slam-research/results/scpgo/scpgo-corridor01-20261105-001`。容器名：`scpgo-corridor01-20261105-001-container`。准备动作只创建容器，不启动；实时状态通过 `bash "$RUN/run.sh" status` 查看。

`baseline` 对应的原生PCD策略：Corridor01=true、Corridor02=false，均 blind=1。`no-online-pcd` 明确关闭前端累计PCD；后端保存关键帧及位姿不受此开关影响。配置检查会与该序列留档 runtime.yaml 比较，拒绝未经列明的参数差异。

## 确定配置后的准备方式

在 WSL 运行 `python3 /home/lw/slam-research/deployment-scpgo/runtime/prepare.py --help` 查看必填项。全部必填，没有自动选择序列或PCD策略。

| 项目 | 值 |
|---|---|
| `--sequence` | `Corridor01` 或 `Corridor02` |
| `--frontend-policy` | `baseline` 或 `no-online-pcd` |
| `--backend-profile` | 用户确认后可选 `source-defaults` |
| `--id` | 与序列一致的未占用 `scpgo-corridor01-YYYYMMDD-NNN` 或 `scpgo-corridor02-YYYYMMDD-NNN` |

prepare 会原子创建 `results/scpgo/ID/`，冻结工具、输入SHA-256、参数、源码和程序校验值，然后 `docker create` 专用容器，保持 `created`、未启动状态。任何已占用目录或容器名都拒绝复用。失败保留现场，不删除检查标记或重用失败档案。

容器按派生镜像完整 ID 固定：`sha256:f306a0ec73450db1aa499376f84c0aa56d1046069ceb435087bd46b54e7e6db5`。工作空间 `/ws` 和 `/datasets` 只读，只有本次结果目录及其前端 Log/PCD 子挂载可写；镜像根文件系统只读，临时文件放内存目录。网络 `none`，ROS 仅容器内部通信。本工具不安装软件，不附带 RViz 或图形挂载；前后端可视化话题仍发布并留档。

结果目录：

| 路径 | 内容 |
|---|---|
| `config/` | 参数、参考配置、输入哈希、镜像身份、挂载及容器身份 |
| `backend/Scans/` | 原版后端保存的 body 系关键帧PCD |
| `backend/odom_poses.txt`、`optimized_poses.txt`、`times.txt` | 原版关键帧位姿与时间 |
| `raw/` | 接口消息分卷bag，保留里程计、body点云、后端输出、TF和clock |
| `checks/` | 启动保护、控制台、播放前图与参数检查、外部观察器 |
| `Log/`、`PCD/`、`ros_logs/` | 本次前端与ROS输出 |
| `derived/` | 保留给后续外部导出，当前为空 |

## 手动启动接口（准备成功后）

所有启动动作都由用户选择执行。将 `RUN` 设为 prepare 实际输出的完整目录，每个新WSL终端都设置一次。`bash "$RUN/run.sh" status` 仅查看状态；其余启动动作限定一次。

本次已经准备完成，不要重跑prepare。每个WSL终端先执行：

```bash
RUN=/home/lw/slam-research/results/scpgo/scpgo-corridor01-20261105-001
```

1. 终端1依次执行 `bash "$RUN/run.sh" start`、`bash "$RUN/run.sh" master`；master保持运行。
2. 新档案使用修正后的模板执行 `bash "$RUN/run.sh" frontend`；本次001已有失败标记，必须使用上文retry_frontend_lookup.sh。它使用前端进程专用查找顺序；前端参数未改，仍等待输入。
3. 终端3执行 `bash "$RUN/run.sh" backend`；检查 backend 目录完全为空、配置和源码校验值匹配，再启动原版后端。首次启动生成 Scans；禁止重启此后端到同一路径。
4. 终端4执行 `bash "$RUN/run.sh" record`，保留实际接口消息；无输入时等待 clock 是预期行为。
5. 终端5执行 `bash "$RUN/run.sh" watch`，等待 Observer ready。
6. 终端6执行 `bash "$RUN/run.sh" ready`；只检查实际图、类型和ROS有效参数，不要求播放前存在TF或有效点云。

`PREPLAY_GRAPH_PASS` 仅证明播放前连线与配置正确。消息内容、配对、覆盖、回环及效果仍未验证。每次 ready 都保存独立快照。

## 数据回放与退出边界

**当前不执行播放，也未选择本次范围和速度。**`manual_play.sh`仅在用户手动调用时执行，先重查ready，再交互要求选择full/short、短片段时长和倍速，没有默认选择。固定从起点播放Avia两路输入并发布clock，播放器初始暂停、末尾keep-alive，单次启动且保存选择和控制台记录。冻结run.sh未修改。用户启动完成后调用：`bash /home/lw/slam-research/deployment-scpgo/runtime/manual_play.sh "$RUN"`。

外部观察器每秒记录输入计数、首末消息时间、Odometry与body点云的整数纳秒精确配对、未配对数量、点云字段布局和clock相对header延迟。运行中未配对可能只是消息先后到达，须在停止输入且输出稳定后再判断。它不证明后端内部队列没有积压或内部逐条无损；原版内部配对未改。PGO会重发最近关键帧，其重复时间戳不记作输入错误。

原版后端有 `while(1)` 工作线程，且 main 中线程对象没有 join/detach；静态代码不足以将真实退出记为成功。手动停止时保留前端/后端分开的控制台和退出码，记录任何异常终止或SIGTERM升级；不把异常收尾写成正常保存，也不通过重启同一路径补救。先停止前端且保持播放器clock推进，检查后端输出稳定，再停止后端、记录器、观察器；最后停止播放器和master。真实退出行为如阻塞，先保留证据再审核最小源码修改。

后台每轮iSAM更新会重写原始/优化位姿文件，外部拷贝需在写入稳定后进行；不能默认退出时保存过所有缓冲数据。原版没有独立回环因子日志：SC候选、ICP通过日志、实际入图与优化改善分开验收。在线稀疏地图不是最终完整导出，后续应按同组关键帧和同一规则重建优化前后地图。

## 外部地图对照工具（用户结束采集后执行）

新增 `export_keyframe_maps.py`，不修改原算法。要求原/优化位姿行数、时间行数和从000000连续编号的Scans完全一致；不默默截短。不删除近点、不降采样、不额外对齐，保留XYZ/intensity，按同组关键帧导出before_pgo.pcd和after_pgo.pcd。输出目录已存在即拒绝覆盖。严格计数检查失败时保留文件并分析，不能通过删帧凑齐。

原版上游makeMergedMap.py硬编码目录和[0,200]范围，并默认移除2米内点，不能直接作为本次全量公平对照入口。新增工具已通过合成旋转/平移、填充PCD字段、intensity保留、输入不变、防覆盖和数量错配拒绝测试；证据为 `results/validation/scpgo-runtime-20261105-003/synthetic/report.json`。没有执行真实数据导出。

确认前后端已停止、bag正常关闭后，保留容器驻留进程，执行：

```bash
docker exec scpgo-corridor01-20261105-001-container /entrypoint-scpgo.sh python3 \
  /ws/src/M3DGR/deployment-scpgo/runtime/export_keyframe_maps.py \
  /run/scpgo /run/scpgo/derived/keyframe-maps
```

成功为EXPORT_STRUCTURE_PASS，只表示导出结构通过。用CloudCompare分别显示两份地图，保持同一视角和尺度；不同时叠加后判断重影，也不对优化前后地图另做ICP。原前端PCD/scans.pcd不随PGO重排。回环正确性和精度改善仍需结合日志、轨迹与适用参考判断。
