# Corridor02原始数据完整重跑：几何修正版与固定参数

2026-11-07。用户已确认沿用上轮正式配置。新入口已通过真实环境程序身份/launch展开/参数检查，以及合成录包→导图→量化/出图→防覆盖检查；尚未准备正式结果目录或运行真实数据。所有正式阶段由用户手动执行。

2026-11-08路径纠正：工具目录`scpgo_corridor02_geometry_20261107`是实际存在的工具版本目录，不能随当天日期手动改名。本次实验编号单独用`--id`明确指定为`scpgo-corridor02-geometry-20261108-001`。准备后的副本自动读取冻结manifest中的编号，run.sh读取并核对自身目录名；本页命令已统一为这一组合。先前07编号入口保留兼容，但下文不混用。

## 实验范围与固定值

原始`/datasets/Corridor02.bag`的LiDAR＋IMU→原FAST-LIO2→几何修正SC-PGO，完整1倍速。比较本次相同关键帧/点集的PGO前后，不将旧前端运行当成严格相同输入对照。旧基线2930条输出只作覆盖参考，不能强行补齐/删除成该数量。新关键帧数由本轮实际输出确定，不套用Corridor01的383帧。

| 项目 | 固定值 |
|---|---:|
| 前端blind | 1 m |
| SC半径 | 40 m |
| 关键帧平移/旋转间隔 | 1 m / 10° |
| SC距离阈值 | 0.2 |
| 地图显示体素 | 0.2 m |
| SC/ICP体素 | 0.4 / 0.4 m |
| ICP最大对应距离 | 150 m |
| ICP接受门限 | 0.3 m² |

其它值逐字复制批准的corridor01-indoor.yaml，启动时冻结。程序为`devel_scpgo_geometry`，不是旧`devel_scpgo`。前端估计参数沿用Corridor02 blind=1基线；在线累计PCD关闭，与该基线一致，记录原生body点云后离线导出地图。无RViz、安装或重新编译步骤；最终用PNG查看。本轮没有增加前端雅可比/特征值模块，不能据此确认几何退化。

新结果目录（本次只读检查时未占用）：

`/home/lw/slam-research/results/scpgo/scpgo-corridor02-geometry-20261108-001`

输入、工作空间只读挂载；Log/PCD分别挂载到本次目录。任何阶段出错就停止并保留日志，不能删除标记、重复backend或复用旧结果路径。后端启动有清理自身Scans的行为，所以设置单次启动保护。

## 逐步手动执行

### 1. 两个终端进入WSL，分别设置变量（必需）

若当前是PowerShell，先执行：

```powershell
wsl.exe -d Ubuntu-24.04 -u lw
```

两个WSL终端都执行：

```bash
C2TOOLS=/mnt/d/projects/fast-lio2-learning/tools/scpgo_corridor02_geometry_20261107
C2ID=scpgo-corridor02-geometry-20261108-001
C2RUN=/home/lw/slam-research/results/scpgo/$C2ID
```

终端A负责启动和播放，B负责观察和收尾。变量只影响当前终端，不启动任何程序。

### 2. 终端A预检与准备（必需）

```bash
ls -l "$C2TOOLS/manage.py"
python3 "$C2TOOLS/manage.py" preflight --id "$C2ID"
```

检查镜像、原前端/几何修正版哈希、批准配置、原包存在性和脚本语法；存在其它运行容器时停止，先核对干扰。预期`PREFLIGHT_PASS`。

```bash
python3 "$C2TOOLS/manage.py" prepare --id "$C2ID"
```

只创建新输出目录，复制冻结脚本和配置，读取bag元数据，展开launch，计算输入校验值；不启动ROS、估计器或播放。约5GB原包哈希会读取磁盘。预检必须打印08实验编号及同一输出路径，准备预期`PREPARED_NOT_STARTED`。目录或同名容器已存在时拒绝，失败准备保留现场，不能直接重跑。后续从`C2RUN`内调用manage.py时不需要再传ID，它自动读取本次manifest并拒绝不同编号，防止跨目录操作。

### 3. 终端A启动主服务（必需）

```bash
bash "$C2RUN/run.sh" master
```

创建独立容器、启动roscore、设仿真时间、检查真实程序解析及配置校验；预期`MASTER_AND_BINARY_CHECK_PASS`。还没有LiDAR/IMU输入。失败看`checks/master-console.log`、`startup-validation.log`，不继续。

### 4. 终端A依次启动前端、后端、记录（必需，逐条执行）

```bash
bash "$C2RUN/run.sh" frontend
```

启动原前端，预期`FRONTEND_READY`；等待输入，不开始播放。

```bash
bash "$C2RUN/run.sh" backend
```

启动几何修正版，预期`BACKEND_READY`。不得在同一目录重启后端。

```bash
bash "$C2RUN/run.sh" record
```

启动分卷录包和外部观察器，保存body/odom、PGO输出、调试点云、TF与clock。原始LiDAR/IMU由观察器计数，原包不重复录一份。预期`RECORDER_AND_WATCH_READY`。

### 5. 终端A检查播放前状态（必需）

```bash
bash "$C2RUN/run.sh" ready
```

要求前端发布者、后端/记录订阅者、实际参数与展开配置一致；检查没有提前播放及GPS发布。预期`PREPLAY_GRAPH_AND_PARAMETERS_PASS`。播放前不要求TF已有数据，计数为空正常。

```bash
bash "$C2RUN/run.sh" status
```

打印节点、实际后端参数、容器CPU/内存当前采样和最新计数。status是查看步骤，可重复；资源单次采样不是峰值或性能基准。

### 6. 终端A完整播放（必需）

```bash
bash "$C2RUN/run.sh" play
```

播放器初始暂停，出现提示后按一次空格。完整从头1倍播放，只发原始LiDAR/IMU并生成clock，不播放相机/GPS。不添加duration、不跳段、不循环；原包约293秒，即约4分53秒，不含启动暂停与收尾。

播放终端不要输入其它命令或按Ctrl+C。`--keep-alive`使原消息结束后继续推进时钟，便于前后端退出，因此终端A不会在尾部自动返回，也不需要等待`Done.`。计时超过原包时长不代表原始传感器消息重复播放。[ROS Noetic播放器实现](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/player.cpp)的keep-alive循环位于Done输出之前，并继续推进clock。

### 7. 终端B观察，到尾部及时收尾（必需）

```bash
tail -n 1 "$C2RUN/checks/interface-observer.jsonl"
```

可重复查看；最终原始输入应为LiDAR2933、IMU59655（准备时还会从实际bag元数据核对），odom/body应配对、errors为空。播放过程中短暂计数差可能是回调时差，以最终检查为准。

进入尾部且原始计数达到总数后，保持A运行，在B执行下一步；不要长时间空等，因为后端会重复检测最后一帧，影响事件计数。真实末段等待时间及原始事件保留在报告中。

### 8. 终端B检查并关闭（必需）

```bash
python3 "$C2RUN/manage.py" finish
```

先核对完整原始输入接收、前端配对以及末段输出已到达最后原始扫描的时间范围，再停前端；防止原始消息播完但前端尚未处理到末尾就被关闭。随后给后端短暂处理余量，检查所有关键帧与位姿文件、保存哈希，停后端/观察器/录包，确认没有active包，最后停播放器与容器。A将随播放器关闭而返回。若末段尚未到达，检查在停止节点之前报错，先保留时钟观察输出是否继续推进，不绕过此检查。

需要依次出现`INPUT_COMPLETE_PASS`、`SAVED_KEYFRAMES_AND_POSES_PASS`、`CLOSED_AND_FROZEN_PASS`。固定等待不等于内部队列消费证明，后续报告仍列待处理候选和退出异常。任何检查失败停止后续流程，不为通过而改计数、跳检查或强杀进程；保存日志并报告失败步骤。已完成finish不能重跑。

### 9. 导出相同关键帧的完整地图（必需）

```bash
python3 "$C2RUN/manage.py" export
```

读取同一批完整关键帧，分别按原始/优化位姿拼图，不降采样、不额外配准。输出`derived/keyframe-maps/before_pgo.pcd`、`after_pgo.pcd`、`report.json`。预期`EXPORT_STRUCTURE_PASS`。不覆盖已有导出目录；PCD用于完整留档，不需要安装pcl_viewer。

### 10. 生成量化报告、轨迹和统一尺度预览（必需）

```bash
python3 "$C2RUN/manage.py" analyze
```

逐条读取本次录包核对时间、坐标系、有限数值与配对，再核验地图源文件和校验值。输出`derived/quantitative/`，预期`QUANTITATIVE_REPORT_AND_PREVIEWS_PASS`。该步骤是离线分析，不运行新ICP、优化或播放；不覆盖已有目录。

### 11. 打开结果（查看必需，无新增安装）

```bash
explorer.exe "$(wslpath -w "$C2RUN/derived/quantitative")"
```

先看`before-after.png`，再看四个固定窗口`quadrant-details.png`与`pose-changes.png`，最后读`report.json`。总览与局部图使用相同帧集/点集、坐标范围、采样规则，叠加相应轨迹和起终点，不进行额外对齐。局部窗口为总览的四象限，不能替代明确墙面的精确对应评价；需要时再指定真实重影区域补图。

## 量化输出及解释边界

| 文件/指标 | 用途 | 不能说明 |
|---|---|---|
| report.json输入接收、覆盖时间、配对、数值有效性 | 判断实验是否完整可分析 | 精度合格或每条输入都用于估计 |
| frontend-full.tum | 原生完整前端轨迹，保留整数纳秒时间戳 | 全帧最终PGO轨迹 |
| before/after-keyframes.tum | 同批关键帧原始/最终优化轨迹 | 完整高频PGO位姿或GT；时间继承保存文件精度 |
| keyframe-changes.csv与pose-changes.png | 平移米/旋转度，逐帧及中位数/P95/P99/最大值 | ATE/RPE或精度提升 |
| 对应地图点移动，全点及原始局部距离≤40m分层 | 量化地图实际发生多大变化 | 移动方向正确、墙面更准 |
| icp-events.csv及候选/接受/拒绝/唯一配对 | 检查后端实际判定、重复和未处理候选 | 回环真实性或完整优化器因子清单 |
| 前端零有效点/跳帧调用、PCL/退出异常 | 定位需复查窗口和运行问题 | 警告次数等于退化帧数 |
| 首末位置间距、轨迹步长与路程 | 描述轨迹结构和异常变化 | 有真值的闭合误差或绝对精度 |

`GTCorridor02.txt`只有一组3×3旋转、3×1平移和bag_time，非逐时刻GT。其参考点/方向/端点尚未核实，不输出ATE/RPE或提升百分比。现有ArUco_evaluate.py将旋转矩阵差与平移差混合为一个RMSE，不能将其直接当作标准轨迹ATE，本轮不调用该指标。

本轮是单序列一次完整效果验证，不评价前端退化因果、跨重复显著性或算法计算加速。若仅发生很小变化或完全无回环，也照常保存报告和图，不挑选有利结果。
