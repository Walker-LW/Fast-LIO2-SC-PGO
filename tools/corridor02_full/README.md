# Corridor02 全序列手动运行方案

**安装步骤纠正（2026-11-02）：实验容器以 `--network none` 创建，不能直接在其中联网执行 apt-get。此前要求在该容器中直接安装 pcl-tools 的步骤有误，停止沿用。已发生的软件源域名解析失败与该网络隔离配置一致，并非 PCD 或算法错误。后续应准备具有联网能力的独立查看容器，只读挂载本次结果及所需图形接口；先验证联网，再安装并查看。保留原实验容器、失败日志和全部结果，不通过重建原实验容器解决。**

## 命令和脚本分别做什么（2026-11-02 补充）

以下是流程解释，不是要求对已完成实验重复执行。`RUN=...` 仅在当前 WSL shell 定义路径变量；`bash "$RUN/run.sh" 动作` 运行本次档案内冻结的脚本副本。Windows 工具修改不会自动更新冻结副本。

| 命令/脚本 | 做什么、目的和必要性 | 输出或成功依据 |
|---|---|---|
| prepare.sh | 创建独立目录，复制配置/工具，固定版本和哈希；每次新实验必需，已有目录拒绝覆盖 | Prepared；不启动算法或容器 |
| run.sh start | 创建带图形和本次结果挂载的独立容器；供后续 ROS 进程使用 | 容器运行，挂载与 GPU 信息留档 |
| master | 启动 ROS 注册和参数服务；本次运行必需，不进行估计 | roscore 持续运行 |
| init | 将 ROS 时间设为仿真时间；必须在节点/记录启动前确定 | use_sim_time=true |
| record-odom | 订阅并原样保存 Odometry/TF/path/clock；保存完整轨迹证据 | raw/odometry.bag 正常关闭 |
| record-cloud | 保存世界系配准点云，达到大小门限分卷；本次地图留档必需 | raw/cloud*.bag |
| algorithm | 用 full.launch 启动实际 FAST-LIO2；输入到来才进行处理 | 算法节点、日志和输出话题 |
| rviz | 在线显示点云/轨迹；帮助观察，不参与估计 | 图形窗口与渲染日志 |
| watch / watch.py | 独立订阅输入和里程计并记录数量/首末时间；为覆盖检查提供旁证 | live-progress.log；不证明算法内部逐条接收 |
| ready | 检查节点、话题和关键参数，保存快照；播放前检查 | ready.ok；失败不继续播放 |
| play | 播放原始 Avia LiDAR/IMU，发布时钟；空格才开始，keep-alive 在 EOF 后只继续时钟 | 播放器日志；超过总时长不代表又播放了数据 |
| status | 保存单个时刻的资源和节点快照；可选，不是全程性能测量 | 带时间文件名的 status 日志 |
| convert / convert.py | 从已关闭 bag 的 Odometry header/位姿生成 TUM，结构检查，不修正轨迹 | trajectory.tum、trajectory.json；已有产物拒绝覆盖 |
| verify / verify.py | 检查输入/输出数量时间、TUM 对应、点云有限值、哈希和部分日志 | full-run.json；成功不等于精度正常 |
| export-pcd / export_pcd.py | 从点云 bag 拼接完整 XYZ/intensity PCD，不降采样或校正 | final_registered.pcd、final-pcd.json |
| pcl_viewer | 打开已保存 PCD；无需 ROS master，不运行 FAST-LIO2 | 静态点云窗口；不能独立证明退化 |

本次实验已完成采集和转换，不应重跑上述一次性动作。后续只需准备查看工具、查看已有 PCD，再分析异常区间。

本轮补充：最终必须保留可查看的整段点云。完成 convert、verify 后，容器停止前执行 `bash "$RUN/run.sh" export-pcd`，生成 `derived/final_registered.pcd` 和 `checks/final-pcd.json`。逐帧合并 /cloud_registered 的 XYZ/intensity，不降采样、不修正位姿；保留分卷 bag 以追查异常时间段。当前容器没有 pcl_viewer，查看前需由用户执行容器内 `apt-get update`、`apt-get install -y pcl-tools`，并用 `command -v pcl_viewer` 确认可用。安装后在图形容器执行 `pcl_viewer` 加 PCD 绝对路径。安装依赖会改变容器状态，应在采集结束后进行并保存安装日志。PCD 是累积配准扫描，目视异常需与分时段点云、轨迹和输入对应；确认几何退化仍需几何约束/观测信息诊断，不能仅凭最终图认定。

2026-11-02。状态：方案与工具准备；尚未运行全序列，尚未验收本工具的实际采集和退出行为。

## 已核查与已选择

实际工作空间为 WSL `/home/lw/slam-research/fast_lio_ws`。Ubuntu 24.04 / WSL2、Docker Engine 29.9.2 可用；检查时历史容器均已停止。源码提交 e0cf7d59c9a5a3df515624034698d976abc26549，工作树干净。镜像固定为 sha256:fcfce4546a744e9e76b02e79485ac60f40cc54522362cc8ad7654871c98c2f7f。镜像内 Ubuntu 20.04、rosbag 1.17.4、RViz 1.14.26；临时诊断容器的 glxinfo 显示 RTX 4060 D3D12、Accelerated: yes。

检查时 WSL 可用内存约 7 GiB，D 盘空闲约 160 GiB。Linux df 的 912 GiB 不能当作 Windows 实际可写容量。核查日志为同目录 environment-audit-20261102-003.log；001/002 为诊断命令失败记录，保留。没有启动算法或数据回放。

用户已选择点云 bag、关闭在线 PCD、1 倍速。blind=1 为已诊断的预处理调整，不能称为最优值。其余估计参数沿用现有基线；开启 path 供显示。输出配置和可视化、观察器带来的负载均应披露，不用本次结果直接宣称实时性提升。

## 输出与显示

| 项目 | 本次方案 |
|---|---|
| 新 ID | corridor02-full-b1-20261102-001；准备时原子创建目录，已存在则拒绝 |
| 结果根目录 | /home/lw/slam-research/results/baseline/上述 ID |
| 轨迹 | raw/odometry.bag：/Odometry、/tf、/path、/clock |
| 点云 | raw/cloud*.bag：/cloud_registered；约 1 GiB 分卷，不自动删除旧分卷 |
| PCD | 在线关闭；需要时另行离线导出，保留原始 bag |
| RViz | camera_init；10 秒配准点云、/path、body 坐标轴；20 FPS |
| 完整轨迹 | derived/trajectory.tum；使用 Odometry header 时间 |
| 日志 | checks、ros_logs、Log；screenshots 用于人工保存画面 |

/path 在此源码中每 10 次调用发布一次，不能替代完整 /Odometry。配准点云是世界系扫描集合，不等同于内部 ikd-Tree 地图快照。10 秒显示窗口用于控制内存；完整场景留在点云 bag 中。默认不显示 /Laser_map，因为当前源码没有调用其发布函数。回放末尾时钟继续推进后，10 秒点云会逐渐消失，这是显示窗口行为。

## 手动启动

以下全部在 WSL Ubuntu-24.04 终端执行。先阅读全篇，再决定启动。不要执行旧 P0 README 的历史命令。准备失败也保留目录，使用新的序号。

```bash
bash /mnt/d/projects/fast-lio2-learning/tools/corridor02_full/prepare.sh corridor02-full-b1-20261102-001
```

准备只创建新档案，不启动容器。每个终端先执行：

```bash
RUN=/home/lw/slam-research/results/baseline/corridor02-full-b1-20261102-001
```

依次操作：

| 终端 | 命令与等待条件 |
|---|---|
| 1 | `bash "$RUN/run.sh" start`，成功后 `bash "$RUN/run.sh" master`，等待 master 就绪 |
| 2 | `bash "$RUN/run.sh" init`，确认 true，再 `bash "$RUN/run.sh" record-odom` |
| 3 | `bash "$RUN/run.sh" record-cloud` |
| 4 | `bash "$RUN/run.sh" algorithm` |
| 5 | `bash "$RUN/run.sh" rviz` |
| 6 | `bash "$RUN/run.sh" watch`，等待 Observer ready |
| 7 | `bash "$RUN/run.sh" ready`，全部成功后 `bash "$RUN/run.sh" play` |

容器仅挂载本次结果目录；工作空间和数据只读，Log/PCD 定向挂载本次目录。沿用已核查的 WSLg X11、/dev/dxg 和 WSL 图形库，不修改原 Compose。所有 ROS 进程位于同一隔离容器。

播放器初始暂停。确认参数为 blind=1.0、path=true、pcd=false、use_sim_time=true；节点检查通过。检查 /Odometry 的订阅者有 record_pose，/cloud_registered 的订阅者有 record_cloud 与 RViz。随后在终端 7 按空格开始。仿真时间在记录器启动前设置，因此起始无 /clock 时记录器可能等待；不要为消除等待擅自切换墙钟。

开始后确认两个记录器出现 Recording to，RViz 点云、轨迹正常，观察器计数增加。另开终端可运行 `bash "$RUN/run.sh" status` 保存资源快照，并在 Windows 查看 D 盘空间。若持续积压、丢包告警、内存/磁盘不足或异常跳变，保留日志和当时输入时间；如中止，此次标为不完整，调整条件后换新 ID。

## 完整结束与收尾

播放器使用 `--clock --hz 100 --keep-alive -r 1`，只播放 Avia LiDAR/IMU，不设置截断或循环。Noetic 的 [player.cpp doKeepAlive](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/player.cpp) 在输入结束后继续推进时钟；不能等待普通模式的 Done 后再操作。本地已验证选项存在，实际完整退出仍待本次验证。

1. 观察播放器进度达到输入末尾；观察器应收到 2933 条 LiDAR、59655 条 IMU。它证明观察器接收，不能单独证明算法接收无损。
2. 保持播放器运行、不暂停，等待 Odometry 计数和末时间稳定至少 10 秒，并核对末时间接近输入尾段。10 秒是操作等待条件，不是完整性证明；若仍在持续追赶，继续等待。
3. 在终端 4 按一次 Ctrl+C，等待算法正常退出。不要先停时钟或记录器。若升级 SIGTERM、退出卡住或报错，保留现场并反馈，不宣称正常完成。
4. 算法退出后，在终端 2、3 分别 Ctrl+C，等待两个记录器关闭文件并完成 info 检查。不能只看命令返回；检查没有 .active，分卷均可读。
5. 停止观察器（终端 6）和 RViz（终端 5），最后 Ctrl+C 停止播放器（终端 7）。
6. 保持 master/容器运行，执行：

```bash
bash "$RUN/run.sh" convert
bash "$RUN/run.sh" verify
```

若转换报告 REVIEW_REQUIRED，先保留并查看问题；已有转换产物可供 verify 核查，不重跑覆盖。verify 顺序读取输入所选话题、全部输出轨迹及点云，可能需要一段时间。检查逐行 TUM 一致性、点云 XYZ 有限值、点云/位姿时间对应、首尾覆盖、日志警告和原始 bag 哈希。

7. 验收后停止 master（终端 1），再执行：

```bash
docker stop corridor02-full-b1-20261102-001-container
```

保留容器和全部档案。不删除 once 标记重用同一实验。

## 验收边界与末帧

已读输入尾部：最后 LiDAR header 为 1737472315.453933477，最大原始点偏移 0.099824165 秒；最后 IMU header 为 1737472315.549438238。原始扫描末点比末 IMU 晚约 4.319 ms。源码同步等待 IMU 覆盖过滤后的扫描结束时间，因此最后一帧是否可处理需结合实际输出和过滤后末点复核；不能伪造 IMU 或调整轨迹时间来补齐。时钟推进不会产生新的 IMU 测量。

验收需同时检查：播放器输入结束、算法正常退出、记录正常关闭、覆盖缺口可解释、警告与跳变按全序列报告、地图目视记录。初始化通常无输出，不能要求输出数量等于原始雷达帧数；也不能把任何数量差异都归因于初始化。/clock 包含收尾时钟，轨迹覆盖应看 /Odometry 的 header，不能看整个输出 bag 时长。

STRUCTURE_OK_REVIEW_COVERAGE 仅为结构检查结果。核实起始/末尾缺口、中间间隔、输入观察器计数与实时负载后才能记录完整运行状态。本次没有连续 GT 精度验收；GT 参考点、方向和端点含义继续待核实。不得从平滑轨迹或地图截图推出 ATE/RPE 提升。

## 本轮验证范围

环境检查、图形加速和 rosbag 选项已实查；Bash 语法和实际 ROS 镜像内 Python 语法、launch 展开/参数覆盖、RViz YAML 静态解析已通过，见 static-check-20261102-001.log。此工具未做真实数据端到端运行，实际显示、录包和正常退出须由上述手动运行确认。任何失败都保留原件，不静默修复或筛除异常。
