# Corridor01 全序列：逐步操作与解释

2026-11-05最新任务：用户选择先补Corridor01的blind=4完整对照。已准备WSL档案corridor01-full-b4-20261105-001，未启动回放；本轮使用 [BLIND4.md](BLIND4.md)，不要重跑prepare或套下方旧001的resume。保持上一轮在线PCD=true等配置，估计参数仅blind 1→4；准备脚本现在支持ID中的b1/b4，ready按output-policy和参考运行时配置核验。

## 当前采用：复用001（用户明确要求）

用户要求保留001为正式实验目录，不创建002。已确认此前未播放，raw/derived/PCD为空、所有ROS进程已结束；仅容器驻留进程还在。助手将旧配置、脚本、日志完整备份到001/preplay-backup-20261103-001，退休旧一次性启动标记，更新ready与在线PCD配置。不得对有真实输出的运行自动使用此清理流程。

本次不要再次执行prepare或start（start会创建重名容器）。每终端仍设置001的RUN。终端1先 `bash "$RUN/run.sh" resume` 核对哈希并复用现有容器，再 `bash "$RUN/run.sh" master`；其余按init、record-odom、record-cloud、algorithm、rviz、watch、ready、play顺序操作。在线PCD预先配置true、interval=-1；ready无提前/tf要求。结束后增加compare-pcd，比较原生与导出PCD。此前建议002由用户选择覆盖，不再使用002流程。

## 2026-11-03 检查器修正与在线PCD对照

001档案在预播放检查失败，原因是旧ready.py错误要求此时存在/tf发布者。实际C++在首次publish_odometry内创建静态TF广播器，输入未播放前没有发布者是正常行为。新ready已移除此项提前要求，仍检查Odometry/cloud/path及记录器，/tf在播放后和输出中检查。旧档案不改写，不删除once标记。

本次用户要求同时保存算法原生PCD与bag导出PCD。已检查001的launch、runtime快照和实时参数均为pcd_save_en=false，尚未生效。此参数仅在算法启动时读入，启动后rosparam set不能可靠改变节点内部开关。采用新ID `corridor01-full-b1-20261103-002`，准备命令：

```bash
bash /mnt/d/projects/fast-lio2-learning/tools/corridor01_full/prepare.sh corridor01-full-b1-20261103-002 --online-pcd
```

--online-pcd明确设置原生保存true、interval=-1；output-policy.json留档，ready按该策略检查。须先由用户停止旧001失败容器再启动002，保留旧档案。001记录器/观察器/RViz已停止，不能继续套原RUN或重跑被锁定动作。

其余步骤沿用下文，但所有RUN和停止容器名改为002。在线原生保存会累计内存并增加退出写盘开销，需要status关注内存。它没有改变估计方程，却改变运行负载，不能与之前关闭在线PCD的运行当作性能等条件对比。算法正常退出后确认PCD/scans.pcd实际写出，再结束记录。

完成convert、verify、export-pcd后执行 `bash "$RUN/run.sh" compare-pcd`。它逐点按顺序比较PCD/scans.pcd与derived/final_registered.pcd的XYZ/intensity，报告checks/native-vs-exported-pcd.json；额外normal/curvature字段单独保留，不拿它们假定有效法向。MATCH_XYZI只表示两种保存途径共有字段一致，不等于地图准确；若不同保留原件分析，不用ICP把差异对齐掉。随后downsample和trajectory照常执行。

修正与合成测试日志static-check-20261103-002.log；真实双路径对照尚未运行。下方001和PCD关闭描述为初版流程，采用本段002及开启策略覆盖这些条件。

2026-11-03。工具已准备和检查，尚未创建正式实验目录或启动回放。全部操作在 WSL Ubuntu-24.04 执行；Windows 仓库只是工具入口。不要复用 Corridor02 的冻结脚本。

## 已核查的输入与方案

实际输入 `/home/lw/slam-research/datasets/Corridor01.bag`，6860418551字节；完整bag时间跨度403.886228秒。选用 `/livox/avia/lidar`（livox_ros_driver/CustomMsg，4038条）和 `/livox/avia/imu`（sensor_msgs/Imu，82126条）。已解码全部所选消息，header严格递增，最大间隔约0.100665秒/0.005111秒。不等于完成全点数值、点内时间、标定与同步验证。

本地输入SHA-256 `c6200a1860ed7c8e66a6bbdb464771bd7de7b154f79812d79cc226801b8e1566`，只标识文件，未与官方可信哈希对照。`input-audit-20261103-001.json/.log`保存核查证据。

工作空间、镜像与Corridor02一致；源码提交e0cf7d59c9a5a3df515624034698d976abc26549，干净；镜像sha256:fcfce4546a744e9e76b02e79485ac60f40cc54522362cc8ad7654871c98c2f7f。历史容器已停止。D盘核查时约148GiB空闲，WSL约6.9GiB可用内存；此为检查时状态。

本轮沿用Avia、blind=1、1倍速、同一估计参数，目的为跨序列对照，不声称原版blind=4配置复现或最优参数。起始抽查的三取一/line/tag候选中，blind=1约800个而blind=4约20个；这是距离候选统计，不是实际匹配数。RViz开启；在线PCD关闭；原始点云bag留档后导出完整与5cm副本，以及独立轨迹PLY。PCD是累计配准扫描，不是ikd-Tree快照。

新ID `corridor01-full-b1-20261103-001` 已查未占用。已有目录/容器或一次性操作标记时拒绝重用。失败保留档案，不删除锁或覆盖结果。不要直接换ID重复跑而不检查错误。

## 1. 准备实验（必需、仅一次）

```bash
bash /mnt/d/projects/fast-lio2-learning/tools/corridor01_full/prepare.sh corridor01-full-b1-20261103-001
```

作用：创建独立档案，复制本次工具、YAML/launch、GT参考，保存版本/哈希与输入核查。目的：避免覆盖并固定实验条件。成功显示Prepared与No experiment has started。它不启动容器或算法。报错时保留输出并停止。GT仅归档，未执行精度评价。

每个新WSL终端先设置：

```bash
RUN=/home/lw/slam-research/results/baseline/corridor01-full-b1-20261103-001
```

作用：定义本终端的路径简称，不启动程序。下面 `bash "$RUN/run.sh" 动作` 调用档案内冻结的入口，只有指定动作才执行对应功能；Windows工具后续修改不会自动改变副本。

## 2. 终端1：容器与ROS master（必需）

```bash
bash "$RUN/run.sh" start
bash "$RUN/run.sh" master
```

start：核对工具、源码、可执行文件与输入哈希；拒绝同时运行共享工作空间容器；创建独立实验容器，工作空间/输入只读，本次结果可写，Log/PCD挂载到本次目录，接入WSLg/GPU。network none仅容器内部ROS通信，不能在此联网安装工具。成功返回容器ID并保存container.json/input-bag.txt/gpu.txt；失败不要继续master。

master：启动节点注册和参数服务，使同容器内ROS进程相互发现。它不处理雷达数据，保持本终端运行。确认roscore就绪后继续。

## 3. 终端2：时间与轨迹记录（必需）

```bash
bash "$RUN/run.sh" init
bash "$RUN/run.sh" record-odom
```

init把use_sim_time设true，必须先确定时间再启动记录器和算法；应输出true。record-odom直接运行C++记录器，保存Odometry/TF/path/clock到raw/odometry.bag。PCD不能替代原始轨迹。开始没有clock时等待是预期行为，播放后才应出现Recording to，不切回墙钟。

## 4. 终端3：配准点云记录（必需）

```bash
bash "$RUN/run.sh" record-cloud
```

订阅/cloud_registered，写raw/cloud*.bag，约1GiB分卷，不自动删旧卷。作用：保留逐帧时间关系，支持导出地图和追查异常。开始未播放时等待；播放后应开始写入。缓冲区/磁盘丢包报错须记录时间并反馈。

## 5. 终端4：算法（必需）

```bash
bash "$RUN/run.sh" algorithm
```

读取full.launch并启动FAST-LIO2，尚未播放时等待输入。blind=1、path开启、在线PCD关闭；同一基线算法，未增加退化处理。输出和警告写checks/algorithm-console.log及ros_logs。启动报错或进程死亡时停止后续流程。

## 6. 终端5：RViz（本轮方案开启）

```bash
bash "$RUN/run.sh" rviz
```

显示camera_init下最近10秒点云、path和body轴；目的为在线观察，独立于估计。path约每10次调用发布，不是完整轨迹。画面空白在没有输入时正常，播放后核查显示状态；显式节点名用于启动检查。截图与异常输入时间保存在screenshots/。10秒点云在EOF时钟持续推进后会消失，不代表文件丢失。

## 7. 终端6：覆盖观察器（本轮必需）

```bash
bash "$RUN/run.sh" watch
```

watch.py独立订阅输入和Odometry，每墙钟秒打印数量、首末时间与多久没收到消息，写checks/live-progress.log。等待Observer ready。其接收数量是旁证，不证明算法内部逐条无损接收，也不代表有效匹配。

## 8. 终端7：检查后手动播放（必需）

```bash
bash "$RUN/run.sh" ready
```

ready.py检查真实发布/订阅关系、节点和参数，保存runtime.yaml与ready-console.log。应PASS并生成ready.ok。失败不要继续play。检查通过后：

```bash
bash "$RUN/run.sh" play
```

仅播放Avia LiDAR/IMU，1倍速、初始暂停、clock 100Hz、keep-alive，未设置截断/循环。按空格才开始。目的：从头完整输入；播放后确认两记录器Recording to、观察计数上升、RViz有点云/路径。总时长以播放器所选话题显示为准，约404秒，不在固定秒数手动提前停。

## 9. 运行期间：负载与异常（建议做）

另开终端8，先设置RUN，再执行：

```bash
bash "$RUN/run.sh" status
```

保存当时CPU/内存/节点和Linux磁盘状态；可重复执行，每次新时间名。Windows还需看D盘实际空间，Linux df不是物理剩余空间。零匹配、跳变等记录输入时间，不立即调参抹去现象；进程死亡、持续丢包或资源不足时保留现场并中止本次，标记不完整。

## 10. 完整输入结束后的检查（必需）

保持播放器运行且不暂停。终端8：

```bash
bash "$RUN/run.sh" end-status
```

end_status.py只读观察日志，检查4038条雷达/82126条IMU已收到、存在里程计且输出稳定至少10秒、观察器日志仍新鲜。未达到时按提示等待或检查，可重复执行。结合播放器进度达到末尾、算法没有崩溃、末输出时间接近输入尾部判断；这不是GT或完整跟踪证明。

keep-alive会让Duration超过总时长，只继续clock，不重复发送数据。无需等Done，不因超过总时长而认为自己操作错误；也不要无限等待。

## 11. 正常收尾（顺序必需）

1. 终端4 Ctrl+C一次，保持播放器运行；等待算法正常结束。目的是让尚需仿真时钟的退出过程完成。
2. 算法结束后，终端2 Ctrl+C，等待轨迹记录器关闭并完成info检查。
3. 终端3 Ctrl+C，等待所有点云分卷关闭并完成info检查。
4. 终端6 Ctrl+C停止观察器；终端5关闭RViz或Ctrl+C。
5. 最后终端7 Ctrl+C停止播放器。master和容器暂时保留。

出现SIGTERM升级、进程死亡、无法正常关闭时，不反复强制终止或重命名.active；保存当时输出并反馈。正常关闭的bag和actions.log是收尾依据。

## 12. 轨迹转换与文件验收（必需）

终端7设置RUN后逐条执行，成功才继续：

```bash
bash "$RUN/run.sh" convert
```

convert.py将Odometry header和位姿原样导出derived/trajectory.tum，检查有限值/时间/四元数/坐标标签，不做修正。PASS只表示这些检查通过；REVIEW_REQUIRED或其他错误先停下检查，不覆盖重跑。

```bash
bash "$RUN/run.sh" verify
```

verify.py顺序读取输入与输出、核对TUM/点云时间、有限值、首尾覆盖、原始哈希和部分警告，生成checks/full-run.json。全包读取可能耗时数分钟。STRUCTURE_OK_REVIEW_COVERAGE仍要求人工复核首尾缺口、警告、退出码、TF数量和地图，不等于精度正常或不存在退化。原始帧数不能直接要求等于里程计帧数。

## 13. 导出地图与轨迹（本次交付必需）

```bash
bash "$RUN/run.sh" export-pcd
```

export_pcd.py从完整点云bag拼接XYZ/intensity，输出derived/final_registered.pcd和检查报告；不降采样、不优化；不用再找PCD/目录（在线保存已关闭）。

```bash
bash "$RUN/run.sh" downsample
```

downsample_pcd.py按5cm体素每格保留一个原始点，输出derived/final_registered_voxel_0p05m_first.pcd，保留完整文件并校验源哈希、坐标和文件长度。只降低查看数据量，不改变估计结果，不保证最小欧氏距离5cm。

```bash
bash "$RUN/run.sh" trajectory
```

trajectory.py从TUM导出derived/trajectory-view/trajectory_time_rgb.ply、start_green.ply、end_red.ply、CSV及报告；按时间蓝到红，起终点独立标记，同为camera_init。PLY为位置点序列，不是连接折线，XYZ经读回核对；不要将轨迹混入地图或对两者另做ICP。Z和首末距离不直接视为真实高度/闭环误差。

以上动作拒绝已有产物；失败时保留现场并反馈，不删除锁重做。检查通过后输出才用于地图观察。

## 14. 停止环境（完成处理后）

终端1 Ctrl+C停止master，再执行：

```bash
docker stop corridor01-full-b1-20261103-001-container
```

只停止本次容器，结果与容器保留。静态CloudCompare查看不需要容器或ROS继续运行。

## 15. Windows CloudCompare观察（本次目视检查必需）

文件资源管理器打开：

```text
\\wsl.localhost\Ubuntu-24.04\home\lw\slam-research\results\baseline\corridor01-full-b1-20261103-001\derived
```

加载5cm PCD，再加载trajectory-view中的轨迹PLY及起终点。必要时选中轨迹调大显示点尺寸，用RGB显示；完整PCD用于可疑局部对照。俯视/侧视都看，保存同一视角和尺度截图。无需安装新容器查看工具。记录结构连续性、重影/缺口位置及末段轨迹，不凭场景名或图形断言退化。

## 输出与验证边界

raw保存原始记录，derived保存TUM/完整与5cm PCD/轨迹查看文件，checks保存操作/结构/数值报告，ros_logs与Log保存运行证据，config保存参数和版本。暂不运行GT精度评价；现有首末参考的参考点、方向和端点含义待核实，不将脚本混合RMSE冒充ATE。也不自动把结果归因于Z轴bug。

已验证：输入离线核查、Bash语法、实际ROS镜像Python语法与launch参数、RViz YAML，以及合成点云/轨迹的导出/5cm抽样/防覆盖。新脚本的真实Corridor01端到端行为仍待本次手动执行确认；未改算法核心或安装依赖。
