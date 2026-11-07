# Corridor01 blind=4 全序列对照（2026-11-05）

最新简化（用户要求）：watch改为可选，本次跳过终端6的watch，也跳过依赖其日志的end-status。当前已准备的b4 ready检查器已同步修正，不强制要求coverage_observer。旧脚本/哈希归档于checks/watch-optional-change-20261105-001。其余节点、录包、参数与参考配置检查保留。播放后确认记录开始；播放器到输入末尾后保持keep-alive运行，留约10秒处理余量，再先停算法、后停记录器、最后停播放器。10秒只是操作余量，不证明处理完整；结束后verify必须核对首尾覆盖、数量及日志。未保存独立实时接收计数，不能声称有该项证据。本次与旧b1的旁路监测负载不同，不用于严格性能比较。下文watch/end-status的必需表述由本段覆盖。

用户选择先补Corridor01，参考为corridor01-full-b1-20261103-001。新档案corridor01-full-b4-20261105-001；已有blind=1原件不得覆盖。此次不是上次无输入失败准备的复用场景，两条件必须分别留档。

## 固定条件与问题

同一Corridor01.bag、Avia、固定提交/镜像、1倍速、RViz开启、path与稠密点云发布开启、在线PCD=true/interval=-1、同时保存轨迹和配准点云bag；估计参数仅blind 1→4。原生PCD与bag导出对照仍可做，但不是本次主问题。目标为完整地图和轨迹的参数对照，不预设blind=4一定失败或把过滤问题当成几何退化。

prepare按ID中的b4写参数，记录output-policy.json并复制参考运行时参数。ready检查实际节点/发布订阅和blind=4，且比较参考的common/mapping/publish/pcd_save等配置组；checks/reference-config-comparison.json应MATCH_EXCEPT_BLIND。不要求播放前存在/tf发布者。单次运行不能证明统计稳定性或性能收益。

## 当前准备状态

工具/静态launch、在线PCD、无TF预播放语义、参考配置对照、合成文件导出与防覆盖已验证，日志static-check-20261105-001.log。未修改旧实验或核心算法。准备脚本仅创建文件，用户决定并手动回放。具体是否已准备以本次prepare输出为准；成功后不要重跑prepare。

## 操作及每步含义

每个WSL终端先设置：

```bash
RUN=/home/lw/slam-research/results/baseline/corridor01-full-b4-20261105-001
```

这是路径变量，不启动程序。本次使用start创建新容器，不用resume。

| 顺序/终端 | 命令 | 作用/目的/成功依据 |
|---|---|---|
| 1，终端1 | `bash "$RUN/run.sh" start` | 核对工具/代码/输入哈希并创建独立容器；返回容器ID，无报错才继续 |
| 2，终端1 | `bash "$RUN/run.sh" master` | 启动ROS注册/参数服务；就绪后保持运行 |
| 3，终端2 | `bash "$RUN/run.sh" init` | 在节点/记录器之前设仿真时间；应true |
| 4，终端2 | `bash "$RUN/run.sh" record-odom` | 原样记录Odometry/TF/path/clock，供轨迹和覆盖分析 |
| 5，终端3 | `bash "$RUN/run.sh" record-cloud` | 记录/cloud_registered分卷bag，保留逐帧时间 |
| 6，终端4 | `bash "$RUN/run.sh" algorithm` | 从新状态启动FAST-LIO2，启动前已设blind=4和在线PCD；无输入先等待 |
| 7，终端5 | `bash "$RUN/run.sh" rviz` | 显示最近10秒点云和路径，不参与估计 |
| 8，终端6 | `bash "$RUN/run.sh" watch` | 记录输入/输出接收计数；等Observer ready，旁证不等于内部无损或匹配成功 |
| 9，终端7 | `bash "$RUN/run.sh" ready` | 检查链路、运行参数及参考差异；必须PASS |
| 10，终端7 | `bash "$RUN/run.sh" play` | 1倍速、初始暂停、keep-alive，按空格开始；不截断、不循环 |

ready通过后可在终端7分别执行 `docker exec corridor01-full-b4-20261105-001-container /entrypoint.sh rosparam get /preprocess/blind` 和相同前缀 `rosparam get /pcd_save`，应4.0、true/-1；这是核实实际加载，不在启动后临时set。

播放前记录器等待时钟、画面空白可正常；播放后确认Recording to、观察计数和画面增加。即使出现预期的零匹配/轨迹异常，也先记下输入时间并保存截图，不边跑边调参或清图重来。正常可继续至EOF；若崩溃、持续录包丢数据或资源不足则保留现场，中止并标记不完整。失败结果也用于本次对照，不伪装完成。

另开终端8设置RUN后可多次 `bash "$RUN/run.sh" status` 保存负载快照，并留意Windows D盘实际空间。在线PCD会累计内存，不能无限等。快照不是全程峰值。

## EOF与收尾

1. 保持播放器运行，不暂停。终端8 `bash "$RUN/run.sh" end-status`，核对4038 LiDAR/82126 IMU、最新观察器和输出稳定至少10秒；可重复检查。结合播放器EOF和末位姿时间判断，不能只有计数。若blind=4严重失败导致无轨迹，保存失败，不强求检查通过。
2. 终端4 Ctrl+C一次，先停算法，等待写出PCD/scans.pcd和正常退出。播放器时钟仍走。
3. 终端2、3分别Ctrl+C，等bag关闭与info检查。
4. 停观察器、RViz，最后停播放器。master和容器先保留。
5. 退出卡住或SIGTERM升级时保留输出，不重复强杀、重命名.active或删除锁。keep-alive使Duration超过总时长属于收尾时钟，不重复数据。

## 导出验收（每步成功才继续）

```bash
bash "$RUN/run.sh" convert
bash "$RUN/run.sh" verify
bash "$RUN/run.sh" export-pcd
bash "$RUN/run.sh" compare-pcd
bash "$RUN/run.sh" downsample
bash "$RUN/run.sh" trajectory
```

convert导出header时间TUM并检查结构；verify核对轨迹/点云/覆盖/日志和哈希，可能数分钟；export-pcd合并点云不修正；compare-pcd比较同次两保存路径的共有XYZI；downsample生成独立5cm首点体素副本；trajectory输出同坐标时间着色PLY/起终点/CSV。任何异常保留原件先查看，不覆盖重跑；没有轨迹或点云时按失败结果审计，不人工补数据。结构检查通过不等于精度正常。

成功产物：PCD/scans.pcd、derived/final_registered.pcd、derived/final_registered_voxel_0p05m_first.pcd、derived/trajectory.tum、derived/trajectory-view/*，checks/日志/参数独立保留。原生额外normal字段不表示有效法向。

处理完成后终端1停止master，再 `docker stop corridor01-full-b4-20261105-001-container`。保留容器、档案和失败证据。

## 如何与blind=1比较

Windows打开 `\\wsl.localhost\Ubuntu-24.04\home\lw\slam-research\results\baseline\corridor01-full-b4-20261105-001\derived`，加载5cm点云及trajectory-view里的轨迹，先俯视再侧视。与blind=1采用相同点大小、颜色和相机视角；地图不同不能只从颜色推断。每次运行各有初始化世界参考，名称相同不替代坐标关系核查；若对齐要明确固定刚体变换及参考，不能拉伸/非刚性修图。

分析全序列覆盖、零匹配时间分布、起始/后段跳变、地图错位与轨迹；相同5cm规则不意味着两图应有相同点数。最初过滤导致的估计异常可能影响后续地图，不能自动解释成后段几何退化。首末参考核实前不报告ATE或把首末距离直接当定位误差。不只挑最差截图或最好片段。
