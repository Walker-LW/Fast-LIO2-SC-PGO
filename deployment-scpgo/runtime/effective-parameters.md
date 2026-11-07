# 原版 SC-PGO 有效参数与接口

来源固定为当前仓库的 `baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO`，导入来源提交 `10eab538db70678f41aaa24247d16831ca921374`，当前工程 HEAD `614691499ffd5a76abe6c53511fc60902a863f99`。用户已确认先采用下列源码默认值完成链路验证；未进行走廊调优。

## ROS 可配项：仅6项

读取位置：`src/laserPosegraphOptimization.cpp` 的 main，使用普通 `ros::NodeHandle nh`。专用launch把节点与参数共同放在 `/scpgo` 命名空间，不能放到节点私有 `~` 参数域。

| 参数 | 源码默认值 | 单位/用途 | 读取行 |
|---|---:|---|---:|
| `save_directory` | `/` | 输出根目录，字符串必须以 `/` 结尾；专用配置固定为 `/run/scpgo/backend/` | 775 |
| `keyframe_meter_gap` | 2.0 | 米；相邻里程计平移模长累计超过阈值选择关键帧 | 784 |
| `keyframe_deg_gap` | 10.0 | 度；内部转弧度，与相邻roll/pitch/yaw变化绝对值之和的累计比较 | 785 |
| `sc_dist_thres` | 0.2 | 无量纲，SC距离接受阈值 | 788 |
| `sc_max_radius` | 80.0 | 米，SC描述子考虑的最大平面半径 | 789 |
| `mapviz_filter_size` | 0.4 | 米，发布PGO可视化地图的体素边长 | 805 |

保存路径覆盖源码危险默认值 `/`。其余五项候选配置与源码默认值相同，用途是建立有来源的初始状态，不代表80米等设置适合Avia走廊。原上游Ouster launch显式使用关键帧0.5米、SC阈值0.3、显示体素0.05米；本候选不照搬该示例。

上游示例中的 `scan_line`、`minimum_range`、`mapping_line_resolution`、`mapping_plane_resolution`、`lidar_type` 没有被本后端 main 或当前编译的 Scancontext 读取，因此不放入后端 YAML。前端的 `/preprocess/lidar_type`、`scan_line`、`blind` 则是另一组有效前端参数，继续从原前端配置读取。

## 源码常量：不能通过 YAML 调整

| 内容 | 原值 | 来源 |
|---|---|---|
| SC/ICP降采样体素 | 0.4 m | main，800～802行 |
| ICP目标子图邻域 | 历史索引前后各25个关键帧，实际受边界限制 | doICPVirtualRelative，430行及loopFindNearKeyframesCloud |
| ICP最大对应距离 | 150 m | 449行 |
| ICP迭代上限 | 100 | 450行 |
| ICP变换/fitness停止阈值 | 各1e-6；具体量纲由PCL接口定义 | 451～452行 |
| ICP RANSAC迭代 | 0 | 453行 |
| ICP接受门限 | 收敛且fitness ≤ 0.3；fitness为对应点均方距离，坐标米时为m² | 461～466行 |
| SC描述子 | 20 rings × 60 sectors；高度偏置2 m | Scancontext.h，SCManager常量 |
| SC排除最近关键帧 | 30帧（不是30秒） | NUM_EXCLUDE_RECENT |
| KD树候选数/重建周期 | 3个 / 每30次检测调用 | NUM_CANDIDATES_FROM_TREE、TREE_MAKING_PERIOD_ |
| SC检测/iSAM/路径/地图发布 | 1 / 1 / 10 / 0.1 Hz，ROS仿真时间 | 对应process函数 |
| 订阅队列 | 点云100、里程计100、GPS100 | main，808～810行 |
| iSAM重线性化 | threshold=0.01，skip=1 | main，792～793行 |

先保留全部常量。`sc_max_radius` setter实际更新 `PC_MAX_RADIUS`，不把其它常量假称为随该参数动态联动。后端处理路径中未发现ROS可配置的同步容差、ICP门限、GPS开关或退出策略。

## 输入输出映射

| 后端原订阅 | 专用映射 | 类型与坐标 |
|---|---|---|
| `/aft_mapped_to_init` | `/Odometry` | nav_msgs/Odometry；camera_init→body |
| `/velodyne_cloud_registered_local` | `/cloud_registered_body` | sensor_msgs/PointCloud2；body局部系，XYZ/intensity |
| `/gps/fix` | `/scpgo/unused_gps` | sensor_msgs/NavSatFix；不回放GPS，ready拒绝该话题存在发布者 |

当前编译后端没有订阅 `/cloud_for_scancontext`，因此不做上游示例的第三路映射，也不假设有独立雷达系描述子输入。实际SC使用body点云经0.4米降采样。body是前端IMU参考坐标，不能把世界系 `/cloud_registered` 替换进来。前端两路输出使用相同 `lidar_end_time`；真实运行仍需观察精确配对、字段和覆盖。

输出沿用 `/aft_pgo_odom`、`/aft_pgo_path`、`/aft_pgo_map`，以及 `/loop_scan_local`、`/loop_submap_local`。优化不反馈前端滤波器或ikd-Tree。调试话题发布不等同于ICP接受；`Loop detected!`是SC候选；ICP `passed`日志在实际add因子之前打印，不能单凭此句声称已独立验证回环入图。

## 保存与保护

原 main 的778～782行打开times.txt并清理/重建Scans。仅可用于全新、未占用目录；外部guard在启动前要求整个backend目录为空，并为backend动作建立单次启动标记。源码/原build/devel只读，旧results目录不挂入此容器。本工具不删除旧Scans、不修改原版清理代码。
