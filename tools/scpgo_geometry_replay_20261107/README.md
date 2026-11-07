# 用户手动执行：同输入旧/几何修正版后端整图对照

2026-11-07完成状态：用户已完成本001的old/fixed，383相同关键帧及双地图对照通过，未见可辨认重影改善。结果见[效果审查](../scpgo_geometry_effect_audit_20261107/README.md)。下文清理/启动命令是已执行过程记录，不能再对完成目录执行；新实验须使用新的目录和容器身份。reset_old.py的完成标记保护仍保留。

当前中断恢复约定：用户明确允许覆盖本次001/old临时试跑输出。执行`python3 reset_old.py`会关闭录包与后端、移除旧容器并清理该子目录输出，保留配置和prepared.json；同路径重新启动，不改实验编号，不碰来源002录包或fixed试验输出。当前播放器已取消--quiet，重置也会更新尚未启动的fixed播放器脚本。该授权覆盖下文历史“不可复用临时中断目录”的保守约定；完成的正式结果仍不应误清理。

准备工具已经通过shell/Python语法和当前二进制、镜像、输入文件只读预检；没有替用户启动实验，也不声称整段流程已经运行通过。

输入为002的两份原始接口录包，仅重放Odometry、cloud_registered_body和已录clock；两份录包合计4034组前端输入，记录时长约436秒，包含原实验时钟收尾。这里不重跑FAST-LIO2、不发布旧PGO输出、不使用新合成clock、不改数值配置。

脚本目录WSL：`/mnt/d/projects/fast-lio2-learning/tools/scpgo_geometry_replay_20261107`。

新输出根：`/home/lw/slam-research/results/scpgo/scpgo-corridor01-geometry-ab-20261107-001`，下分old/fixed。目录或容器名被占用时准备会停止；不删除旧结果，不复用旧实验输出。

1. 用户运行`python3 manage.py preflight`，再`python3 manage.py prepare`。prepare只创建新目录、复制002配置和观察/导图脚本，不启动容器。
2. 用户依次运行`bash "$ROOT/old/replay.sh" old master`、`backend`、`record`、`status`。每个阶段有成功标志；检查配置仍为40m/1m/10°/0.2/0.2m，SC与ICP体素0.4m、对应150m、接受0.3m²。
3. 交互终端运行`... old play`，按一次空格。播放器会自动结束，约7分多钟；不再加`--clock`、`--keep-alive`或第二次播放。另一终端查看checks日志。
4. 正常播放结束后运行`... old finish`。先检查输入和保存覆盖，再停止观察器、关闭录包、停止后端和容器，核对停止前后保存文件哈希。原后端停止时可能保留历史异常；保存检查不代表退出无故障。
5. 运行`... old export`。以同一完整保存关键帧生成before_pgo.pcd和after_pgo.pcd，原PCD不滤波。
6. 对fixed重复master/backend/record/status/play/finish/export；不要使用旧002的env.sh。
7. 两个容器停止并导图后，在现有镜像中挂载根目录为/pair、脚本为/scripts，执行`python3 /scripts/compare.py`。脚本核对383帧、每帧点云、时间和原始里程计文件一致；不一致则拒绝输出严格同帧对照。生成`derived/comparison/old-fixed-before-after.png`和`report.json`。

各命令明确调用才运行，不自动串行播放。每个运行目录只允许启动一次后端、录包和播放器。backend启动会触发上游Scans清理，所以不允许在同一目录重启；任一步失败停止并提供日志，由具体失败决定恢复办法。

如果finish因计数或位姿文件不足停止，录包和容器会继续保留，不能为了通过检查直接跳过。若闭包后文件改变或出现active文件，也会停止。export/compare输出目录/文件已存在时不可盲目重跑。

这是一项同输入诊断验收，不是精度提升结论。两个后端的线程调度、候选检测时机和因子集合可能不同；图和report只能先证明输入/关键帧可比，后续还须核查日志与入图影响。watch看到的lidar/imu为0是预期，因为本次只重放前端输出；不应启动laserMapping补发输入。
