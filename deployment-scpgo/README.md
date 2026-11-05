# SC-PGO 部署记录

本目录与当前工程共用一个 Git 仓库。外层 `/home/lw/slam-research/deployment-scpgo` 是指向本目录的符号链接。

## 当前完成情况

1. 已固定原 FAST-LIO2 源码、镜像、程序校验值及两条 blind=1 实验的实际配置。
2. 已在指定包目录原样引入 SC-PGO，来源及逐文件清单见 `upstream/fast_lio_slam.json`。
3. 已构建并验收依赖镜像 `slam-research-fastlio-scpgo:noetic-gtsam403-v1`。精确镜像身份、包版本及验证结果见 `locks/scpgo-image.json`。
4. 已在独立 build_scpgo/devel_scpgo 中编译 alaserPGO，动态库、包路径和环境叠加检查通过，构建记录见 `locks/scpgo-backend-build.json`。

SC-PGO 本体已编译，但未执行节点，未实施接口或后端算法修正，未开始 Corridor 新实验。编译通过不代表回环约束和地图优化已验证。

## 原环境保护

原镜像 `slam-research-fastlio:noetic` 和 `slam-research-fastlio:baseline-pre-scpgo-20261105` 均保持原 ID。新镜像保留其全部基础层。原前端程序 SHA-256 未变，原 build/devel 原地保留。

新镜像仅保存在当前 WSL Docker 中，没有上传镜像仓库。Git 远端保存构建文件和版本记录，不保存镜像、依赖源码缓存、编译产物、bag 或实验点云。

`baseline/environment/` 中的原 Dockerfile 引用 Livox SDK 提交 `9306596a2bf15c1343bc023b497465ed0a32909d`。原 Compose 仍包含旧 Dynamic01 输出挂载，仅作基线参考，不用来启动新实验。

## 重建依赖环境

在 WSL 执行：

```bash
bash /home/lw/slam-research/deployment-scpgo/scripts/prepare-gtsam.sh
bash /home/lw/slam-research/deployment-scpgo/scripts/build-environment.sh
```

准备脚本下载并核对官方 GTSAM 4.0.3 提交 `6425000775da29c93719f37dce3f2de38a0064ec`。源码缓存在 `.build-context/gtsam`，不进 Git。构建脚本核对基线镜像，目标标签已存在时拒绝覆盖；无论成功或失败均复核原镜像身份。

Dockerfile 在派生层中使用官方 Ubuntu HTTPS 地址、ROS Noetic final snapshot 源以及下载缓存和重试。没有执行系统整体升级。`locks/scpgo-packages.txt` 记录本次实际安装版本；它是本次镜像的环境清单，不代表未来软件源一定能按同样字节重建。

## 验收

```bash
python3 /home/lw/slam-research/deployment-scpgo/scripts/verify-environment.py \
  --output /home/lw/slam-research/results/validation/scpgo-image-check-NEW
```

输出目录必须未占用。验收使用只读容器和内存临时目录，不启动 ROS 节点，不写工作空间。覆盖：GTSAM ISAM2 小型位姿图、Ceres 链接、OpenCV/cv_bridge 图像转换、原前端动态库解析、镜像基础层及程序校验值、工作空间缺失时拒绝启动。

运行入口要求 `/ws/devel/setup.bash` 和 `/ws/devel_scpgo/setup.bash` 均存在，缺失则拒绝启动。第4步已通过默认入口验证两个环境按新后端、原工作空间、ROS的顺序叠加；未启动SLAM节点。

## 本次记录

日志在 `/home/lw/slam-research/results/validation/scpgo-image-20261105-001/`。首次构建遇到 Ubuntu HTTP 下载超时，保留 build-001.log；切换派生层 HTTPS 并缓存重试后 build-002.log 成功。首次只读验收因 ROS setup 需要 /tmp 临时文件未能完成，补充内存临时目录后 verification-002 全部通过；首次记录仍保留。

## 独立后端构建（第4步已完成）

```bash
bash /home/lw/slam-research/deployment-scpgo/scripts/build-backend.sh
```

只构建 aloam_velodyne 的 alaserPGO 目标，使用 build_scpgo/devel_scpgo；源码、原build、原devel只读挂载，构建不联网。没有重新编译前端，也没有编译附带的A-LOAM节点。部署目录中的CATKIN_IGNORE用于阻止catkin将GTSAM下载缓存当作工作空间包。

验收示例：

```bash
python3 /home/lw/slam-research/deployment-scpgo/scripts/verify-backend.py \
  --baseline-artifacts /home/lw/slam-research/results/validation/scpgo-build-20261105-001/baseline-build-artifacts.json \
  --output /home/lw/slam-research/results/validation/scpgo-backend-check-NEW
```

输出目录必须未占用，基线清单来自首次构建前。该清单及构建产物只保存在本机，不纳入Git。此次原build/devel的997项文件/链接记录全部一致。alaserPGO动态库解析成功，前端仍从原devel提供，SC-PGO66个源文件字节未改变。

实际目标编译选项为 -O3 -Wall -g -fopenmp -std=gnu++17，沿用上游配置，没有补加NDEBUG。原版编译出现两条警告：xy2theta存在未覆盖返回路径、目录清理的返回值变量未使用。本轮只记录，未修改原算法。启动清理目录、时间配对、子图坐标和退出保存等仍须在第5步处理，因此本轮没有执行alaserPGO。

下一步为第5步运行适配与独立实验配置，尚未开始。
