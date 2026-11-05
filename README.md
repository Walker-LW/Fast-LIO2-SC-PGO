# Fast-LIO2-SC-PGO

基于 M3DGR FAST-LIO2 适配工程的 SC-PGO 集成项目。当前保留已有前端，逐步引入独立后端，用于 Corridor01 / Corridor02 建图对照。

## 目录

- `baseline_systems/Fast_LIO2_M3DGR/src/`：实际 catkin 包源码，保持现有 WSL 路径。
- `deployment-scpgo/`：环境、配置和版本记录。

实际工作空间在 WSL `/home/lw/slam-research/fast_lio_ws`，容器内为 `/ws`；不是在本仓库根目录直接构建。

## 版本与回退

`main` 固定引入 SC-PGO 前的精简工程基线；`integration/scpgo` 用于后端集成。`baseline/fastlio2-before-scpgo` 标记项目基线。基线原前端程序、镜像和运行配置见 `deployment-scpgo/locks/` 与 `baseline/`。

本仓库历史仅包含当前适配工程及部署记录，没有导入整个 M3DGR 历史。原始来源为 [M3DGR](https://github.com/sjtuyinjie/M3DGR)，提交 `e0cf7d59c9a5a3df515624034698d976abc26549`。原 M3DGR Git 历史仍保留在本地上游引用及独立备份中。各第三方文件保留原有作者、许可和来源说明。

编译产物、数据集和实验输出不纳入 Git。回退需要同时使用对应源码、环境镜像及运行配置；源码回退不会自动恢复编译产物。

当前准备阶段不代表 SC-PGO 已完成环境部署、编译、接口适配或真实数据验证。
