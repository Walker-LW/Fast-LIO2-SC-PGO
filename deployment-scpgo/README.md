# SC-PGO 部署记录

本目录与当前工程共用一个 Git 仓库。外层 `/home/lw/slam-research/deployment-scpgo` 为指向本目录的符号链接。

`baseline/` 保存现有环境说明和两条 blind=1 运行的配置；`locks/` 保存源码、镜像和原程序校验记录。没有保存编译产物、实验点云或凭据。原环境 Dockerfile 中引用的 Livox SDK 提交为 `9306596a2bf15c1343bc023b497465ed0a32909d`，SDK 本体不在本仓库。

当前仅固定基线。依赖镜像、独立后端构建和新实验启动配置尚未实施。不要使用 baseline/environment/compose.yaml 直接启动新实验：该文件保留了旧 Dynamic01 输出挂载，仅作历史参考。

后续后端构建目录为 `/ws/build_scpgo`，产物目录为 `/ws/devel_scpgo`；原 `/ws/build`、`/ws/devel` 应只读保护。
