#!/usr/bin/env bash
set -euo pipefail
workspace=/home/lw/slam-research/fast_lio_ws
docker run --rm --network none \
  --mount "type=bind,source=$workspace,target=/ws" \
  --mount "type=bind,source=$workspace/src,target=/ws/src,readonly" \
  --mount "type=bind,source=$workspace/build,target=/ws/build,readonly" \
  --mount "type=bind,source=$workspace/devel,target=/ws/devel,readonly" \
  --mount "type=bind,source=$workspace/build_scpgo,target=/ws/build_scpgo,readonly" \
  --mount "type=bind,source=$workspace/devel_scpgo,target=/ws/devel_scpgo,readonly" \
  --mount "type=bind,source=$workspace/build_scpgo_params,target=/ws/build_scpgo_params,readonly" \
  --mount "type=bind,source=$workspace/devel_scpgo_params,target=/ws/devel_scpgo_params,readonly" \
  --entrypoint /bin/bash sha256:f306a0ec73450db1aa499376f84c0aa56d1046069ceb435087bd46b54e7e6db5 -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    cd /ws
    catkin_make --build build_scpgo_geometry \
      -DCATKIN_DEVEL_PREFIX=/ws/devel_scpgo_geometry \
      -DCATKIN_WHITELIST_PACKAGES=aloam_velodyne \
      -DCMAKE_INSTALL_PREFIX=/ws/install_scpgo_geometry \
      -j2 -l2 alaserPGO
  ' > /mnt/d/projects/fast-lio2-learning/tools/scpgo_geometry_fix_20261106/backend-build.log 2>&1
