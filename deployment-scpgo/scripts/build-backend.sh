#!/usr/bin/env bash
set -euo pipefail
base=/home/lw/slam-research
workspace=$base/fast_lio_ws
image=slam-research-fastlio-scpgo:noetic-gtsam403-v1
[[ "$(docker image inspect "$image" --format '{{.Id}}')" == sha256:f306a0ec73450db1aa499376f84c0aa56d1046069ceb435087bd46b54e7e6db5 ]]
[[ "$(sha256sum "$workspace/devel/lib/fast_lio/fastlio_mapping" | cut -d ' ' -f 1)" == 607d6fc36c4efeaefbb8d375e23fbe199da8356d014f75fbe8ca802f289773be ]]
[[ -z "$(docker ps -q --filter volume="$workspace")" ]]
docker run --rm --name scpgo-backend-build --network none \
  --mount "type=bind,source=$workspace,target=/ws" \
  --mount "type=bind,source=$workspace/src,target=/ws/src,readonly" \
  --mount "type=bind,source=$workspace/build,target=/ws/build,readonly" \
  --mount "type=bind,source=$workspace/devel,target=/ws/devel,readonly" \
  --entrypoint /bin/bash "$image" -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    cd /ws
    catkin_make --build build_scpgo \
      -DCATKIN_DEVEL_PREFIX=/ws/devel_scpgo \
      -DCATKIN_WHITELIST_PACKAGES=aloam_velodyne \
      -DCMAKE_INSTALL_PREFIX=/ws/install_scpgo \
      -j2 -l2 alaserPGO
  '
[[ "$(sha256sum "$workspace/devel/lib/fast_lio/fastlio_mapping" | cut -d ' ' -f 1)" == 607d6fc36c4efeaefbb8d375e23fbe199da8356d014f75fbe8ca802f289773be ]]
