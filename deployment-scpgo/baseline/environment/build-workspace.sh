#!/bin/bash
set -eo pipefail
cd /home/lw/slam-research
docker compose exec -T fastlio bash -c 'set -e; cmake -P /opt/verify-sdk.cmake; /opt/sdk-probe/build/sdk_probe; source /opt/ros/noetic/setup.bash; cd /ws; catkin_make -j2 -l2 fast_lio_generate_messages livox_ros_driver_generate_messages && catkin_make -j2 -l2' </dev/null 2>&1 | tee results/validation/workspace-build.log
