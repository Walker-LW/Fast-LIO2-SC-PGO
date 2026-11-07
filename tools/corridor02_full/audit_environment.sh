#!/usr/bin/env bash
# Read-only inspection. No ROS master, algorithm, recorder, or player is started.
set -euo pipefail
base=/home/lw/slam-research
src=$base/fast_lio_ws/src/M3DGR
image=$(docker image inspect slam-research-fastlio:noetic --format '{{.Id}}')
date -Is
uname -a
docker version
docker ps -a --no-trunc
git -C "$src" rev-parse HEAD
git -C "$src" status --short
sha256sum "$src/baseline_systems/Fast_LIO2_M3DGR/src/fast_lio2/src/laserMapping.cpp"
sha256sum "$base/datasets/Corridor02.bag"
df -h "$base"
free -h
cat "$base/compose.yaml"
docker run --rm --read-only --network none --init --tmpfs /tmp --entrypoint /bin/bash \
  --device /dev/dxg:/dev/dxg \
  --mount type=bind,source=/mnt/wslg/.X11-unix,target=/tmp/.X11-unix,readonly \
  --mount type=bind,source=/usr/lib/wsl,target=/usr/lib/wsl,readonly \
  --mount "type=bind,source=$base/fast_lio_ws,target=/ws,readonly" \
  --mount "type=bind,source=$base/datasets,target=/datasets,readonly" \
  -e "DISPLAY=${DISPLAY:?Run in WSL with DISPLAY set}" \
  -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros_logs \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  "$image" -c '
    set -e
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    cat /etc/os-release
    rosversion rosbag
    rosversion rviz
    glxinfo -B
    /opt/ros/noetic/lib/rosbag/play --help
    /opt/ros/noetic/lib/rosbag/record --help
    rosbag info /datasets/Corridor02.bag
    ldd /ws/devel/lib/fast_lio/fastlio_mapping
    python3 - <<'\''PY'\''
import json
import rosbag
import rospy
with rosbag.Bag("/datasets/Corridor02.bag") as bag:
    report = {}
    for topic in ("/livox/avia/lidar", "/livox/avia/imu"):
        first = next(bag.read_messages(topics=[topic]))
        last = None
        for row in bag.read_messages(topics=[topic], start_time=rospy.Time.from_sec(bag.get_end_time()-2)):
            last = row
        if last is None:
            raise RuntimeError("No tail sample for " + topic)
        report[topic] = {
            "first_header_ns": first.message.header.stamp.to_nsec(),
            "last_header_ns": last.message.header.stamp.to_nsec(),
            "last_bag_time_ns": last.timestamp.to_nsec(),
        }
        if topic.endswith("lidar"):
            offsets = [p.offset_time for p in last.message.points]
            report[topic]["last_max_point_offset_ns"] = max(offsets) if offsets else None
    print(json.dumps(report, indent=2))
PY
  '
