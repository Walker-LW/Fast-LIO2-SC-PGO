#!/usr/bin/env bash
set -euo pipefail
run=$(cd -- "$(dirname -- "$0")" && pwd)
name=$(basename -- "$run")
[[ "$name" =~ ^scpgo-corridor02-geometry-[0-9]{8}-[0-9]{3}$ ]]
workspace=/home/lw/slam-research/fast_lio_ws
image=sha256:f306a0ec73450db1aa499376f84c0aa56d1046069ceb435087bd46b54e7e6db5
action=${1:?Use master|frontend|backend|record|ready|status|play}
[[ "$run" == /home/lw/slam-research/results/scpgo/$name ]]
[[ -f "$run/config/manifest.json" ]]
python3 - "$run/config/manifest.json" "$name" <<'PY'
import json,sys
assert json.load(open(sys.argv[1]))['id'] == sys.argv[2], 'Directory name and frozen experiment ID differ'
PY
r() { docker exec "$name" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; source /ws/devel_scpgo_geometry/setup.bash; exec "$@"' _ "$@"; }
ready_node() {
  local node=$1
  for i in {1..30}; do if r rosnode info "$node" >/dev/null 2>&1; then return; fi; sleep 1; done
  echo "Node not ready: $node; inspect checks logs"; exit 1
}
case "$action" in
 master)
  mkdir "$run/checks/master-once"
  docker run -d --name "$name" --network none --read-only --tmpfs /tmp:rw,size=512m \
    -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
    -e ROS_HOME=/tmp/corridor02-ros -e ROS_LOG_DIR=/run/scpgo/ros_logs \
    --mount "type=bind,source=$workspace,target=/ws,readonly" \
    --mount "type=bind,source=/home/lw/slam-research/datasets,target=/datasets,readonly" \
    --mount "type=bind,source=$run,target=/run/scpgo" \
    --mount "type=bind,source=$run/Log,target=/ws/src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/fast_lio2/Log" \
    --mount "type=bind,source=$run/PCD,target=/ws/src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/fast_lio2/PCD" \
    --entrypoint /bin/bash "$image" -c 'source /opt/ros/noetic/setup.bash; exec roscore > /run/scpgo/checks/master-console.log 2>&1'
  ready_node /rosout
  r rosparam set /use_sim_time true
  r python3 /run/scpgo/runtime.py identity
  r roslaunch /run/scpgo/config/backend.launch run_dir:=/run/scpgo config:=/run/scpgo/config/backend.yaml validate_only:=true > "$run/checks/startup-validation.log" 2>&1
  grep -q 'Configuration valid;' "$run/checks/startup-validation.log"
  echo MASTER_AND_BINARY_CHECK_PASS
  ;;
 frontend)
  mkdir "$run/checks/frontend-once"
  docker exec -d "$name" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel_scpgo_geometry/setup.bash; source /ws/devel/setup.bash; export CMAKE_PREFIX_PATH=/ws/devel:/ws/devel_scpgo_geometry:/opt/ros/noetic; exec roslaunch /run/scpgo/config/frontend.launch > /run/scpgo/checks/frontend-console.log 2>&1'
  ready_node /laserMapping
  echo FRONTEND_READY
  ;;
 backend)
  mkdir "$run/checks/backend-once"
  docker exec -d "$name" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; source /ws/devel_scpgo_geometry/setup.bash; exec roslaunch /run/scpgo/config/backend.launch run_dir:=/run/scpgo config:=/run/scpgo/config/backend.yaml validate_only:=false > /run/scpgo/checks/backend-console.log 2>&1'
  ready_node /scpgo/alaserPGO
  echo BACKEND_READY
  ;;
 record)
  ready_node /laserMapping
  ready_node /scpgo/alaserPGO
  mkdir "$run/checks/record-once"
  docker exec -d "$name" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; exec /opt/ros/noetic/lib/rosbag/record --buffsize 256 --split --size 1024 -O /run/scpgo/raw/interfaces.bag /Odometry /cloud_registered_body /aft_pgo_odom /aft_pgo_path /aft_pgo_map /loop_scan_local /loop_submap_local /tf /clock __name:=scpgo_record > /run/scpgo/checks/record-console.log 2>&1'
  docker exec -d "$name" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; exec python3 -u /run/scpgo/watch.py /run/scpgo > /run/scpgo/checks/watch-console.log 2>&1'
  ready_node /scpgo_record
  ready_node /scpgo_interface_watch
  echo RECORDER_AND_WATCH_READY
  ;;
 ready)
  r python3 /run/scpgo/runtime.py ready
  ;;
 status)
  r rosnode list
  r rosparam get /scpgo
  docker stats --no-stream "$name"
  if [[ -s "$run/checks/interface-observer.jsonl" ]]; then tail -n 1 "$run/checks/interface-observer.jsonl"; fi
  ;;
 play)
  [[ -t 0 ]] || { echo 'Use an interactive WSL terminal'; exit 1; }
  r python3 /run/scpgo/runtime.py ready
  mkdir "$run/checks/player-once"
  date -Is > "$run/checks/play-start-wall.txt"
  echo 'Full raw Corridor02, rate1, initially paused. Press SPACE once. At EOF retain this terminal for clock; run finish from terminal B.'
  docker exec -it "$name" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; exec /opt/ros/noetic/lib/rosbag/play --clock --hz 100 --pause --delay 3 --keep-alive -r 1 /datasets/Corridor02.bag --topics /livox/avia/lidar /livox/avia/imu __name:=scpgo_input' 2>&1 | tee "$run/checks/player-console.log"
  date -Is > "$run/checks/play-exited-wall.txt"
  ;;
 *) echo 'Unknown action'; exit 2 ;;
esac
