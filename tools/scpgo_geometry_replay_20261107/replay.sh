#!/usr/bin/env bash
# Each stage runs only when the user explicitly invokes it from WSL.
set -euo pipefail
mode=${1:?Use old or fixed}
action=${2:?Use master, backend, record, status, play, finish, export}
case "$mode" in
 old) overlay=/ws/devel_scpgo_params; expected=65fbef68a38561c1175fa6947687983a3162dd2d3bf86c487db1e1394b3a94d1 ;;
 fixed) overlay=/ws/devel_scpgo_geometry; expected=03d2e316a408a1906a9613757794af4888512b34b2aae5683d7aab17d4b3c9c6 ;;
 *) echo 'Invalid mode'; exit 2 ;;
esac
base=/home/lw/slam-research/results/scpgo/scpgo-corridor01-geometry-ab-20261107-001
run=$base/$mode
container=scpgo-corridor01-geometry-$mode-20261107-001
workspace=/home/lw/slam-research/fast_lio_ws
input=/home/lw/slam-research/results/scpgo/scpgo-corridor01-20261106-002
image=sha256:f306a0ec73450db1aa499376f84c0aa56d1046069ceb435087bd46b54e7e6db5
tools=/mnt/d/projects/fast-lio2-learning/tools/scpgo_geometry_replay_20261107
[[ -f "$run/checks/prepared.json" ]]
r() { docker exec "$container" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; source "$BACKEND_OVERLAY/setup.bash"; exec "$@"' _ "$@"; }
case "$action" in
 master)
  docker run -d --name "$container" --network none --read-only --tmpfs /tmp:rw,size=512m \
   -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 -e ROS_HOME=/tmp/geometry-replay-ros \
   -e ROS_LOG_DIR=/run/scpgo/ros_logs -e BACKEND_OVERLAY="$overlay" \
   --mount "type=bind,source=$workspace,target=/ws,readonly" \
   --mount "type=bind,source=$input,target=/input,readonly" \
   --mount "type=bind,source=$run,target=/run/scpgo" \
   --entrypoint /bin/bash "$image" -c 'source /opt/ros/noetic/setup.bash; exec roscore > /run/scpgo/checks/master-console.log 2>&1'
  ready=0
  for i in {1..30}; do if r rosnode list > /dev/null 2>&1; then ready=1; break; fi; sleep 1; done
  [[ "$ready" == 1 ]] || { echo 'Master not ready; inspect master-console.log'; exit 1; }
  r rosparam set /use_sim_time true
  r python3 -c 'import os,hashlib; from roslib.packages import find_node; p=find_node("aloam_velodyne","alaserPGO"); assert p==[os.environ["BACKEND_OVERLAY"]+"/lib/aloam_velodyne/alaserPGO"],p; print(p); print(hashlib.sha256(open(p[0],"rb").read()).hexdigest())' | tee "$run/checks/resolved-binary.txt"
  grep -q "$expected" "$run/checks/resolved-binary.txt"
  r roslaunch /run/scpgo/config/backend.launch run_dir:=/run/scpgo config:=/run/scpgo/config/backend.yaml validate_only:=true > "$run/checks/startup-validation.log" 2>&1
  grep -q 'Configuration valid;' "$run/checks/startup-validation.log"
  echo 'MASTER_AND_BINARY_CHECK_PASS'
  ;;
 backend)
  mkdir "$run/checks/backend-started-once"
  docker exec -d "$container" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; source "$BACKEND_OVERLAY/setup.bash"; exec roslaunch /run/scpgo/config/backend.launch run_dir:=/run/scpgo config:=/run/scpgo/config/backend.yaml validate_only:=false > /run/scpgo/checks/backend-console.log 2>&1'
  ready=0
  for i in {1..30}; do if r rosnode info /scpgo/alaserPGO >/dev/null 2>&1; then ready=1; break; fi; sleep 1; done
  [[ "$ready" == 1 ]] || { echo 'Backend not ready; inspect backend-console.log'; exit 1; }
  r rosparam dump /run/scpgo/checks/effective-parameters.yaml
  echo 'BACKEND_READY'
  ;;
 record)
  r rosnode info /scpgo/alaserPGO >/dev/null
  mkdir "$run/checks/record-started-once"
  docker exec -d "$container" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; source "$BACKEND_OVERLAY/setup.bash"; exec /opt/ros/noetic/lib/rosbag/record --buffsize 256 --split --size 1024 -O /run/scpgo/raw/interfaces.bag /Odometry /cloud_registered_body /aft_pgo_odom /aft_pgo_path /aft_pgo_map /loop_scan_local /loop_submap_local /tf /clock __name:=scpgo_record > /run/scpgo/checks/record-console.log 2>&1'
  docker exec -d "$container" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; source "$BACKEND_OVERLAY/setup.bash"; exec python3 -u /run/scpgo/watch.py /run/scpgo > /run/scpgo/checks/watch-console.log 2>&1'
  ready=0
  for i in {1..30}; do if r rosnode info /scpgo_record >/dev/null 2>&1 && r rosnode info /scpgo_interface_watch >/dev/null 2>&1; then ready=1; break; fi; sleep 1; done
  [[ "$ready" == 1 ]] || { echo 'Recorder/watch not ready'; exit 1; }
  r rosnode list
  echo 'RECORDER_AND_WATCH_READY'
  ;;
 status)
  r rosnode list
  r rosparam get /scpgo
  r rosnode info /scpgo/alaserPGO
  r rostopic info /scpgo/unused_gps
  if [[ -s "$run/checks/interface-observer.jsonl" ]]; then tail -n 1 "$run/checks/interface-observer.jsonl"; fi
  ;;
 play)
  [[ -t 0 ]] || { echo 'Run play in an interactive WSL terminal'; exit 1; }
  r rosnode info /scpgo/alaserPGO >/dev/null
  r rosnode info /scpgo_record >/dev/null
  r rosnode info /scpgo_interface_watch >/dev/null
  mkdir "$run/checks/player-started-once"
  printf 'Recorded002 inputs only, identity backend configuration, recorded clock, rate1.0\n' > "$run/checks/playback-controls.txt"
  echo 'Initially paused: press SPACE once after startup. Replay includes original clock tail. Do not add --clock.'
  docker exec -it "$container" /bin/bash -c 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; source "$BACKEND_OVERLAY/setup.bash"; exec /opt/ros/noetic/lib/rosbag/play --pause --delay 3 -r 1 /input/raw/interfaces_0.bag /input/raw/interfaces_1.bag --topics /Odometry /cloud_registered_body /clock __name:=scpgo_input' 2>&1 | tee "$run/checks/player-console.log"
  touch "$run/checks/player-exited"
  echo 'PLAYER_EXITED: finish from another terminal after checking status'
  ;;
 finish)
  [[ -f "$run/checks/player-exited" ]] || { echo 'Player has not finished normally'; exit 1; }
  sleep 3
  python3 "$tools/manage.py" end-check "$mode"
  r rosnode kill /scpgo_interface_watch
  r rosnode kill /scpgo_record
  for i in {1..30}; do if ! find "$run/raw" -maxdepth 1 -name '*.active' -print -quit | grep -q .; then break; fi; sleep 1; done
  if find "$run/raw" -maxdepth 1 -name '*.active' -print -quit | grep -q .; then echo 'Recorder not closed; keep container running'; exit 1; fi
  r rosnode kill /scpgo/alaserPGO || true
  docker stop -t 20 "$container"
  python3 "$tools/manage.py" frozen "$mode"
  ;;
 export)
  [[ -f "$run/checks/stopped-files-check.json" ]]
  docker run --rm --network none --read-only --tmpfs /tmp:rw,size=128m \
    --mount "type=bind,source=$run,target=/run/scpgo" --entrypoint python3 "$image" \
    /run/scpgo/export_keyframe_maps.py /run/scpgo /run/scpgo/derived/keyframe-maps
  ;;
 *) echo 'Invalid stage'; exit 2 ;;
esac
