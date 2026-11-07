#!/usr/bin/env bash
set -euo pipefail
run=$(cd -- "$(dirname -- "$0")" && pwd)
action=${1:?Use start|master|frontend|backend|record|watch|ready|status}
name=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["container"])' "$run/config/manifest.json")
inside=/run/scpgo
ros() { docker exec "$name" /entrypoint-scpgo.sh "$@"; }
interactive() { docker exec -it "$name" /entrypoint-scpgo.sh "$@"; }
printf '%s %s\n' "$(date -Is)" "$action" >> "$run/checks/actions.log"
trap 'rc=$?; printf "%s %s exit=%s\n" "$(date -Is)" "$action" "$rc" >> "$run/checks/actions.log"' EXIT
python3 "$run/guard.py" "$run" "$action"
case "$action" in
  start) docker start "$name" ;;
  master) interactive roscore 2>&1 | tee "$run/checks/master-console.log" ;;
  frontend)
    ros rosparam set /use_sim_time true
    # Both devel spaces share /ws/src. Resolve this process in its own devel first.
    ros env CMAKE_PREFIX_PATH=/ws/devel:/ws/devel_scpgo:/opt/ros/noetic python3 -c \
      'from roslib.packages import find_node; assert find_node("fast_lio","fastlio_mapping") == ["/ws/devel/lib/fast_lio/fastlio_mapping"]'
    interactive env CMAKE_PREFIX_PATH=/ws/devel:/ws/devel_scpgo:/opt/ros/noetic \
      roslaunch "$inside/config/frontend.launch" 2>&1 | tee "$run/checks/frontend-console.log" ;;
  backend)
    [[ "$(ros rosparam get /use_sim_time)" == true ]]
    interactive roslaunch "$inside/config/backend.launch" "run_dir:=$inside" 2>&1 | tee "$run/checks/backend-console.log" ;;
  record)
    [[ "$(ros rosparam get /use_sim_time)" == true ]]
    interactive /opt/ros/noetic/lib/rosbag/record --buffsize 256 --split --size 1024 \
      -O "$inside/raw/interfaces.bag" /Odometry /cloud_registered_body \
      /aft_pgo_odom /aft_pgo_path /aft_pgo_map /loop_scan_local /loop_submap_local /tf /clock \
      __name:=scpgo_record 2>&1 | tee "$run/checks/record-console.log" ;;
  watch)
    interactive python3 -u "$inside/watch.py" "$inside" 2>&1 | tee "$run/checks/watch-console.log" ;;
  ready)
    ros python3 "$inside/ready.py" "$inside" 2>&1 | tee "$run/checks/ready-$(date +%Y%m%dT%H%M%S%N).log" ;;
  status)
    docker inspect --format '{{json .State}}' "$name"
    if [[ "$(docker inspect --format '{{.State.Running}}' "$name")" == true ]]; then
      ros rosnode list
      docker stats --no-stream "$name"
    fi
    ;;
  *) echo 'Unknown action'; exit 2 ;;
esac
