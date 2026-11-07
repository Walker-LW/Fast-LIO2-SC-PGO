#!/usr/bin/env bash
set -euo pipefail
run=$(cd -- "$(dirname -- "$0")" && pwd)
id=$(basename "$run")
[[ "$id" =~ ^corridor01-full-b[14]-[0-9]{8}-[0-9]{3}$ ]] || { echo 'Run prepare.sh first; use its frozen copy'; exit 1; }
name=$id-container
inside=/results/baseline/$id
base=/home/lw/slam-research
pkg=/ws/src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/fast_lio2
action=${1:?Use: start|master|init|record-odom|record-cloud|algorithm|rviz|watch|ready|play|status|end-status|convert|verify|export-pcd|downsample|trajectory}
printf '%s %s\n' "$(date -Is)" "$action" >> "$run/checks/actions.log"
trap 'rc=$?; printf "%s %s exit=%s\n" "$(date -Is)" "$action" "$rc" >> "$run/checks/actions.log"' EXIT
ros() { docker exec "$name" /entrypoint.sh "$@"; }
interactive() { docker exec -it "$name" /entrypoint.sh "$@"; }
once() { mkdir -- "$run/checks/$action-once"; }
case "$action" in
  resume)
    sha256sum -c "$run/config/tools.sha256"
    sha256sum -c "$run/config/prepared-files.sha256"
    sha256sum -c "$run/config/source.sha256"
    sha256sum -c "$run/config/executable.sha256"
    sha256sum -c "$run/config/input.sha256"
    once
    if [[ "$(docker inspect --format '{{.Image}}' "$name")" != "$(cat "$run/config/image.txt")" ]]; then
      echo 'Existing container image differs'; exit 1
    fi
    if [[ "$(docker inspect --format '{{.State.Running}}' "$name")" != true ]]; then docker start "$name"; fi
    docker inspect "$name" > "$run/config/container-resumed.json"
    echo 'Existing container ready. Start a fresh master next; no algorithm or playback started.'
    ;;
  start)
    [[ -z "$(docker ps -q --filter volume="$base/fast_lio_ws")" ]] || { echo 'Another workspace container is running; inspect first'; exit 1; }
    sha256sum -c "$run/config/tools.sha256"
    sha256sum -c "$run/config/prepared-files.sha256"
    sha256sum -c "$run/config/source.sha256"
    sha256sum -c "$run/config/executable.sha256"
    sha256sum -c "$run/config/input.sha256"
    once
    docker run -d --name "$name" --init --network none \
      --device /dev/dxg:/dev/dxg \
      --mount "type=bind,source=$base/fast_lio_ws,target=/ws,readonly" \
      --mount "type=bind,source=$base/datasets,target=/datasets,readonly" \
      --mount "type=bind,source=$run,target=$inside" \
      --mount "type=bind,source=$run/Log,target=$pkg/Log" \
      --mount "type=bind,source=$run/PCD,target=$pkg/PCD" \
      --mount type=bind,source=/mnt/wslg/.X11-unix,target=/tmp/.X11-unix,readonly \
      --mount type=bind,source=/usr/lib/wsl,target=/usr/lib/wsl,readonly \
      -e "DISPLAY=${DISPLAY:?Use a WSL terminal}" \
      -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 \
      -e LD_LIBRARY_PATH=/usr/lib/wsl/lib -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
      -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
      -e "ROS_LOG_DIR=$inside/ros_logs" \
      "$(cat "$run/config/image.txt")" sleep infinity
    docker inspect "$name" > "$run/config/container.json"
    ros glxinfo -B > "$run/checks/gpu.txt"
    ros rosbag info /datasets/Corridor01.bag > "$run/config/input-bag.txt"
    ;;
  master) once; interactive roscore 2>&1 | tee "$run/checks/master-console.log" ;;
  init)
    once
    ros rosparam set /use_sim_time true
    ros rosparam get /use_sim_time
    ;;
  record-odom|record-cloud)
    [[ "$(ros rosparam get /use_sim_time)" == true ]] || { echo 'Set /use_sim_time before recording'; exit 1; }
    once
    if [[ "$action" == record-odom ]]; then
      interactive /opt/ros/noetic/lib/rosbag/record --buffsize 64 \
        -O "$inside/raw/odometry.bag" /Odometry /tf /path /clock __name:=record_pose \
        2>&1 | tee "$run/checks/record-odom-console.log"
      [[ -f "$run/raw/odometry.bag" && ! -e "$run/raw/odometry.bag.active" ]]
      ros rosbag info "$inside/raw/odometry.bag" > "$run/checks/odometry-info.txt"
    else
      interactive /opt/ros/noetic/lib/rosbag/record --buffsize 256 --split --size 1024 \
        -O "$inside/raw/cloud.bag" /cloud_registered __name:=record_cloud \
        2>&1 | tee "$run/checks/record-cloud-console.log"
      shopt -s nullglob
      bags=("$run"/raw/cloud*.bag); active=("$run"/raw/cloud*.active)
      [[ ${#bags[@]} -gt 0 && ${#active[@]} -eq 0 ]]
      for bag in "${bags[@]}"; do ros rosbag info "$inside/raw/$(basename "$bag")"; done > "$run/checks/cloud-info.txt"
    fi
    ;;
  algorithm) once; interactive roslaunch "$inside/config/full.launch" 2>&1 | tee "$run/checks/algorithm-console.log" ;;
  rviz) once; interactive rviz -d "$inside/config/view.rviz" __name:=corridor_rviz 2>&1 | tee "$run/checks/rviz-console.log" ;;
  watch) once; interactive python3 -u "$inside/watch.py" 2>&1 | tee "$run/checks/live-progress.log" ;;
  ready)
    once
    ros rosparam dump "$inside/config/runtime.yaml"
    ros rosnode list | tee "$run/checks/nodes-before-play.txt"
    ros python3 "$inside/ready.py" 2>&1 | tee "$run/checks/ready-console.log"
    touch "$run/checks/ready.ok"
    ;;
  play)
    [[ -f "$run/checks/ready.ok" && -f "$run/config/runtime.yaml" ]]
    once
    interactive /opt/ros/noetic/lib/rosbag/play --clock --hz 100 --pause --keep-alive -r 1 \
      /datasets/Corridor01.bag --topics /livox/avia/lidar /livox/avia/imu __name:=corridor_input \
      2>&1 | tee "$run/checks/player-console.log"
    ;;
  status)
    stamp=$(date +%Y%m%dT%H%M%S%N)
    { date -Is; docker stats --no-stream "$name"; free -h; df -h "$run"; ros rosnode list; } \
      | tee "$run/checks/status-$stamp.log"
    ;;
  convert)
    [[ -f "$run/raw/odometry.bag" && ! -e "$run/raw/odometry.bag.active" ]]
    once
    ros python3 "$inside/convert.py" "$inside/raw/odometry.bag" \
      "$inside/derived/trajectory.tum" "$inside/checks/trajectory.json" \
      2>&1 | tee "$run/checks/convert-console.log"
    ;;
  verify)
    once
    ros python3 "$inside/verify.py" "$inside" 2>&1 | tee "$run/checks/verify-console.log"
    ;;
  export-pcd)
    once
    ros python3 "$inside/export_pcd.py" "$inside" 2>&1 | tee "$run/checks/export-pcd-console.log"
    ;;
  downsample)
    once
    ros python3 "$inside/downsample_pcd.py" "$inside" 2>&1 | tee "$run/checks/downsample-console.log"
    ;;
  trajectory)
    once
    ros python3 "$inside/trajectory.py" "$inside" 2>&1 | tee "$run/checks/trajectory-export-console.log"
    ;;
  end-status)
    ros python3 "$inside/end_status.py" "$inside"
    ;;
  compare-pcd)
    once
    ros python3 "$inside/compare_pcd.py" "$inside" 2>&1 | tee "$run/checks/compare-pcd-console.log"
    ;;
  *) echo 'Unknown action'; exit 1 ;;
esac
