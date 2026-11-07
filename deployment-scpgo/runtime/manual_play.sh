#!/usr/bin/env bash
# Only runs when explicitly invoked in the user's interactive WSL terminal.
set -euo pipefail
run=${1:?Pass the prepared run directory}
run=$(cd -- "$run" && pwd)
[[ "$run" == /home/lw/slam-research/results/scpgo/scpgo-corridor01-20261105-001 ]]
name=scpgo-corridor01-20261105-001-container
bash "$run/run.sh" ready
read -r -p '回放范围：输入 full（全序列）或 short（起始片段），无默认值：' mode
extra=()
duration=''
case "$mode" in
  full) ;;
  short)
    read -r -p '起始片段时长（秒，正数）：' duration
    python3 -c 'import math,sys; x=float(sys.argv[1]); assert math.isfinite(x) and x>0' "$duration"
    extra=(--duration "$duration")
    ;;
  *) echo '范围未明确选择，不播放'; exit 2 ;;
esac
read -r -p '回放倍速（正数，例如 1；无默认值）：' rate
python3 -c 'import math,sys; x=float(sys.argv[1]); assert math.isfinite(x) and x>0' "$rate"
mkdir -- "$run/checks/player-once"
python3 - "$run" "$mode" "$duration" "$rate" <<'PY'
import json,sys,time
from pathlib import Path
run=Path(sys.argv[1])
with (run/'checks/playback-controls.json').open('x') as stream:
    json.dump(dict(mode=sys.argv[2],duration_seconds=float(sys.argv[3]) if sys.argv[3] else None,
                   rate=float(sys.argv[4]),start_seconds=0,clock_hz=100,
                   topics=['/livox/avia/lidar','/livox/avia/imu'],wall_time_ns=time.time_ns()),stream,indent=2)
PY
printf '%s manual-player start\n' "$(date -Is)" >> "$run/checks/actions.log"
trap 'rc=$?; printf "%s manual-player exit=%s\n" "$(date -Is)" "$rc" >> "$run/checks/actions.log"' EXIT
echo '播放器将初始暂停。确认启动成功后，按空格开始；末尾保留clock，最后收尾时才Ctrl+C。'
docker exec -it "$name" /entrypoint-scpgo.sh /opt/ros/noetic/lib/rosbag/play \
  --clock --hz 100 --pause --keep-alive -r "$rate" "${extra[@]}" \
  /datasets/Corridor01.bag --topics /livox/avia/lidar /livox/avia/imu \
  __name:=scpgo_input 2>&1 | tee "$run/checks/player-console.log"
