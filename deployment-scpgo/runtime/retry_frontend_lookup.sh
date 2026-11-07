#!/usr/bin/env bash
# Explicit manual retry after the documented preplay node-lookup failure.
# Leaves the original frozen run.sh, failed log and frontend-once marker intact.
set -euo pipefail
run=${1:?Pass the prepared run directory}
run=$(cd -- "$run" && pwd)
[[ "$run" == /home/lw/slam-research/results/scpgo/scpgo-corridor01-20261105-001 ]]
name=scpgo-corridor01-20261105-001-container
python3 "$run/guard.py" "$run" ready
python3 - "$run" <<'PY'
from pathlib import Path
import sys
run=Path(sys.argv[1])
assert (run/'checks/frontend-once').is_dir()
log=(run/'checks/frontend-console.log').read_text(errors='replace')
assert 'Cannot locate node of type [fastlio_mapping]' in log
for name in ['backend','raw','PCD','Log']:
    assert not any(p.is_file() for p in (run/name).rglob('*')), 'Existing output: '+name
PY
# -i is necessary to supply this read-only Python preflight through stdin.
docker exec -i "$name" /entrypoint-scpgo.sh python3 - <<'PY'
import rosgraph
master=rosgraph.Master('/frontend_retry_preflight')
pubs,subs,services=master.getSystemState()
nodes={n for rows in [pubs,subs,services] for topic,names in rows for n in names}
assert '/laserMapping' not in nodes and '/scpgo/alaserPGO' not in nodes, nodes
for topic,names in pubs:
    assert topic not in ['/clock','/livox/avia/lidar','/livox/avia/imu'], (topic,names)
PY
docker exec "$name" /entrypoint-scpgo.sh env \
  CMAKE_PREFIX_PATH=/ws/devel:/ws/devel_scpgo:/opt/ros/noetic \
  roslaunch --args=/laserMapping /run/scpgo/config/frontend.launch
mkdir -- "$run/checks/frontend-lookup-retry-once"
printf '%s frontend-lookup-retry start\n' "$(date -Is)" >> "$run/checks/actions.log"
trap 'rc=$?; printf "%s frontend-lookup-retry exit=%s\n" "$(date -Is)" "$rc" >> "$run/checks/actions.log"' EXIT
docker exec -it "$name" /entrypoint-scpgo.sh env \
  CMAKE_PREFIX_PATH=/ws/devel:/ws/devel_scpgo:/opt/ros/noetic \
  roslaunch /run/scpgo/config/frontend.launch \
  2>&1 | tee "$run/checks/frontend-lookup-retry-console.log"
