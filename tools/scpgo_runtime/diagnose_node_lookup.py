#!/usr/bin/env python3
"""Execute only resource resolution and launch command expansion, never nodes."""
import hashlib
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys
import catkin.find_in_workspaces
import roslib.packages

out = Path(sys.argv[1])
out.mkdir(exist_ok=False)
prefixes = {'original': '/ws/devel_scpgo:/ws/devel:/opt/ros/noetic',
            'frontend_process': '/ws/devel:/ws/devel_scpgo:/opt/ros/noetic'}
results = {}
for label, prefix in prefixes.items():
    os.environ['CMAKE_PREFIX_PATH'] = prefix
    results[label] = dict(cmake_prefix_path=prefix,
        frontend=roslib.packages.find_node('fast_lio','fastlio_mapping'),
        backend=roslib.packages.find_node('aloam_velodyne','alaserPGO'))
assert results['original']['frontend'] == []
assert results['original']['backend'] == ['/ws/devel_scpgo/lib/aloam_velodyne/alaserPGO']
assert results['frontend_process']['frontend'] == ['/ws/devel/lib/fast_lio/fastlio_mapping']
assert results['frontend_process']['backend'] == []
commands = {}
for label, node, args in [
    ('frontend_process','/laserMapping',['/run/scpgo/config/frontend.launch']),
    ('original','/scpgo/alaserPGO',['/run/scpgo/config/backend.launch','run_dir:=/run/scpgo'])]:
    env = dict(os.environ, CMAKE_PREFIX_PATH=prefixes[label])
    r = subprocess.run(['roslaunch','--args='+node]+args, env=env, text=True, capture_output=True)
    assert r.returncode == 0, r.stdout+r.stderr
    commands[node] = r.stdout.strip()
    (out/(label+'-launch-args.txt')).write_text(r.stdout+r.stderr)
frontend = Path('/ws/devel/lib/fast_lio/fastlio_mapping')
digest = hashlib.sha256(frontend.read_bytes()).hexdigest()
assert digest == '607d6fc36c4efeaefbb8d375e23fbe199da8356d014f75fbe8ca802f289773be'
ldd = subprocess.run(['ldd',str(frontend)],text=True,capture_output=True)
assert ldd.returncode == 0 and 'not found' not in ldd.stdout
(out/'frontend-ldd.txt').write_text(ldd.stdout+ldd.stderr)
(out/'installed-find-resource.py.txt').write_text(inspect.getsource(roslib.packages.find_resource))
(out/'installed-catkin-find.py.txt').write_text(inspect.getsource(catkin.find_in_workspaces.find_in_workspaces))
report=dict(status='ROOT_CAUSE_REPRODUCED_PROCESS_ORDER_FIX_VERIFIED',lookups=results,
    resolved_launch_commands=commands,frontend_sha256=digest,frontend_executable=os.access(frontend,os.X_OK),
    libraries_resolved=True,algorithm_started=False,playback_started=False,
    diagnosis='Shared source space causes first_matching_workspace_only to mask the other devel executable.')
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
