#!/usr/bin/env python3
"""No master: validate effective params and node remaps via installed ROS parser."""
import json
import os
from pathlib import Path
import subprocess
import sys
import roslaunch
import yaml

run = Path(sys.argv[1])
frontend = yaml.safe_load((run / 'config/frontend-expanded.yaml').read_text())
backend = yaml.safe_load((run / 'config/backend-expanded.yaml').read_text())
expected = {'keyframe_meter_gap': 2.0, 'keyframe_deg_gap': 10.0,
            'sc_dist_thres': 0.2, 'sc_max_radius': 80.0,
            'mapviz_filter_size': 0.4, 'save_directory': str(run) + '/backend/'}
assert backend == {'/scpgo/' + k: v for k, v in expected.items()}, backend
assert frontend['/preprocess/blind'] == 1.0
assert frontend['/publish/scan_bodyframe_pub_en'] is True
assert frontend['/publish/scan_publish_en'] is True
assert frontend['/use_sim_time'] is True
meta = json.loads((run / 'config/manifest.json').read_text())
assert frontend['/pcd_save/pcd_save_en'] == meta['online_pcd']
reference = yaml.safe_load((run / 'config/reference-runtime.yaml').read_text())
allowed = {'/publish/scan_bodyframe_pub_en', '/publish/scan_publish_en', '/pcd_save/pcd_save_en'}
diffs = {}
for key, value in frontend.items():
    old = reference
    try:
        for part in key.strip('/').split('/'):
            old = old[part]
    except (KeyError, TypeError):
        raise AssertionError('Missing reference parameter ' + key)
    if value != old:
        assert key in allowed, (key, old, value)
        diffs[key] = {'reference': old, 'configured': value}
config = roslaunch.config.load_config_default([(str(run / 'config/backend.launch'),
             ['run_dir:=' + str(run)])], None)
assert len(config.nodes) == 1
node = config.nodes[0]
assert node.namespace == '/scpgo/' and node.name == 'alaserPGO'
assert dict(node.remap_args) == {'/aft_mapped_to_init': '/Odometry',
    '/velodyne_cloud_registered_local': '/cloud_registered_body', '/gps/fix': '/scpgo/unused_gps'}
resolved = {}
for name, prefix, launch, extra, executable in [
    ('frontend', '/ws/devel:/ws/devel_scpgo:/opt/ros/noetic', 'frontend.launch', [], '/ws/devel/lib/fast_lio/fastlio_mapping'),
    ('backend', '/ws/devel_scpgo:/ws/devel:/opt/ros/noetic', 'backend.launch', ['run_dir:='+str(run)], '/ws/devel_scpgo/lib/aloam_velodyne/alaserPGO')]:
    node_name = '/laserMapping' if name == 'frontend' else '/scpgo/alaserPGO'
    result = subprocess.run(['roslaunch', '--args='+node_name, str(run/'config'/launch)] + extra,
        env=dict(os.environ, CMAKE_PREFIX_PATH=prefix), text=True, capture_output=True)
    assert result.returncode == 0 and executable in result.stdout, result.stdout+result.stderr
    resolved[name] = result.stdout.strip()
print(json.dumps({'status': 'PASS', 'backend_effective_parameters': backend,
                  'frontend_reference_differences': diffs, 'remaps': dict(node.remap_args),
                  'resolved_launch_commands': resolved,
                  'ros_master_started': False, 'algorithm_started': False}, indent=2))
