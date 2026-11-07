#!/usr/bin/env python3
"""Preplay graph/parameter checks, without requiring measurements or TF yet."""
import json
from pathlib import Path
import sys
import rosgraph
import rospy
import yaml

run = Path(sys.argv[1])
master = rosgraph.Master('/scpgo_preplay_check')
pubs, subs, services = master.getSystemState()
pubs, subs = dict(pubs), dict(subs)
assert not pubs.get('/clock'), 'A clock publisher already exists; inspect before replay'
assert not pubs.get('/livox/avia/lidar') and not pubs.get('/livox/avia/imu'), 'Input already running'
assert not pubs.get('/scpgo/unused_gps'), 'Unexpected GPS publisher'
for topic in ['/Odometry', '/cloud_registered_body']:
    assert '/laserMapping' in pubs.get(topic, []), (topic, pubs.get(topic))
    assert '/scpgo/alaserPGO' in subs.get(topic, []), (topic, subs.get(topic))
    assert '/scpgo_record' in subs.get(topic, []), (topic, subs.get(topic))
    assert '/scpgo_interface_watch' in subs.get(topic, []), (topic, subs.get(topic))
for topic in ['/aft_pgo_odom', '/aft_pgo_path', '/aft_pgo_map']:
    assert '/scpgo/alaserPGO' in pubs.get(topic, []), (topic, pubs.get(topic))
assert '/laserMapping' in subs.get('/livox/avia/lidar', [])
assert '/laserMapping' in subs.get('/livox/avia/imu', [])
expected = {}
for label in ['frontend', 'backend']:
    expected.update(yaml.safe_load((run / f'config/{label}-expanded.yaml').read_text()))
for name, value in expected.items():
    assert rospy.get_param(name) == value, (name, rospy.get_param(name), value)
topics = dict(master.getPublishedTopics(''))
assert topics['/Odometry'] == 'nav_msgs/Odometry'
assert topics['/cloud_registered_body'] == 'sensor_msgs/PointCloud2'
stamp = __import__('time').time_ns()
out = run / 'checks' / ('preplay-' + str(stamp) + '.json')
with out.open('x') as stream:
    json.dump({'status': 'PREPLAY_GRAPH_PASS', 'parameters': {k: rospy.get_param(k) for k in expected},
               'publishers': pubs, 'subscribers': subs,
               'message_fields_timestamps_coverage_and_loops': 'NOT_YET_TESTED'}, stream, indent=2)
print('PASS: graph and effective ROS parameters. No message or loop validation yet.')
