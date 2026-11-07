"""Checks inside the experiment container. No estimator or playback start."""
from pathlib import Path
import hashlib
import json
import sys
import time
import numpy as np
import rosgraph
import rospy
from roslib.packages import find_node
import yaml

RUN = Path('/run/scpgo')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(name, report):
    with (RUN / 'checks' / name).open('x') as f: json.dump(report, f, indent=2)


def identity():
    m = json.loads((RUN / 'config/manifest.json').read_text())
    paths = {'fast_lio': '/ws/devel/lib/fast_lio/fastlio_mapping', 'aloam_velodyne': '/ws/devel_scpgo_geometry/lib/aloam_velodyne/alaserPGO'}
    import os
    prefix = os.environ.get('CMAKE_PREFIX_PATH', '')
    for package, path in paths.items():
        overlay = '/ws/devel' if package == 'fast_lio' else '/ws/devel_scpgo_geometry'
        os.environ['CMAKE_PREFIX_PATH'] = overlay + ':/ws/devel_scpgo_geometry:/ws/devel:/opt/ros/noetic'
        assert find_node(package, Path(path).name) == [path], (package, find_node(package, Path(path).name))
        assert sha(Path(path)) == m['binaries'][path[len('/ws/'):]]
    os.environ['CMAKE_PREFIX_PATH'] = prefix
    for rel, digest in m['frozen'].items(): assert sha(RUN / rel) == digest, rel
    print('IDENTITY_AND_FROZEN_CONFIG_PASS')


def ready():
    identity()
    master = rosgraph.Master('/corridor02_preplay_check')
    pubs, subs, _ = master.getSystemState()
    pubs, subs = dict(pubs), dict(subs)
    for topic in ['/clock', '/livox/avia/lidar', '/livox/avia/imu', '/scpgo/unused_gps']:
        assert not pubs.get(topic), (topic, pubs.get(topic))
    for topic in ['/Odometry', '/cloud_registered_body']:
        assert '/laserMapping' in pubs.get(topic, []), topic
        for node in ['/scpgo/alaserPGO', '/scpgo_record', '/scpgo_interface_watch']:
            assert node in subs.get(topic, []), (topic, node)
    for label in ['frontend', 'backend']:
        expected = yaml.safe_load((RUN / f'config/{label}-expanded.yaml').read_text())
        for key, value in expected.items():
            actual = rospy.get_param(key)
            assert actual == value, (key, actual, value)
    assert rospy.get_param('/preprocess/blind') == 1
    assert rospy.get_param('/pcd_save/pcd_save_en') is False
    write('preplay-' + str(time.time_ns()) + '.json', dict(status='PREPLAY_GRAPH_AND_PARAMETERS_PASS', publishers=pubs, subscribers=subs))
    print('PREPLAY_GRAPH_AND_PARAMETERS_PASS')


def complete():
    info = yaml.safe_load((RUN / 'config/input-info.yaml').read_text())
    expected = {x['topic']: x['messages'] for x in info['topics']}
    observed = json.loads((RUN / 'checks/interface-observer.jsonl').read_text().splitlines()[-1])
    for topic, key in [('/livox/avia/lidar', 'lidar'), ('/livox/avia/imu', 'imu')]:
        assert observed['counts'].get(key) == expected[topic], (key, observed['counts'], expected)
    assert observed['counts'].get('odom', 0) == observed['counts'].get('body') == observed['exact_stamp_pairs'] > 1
    assert not observed['errors'] and observed['odom_without_body'] == observed['body_without_odom'] == 0
    last_output=observed['last']['body']['stamp_ns']
    assert observed['last']['lidar']['stamp_ns'] <= last_output <= observed['last']['imu']['stamp_ns'], \
        'Frontend output has not reached the final raw scan time window. Keep clock running and inspect; do not stop early.'
    write('input-complete.json', dict(observer=observed, input_topic_counts=expected, checked_wall_time_ns=time.time_ns(),
        tail_since_last_imu_wall_s=time.monotonic()-observed['last']['imu']['wall_monotonic'],
        limit='External input receipts and frontend pairing, not per-input internal processing proof.'))
    print('INPUT_COMPLETE_PASS:', observed['counts'])


def saved():
    scans = sorted((RUN / 'backend/Scans').glob('*.pcd'))
    assert len(scans) > 1 and [p.name for p in scans] == [f'{i:06d}.pcd' for i in range(len(scans))]
    matrices = [np.loadtxt(RUN / 'backend' / name, ndmin=2) for name in ['odom_poses.txt', 'optimized_poses.txt']]
    times = np.loadtxt(RUN / 'backend/times.txt', ndmin=1)
    assert all(p.shape == (len(scans), 12) and np.isfinite(p).all() for p in matrices)
    assert len(times) == len(scans) and np.all(np.diff(times) > 0)
    files = scans + [RUN / 'backend' / n for n in ['odom_poses.txt', 'optimized_poses.txt', 'times.txt']]
    write('end-before-stop.json', dict(keyframes=len(scans), checked_wall_time_ns=time.time_ns(), saved_sha256={str(p.relative_to(RUN)): sha(p) for p in files}))
    print('SAVED_KEYFRAMES_AND_POSES_PASS:', len(scans))


if __name__ == '__main__':
    {'identity': identity, 'ready': ready, 'input-complete': complete, 'save-check': saved}[sys.argv[1]]()
