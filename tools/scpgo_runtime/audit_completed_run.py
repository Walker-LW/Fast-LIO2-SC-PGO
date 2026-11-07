#!/usr/bin/env python3
"""Read archived run inputs only; never starts ROS nodes or changes results."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sys
import numpy as np
import rosbag
from export_keyframe_maps import read_pcd, sha

run, output = map(Path, sys.argv[1:3])
assert not (output / 'report.json').exists()
report = {'scope': 'Single completed Corridor01 run; structural/descriptive audit, no GT accuracy claim.',
          'exclusions_or_repairs': 'none'}
report['controls'] = json.loads((run / 'checks/playback-controls.json').read_text())
snapshots = [json.loads(s) for s in (run / 'checks/interface-observer.jsonl').read_text().splitlines()]
report['observer_final'] = snapshots[-1]
assert snapshots[-1]['counts']['lidar'] == 4038 and snapshots[-1]['counts']['imu'] == 82126
paths = sorted((run / 'raw').glob('*.bag'))
assert paths and not list((run / 'raw').glob('*.active'))
counts = Counter()
stamps = {'/Odometry': [], '/cloud_registered_body': []}
clouds = dict(points=0, invalid_xyzi_points=0, invalid_frames=0, layout_errors=0)
poses_by_stamp = {}
bag_inventory = []
for path in paths:
    with rosbag.Bag(str(path)) as bag:
        info = bag.get_type_and_topic_info().topics
        local = {k: v.message_count for k, v in info.items()}
        counts.update(local)
        bag_inventory.append(dict(name=path.name, bytes=path.stat().st_size, topic_counts=local))
        for topic, msg, received in bag.read_messages(topics=list(stamps)):
            stamp = msg.header.stamp.to_nsec()
            stamps[topic].append(stamp)
            if topic == '/Odometry':
                assert msg.header.frame_id == 'camera_init' and msg.child_frame_id == 'body'
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                pose = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                assert np.isfinite(pose).all()
                poses_by_stamp[stamp] = pose
            else:
                clouds['invalid_frames'] += int(msg.header.frame_id != 'body')
                fields = {f.name:f for f in msg.fields}
                assert len(msg.data) == msg.row_step * msg.height
                cols = []
                for name in ['x','y','z','intensity']:
                    f = fields[name]
                    assert f.datatype == 7 and f.count == 1
                    cols.append(np.ndarray((msg.height,msg.width), dtype='>f4' if msg.is_bigendian else '<f4',
                        buffer=msg.data, offset=f.offset, strides=(msg.row_step,msg.point_step)).reshape(-1))
                points = np.column_stack(cols)
                clouds['points'] += len(points)
                clouds['invalid_xyzi_points'] += int(np.count_nonzero(~np.isfinite(points).all(axis=1)))
    print('Read bag:', path.name, flush=True)
assert stamps['/Odometry'] == stamps['/cloud_registered_body']
assert all(b>a for a,b in zip(stamps['/Odometry'],stamps['/Odometry'][1:]))
assert clouds['invalid_frames'] == clouds['invalid_xyzi_points'] == 0
report['bags'] = bag_inventory
report['recorded_topic_counts'] = dict(counts)
report['frontend_pairing'] = dict(exact_ordered_pairs=len(stamps['/Odometry']),
    first_stamp_ns=stamps['/Odometry'][0],last_stamp_ns=stamps['/Odometry'][-1], **clouds)
backend = (run / 'checks/backend-console.log').read_text(errors='replace')
report['backend_log'] = dict(sc_candidate_events=backend.count('Loop detected!'),
    icp_pass_events=backend.count('ICP fitness test passed'),icp_reject_events=backend.count('ICP fitness test failed'),
    isam_updates=backend.count('running isam2 optimization'),gps_factor_messages=backend.count('GPS factor'),
    backlog_warnings=backend.count('Too many'),pcl_integer_overflow_warnings=backend.count('Integer indices would overflow'),
    terminate_message='terminate called without an active exception' in backend,
    limitation='Event counts are not distinct loop pairs or independently logged graph factors.')
exits = []
for log in (run / 'ros_logs').rglob('roslaunch-*.log'):
    for line in log.read_text(errors='replace').splitlines():
        if 'SIGINT killed with return value' in line:
            match = re.search(r'process\[(.*?)\]: SIGINT killed with return value (-?\d+)',line)
            if match:
                exits.append({'process':match.group(1),'code':int(match.group(2))})
report['process_exit_evidence'] = exits
odom = np.loadtxt(run/'backend/odom_poses.txt').reshape(-1,3,4)
optimized = np.loadtxt(run/'backend/optimized_poses.txt').reshape(-1,3,4)
times = np.loadtxt(run/'backend/times.txt')
scans = sorted((run/'backend/Scans').glob('*.pcd'))
assert odom.shape == optimized.shape and len(odom) == len(times) == len(scans)
delta = np.linalg.norm(optimized[:,:,3]-odom[:,:,3],axis=1)
rot = optimized[:,:,:3] @ odom[:,:,:3].transpose(0,2,1)
angles = np.degrees(np.arccos(np.clip((np.trace(rot,axis1=1,axis2=2)-1)/2,-1,1)))
report['keyframe_pose_changes'] = dict(keyframes=len(odom),
    translation_m=dict(max=float(delta.max()),mean=float(delta.mean()),median=float(np.median(delta))),
    rotation_deg_from_rounded_matrices=dict(max=float(angles.max()),note='KITTI files contain rounded matrices; not a precision rotation metric.'),
    meaning='Difference between saved original and optimized poses, not GT error or improvement.')
saved = json.loads((run/'derived/keyframe-maps/report.json').read_text())
for source, digest in {**saved['pose_and_time_sha256'],**saved['scan_sha256']}.items():
    assert sha(Path(source)) == digest, source
maps = [read_pcd(run/'derived/keyframe-maps'/name) for name in ['before_pgo.pcd','after_pgo.pcd']]
assert maps[0].shape == maps[1].shape == (saved['points_per_map'],4)
assert np.array_equal(maps[0][:,3],maps[1][:,3])
for row in saved['outputs']:
    assert sha(run/'derived/keyframe-maps'/row['name']) == row['sha256']
movement = np.linalg.norm(maps[1][:,:3].astype('f8')-maps[0][:,:3].astype('f8'),axis=1)
report['maps'] = dict(keyframes=len(scans),points_per_map=len(maps[0]),
    archived_input_and_output_hashes_verified=True,intensity_identical=True,
    bounds=[dict(label=label,min_xyz=m[:,:3].min(axis=0).tolist(),max_xyz=m[:,:3].max(axis=0).tolist())
        for label,m in zip(['before_pgo','after_pgo'],maps)],
    same_point_displacement_m=dict(max=float(movement.max()),median=float(np.median(movement))),
    meaning='Corresponding-point position change only; not map accuracy.')
report['status'] = 'CAPTURE_AND_MAP_STRUCTURE_VERIFIED_BACKEND_EXIT_ABNORMAL'
with (output/'report.json').open('x') as stream:
    json.dump(report,stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k not in ['observer_final','bags']},indent=2))
