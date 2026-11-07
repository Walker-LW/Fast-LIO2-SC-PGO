"""Completed-run capture audit and descriptive outputs, without GT claims."""
from pathlib import Path
from collections import Counter, deque
from decimal import Decimal
import csv
import hashlib
import json
import math
import re
import sys
import numpy as np
import rosbag
from tf.transformations import quaternion_from_matrix
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from export_keyframe_maps import read_pcd, sha


def summary(values):
    a = np.asarray(values)
    assert len(a) and np.isfinite(a).all()
    return dict(zip(['min', 'median', 'p95', 'p99', 'max'], map(float, np.quantile(a, [0, .5, .95, .99, 1]))))


def project_rotations(poses):
    u, _, vt = np.linalg.svd(poses[:, :, :3])
    d = np.tile(np.eye(3), (len(poses), 1, 1))
    d[:, 2, 2] = np.linalg.det(u @ vt)
    return u @ d @ vt


def angles(a, b):
    r = np.swapaxes(a, 1, 2) @ b
    return np.degrees(np.arccos(np.clip((np.trace(r, axis1=1, axis2=2) - 1) / 2, -1, 1)))


def loop_events(log):
    queue, events = deque(), []
    pairs, unassigned, max_pending = [], 0, 0
    pattern = re.compile(r'Loop detected! - between (\d+) and (\d+)|\[SC loop\] ICP fitness test (passed|failed) \(([-+\d.eE]+)')
    for m in pattern.finditer(log):
        if m.group(1) is not None:
            pair = (int(m.group(1)), int(m.group(2)))
            queue.append(pair); pairs.append(pair); max_pending = max(max_pending, len(queue))
        elif queue:
            pair = queue.popleft()
            events.append([len(events), pair[0], pair[1], m.group(3), float(m.group(4))])
        else: unassigned += 1
    parsed = len(pairs) == log.count('Loop detected! - between') and len(events) + unassigned == log.count('ICP fitness test passed') + log.count('ICP fitness test failed')
    report = dict(candidate_events=log.count('Loop detected! - between'), parsed_unique_candidate_pairs=len(set(pairs)),
        icp_pass_events=log.count('ICP fitness test passed'), icp_reject_events=log.count('ICP fitness test failed'),
        fifo_parse_complete=parsed, unassigned_results=unassigned, pending_pairs=list(queue), max_observed_pending=max_pending,
        unique_passed_pairs_inferred=len(set((e[1],e[2]) for e in events if e[3]=='passed')) if parsed and not unassigned else None,
        interpretation='FIFO-inferred pairs are not independent true-loop labels or a serialized optimizer factor graph.',
        isam_update_messages=log.count('running isam2 optimization'),
        pcl_overflow_warnings=log.count('Integer indices would overflow'),
        backlog_warnings=log.count('Too many loop clousre candidates'),
        abnormal_termination='terminate called without an active exception' in log)
    return report, events


def main(run):
    out = run / 'derived/quantitative'
    assert not out.exists(), 'Refuse to overwrite analysis: ' + str(out)
    closed = json.loads((run / 'checks/closed.json').read_text())
    assert not closed['saved_files_changed_at_stop']
    saved = json.loads((run / 'derived/keyframe-maps/report.json').read_text())
    for name, digest in {**saved['pose_and_time_sha256'], **saved['scan_sha256']}.items(): assert sha(Path(name)) == digest
    for row in saved['outputs']: assert sha(run / 'derived/keyframe-maps' / row['name']) == row['sha256']
    counts, poses, body_stamps, inventory = Counter(), {}, [], []
    points, invalid = 0, 0
    assert not list((run / 'raw').glob('*.active'))
    for path in sorted((run / 'raw').glob('*.bag')):
        with rosbag.Bag(str(path)) as bag:
            local = {k:v.message_count for k,v in bag.get_type_and_topic_info().topics.items()}
            counts.update(local); inventory.append(dict(name=path.name, bytes=path.stat().st_size, topic_counts=local))
            for topic, m, _ in bag.read_messages(topics=['/Odometry', '/cloud_registered_body']):
                stamp = m.header.stamp.to_nsec()
                if topic == '/Odometry':
                    assert m.header.frame_id == 'camera_init' and m.child_frame_id == 'body'
                    assert stamp not in poses, 'Duplicate odometry stamp'
                    p,q=m.pose.pose.position,m.pose.pose.orientation
                    value=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                    assert np.isfinite(value).all() and abs(np.linalg.norm(value[3:])-1)<1e-3
                    poses[stamp]=value
                else:
                    assert m.header.frame_id=='body' and len(m.data)==m.row_step*m.height
                    fields={f.name:f for f in m.fields}; columns=[]
                    for name in ['x','y','z','intensity']:
                        f=fields[name]; assert f.datatype==7 and f.count==1
                        columns.append(np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',
                            buffer=m.data,offset=f.offset,strides=(m.row_step,m.point_step)).reshape(-1))
                    a=np.column_stack(columns);points+=len(a);invalid+=int(np.count_nonzero(~np.isfinite(a).all(axis=1)))
                    body_stamps.append(stamp)
    assert list(poses)==body_stamps and all(b>a for a,b in zip(body_stamps,body_stamps[1:]))
    assert not invalid and len(poses)>1
    capture=json.loads((run/'checks/input-complete.json').read_text())
    observer=capture['observer']
    assert len(poses)==observer['exact_stamp_pairs'], 'Observer/recorded bag counts differ'
    before,after=[np.loadtxt(run/'backend'/name).reshape(-1,3,4) for name in ['odom_poses.txt','optimized_poses.txt']]
    time_strings=(run/'backend/times.txt').read_text().splitlines();times=np.array([float(x) for x in time_strings])
    assert len(before)==len(after)==len(times)==saved['keyframes']
    rb,ra=project_rotations(before),project_rotations(after)
    delta=np.linalg.norm(after[:,:,3]-before[:,:,3],axis=1); angle=angles(rb,ra)
    maps=[read_pcd(run/'derived/keyframe-maps'/n) for n in ['before_pgo.pcd','after_pgo.pcd']]
    assert maps[0].shape==maps[1].shape and np.array_equal(maps[0][:,3],maps[1][:,3])
    movement=np.linalg.norm(maps[1][:,:3].astype(float)-maps[0][:,:3],axis=1)
    near=np.concatenate([np.linalg.norm(read_pcd(p)[:,:3].astype(float),axis=1)<=40 for p in sorted((run/'backend/Scans').glob('*.pcd'))])
    assert len(near)==len(movement)
    backend=(run/'checks/backend-console.log').read_text(errors='replace')
    loops,events=loop_events(backend)
    front=(run/'checks/frontend-console.log').read_text(errors='replace')
    out.mkdir()
    with (out/'frontend-full.tum').open('x') as f:
        for stamp,value in poses.items(): f.write(str(stamp//10**9)+'.'+str(stamp%10**9).zfill(9)+' '+' '.join(format(x,'.17g') for x in value)+'\n')
    for label,p,r in [('before',before,rb),('after',after,ra)]:
        with (out/(label+'-keyframes.tum')).open('x') as f:
            for t,matrix,rot in zip(time_strings,p,r):
                transform=np.eye(4);transform[:3,:3]=rot
                q=quaternion_from_matrix(transform)
                f.write(t.strip()+' '+' '.join(format(x,'.17g') for x in list(matrix[:,3])+list(q))+'\n')
    with (out/'keyframe-changes.csv').open('x',newline='') as f:
        w=csv.writer(f);w.writerow(['frame','stored_time_s','seconds_from_first','translation_change_m','rotation_change_deg']);w.writerows(zip(range(len(times)),time_strings,times-times[0],delta,angle))
    with (out/'icp-events.csv').open('x',newline='') as f:
        w=csv.writer(f);w.writerow(['event','previous_inferred_fifo','current_inferred_fifo','status','logged_fitness_m2']);w.writerows(events)
    fig,axes=plt.subplots(1,2,figsize=(16,7),constrained_layout=True)
    xy=np.concatenate([p[:,:2,3] for p in [before,after]])
    lo,hi=xy.min(0)-5,xy.max(0)+5
    stride=max(1,math.ceil(len(maps[0])/300000))
    for ax,cloud,p,label in zip(axes,maps,[before,after],['Before PGO','After fixed-geometry PGO']):
        a=cloud[::stride];ax.scatter(a[:,0],a[:,1],s=.1,c='#22689b');ax.plot(p[:,0,3],p[:,1,3],c='#e08020',lw=.8)
        ax.scatter(p[0,0,3],p[0,1,3],marker='*',c='red',s=65);ax.scatter(p[-1,0,3],p[-1,1,3],marker='x',c='black',s=35)
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='X / m',ylabel='Y / m',title=label);ax.set_aspect('equal')
    fig.suptitle('Same complete keyframe set and XY window; display sampling only; no extra alignment')
    fig.savefig(out/'before-after.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True,constrained_layout=True)
    axes[0].plot(times-times[0],delta*100);axes[0].set_ylabel('Position change / cm')
    axes[1].plot(times-times[0],angle);axes[1].set_ylabel('Orientation change / deg');axes[1].set_xlabel('Seconds from first keyframe')
    fig.suptitle('Changes from frontend estimates, not ground-truth errors');fig.savefig(out/'pose-changes.png',dpi=180);plt.close(fig)
    # Four exhaustive quadrants of the overview, selected without outcome tuning.
    mid=(lo+hi)/2;regions=[(lo[0],mid[0],lo[1],mid[1]),(mid[0],hi[0],lo[1],mid[1]),(lo[0],mid[0],mid[1],hi[1]),(mid[0],hi[0],mid[1],hi[1])]
    fig,axes=plt.subplots(4,2,figsize=(14,18),constrained_layout=True)
    for row,(xmin,xmax,ymin,ymax) in enumerate(regions):
        for col,(cloud,label) in enumerate(zip(maps,['Before','After'])):
            # Shared original point indices in both panels; viewport crop may differ.
            a=cloud[::stride];m=(a[:,0]>=xmin)&(a[:,0]<=xmax)&(a[:,1]>=ymin)&(a[:,1]<=ymax)
            axes[row,col].scatter(a[m,0],a[m,1],s=.12,c='#22689b');axes[row,col].set(xlim=(xmin,xmax),ylim=(ymin,ymax),xlabel='X / m',ylabel='Y / m',title=label+' / quadrant '+str(row+1));axes[row,col].set_aspect('equal')
    fig.suptitle('Four fixed overview quadrants, identical paired display settings');fig.savefig(out/'quadrant-details.png',dpi=180);plt.close(fig)
    raw_xyz=np.array([v[:3] for v in poses.values()]);steps=np.linalg.norm(np.diff(raw_xyz,axis=0),axis=1)
    coverage=None
    if 'first_stamp_ns' in observer and 'last' in observer:
        coverage=dict(start_gap_from_first_lidar_header_s=(body_stamps[0]-observer['first_stamp_ns']['lidar'])/1e9,
            last_output_minus_last_lidar_begin_header_s=(body_stamps[-1]-observer['last']['lidar']['stamp_ns'])/1e9,
            last_output_minus_last_imu_header_s=(body_stamps[-1]-observer['last']['imu']['stamp_ns'])/1e9,
            note='Offsets against raw message headers; LiDAR header is scan begin, not scan end.')
    report=dict(status='COMPLETED_CAPTURE_AND_MAP_AUDIT',scope='Single Corridor02 run; fixed approved numerical profile; descriptive effect audit.',
        input_observer=observer,input_completion_check=capture,closure_check=closed,recorded_topic_counts=dict(counts),bag_inventory=inventory,
        frontend=dict(exact_recorded_pairs=len(poses),points=points,invalid_xyzi_points=invalid,
            first_stamp_ns=body_stamps[0],last_stamp_ns=body_stamps[-1],header_duration_s=(body_stamps[-1]-body_stamps[0])/1e9,
            coverage_against_observed_raw_headers=coverage,prior_output_count_reference=2930,
            difference_from_prior_count=len(poses)-2930,consecutive_position_step_m=summary(steps),path_length_m=float(steps.sum()),
            largest_step_stamp_pair_ns=[body_stamps[int(np.argmax(steps))],body_stamps[int(np.argmax(steps))+1]],
            no_effective_points_log_calls=front.count('No Effective Points'),no_point_skip_log_calls=front.count('No point, skip this scan')),
        keyframes=len(times),pose_translation_change_m=summary(delta),pose_rotation_change_deg=summary(angle),
        start_end_keyframe_position_distance_m=dict(before=float(np.linalg.norm(before[-1,:,3]-before[0,:,3])),after=float(np.linalg.norm(after[-1,:,3]-after[0,:,3]))),
        maps=dict(points_each=len(maps[0]),same_source_points=True,point_movement_m=summary(movement),
            near_local_range_le40m_points=int(near.sum()),near_local_range_le40m_movement_m=summary(movement[near]) if near.any() else None,
            display_stride=stride,displayed_points_before_crop=len(maps[0][::stride]),xy_min=lo.tolist(),xy_max=hi.tolist(),local_windows=regions,
            outputs=saved['outputs']),backend=loops,
        limits=['No ATE, RPE, or GT improvement percentage. Endpoint distance is descriptive, not verified closure error.',
                'Changes are not accuracy gains; both improvement and degradation require independent reference/geometry evidence.',
                'Known shutdown abort/PCL warnings must be reviewed; complete capture does not establish all factors consumed.',
                'Repeated candidates during keep-alive may affect event counts; raw events and checked stop times retained.',
                'No added frontend geometric-direction instrumentation, so no confirmed degeneracy attribution.',
                'Saved KITTI matrices/time strings are rounded; rotations projected to SO(3) for angle/TUM conversion.',
                'Single run on previously used data; no independent-repeat confidence interval or causal scene conclusion.'])
    with (out/'report.json').open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps({k:report[k] for k in ['status','keyframes','pose_translation_change_m','pose_rotation_change_deg','backend','limits']},indent=2))
    print('QUANTITATIVE_REPORT_AND_PREVIEWS_PASS:',out)


if __name__=='__main__':main(Path(sys.argv[1]))
