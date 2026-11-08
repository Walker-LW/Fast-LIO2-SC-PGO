"""Read-only late-shutdown audit. No replay, registration, or optimization."""
from collections import Counter, deque
from decimal import Decimal
from pathlib import Path
import csv
import hashlib
import json
import re
import sys
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RUN=Path('/run/scpgo');OUT=Path('/out/verified')
sys.path.insert(0,str(RUN))
from export_keyframe_maps import sha,read_pcd


def angle(q1,q2):
    a=np.asarray(q1,dtype=float);b=np.asarray(q2,dtype=float)
    a=a/np.linalg.norm(a,axis=-1,keepdims=True);b=b/np.linalg.norm(b,axis=-1,keepdims=True)
    b=b*np.where(np.sum(a*b,axis=-1,keepdims=True)<0,-1,1)
    return np.degrees(4*np.arctan2(np.linalg.norm(a-b,axis=-1),np.linalg.norm(a+b,axis=-1)))


def tum(path):
    rows=[s.split() for s in path.read_text().splitlines() if s.strip()]
    return np.array([int(Decimal(x[0])*10**9) for x in rows],dtype=np.int64),np.array([[float(v) for v in x[1:]] for x in rows])


report=json.loads((RUN/'derived/quantitative/report.json').read_text())
manifest=json.loads((RUN/'config/manifest.json').read_text())
for rel,digest in manifest['frozen'].items():assert sha(RUN/rel)==digest,rel
for rel,digest in json.loads((RUN/'checks/end-before-stop.json').read_text())['saved_sha256'].items():assert sha(RUN/rel)==digest,rel
assert not list((RUN/'raw').glob('*.active'))
saved=json.loads((RUN/'derived/keyframe-maps/report.json').read_text())
for row in saved['outputs']:assert sha(RUN/'derived/keyframe-maps'/row['name'])==row['sha256']

log=(RUN/'checks/backend-console.log').read_text(errors='replace')
# stdout from iSAM and ICP threads can interleave inside a fitness line.
# Remove only this exact independently emitted marker, retain all raw evidence.
marker='running isam2 optimization ...'
clean=log.replace(marker,'')
pattern=re.compile(r'Loop detected! - between\s+(\d+)\s+and\s+(\d+)|\[SC loop\] ICP fitness test\s+(passed|failed)\s+\(\s*([-+\d.eE]+)')
queue=deque();events=[];detections=[];max_pending=0
for m in pattern.finditer(clean):
    if m.group(1) is not None:
        p=(int(m.group(1)),int(m.group(2)));queue.append(p);detections.append(p);max_pending=max(max_pending,len(queue))
    else:
        assert queue,'Result without a preceding candidate'
        pair=queue.popleft();events.append(dict(pair=pair,status=m.group(3),fitness_m2=float(m.group(4))))
assert len(detections)==log.count('Loop detected! - between')
assert len(events)==log.count('ICP fitness test passed')+log.count('ICP fitness test failed')

max_nodes=0;snapshots=[];lengths=Counter();debug_times=[];target_times=[];map_hashes=[]
last_input_receipt=0;last_input_header=0;input_counts=Counter()
for file in sorted((RUN/'raw').glob('*.bag')):
    with rosbag.Bag(str(file)) as bag:
        for topic,msg,t in bag.read_messages(topics=['/Odometry','/cloud_registered_body','/aft_pgo_path','/loop_scan_local','/loop_submap_local','/aft_pgo_map']):
            if topic in ['/Odometry','/cloud_registered_body']:
                input_counts[topic]+=1;last_input_receipt=max(last_input_receipt,t.to_nsec());last_input_header=max(last_input_header,msg.header.stamp.to_nsec())
            elif topic=='/loop_scan_local':debug_times.append(t.to_nsec())
            elif topic=='/loop_submap_local':target_times.append(t.to_nsec())
            elif topic=='/aft_pgo_map':map_hashes.append((t.to_nsec(),hashlib.sha256(msg.data).hexdigest(),msg.width*msg.height))
            else:
                n=len(msg.poses);lengths[n]+=1
                if n>max_nodes:max_nodes=n;snapshots=[]
                if n==max_nodes:
                    a=np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z,p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w] for p in msg.poses])
                    snapshots.append((t.to_nsec(),a))
    print('READ',file.name,flush=True)
assert input_counts['/Odometry']==input_counts['/cloud_registered_body']==2930
assert len(debug_times)==len(target_times)==len(events), (len(debug_times),len(target_times),len(events))
assert max_nodes==saved['keyframes']-1
for event,t in zip(events,debug_times):event['debug_source_receipt_ns']=t
tail_events=[e for e in events if e['debug_source_receipt_ns']>=last_input_receipt]
before_events=[e for e in events if e['debug_source_receipt_ns']<last_input_receipt]
accepted=[e for e in events if e['status']=='passed']
old_stamps,old_poses=tum(Path('/baseline/derived/trajectory.tum'))
new_stamps,new_poses=tum(RUN/'derived/quantitative/frontend-full.tum')
assert old_poses.shape==new_poses.shape
assert np.array_equal(old_stamps,new_stamps)
old_comparison=dict(count=len(old_stamps),integer_ns_stamps_equal=True,all_pose_components_equal=bool(np.array_equal(old_poses,new_poses)),
    max_position_difference_m=float(np.linalg.norm(old_poses[:,:3]-new_poses[:,:3],axis=1).max()),
    max_rotation_difference_deg=float(angle(old_poses[:,3:],new_poses[:,3:]).max()),
    old_file_sha256=sha(Path('/baseline/derived/trajectory.tum')),new_file_sha256=sha(RUN/'derived/quantitative/frontend-full.tum'),
    note='Numeric direct comparison in the same frontend frame; no alignment or rescaling. File bytes may differ through formatting.')

prior=[x for x in snapshots if x[0]<=last_input_receipt]
reference=prior[-1] if prior else next(x for x in snapshots if x[0]>=last_input_receipt)
tail=[x for x in snapshots if x[0]>=last_input_receipt]
assert tail
changes=[]
for t,a in tail:
    assert a.shape==reference[1].shape
    changes.append((t,float(np.linalg.norm(a[:,:3]-reference[1][:,:3],axis=1).max()),float(angle(a[:,3:],reference[1][:,3:]).max())))
ref_bytes=reference[1].tobytes()
tail_equal=all(a.tobytes()==ref_bytes for _,a in tail)
scans=sorted((RUN/'backend/Scans').glob('*.pcd'))
last_points=len(read_pcd(scans[-1]))
tail_map_hashes=[x for x in map_hashes if x[0]>=last_input_receipt]
audit=dict(status='READ_ONLY_LATE_STOP_AND_EFFECT_AUDIT',
    experiment='/home/lw/slam-research/results/scpgo/scpgo-corridor02-geometry-20261108-001',
    integrity=dict(frozen_sources_unchanged=True,saved_307_files_unchanged=True,map_output_hashes_verified=True,no_active_bag=True,exact_frontend_pairs=2930),
    frontend_vs_previous_baseline=old_comparison,
    late_stop=dict(additional_since_original_finish_attempt_s=report['closure_check']['recovery']['additional_tail_since_original_input_check_s'],
        last_frontend_message_receipt_ns=last_input_receipt,last_frontend_header_ns=last_input_header,
        last_pgo_path_receipt_ns=tail[-1][0],recorded_path_tail_duration_s=(tail[-1][0]-last_input_receipt)/1e9,
        path_reference_receipt_ns=reference[0],reference_offset_from_last_frontend_receipt_s=(reference[0]-last_input_receipt)/1e9,
        published_path_max_nodes=max_nodes,final_saved_nodes=saved['keyframes'],tail_path_messages=len(tail),
        all_tail_path_pose_arrays_exactly_equal_to_reference=tail_equal,
        maximum_tail_position_change_m=max(x[1] for x in changes),maximum_tail_orientation_change_deg=max(x[2] for x in changes),
        terminal_unpublished_node=len(scans)-1,terminal_node_points=last_points,terminal_node_point_fraction=last_points/saved['points_per_map'],
        path_limit='Source intentionally publishes indices <recentIdxUpdated: 303/304 nodes. No earlier complete 304-node saved snapshot, so last node history cannot be verified directly.',
        recorded_map_messages_in_tail=len(tail_map_hashes),distinct_map_payload_hashes_in_tail=len(set(x[1] for x in tail_map_hashes)),
        total_processed_icp_events=len(events),during_input_or_before_tail_events=len(before_events),in_tail_processed_events=len(tail_events),
        in_tail_passed_events=sum(x['status']=='passed' for x in tail_events),in_tail_failed_events=sum(x['status']=='failed' for x in tail_events),
        tail_pairs=sorted(set(tuple(x['pair']) for x in tail_events)),
        last_accepted_start_seconds_before_last_frontend_receipt=(last_input_receipt-accepted[-1]['debug_source_receipt_ns'])/1e9 if accepted else None,
        event_time_limit='ICP start times inferred by FIFO order and one-to-one recorded debug source count; result completion timestamps are not independently logged.'),
    corrected_log_parse=dict(removed_exact_isam_markers=log.count(marker),candidate_events=len(detections),unique_candidate_pairs=len(set(detections)),
        icp_results=len(events),passed=sum(e['status']=='passed' for e in events),failed=sum(e['status']=='failed' for e in events),
        unique_passed_pairs=len(set(tuple(e['pair']) for e in events if e['status']=='passed')),pending_pairs=list(queue),max_pending=max_pending,
        normalized_parse_verified_against_all_raw_event_prefix_counts=True,
        note='Original report retained untouched. Exact known iSAM marker removed from a derived log view to recover one interleaved fitness number. Pair identity remains FIFO-derived, not a factor graph dump.'),
    effect={k:report[k] for k in ['keyframes','pose_translation_change_m','pose_rotation_change_deg','start_end_keyframe_position_distance_m','maps']},
    limits=['Movement is not GT accuracy; no ATE/RPE or verified closure-error improvement.',
            'Current run has no old-geometry Corridor02 counterpart; cannot isolate geometry-fix gains on this sequence.',
            'No frame-level Jacobian/localizability diagnostics: neither corridor degeneracy nor packet corruption established.',
            'Late tail affects raw event counts, recording duration and warnings; do not use full-run counts as normal-duration rates.'])

OUT.mkdir(parents=True,exist_ok=False)
with (OUT/'tail-path-changes.csv').open('x',newline='') as f:
    w=csv.writer(f);w.writerow(['receipt_ns','maximum_node_translation_change_m','maximum_node_rotation_change_deg']);w.writerows(changes)
with (OUT/'normalized-icp-events.csv').open('x',newline='') as f:
    w=csv.writer(f);w.writerow(['previous_fifo','current_fifo','status','fitness_m2','debug_source_receipt_ns','phase'])
    w.writerows([*e['pair'],e['status'],e['fitness_m2'],e['debug_source_receipt_ns'],'tail' if e['debug_source_receipt_ns']>=last_input_receipt else 'input'] for e in events)
maps=[read_pcd(RUN/'derived/keyframe-maps'/name) for name in ['before_pgo.pcd','after_pgo.pcd']]
regions=[(-12,8,-16,6),(-13,2,20,36),(50,66,-20,0)]
fig,axes=plt.subplots(3,2,figsize=(12,13),constrained_layout=True)
for row,(xmin,xmax,ymin,ymax) in enumerate(regions):
    for col,(cloud,label) in enumerate(zip(maps,['Before','After fixed PGO'])):
        keep=(cloud[:,0]>=xmin)&(cloud[:,0]<=xmax)&(cloud[:,1]>=ymin)&(cloud[:,1]<=ymax)
        points=cloud[keep];ax=axes[row,col];ax.scatter(points[:,0],points[:,1],s=.08,alpha=.5,c='#22689b')
        ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax),xlabel='X / m',ylabel='Y / m',title=label);ax.set_aspect('equal')
fig.suptitle('Matched XY windows; all in-window points; no extra alignment')
fig.savefig(OUT/'local-all-points.png',dpi=180);plt.close(fig)
audit['local_windows']=regions
with (OUT/'audit.json').open('x') as f:json.dump(audit,f,indent=2)
print(json.dumps({k:v for k,v in audit.items() if k not in ['effect']},indent=2))
