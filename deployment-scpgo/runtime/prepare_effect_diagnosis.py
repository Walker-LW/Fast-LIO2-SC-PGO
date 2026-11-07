"""Build a complete event inventory and export a declared diagnostic sample."""
from collections import deque, Counter
from pathlib import Path
import bisect
import csv
import json
import re
import sys
import numpy as np
import rosbag
from export_keyframe_maps import read_pcd

run, out = map(Path, sys.argv[1:3])
poses = np.loadtxt(run/'backend/odom_poses.txt').reshape(-1,3,4)
queue, events = deque(), []
text=(run/'checks/backend-console.log').read_text(errors='replace')
for line_number,line in enumerate(text.splitlines(),1):
    match=re.search(r'Loop detected! - between (\d+) and (\d+)',line)
    if match:
        queue.append(tuple(map(int,match.groups())))
    match=re.search(r'ICP fitness test (passed|failed) \(([^ ]+)',line)
    if match:
        prev,curr=queue.popleft()
        events.append(dict(event=len(events),previous=prev,current=curr,result=match[1],
            log_score=float(match[2]),log_line=line_number,
            odom_separation_m=float(np.linalg.norm(poses[prev,:,3]-poses[curr,:,3]))))
passed=[e for e in events if e['result']=='passed']
# Predeclared rule: first accepted, accepted with largest odom separation,
# first occurrence of the terminal rejected pair. No effect-based selection.
selected={passed[0]['event'], max(passed,key=lambda e:e['odom_separation_m'])['event'],
    next(e['event'] for e in events if (e['previous'],e['current'])==(events[-1]['previous'],events[-1]['current']))}
with (out/'events.csv').open('x',newline='') as stream:
    writer=csv.DictWriter(stream,fieldnames=list(events[0]))
    writer.writeheader(); writer.writerows(events)

def xyzi(msg):
    fields={f.name:f for f in msg.fields}
    return np.column_stack([np.ndarray((msg.height,msg.width),dtype='>f4' if msg.is_bigendian else '<f4',
        buffer=msg.data,offset=fields[k].offset,strides=(msg.row_step,msg.point_step)).reshape(-1)
        for k in ['x','y','z','intensity']])

def save(path,points):
    count=len(points)
    header=('VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        'WIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n')%(count,count)
    with path.open('xb') as stream:
        stream.write(header.encode('ascii')); stream.write(points.astype('<f4').tobytes())

times=np.loadtxt(run/'backend/times.txt')
key_stamps=[int(round(t*1e9)) for t in times]
match_results={}
debug_index=Counter()
for path in sorted((run/'raw').glob('*.bag')):
    with rosbag.Bag(str(path)) as bag:
        for topic,msg,received in bag.read_messages(topics=['/cloud_registered_body','/loop_scan_local','/loop_submap_local']):
            if topic=='/cloud_registered_body':
                stamp=msg.header.stamp.to_nsec()
                at=bisect.bisect_left(key_stamps,stamp)
                nearest=min([i for i in [at-1,at] if 0<=i<len(times)],key=lambda i:abs(key_stamps[i]-stamp))
                if abs(key_stamps[nearest]-stamp)<=512:
                    points=xyzi(msg)
                    archived=read_pcd(run/'backend/Scans'/('%06d.pcd'%nearest))
                    assert np.array_equal(points,archived), nearest
                    match_results[nearest]=dict(stamp_difference_ns=int(key_stamps[nearest]-stamp),points=len(points))
            else:
                index=debug_index[topic]; debug_index[topic]+=1
                if index in selected:
                    label='source' if topic=='/loop_scan_local' else 'target'
                    save(out/('event-%03d-%s.pcd'%(index,label)),xyzi(msg))
    print('Read',path.name,flush=True)
assert len(match_results)==264
assert sum(v['points'] for v in match_results.values())==1391336
assert debug_index['/loop_scan_local']==debug_index['/loop_submap_local']==len(events)
report=dict(status='ARCHIVED_INPUTS_VERIFIED',events=events,
    selected_events=[events[i] for i in sorted(selected)],
    selection_rule='First accepted; accepted with greatest frontend-odom separation; first terminal-pair rejection.',
    accepted_events=len(passed),accepted_distinct_pairs=len({(e['previous'],e['current']) for e in passed}),
    pending_candidates=list(queue),keyframe_clouds_exact_xyzi_match_body_messages=True,
    matched_keyframes=len(match_results),saved_keyframe_points=sum(v['points'] for v in match_results.values()),
    max_timestamp_rounding_ns=max(abs(v['stamp_difference_ns']) for v in match_results.values()),
    diagnostic_pairing_note='ICP debug source/target sequences paired by FIFO event order; counts verified. Probe score reproduction is required next.')
(out/'prepared.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='events'},indent=2))
