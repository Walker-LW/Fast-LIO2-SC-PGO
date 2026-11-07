"""Offline bag audit: no master, algorithm or player is started."""
import hashlib, json
from pathlib import Path
import rosbag

path=Path('/datasets/Corridor01.bag')
out=Path('/audit/input-audit-20261103-001.json')
if out.exists(): raise FileExistsError(out)
report={'bag':str(path),'bytes':path.stat().st_size,'topics':{},'selected':{}}
with rosbag.Bag(str(path)) as bag:
    report.update(start=bag.get_start_time(),end=bag.get_end_time(),duration=bag.get_end_time()-bag.get_start_time())
    report['topics']={k:dict(type=v.msg_type,count=v.message_count) for k,v in bag.get_type_and_topic_info().topics.items()}
    for topic in ['/livox/avia/lidar','/livox/avia/imu']:
        n=0;first=last=None;maxgap=0;noninc=0;last_offset=None;sample=[]
        for _,m,t in bag.read_messages(topics=[topic]):
            ns=m.header.stamp.to_nsec()
            if last is not None:
                noninc+=int(ns<=last);maxgap=max(maxgap,ns-last)
            first=ns if first is None else first
            n+=1;last=ns
            if topic.endswith('lidar'):
                last_offset=max((p.offset_time for p in m.points),default=0)
                if n in [1,51,201,601]:
                    eligible=[p for p in m.points[1:] if p.line<6 and (p.tag&48) in [0,16]][2::3]
                    sample.append(dict(frame=n,relative_header_s=(ns-first)/1e9,raw_points=len(m.points),
                        sampled_line_tag_candidates=len(eligible),
                        gt1=sum(p.x*p.x+p.y*p.y+p.z*p.z>1 for p in eligible),
                        gt4=sum(p.x*p.x+p.y*p.y+p.z*p.z>16 for p in eligible)))
        if n==0: raise ValueError('No messages: '+topic)
        report['selected'][topic]=dict(count=n,first_header_ns=first,last_header_ns=last,max_gap_ns=maxgap,nonincreasing=noninc,
                                       last_raw_point_offset_ns=last_offset,candidate_samples=sample)
digest=hashlib.sha256()
with path.open('rb') as f:
    for data in iter(lambda:f.read(8*1024*1024),b''):digest.update(data)
report['sha256']=digest.hexdigest()
report['limits']='Decoded all selected messages and checked header ordering; did not validate all XYZ/IMU values, every point offset, calibration or synchronization. Candidate samples are offline counts, not actual deskew/map matching. SHA identifies local input only.'
with out.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
