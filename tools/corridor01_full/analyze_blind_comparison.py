"""Analyze archived runs only; never alter their raw/derived files."""
from pathlib import Path
from decimal import Decimal
import hashlib,json,re
import numpy as np
import rosbag,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

base=Path('/results/baseline')
out=base/'corridor01-blind-ab-20261105-001';out.mkdir()
runs={1:base/'corridor01-full-b1-20261103-001',4:base/'corridor01-full-b4-20261105-001'}
report={'runs':{},'limits':['One independent run per parameter; poses are not independent repetitions.',
 'blind=1 had an auxiliary observer; blind=4 omitted it. Do not claim matched performance workload.',
 'All trajectory quantities are estimates, not GT errors. Log sim times are not exact scan identifiers.',
 'Structural validity and point-wise save consistency do not imply localization accuracy.']}
arrays={};warnings={}
def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()
def stamp_relative(s,start):return float(Decimal(s)-Decimal(str(start)))
for blind,r in runs.items():
    check=json.loads((r/'checks/full-run.json').read_text());start=check['input_bag_start']
    rows=[line.split() for line in (r/'derived/trajectory.tum').read_text().splitlines()]
    a=np.array(rows,dtype=np.float64);t=a[:,0]-start;xyz=a[:,1:4]
    steps=np.linalg.norm(np.diff(xyz,axis=0),axis=1);dt=np.diff(a[:,0]);speed=steps/dt
    text=(r/'checks/algorithm-console.log').read_text(errors='replace')
    events=[];previous_clock=None;pcl=[];skips=[]
    for line in text.splitlines():
        m=re.search(r'\[([0-9]+\.[0-9]+),\s*([0-9]+\.[0-9]+)\]',line)
        if m:previous_clock=stamp_relative(m.group(2),start)
        if 'No Effective Points' in line:events.append(previous_clock)
        if 'No point, skip this scan' in line:skips.append(previous_clock)
        if 'Integer indices would overflow' in line:pcl.append(previous_clock)
    events=np.array(events);warnings[blind]=events;arrays[blind]=(t,xyz,steps,speed)
    result=dict(recorded_count=len(a),first_relative_s=float(t[0]),last_relative_s=float(t[-1]),
        endpoint_xyz_m=xyz[-1].tolist(),endpoint_norm_m=float(np.linalg.norm(xyz[-1])),
        raw_xyz_ranges_m=[[float(xyz[:,i].min()),float(xyz[:,i].max())] for i in range(3)],
        max_gap_s=float(dt.max()),max_step_m=float(steps.max()),
        max_step_relative_s=[float(t[np.argmax(steps)]),float(t[np.argmax(steps)+1])],
        estimated_speed_quantiles_mps=np.quantile(speed,[.5,.95,1]).tolist(),
        zero_match_calls=len(events),distinct_warning_sim_times=len(set(events.tolist())),
        first_warning_log_relative_s=float(events[0]) if len(events) else None,
        last_warning_log_relative_s=float(events[-1]) if len(events) else None,
        skip_calls=len(skips),skip_log_relative_s=skips,
        pcl_voxel_overflow_calls=len(pcl),first_pcl_overflow_previous_log_relative_s=pcl[0] if pcl else None,
        bins=[],samples=[],crossings=[],hashes={},raw_tum_audit={})
    for lo,hi in [(0,1),(1,5),(5,10),(10,30),(30,60),(60,120),(120,240),(240,404)]:
        sel=(t>=lo)&(t<hi);selstep=(t[1:]>=lo)&(t[1:]<hi)
        result['bins'].append(dict(seconds=[lo,hi],poses=int(sel.sum()),zero_match_calls=int(((events>=lo)&(events<hi)).sum()),
             max_estimated_radius_m=float(np.linalg.norm(xyz[sel],axis=1).max()) if sel.any() else None,
             max_step_m=float(steps[selstep].max()) if selstep.any() else None))
    for sec in [1,2,5,10,20,30,60,120,200,300,400]:
        i=int(np.argmin(abs(t-sec)));result['samples'].append(dict(requested_relative_s=sec,actual_relative_s=float(t[i]),xyz_m=xyz[i].tolist()))
    for threshold in [1,10,100,1000,10000,100000]:
        indices=np.flatnonzero(np.linalg.norm(xyz,axis=1)>threshold)
        result['crossings'].append(dict(estimated_radius_threshold_m=threshold,first_relative_s=float(t[indices[0]]) if len(indices) else None,
            rule='descriptive diagnostic threshold, not GT error or algorithm failure criterion'))
    with rosbag.Bag(str(r/'raw/odometry.bag')) as bag:
        count=0;bad=[];tf=[];odom=[]
        for topic,m,_ in bag.read_messages(topics=['/Odometry','/tf']):
            if topic=='/tf':
                tf.extend(v.header.stamp.to_nsec() for v in m.transforms if v.header.frame_id=='camera_init' and v.child_frame_id=='body');continue
            ns=m.header.stamp.to_nsec();odom.append(ns)
            p=m.pose.pose.position;q=m.pose.pose.orientation
            expected=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
            row=rows[count]
            if Decimal(row[0])*1000000000!=ns or [float(v) for v in row[1:]]!=expected:bad.append(count+1)
            count+=1
        result['raw_tum_audit']=dict(raw_count=count,tum_count=len(rows),mismatched_rows=bad,exact_all_rows=not bad and count==len(rows),
             missing_TF_relative_s=[ns/1e9-start for ns in sorted(set(odom)-set(tf))])
    for rel in ['raw/odometry.bag','raw/cloud_0.bag','derived/final_registered.pcd']:
        p=r/rel;result['hashes'][rel]=dict(sha256=digest(p),bytes=p.stat().st_size)
    if blind==4:
        p=r/'derived/final_registered.pcd'
        with p.open('rb') as f:
            while f.readline()!=b'DATA binary\n':pass
            offset=f.tell()
        cloud=np.memmap(str(p),dtype='<f4',offset=offset,mode='r',shape=(check['output']['cloud_points'],4))
        low=np.full(3,np.inf);high=np.full(3,-np.inf)
        for k in range(0,len(cloud),500000):
            block=cloud[k:k+500000,:3];assert np.isfinite(block).all()
            low=np.minimum(low,block.min(axis=0));high=np.maximum(high,block.max(axis=0))
        dims=np.floor(high/.05).astype(np.int64)-np.floor(low/.05).astype(np.int64)+1
        result['pcd_extent']=dict(min_xyz=low.tolist(),max_xyz=high.tolist(),voxel_5cm_dimensions=dims.tolist(),
             grid_product=str(int(dims[0])*int(dims[1])*int(dims[2])),packed_int64_limit=str(2**63),
             xyz_float32_spacing_at_max_abs_m=np.abs(np.spacing(np.maximum(abs(low),abs(high)).astype(np.float32))).tolist())
    report['runs'][str(blind)]=result
# Explicitly compare algorithm parameter groups, ignore run-generated ROS metadata.
cfgs={b:yaml.safe_load((r/'config/runtime.yaml').read_text()) for b,r in runs.items()}
diff=[]
for key in ['common','cube_side_length','feature_extract_enable','filter_size_map','filter_size_surf','mapping','max_iteration','point_filter_num','publish','runtime_pos_log_enable','use_sim_time','pcd_save']:
    if cfgs[1][key]!=cfgs[4][key]:diff.append(key)
pre=dict(cfgs[1]['preprocess']);pre['blind']=4.
if pre!=cfgs[4]['preprocess']:diff.append('preprocess_unplanned')
report['unexpected_config_differences']=diff
report['input_hash_records_equal']=(runs[1]/'config/input.sha256').read_text()==(runs[4]/'config/input.sha256').read_text()
with (out/'comparison.json').open('x') as f:json.dump(report,f,indent=2)
fig,axes=plt.subplots(2,2,figsize=(13,9))
for blind,(t,xyz,steps,speed) in arrays.items():
    label='blind='+str(blind);color='#1565c0' if blind==1 else '#d32f2f'
    axes[0,0].plot(t,np.linalg.norm(xyz,axis=1),label=label,c=color)
    axes[0,1].plot(t,np.linalg.norm(xyz,axis=1),label=label,c=color)
    axes[1,0].plot(t[1:],steps,label=label,c=color,lw=.7)
    bins=np.arange(0,410,5);counts,_=np.histogram(warnings[blind],bins)
    axes[1,1].step(bins[:-1],counts,where='post',label=label,c=color)
axes[0,0].set_yscale('symlog',linthresh=1);axes[0,0].set_title('Estimated position norm (not GT error)');axes[0,0].set_ylabel('Meters; symlog scale')
axes[0,1].set_xlim(0,30);axes[0,1].set_yscale('symlog',linthresh=1);axes[0,1].set_title('First 30 seconds');axes[0,1].set_ylabel('Meters; symlog scale')
axes[1,0].set_yscale('symlog',linthresh=.1);axes[1,0].set_title('Adjacent estimated position change');axes[1,0].set_ylabel('Meters; symlog scale')
axes[1,1].set_title('Zero-effective-match calls per 5 s (not failed frames)');axes[1,1].set_ylabel('Log calls')
for ax in axes.flat:ax.set_xlabel('Seconds from input bag start');ax.legend();ax.grid(alpha=.25)
fig.tight_layout();fig.savefig(str(out/'blind_comparison.png'),dpi=150);plt.close(fig)
print(json.dumps(report,indent=2))
