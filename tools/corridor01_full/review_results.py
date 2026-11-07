"""Independent read-only result review; outputs only to a new checks folder."""
from pathlib import Path
import hashlib,json
import numpy as np
import rosbag
r=Path('/experiment');out=r/'checks/result-review-20261103-001';out.mkdir()
a=np.loadtxt(str(r/'derived/trajectory.tum'));st=np.loadtxt(str(r/'Log/mat_pre.txt'))
start=json.loads((r/'checks/full-run.json').read_text())['input_bag_start']
t=a[:,0]-start;delta=np.diff(a[:,1:4],axis=0);steps=np.linalg.norm(delta,axis=1);speed=steps/np.diff(a[:,0])
g0=st[0,-3:];g1=st[-1,-3:];up=-g0/np.linalg.norm(g0);h=(a[:,1:4]-a[0,1:4]).dot(up)
result=dict(trajectory_count=len(a),trajectory_span_s=float(t[-1]-t[0]),
            estimated_endpoint_xyz_change=(a[-1,1:4]-a[0,1:4]).tolist(),estimated_endpoint_distance_m=float(np.linalg.norm(a[-1,1:4]-a[0,1:4])),
            raw_Z_range_m=[float(a[:,3].min()),float(a[:,3].max())],
            initial_gravity=g0.tolist(),final_gravity=g1.tolist(),
            fixed_initial_gravity_projected_height_range_m=[float(h.min()),float(h.max())],
            fixed_initial_gravity_projected_height_span_m=float(np.ptp(h)),
            fixed_initial_gravity_projected_endpoint_change_m=float(h[-1]),
            estimated_step_speed_quantiles_mps=np.quantile(speed,[0,.5,.95,1]).tolist(),
            top_steps=[dict(relative_start=float(t[i]),relative_end=float(t[i+1]),step_m=float(steps[i]),dt_s=float(t[i+1]-t[i]),estimated_speed_mps=float(speed[i])) for i in np.argsort(steps)[-8:][::-1]],
            files={},TF={})
for path in [r/'raw/odometry.bag',r/'raw/cloud_0.bag',r/'PCD/scans.pcd',r/'derived/final_registered.pcd']:
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(8*1024*1024),b''):digest.update(chunk)
    result['files'][str(path.relative_to(r))]=dict(bytes=path.stat().st_size,sha256=digest.hexdigest())
with rosbag.Bag(str(r/'raw/odometry.bag')) as bag:
    odom=[];tf=[]
    for topic,m,_ in bag.read_messages(topics=['/Odometry','/tf']):
        if topic=='/Odometry':odom.append(m.header.stamp.to_nsec())
        else:tf.extend(x.header.stamp.to_nsec() for x in m.transforms if x.header.frame_id=='camera_init' and x.child_frame_id=='body')
    result['TF']=dict(transforms=len(tf),missing_pose_relative_s=[ns/1e9-start for ns in sorted(set(odom)-set(tf))])
result['limits']='Estimated motion and coordinates, not GT errors or proof of degeneracy. Fixed initial gravity projection is a diagnostic only. Screenshot coloring and physical identity of points require display settings/local geometry.'
with (out/'report.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
