from pathlib import Path
import json,sys,math,re,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
pair=Path('/pair');sys.path.insert(0,str(pair/'old'))
from export_keyframe_maps import read_pcd
out=pair/'derived/comparison';out.mkdir(parents=True,exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
roots=[pair/'old',pair/'fixed'];scans=[sorted((r/'backend/Scans').glob('*.pcd')) for r in roots]
assert len(scans[0])==len(scans[1])==383
assert [p.name for p in scans[0]]==[p.name for p in scans[1]]
assert all(sha(a)==sha(b) for a,b in zip(*scans)),'Different keyframe scans; cannot make a strict same-keyframe comparison'
for name in ['times.txt','odom_poses.txt']:
 assert sha(roots[0]/'backend'/name)==sha(roots[1]/'backend'/name),name+' differs'
poses=[[np.loadtxt(r/'backend'/name).reshape(-1,3,4) for name in ['odom_poses.txt','optimized_poses.txt']] for r in roots]
lo=np.concatenate([p[:,:2,3] for pp in poses for p in pp]).min(0)-5
hi=np.concatenate([p[:,:2,3] for pp in poses for p in pp]).max(0)+5
fig,axes=plt.subplots(2,2,figsize=(16,10),constrained_layout=True);metrics={}
for row,(r,pp,label) in enumerate(zip(roots,poses,['Old geometry','Fixed geometry'])):
 for col,(p,name,title) in enumerate(zip(pp,['before_pgo.pcd','after_pgo.pcd'],['Before PGO','After PGO'])):
  path=r/'derived/keyframe-maps'/name;cloud=read_pcd(path);step=max(1,math.ceil(len(cloud)/250000));shown=cloud[::step]
  ax=axes[row,col];ax.scatter(shown[:,0],shown[:,1],s=.3,c='steelblue');ax.plot(p[:,0,3],p[:,1,3],lw=.8,c='darkorange')
  ax.scatter(p[0,0,3],p[0,1,3],marker='*',c='red',s=60);ax.scatter(p[-1,0,3],p[-1,1,3],marker='x',c='black',s=35)
  ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='X / m',ylabel='Y / m',title=label+' / '+title);ax.set_aspect('equal')
  metrics.setdefault(r.name,{})[name]={'points':len(cloud),'stride':step,'sha256':sha(path)}
 d=np.linalg.norm(pp[1][:,:,3]-pp[0][:,:,3],axis=1)
 log=(r/'checks/backend-console.log').read_text(errors='replace')
 metrics[r.name].update(max_pose_translation_change_m=float(d.max()),median_pose_translation_change_m=float(np.median(d)),
   loop_detections=len(re.findall('Loop detected! - between',log)),icp_passed=len(re.findall('ICP fitness test passed',log)),icp_failed=len(re.findall('ICP fitness test failed',log)),
   pcl_overflow_warnings=len(re.findall('Integer indices would overflow',log)),
   shutdown_log_tail=log.splitlines()[-12:])
fig.suptitle('Same saved keyframes and display axes; display crop only, no extra alignment')
fig.savefig(out/'old-fixed-before-after.png',dpi=180);plt.close(fig)
assert metrics['old']['before_pgo.pcd']['sha256']==metrics['fixed']['before_pgo.pcd']['sha256'],'Before maps differ'
report={'status':'SAME_KEYFRAME_COMPARISON_EXPORTED','keyframes':383,'all_scan_hashes_equal':True,'times_and_odom_equal':True,
 'xy_min':lo.tolist(),'xy_max':hi.tolist(),'metrics':metrics,
 'limits':['No ground-truth accuracy evaluation.','Same input/keyframes do not guarantee same candidate/factor schedules; asynchronous backend timing must be reviewed.','ICP pass counts do not establish loop truth.','Recorded002 clock tail included in both runs.','Image crops display bounds only.']}
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'status':report['status'],'preview':str(out/'old-fixed-before-after.png'),'keyframes':383,'metrics':metrics},indent=2))
