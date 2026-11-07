"""Read-only source review; write exclusively to a new analysis directory."""
from pathlib import Path
import json, re, csv, hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

r=Path('/experiment')
out=r/'checks/map-review-20261102-002'
out.mkdir()
rows=[line.split() for line in (r/'derived/trajectory.tum').read_text().splitlines()]
a=np.array(rows,dtype=np.float64)
t=a[:,0]-1737472022.2431047
xyz=a[:,1:4]; dt=np.diff(a[:,0]); delta=np.diff(xyz,axis=0)
speed=np.linalg.norm(delta,axis=1)/dt
log=(r/'checks/algorithm-console.log').read_text(errors='replace')
warn=[]
for line in log.splitlines():
    if 'No Effective Points' in line:
        m=re.search(r'\[[^\]]*?,\s*([0-9.]+)\]',line)
        if m:warn.append(float(m.group(1))-1737472022.2431047)
wt=np.array(sorted(set(warn)))
wi=np.array([int(np.argmin(np.abs(t-v))) for v in wt])
colors=np.zeros((len(t),3),dtype=np.uint8)
# time ramp: early blue to late red; last 15 seconds are red.
colors[:,0]=(255*(t-t.min())/(t.max()-t.min())).astype(np.uint8)
colors[:,1]=80; colors[:,2]=255-colors[:,0]
colors[t>=278]=[255,50,30]
def ply(path,points,rgb,times):
    with path.open('x') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex %d\nproperty double x\nproperty double y\nproperty double z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty double time_relative_s\nend_header\n'%len(points))
        for p,c,s in zip(points,rgb,times):
            f.write('%.17g %.17g %.17g %d %d %d %.9f\n'%(*p,*c,s))
ply(out/'trajectory_time_rgb.ply',xyz,colors,t)
ply(out/'start_green.ply',xyz[:1],np.array([[0,255,0]]),t[:1])
ply(out/'end_red.ply',xyz[-1:],np.array([[255,0,0]]),t[-1:])
with (out/'trajectory_with_time.csv').open('x') as f:
    w=csv.writer(f);w.writerow(['timestamp','x','y','z','relative_seconds'])
    for row,s in zip(rows,t):w.writerow(row[:4]+[format(s,'.9f')])
summary={'origin':'bag start 1737472022.2431047', 'frames':'camera_init -> body',
         'trajectory_count':len(a), 'estimated_path_length_m':float(np.linalg.norm(delta,axis=1).sum()),
         'estimated_endpoint_displacement_xyz_m':(xyz[-1]-xyz[0]).tolist(),
         'estimated_endpoint_distance_m':float(np.linalg.norm(xyz[-1]-xyz[0])),
         'z_range_m':[float(xyz[:,2].min()),float(xyz[:,2].max())],
         'windows':[], 'warning_times_log_relative':wt.tolist(),
         'warnings_nearest_pose_visualization_only':[
             dict(log_time=float(v),nearest_pose_time=float(t[i]),xyz=xyz[i].tolist(),
                  time_difference=float(t[i]-v)) for v,i in zip(wt,wi)],
         'limits':'Estimated motion, not GT error. Log-clock association is approximate, not exact warning-frame identification. Colored trajectory is separate from map. No degeneracy metric was recorded.'}
for lo,hi in [(0,30),(30,270),(270,278),(278,290),(290,294)]:
    mask=(t[1:]>=lo)&(t[1:]<hi)
    summary['windows'].append(dict(seconds=[lo,hi],intervals=int(mask.sum()),
        estimated_speed_mps_quantiles=np.quantile(speed[mask],[0,.5,.95,1]).tolist(),
        max_step_m=float(np.linalg.norm(delta[mask],axis=1).max())))
# Plot map and trajectory on equal XY scales; map subset only for display.
pcd=r/'derived/final_registered_voxel_0p05m_first.pcd'
with pcd.open('rb') as f:
    while f.readline()!=b'DATA binary\n': pass
    off=f.tell()
n=(pcd.stat().st_size-off)//16
cloud=np.memmap(str(pcd),mode='r',dtype='<f4',offset=off,shape=(n,4))
fig,axes=plt.subplots(2,2,figsize=(14,10))
ax=axes[0,0]; ax.scatter(cloud[::8,0],cloud[::8,1],s=.1,c='#999999',rasterized=True)
ax.plot(xyz[:,0],xyz[:,1],lw=1,c='#1565c0',label='Estimated body trajectory')
ax.plot(xyz[t>=278,0],xyz[t>=278,1],lw=2,c='#d32f2f',label='t >= 278 s')
ax.scatter(xyz[0,0],xyz[0,1],c='green',marker='o',s=50,label='Start')
ax.scatter(xyz[-1,0],xyz[-1,1],c='red',marker='x',s=50,label='End')
ax.scatter(xyz[wi,0],xyz[wi,1],c='orange',marker='^',s=30,label='Warning nearest pose (approx.)')
for sec in range(0,294,30):
    i=np.argmin(abs(t-sec));ax.annotate(str(sec)+'s',xyz[i,:2],fontsize=8)
ax.set_aspect('equal',adjustable='datalim');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.legend(fontsize=7);ax.set_title('XY map and estimated trajectory')
ax.set_aspect('equal',adjustable='box')
ax.set_xlim(float(xyz[:,0].min()-12),float(xyz[:,0].max()+12))
ax.set_ylim(float(xyz[:,1].min()-12),float(xyz[:,1].max()+12))
ax.set_title('XY detail: trajectory bounds + 12 m (display crop only)')
summary['plot_display_only']='XY axes restricted to trajectory bounds plus 12 m; distant map points not removed from any data'
axes[0,1].plot(t,xyz[:,2]);axes[0,1].set_ylabel('Estimated Z (m)');axes[0,1].set_title('Height estimate')
axes[1,0].plot(t[1:],speed,lw=.7);axes[1,0].set_ylabel('Estimated step / dt (m/s)');axes[1,0].set_title('Estimated motion, not speed ground truth')
axes[1,1].plot(t[1:],speed,lw=.8);axes[1,1].set_xlim(270,294);axes[1,1].set_ylabel('Estimated step / dt (m/s)');axes[1,1].set_title('Tail detail')
for ax in [axes[0,1],axes[1,0],axes[1,1]]:
    ax.set_xlabel('Seconds from input bag start');ax.grid(alpha=.25)
    for v in wt:ax.axvline(v,c='orange',lw=.7,alpha=.7)
fig.tight_layout();fig.savefig(str(out/'map_trajectory_diagnostics.png'),dpi=160);plt.close(fig)
summary['trajectory_export_verified']=bool(len(rows)==len(xyz))
with (out/'summary.json').open('x') as f:json.dump(summary,f,indent=2)
print(json.dumps(summary,indent=2))
