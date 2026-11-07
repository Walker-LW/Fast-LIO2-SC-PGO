from pathlib import Path
import sys,json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,'/run/scpgo')
from export_keyframe_maps import read_pcd
out=Path('/out');rows=[json.loads(l) for l in (out/'diagnostics.jsonl').read_text().splitlines()]
summary=[]
for r in rows:
    tag=f"{r['previous']}-{r['current']}";rr=np.array(r['source_ranges']);near=rr<=40
    curve=np.array([np.nan if x is None else x for x in r['sc_distance_by_shift']])
    desc={'pair':tag,'sc_returned_yaw_deg':r['sc_returned_yaw_deg'],'sc_icp_initial_yaw_deg':float(((-r['sc_returned_yaw_deg']+180)%360)-180),'sc_distance':r['sc_score'],'sc_shifts_within_min_plus_001':int(np.sum(curve[np.isfinite(curve)]<=np.nanmin(curve)+.01)),'odom_yaw_deg':r['odom_yaw_deg'],'source_points':len(rr),'source_within40m':int(near.sum()),'initializations':[]}
    for v in r['initials']:
        d0=np.array(v['initial_nn_distances_m']);d=np.array(v['final_nn_distances_m']);single=np.array(v['final_nn_to_single_previous_m'])
        assert np.isclose(np.mean(d*d),v['fitness_m2'],rtol=1e-4,atol=1e-5)
        a={'name':v['name'],'fitness_m2':v['fitness_m2'],'accepted_by_current_rule':v['converged'] and v['fitness_m2']<=.3,
           'initial_distance_quantiles_m':np.quantile(d0,[0,.5,.9,1]).tolist(),'initial_support_counts':{str(k):int(np.sum(d0<=k)) for k in [.2,.5,1,2,5,10,150]},
           'initial_gt10m_squared_distance_fraction':float(np.sum(d0[d0>10]**2)/np.sum(d0*d0)),
           'initial_far40m_squared_distance_fraction':float(np.sum(d0[~near]**2)/np.sum(d0*d0)),
           'final_distance_quantiles_m':np.quantile(d,[0,.5,.9,1]).tolist(),'final_support_counts':{str(k):int(np.sum(d<=k)) for k in [.2,.5,1,2,5,10,150]},
           'near40_initial_mse':float(np.mean(d0[near]**2)),'near40_final_mse':float(np.mean(d[near]**2)),
           'final_to_single_previous_within05m':int(np.sum(single<=.5))}
        desc['initializations'].append(a)
    clouds={k:read_pcd(out/(tag+'-'+k+'.pcd'))[:,:3] for k in ['raw-current','raw-previous','source','previous']}
    for key,p in clouds.items():
        norms=np.linalg.norm(p,axis=1);pnear=p[norms<=4].astype(float)
        s=np.linalg.svd(pnear-pnear.mean(0),compute_uv=False)/np.sqrt(len(pnear))
        desc.setdefault('clouds',{})[key]={'points':len(p),'within4m':len(pnear),'within40m':int(np.sum(norms<=40)),
                     'near4_xyz_min':pnear.min(0).tolist(),'near4_xyz_max':pnear.max(0).tolist(),'near4_z_quantiles':np.quantile(pnear[:,2],[0,.25,.5,.75,1]).tolist(),'near4_pca_rms_axes_m':s.tolist()}
    summary.append(desc)

last=next(r for r in rows if r['current']==382);tag='0-382'
target=read_pcd(out/(tag+'-target.pcd'))[:,:3];source=read_pcd(out/(tag+'-source.pcd'))[:,:3]
rawprev=read_pcd(out/(tag+'-raw-previous.pcd'))[:,:3];rawcurr=read_pcd(out/(tag+'-raw-current.pcd'))[:,:3]
fig,axes=plt.subplots(2,3,figsize=(15,9),constrained_layout=True)
for ax,(i,j),lab in zip(axes[0],[(0,1),(0,2),(1,2)],['XY','XZ','YZ']):
    for p,label,color in [(rawprev,'Historical scan 0','tab:blue'),(rawcurr,'Current scan 382','tab:orange')]:
        q=p[np.linalg.norm(p,axis=1)<=4];ax.scatter(q[:,i],q[:,j],s=2,alpha=.5,label=label,color=color)
    ax.set_aspect('equal',adjustable='datalim');ax.set_xlabel('XYZ'[i]+' / m');ax.set_ylabel('XYZ'[j]+' / m');ax.set_title('Raw local clouds, range <= 4 m: '+lab);ax.legend(fontsize=8)
for ax,v in zip(axes[1],last['initials'][:3]):
    aligned=read_pcd(out/(tag+'-'+v['name']+'-aligned.pcd'))[:,:3]
    ax.scatter(target[:,0],target[:,1],s=3,c='0.7',label='Historical submap')
    ax.scatter(source[:,0],source[:,1],s=12,c='tab:blue',label='Source at identity')
    ax.scatter(aligned[:,0],aligned[:,1],s=12,c='tab:red',label='After ICP')
    ax.set(xlim=(-5,5),ylim=(-5,5),xlabel='X / m',ylabel='Y / m',title=f"Init: {v['name']}; MSE={v['fitness_m2']:.2f} m2")
    ax.set_aspect('equal');ax.legend(fontsize=8)
fig.suptitle('Pair (0,382): geometric proximity is not verified physical correspondence\nTop: separate body coordinates, no registration; bottom: display crop only, all points used in ICP',fontsize=12)
fig.savefig(out/'terminal-geometry.png',dpi=180);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
for r in rows[:8]:
    v=r['initials'][0];d=np.sort(v['initial_nn_distances_m']);axes[0].plot(d,np.arange(1,len(d)+1)/len(d),label=f"{r['previous']}-{r['current']}")
axes[0].set(xlabel='Initial nearest-neighbor distance / m',ylabel='Fraction of source points',title='Identity start, before ICP',xlim=(0,150));axes[0].legend(fontsize=8,ncol=2)
for r in [rows[0],rows[5],rows[7]]:
    axes[1].plot(np.arange(60)*6,r['sc_distance_by_shift'],'.-',label=f"{r['previous']}-{r['current']}")
axes[1].set(xlabel='SC returned shift / degree',ylabel='Descriptor distance',title='Several yaw shifts can have similar scores');axes[1].legend()
fig.savefig(out/'distance-yaw-evidence.png',dpi=180);plt.close(fig)

(out/'summary.json').write_text(json.dumps({'pairs':summary,'production_modified':False,'thresholds_changed':False,'physical_overlap_verified':False},indent=2)+'\n')
print(json.dumps({'terminal':[dict({k:s[k] for k in ['pair','sc_icp_initial_yaw_deg','sc_shifts_within_min_plus_001','source_points','source_within40m']}, initial_identity=s['initializations'][0]) for s in summary[:8]],'last_raw_geometry':summary[7]['clouds']},indent=2))
