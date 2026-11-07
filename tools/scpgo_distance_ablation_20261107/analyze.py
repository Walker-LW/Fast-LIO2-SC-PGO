from pathlib import Path
import sys,json,csv,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,'/run/scpgo')
from export_keyframe_maps import read_pcd

out=Path('/out');previous=Path('/previous')
meta=json.loads((out/'manifest.json').read_text())
rows=[json.loads(l) for l in (out/'results.jsonl').read_text().splitlines()]
assert len(rows)==48
prev={(r['previous'],r['current']):r for r in map(json.loads,(previous/'diagnostics.jsonl').read_text().splitlines())}
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for name,h in meta['input_sha256'].items():assert sha(previous/name)==h,name
summary=[];by={}
for r in rows:
    p,c,gate=r['previous'],r['current'],int(r['gate_m']);tag=f'{p}-{c}'
    source=read_pcd(previous/(tag+'-source.pcd'))[:,:3].astype(float)
    d=np.array(r['nn_distances_m']);near=np.linalg.norm(source,axis=1)<=40;inside=d<=1.
    assert len(d)==len(source)==r['source_points']
    assert np.isfinite(d).all()
    assert np.isclose(np.mean(d*d),r['full_fitness_m2'],rtol=1e-5,atol=1e-5)
    if gate==150:assert abs(r['full_fitness_m2']-prev[(p,c)]['initials'][0]['fitness_m2'])<1e-6,(p,c)
    xyz=source[inside];targets=np.array(r['target_indices']);reciprocal=np.array(r['reciprocal'],dtype=bool)
    axes=np.linalg.svd(xyz-xyz.mean(0),compute_uv=False)/np.sqrt(len(xyz)) if len(xyz)>=3 else []
    s={'pair':tag,'previous':p,'current':c,'group':'terminal' if [p,c] in meta['pairs'][:8] else 'control','gate_m':gate,
       'converged':r['converged'],'iterations':r['iterations'],'convergence_state':r['convergence_state'],'full_fitness_m2':r['full_fitness_m2'],
       'original_acceptance':bool(r['converged'] and r['full_fitness_m2']<=.3),
       'source_n':len(d),'fixed1m_inlier_n':int(inside.sum()),'fixed1m_inlier_ratio':float(inside.mean()),
       'fixed1m_mse':float(np.mean(d[inside]**2)) if inside.any() else None,
       'near40m_n':int(near.sum()),'near40m_mse':float(np.mean(d[near]**2)),'near40m_fixed1m_n':int(np.sum(near&inside)),
       'far40m_fixed1m_n':int(np.sum(~near&inside)),
       'fixed1m_unique_target_n':len(np.unique(targets[inside])),'fixed1m_reciprocal_n':int(np.sum(inside&reciprocal)),
       'fixed1m_source_xyz_span_m':np.ptp(xyz,axis=0).tolist() if inside.any() else [],'fixed1m_source_pca_rms_axes_m':list(axes),
       'final_translation_m':np.array(r['final_matrix']).reshape(4,4)[:3,3].tolist(),'final_matrix':r['final_matrix']}
    summary.append(s);by[(p,c,gate)]=s

def rotation(m):
    u,_,v=np.linalg.svd(m[:3,:3]);return u@np.diag([1,1,np.linalg.det(u@v)])@v
stability=[]
for p,c in meta['pairs']:
    for a,b in [(150,5),(5,2),(2,1),(5,1)]:
        A=np.array(by[(p,c,a)]['final_matrix']).reshape(4,4);B=np.array(by[(p,c,b)]['final_matrix']).reshape(4,4)
        R=rotation(A).T@rotation(B)
        angle=np.arctan2(np.linalg.norm([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/2,(np.trace(R)-1)/2)*180/np.pi
        stability.append({'pair':f'{p}-{c}','gates':[a,b],'translation_difference_m':float(np.linalg.norm(A[:3,3]-B[:3,3])),'rotation_difference_deg':float(angle)})

aggregate={}
for gate in [150,5,2,1]:
    s=[r for r in summary if r['gate_m']==gate and r['group']=='terminal']
    aggregate[str(gate)]={'terminal_accepted':sum(r['original_acceptance'] for r in s),'terminal_converged':sum(r['converged'] for r in s),
        'median_near40m_mse':float(np.median([r['near40m_mse'] for r in s])),
        'median_fixed1m_ratio':float(np.median([r['fixed1m_inlier_ratio'] for r in s])),
        'near40m_mse_lower_than_150_pairs':sum(r['near40m_mse']<by[(r['previous'],r['current'],150)]['near40m_mse'] for r in s)}
(out/'summary.json').write_text(json.dumps({'aggregate':aggregate,'results':summary,'stability':stability,'baseline_scores_reproduced':True,'all_input_hashes_unchanged':True},indent=2)+'\n')
fields=['pair','group','gate_m','converged','iterations','full_fitness_m2','original_acceptance','source_n','fixed1m_inlier_n','fixed1m_inlier_ratio','fixed1m_mse','near40m_n','near40m_mse','fixed1m_unique_target_n','fixed1m_reciprocal_n']
with (out/'metrics.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(summary)

fig,axes=plt.subplots(1,2,figsize=(13,5),constrained_layout=True)
for p,c in meta['pairs'][:8]:
    vals=[by[(p,c,g)] for g in [150,5,2,1]];label=f'{p}-{c}'
    axes[0].plot(range(4),[v['near40m_mse'] for v in vals],'.-',label=label)
    axes[1].plot(range(4),[v['fixed1m_inlier_ratio'] for v in vals],'.-',label=label)
for ax in axes:
    ax.set_xticks(range(4));ax.set_xticklabels(['150','5','2','1']);ax.set_xlabel('Maximum correspondence distance / m (categorical)');ax.grid(alpha=.2)
axes[0].set(yscale='log',ylabel='MSE of fixed source range <=40 m / m2',title='Near-source alignment; no residual trimming')
axes[1].set(ylabel='Fraction of all source points',ylim=(0,1),title='Fixed 1 m nearest-neighbor support');axes[1].legend(fontsize=8,ncol=2)
fig.suptitle('Eight terminal pairs; identity initial pose; voxel 0.4 m; no acceptance-rule change')
fig.savefig(out/'terminal-distance-comparison.png',dpi=180);plt.close(fig)

fig,axes=plt.subplots(1,4,figsize=(16,4.8),constrained_layout=True)
source=read_pcd(previous/'0-382-source.pcd')[:,:3];target=read_pcd(previous/'0-382-target.pcd')[:,:3]
for ax,g in zip(axes,[150,5,2,1]):
    aligned=read_pcd(out/f'0-382-gate{g}-aligned.pcd')[:,:3];s=by[(0,382,g)]
    ax.scatter(target[:,0],target[:,1],s=4,c='0.65',label='Historical submap')
    ax.scatter(source[:,0],source[:,1],s=12,c='tab:blue',label='Source at identity')
    ax.scatter(aligned[:,0],aligned[:,1],s=14,c='tab:red',label='After ICP')
    ax.set(xlim=(-1,4),ylim=(-2,3),xlabel='X / m',ylabel='Y / m',title=f"Gate {g} m\nnear MSE={s['near40m_mse']:.4f}; support={s['fixed1m_inlier_n']}/47")
    ax.set_aspect('equal');ax.legend(fontsize=7)
fig.suptitle('Pair (0,382), display crop only. Small local residual does not establish a valid physical loop.')
fig.savefig(out/'last-pair-alignment.png',dpi=180);plt.close(fig)
print(json.dumps({'aggregate':aggregate,'last_pair':[{k:r[k] for k in fields} for r in summary if r['pair']=='0-382'],
 'terminal_2_to_1_stability':[s for s in stability if s['gates']==[2,1] and tuple(map(int,s['pair'].split('-'))) in [tuple(p) for p in meta['pairs'][:8]]]},indent=2))
