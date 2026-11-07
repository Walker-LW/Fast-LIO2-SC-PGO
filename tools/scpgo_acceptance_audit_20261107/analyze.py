from pathlib import Path
import json,sys
import numpy as np
root=Path('/out');distance=Path('/distance')
data=json.loads((distance/'summary.json').read_text());rows=data['results']
poses0=np.loadtxt('/run/scpgo/backend/odom_poses.txt').reshape(-1,3,4)
poses=np.tile(np.eye(4),(len(poses0),1,1));poses[:,:3,:4]=poses0
times=np.loadtxt('/run/scpgo/backend/times.txt')
by={(r['previous'],r['current'],r['gate_m']):r for r in rows}
def angular(R):
 u,_,v=np.linalg.svd(R);R=u@np.diag([1,1,np.linalg.det(u@v)])@v
 return float(np.arctan2(np.linalg.norm([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/2,(np.trace(R)-1)/2)*180/np.pi)
temporal=[]
pairs=[(0,375),(0,376),(0,377),(7,378),(0,379),(0,380),(0,381),(0,382)]
for gate in [150,5,2,1]:
 for (pa,ca),(pb,cb) in zip(pairs,pairs[1:]):
  Za=np.array(by[(pa,ca,gate)]['final_matrix']).reshape(4,4)
  Zb=np.array(by[(pb,cb,gate)]['final_matrix']).reshape(4,4)
  # Express both loop-implied current poses in historical-frame0 coordinates.
  A=np.linalg.inv(poses[0])@poses[pa]@Za;B=np.linalg.inv(poses[0])@poses[pb]@Zb
  loop=np.linalg.inv(A)@B;odom=np.linalg.inv(poses[ca])@poses[cb];error=np.linalg.inv(odom)@loop
  temporal.append({'gate_m':gate,'current_pair':[ca,cb],'seconds':float(times[cb]-times[ca]),
   'loop_implied_delta_translation_m':float(np.linalg.norm(loop[:3,3])),
   'saved_odom_delta_translation_m':float(np.linalg.norm(odom[:3,3])),
   'difference_translation_m':float(np.linalg.norm(error[:3,3])),
   'difference_rotation_deg':angular(error[:3,:3])})

# No learned thresholds: test dominance in the usual acceptance direction.
true=by[(0,382,5)];suspect=by[(41,151,5)]
directions={'fixed1m_mse':'lower','fixed1m_inlier_ratio':'higher','fixed1m_inlier_n':'higher','fixed1m_unique_target_n':'higher','fixed1m_reciprocal_n':'higher'}
dominance={k:{'return_endpoint':true[k],'suspect':suspect[k],'suspect_at_least_as_favorable':suspect[k]<=true[k] if d=='lower' else suspect[k]>=true[k]} for k,d in directions.items()}
assert all(v['suspect_at_least_as_favorable'] for v in dominance.values())
for k in ['pair','fixed1m_mse','fixed1m_inlier_ratio','fixed1m_inlier_n','fixed1m_unique_target_n','fixed1m_reciprocal_n']:
 print(k,'endpoint',true[k],'suspect',suspect[k])
report={'scope':'Read-only reuse of48ICP results plus16independent camera frames. No newICP or acceptance thresholds.',
 'visual_review':{'0-382':'Camera marker board supports revisit; LiDAR pointwise transform is not ground truth.',
 '135-287':'Fixed scene structures differ: ceiling, railing/open atrium, window/display layout. Not accepted as a trusted co-located return; qualitative negative evidence, no metric ground truth.',
 '41-151':'Camera views do not establish a trusted same-place return; retained as suspect/unresolved, not asserted false from odom distance.',
 '0-32':'No trusted same-place evidence from these frames; reject quantitative truth labeling.',
 '147-310':'No trusted same-place evidence from these frames; reject quantitative truth labeling.'},
 'monotone_metric_dominance':dominance,
 'consequence':'Any conjunction of lower-MSE and minimum count/ratio/unique-target/reciprocal thresholds accepting endpoint0-382 atgate5 also accepts suspect41-151. This is a diagnostic counterexample, not a measured classifier false-positive rate.',
 'temporal_checks':temporal,'temporal_limits':'Uses saved short-interval odom as a consistency reference, not ground truth. Different historical roots are converted using saved odom. No hard acceptance threshold calibrated.',
 'production_modified':False,'joint_rule_deployed':False}
(root/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print('gate5 temporal',json.dumps([r for r in temporal if r['gate_m']==5],indent=2))
