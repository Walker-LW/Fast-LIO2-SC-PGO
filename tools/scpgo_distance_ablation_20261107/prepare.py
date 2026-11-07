from pathlib import Path
import json,hashlib
out=Path('/out');previous=Path('/previous')
meta=json.loads((previous/'manifest.json').read_text())
pairs=meta['pairs'];assert len(pairs)==12
(out/'pairs.txt').write_text(''.join(f'{p} {c}\n' for p,c in pairs))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files={}
for p,c in pairs:
    for kind in ['source','target']:
        name=f'{p}-{c}-{kind}.pcd';files[name]=sha(previous/name)
(out/'manifest.json').write_text(json.dumps({
 'date':'2026-11-07','authorization':'User approved offline single-factor maximum-correspondence-distance comparison.',
 'question':'Does reducing excessive correspondence distances improve local alignment without losing geometric support?',
 'pairs':pairs,'selection':'Frozen same 12 pairs as prior terminal diagnosis; first eight are terminal, four are controls.',
 'input_sha256':files,'input_scope':'Exact already prepared source/target PCDs from geometry-corrected code and saved odom poses; no re-filtering or input range removal.',
 'max_correspondence_distances_m':[150,5,2,1],
 'fixed':{'initial_transform':'identity','sc_voxel_size_m':.4,'icp_voxel_size_m':.4,'icp_max_iterations':100,'transformation_epsilon':1e-6,'fitness_epsilon':1e-6,'ransac_iterations':0,'production_fitness_threshold_m2':.3},
 'metrics':{'full_fitness_m2':'All source nearest-neighbor squared distances, as original PCL scorer; no gate.',
            'fixed_1m_support':'NN distance <=1m, denominator all source points; statistic only, not a new acceptance rule.',
            'near40m_mse':'All source points whose original local range <=40m; no residual trimming.',
            'inlier_geometry':'1m inlier spatial spans/PCA axes, unique target matches, reciprocal nearest matches; no observability or true-match labels.',
            'stability':'Between resulting rigid transforms at different thresholds; agreement is not ground-truth accuracy.'},
 'validation':'All 12 threshold150 runs must reproduce prior scores within 1e-6; source/target hashes and production hashes unchanged.',
 'stopping':'One run per fixed pair/threshold (48 total), preserve every outcome; no adaptive threshold search.',
 'limitations':['No independent pointwise correspondence truth.','Saved odom-based historical submap may contain frontend distortion.','No production modification or full SLAM replay.','Convergence and a small inlier score do not establish a valid loop.']
},indent=2)+'\n')
