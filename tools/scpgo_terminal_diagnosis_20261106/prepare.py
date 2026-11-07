from pathlib import Path
import json,hashlib
out=Path('/out');fix=Path('/fix')
prior=json.loads((fix/'results-summary.json').read_text())
accepted=[r for r in prior['all_pairs']['fixed-odom'] if r['accepted']]
pairs=[(0,375),(0,376),(0,377),(7,378),(0,379),(0,380),(0,381),(0,382),(0,32),(147,310)]
for r in [min(accepted,key=lambda r:r['fitness_m2']),max(accepted,key=lambda r:r['disagreement_with_saved_poses_translation_m'])]:
    p=(r['previous'],r['current'])
    if p not in pairs:pairs.append(p)
(out/'pairs.txt').write_text(''.join(f'{p} {c}\n' for p,c in pairs))
src=Path('/source/src/laserPosegraphOptimization.cpp')
assert hashlib.sha256(src.read_bytes()).hexdigest()==json.loads((fix/'final-report.json').read_text())['source_sha256']
text=src.read_text()
def extract(signature):
    start=text.index(signature);brace=text.index('{',start);end=brace+1;depth=1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]
names=['gtsam::Pose3 Pose6DtoGTSAMPose3(', 'pcl::PointCloud<PointType>::Ptr local2global(', 'void loopFindNearKeyframesCloud(']
(out/'probe.cpp').write_text((fix/'synthetic/prefix.cpp').read_text()+'\n'+'\n\n'.join(extract(n) for n in names)+'\n'+(out/'diagnose.cpp').read_text())
(out/'manifest.json').write_text(json.dumps({'question':'Diagnose yaw initialization, directional geometric support and distance scales; no production changes.',
 'pairs':pairs,'selection':'All eight terminal pairs; two previous controls; corrected minimum-fitness accepted and maximum-odom-disagreement accepted controls.',
 'initialization_variants':['identity','negative_returned_sc_yaw','saved_odom_rotation_only','saved_odom_full_pose'],
 'fixed_parameters':'SC radius40/rings20/sectors60/height2; SC and ICP voxel0.4; ICP correspondence150/iterations100/eps1e-6/RANSAC0/fitness0.3.',
 'diagnostic_only':'Counts at NN distances0.2,0.5,1,2,5,10,150m are reported without changing correspondence threshold or scoring rule. Range40 subsets are summaries, not filtered ICP inputs.',
 'yaw_sign_test':'Known asymmetric cloud rotated by +30 degrees in sensor pose; descriptor current vs previous order matches production.',
 'scope_limit':'Saved odom is not truth. Close NN support is not independently verified physical overlap. Local GT file is an endpoint transform with bag_time383s, not a continuous trajectory; no endpoint GT applied to403s tail.',
 'stopping':'Run fixed pairs and four initials once; preserve all outcomes. No tuning or adaptive choice of thresholds.',
 'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest()},indent=2)+'\n')
