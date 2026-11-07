from pathlib import Path
import json,hashlib
out=Path(__file__).resolve().parent
fix=out.parent/'scpgo_geometry_fix_20261106'
old=json.loads((fix/'final-report.json').read_text())
ws=Path('/home/lw/slam-research/fast_lio_ws')
source=ws/'src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO/src/laserPosegraphOptimization.cpp'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(source)==old['source_sha256']
assert sha(Path(old['binary']))==old['binary_sha256']
installed=json.loads((fix/'installation.json').read_text())
assert sha(ws/'src/M3DGR/deployment-scpgo/runtime/corridor01-indoor.yaml')==installed['profile_sha256']
rows=[json.loads(l) for l in (out/'diagnostics.jsonl').read_text().splitlines()]
prior=json.loads((fix/'results-summary.json').read_text())['all_pairs']['fixed-odom']
baseline={(r['previous'],r['current']):r for r in prior}
for r in rows:
    assert len(r['initials'])==4
    assert abs(r['initials'][0]['fitness_m2']-baseline[(r['previous'],r['current'])]['fitness_m2'])<1e-6
sign=json.loads((out/'yaw-sign-test.json').read_text());assert sign['negative_yaw_matrix_error']<1e-4
report={'completed_date_local':'2026-11-07','folder_date_note':'Directory name retained from start before midnight.',
        'production_source_binary_profile_unchanged':True,'baseline_scores_reproduced':True,'pair_count':len(rows),'icp_runs':len(rows)*4,
        'terminal_pairs':8,'terminal_pass_by_initialization':{name:sum(r['initials'][i]['converged'] and r['initials'][i]['fitness_m2']<=.3 for r in rows[:8]) for i,name in enumerate(['identity','sc_yaw','odom_rotation','odom_full'])},
        'yaw_sign_test':sign,'camera_visual_review':'Same distinctive marker board and wall corner visible at keyframe0 and375/378/382; timestamps within16ms. Supports physical revisit, not calibrated LiDAR correspondences.',
        'physical_lidar_overlap_quantified':False,'source_sha256':sha(source),'binary_sha256':sha(Path(old['binary'])),
        'not_tested':['smaller maximum correspondence distances','smaller voxel sizes','multi-scale ICP','additional current-frame accumulation','production yaw integration','full SLAM replay'],
        'limitations':['No continuous ground truth for 390-403s tail. Existing endpoint GT declares383s.','Saved odom orientation/full-pose initializations are diagnostics, not truth.','Low nearest-neighbor distance and low descriptor score do not prove correct physical correspondences.','SC yaw recomputed for fixed candidate pairs from saved scans; online yaw was not recorded.']}
report['artifact_sha256']={p.name:sha(p) for p in out.iterdir() if p.is_file() and p.suffix in ['.cpp','.py','.sh','.jsonl','.png']}
(out/'final-report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['completed_date_local','production_source_binary_profile_unchanged','baseline_scores_reproduced','pair_count','icp_runs','terminal_pass_by_initialization']},indent=2))
