from pathlib import Path
import json,hashlib
p=Path(__file__).resolve().parent
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
prior=json.loads((p.parent/'scpgo_geometry_fix_20261106/final-report.json').read_text())
installation=json.loads((p.parent/'scpgo_geometry_fix_20261106/installation.json').read_text())
ws=Path('/home/lw/slam-research/fast_lio_ws')
source=ws/'src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO/src/laserPosegraphOptimization.cpp'
assert sha(source)==prior['source_sha256']
assert sha(Path(prior['binary']))==prior['binary_sha256']
assert sha(ws/'src/M3DGR/deployment-scpgo/runtime/corridor01-indoor.yaml')==installation['profile_sha256']
analysis=json.loads((p/'analysis.json').read_text());frames=json.loads((p/'camera-timestamps.json').read_text())
assert len(frames)==16 and len(analysis['temporal_checks'])==28
assert all(v['suspect_at_least_as_favorable'] for v in analysis['monotone_metric_dominance'].values())
report={'status':'ACCEPTANCE_EVIDENCE_AUDIT_COMPLETE','date':'2026-11-07','camera_frames':16,'reused_icp_runs':48,'new_icp_runs':0,'temporal_comparisons':28,
 'production_source_binary_profile_unchanged':True,'new_acceptance_rule_deployed':False,'independent_relative_pose_obtained':False,
 'max_camera_time_difference_s':max(abs(r['delta_s']) for r in frames),
 'result':'No validated scalar-threshold joint acceptance rule established. Independent reference and trusted controls required.',
 'artifact_sha256':{f.name:sha(f) for f in p.iterdir() if f.is_file() and f.name!='final-report.json'}}
(p/'final-report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['status','camera_frames','temporal_comparisons','production_source_binary_profile_unchanged','independent_relative_pose_obtained']},indent=2))
