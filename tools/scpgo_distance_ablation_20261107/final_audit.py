from pathlib import Path
import json,hashlib,subprocess

out=Path(__file__).resolve().parent
previous=out.parent/'scpgo_terminal_diagnosis_20261106'
fix=out.parent/'scpgo_geometry_fix_20261106'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
meta=json.loads((out/'manifest.json').read_text())
summary=json.loads((out/'summary.json').read_text())
assert len(summary['results'])==48 and summary['baseline_scores_reproduced']
for name,h in meta['input_sha256'].items():assert sha(previous/name)==h,name
prior=json.loads((previous/'final-report.json').read_text())
installed=json.loads((fix/'installation.json').read_text())
ws=Path('/home/lw/slam-research/fast_lio_ws')
source=ws/'src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO/src/laserPosegraphOptimization.cpp'
binary=ws/'devel_scpgo_geometry/lib/aloam_velodyne/alaserPGO'
profile=ws/'src/M3DGR/deployment-scpgo/runtime/corridor01-indoor.yaml'
assert sha(source)==prior['source_sha256'] and sha(binary)==prior['binary_sha256']
assert sha(profile)==installed['profile_sha256']
for p,h in installed['protected_binaries'].items():assert sha(Path(p))==h,p
image=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}','slam-research-fastlio-scpgo:noetic-gtsam403-v1'],text=True).strip()
assert image=='sha256:f306a0ec73450db1aa499376f84c0aa56d1046069ceb435087bd46b54e7e6db5'
report={'status':'OFFLINE_SINGLE_FACTOR_COMPARISON_COMPLETE','date':'2026-11-07','image':image,'pairs':12,'runs':48,
        'baseline_150m_scores_reproduced':True,'exact_input_hashes_unchanged':True,'production_source_binaries_profile_unchanged':True,
        'original_acceptance_rule_unchanged':True,'full_slam_replayed':False,'map_improvement_demonstrated':False,
        'aggregate':summary['aggregate'],'limits':meta['limitations'],
        'conclusions':['All eight terminal pairs have lower fixed-source-range<=40m MSE under every tested smaller gate than150m.',
                       'All eight terminal pairs still fail the original full-point fitness acceptance rule at every gate.',
                       'Smaller gates are not monotonically better; gate5-to2 changes (0,381) by3.136m and14.279deg.',
                       'Tight gates do not remove the low-fitness control (41,151); physical loop validity remains unverified.'],
        'artifact_sha256':{p.name:sha(p) for p in out.iterdir() if p.is_file() and p.name!='final-report.json'}}
(out/'final-report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['status','pairs','runs','baseline_150m_scores_reproduced','exact_input_hashes_unchanged','production_source_binaries_profile_unchanged','full_slam_replayed']},indent=2))
