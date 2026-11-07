from pathlib import Path
import hashlib,json,shutil

out=Path(__file__).resolve().parent
rows=json.loads((out/'synthetic/results.json').read_text())
assert len(rows)==6
for r in rows:
    assert r['accepted'] and r['translation_error_m']<.001 and r['rotation_error_deg']<.01 and r['target_world_nn_rms_m']<.001,r
ws=Path('/home/lw/slam-research/fast_lio_ws')
src=ws/'src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO/src/laserPosegraphOptimization.cpp'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((out/'change-manifest.json').read_text())
assert sha(src)==manifest['baseline_sha256'], 'Source changed since reviewed snapshot'
archive=Path('/home/lw/slam-research/results/validation/scpgo-geometry-fix-20261106-001')
archive.mkdir(parents=True,exist_ok=False)
shutil.copy2(src,archive/'laserPosegraphOptimization.before.cpp')
protected={str(ws/p):sha(ws/p) for p in ['devel/lib/fast_lio/fastlio_mapping','devel_scpgo/lib/aloam_velodyne/alaserPGO','devel_scpgo_params/lib/aloam_velodyne/alaserPGO']}
(archive/'protected-binaries.json').write_text(json.dumps(protected,indent=2)+'\n')
profile=ws/'src/M3DGR/deployment-scpgo/runtime/corridor01-indoor.yaml'
shutil.copy2(profile,archive/'corridor01-indoor.before.yaml')
src.write_bytes((out/'laserPosegraphOptimization.cpp').read_bytes())
assert sha(src)==manifest['candidate_sha256']
shutil.copy2(out/'geometry-fix.patch',archive/'geometry-fix.patch')
(out/'installation.json').write_text(json.dumps({'source':str(src),'backup':str(archive),'source_sha256':sha(src),'protected_binaries':protected,'profile_sha256':sha(profile),'synthetic_tests_pass':True},indent=2)+'\n')
print('INSTALLED',src)
