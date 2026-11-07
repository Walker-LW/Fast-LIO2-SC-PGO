from pathlib import Path
import hashlib, json, re
import numpy as np

out=Path('/out')
rows=json.loads((out/'results.json').read_text())
scores=re.findall(r'fitness test passed \(([^ ]+)',(out/'console.log').read_text())
assert len(scores)==len(rows)==6
for r,score in zip(rows,scores):
    r['icp_fitness_m2']=float(score)
    r['measurement_check_pass']=r['translation_error_m']<.001 and r['rotation_error_deg']<.01
    r['submap_check_pass']=r['target_world_nn_rms_m']<.001
by={r['case']:r for r in rows}
invariance={}
for name in ['translation','mixed']:
    a=np.array(by[name]['measured']).reshape(4,4)
    b=np.array(by[name+'_changed_world']['measured']).reshape(4,4)
    invariance[name]={'translation_change_m':float(np.linalg.norm(a[:3,3]-b[:3,3])),
                      'expected_translation_change_m':0.0}
source=Path('/source/src/laserPosegraphOptimization.cpp')
snapshot=Path('/snapshot/modified/laserPosegraphOptimization.cpp')
binary=Path('/backend/alaserPGO')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
audit={'source_sha256':sha(source),'source_matches_compiled_snapshot':source.read_bytes()==snapshot.read_bytes(),
       'backend_sha256':sha(binary),'expected_backend_sha256':json.loads(Path('/snapshot/final-report.json').read_text())['new_binary_sha256'],
       'reference_invariance':invariance,'results':rows,
       'production_modified':False,
       'verification_revision':'Second run adds pre-voxel and identical-voxel composition controls; same six cases and unchanged settings. First-run results preserved.',
       'limit':'These tests establish geometry defects, not their share of Corridor01 error or whether fixing them alone will close the real sequence.'}
assert audit['source_matches_compiled_snapshot']
assert audit['backend_sha256']==audit['expected_backend_sha256']
for p in ['prepare.py','prefix.cpp','cases.cpp','run.sh','extracted-functions.cpp','harness.cpp','geometry_probe','results.json','console.log']:
    audit.setdefault('artifact_sha256',{})[p]=sha(out/p)
(out/'verified-results.json').write_text(json.dumps(audit,indent=2)+'\n')
print(json.dumps({'invariance':invariance,'cases':[{k:r[k] for k in ['case','icp_fitness_m2','translation_error_m','rotation_error_deg','target_world_nn_rms_m','original_before_voxel_nn_rms_m','independent_composition_after_voxel_nn_rms_m']} for r in rows]},indent=2))
