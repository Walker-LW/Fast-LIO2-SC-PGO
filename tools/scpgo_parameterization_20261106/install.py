from pathlib import Path
import hashlib, json, shutil

root = Path(__file__).resolve().parent
base = Path('/home/lw/slam-research')
repo = base/'fast_lio_ws/src/M3DGR'
pkg = repo/'baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO'
record = base/'results/validation/scpgo-parameterization-20261106-001'
record.mkdir(exist_ok=False)
targets = {'laserPosegraphOptimization.cpp': pkg/'src/laserPosegraphOptimization.cpp',
           'Scancontext.h': pkg/'include/scancontext/Scancontext.h',
           'Scancontext.cpp': pkg/'include/scancontext/Scancontext.cpp'}
for name,path in targets.items():
    assert path.read_text() == (root/'source'/name).read_text(), f'Source changed: {path}'
    shutil.copy2(path,record/name)
protected = [base/'fast_lio_ws/devel/lib/fast_lio/fastlio_mapping',
             base/'fast_lio_ws/devel_scpgo/lib/aloam_velodyne/alaserPGO']
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
(record/'protected-binaries.json').write_text(json.dumps({str(p):sha(p) for p in protected},indent=2))
for name,path in targets.items(): shutil.copyfile(root/'modified'/name,path)
shutil.copyfile(root/'modified/BackendParameters.h',pkg/'include/aloam_velodyne/BackendParameters.h')
runtime = repo/'deployment-scpgo/runtime'
for name in ('corridor01-indoor.yaml','backend-parameterized.launch','build-parameterized.sh'):
    shutil.copyfile(root/name,runtime/name)
print(record)
