"""Update an unstarted prepared b4 checker, retaining old files and manifests."""
from pathlib import Path
import shutil,hashlib,json
r=Path('/home/lw/slam-research/results/baseline/corridor01-full-b4-20261105-001')
tools=Path('/mnt/d/projects/fast-lio2-learning/tools/corridor01_full')
if (r/'checks/ready-once').exists() or (r/'checks/play-once').exists():
    raise RuntimeError('Check/play already attempted; inspect before changing frozen checker')
archive=r/'checks/watch-optional-change-20261105-001';archive.mkdir()
for p in [r/'ready.py',r/'config/tools.sha256',r/'config/prepared-files.sha256']:
    shutil.copy2(p,archive/p.name)
shutil.copyfile(tools/'ready.py',r/'ready.py')
def manifest(path,files):
    with path.open('w') as f:
        for p in sorted(files):f.write(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p)+'\n')
manifest(r/'config/tools.sha256',[p for p in r.iterdir() if p.is_file() and p.suffix in ['.sh','.py']])
manifest(r/'config/prepared-files.sha256',[p for p in (r/'config').iterdir() if p.is_file() and p.name!='prepared-files.sha256'])
with (archive/'reason.json').open('x') as f:
    json.dump(dict(reason='User requested omission of noisy optional coverage observer. Algorithm, recorded topics and estimation parameters unchanged.',
                   workflow='Skip watch and its dependent end-status; review player EOF and post-recording coverage.',
                   limitation='No independent live input counts or live odometry-stability evidence; optional instrumentation load differs from prior run, not a performance comparison.'),f,indent=2)
print('Updated prepared b4 ready checker: watch optional. Old checker/manifests archived. No processes started.')
