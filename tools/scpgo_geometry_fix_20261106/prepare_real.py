from pathlib import Path
from collections import deque
import re,json,hashlib

out=Path('/fix/real');out.mkdir(exist_ok=True)
run=Path('/run/scpgo');queue=deque();events=[]
for line in (run/'checks/backend-console.log').read_text(errors='replace').splitlines():
    m=re.search(r'Loop detected! - between (\d+) and (\d+)',line)
    if m:queue.append(tuple(map(int,m.groups())))
    m=re.search(r'ICP fitness test (passed|failed) \(([^ ]+)',line)
    if m:
        p,c=queue.popleft();events.append({'previous':p,'current':c,'online_result':m[1],'online_score_rounded':float(m[2])})
selected={}
for e in events:
    if e['online_result']=='passed':selected.setdefault((e['previous'],e['current']),e)
# These are all eight unique terminal pairs in the preceding >=390 s audit.
terminal={(0,375),(0,376),(0,377),(7,378),(0,379),(0,380),(0,381),(0,382)}
for e in events:
    if (e['previous'],e['current']) in terminal:selected.setdefault((e['previous'],e['current']),e)
assert terminal.issubset(selected)
rows=list(selected.values())
(out/'pairs.txt').write_text(''.join(f"{r['previous']} {r['current']}\n" for r in rows))
manifest={'scope':'Paired offline reconstruction with fixed saved poses; not online replay, optimization or ground-truth accuracy.',
          'selection':'All unique pairs with at least one online ICP acceptance plus all eight terminal unique pairs, fixed before repair results.',
          'count':len(rows),'online_accepted_unique':sum(1 for r in rows if r['online_result']=='passed'),
          'variants':['original/odom','fixed/odom','original/optimized','fixed/optimized'],
          'input':'Saved raw body scans are downsampled with SC voxel 0.4 m before original/fixed functions; ICP voxel 0.4 m, all defaults unchanged.',
          'limitations':['Saved poses have text rounding. Online pose snapshots not recorded; old offline results need not equal online logs.','Final optimized poses include old backend feedback; odom snapshot is primary, final optimized snapshot is sensitivity check.','No physical loop labels, GT or full graph optimization. Accepted ICP is not proof of a valid loop.'],
          'pairs':rows,'pending_online_candidates':list(queue)}
for p in [run/'backend/odom_poses.txt',run/'backend/optimized_poses.txt',run/'checks/backend-console.log']:
    manifest.setdefault('input_sha256',{})[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
manifest['scan_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((run/'backend/Scans').glob('*.pcd'))}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
prefix=Path('/fix/synthetic/prefix.cpp').read_text()+'\n#include <pcl/io/pcd_io.h>\n'
tail=Path('/fix/real_cases.cpp').read_text()
for variant,path in [('original',Path('/baseline/laserPosegraphOptimization.cpp')),('fixed',Path('/fix/laserPosegraphOptimization.cpp'))]:
    text=path.read_text()
    def extract(signature):
        start=text.index(signature);brace=text.index('{',start);end=brace+1;depth=1
        while depth:
            depth+=(text[end]=='{')-(text[end]=='}');end+=1
        return text[start:end]
    names=['gtsam::Pose3 Pose6DtoGTSAMPose3(', 'pcl::PointCloud<PointType>::Ptr local2global(', 'void loopFindNearKeyframesCloud(', 'std::optional<gtsam::Pose3> doICPVirtualRelative(']
    (out/(variant+'.cpp')).write_text(prefix+'\n'+'\n\n'.join(extract(n) for n in names)+'\n'+tail)
print('PREPARED',len(rows),'pairs',flush=True)
