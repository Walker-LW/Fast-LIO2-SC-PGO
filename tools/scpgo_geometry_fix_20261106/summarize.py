from pathlib import Path
import json,re,hashlib
from collections import Counter

root=Path(__file__).resolve().parent
real=root/'real'
meta=json.loads((real/'manifest.json').read_text())
terminal={(0,375),(0,376),(0,377),(7,378),(0,379),(0,380),(0,381),(0,382)}
results={}
for variant in ['original','fixed']:
    for poses in ['odom','optimized']:
        key=variant+'-'+poses
        rows=[json.loads(l) for l in (real/(key+'.jsonl')).read_text().splitlines()]
        log=(real/(key+'.log')).read_text()
        matches=re.findall(r'ICP fitness test (passed|failed) \(([^ ]+)',log)
        assert len(rows)==len(matches)==meta['count']
        for r,m in zip(rows,matches):
            r['fitness_m2']=float(m[1]);assert r['accepted']==(m[0]=='passed')
        results[key]=rows
summary={}
for poses in ['odom','optimized']:
    before=results['original-'+poses];after=results['fixed-'+poses]
    transitions=Counter()
    for a,b in zip(before,after):
        assert (a['previous'],a['current'])==(b['previous'],b['current'])
        transitions[f"{'pass' if a['accepted'] else 'fail'} -> {'pass' if b['accepted'] else 'fail'}"]+=1
    summary[poses]={'transitions':dict(transitions),'original_accepted':sum(r['accepted'] for r in before),'fixed_accepted':sum(r['accepted'] for r in after),
                    'terminal':[{'pair':[a['previous'],a['current']],'original_score':a['fitness_m2'],'fixed_score':b['fitness_m2'],'fixed_accepted':b['accepted'],'fixed_source':b['source_points'],'fixed_target':b['target_points']} for a,b in zip(before,after) if (a['previous'],a['current']) in terminal]}
controls={}
for key,rows in results.items():
    controls[key]=[r for r in rows if (r['previous'],r['current']) in {(0,32),(147,310),(0,382)}]
synthetic=json.loads((root/'synthetic/results.json').read_text())
assert all(r['accepted'] and r['translation_error_m']<.001 and r['rotation_error_deg']<.01 and r['target_world_nn_rms_m']<.001 for r in synthetic)
report={'summary':summary,'controls':controls,'all_pairs':results,'synthetic':{'cases':len(synthetic),'pass':True,'max_translation_error_m':max(r['translation_error_m'] for r in synthetic),'max_rotation_error_deg':max(r['rotation_error_deg'] for r in synthetic)},'limitations':meta['limitations']}
(root/'results-summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'summary':summary,'controls':controls,'synthetic':report['synthetic']},indent=2))
