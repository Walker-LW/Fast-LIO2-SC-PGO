"""Summarize observer logs without stopping any process."""
from pathlib import Path
import json,sys,time
r=Path(sys.argv[1]);latest=None
for line in (r/'checks/live-progress.log').read_text(errors='replace').splitlines():
    if line.startswith('{'):
        try:latest=json.loads(line)
        except ValueError:pass
if latest is None:raise RuntimeError('No observer snapshot')
print(json.dumps(latest,indent=2))
for topic,count in latest['expected_input_counts'].items():
    if latest['observed'].get(topic,{}).get('count')!=count:
        raise RuntimeError('Input observer has not received expected count: '+topic)
odom=latest['observed'].get('/Odometry',{})
if not odom.get('count'):raise RuntimeError('No observed odometry')
if time.time()-latest['wall_unix']>5:raise RuntimeError('Observer snapshot is stale; inspect observer terminal')
if odom['seconds_since_last_message']<10:raise RuntimeError('Odometry not stable for 10 seconds; wait while clock runs')
print('OBSERVER_COUNTS_COMPLETE_AND_OUTPUT_STABLE: inspect player EOF and output endpoint; not an accuracy or lossless guarantee. Keep player running during algorithm exit.')
