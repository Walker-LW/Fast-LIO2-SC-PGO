"""User-authorized overwrite of only the interrupted old/001 trial. No playback."""
from pathlib import Path
import json,shutil,subprocess,time

run=Path('/home/lw/slam-research/results/scpgo/scpgo-corridor01-geometry-ab-20261107-001/old')
container='scpgo-corridor01-geometry-old-20261107-001'
assert run.resolve()==run and run.is_dir()
prepared=(run/'checks/prepared.json').read_bytes()
meta=json.loads(prepared);assert meta['variant']=='old' and meta['run']==str(run)
assert not (run/'checks/stopped-files-check.json').exists(),'Already completed; this reset is for the interrupted attempt only.'
inspect=subprocess.run(['docker','inspect',container],capture_output=True,text=True)
if inspect.returncode==0:
 info=json.loads(inspect.stdout)[0]
 mounts={m['Destination']:m['Source'] for m in info['Mounts']}
 assert Path(mounts['/run/scpgo']).resolve()==run
 if info['State']['Running']:
  def ros(*args):
   return subprocess.run(['docker','exec',container,'/bin/bash','-c','source /opt/ros/noetic/setup.bash; exec "$@"','_',*args],capture_output=True,text=True)
  for node in ['/scpgo_input','/scpgo_interface_watch','/scpgo_record']:
   if ros('rosnode','info',node).returncode==0:
    result=ros('rosnode','kill',node);assert result.returncode==0,result.stdout+result.stderr
  for _ in range(30):
   if not list((run/'raw').glob('*.active')):break
   time.sleep(1)
  assert not list((run/'raw').glob('*.active')),'Recorder not closed; output retained, no cleanup performed.'
  if ros('rosnode','info','/scpgo/alaserPGO').returncode==0:ros('rosnode','kill','/scpgo/alaserPGO')
  subprocess.run(['docker','stop','-t','20',container],check=True)
 subprocess.run(['docker','rm',container],check=True)
# Delete only trial output; source002 input, configuration, fixed trial and binaries remain outside this list.
for name in ['backend','raw','derived','ros_logs','checks']:
 path=run/name
 assert path.resolve()==path and path.parent==run
 if path.exists():shutil.rmtree(path)
 path.mkdir()
(run/'checks/prepared.json').write_bytes(prepared)
script=(run/'replay.sh').read_text()
script=script.replace('rosbag/play --quiet --pause','rosbag/play --pause')
(run/'replay.sh').write_text(script)
fixed=run.parent/'fixed'
if (fixed/'replay.sh').exists() and not (fixed/'checks/backend-started-once').exists():
 text=(fixed/'replay.sh').read_text().replace('rosbag/play --quiet --pause','rosbag/play --pause')
 (fixed/'replay.sh').write_text(text)
print('RESET_OLD_PASS: same001/old directory ready; progress enabled; no container or player started.')
