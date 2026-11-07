"""Manual experiment preparation/checking. Does not start ROS or playback."""
from pathlib import Path
import hashlib,json,shutil,subprocess,sys

HERE=Path(__file__).resolve().parent
WS=Path('/home/lw/slam-research/fast_lio_ws')
INPUT=Path('/home/lw/slam-research/results/scpgo/scpgo-corridor01-20261106-002')
ROOT=Path('/home/lw/slam-research/results/scpgo/scpgo-corridor01-geometry-ab-20261107-001')
IMAGE='sha256:f306a0ec73450db1aa499376f84c0aa56d1046069ceb435087bd46b54e7e6db5'
VERSIONS={'old':('devel_scpgo_params','65fbef68a38561c1175fa6947687983a3162dd2d3bf86c487db1e1394b3a94d1'),
          'fixed':('devel_scpgo_geometry','03d2e316a408a1906a9613757794af4888512b34b2aae5683d7aab17d4b3c9c6')}
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
def preflight():
 for overlay,expected in VERSIONS.values():assert sha(WS/overlay/'lib/aloam_velodyne/alaserPGO')==expected,overlay
 subprocess.run(['docker','image','inspect',IMAGE],check=True,stdout=subprocess.DEVNULL)
 for name in ['interfaces_0.bag','interfaces_1.bag']:assert (INPUT/'raw'/name).is_file()
 for name in ['backend.launch','backend.yaml']:assert (INPUT/'config'/name).is_file()
 for name in ['watch.py','export_keyframe_maps.py']:assert (INPUT/name).is_file()
 print('PREFLIGHT_PASS: both binaries, image, recorded input and support files exist')

def prepare():
 preflight();assert not ROOT.exists(),'Output root already exists; do not reuse it: '+str(ROOT)
 for variant in VERSIONS:
  name='scpgo-corridor01-geometry-'+variant+'-20261107-001'
  p=subprocess.run(['docker','container','inspect',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  assert p.returncode!=0,'Container name already exists: '+name
 ROOT.mkdir(parents=True)
 inputs={str(p):sha(p) for p in [INPUT/'raw/interfaces_0.bag',INPUT/'raw/interfaces_1.bag']}
 for variant,(overlay,digest) in VERSIONS.items():
  run=ROOT/variant
  for d in ['checks','config','backend','raw','derived','ros_logs']:(run/d).mkdir(parents=True,exist_ok=True)
  for f in ['backend.launch','backend.yaml']:shutil.copy2(INPUT/'config'/f,run/'config'/f)
  for f in ['watch.py','export_keyframe_maps.py']:shutil.copy2(INPUT/f,run/f)
  # The original launch's default config path is explicitly overridden when starting.
  manifest={'variant':variant,'overlay':'/ws/'+overlay,'binary_sha256':digest,'run':str(run),'image':IMAGE,
   'input_files_sha256':inputs,'topics':['/Odometry','/cloud_registered_body','/clock'],
   'clock':'Recorded clock only; no --clock, no synthetic tail. Includes original002 post-input clock interval.',
   'expected_frontend_pairs':4034,'expected_keyframes':383,
   'config_sha256':sha(run/'config/backend.yaml'),'launch_sha256':sha(run/'config/backend.launch')}
  (run/'checks/prepared.json').write_text(json.dumps(manifest,indent=2)+'\n')
  shutil.copy2(HERE/'replay.sh',run/'replay.sh')
 print('PREPARED:',ROOT)
 print('No container, backend or player started.')

def end_check(run):
 m=json.loads((run/'checks/prepared.json').read_text())
 lines=(run/'checks/interface-observer.jsonl').read_text().splitlines();assert lines,'No observer output'
 obs=json.loads(lines[-1]);assert obs['counts'].get('odom')==obs['counts'].get('body')==4034,obs
 assert obs['exact_stamp_pairs']==4034 and obs['odom_without_body']==obs['body_without_odom']==0,obs
 assert not obs['errors'],obs['errors']
 scans=sorted((run/'backend/Scans').glob('*.pcd'))
 counts={name:len((run/'backend'/name).read_text().splitlines()) for name in ['times.txt','odom_poses.txt','optimized_poses.txt']}
 assert len(scans)==383 and set(counts.values())=={383},(len(scans),counts)
 files=scans+[run/'backend'/name for name in counts]
 hashes={str(p.relative_to(run)):sha(p) for p in files}
 report={'status':'END_INPUT_AND_SAVE_CHECK_PASS','observer':obs,'keyframes':len(scans),'pose_counts':counts,'backend_sha256':hashes,
 'limit':'External receipt and saved file coverage only, not internal queue/factor/accuracy verification.'}
 path=run/'checks/end-before-stop.json';assert not path.exists();path.write_text(json.dumps(report,indent=2)+'\n')
 print('END_INPUT_AND_SAVE_CHECK_PASS: 4034 exact pairs, 383 scans and both pose files')
def frozen(run):
 report=json.loads((run/'checks/end-before-stop.json').read_text())
 for rel,digest in report['backend_sha256'].items():assert sha(run/rel)==digest,rel
 assert not list((run/'raw').glob('*.active')),'Recorder still has .active file'
 (run/'checks/stopped-files-check.json').write_text(json.dumps({'backend_files_unchanged':True,'no_active_bag':True},indent=2)+'\n')
 print('STOPPED_FILES_PASS')

if __name__=='__main__':
 action=sys.argv[1]
 if action=='preflight':preflight()
 elif action=='prepare':prepare()
 elif action=='end-check':end_check(ROOT/sys.argv[2])
 elif action=='frozen':frozen(ROOT/sys.argv[2])
 else:raise ValueError(action)
