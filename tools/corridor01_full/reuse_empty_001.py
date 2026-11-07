"""User-authorized reuse of an unplayed attempt; preserve a complete backup."""
from pathlib import Path
import subprocess,json,shutil,hashlib,datetime
r=Path('/home/lw/slam-research/results/baseline/corridor01-full-b1-20261103-001')
tools=Path('/mnt/d/projects/fast-lio2-learning/tools/corridor01_full')
name=r.name+'-container'
top=subprocess.check_output(['docker','top',name],text=True)
if any(s in top for s in ['roscore','rosmaster','fastlio_mapping','roslaunch','rosbag','coverage_observer','rviz']):
    raise RuntimeError('ROS processes still running; preserve state and stop before reuse')
for folder in ['raw','derived','PCD']:
    if any(p.is_file() for p in (r/folder).rglob('*')):raise RuntimeError('Real output exists: '+folder)
if (r/'checks/play-once').exists():raise RuntimeError('Playback attempt exists; review before reuse')
backup=r/'preplay-backup-20261103-001'
backup.mkdir()
for folder in ['config','checks','ros_logs','Log','PCD','raw','derived']:
    shutil.copytree(r/folder,backup/folder,symlinks=True)
for p in r.iterdir():
    if p.is_file():shutil.copy2(p,backup/p.name)
with (backup/'reason.json').open('x') as f:
    json.dump(dict(reason='User explicitly requested reuse of 001 after preplay TF check failure; raw/derived/PCD empty; ROS processes stopped',
                   time=datetime.datetime.now().astimezone().isoformat(),container_processes=top),f,indent=2)
# Retire launch markers reversibly; start-once remains because the existing container is reused.
retired=backup/'retired-markers';retired.mkdir()
for p in (r/'checks').glob('*-once'):
    if p.name!='start-once':shutil.move(str(p),str(retired/p.name))
for name in ['run.sh','ready.py','compare_pcd.py']:
    shutil.copyfile(tools/name,r/name)
launch=r/'config/full.launch'
text=launch.read_text()
assert 'name="pcd_save/pcd_save_en" value="false"' in text
launch.write_text(text.replace('name="pcd_save/pcd_save_en" value="false"','name="pcd_save/pcd_save_en" value="true"'))
(r/'config/output-policy.json').write_text(json.dumps(dict(online_pcd=True,pcd_interval=-1)))
shutil.copyfile(tools/'README.md',r/'config/protocol-current.md')
def manifest(path,files):
    with path.open('w') as f:
        for p in sorted(files):f.write(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p)+'\n')
manifest(r/'config/tools.sha256',[p for p in r.iterdir() if p.is_file() and p.suffix in ['.sh','.py']])
manifest(r/'config/prepared-files.sha256',[p for p in (r/'config').iterdir() if p.is_file() and p.name!='prepared-files.sha256'])
with (r/'checks/actions.log').open('a') as f:
    f.write(datetime.datetime.now().astimezone().isoformat()+' reuse-unplayed-001 backup='+str(backup)+' online_pcd=true interval=-1\n')
print('Reused existing 001; previous files backed up at',backup)
print('PCD enabled in launch before next algorithm start. No processes started, no playback.')
