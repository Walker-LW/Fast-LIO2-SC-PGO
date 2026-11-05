#!/usr/bin/env python3
"""Check the built backend and overlay without executing SLAM nodes."""
from pathlib import Path
import argparse, hashlib, json, os, subprocess
parser=argparse.ArgumentParser()
parser.add_argument('--baseline-artifacts',required=True)
parser.add_argument('--output',required=True)
args=parser.parse_args()
deploy=Path(__file__).resolve().parent.parent
repo=deploy.parent
base=repo.parent.parent.parent
workspace=base/'fast_lio_ws'
output=Path(args.output).resolve()
assert (base/'results/validation').resolve() in output.parents
output.mkdir(parents=True,exist_ok=False)
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda:f.read(1024*1024),b''): h.update(data)
    return h.hexdigest()
def call(argv):
    r=subprocess.run(argv,capture_output=True,text=True)
    if r.returncode: raise RuntimeError(str(argv)+'\n'+r.stdout+'\n'+r.stderr)
    return r.stdout
baseline=json.loads(Path(args.baseline_artifacts).read_text())
actual={}
for name in ['build','devel']:
    for path in (workspace/name).rglob('*'):
        if path.is_symlink(): actual[str(path.relative_to(workspace))]={'symlink':str(path.readlink())}
        elif path.is_file(): actual[str(path.relative_to(workspace))]={'sha256':digest(path),'bytes':path.stat().st_size}
assert actual==baseline,'Original build/devel contents changed'
image_lock=json.loads((deploy/'locks/scpgo-image.json').read_text())
new_image=image_lock['new_image_tag']
assert call(['docker','image','inspect',new_image,'--format','{{.Id}}']).strip()==image_lock['new_image_id']
for tag in ['slam-research-fastlio:noetic','slam-research-fastlio:baseline-pre-scpgo-20261105']:
    assert call(['docker','image','inspect',tag,'--format','{{.Id}}']).strip()==image_lock['base_image_id']
frontend=workspace/'devel/lib/fast_lio/fastlio_mapping'
assert digest(frontend)==image_lock['frontend_sha256']
backend=workspace/'devel_scpgo/lib/aloam_velodyne/alaserPGO'
assert backend.is_file() and os.access(backend,os.X_OK)
assert not (workspace/'devel_scpgo/lib/fast_lio/fastlio_mapping').exists()
cache={}
for line in (workspace/'build_scpgo/CMakeCache.txt').read_text().splitlines():
    if line and not line.startswith(('#','//')) and ':' in line and '=' in line: cache[line.split(':',1)[0]]=line.split('=',1)[1]
for key,value in {'CMAKE_HOME_DIRECTORY':'/ws/src','CATKIN_DEVEL_PREFIX':'/ws/devel_scpgo','CATKIN_WHITELIST_PACKAGES':'aloam_velodyne','CMAKE_INSTALL_PREFIX':'/ws/install_scpgo'}.items():
    assert cache.get(key)==value,(key,cache.get(key))
flags=(workspace/'build_scpgo/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO/CMakeFiles/alaserPGO.dir/flags.make').read_text()
assert '-std=gnu++17' in flags or '-std=c++17' in flags
probe=r"""
from pathlib import Path
import subprocess,json,os
from catkin_pkg.packages import find_packages
Path(os.environ['ROS_HOME']).mkdir(parents=True,exist_ok=True)
def out(args):return subprocess.check_output(args,text=True).strip()
backend_pkg=out(['rospack','find','aloam_velodyne'])
frontend_pkg=out(['rospack','find','fast_lio'])
assert backend_pkg=='/ws/src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO'
assert frontend_pkg=='/ws/src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/fast_lio2'
prefix=os.environ['CMAKE_PREFIX_PATH'].split(':')
assert prefix[:2]==['/ws/devel_scpgo','/ws/devel'],prefix
ldd=out(['ldd','/ws/devel_scpgo/lib/aloam_velodyne/alaserPGO'])
assert 'not found' not in ldd
packages=sorted(p.name for p in find_packages('/ws/src',exclude_subspaces=True).values())
assert packages==sorted(['aloam_velodyne','fast_lio','livox_ros_driver','imu_transformer','rs_to_velodyne']),packages
print(json.dumps({'backend_package':backend_pkg,'frontend_package':frontend_pkg,'cmake_prefix_path':prefix,'workspace_packages':packages,'ldd':ldd}))
"""
cmd=['docker','run','--rm','-i','--network','none','--read-only','--tmpfs','/tmp:rw,noexec,nosuid,size=64m','--env','ROS_HOME=/tmp/scpgo-verify-ros','--mount','type=bind,source='+str(workspace)+',target=/ws,readonly',new_image,'python3','-']
r=subprocess.run(cmd,input=probe,capture_output=True,text=True)
(output/'runtime-inspection.log').write_text(r.stdout+'\n'+r.stderr)
if r.returncode:raise RuntimeError(r.stdout+'\n'+r.stderr)
runtime=json.loads(r.stdout)
(output/'backend-ldd.txt').write_text(runtime.pop('ldd')+'\n')
manifest=json.loads((deploy/'upstream/fast_lio_slam.json').read_text())
source_root=repo/manifest['target_directory']
assert all(digest(source_root/name)==data['sha256'] for name,data in manifest['files'].items())
report={'status':'PASS','backend_executable':'/ws/devel_scpgo/lib/aloam_velodyne/alaserPGO','backend_sha256':digest(backend),'backend_bytes':backend.stat().st_size,'image_id':image_lock['new_image_id'],'original_artifact_entries_unchanged':len(baseline),'original_frontend_sha256':digest(frontend),'original_images_unchanged':True,'scpgo_source_files_unchanged':len(manifest['files']),'cache':{key:cache[key] for key in ['CMAKE_HOME_DIRECTORY','CATKIN_DEVEL_PREFIX','CATKIN_WHITELIST_PACKAGES','CMAKE_INSTALL_PREFIX','CMAKE_BUILD_TYPE']},'cpp_standard':'C++17','backend_libraries_resolved':True,'backend_node_started':False,'experiment_started':False,'runtime':runtime,'output_directory':str(output)}
(output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
