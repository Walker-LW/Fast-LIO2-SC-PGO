#!/usr/bin/env python3
"""Validate the dependency image without starting ROS nodes or writing the workspace."""
from pathlib import Path
import argparse, hashlib, json, subprocess
parser=argparse.ArgumentParser()
parser.add_argument('--output',required=True)
args=parser.parse_args()
deploy=Path(__file__).resolve().parent.parent
# The repository is /home/lw/slam-research/fast_lio_ws/src/M3DGR.
base=deploy.parent.parent.parent.parent
output=Path(args.output).resolve()
assert base.name=='slam-research', str(base)
assert (base/'results/validation').resolve() in output.parents
output.mkdir(parents=True,exist_ok=False)
expected='sha256:fcfce4546a744e9e76b02e79485ac60f40cc54522362cc8ad7654871c98c2f7f'
new_tag='slam-research-fastlio-scpgo:noetic-gtsam403-v1'
def call(argv,log=None,expected_rc=0):
    r=subprocess.run(argv,capture_output=True,text=True)
    if log: (output/log).write_text(r.stdout+'\n'+r.stderr)
    if r.returncode!=expected_rc:
        raise RuntimeError(str(argv)+' returned '+str(r.returncode)+'\n'+r.stderr)
    return r.stdout
bases=json.loads(call(['docker','image','inspect','slam-research-fastlio:noetic','slam-research-fastlio:baseline-pre-scpgo-20261105']))
assert all(item['Id']==expected for item in bases)
new=json.loads(call(['docker','image','inspect',new_tag]))[0]
assert new['Id']!=expected
assert new['RootFS']['Layers'][:len(bases[0]['RootFS']['Layers'])]==bases[0]['RootFS']['Layers']
assert new['Config']['Labels']['slam.base.image']==expected
assert new['Config']['Labels']['slam.gtsam.commit']=='6425000775da29c93719f37dce3f2de38a0064ec'
exe=base/'fast_lio_ws/devel/lib/fast_lio/fastlio_mapping'
exe_sha=hashlib.sha256(exe.read_bytes()).hexdigest()
assert exe_sha=='607d6fc36c4efeaefbb8d375e23fbe199da8356d014f75fbe8ca802f289773be'
common=['docker','run','--rm','--network','none','--read-only','--tmpfs','/tmp:rw,noexec,nosuid,size=64m','--entrypoint','bash']
probe=call(common+[new_tag,'-lc','set -e; source /opt/ros/noetic/setup.bash; /opt/scpgo-dependency-probe/build/scpgo_dependency_probe'],'dependency-probe.log')
assert 'PASS: GTSAM ISAM2' in probe and 'cv_bridge image conversion' in probe
source_commit=call(common+[new_tag,'-lc','cat /opt/scpgo-environment/gtsam-commit.txt']).strip()
assert source_commit=='6425000775da29c93719f37dce3f2de38a0064ec'
ldd=call(common+[new_tag,'-lc','set -e; source /opt/ros/noetic/setup.bash; ldd /opt/scpgo-dependency-probe/build/scpgo_dependency_probe'],'probe-ldd.log')
assert 'not found' not in ldd
frontend_ldd=call(['docker','run','--rm','--network','none','--read-only','--tmpfs','/tmp:rw,noexec,nosuid,size=64m','--mount','type=bind,source='+str(exe)+',target=/baseline-fastlio,readonly','--entrypoint','bash',new_tag,'-lc','set -e; source /opt/ros/noetic/setup.bash; ldd /baseline-fastlio'],'frontend-ldd.log')
assert 'not found' not in frontend_ldd
packages=call(common+[new_tag,'-lc','cat /opt/scpgo-environment/dpkg-packages.txt'],'packages.txt')
cache=call(common+[new_tag,'-lc','cat /opt/scpgo-environment/gtsam-cmake-cache.txt'],'gtsam-cmake-cache.txt')
options={line.split(':',1)[0]:line.split('=',1)[1] for line in cache.splitlines() if line and not line.startswith(('#','//')) and ':' in line and '=' in line}
for name,value in {'GTSAM_USE_SYSTEM_EIGEN':'ON','GTSAM_WITH_TBB':'OFF','GTSAM_WITH_EIGEN_MKL':'OFF','GTSAM_BUILD_TESTS':'OFF','GTSAM_BUILD_WRAP':'OFF','GTSAM_BUILD_UNSTABLE':'OFF','GTSAM_BUILD_WITH_MARCH_NATIVE':'OFF'}.items():
    assert options.get(name)==value,(name,options.get(name))
call(['docker','run','--rm','--network','none','--read-only','--tmpfs','/tmp:rw,noexec,nosuid,size=64m',new_tag,'true'],'entrypoint-missing-workspace.log',expected_rc=2)
report={'status':'PASS','new_image_tag':new_tag,'new_image_id':new['Id'],'image_size_bytes':new['Size'],'base_image_id':expected,'base_tags_unchanged':True,'base_layers_preserved':True,'frontend_sha256':exe_sha,'frontend_runtime_libraries_resolved':True,'dependency_runtime_probe':probe.strip(),'runtime_workspace_guard':'Refuses startup without both workspace environments','gtsam_commit':new['Config']['Labels']['slam.gtsam.commit'],'gtsam_cmake_options':{key:value for key,value in options.items() if key.startswith(('GTSAM_','BUILD_SHARED_LIBS'))},'workspace_backend_compiled':False,'experiment_started':False,'logs':str(output)}
(output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
