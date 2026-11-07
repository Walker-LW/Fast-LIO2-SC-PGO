"""Host WSL static real-environment check: no ROS master or estimator start."""
from pathlib import Path
import json
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import manage

with tempfile.TemporaryDirectory(prefix='corridor02-static-') as d:
    root=Path(d);(root/'config').mkdir()
    config=manage.REPO/'deployment-scpgo/runtime'
    for name in ['backend-parameterized.launch','corridor01-indoor.yaml']:
        shutil.copyfile(config/name,root/'config'/('backend.launch' if name.endswith('.launch') else 'backend.yaml'))
    frontend=ET.parse(manage.REPO/'deployment-scpgo/baseline/corridor02/full.launch').getroot()
    for key in ['publish/scan_publish_en','publish/scan_bodyframe_pub_en','publish/dense_publish_en']:
        ET.SubElement(frontend,'param',name=key,type='bool',value='true')
    ET.ElementTree(frontend).write(root/'config/frontend.launch',encoding='unicode')
    (root/'config/manifest.json').write_text(json.dumps({'binaries':manage.BINARY,'frozen':{}}))
    script='''set -e
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
source /ws/devel_scpgo_geometry/setup.bash
python3 /review/runtime.py identity
export CMAKE_PREFIX_PATH=/ws/devel:/ws/devel_scpgo_geometry:/opt/ros/noetic
roslaunch --dump-params /run/scpgo/config/frontend.launch > /tmp/front.yaml
export CMAKE_PREFIX_PATH=/ws/devel_scpgo_geometry:/ws/devel:/opt/ros/noetic
roslaunch --dump-params /run/scpgo/config/backend.launch run_dir:=/run/scpgo config:=/run/scpgo/config/backend.yaml > /tmp/back.yaml
python3 - <<'PY'
import yaml
front=yaml.safe_load(open('/tmp/front.yaml'));back=yaml.safe_load(open('/tmp/back.yaml'))
assert front['/preprocess/blind']==1 and front['/pcd_save/pcd_save_en'] is False
assert front['/publish/scan_bodyframe_pub_en'] is True and front['/publish/dense_publish_en'] is True
assert back['/scpgo/sc_max_radius']==40 and back['/scpgo/keyframe_meter_gap']==1
assert back['/scpgo/icp_max_correspondence_distance']==150 and back['/scpgo/icp_fitness_threshold']==.3
print('REAL_LAUNCH_EXPANSION_AND_APPROVED_VALUES_PASS')
PY
'''
    command=['docker','run','--rm','--network','none','--read-only','--tmpfs','/tmp:rw,size=128m',
             '-e','ROS_HOME=/tmp/static-ros','-e','ROS_LOG_DIR=/tmp/ros_logs',
             '--mount','type=bind,source='+str(manage.WS)+',target=/ws,readonly',
             '--mount','type=bind,source='+str(root)+',target=/run/scpgo,readonly',
             '--mount','type=bind,source='+str(manage.HERE)+',target=/review,readonly',
             '--entrypoint','/bin/bash',manage.IMAGE,'-c',script]
    subprocess.run(command,check=True)
print('STATIC_REAL_ENVIRONMENT_PASS_NO_ROS_OR_PLAYBACK_STARTED')
