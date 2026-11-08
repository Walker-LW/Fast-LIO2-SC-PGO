"""Manual full Corridor02 run orchestration. Never starts playback implicitly."""
import argparse
import ast
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
BASE = Path('/home/lw/slam-research')
WS = BASE / 'fast_lio_ws'
REPO = WS / 'src/M3DGR'
IMAGE = 'sha256:f306a0ec73450db1aa499376f84c0aa56d1046069ceb435087bd46b54e7e6db5'
ID = 'scpgo-corridor02-geometry-20261107-001'
RUN = BASE / 'results/scpgo' / ID
BINARY = {'devel/lib/fast_lio/fastlio_mapping': '607d6fc36c4efeaefbb8d375e23fbe199da8356d014f75fbe8ca802f289773be',
          'devel_scpgo_geometry/lib/aloam_velodyne/alaserPGO': '03d2e316a408a1906a9613757794af4888512b34b2aae5683d7aab17d4b3c9c6'}


def select_run_id(requested=None):
    """Tool version directory stays fixed; prepared copies use their manifest ID."""
    global ID, RUN
    manifest = HERE / 'config/manifest.json'
    if manifest.is_file():
        recorded = json.loads(manifest.read_text())['id']
        assert requested is None or requested == recorded, 'ID conflicts with this prepared run manifest'
        selected = recorded
    else:
        selected = requested if requested is not None else ID  # Preserve the original command interface.
    match = re.fullmatch(r'scpgo-corridor02-geometry-(\d{8})-(\d{3})', selected)
    assert match, 'Use scpgo-corridor02-geometry-YYYYMMDD-NNN'
    datetime.strptime(match.group(1), '%Y%m%d')
    ID = selected
    RUN = BASE / 'results/scpgo' / ID
    if manifest.is_file():
        assert HERE == RUN, 'Prepared script must execute from its own recorded output directory'


def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''): h.update(b)
    return h.hexdigest()


def call(args, **kw):
    return subprocess.check_output(args, text=True, **kw).strip()


def write(p, value):
    with Path(p).open('x') as f: json.dump(value, f, indent=2)


def preflight():
    for rel, digest in BINARY.items(): assert sha(WS / rel) == digest, rel
    assert sha(REPO / 'deployment-scpgo/runtime/corridor01-indoor.yaml') == 'c6e942ce7ab4cf5e2190834c9303e72f877a796e10deeadbfbb2b670e85a797f'
    assert call(['git', '-C', str(REPO), 'branch', '--show-current']) == 'integration/scpgo'
    assert not call(['git', '-C', str(REPO), 'status', '--porcelain', '--', 'baseline_systems'])
    call(['docker', 'image', 'inspect', IMAGE])
    assert (BASE / 'datasets/Corridor02.bag').is_file()
    assert not call(['docker', 'ps', '-q']), 'Other running containers: inspect before starting the experiment.'
    for p in HERE.glob('*.py'): ast.parse(p.read_text())
    subprocess.run(['bash', '-n', str(HERE / 'run.sh')], check=True)
    print('PREFLIGHT_PASS: binaries, approved profile, input and syntax; no algorithm started')
    print('EXPERIMENT_ID:', ID)
    print('OUTPUT_DIRECTORY:', RUN)


def prepare():
    preflight()
    assert not RUN.exists(), 'Output directory occupied; preserve it and stop: ' + str(RUN)
    assert subprocess.run(['docker', 'container', 'inspect', ID], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0
    RUN.mkdir(parents=True)
    for name in ['checks', 'config', 'raw', 'derived', 'backend', 'ros_logs', 'Log', 'PCD']: (RUN / name).mkdir()
    for name in ['manage.py', 'run.sh', 'runtime.py', 'analyze.py']:
        shutil.copyfile(HERE / name, RUN / name)
    shutil.copyfile(REPO / 'deployment-scpgo/runtime/export_keyframe_maps.py', RUN / 'export_keyframe_maps.py')
    shutil.copyfile(REPO / 'deployment-scpgo/runtime/watch.py', RUN / 'watch.py')
    shutil.copyfile(REPO / 'deployment-scpgo/runtime/corridor01-indoor.yaml', RUN / 'config/backend.yaml')
    shutil.copyfile(REPO / 'deployment-scpgo/runtime/backend-parameterized.launch', RUN / 'config/backend.launch')
    shutil.copyfile(BASE / 'datasets/GTCorridor02.txt', RUN / 'config/GTCorridor02.txt')
    root = ET.parse(REPO / 'deployment-scpgo/baseline/corridor02/full.launch').getroot()
    for key in ['publish/scan_publish_en', 'publish/scan_bodyframe_pub_en', 'publish/dense_publish_en']:
        ET.SubElement(root, 'param', name=key, type='bool', value='true')
    ET.ElementTree(root).write(RUN / 'config/frontend.launch', encoding='unicode')
    # Bag metadata and launch expansion only. No ROS master or estimator is run.
    bag = BASE / 'datasets/Corridor02.bag'
    command = ['docker', 'run', '--rm', '--network', 'none', '--read-only', '--tmpfs', '/tmp:rw,size=128m',
               '-e', 'ROS_HOME=/tmp/preparation-ros', '-e', 'ROS_LOG_DIR=/tmp/ros_logs',
               '--mount', f'type=bind,source={WS},target=/ws,readonly',
               '--mount', f'type=bind,source={BASE / "datasets"},target=/datasets,readonly',
               '--mount', f'type=bind,source={RUN},target=/run/scpgo', '--entrypoint', '/bin/bash', IMAGE, '-c',
               'set -e; source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; source /ws/devel_scpgo_geometry/setup.bash; '
               'export CMAKE_PREFIX_PATH=/ws/devel:/ws/devel_scpgo_geometry:/opt/ros/noetic; '
               'rosbag info --yaml /datasets/Corridor02.bag > /run/scpgo/config/input-info.yaml && '
               'roslaunch --dump-params /run/scpgo/config/frontend.launch > /run/scpgo/config/frontend-expanded.yaml && '
               'export CMAKE_PREFIX_PATH=/ws/devel_scpgo_geometry:/ws/devel:/opt/ros/noetic; '
               'roslaunch --dump-params /run/scpgo/config/backend.launch run_dir:=/run/scpgo config:=/run/scpgo/config/backend.yaml > /run/scpgo/config/backend-expanded.yaml']
    # Use explicit subprocess status plus required nonempty outputs.
    subprocess.run(command, check=True)
    for name in ['input-info.yaml', 'frontend-expanded.yaml', 'backend-expanded.yaml']:
        assert (RUN / 'config' / name).stat().st_size > 0
    frozen = {str(p.relative_to(RUN)): sha(p) for p in RUN.rglob('*') if p.is_file()}
    write(RUN / 'config/manifest.json', dict(id=ID, image=IMAGE, source_commit=call(['git', '-C', str(REPO), 'rev-parse', 'HEAD']),
        input_sha256=sha(bag), input_bytes=bag.stat().st_size, binaries=BINARY, frozen=frozen,
        experiment='Full raw LiDAR+IMU, frontend blind1 and fixed geometry backend with unchanged approved numerical profile',
        playback='Manual full 1x playback; keep-alive clock remains until finish; record EOF-to-stop interval.',
        prior_frontend_output_count=2930, prior_count_is_reference_not_forced_result=True,
        reference_limit='GTCorridor02 is one transform, not a sampled GT trajectory; no ATE/RPE evaluated.',
        frontend_diagnostics='No added Jacobian/eigenvalue instrumentation; cannot diagnose degeneracy from this run alone.'))
    print('PREPARED_NOT_STARTED:', RUN)


def ros(*args):
    return ['docker', 'exec', ID, '/bin/bash', '-c',
            'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; source /ws/devel_scpgo_geometry/setup.bash; exec "$@"', '_', *args]


def node_present(name):
    # rosnode info may return 0 even when it prints "unknown node".
    # Require a successful master query, then test the registered names explicitly.
    result = subprocess.run(ros('rosnode', 'list'), capture_output=True, text=True, check=True, timeout=15)
    return name in result.stdout.splitlines()


def stop_node(name):
    if node_present(name):
        subprocess.run(ros('rosnode', 'kill', name), check=True)
        for _ in range(20):
            if not node_present(name): return
            time.sleep(1)
        raise RuntimeError('Node did not stop; preserve container: ' + name)


def finish():
    assert (RUN / 'checks/player-once').exists(), 'Playback not started'
    assert not (RUN / 'checks/closed.json').exists(), 'Already closed; do not repeat'
    assert node_present('/scpgo_input'), 'Keep-alive player absent: preserve state and inspect'
    # This succeeds only after all expected raw input messages were observed.
    subprocess.run(ros('python3', '/run/scpgo/runtime.py', 'input-complete'), check=True)
    stop_node('/laserMapping')
    time.sleep(3)
    subprocess.run(ros('python3', '/run/scpgo/runtime.py', 'save-check'), check=True)
    stop_node('/scpgo/alaserPGO')
    stop_node('/scpgo_interface_watch')
    stop_node('/scpgo_record')
    for _ in range(30):
        if not list((RUN / 'raw').glob('*.active')): break
        time.sleep(1)
    assert not list((RUN / 'raw').glob('*.active')), 'Recorder not closed; keep player/container'
    stop_node('/scpgo_input')
    subprocess.run(['docker', 'stop', '-t', '20', ID], check=True)
    before = json.loads((RUN / 'checks/end-before-stop.json').read_text())
    changed = [rel for rel, digest in before['saved_sha256'].items() if sha(RUN / rel) != digest]
    write(RUN / 'checks/closed.json', dict(no_active_bag=True, saved_files_changed_at_stop=changed, closed_wall_time_ns=time.time_ns(),
        note='Node shutdown may retain known backend abort. Examine logs; closure is not proof of factor consumption or accuracy.'))
    assert not changed, 'Saved files changed during shutdown; retain results and inspect: ' + str(changed)
    print('CLOSED_AND_FROZEN_PASS')


def offline(action):
    closed = json.loads((RUN / 'checks/closed.json').read_text())
    assert not closed['saved_files_changed_at_stop']
    for rel, digest in json.loads((RUN / 'checks/end-before-stop.json').read_text())['saved_sha256'].items():
        assert sha(RUN / rel) == digest, rel
    script = 'export_keyframe_maps.py /run/scpgo /run/scpgo/derived/keyframe-maps' if action == 'export' else 'analyze.py /run/scpgo'
    args = ['docker', 'run', '--rm', '--network', 'none', '--read-only', '--tmpfs', '/tmp:rw,size=256m',
            '-e', 'MPLCONFIGDIR=/tmp/matplotlib', '-e', 'PYTHONDONTWRITEBYTECODE=1',
            '--mount', f'type=bind,source={RUN},target=/run/scpgo', '--entrypoint', '/bin/bash', IMAGE, '-c',
            'source /opt/ros/noetic/setup.bash; exec python3 /run/scpgo/' + script]
    subprocess.run(args, check=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['preflight', 'prepare', 'finish', 'export', 'analyze'])
    p.add_argument('--id', dest='run_id', help='Explicit experiment ID for preflight/prepare; prepared copies read their frozen manifest')
    args = p.parse_args()
    select_run_id(args.run_id)
    action = args.action
    if action == 'preflight': preflight()
    elif action == 'prepare': prepare()
    elif action == 'finish': finish()
    else: offline(action)
