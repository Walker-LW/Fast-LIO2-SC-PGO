#!/usr/bin/env python3
"""Prepare an isolated, stopped container. Never starts ROS or plays data."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


def call(*args):
    return subprocess.check_output(args, text=True).strip()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequence', required=True, choices=['Corridor01', 'Corridor02'])
    parser.add_argument('--frontend-policy', required=True, choices=['baseline', 'no-online-pcd'])
    parser.add_argument('--backend-profile', required=True, choices=['source-defaults'])
    parser.add_argument('--id', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'scpgo-corridor0[12]-[0-9]{8}-[0-9]{3}', args.id):
        parser.error('ID must be scpgo-corridor01/02-YYYYMMDD-NNN')
    if args.sequence.lower() not in args.id:
        parser.error('Sequence and ID differ')
    deploy = Path(__file__).resolve().parent.parent
    repo = deploy.parent
    base = repo.parents[2]
    workspace = base / 'fast_lio_ws'
    assert base == Path('/home/lw/slam-research'), base
    assert call('git', '-C', str(repo), 'branch', '--show-current') == 'integration/scpgo'
    source = repo / 'baseline_systems/Fast_LIO2_M3DGR/src'
    assert not call('git', '-C', str(repo), 'status', '--porcelain', '--', str(source))
    image = json.loads((deploy / 'locks/scpgo-image.json').read_text())
    built = json.loads((deploy / 'locks/scpgo-backend-build.json').read_text())
    for tag in ['slam-research-fastlio:noetic', 'slam-research-fastlio:baseline-pre-scpgo-20261105']:
        assert call('docker', 'image', 'inspect', tag, '--format', '{{.Id}}') == image['base_image_id']
    assert call('docker', 'image', 'inspect', image['new_image_tag'], '--format', '{{.Id}}') == built['image_id']
    assert digest(workspace / 'devel/lib/fast_lio/fastlio_mapping') == built['original_frontend_sha256']
    assert digest(workspace / 'devel_scpgo/lib/aloam_velodyne/alaserPGO') == built['backend_sha256']
    # No interference with any currently running container sharing this workspace.
    for cid in call('docker', 'ps', '-q').splitlines():
        info = json.loads(call('docker', 'inspect', cid))[0]
        assert not any(m.get('Source') == str(workspace) for m in info['Mounts']), info['Name']
    name = args.id + '-container'
    assert name not in call('docker', 'ps', '-a', '--format', '{{.Names}}').splitlines()
    bag = base / 'datasets' / (args.sequence + '.bag')
    assert bag.is_file() and bag.stat().st_size > 0
    run = base / 'results/scpgo' / args.id
    assert not run.exists() and not run.is_symlink()
    # Atomic reservation. Failure retains this directory and all available evidence.
    run.parent.mkdir(exist_ok=True)
    run.mkdir()
    for folder in ['config', 'checks', 'raw', 'derived', 'backend', 'Log', 'PCD', 'ros_logs']:
        (run / folder).mkdir()
    for path in Path(__file__).parent.iterdir():
        if path.is_file():
            shutil.copyfile(path, run / path.name)
    baseline = deploy / 'baseline' / args.sequence.lower()
    root = ET.parse(baseline / 'full.launch').getroot()
    values = {p.attrib['name']: p for p in root.findall('param')}
    assert values['preprocess/blind'].attrib['value'] == '1.0'
    if args.frontend_policy == 'no-online-pcd':
        values['pcd_save/pcd_save_en'].set('value', 'false')
    # Explicit publishing requirements; source and estimator parameters stay intact.
    for key in ['publish/scan_publish_en', 'publish/scan_bodyframe_pub_en']:
        ET.SubElement(root, 'param', name=key, type='bool', value='true')
    ET.ElementTree(root).write(run / 'config/frontend.launch', encoding='unicode')
    shutil.copyfile(baseline / 'full.launch', run / 'config/reference-frontend.launch')
    shutil.copyfile(baseline / 'runtime.yaml', run / 'config/reference-runtime.yaml')
    shutil.copyfile(Path(__file__).parent / 'backend-defaults.yaml', run / 'config/backend.yaml')
    shutil.copyfile(Path(__file__).parent / 'backend.launch', run / 'config/backend.launch')
    inside = '/run/scpgo'
    pkg = '/ws/src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/fast_lio2'
    command = ['docker', 'create', '--name', name, '--init', '--network', 'none',
               '--read-only', '--tmpfs', '/tmp:rw,nosuid,size=256m',
               '--tmpfs', '/root/.ros:rw,nosuid,size=64m',
               '--mount', f'type=bind,source={workspace},target=/ws,readonly',
               '--mount', f'type=bind,source={base / "datasets"},target=/datasets,readonly',
               '--mount', f'type=bind,source={run},target={inside}',
               '--mount', f'type=bind,source={run / "Log"},target={pkg}/Log',
               '--mount', f'type=bind,source={run / "PCD"},target={pkg}/PCD',
               '-e', 'ROS_MASTER_URI=http://127.0.0.1:11311', '-e', 'ROS_IP=127.0.0.1',
               '-e', f'ROS_LOG_DIR={inside}/ros_logs', built['image_id'], 'sleep', 'infinity']
    meta = dict(id=args.id, container=name, sequence=args.sequence,
                frontend_policy=args.frontend_policy, backend_profile=args.backend_profile,
                image_id=built['image_id'], base_image_id=image['base_image_id'],
                source_commit=call('git', '-C', str(repo), 'rev-parse', 'HEAD'),
                input_host_path=str(bag), input_container_path='/datasets/' + bag.name,
                input_bytes=bag.stat().st_size, input_mtime_ns=bag.stat().st_mtime_ns,
                input_sha256=digest(bag), online_pcd=values['pcd_save/pcd_save_en'].attrib['value'] == 'true',
                run_host_path=str(run), run_container_path=inside,
                algorithm_started=False, playback_started=False,
                visualization='No RViz process or GPU mount; published topics can be recorded.')
    write_json(run / 'config/manifest.json', meta)
    # Protect all algorithm files (including untracked imported sample files), setups,
    # and both executables. Original build/devel checked separately against step-4 list.
    paths = [p for p in source.rglob('*') if p.is_file()]
    paths += [workspace / p for p in ['devel/lib/fast_lio/fastlio_mapping',
              'devel_scpgo/lib/aloam_velodyne/alaserPGO', 'devel/setup.bash', 'devel_scpgo/setup.bash']]
    protected = {str(p): digest(p) for p in sorted(set(paths))}
    write_json(run / 'config/protected-files.json', protected)
    # Validate launch expansion in a temporary read-only container. No ROS master/nodes.
    probe = ['docker', 'run', '--rm', '--network', 'none', '--read-only',
             '--tmpfs', '/tmp:rw,nosuid,size=256m', '--tmpfs', '/root/.ros:rw,nosuid,size=64m',
             '--mount', f'type=bind,source={workspace},target=/ws,readonly',
             '--mount', f'type=bind,source={run},target={inside},readonly',
             '-e', 'ROS_LOG_DIR=/tmp/ros_logs', built['image_id']]
    for label, launch in [('frontend', 'frontend.launch'), ('backend', 'backend.launch')]:
        launch_args = [f'{inside}/config/{launch}']
        if label == 'backend':
            launch_args.append('run_dir:=' + inside)
        dump = call(*probe, 'roslaunch', '--dump-params', *launch_args)
        (run / f'config/{label}-expanded.yaml').write_text(dump + '\n')
        nodes = call(*probe, 'roslaunch', '--nodes', *launch_args)
        (run / f'checks/{label}-nodes.txt').write_text(nodes + '\n')
    # ROS Python validates real namespace/remap resolution and exact effective keys.
    (run / 'checks/launch-validation.json').write_text(call(*probe, 'python3',
             inside + '/validate_launch.py', inside) + '\n')
    write_json(run / 'config/create-container-command.json', command)
    cid = call(*command)
    info = json.loads(call('docker', 'inspect', name))[0]
    assert info['State']['Status'] == 'created' and info['Image'] == built['image_id']
    assert info['HostConfig']['ReadonlyRootfs'] and info['HostConfig']['NetworkMode'] == 'none'
    mounts = {m['Destination']: m for m in info['Mounts']}
    assert mounts['/ws']['RW'] is False and mounts['/datasets']['RW'] is False
    assert mounts[inside]['RW'] is True
    assert mounts[pkg + '/Log']['Source'] == str(run / 'Log')
    assert mounts[pkg + '/PCD']['Source'] == str(run / 'PCD')
    write_json(run / 'config/container-created.json', info)
    frozen = {str(p.relative_to(run)): digest(p) for p in run.rglob('*')
              if p.is_file() and (p.parent == run or p.parent == run / 'config')}
    write_json(run / 'checks/frozen-files.json', frozen)
    write_json(run / 'checks/preparation.json', dict(status='PREPARED_NOT_RUN',
               container_id=cid, container_state=info['State']['Status'],
               launch_expansion_passed=True, image_pinned=True,
               workspace_and_input_readonly=True, algorithm_started=False, playback_started=False))
    print('Prepared:', run)
    print('Container created, stopped. No ROS nodes or replay started.')


if __name__ == '__main__':
    main()
