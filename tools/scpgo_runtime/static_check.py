#!/usr/bin/env python3
"""Validate candidate config combinations, without selecting an experiment."""
import ast
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from prepare import call, write_json

out = Path(sys.argv[1]).resolve()
assert out.is_dir() and not (out / 'static').exists()
fixtures = out / 'static'
fixtures.mkdir()
local = Path(__file__).resolve().parent
deploy = Path('/home/lw/slam-research/deployment-scpgo')
workspace = Path('/home/lw/slam-research/fast_lio_ws')
image = json.loads((deploy / 'locks/scpgo-image.json').read_text())['new_image_id']
for path in local.glob('*.py'):
    ast.parse(path.read_text(), filename=str(path))
subprocess.run(['bash', '-n', str(local / 'run.sh')], check=True)
reports = []
for sequence in ['corridor01', 'corridor02']:
    for policy in ['baseline', 'no-online-pcd']:
        fixture = fixtures / (sequence + '-' + policy)
        fixture.mkdir()
        (fixture / 'config').mkdir()
        baseline = deploy / 'baseline' / sequence
        root = ET.parse(baseline / 'full.launch').getroot()
        values = {p.attrib['name']: p for p in root.findall('param')}
        if policy == 'no-online-pcd':
            values['pcd_save/pcd_save_en'].set('value', 'false')
        for key in ['publish/scan_publish_en', 'publish/scan_bodyframe_pub_en']:
            ET.SubElement(root, 'param', name=key, type='bool', value='true')
        ET.ElementTree(root).write(fixture / 'config/frontend.launch', encoding='unicode')
        shutil.copyfile(baseline / 'runtime.yaml', fixture / 'config/reference-runtime.yaml')
        shutil.copyfile(local / 'backend-defaults.yaml', fixture / 'config/backend.yaml')
        shutil.copyfile(local / 'backend.launch', fixture / 'config/backend.launch')
        write_json(fixture / 'config/manifest.json', {'online_pcd': values['pcd_save/pcd_save_en'].attrib['value'] == 'true'})
        probe = ['docker', 'run', '--rm', '--network', 'none', '--read-only',
                 '--tmpfs', '/tmp:rw,size=256m', '--tmpfs', '/root/.ros:rw,size=64m',
                 '--mount', f'type=bind,source={workspace},target=/ws,readonly',
                 '--mount', f'type=bind,source={fixture},target=/run/scpgo,readonly',
                 '--mount', f'type=bind,source={local},target=/tools,readonly',
                 '-e', 'ROS_LOG_DIR=/tmp/ros_logs', image]
        for label in ['frontend', 'backend']:
            arguments = [f'/run/scpgo/config/{label}.launch']
            if label == 'backend':
                arguments.append('run_dir:=/run/scpgo')
            dump = call(*probe, 'roslaunch', '--dump-params', *arguments)
            (fixture / f'config/{label}-expanded.yaml').write_text(dump + '\n')
        result = json.loads(call(*probe, 'python3', '/tools/validate_launch.py', '/run/scpgo'))
        write_json(fixture / 'validation.json', result)
        reports.append({'sequence': sequence, 'policy': policy, **result})
write_json(fixtures / 'report.json', {'status': 'PASS', 'checks': reports,
           'note': 'Candidate fixtures only. No experiment selected, ROS nodes started, or dataset replayed.'})
print(fixtures / 'report.json')
