#!/usr/bin/env python3
"""Host-side guard; never remove a single-use marker after failure."""
from pathlib import Path
import json
import subprocess
import sys
from prepare import digest

run = Path(sys.argv[1]).resolve()
action = sys.argv[2]
assert action in ['start', 'master', 'frontend', 'backend', 'record', 'watch', 'ready', 'status']
meta = json.loads((run / 'config/manifest.json').read_text())
assert str(run) == meta['run_host_path']
for relative, expected in json.loads((run / 'checks/frozen-files.json').read_text()).items():
    assert digest(run / relative) == expected, relative
info = json.loads(subprocess.check_output(['docker', 'inspect', meta['container']], text=True))[0]
assert info['Image'] == meta['image_id']
assert info['HostConfig']['ReadonlyRootfs'] and info['HostConfig']['NetworkMode'] == 'none'
mounts = {m['Destination']: m for m in info['Mounts']}
assert mounts['/ws']['RW'] is False and mounts['/datasets']['RW'] is False
assert mounts['/run/scpgo']['Source'] == str(run) and mounts['/run/scpgo']['RW']
if action == 'status':
    sys.exit(0)
for path, expected in json.loads((run / 'config/protected-files.json').read_text()).items():
    assert digest(path) == expected, path
bag = Path(meta['input_host_path'])
assert bag.stat().st_size == meta['input_bytes'] and bag.stat().st_mtime_ns == meta['input_mtime_ns']
if action == 'start':
    assert info['State']['Status'] == 'created', info['State']
    assert digest(bag) == meta['input_sha256']
    workspace = mounts['/ws']['Source']
    ids = subprocess.check_output(['docker', 'ps', '-q'], text=True).splitlines()
    for cid in ids:
        other = json.loads(subprocess.check_output(['docker', 'inspect', cid], text=True))[0]
        assert not any(m.get('Source') == workspace for m in other['Mounts']), other['Name']
else:
    assert info['State']['Running'], 'Start the prepared container first'
if action == 'backend':
    target = run / 'backend'
    assert target.resolve() == run / 'backend' and not target.is_symlink()
    assert not list(target.iterdir()), 'Backend directory occupied; do not restart or clear it'
if action not in ['ready', 'status']:
    (run / 'checks' / (action + '-once')).mkdir()
