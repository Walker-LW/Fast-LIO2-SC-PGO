#!/usr/bin/env python3
"""Read-only checks of protected environment; save results in a new directory."""
import argparse
import json
from pathlib import Path
from prepare import call, digest, write_json

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
args = parser.parse_args()
base = Path('/home/lw/slam-research')
repo = base / 'fast_lio_ws/src/M3DGR'
deploy = repo / 'deployment-scpgo'
out = Path(args.output).resolve()
assert (base / 'results/validation') in out.parents
out.mkdir(parents=True, exist_ok=False)
workspace = base / 'fast_lio_ws'
previous = json.loads((base / 'results/validation/scpgo-build-20261105-001/baseline-build-artifacts.json').read_text())
actual = {}
for name in ['build', 'devel']:
    for path in (workspace / name).rglob('*'):
        key = str(path.relative_to(workspace))
        if path.is_symlink():
            actual[key] = {'symlink': str(path.readlink())}
        elif path.is_file():
            actual[key] = {'sha256': digest(path), 'bytes': path.stat().st_size}
assert actual == previous, 'Original build/devel differ from step-4 protection list'
built = json.loads((deploy / 'locks/scpgo-backend-build.json').read_text())
images = {tag: call('docker', 'image', 'inspect', tag, '--format', '{{.Id}}') for tag in [
    'slam-research-fastlio:noetic', 'slam-research-fastlio:baseline-pre-scpgo-20261105',
    'slam-research-fastlio-scpgo:noetic-gtsam403-v1']}
assert images['slam-research-fastlio:noetic'] == images['slam-research-fastlio:baseline-pre-scpgo-20261105'] == 'sha256:fcfce4546a744e9e76b02e79485ac60f40cc54522362cc8ad7654871c98c2f7f'
assert images['slam-research-fastlio-scpgo:noetic-gtsam403-v1'] == built['image_id']
assert digest(workspace / 'devel/lib/fast_lio/fastlio_mapping') == built['original_frontend_sha256']
assert digest(workspace / 'devel_scpgo/lib/aloam_velodyne/alaserPGO') == built['backend_sha256']
source = repo / 'baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO'
assert not call('git', '-C', str(repo), 'diff', 'e696fd5', '--', str(source))
assert not call('git', '-C', str(repo), 'status', '--porcelain', '--', str(source))
write_json(out / 'report.json', dict(status='PASS', original_artifact_entries_unchanged=len(actual),
    images=images, frontend_sha256=built['original_frontend_sha256'], backend_sha256=built['backend_sha256'],
    scpgo_diff_from_import='none', branch=call('git', '-C', str(repo), 'branch', '--show-current'),
    head=call('git', '-C', str(repo), 'rev-parse', 'HEAD'),
    git_status=call('git', '-C', str(repo), 'status', '--short'),
    containers=call('docker', 'ps', '-a', '--format', '{{.Names}} {{.Status}}'),
    algorithm_started=False, playback_started=False))
print(out / 'report.json')
