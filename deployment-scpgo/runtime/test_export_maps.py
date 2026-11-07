"""Synthetic coordinate/field and refusal checks; never uses dataset input."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

tool = Path(__file__).parent / 'export_keyframe_maps.py'
spec = importlib.util.spec_from_file_location('export_maps', tool)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
base = Path(sys.argv[1]) / 'synthetic'
base.mkdir()
run = base / 'fake-run'
(run / 'backend/Scans').mkdir(parents=True)
(run / 'derived').mkdir()
header = ('VERSION 0.7\nFIELDS x y z _ intensity _\nSIZE 4 4 4 4 4 4\n'
          'TYPE F F F F F F\nCOUNT 1 1 1 1 1 3\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n')
scan = run / 'backend/Scans/000000.pcd'
scan.write_bytes(header.encode('ascii') + np.array([1, 2, 3, 0, 4, 0, 0, 0], dtype='<f4').tobytes())
(run / 'backend/odom_poses.txt').write_text('1 0 0 10 0 1 0 0 0 0 1 0\n')
(run / 'backend/optimized_poses.txt').write_text('0 -1 0 20 1 0 0 0 0 0 1 0\n')
(run / 'backend/times.txt').write_text('100.123456789\n')
sources = {p: p.read_bytes() for p in (run / 'backend').rglob('*') if p.is_file()}
out = run / 'derived/comparison'
first = subprocess.run([sys.executable, str(tool), str(run), str(out)], capture_output=True, text=True)
assert first.returncode == 0, first.stdout + first.stderr
assert np.array_equal(module.read_pcd(out / 'before_pgo.pcd'), [[11, 2, 3, 4]])
assert np.array_equal(module.read_pcd(out / 'after_pgo.pcd'), [[18, 1, 3, 4]])
assert all(p.read_bytes() == content for p, content in sources.items())
second = subprocess.run([sys.executable, str(tool), str(run), str(out)], capture_output=True, text=True)
assert second.returncode != 0 and 'FileExistsError' in second.stderr
(run / 'backend/optimized_poses.txt').write_text('1 0 0 0 0 1 0 0 0 0 1 0\n' * 2)
bad = subprocess.run([sys.executable, str(tool), str(run), str(run / 'derived/mismatched')], capture_output=True, text=True)
assert bad.returncode != 0 and not (run / 'derived/mismatched').exists()
report = dict(status='PASS', tests=['row-major KITTI transforms, including rotation and translation',
    'padded PCD XYZI fields and intensity preservation', 'identical scan set for both outputs',
    'source bytes unchanged', 'refuses occupied output', 'refuses scan/pose count mismatch'],
    real_dataset_processed=False, ros_nodes_started=False)
(base / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
