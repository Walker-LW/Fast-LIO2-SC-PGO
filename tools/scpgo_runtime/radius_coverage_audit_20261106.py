"""Descriptive coverage of saved body scans; does not rerun or change SC/ICP."""
import json
from pathlib import Path
import numpy as np
from export_keyframe_maps import read_pcd

root = Path('/results/scpgo/scpgo-corridor01-20261105-001/backend')
rows = []
for p in sorted((root/'Scans').glob('*.pcd')):
    xyz = read_pcd(p)[:, :3].astype(float)
    radius = np.linalg.norm(xyz[:, :2], axis=1)
    item = {'keyframe': int(p.stem), 'points': len(xyz),
            'xy_radius_quantiles_m': np.quantile(radius, [0, .25, .5, .75, .95, .99, 1]).tolist(),
            'coverage': {str(r): float(np.mean(radius <= r)) for r in (4, 10, 20, 40, 80)},
            'occupied_body_grid_cells': {str(v): int(len(np.unique(np.floor(xyz/v).astype(np.int64), axis=0))) for v in (.4, .2, .1)}}
    rows.append(item)
weights = np.array([r['points'] for r in rows])
report = {'scope': 'All 264 saved body keyframes, before backend voxelization; descriptive XY coverage and grid occupancy only.',
          'total_points': int(weights.sum()),
          'all_keyframes_point_weighted_coverage': {str(r): float(np.average([a['coverage'][str(r)] for a in rows], weights=weights)) for r in (4, 10, 20, 40, 80)},
          'keyframe_coverage_min_median_max': {str(r): np.quantile([a['coverage'][str(r)] for a in rows], [0, .5, 1]).tolist() for r in (20, 40, 80)},
          'selected_keyframes': [a for a in rows if a['keyframe'] in (0, 1, 7, 59, 101, 210, 256, 257, 258, 259, 260, 261, 262, 263)],
          'limitations': ['Coverage counts do not establish scene geometry validity or optimal radius.',
                          'Unweighted grid occupancy is not reproduced PCL voxel output, SC occupancy or independent constraints.',
                          'Smaller radii may exclude informative distant geometry even when retained-point fraction is high.',
                          'No SC scores, retrieval performance, ICP or map change measured.']}
print(json.dumps(report, indent=2))
