"""Describe one archived rejected ICP input; no new registration or filtering."""
import json
from pathlib import Path
import numpy as np
from export_keyframe_maps import read_pcd, sha

base = Path('/results/validation/scpgo-effect-diagnosis-20261105-001')
run = Path('/results/scpgo/scpgo-corridor01-20261105-001/backend')
source_path = base / 'event-319-source.pcd'
target_path = base / 'event-319-target.pcd'
source = read_pcd(source_path)[:, :3].astype(float)
target = read_pcd(target_path)[:, :3].astype(float)
probe = json.loads(Path('/workspace/tools/scpgo_runtime/verification/effect-icp-probe.json').read_text())
probe = next(r for r in probe['results'] if r['event']['event'] == 319)
transform = np.array(probe['icp_transform']).reshape(4, 4)
aligned = source @ transform[:3, :3].T + transform[:3, 3]
d2 = np.min(np.sum((aligned[:, None, :] - target[None, :, :]) ** 2, axis=2), axis=1)
poses = np.loadtxt(str(run / 'optimized_poses.txt')).reshape(-1, 3, 4)
# A common rigid transform preserves pairwise source distances; root-relative
# ranges use saved root translation, which can differ slightly from its online value.
ranges = np.linalg.norm(source - poses[7, :, 3], axis=1)
order = np.argsort(d2)[::-1]
report = {
    'scope': 'One terminal rejected event, descriptive decomposition at the previously reproduced ICP transform.',
    'input_sha256': {str(p): sha(p) for p in (source_path, target_path)},
    'source_points': len(source), 'target_points': len(target),
    'archived_pcl_fitness_m2': probe['fitness'],
    'recomputed_nearest_neighbor_mse_m2': float(d2.mean()),
    'source_approx_root_range_quantiles_m': np.quantile(ranges, [0, .25, .5, .75, 1]).tolist(),
    'nearest_neighbor_distance_quantiles_m': np.quantile(np.sqrt(d2), [0, .25, .5, .75, 1]).tolist(),
    'top1_fraction_squared_error': float(d2[order[:1]].sum()/d2.sum()),
    'top3_fraction_squared_error': float(d2[order[:3]].sum()/d2.sum()),
    'top5_fraction_squared_error': float(d2[order[:5]].sum()/d2.sum()),
    'range_groups_at_existing_sc_radius_80m': {
        'within80_count': int(np.sum(ranges <= 80)),
        'beyond80_count': int(np.sum(ranges > 80)),
        'within80_mse_at_unchanged_transform_m2': float(d2[ranges <= 80].mean()),
        'beyond80_mse_at_unchanged_transform_m2': float(d2[ranges > 80].mean())},
    'points': [{'index': int(i), 'approx_root_range_m': float(ranges[i]),
                'nearest_neighbor_distance_m': float(np.sqrt(d2[i]))} for i in order],
    'limitations': ['Single rejected pair, not all terminal pairs.',
                    'Nearest-neighbor residual is not ground-truth error.',
                    'No points removed and no ICP rerun; error concentration does not establish a filter as a repair.',
                    'Approximate ranges use final rounded root pose, not logged online pose.']
}
assert abs(d2.mean()-probe['fitness']) < 0.01, 'Score decomposition must reproduce archived score'
print(json.dumps(report, indent=2))
