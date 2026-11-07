"""Read-only audit of completed geometry A/B runs; writes only to /out.

Point movements are changes, not accuracy errors. No ICP, alignment, or tuning.
"""
from pathlib import Path
from collections import Counter, deque
import csv
import hashlib
import json
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path('/pair')
OUT = Path('/out/verified')
sys.path.insert(0, str(ROOT / 'old'))
from export_keyframe_maps import read_pcd


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def stats(values):
    values = np.asarray(values)
    assert len(values) and np.isfinite(values).all()
    return {name: float(value) for name, value in zip(
        ['min', 'median', 'p95', 'p99', 'max'],
        np.quantile(values, [0, .5, .95, .99, 1]))}


def rotations(poses):
    # Saved KITTI rotations are rounded. Project them to SO(3) for angle summaries.
    u, _, vt = np.linalg.svd(poses[:, :, :3])
    sign = np.linalg.det(u @ vt)
    correction = np.tile(np.eye(3), (len(poses), 1, 1))
    correction[:, 2, 2] = sign
    return u @ correction @ vt


def angles(a, b):
    relative = np.swapaxes(a, 1, 2) @ b
    return np.degrees(np.arccos(np.clip((np.trace(relative, axis1=1, axis2=2) - 1) / 2, -1, 1)))


def parse_log(path):
    log = path.read_text(errors='replace')
    pattern = re.compile(r'Loop detected! - between (\d+) and (\d+)|'
                         r'\[SC loop\] ICP fitness test (passed|failed) \(([-+\d.eE]+)')
    queue = deque()
    detections, results, unassigned = [], [], []
    max_pending = 0
    for event in pattern.finditer(log):
        if event.group(1) is not None:
            pair = (int(event.group(1)), int(event.group(2)))
            detections.append(pair)
            queue.append(pair)
            max_pending = max(max_pending, len(queue))
        else:
            item = dict(status=event.group(3), fitness_m2=float(event.group(4)),
                        offset=event.start())
            if queue:
                item['pair'] = queue.popleft()
                results.append(item)
            else:
                unassigned.append(item)
    assert len(detections) == log.count('Loop detected! - between')
    assert len(results) + len(unassigned) == log.count('ICP fitness test passed') + log.count('ICP fitness test failed')
    pairs = Counter(detections)
    status_pairs = {status: sorted(set(tuple(x['pair']) for x in results if x['status'] == status))
                    for status in ['passed', 'failed']}
    warning_lines = [line for line in log.splitlines() if 'Integer indices would overflow' in line]
    # FIFO association is derived from source queue order, not an explicit pair ID
    # in ICP result messages. Retain that limitation in every derived report.
    return dict(detections=len(detections), unique_detected_pairs=len(pairs),
                results=len(results), passed=sum(x['status'] == 'passed' for x in results),
                failed=sum(x['status'] == 'failed' for x in results),
                unique_passed_pairs=len(status_pairs['passed']), unique_failed_pairs=len(status_pairs['failed']),
                pending_pairs=list(queue), unassigned_results=unassigned, max_observed_pending=max_pending,
                most_repeated_pairs=[{'pair': pair, 'count': count} for pair, count in pairs.most_common(12)],
                terminal_pair_results=[x for x in results if x['pair'] == (0, 382)],
                overflow_warnings=log.count('Integer indices would overflow'),
                overflow_log_lines=len(warning_lines),
                abnormal_termination='terminate called without an active exception' in log,
                backlog_warning_count=log.count('Too many loop clousre candidates'),
                isam_update_prints=log.count('running isam2 optimization'),
                detected_pairs=sorted(pairs), passed_pairs=status_pairs['passed'], events=results,
                mapping_limit='ICP pair labels inferred by FIFO source queue and complete log counts; not explicit optimizer factor records.')


OUT.mkdir(parents=True, exist_ok=True)
assert not (OUT / 'audit.json').exists(), 'Refuse to overwrite audit'
report = {'experiment': '/home/lw/slam-research/results/scpgo/scpgo-corridor01-geometry-ab-20261107-001',
          'container_input_mount': str(ROOT), 'meaning': 'Movement and implementation behavior, not GT accuracy.',
          'integrity': {}, 'runs': {}, 'limits': [
              'One run per version on one previously inspected sequence; no independent-scenario claim.',
              'Asynchronous candidate schedules differ; comparisons of total counts are not paired efficacy estimates.',
              'No ground truth or independent wall/loop correspondences; no accuracy improvement percentage.',
              'Local display regions selected from the overview for inspection, not used to tune parameters.',
              'Rounded saved pose files limit precision; rotations projected to SO(3) for angle summaries.']}
scans = [sorted((ROOT / v / 'backend/Scans').glob('*.pcd')) for v in ['old', 'fixed']]
assert len(scans[0]) == len(scans[1]) == 383
assert all(a.name == b.name and sha(a) == sha(b) for a, b in zip(*scans))
for name in ['times.txt', 'odom_poses.txt']:
    assert sha(ROOT / 'old/backend' / name) == sha(ROOT / 'fixed/backend' / name)
report['integrity'].update(same_383_scans=True, same_times_and_odom=True)
pose_data, logs, maps = {}, {}, {}
near_masks = []
for p in scans[0]:
    cloud = read_pcd(p)
    near_masks.append(np.linalg.norm(cloud[:, :3].astype(float), axis=1) <= 40)
near = np.concatenate(near_masks)
for version in ['old', 'fixed']:
    run = ROOT / version
    end = json.loads((run / 'checks/end-before-stop.json').read_text())
    assert end['observer']['exact_stamp_pairs'] == 4034
    assert end['observer']['counts']['odom'] == end['observer']['counts']['body'] == 4034
    assert not end['observer']['errors']
    assert end['observer']['odom_without_body'] == end['observer']['body_without_odom'] == 0
    assert all(sha(run / rel) == digest for rel, digest in end['backend_sha256'].items())
    assert not list((run / 'raw').glob('*.active'))
    prepared = json.loads((run / 'checks/prepared.json').read_text())
    assert sha(run / 'config/backend.yaml') == prepared['config_sha256']
    assert sha(run / 'config/backend.launch') == prepared['launch_sha256']
    assert prepared['binary_sha256'] in (run / 'checks/resolved-binary.txt').read_text()
    before, after = [np.loadtxt(run / 'backend' / name).reshape(-1, 3, 4)
                     for name in ['odom_poses.txt', 'optimized_poses.txt']]
    times = np.loadtxt(run / 'backend/times.txt')
    assert len(before) == len(after) == len(times) == 383 and np.all(np.diff(times) > 0)
    pose_data[version] = (before, after, times)
    movement = np.linalg.norm(after[:, :, 3] - before[:, :, 3], axis=1)
    angle = angles(rotations(before), rotations(after))
    bm, am = [read_pcd(run / 'derived/keyframe-maps' / name)
              for name in ['before_pgo.pcd', 'after_pgo.pcd']]
    assert len(bm) == len(am) == len(near) == 2102827
    assert np.array_equal(bm[:, 3], am[:, 3])
    if version == 'old':
        original_map = bm
    else:
        assert np.array_equal(original_map, bm)
    maps[version] = am
    point_movement = np.linalg.norm(am[:, :3].astype(float) - bm[:, :3], axis=1)
    logs[version] = parse_log(run / 'checks/backend-console.log')
    report['runs'][version] = dict(
        config_sha256=prepared['config_sha256'], binary_sha256=prepared['binary_sha256'],
        input_pairs=4034, keyframes=383, map_points=len(bm),
        pose_translation_change_m=stats(movement), pose_rotation_change_deg=stats(angle),
        max_translation_frame=int(np.argmax(movement)), max_rotation_frame=int(np.argmax(angle)),
        final_keyframe_translation_change_m=float(movement[-1]),
        start_end_position_gap_before_m=float(np.linalg.norm(before[-1, :, 3] - before[0, :, 3])),
        start_end_position_gap_after_m=float(np.linalg.norm(after[-1, :, 3] - after[0, :, 3])),
        map_point_change_m=stats(point_movement),
        map_point_change_local_range_le40m=stats(point_movement[near]),
        log=logs[version])
    with (OUT / (version + '-per-keyframe.csv')).open('x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['frame', 'seconds_from_first_keyframe', 'translation_change_m', 'rotation_change_deg'])
        writer.writerows(zip(range(len(times)), times - times[0], movement, angle))
assert report['runs']['old']['config_sha256'] == report['runs']['fixed']['config_sha256']
report['integrity']['same_configuration'] = True
old, fixed = pose_data['old'][1], pose_data['fixed'][1]
report['old_vs_fixed_pose_translation_m'] = stats(np.linalg.norm(old[:, :, 3] - fixed[:, :, 3], axis=1))
report['old_vs_fixed_pose_rotation_deg'] = stats(angles(rotations(old), rotations(fixed)))
for v in logs:
    assert not logs[v]['unassigned_results']
common = sorted(set(map(tuple, logs['old']['detected_pairs'])) & set(map(tuple, logs['fixed']['detected_pairs'])))
grouped = {v: {} for v in logs}
for v in logs:
    for event in logs[v]['events']:
        grouped[v].setdefault(tuple(event['pair']), []).append(event)
contrasts = []
for pair in common:
    entries = [grouped[v].get(pair, []) for v in ['old', 'fixed']]
    if not all(entries):
        continue
    statuses = [sorted(set(e['status'] for e in events)) for events in entries]
    contrasts.append(dict(pair=pair, old_statuses=statuses[0], fixed_statuses=statuses[1],
                          old_attempts=len(entries[0]), fixed_attempts=len(entries[1]),
                          old_score_median=float(np.median([e['fitness_m2'] for e in entries[0]])),
                          fixed_score_median=float(np.median([e['fitness_m2'] for e in entries[1]]))))
report['common_pair_contrasts'] = contrasts
report['common_pair_status_counts'] = dict(Counter('/'.join(x['old_statuses']) + ' -> ' + '/'.join(x['fixed_statuses']) for x in contrasts))
report['near_point_count'] = int(near.sum())
report['paired_point_processing'] = 'Same exported keyframe-point order, no matching or alignment; intensity/order checked. Near mask uses raw saved local XYZ norm <=40m.'

fig, axs = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
for version in ['old', 'fixed']:
    before, after, times = pose_data[version]
    axs[0].plot(times - times[0], np.linalg.norm(after[:, :, 3] - before[:, :, 3], axis=1) * 100, label=version)
    axs[1].plot(times - times[0], angles(rotations(before), rotations(after)), label=version)
axs[0].set_ylabel('Position change / cm')
axs[1].set_ylabel('Orientation change / deg')
axs[1].set_xlabel('Seconds from first saved keyframe')
for ax in axs:
    ax.grid(alpha=.25)
    ax.legend()
fig.suptitle('Change from saved frontend poses; these are not ground-truth errors')
fig.savefig(OUT / 'pose-changes.png', dpi=180)
plt.close(fig)

regions = [(-35, 2, 8, 25), (-12, 6, -5, 19), (-65, -44, 23, 38), (48, 67, -28, -16)]
fig, axs = plt.subplots(len(regions), 3, figsize=(16, 15), constrained_layout=True)
display_meta = []
for row, (xmin, xmax, ymin, ymax) in enumerate(regions):
    row_meta = dict(xlim=[xmin, xmax], ylim=[ymin, ymax], panels=[])
    for col, (cloud, label) in enumerate([(original_map, 'Frontend'), (maps['old'], 'Old PGO'), (maps['fixed'], 'Fixed PGO')]):
        mask = (cloud[:, 0] >= xmin) & (cloud[:, 0] <= xmax) & (cloud[:, 1] >= ymin) & (cloud[:, 1] <= ymax)
        points = cloud[mask]
        # Draw every point inside each region, with fixed size/alpha/axes per row.
        axs[row, col].scatter(points[:, 0], points[:, 1], s=.08, alpha=.5, c='#22689b', rasterized=True)
        axs[row, col].set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel='X / m', ylabel='Y / m', title=label)
        axs[row, col].set_aspect('equal')
        row_meta['panels'].append(dict(label=label, points=len(points), stride=1))
    display_meta.append(row_meta)
fig.suptitle('Identical XY display windows; all points within each window; no extra alignment')
fig.savefig(OUT / 'local-comparisons.png', dpi=180)
plt.close(fig)
report['local_display'] = display_meta
fig, axs = plt.subplots(1, 3, figsize=(15, 4), constrained_layout=True)
xmin, xmax, ymin, ymax = regions[0]
for ax, (cloud, label) in zip(axs, [(original_map, 'Frontend'), (maps['old'], 'Old PGO'), (maps['fixed'], 'Fixed PGO')]):
    mask = (cloud[:, 0] >= xmin) & (cloud[:, 0] <= xmax) & (cloud[:, 1] >= ymin) & (cloud[:, 1] <= ymax)
    points = cloud[mask]
    ax.scatter(points[:, 0], points[:, 1], s=.08, alpha=.5, c='#22689b', rasterized=True)
    ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel='X / m', ylabel='Y / m', title=label)
    ax.set_aspect('equal')
fig.suptitle('Local map comparison: same XY range, all points in window, no extra alignment')
fig.savefig(OUT / 'overlap-detail.png', dpi=200)
plt.close(fig)
report['source_comparison_report_sha256'] = sha(ROOT / 'derived/comparison/report.json')
with (OUT / 'audit.json').open('x') as f:
    json.dump(report, f, indent=2)
compact = dict(report)
compact['runs'] = {v: {k: ({q: val for q, val in x.items() if q not in ['events', 'detected_pairs', 'passed_pairs', 'terminal_pair_results']} if k == 'log' else x)
                       for k, x in data.items()} for v, data in report['runs'].items()}
compact.pop('common_pair_contrasts')
print(json.dumps(compact, indent=2))
