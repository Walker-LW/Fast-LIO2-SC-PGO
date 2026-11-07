#!/usr/bin/env python3
"""Export both maps from exactly the same saved keyframes; no point filtering."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_pcd(path):
    header = {}
    with path.open('rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Incomplete PCD header: ' + str(path))
            words = line.decode('ascii').strip().split()
            if not words or words[0].startswith('#'):
                continue
            header[words[0]] = words[1:]
            if words[0] == 'DATA':
                break
        if header['DATA'] != ['binary']:
            raise ValueError('Expected original backend binary PCD: ' + str(path))
        names = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        if not (len(names) == len(sizes) == len(types) == len(counts)):
            raise ValueError('Invalid PCD field layout')
        dtype = np.dtype([(f'f{i}', '<' + {'F': 'f', 'I': 'i', 'U': 'u'}[typ] + str(size), (count,))
                          for i, (typ, size, count) in enumerate(zip(types, sizes, counts))])
        number = int(header['POINTS'][0])
        if number != int(header['WIDTH'][0]) * int(header['HEIGHT'][0]):
            raise ValueError('PCD point count mismatch')
        payload = stream.read()
    if len(payload) != number * dtype.itemsize:
        raise ValueError('PCD payload length mismatch')
    data = np.frombuffer(payload, dtype=dtype)
    columns = []
    for name in ['x', 'y', 'z', 'intensity']:
        if names.count(name) != 1:
            raise ValueError('Missing or duplicated field: ' + name)
        i = names.index(name)
        if sizes[i] != 4 or types[i] != 'F' or counts[i] != 1:
            raise ValueError('Expected float32 scalar: ' + name)
        columns.append(data[f'f{i}'][:, 0])
    points = np.column_stack(columns)
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite XYZI in ' + str(path))
    return points


def main():
    run = Path(sys.argv[1]).resolve()
    out = Path(sys.argv[2]).resolve()
    if run / 'derived' not in out.parents:
        raise ValueError('Output must be a new directory inside this run/derived')
    backend = run / 'backend'
    scans = sorted((backend / 'Scans').glob('*.pcd'))
    if not scans or [p.name for p in scans] != [f'{i:06d}.pcd' for i in range(len(scans))]:
        raise ValueError('Missing, nonsequential or empty saved keyframes')
    inputs = [backend / name for name in ['odom_poses.txt', 'optimized_poses.txt', 'times.txt']]
    original_hashes = {str(p): sha(p) for p in inputs}
    matrices = [np.loadtxt(p, ndmin=2) for p in inputs[:2]]
    times = np.loadtxt(inputs[2], ndmin=1)
    if any(m.shape != (len(scans), 12) or not np.isfinite(m).all() for m in matrices):
        raise ValueError('Scan and pose counts differ, or invalid pose values; preserve and investigate')
    if times.shape != (len(scans),) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Invalid keyframe timestamps/counts')
    # Validate every source before reserving output, then verify bytes on export.
    scan_hashes = {str(p): sha(p) for p in scans}
    counts = [len(read_pcd(p)) for p in scans]
    count = sum(counts)
    if not count:
        raise ValueError('No saved points')
    out.mkdir(exist_ok=False)
    header = ('# PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
              'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\n'
              'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n') % (count, count)
    outputs = []
    for label, poses in zip(['before_pgo', 'after_pgo'], matrices):
        path = out / (label + '.pcd')
        with path.open('xb') as stream:
            stream.write(header.encode('ascii'))
            for i, scan in enumerate(scans):
                if sha(scan) != scan_hashes[str(scan)]:
                    raise ValueError('Input changed during export: ' + str(scan))
                points = read_pcd(scan)
                if len(points) != counts[i]:
                    raise ValueError('Point count changed during export')
                transform = poses[i].reshape(3, 4)
                result = np.empty_like(points, dtype='<f4')
                result[:, :3] = points[:, :3].astype(np.float64) @ transform[:, :3].T + transform[:, 3]
                result[:, 3] = points[:, 3]
                if not np.isfinite(result).all():
                    raise ValueError('Nonfinite transformed points')
                stream.write(result.tobytes())
        if path.stat().st_size != len(header.encode('ascii')) + count * 16:
            raise ValueError('Export length mismatch')
        outputs.append({'name': path.name, 'points': count, 'sha256': sha(path)})
    if any(sha(p) != original_hashes[str(p)] for p in inputs):
        raise ValueError('Pose/time inputs changed during export; retain outputs as invalid')
    report = dict(status='EXPORT_STRUCTURE_PASS', keyframes=len(scans), points_per_map=count,
                  frame='camera_init', processing='Same complete keyframe set, no filtering/downsampling/alignment; float32 output.',
                  outputs=outputs, pose_and_time_sha256=original_hashes, scan_sha256=scan_hashes,
                  meaning='Structural export validation only; does not establish loop correctness or accuracy.')
    with (out / 'report.json').open('x') as stream:
        json.dump(report, stream, indent=2)
    print('EXPORT_STRUCTURE_PASS:', out, 'keyframes=', len(scans), 'points per map=', count)


if __name__ == '__main__':
    main()
