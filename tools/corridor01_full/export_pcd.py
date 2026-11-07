"""Export all archived world-frame scans to one binary XYZ/intensity PCD.
No downsampling, alignment, filtering or pose correction is performed.
"""
import json
from pathlib import Path
import sys
import numpy as np
import rosbag

run = Path(sys.argv[1])
paths = sorted((run/'raw').glob('cloud*.bag'))
out = run/'derived/final_registered.pcd'
report = run/'checks/final-pcd.json'
if not paths or list((run/'raw').glob('*.active')):
    raise RuntimeError('Missing finalized cloud bags or active recording exists')
if out.exists() or report.exists():
    raise FileExistsError('Output already exists; preserve it')

def messages():
    for path in paths:
        with rosbag.Bag(str(path)) as bag:
            for _, msg, _ in bag.read_messages(topics=['/cloud_registered']):
                if msg.header.frame_id != 'camera_init':
                    raise ValueError('Unexpected frame')
                yield msg

count = sum(m.width*m.height for m in messages())
if not count:
    raise ValueError('Empty cloud')
header = ('# PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
          'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\n'
          'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n') % (count, count)
written = 0
with out.open('xb') as stream:
    stream.write(header.encode('ascii'))
    for msg in messages():
        fields = {f.name: f for f in msg.fields}
        block = np.empty((msg.height*msg.width, 4), dtype='<f4')
        for index, name in enumerate(('x', 'y', 'z', 'intensity')):
            field = fields[name]
            if field.datatype != 7 or field.count != 1:
                raise ValueError('Expected scalar float32 field: '+name)
            values = np.ndarray((msg.height, msg.width),
                                dtype='>f4' if msg.is_bigendian else '<f4',
                                buffer=msg.data, offset=field.offset,
                                strides=(msg.row_step, msg.point_step))
            block[:, index] = values.reshape(-1)
        if not np.isfinite(block).all():
            raise ValueError('Nonfinite points: preserve partial output; investigate source')
        stream.write(block.tobytes())
        written += len(block)
if written != count or out.stat().st_size != len(header)+count*16:
    raise RuntimeError('PCD size/count mismatch')
with report.open('x') as stream:
    json.dump(dict(status='PASS', points=count, frame='camera_init',
                   bytes=out.stat().st_size, sources=[p.name for p in paths],
                   fields=['x','y','z','intensity'],
                   processing='Concatenation only; no filtering, voxelization or corrections',
                   meaning='Accumulated registered scans, not an internal ikd-Tree snapshot'), stream, indent=2)
print('PASS:', out, 'points=', count, 'bytes=', out.stat().st_size)
