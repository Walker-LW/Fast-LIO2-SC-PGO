"""Verify every saved keyframe against native recorded body XYZI; no transformation."""
from pathlib import Path
import json,sys
import numpy as np
import rosbag

run=Path('/run/scpgo');sys.path.insert(0,str(run))
from export_keyframe_maps import read_pcd
times=np.loadtxt(run/'backend/times.txt');scans=sorted((run/'backend/Scans').glob('*.pcd'))
assert len(times)==len(scans)==304
matched={};offsets=[];points=0
for p in sorted((run/'raw').glob('*.bag')):
    with rosbag.Bag(str(p)) as bag:
        for _,m,_ in bag.read_messages(topics=['/cloud_registered_body']):
            t=m.header.stamp.to_sec();i=int(np.argmin(np.abs(times-t)))
            # Text/double timestamp round-off only; scanner spacing is orders larger.
            offset=abs(times[i]-t)
            if offset>1e-6:continue
            assert i not in matched
            fields={f.name:f for f in m.fields};columns=[]
            for name in ['x','y','z','intensity']:
                f=fields[name];assert f.datatype==7 and f.count==1
                columns.append(np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',buffer=m.data,offset=f.offset,strides=(m.row_step,m.point_step)).reshape(-1))
            raw=np.column_stack(columns);saved=read_pcd(scans[i])
            assert raw.shape==saved.shape and np.array_equal(raw,saved),i
            matched[i]=m.header.stamp.to_nsec();offsets.append(offset*1e9);points+=len(raw)
assert len(matched)==304
report=dict(matched_keyframes=len(matched),all_saved_xyzi_exactly_equal_native_recorded_body=True,
            source_points=points,max_float_timestamp_matching_offset_ns=max(offsets),
            timestamp_matching_tolerance_s=1e-6,
            meaning='Only the correspondence lookup tolerates rounded timestamps. All matched XYZ/intensity arrays are checked exactly without alignment, filtering or correction.')
with Path('/out/verified/keyframe-source-check.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
