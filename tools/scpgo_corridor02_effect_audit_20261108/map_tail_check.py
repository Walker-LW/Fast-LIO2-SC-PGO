"""Inspect published map semantics during tail; ignore no bytes without checking fields."""
from pathlib import Path
import hashlib,json
import numpy as np
import rosbag

run=Path('/run/scpgo');out=Path('/out/verified')
audit=json.loads((out/'audit.json').read_text())
cutoff=audit['late_stop']['last_frontend_message_receipt_ns']
rows=[];reference=None;layout=None
for file in sorted((run/'raw').glob('*.bag')):
    with rosbag.Bag(str(file)) as bag:
        for _,m,t in bag.read_messages(topics=['/aft_pgo_map']):
            if t.to_nsec()<cutoff:continue
            fields={f.name:f for f in m.fields}
            assert set(fields)=={'x','y','z','intensity'}
            columns=[]
            for name in ['x','y','z','intensity']:
                f=fields[name];assert f.datatype==7 and f.count==1
                columns.append(np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',buffer=m.data,offset=f.offset,strides=(m.row_step,m.point_step)).reshape(-1))
            a=np.column_stack(columns);assert np.isfinite(a).all()
            if reference is None:reference=a.copy();layout=dict(point_step=m.point_step,fields={k:v.offset for k,v in fields.items()})
            assert a.shape==reference.shape
            rows.append(dict(receipt_ns=t.to_nsec(),points=len(a),payload_sha256=hashlib.sha256(m.data).hexdigest(),
                xyzi_sha256=hashlib.sha256(a.tobytes()).hexdigest(),max_position_difference_m=float(np.linalg.norm(a[:,:3].astype(float)-reference[:,:3],axis=1).max()),
                max_intensity_difference=float(np.abs(a[:,3].astype(float)-reference[:,3]).max())))
report=dict(messages=len(rows),layout=layout,point_counts=sorted(set(x['points'] for x in rows)),
    distinct_payloads=len(set(x['payload_sha256'] for x in rows)),distinct_declared_xyzi_payloads=len(set(x['xyzi_sha256'] for x in rows)),
    max_position_change_m=max(x['max_position_difference_m'] for x in rows),max_intensity_change=max(x['max_intensity_difference'] for x in rows),
    note='All declared PointCloud2 fields examined. If declared XYZI bytes match but payload differs, differing bytes lie in undeclared padding. Published map is sparse and excludes the final keyframe; not a full saved-map history.',rows=rows)
with (out/'map-tail-fields.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
