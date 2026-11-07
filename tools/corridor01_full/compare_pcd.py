"""Compare native and bag-exported binary PCD XYZ/intensity without alignment."""
from pathlib import Path
import json,sys
import numpy as np
r=Path(sys.argv[1]);report=r/'checks/native-vs-exported-pcd.json'
if report.exists():raise FileExistsError(report)
def load(path):
    header={}
    with path.open('rb') as f:
        while True:
            line=f.readline()
            if not line:raise ValueError('Missing DATA header')
            cells=line.decode('ascii').split()
            if not cells or cells[0].startswith('#'):continue
            header[cells[0]]=cells[1:]
            if cells[0]=='DATA':break
        offset=f.tell()
    if header['DATA']!=['binary']:raise ValueError('Expected uncompressed binary PCD')
    counts=list(map(int,header.get('COUNT',['1']*len(header['FIELDS']))))
    types={('F',4):'<f4',('F',8):'<f8',('U',1):'u1',('U',2):'<u2',('U',4):'<u4',('I',1):'i1',('I',2):'<i2',('I',4):'<i4'}
    fields=[]
    for i,(name,kind,size,count) in enumerate(zip(header['FIELDS'],header['TYPE'],map(int,header['SIZE']),counts)):
        if name=='_':name='padding_%d'%i
        fields.append((name,types[(kind,size)],(count,)) if count!=1 else (name,types[(kind,size)]))
    dtype=np.dtype(fields);n=int(header['POINTS'][0])
    if path.stat().st_size-offset!=n*dtype.itemsize:raise ValueError('Payload size mismatch '+str(path))
    for name in ['x','y','z','intensity']:
        if name not in dtype.names:raise ValueError('Missing field '+name)
    return np.memmap(str(path),mode='r',dtype=dtype,offset=offset,shape=(n,)),header
a,ha=load(r/'PCD/scans.pcd');b,hb=load(r/'derived/final_registered.pcd')
result=dict(native_points=len(a),exported_points=len(b),native_fields=ha['FIELDS'],exported_fields=hb['FIELDS'],fields={})
if len(a)==len(b):
    for name in ['x','y','z','intensity']:
        equal=True;maximum=0.
        for start in range(0,len(a),500000):
            x=a[name][start:start+500000];y=b[name][start:start+500000]
            if not np.isfinite(x).all() or not np.isfinite(y).all():raise ValueError('Nonfinite '+name)
            equal=equal and np.array_equal(x,y)
            maximum=max(maximum,float(np.max(np.abs(x.astype('f8')-y))))
        result['fields'][name]=dict(exactly_equal_in_order=bool(equal),max_abs_difference=maximum)
result['status']='MATCH_XYZI' if len(a)==len(b) and all(v['exactly_equal_in_order'] for v in result['fields'].values()) else 'DIFFERENT_REVIEW_REQUIRED'
result['meaning']='Direct point-order comparison only; equal files do not prove accurate map. No ICP, filtering, coordinate or intensity changes.'
with report.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
