"""5 cm voxel selection: retain the first original point in each occupied cell."""
from pathlib import Path
import hashlib, json, math
import numpy as np

r=Path('/results/baseline/corridor02-full-b1-20261102-001')
src=r/'derived/final_registered.pcd'
out=r/'derived/final_registered_voxel_0p05m_first.pcd'
report=r/'checks/pcd-voxel-0p05m-first.json'
if out.exists() or report.exists(): raise FileExistsError('Refusing to overwrite output/report')
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''): h.update(block)
    return h.hexdigest()
before=digest(src)
with src.open('rb') as f:
    header=[]
    while True:
        line=f.readline(); assert line
        header.append(line.decode('ascii').strip())
        if line==b'DATA binary\n': break
    offset=f.tell()
assert 'FIELDS x y z intensity' in header and 'SIZE 4 4 4 4' in header
assert 'TYPE F F F F' in header and 'HEIGHT 1' in header
n=int(next(s.split()[1] for s in header if s.startswith('POINTS ')))
assert src.stat().st_size==offset+n*16
data=np.memmap(str(src),mode='r',dtype='<f4',offset=offset,shape=(n,4))
step=.05
low=np.full(3,np.inf); high=np.full(3,-np.inf)
for start in range(0,n,500000):
    block=data[start:start+500000]
    assert np.isfinite(block).all()
    cells=np.floor(block[:,:3].astype(np.float64)/step)
    low=np.minimum(low,cells.min(axis=0)); high=np.maximum(high,cells.max(axis=0))
low=low.astype(np.int64); dims=(high.astype(np.int64)-low+1)
assert math.prod(int(v) for v in dims)<2**63
keys=np.empty(n,dtype=np.int64)
for start in range(0,n,500000):
    cells=np.floor(data[start:start+500000,:3].astype(np.float64)/step).astype(np.int64)-low
    keys[start:start+len(cells)]=cells[:,0]+dims[0]*(cells[:,1]+dims[1]*cells[:,2])
_,indices=np.unique(keys,return_index=True)
indices.sort(); count=len(indices)
assert len(np.unique(keys[indices]))==count
text=('# PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
      'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
      'POINTS %d\nDATA binary\n')%(count,count)
with out.open('xb') as f:
    f.write(text.encode('ascii'))
    for start in range(0,count,500000): f.write(data[indices[start:start+500000]].tobytes())
assert out.stat().st_size==len(text)+count*16
saved=np.memmap(str(out),mode='r',dtype='<f4',offset=len(text),shape=(count,4))
for start in range(0,count,500000):
    assert np.array_equal(saved[start:start+500000],data[indices[start:start+500000]])
assert digest(src)==before
result=dict(status='PASS',source=str(src),output=str(out),input_points=n,output_points=count,
            retained_fraction=count/n,voxel_m=step,grid_origin=[0,0,0],
            selection='first point in file order per floor(xyz/0.05) cell; no averaging or coordinate changes',
            source_sha256=before,output_sha256=digest(out),output_bytes=out.stat().st_size,
            validation='all output XYZ/intensity exactly equal selected input rows; unique occupied voxels; input hash unchanged',
            limitation='Visualization subset; not a minimum Euclidean spacing guarantee, not accuracy improvement')
with report.open('x') as f: json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
