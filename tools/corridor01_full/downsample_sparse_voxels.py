"""Sparse integer XYZ voxel keys; same first-point rule, no packed grid limit."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
r=Path(sys.argv[1]);source=r/'derived/final_registered.pcd'
output=r/'derived/final_registered_voxel_0p05m_first_sparse.pcd'
report=r/'checks/pcd-voxel-0p05m-first-sparse.json'
if output.exists() or report.exists():raise FileExistsError('Preserve existing output/report')
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''):h.update(block)
    return h.hexdigest()
before=sha(source)
with source.open('rb') as f:
    header=[]
    while True:
        line=f.readline()
        if not line:raise ValueError('Missing PCD binary header')
        header.append(line.decode('ascii').strip())
        if line==b'DATA binary\n':break
    offset=f.tell()
assert 'FIELDS x y z intensity' in header and 'TYPE F F F F' in header and 'SIZE 4 4 4 4' in header
n=int(next(s.split()[1] for s in header if s.startswith('POINTS ')))
assert source.stat().st_size==offset+16*n
data=np.memmap(str(source),mode='r',dtype='<f4',offset=offset,shape=(n,4))
cells=np.empty((n,3),dtype=np.int64)
for start in range(0,n,500000):
    block=data[start:start+500000];assert np.isfinite(block).all()
    grid=np.floor(block[:,:3].astype(np.float64)/.05)
    assert np.max(np.abs(grid))<2**63-1
    cells[start:start+len(grid)]=grid.astype(np.int64)
keys=cells.view(np.dtype([('x','<i8'),('y','<i8'),('z','<i8')])).reshape(-1)
_,indices=np.unique(keys,return_index=True);indices.sort();count=len(indices)
assert len(np.unique(keys[indices]))==count
text=('# PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
      'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n')%(count,count)
with output.open('xb') as f:
    f.write(text.encode('ascii'))
    for start in range(0,count,500000):f.write(data[indices[start:start+500000]].tobytes())
assert output.stat().st_size==len(text)+16*count
saved=np.memmap(str(output),mode='r',dtype='<f4',offset=len(text),shape=(count,4))
for start in range(0,count,500000):assert np.array_equal(saved[start:start+500000],data[indices[start:start+500000]])
assert sha(source)==before
result=dict(status='PASS',input_points=n,output_points=count,retained_fraction=count/n,output_bytes=output.stat().st_size,
            voxel_m=.05,grid_origin=[0,0,0],method='unique int64 XYZ tuples; retain first source point per voxel, restore source order',
            source_sha256=before,output_sha256=sha(output),source=str(source),output=str(output),
            validation='source hash unchanged; all output XYZ/intensity exactly equal selected source points; unique occupied voxel keys; correct payload size',
            limits='No pose/coordinate corrections, cropping, filtering or alignment. Does not repair diverged map. Large-coordinate float32 source precision is retained, not recovered.')
with report.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
