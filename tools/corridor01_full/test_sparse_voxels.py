import pathlib,tempfile,subprocess,struct,json
import numpy as np
with tempfile.TemporaryDirectory() as directory:
    r=pathlib.Path(directory);(r/'derived').mkdir();(r/'checks').mkdir()
    points=np.array([[0,0,0,1],[.01,.01,.01,2],[1000000,2000000,-3000000,3]],dtype='<f4')
    h=b'FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nPOINTS 3\nDATA binary\n'
    (r/'derived/final_registered.pcd').write_bytes(h+points.tobytes())
    subprocess.run(['python3','/tools/downsample_sparse_voxels.py',str(r)],check=True)
    content=(r/'derived/final_registered_voxel_0p05m_first_sparse.pcd').read_bytes().split(b'DATA binary\n')[1]
    assert np.array_equal(np.frombuffer(content,dtype='<f4').reshape(-1,4),points[[0,2]])
    assert subprocess.run(['python3','/tools/downsample_sparse_voxels.py',str(r)],stdout=subprocess.PIPE,stderr=subprocess.PIPE).returncode!=0
    print('PASS: huge coordinate extent, original first-point selection, exact XYZI and overwrite refusal.')
