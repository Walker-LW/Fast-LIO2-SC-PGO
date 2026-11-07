"""Export same-frame trajectory for CloudCompare; preserve source poses."""
import csv,json,sys
from pathlib import Path
import numpy as np
r=Path(sys.argv[1])
folder=r/'derived/trajectory-view'
folder.mkdir()
rows=[line.split() for line in (r/'derived/trajectory.tum').read_text().splitlines()]
a=np.array(rows,dtype=np.float64)
if len(a)==0 or a.shape[1]!=8 or not np.isfinite(a).all():raise ValueError('Invalid TUM')
audit=json.loads((r/'checks/full-run.json').read_text())
t=a[:,0]-audit['input_bag_start'];xyz=a[:,1:4]
colors=np.empty((len(a),3),dtype=np.uint8)
frac=(t-t.min())/max(float(np.ptp(t)),1e-12)
colors[:,0]=(255*frac).astype(np.uint8);colors[:,1]=70;colors[:,2]=255-colors[:,0]
def ply(name,points,rgb,seconds):
    with (folder/name).open('x') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex %d\nproperty double x\nproperty double y\nproperty double z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty double time_relative_s\nend_header\n'%len(points))
        for p,c,s in zip(points,rgb,seconds):f.write('%.17g %.17g %.17g %d %d %d %.9f\n'%(*p,*c,s))
ply('trajectory_time_rgb.ply',xyz,colors,t)
ply('start_green.ply',xyz[:1],np.array([[0,255,0]]),t[:1])
ply('end_red.ply',xyz[-1:],np.array([[255,0,0]]),t[-1:])
with (folder/'trajectory_with_time.csv').open('x') as f:
    writer=csv.writer(f);writer.writerow(['timestamp','x','y','z','relative_seconds'])
    for row,s in zip(rows,t):writer.writerow(row[:4]+[format(s,'.9f')])
with (folder/'trajectory_time_rgb.ply').open() as f:
    for line in f:
        if line.strip()=='end_header':break
    loaded=np.loadtxt(f,ndmin=2)
assert np.array_equal(loaded[:,:3],xyz)
with (folder/'report.json').open('x') as f:
    json.dump(dict(status='PASS',points=len(a),frame='camera_init',reference='body origin',
                   z_range=[float(xyz[:,2].min()),float(xyz[:,2].max())],
                   estimated_endpoint_distance=float(np.linalg.norm(xyz[-1]-xyz[0])),
                   limits='XYZ points in pose order, not connected polyline. Colors show time. Endpoint distance is not GT error; raw Z is not necessarily gravity-aligned height.'),f,indent=2)
print('PASS:',folder,'points=',len(a),'XYZ roundtrip exact; start green, end red; source unchanged.')
