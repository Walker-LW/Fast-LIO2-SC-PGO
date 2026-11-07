"""Measure fixed endpoint/control samples; does not change any parameter."""
from pathlib import Path
import json
import sys
import numpy as np
import rosbag
import rospy
from export_keyframe_maps import read_pcd

bag_path,run,out=map(Path,sys.argv[1:4])
targets=[0.5,5,10,60,380,390,395,400,403]
rows=[]
with rosbag.Bag(str(bag_path)) as bag:
    start=bag.get_start_time()
    for target in targets:
        candidates=list(bag.read_messages(topics=['/livox/avia/lidar'],
            start_time=rospy.Time.from_sec(start+target-0.15),end_time=rospy.Time.from_sec(start+target+0.15)))
        assert candidates
        topic,msg,time=min(candidates,key=lambda row:abs(row[2].to_sec()-start-target))
        xyz=np.array([[p.x,p.y,p.z] for p in msg.points])
        distances=np.linalg.norm(xyz,axis=1)
        valid=np.array([p.line<6 and (p.tag & 0x30) in [0,0x10] for p in msg.points])
        valid[0]=False
        # Matches valid_num counting and one-in-three selection, not later
        # duplicate-coordinate check, undistortion, voxelization or map matching.
        sampled=valid & (np.cumsum(valid)%3==0)
        nonzero=distances>0
        rows.append(dict(target_seconds=target,receive_relative_s=time.to_sec()-start,
            header_relative_s=msg.header.stamp.to_sec()-start,raw_points=len(xyz),
            raw_nonzero=int(nonzero.sum()),raw_nonfinite=int(np.count_nonzero(~np.isfinite(xyz).all(axis=1))),
            raw_distance_quantiles_m=np.quantile(distances[nonzero],[0,.25,.5,.75,1]).tolist(),
            raw_nonzero_within_1m=int(np.count_nonzero(nonzero & (distances<=1))),
            raw_nonzero_within_4m=int(np.count_nonzero(nonzero & (distances<=4))),
            tag_line_one_in_three=int(sampled.sum()),
            distance_candidates_gt_1m=int(np.count_nonzero(sampled & (distances>1))),
            distance_candidates_gt_4m=int(np.count_nonzero(sampled & (distances>4)))))
times=np.loadtxt(run/'backend/times.txt')
keyframes=[]
for index in [0,1,2,7,22,59,101,210,260,261,262,263]:
    cloud=read_pcd(run/'backend/Scans'/('%06d.pcd'%index))
    xyz=cloud[:,:3]
    az=np.mod(np.arctan2(xyz[:,1],xyz[:,0]),2*np.pi)
    sectors=np.minimum(59,(az/(2*np.pi)*60).astype(int))
    voxels=np.floor(xyz.astype('f8')/0.4).astype('i8')
    keyframes.append(dict(index=index,time_relative_s=float(times[index]-start),saved_points=len(cloud),
        occupied_body_6deg_sectors=int(len(np.unique(sectors))),
        occupied_0p4m_body_grid_cells=int(len(np.unique(voxels,axis=0))),
        xyz_min=xyz.min(axis=0).tolist(),xyz_max=xyz.max(axis=0).tolist()))
report=dict(raw_samples=rows,keyframe_samples=keyframes,
    limitation='Raw distance candidates omit later processing. Body grid cells are occupancy, not reproduced PCL output or match inliers. Camera return does not by itself prove LiDAR overlap quality.')
(out/'overlap-input-statistics.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
