"""Small synthetic capture/export/report integration check; never plays real data."""
from pathlib import Path
import json
import sys
import tempfile
import time
import numpy as np
import rosbag
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, '/ws/src/M3DGR/deployment-scpgo/runtime')
import export_keyframe_maps
import analyze
import runtime

with tempfile.TemporaryDirectory(prefix='corridor02-synthetic-') as directory:
    run=Path(directory)
    for name in ['raw','backend/Scans','checks','derived']:(run/name).mkdir(parents=True)
    before=np.tile(np.eye(4)[:3],(2,1,1));before[1,0,3]=1
    after=before.copy();a=np.radians(1);after[1,:3,:3]=[[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]];after[1,0,3]=1.03
    for name,array in [('odom_poses.txt',before),('optimized_poses.txt',after)]:np.savetxt(run/'backend'/name,array.reshape(2,12),fmt='%.12g')
    (run/'backend/times.txt').write_text('1000.000000000\n1001.000000000\n')
    points=np.array([[0,0,0,1],[1,0,0,2],[0,1,0,3]],dtype='<f4')
    header='# PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA binary\n'
    with rosbag.Bag(str(run/'raw/interfaces_0.bag'),'w') as bag:
        for i in range(2):
            (run/'backend/Scans'/('{:06d}.pcd'.format(i))).write_bytes(header.encode()+points.tobytes())
            t=rospy.Time(1000+i,0)
            odom=Odometry();odom.header.stamp=t;odom.header.frame_id='camera_init';odom.child_frame_id='body';odom.pose.pose.position.x=i;odom.pose.pose.orientation.w=1
            cloud=PointCloud2();cloud.header.stamp=t;cloud.header.frame_id='body';cloud.height=1;cloud.width=3;cloud.point_step=16;cloud.row_step=48;cloud.data=points.tobytes();cloud.is_dense=True
            cloud.fields=[PointField(n,j*4,7,1) for j,n in enumerate(['x','y','z','intensity'])]
            bag.write('/Odometry',odom,t);bag.write('/cloud_registered_body',cloud,t)
    (run/'checks/closed.json').write_text(json.dumps({'saved_files_changed_at_stop':[]}))
    observed={'counts':{'lidar':2,'imu':10,'odom':2,'body':2},'exact_stamp_pairs':2,'errors':{},
              'odom_without_body':0,'body_without_odom':0,'first_stamp_ns':{'lidar':999900000000},
              'last':{'lidar':{'stamp_ns':1000900000000},'imu':{'stamp_ns':1001002000000,'wall_monotonic':time.monotonic()},
                      'body':{'stamp_ns':1000800000000}}}
    (run/'config').mkdir()
    (run/'config/input-info.yaml').write_text('topics:\n- topic: /livox/avia/lidar\n  messages: 2\n- topic: /livox/avia/imu\n  messages: 10\n')
    (run/'checks/interface-observer.jsonl').write_text(json.dumps(observed)+'\n')
    runtime.RUN=run
    try:runtime.complete()
    except AssertionError:pass
    else:raise AssertionError('Lagging frontend must not pass input-complete check')
    assert not (run/'checks/input-complete.json').exists()
    observed['last']['body']['stamp_ns']=1001000000000
    (run/'checks/interface-observer.jsonl').write_text(json.dumps(observed)+'\n')
    runtime.complete()
    (run/'checks/frontend-console.log').write_text('No Effective Points\n')
    (run/'checks/backend-console.log').write_text('Loop detected! - between 0 and 1\n[SC loop] ICP fitness test passed (0.02 < 0.3). Add this SC loop.\n')
    sys.argv=['export',str(run),str(run/'derived/keyframe-maps')]
    export_keyframe_maps.main()
    analyze.main(run)
    report=json.loads((run/'derived/quantitative/report.json').read_text())
    assert abs(report['pose_translation_change_m']['max']-.03)<1e-9
    assert abs(report['pose_rotation_change_deg']['max']-1)<1e-7
    assert report['frontend']['exact_recorded_pairs']==2 and report['maps']['points_each']==6
    assert report['backend']['unique_passed_pairs_inferred']==1
    assert len((run/'derived/quantitative/frontend-full.tum').read_text().splitlines())==2
    assert (run/'derived/quantitative/before-after.png').is_file()
    try:analyze.main(run)
    except AssertionError:pass
    else:raise AssertionError('Analysis must refuse overwrite')
print('SYNTHETIC_END_COVERAGE_CAPTURE_EXPORT_REPORT_AND_NO_OVERWRITE_PASS')
