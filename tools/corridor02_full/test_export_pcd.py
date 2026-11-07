import pathlib
import tempfile
import subprocess
import struct
import rosbag
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs.point_cloud2 import create_cloud

with tempfile.TemporaryDirectory() as directory:
    run = pathlib.Path(directory)
    for name in ('raw', 'derived', 'checks'):
        (run/name).mkdir()
    fields = [PointField(n, i*4, PointField.FLOAT32, 1)
              for i, n in enumerate(('x','y','z','intensity'))]
    with rosbag.Bag(str(run/'raw/cloud_0.bag'), 'w') as bag:
        for i in range(2):
            msg = create_cloud(Header(stamp=rospy.Time(10+i), frame_id='camera_init'),
                               fields, [(float(i), 2., 3., 4.)])
            bag.write('/cloud_registered', msg, msg.header.stamp)
    subprocess.run(['python3','/tools/export_pcd.py',str(run)], check=True)
    content = (run/'derived/final_registered.pcd').read_bytes()
    header, data = content.split(b'DATA binary\n')
    assert b'POINTS 2\n' in header
    assert struct.unpack('<8f',data) == (0.,2.,3.,4.,1.,2.,3.,4.)
    assert subprocess.run(['python3','/tools/export_pcd.py',str(run)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode != 0
    print('PASS: two-frame concatenation, exact XYZ/intensity, binary size, overwrite refusal. No replay.')
