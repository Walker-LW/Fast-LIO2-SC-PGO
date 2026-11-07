#!/usr/bin/env python3
"""External observer: does not alter or synchronize backend input."""
from collections import Counter
import json
from pathlib import Path
import sys
import threading
import time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg
from sensor_msgs.msg import Imu

run = Path(sys.argv[1])
lock = threading.Lock()
counts = Counter()
stamps = {'odom': Counter(), 'body': Counter()}
last = {}
first = {}
errors = Counter()
fields = set()
rospy.init_node('scpgo_interface_watch', anonymous=False)


def receive(msg, label):
    with lock:
        stamp = msg.header.stamp.to_nsec()
        counts[label] += 1
        first.setdefault(label, stamp)
        # PGO intentionally republishes its latest keyframe; repeated stamps there
        # are not an input ordering failure.
        if label != 'pgo' and label in last and stamp <= last[label]['stamp_ns']:
            errors[label + '_nonincreasing_stamp'] += 1
        last[label] = {'stamp_ns': stamp, 'wall_monotonic': time.monotonic(),
                       'clock_minus_header_s': rospy.Time.now().to_sec() - stamp / 1e9}
        if label in stamps:
            stamps[label][stamp] += 1
        if label == 'odom':
            if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
                errors['odom_frame'] += 1
        if label == 'body':
            layout = tuple((f.name, f.offset, f.datatype, f.count) for f in msg.fields)
            fields.add((layout, msg.point_step, msg.is_bigendian))
            if msg.header.frame_id != 'body':
                errors['body_frame'] += 1
            names = {f.name: f for f in msg.fields}
            if not all(k in names and names[k].datatype == 7 and names[k].count == 1
                       for k in ['x', 'y', 'z', 'intensity']):
                errors['body_xyzi_float32_layout'] += 1
            if len(msg.data) != msg.row_step * msg.height or msg.row_step < msg.point_step * msg.width:
                errors['body_data_length'] += 1


subscriptions = [rospy.Subscriber(topic, typ, receive, callback_args=label,
                  queue_size=1000, buff_size=32 * 1024 * 1024)
    for topic, typ, label in [('/Odometry', Odometry, 'odom'),
        ('/cloud_registered_body', PointCloud2, 'body'), ('/livox/avia/lidar', CustomMsg, 'lidar'),
        ('/livox/avia/imu', Imu, 'imu'), ('/aft_pgo_odom', Odometry, 'pgo')]]
output = run / 'checks/interface-observer.jsonl'
print('Observer ready. Counter differences during replay may be transient; inspect final snapshot.')
try:
    with output.open('x', buffering=1) as stream:
        while not rospy.is_shutdown():
            with lock:
                common = stamps['odom'] & stamps['body']
                report = dict(wall_time_ns=time.time_ns(), counts=dict(counts), first_stamp_ns=first,
                    last=last, exact_stamp_pairs=sum(common.values()),
                    odom_without_body=sum((stamps['odom'] - stamps['body']).values()),
                    body_without_odom=sum((stamps['body'] - stamps['odom']).values()),
                    errors=dict(errors), body_layouts=list(fields),
                    limitation='External receipt only; not backend internal queue or ICP/loop-factor evidence.')
                line = json.dumps(report)
            stream.write(line + '\n')
            print(line, flush=True)
            time.sleep(1)
except (KeyboardInterrupt, rospy.ROSInterruptException):
    pass
