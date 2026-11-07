"""Read existing Corridor02 bags only; do not synthesize or publish input."""
from pathlib import Path
from collections import Counter
import json
import rosbag

root = Path('/baseline/raw')
odom, cloud = {}, {}
inventory = {}
for path in sorted(root.glob('*.bag')):
    with rosbag.Bag(str(path)) as bag:
        info = bag.get_type_and_topic_info()
        inventory[path.name] = {topic: entry.message_count for topic, entry in info.topics.items()}
        for topic, msg, _ in bag.read_messages(topics=['/Odometry', '/cloud_registered', '/cloud_registered_body']):
            store = odom if topic == '/Odometry' else cloud
            stamp = msg.header.stamp.to_nsec()
            assert stamp not in store, (topic, stamp, 'duplicate')
            if topic == '/Odometry':
                store[stamp] = [msg.header.frame_id, msg.child_frame_id]
            else:
                store[stamp] = [msg.header.frame_id, topic]
common = set(odom) & set(cloud)
report = dict(inventory=inventory, odom=len(odom), clouds=len(cloud), exact_stamp_pairs=len(common),
              odom_without_cloud=len(set(odom)-common), cloud_without_odom=len(set(cloud)-common),
              odom_frames=dict(Counter('/'.join(v) for v in odom.values())),
              cloud_frames_topics=dict(Counter('/'.join(v) for v in cloud.values())),
              limit='Metadata and pairing only. No conversion performed or equivalence to unrecorded body points claimed.')
assert len(odom) == len(cloud) == len(common) == 2930
assert set(map(tuple, odom.values())) == {('camera_init', 'body')}
assert set(map(tuple, cloud.values())) == {('camera_init', '/cloud_registered')}
print(json.dumps(report, indent=2))
