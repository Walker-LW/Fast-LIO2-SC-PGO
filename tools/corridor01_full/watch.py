"""Observe input delivery and output progress using wall-clock waits.

This subscriber is evidence of its own reception, not of algorithm reception.
"""
import json
import threading
import time

from livox_ros_driver.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import rosbag
import rospy

lock = threading.Lock()
state = {}


def callback(msg, topic):
    now = time.monotonic()
    ns = msg.header.stamp.to_nsec()
    with lock:
        item = state.setdefault(topic, {"count": 0, "first_header_ns": ns})
        item.update(count=item["count"]+1, last_header_ns=ns, last_wall=now)


def snapshot(expected):
    now = time.monotonic()
    with lock:
        data = {topic: dict({key: value for key, value in item.items() if key != "last_wall"},
                            seconds_since_last_message=round(now-item["last_wall"], 3))
                for topic, item in state.items()}
    print(json.dumps({"wall_unix": time.time(), "expected_input_counts": expected,
                      "observed": data}, sort_keys=True), flush=True)


def main():
    with rosbag.Bag("/datasets/Corridor01.bag") as bag:
        expected = {topic: bag.get_message_count(topic_filters=[topic])
                    for topic in ("/livox/avia/lidar", "/livox/avia/imu")}
    rospy.init_node("coverage_observer")
    subscribers = [rospy.Subscriber(topic, cls, callback, callback_args=topic,
                                   queue_size=20000, buff_size=8*1024*1024)
                   for topic, cls in (("/livox/avia/lidar", CustomMsg),
                                      ("/livox/avia/imu", Imu), ("/Odometry", Odometry))]
    print("Observer ready; reception counts are not proof of algorithm reception.", flush=True)
    try:
        while not rospy.is_shutdown():
            snapshot(expected)
            time.sleep(1)
    finally:
        snapshot(expected)


if __name__ == "__main__":
    main()
