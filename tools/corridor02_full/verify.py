"""Audit every saved pose and cloud without repairing or excluding samples.

Coverage against the input file is evidence of extent, not proof that the
algorithm received every input message. No GT accuracy metric is computed.
"""
import collections
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import rosbag


def main():
    run = Path(sys.argv[1])
    out = run / "checks/full-run.json"
    if out.exists():
        raise FileExistsError(out)
    active = list((run / "raw").glob("*.active"))
    if active:
        raise RuntimeError("Unclosed bags; preserve files: " + str(active))
    issues = []
    report = {"issues": issues, "input": {}, "raw_sha256": {}}
    def issue(kind, **values):
        issues.append(dict(issue=kind, **values))
    # Sequential decoding checks all selected input messages; it does not
    # validate every original XYZ value, calibration, or sensor synchronization.
    with rosbag.Bag("/datasets/Corridor02.bag") as bag:
        report["input_bag_start"] = bag.get_start_time()
        report["input_bag_end"] = bag.get_end_time()
        for topic in ("/livox/avia/lidar", "/livox/avia/imu"):
            count, first, last, max_gap = 0, None, None, 0
            for _, msg, _ in bag.read_messages(topics=[topic]):
                ns = msg.header.stamp.to_nsec()
                if last is not None:
                    max_gap = max(max_gap, ns-last)
                    if ns <= last:
                        issue("input_nonincreasing", topic=topic, row=count+1)
                first = ns if first is None else first
                last, count = ns, count+1
                if topic.endswith("lidar"):
                    tail_offset = max((p.offset_time for p in msg.points), default=0)
            report["input"][topic] = dict(count=count, first_header_ns=first,
                                         last_header_ns=last, max_gap_ns=max_gap)
            if topic.endswith("lidar"):
                report["input"][topic]["last_raw_scan_end_ns"] = last + tail_offset
            if count != bag.get_message_count(topic_filters=[topic]):
                issue("input_count_mismatch", topic=topic)
    rows = (run / "derived/trajectory.tum").read_text().splitlines()
    odom_stamps, cloud_stamps = [], []
    previous_xyz = None
    max_step = 0.0
    max_step_interval = None
    with rosbag.Bag(str(run / "raw/odometry.bag")) as bag:
        report["pose_bag_topics"] = {
            key: value.message_count for key, value in bag.get_type_and_topic_info().topics.items()}
        for index, (_, msg, _) in enumerate(bag.read_messages(topics=["/Odometry"])):
            ns = msg.header.stamp.to_nsec()
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if index >= len(rows):
                issue("tum_row_missing", row=index+1)
            else:
                cells = rows[index].split()
                if len(cells) != 8 or Decimal(cells[0])*1000000000 != ns or [float(v) for v in cells[1:]] != values:
                    issue("tum_raw_mismatch", row=index+1)
            if previous_xyz is not None:
                step = math.sqrt(sum((a-b)**2 for a, b in zip(values[:3], previous_xyz)))
                if step > max_step:
                    max_step, max_step_interval = step, [odom_stamps[-1], ns]
            previous_xyz = values[:3]
            odom_stamps.append(ns)
    if len(rows) != len(odom_stamps) or not odom_stamps:
        issue("pose_count_or_empty", tum=len(rows), raw=len(odom_stamps))
    for topic in ("/tf", "/path", "/clock"):
        if not report["pose_bag_topics"].get(topic):
            issue("missing_pose_bag_topic", topic=topic)
    clouds = sorted((run / "raw").glob("cloud*.bag"))
    if not clouds:
        issue("no_cloud_bags")
    total_points = 0
    for path in clouds:
        with rosbag.Bag(str(path)) as bag:
            for _, msg, _ in bag.read_messages(topics=["/cloud_registered"]):
                cloud_stamps.append(msg.header.stamp.to_nsec())
                total_points += msg.width * msg.height
                if msg.header.frame_id != "camera_init" or len(msg.data) != msg.row_step * msg.height:
                    issue("cloud_frame_or_length", stamp=cloud_stamps[-1])
                fields = {f.name: f for f in msg.fields}
                for axis in ("x", "y", "z"):
                    field = fields.get(axis)
                    if field is None or field.datatype != 7 or field.count != 1:
                        issue("unsupported_xyz_layout", stamp=cloud_stamps[-1], axis=axis)
                        continue
                    points = np.ndarray((msg.height, msg.width), dtype=">f4" if msg.is_bigendian else "<f4",
                                        buffer=msg.data, offset=field.offset,
                                        strides=(msg.row_step, msg.point_step))
                    bad = int((~np.isfinite(points)).sum())
                    if bad:
                        issue("nonfinite_cloud_xyz", stamp=cloud_stamps[-1], axis=axis, count=bad)
    if collections.Counter(odom_stamps) != collections.Counter(cloud_stamps):
        issue("cloud_pose_stamp_mismatch",
              pose_only=list((collections.Counter(odom_stamps)-collections.Counter(cloud_stamps)).elements()),
              cloud_only=list((collections.Counter(cloud_stamps)-collections.Counter(odom_stamps)).elements()))
    for label, stamps in (("odometry", odom_stamps), ("cloud", cloud_stamps)):
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            issue("output_nonincreasing", topic=label)
    for path in sorted((run / "raw").glob("*.bag")):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(8*1024*1024), b""):
                digest.update(chunk)
        report["raw_sha256"][path.name] = digest.hexdigest()
    report["output"] = dict(odometry_count=len(odom_stamps), cloud_count=len(cloud_stamps),
                            cloud_points=total_points, first_header_ns=min(odom_stamps, default=None),
                            last_header_ns=max(odom_stamps, default=None),
                            max_step_m=max_step, max_step_header_interval_ns=max_step_interval)
    if odom_stamps:
        report["coverage"] = dict(
            start_gap_from_first_lidar_s=(odom_stamps[0]-report["input"]["/livox/avia/lidar"]["first_header_ns"])/1e9,
            end_gap_to_last_raw_scan_end_s=(report["input"]["/livox/avia/lidar"]["last_raw_scan_end_ns"]-odom_stamps[-1])/1e9)
    console = (run / "checks/algorithm-console.log").read_text(errors="replace")
    report["algorithm_warning_calls"] = {key: console.count(key) for key in
                                         ("No Effective Points", "No point, skip this scan", "loop back")}
    for filename in ("record-odom-console.log", "record-cloud-console.log"):
        for line in (run / "checks" / filename).read_text(errors="replace").splitlines():
            if any(word in line.lower() for word in ("buffer exceeded", "dropping", "dropped", "out of space")):
                issue("recorder_warning", file=filename, line=line)
    report["status"] = "STRUCTURE_OK_REVIEW_COVERAGE" if not issues else "REVIEW_REQUIRED"
    report["limits"] = ("Manual review required for player EOF, node exit codes, startup/tail coverage, "
                        "all warnings, jumps, resource use and RViz/map quality. Reading every recorded message "
                        "does not prove every input reached the algorithm or recording was lossless. "
                        "No GT accuracy, geometric degeneracy or real-time claim is made.")
    with out.open("x") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 2 if issues else 0


if __name__ == "__main__":
    sys.exit(main())
