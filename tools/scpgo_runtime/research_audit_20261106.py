"""Read-only exploratory audit; prints JSON, never publishes ROS messages."""
import csv
import json
import re
from collections import deque
from pathlib import Path

import numpy as np
import rosbag
import rospy


def cloud_sample(msg):
    fields = {f.name: f for f in msg.fields}
    out = {"frame_id": msg.header.frame_id, "stamp": msg.header.stamp.to_sec(),
           "points": msg.width * msg.height,
           "fields": [{"name": f.name, "datatype": f.datatype, "offset": f.offset}
                      for f in msg.fields]}
    types = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    for name in ("t", "time", "timestamp", "ring"):
        if name not in fields:
            continue
        f = fields[name]
        dtype = np.dtype((">" if msg.is_bigendian else "<") + types[f.datatype])
        a = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       offset=f.offset, strides=(msg.row_step, msg.point_step)).ravel()
        out[name] = {"min": float(a.min()), "max": float(a.max()),
                     "nonzero": int(np.count_nonzero(a)), "unique": int(len(np.unique(a))),
                     "adjacent_decreases_in_storage_order": int(np.sum(np.diff(a.astype(float)) < 0))}
    return out


datasets = []
for filename, gtname in (
    ("2023-08-08-17-12-37-tunnel_s.bag", "gt-tunnel_s.csv"),
    ("2023-08-09-16-19-09-intersection_s.bag", "gt-intersection_s.csv"),
):
    p = Path("/datasets") / filename
    with rosbag.Bag(str(p), "r") as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
        info = bag.get_type_and_topic_info().topics
        item = {"file": filename, "bytes": p.stat().st_size, "start": start, "end": end,
                "duration_s": end-start,
                "topics": {k: {"type": v.msg_type, "messages": v.message_count,
                               "frequency": v.frequency} for k, v in info.items()},
                "pointcloud_samples": []}
        for topic, meta in info.items():
            if meta.msg_type != "sensor_msgs/PointCloud2":
                continue
            for relative in (0.0, (end-start)/2, max(0.0, end-start-1.0)):
                sample = next(bag.read_messages(topics=[topic], start_time=rospy.Time.from_sec(start+relative)), None)
                if sample:
                    entry = cloud_sample(sample.message)
                    entry.update(topic=topic, requested_bag_relative_s=relative,
                                 actual_bag_relative_s=sample.timestamp.to_sec()-start)
                    item["pointcloud_samples"].append(entry)
        gt = np.loadtxt(str(Path("/datasets") / gtname))
        dt = np.diff(gt[:, 0])
        item["gt"] = {"file": gtname, "rows": len(gt), "columns": gt.shape[1],
                      "finite": bool(np.isfinite(gt).all()), "start": float(gt[0, 0]),
                      "end": float(gt[-1, 0]), "nonpositive_time_steps": int(np.sum(dt <= 0)),
                      "max_time_gap_s": float(dt.max()),
                      "quaternions_all_identity": bool(np.all(gt[:, 4:7] == 0) and np.all(gt[:, 7] == 1)),
                      "start_end_position_distance_m": float(np.linalg.norm(gt[-1, 1:4]-gt[0, 1:4])),
                      "position_path_length_m": float(np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1).sum())}
        datasets.append(item)

base = Path("/results/scpgo/scpgo-corridor01-20261105-001/backend")
times = np.loadtxt(str(base / "times.txt"))
poses = np.loadtxt(str(base / "odom_poses.txt")).reshape(-1, 3, 4)
opt = np.loadtxt(str(base / "optimized_poses.txt")).reshape(-1, 3, 4)
with rosbag.Bag("/datasets/Corridor01.bag", "r") as bag:
    t0 = bag.get_start_time()
    duration = bag.get_end_time() - t0
with open("/workspace/tools/scpgo_runtime/verification/effect-loop-events.csv") as f:
    events = list(csv.DictReader(f))
queue, verified_events = deque(), []
for line in (base.parent / 'checks/backend-console.log').read_text(errors='replace').splitlines():
    pair = re.search(r'Loop detected! - between (\d+) and (\d+)', line)
    if pair:
        queue.append(tuple(map(int, pair.groups())))
    outcome = re.search(r'ICP fitness test (passed|failed) \(([^ ]+)', line)
    if outcome:
        previous, current = queue.popleft()
        verified_events.append((previous, current, outcome[1], float(outcome[2])))
assert verified_events == [(int(e['previous']), int(e['current']), e['result'], float(e['log_score'])) for e in events]
for e in events:
    for k in ("event", "previous", "current"):
        e[k] = int(e[k])
    e["current_bag_relative_s"] = float(times[e["current"]]-t0)
    e["previous_bag_relative_s"] = float(times[e["previous"]]-t0)


def summarize(rows):
    accepted = [e for e in rows if e["result"] == "passed"]
    return {"events": len(rows), "passed": len(accepted),
            "unique_passed_pairs": len({(e["previous"], e["current"]) for e in accepted}),
            "passed_pairs": sorted({(e["previous"], e["current"]) for e in accepted})}


result = {"scope": "Read-only metadata and existing-output exploration; no SLAM replay or parameter change.",
          "datasets": datasets,
          "corridor01": {"keyframes": len(times),
                         "event_csv_verified_against_original_log": True,
                         "pending_candidates_without_outcome": list(queue),
                         "max_pose_translation_change_m": float(np.linalg.norm(opt[:, :, 3]-poses[:, :, 3], axis=1).max()),
                         "all_events": summarize(events),
                         "last60s": summarize([e for e in events if e["current_bag_relative_s"] >= duration-60]),
                         "from380s": summarize([e for e in events if e["current_bag_relative_s"] >= 380]),
                         "from390s": summarize([e for e in events if e["current_bag_relative_s"] >= 390]),
                         "terminal_events": [e for e in events if e["current_bag_relative_s"] >= 390]},
          "limitations": ["Cloud fields sampled at three bag-time locations per topic, not a full-input validation.",
                          "Identity quaternion columns do not provide attitude ground truth.",
                          "Time-window membership and ICP acceptance do not identify physical loop truth.",
                          "GT path length uses consecutive provided positions, without removing gaps or interpolation."]}
print(json.dumps(result, indent=2))
