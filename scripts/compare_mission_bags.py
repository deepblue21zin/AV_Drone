#!/usr/bin/env python3
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import ParameterEvent


def analyze(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = {
        "/drone1/mission/phase": String,
        "/drone1/mission/goal_reached": Bool,
        "/mavros/local_position/pose": PoseStamped,
        "/parameter_events": ParameterEvent,
    }
    bag_first = None
    phases = []
    goal_time = None
    poses = []
    speed_parameters = []
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        if bag_first is None:
            bag_first = timestamp
        msg_type = topics.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(raw, msg_type)
        if topic.endswith("/phase"):
            if not phases or phases[-1][1] != msg.data:
                phases.append((timestamp, msg.data))
        elif topic.endswith("/goal_reached") and msg.data and goal_time is None:
            goal_time = timestamp
        elif topic.endswith("/pose"):
            p = msg.pose.position
            poses.append((timestamp, p.x, p.y, p.z))
        elif topic == "/parameter_events":
            for parameter in list(msg.new_parameters) + list(msg.changed_parameters):
                if parameter.name in {"v_nom", "v_max", "cruise_speed", "max_speed", "lookahead_distance", "slowdown_dist"}:
                    value = parameter.value
                    if value.type == 3:
                        parsed = value.double_value
                    elif value.type == 2:
                        parsed = value.integer_value
                    else:
                        parsed = str(value)
                    speed_parameters.append((msg.node, parameter.name, parsed))

    start_candidates = [
        (t, phase)
        for t, phase in phases
        if phase in ("FOLLOW_PLAN", "MAPPING_TO_GOAL", "MPPI_GO")
    ]
    if not start_candidates or goal_time is None:
        raise RuntimeError(f"missing mission boundary: phases={phases}, goal={goal_time}")
    start_time, start_phase = start_candidates[0]
    segment = [p for p in poses if start_time <= p[0] <= goal_time]
    distance_3d = sum(
        math.dist(a[1:], b[1:]) for a, b in zip(segment, segment[1:])
    )
    distance_xy = sum(
        math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(segment, segment[1:])
    )
    duration = (goal_time - start_time) / 1e9
    return {
        "bag": path,
        "start_phase": start_phase,
        "start_offset": (start_time - bag_first) / 1e9,
        "goal_offset": (goal_time - bag_first) / 1e9,
        "duration": duration,
        "distance_xy": distance_xy,
        "distance_3d": distance_3d,
        "mean_xy": distance_xy / duration,
        "mean_3d": distance_3d / duration,
        "start_pose": segment[0][1:],
        "goal_pose": segment[-1][1:],
        "phases": [((t - bag_first) / 1e9, phase) for t, phase in phases],
        "samples": len(segment),
        "speed_parameters": speed_parameters,
    }


for bag in sys.argv[1:]:
    result = analyze(bag)
    for key, value in result.items():
        print(f"{key}: {value}")
    print()
