import heapq
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .slam_map import SavedSlamMap, inflate_mask


class SlamPathPlanner(Node):
    def __init__(self):
        super().__init__("slam_path_planner")
        self.declare_parameter("grid_path", "/workspace/AV_Drone/slam_world/obstacle_demo_v3_grid.npy")
        self.declare_parameter("meta_path", "/workspace/AV_Drone/slam_world/obstacle_demo_v3_meta.json")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("waypoint_topic", "/drone1/planner/slam/waypoint")
        self.declare_parameter("path_topic", "/drone1/planner/slam/path")
        self.declare_parameter("goal_x", 140.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("unknown_is_blocked", True)
        self.declare_parameter("robot_radius", 1.3)
        self.declare_parameter("lookahead_distance", 6.0)
        self.declare_parameter("publish_hz", 10.0)

        self.map = SavedSlamMap(
            str(self.get_parameter("grid_path").value),
            str(self.get_parameter("meta_path").value),
        )
        raw_blocked = self.map.blocked(
            int(self.get_parameter("occupied_threshold").value),
            bool(self.get_parameter("unknown_is_blocked").value),
        )
        radius_cells = int(math.ceil(float(self.get_parameter("robot_radius").value) / self.map.resolution))
        self.blocked = inflate_mask(raw_blocked, radius_cells)
        self.pose = None
        self.path = []
        self.progress_index = 0
        self.plan_time_sec = None

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self.on_pose,
            qos_profile_sensor_data,
        )
        self.waypoint_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("waypoint_topic").value), 10
        )
        self.path_pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 10)
        self.create_timer(1.0 / float(self.get_parameter("publish_hz").value), self.tick)
        self.get_logger().info(
            f"SLAM map loaded: shape={self.map.grid.shape}, resolution={self.map.resolution:.3f}, "
            f"blocked={int(np.count_nonzero(self.blocked))}, inflation_cells={radius_cells}"
        )

    def valid(self, cell):
        x, y = cell
        return 0 <= x < self.map.width and 0 <= y < self.map.height and not self.blocked[y, x]

    def nearest_valid(self, cell, max_radius=50):
        if self.valid(cell):
            return cell
        for radius in range(1, max_radius + 1):
            candidates = []
            for offset in range(-radius, radius + 1):
                candidates.extend((
                    (cell[0] + offset, cell[1] - radius),
                    (cell[0] + offset, cell[1] + radius),
                    (cell[0] - radius, cell[1] + offset),
                    (cell[0] + radius, cell[1] + offset),
                ))
            valid = [candidate for candidate in candidates if self.valid(candidate)]
            if valid:
                return min(valid, key=lambda p: math.hypot(p[0] - cell[0], p[1] - cell[1]))
        return None

    def astar(self, start, goal):
        neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
        ]
        queue = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        closed = set()
        while queue:
            _score, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == goal:
                cells = [current]
                while current in came_from:
                    current = came_from[current]
                    cells.append(current)
                return list(reversed(cells))
            closed.add(current)
            for dx, dy, step_cost in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not self.valid(nxt):
                    continue
                candidate = g_score[current] + step_cost
                if candidate < g_score.get(nxt, float("inf")):
                    came_from[nxt] = current
                    g_score[nxt] = candidate
                    heuristic = math.hypot(nxt[0] - goal[0], nxt[1] - goal[1])
                    heapq.heappush(queue, (candidate + heuristic, nxt))
        return []

    def line_is_free(self, start, goal):
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        steps = max(abs(dx), abs(dy)) * 2
        for index in range(max(1, steps) + 1):
            ratio = index / max(1, steps)
            cell = (int(round(start[0] + ratio * dx)), int(round(start[1] + ratio * dy)))
            if not self.valid(cell):
                return False
        return True

    def shortcut(self, cells):
        if len(cells) <= 2:
            return cells
        result = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            candidate = len(cells) - 1
            while candidate > anchor + 1 and not self.line_is_free(cells[anchor], cells[candidate]):
                candidate -= 1
            result.append(cells[candidate])
            anchor = candidate
        return result

    def densify(self, points, spacing=0.5):
        result = [points[0]]
        for start, goal in zip(points, points[1:]):
            distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
            steps = max(1, int(math.ceil(distance / spacing)))
            result.extend((
                start[0] + (goal[0] - start[0]) * i / steps,
                start[1] + (goal[1] - start[1]) * i / steps,
            ) for i in range(1, steps + 1))
        return result

    def on_pose(self, msg):
        self.pose = msg
        if self.path:
            return
        requested_start = self.map.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        requested_goal = self.map.world_to_grid(
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        start = self.nearest_valid(requested_start)
        goal = self.nearest_valid(requested_goal)
        if start is None or goal is None:
            self.get_logger().error(
                f"no valid SLAM plan endpoints near start={requested_start}, goal={requested_goal}"
            )
            return
        if start != requested_start or goal != requested_goal:
            self.get_logger().warn(
                f"SLAM endpoints snapped to inflated free space: "
                f"start={requested_start}->{start}, goal={requested_goal}->{goal}"
            )
        started = time.monotonic()
        raw = self.astar(start, goal)
        if not raw:
            self.get_logger().error("SLAM A* failed to find a path")
            return
        vertices = self.shortcut(raw)
        self.path = self.densify([self.map.grid_to_world(cell) for cell in vertices])
        self.plan_time_sec = time.monotonic() - started
        self.get_logger().info(
            f"SLAM path ready: raw={len(raw)}, vertices={len(vertices)}, tracking={len(self.path)}, "
            f"plan_time={self.plan_time_sec:.3f}s"
        )
        self.publish_path()

    def publish_path(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map.frame_id
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def tick(self):
        if self.pose is None or not self.path:
            return
        x, y = self.pose.pose.position.x, self.pose.pose.position.y
        nearest = min(
            range(self.progress_index, len(self.path)),
            key=lambda i: math.hypot(self.path[i][0] - x, self.path[i][1] - y),
        )
        self.progress_index = max(self.progress_index, nearest)
        lookahead = float(self.get_parameter("lookahead_distance").value)
        target = self.progress_index
        while target < len(self.path) - 1 and math.hypot(self.path[target][0] - x, self.path[target][1] - y) < lookahead:
            target += 1
        waypoint = PoseStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = self.map.frame_id
        waypoint.pose.position.x, waypoint.pose.position.y = self.path[target]
        waypoint.pose.position.z = 3.0
        waypoint.pose.orientation.w = 1.0
        self.waypoint_pub.publish(waypoint)
        self.publish_path()


def main(args=None):
    rclpy.init(args=args)
    node = SlamPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
