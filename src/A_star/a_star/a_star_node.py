#!/usr/bin/env python3

import heapq
import math
import re
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32


def tag(element):
    return element.tag.split("}", 1)[-1] if "}" in element.tag else element.tag


def child(element, name):
    return next((item for item in list(element) if tag(item) == name), None)


def text(element, name, default=""):
    item = child(element, name)
    return (item.text or "").strip() if item is not None else default


def pose_values(element):
    values = [float(value) for value in text(element, "pose", "0 0 0 0 0 0").split()]
    return (values + [0.0] * 6)[:6]


class AStarNode(Node):
    def __init__(self):
        super().__init__("a_star")
        parameters = {
            "pose_topic": "/mavros/local_position/pose",
            "cmd_topic": "/drone1/autonomy/cmd_vel",
            "goal_reached_topic": "/drone1/mission/goal_reached",
            "compute_time_topic": "/drone1/planner/astar/compute_time_ms",
            "world_path": "/workspace/AV_Drone/sim_assets/worlds/obstacle_demo.world",
            "goal_x": 140.0, "goal_y": 0.0,
            "map_min_x": 0.0, "map_max_x": 150.0,
            "map_min_y": -14.0, "map_max_y": 14.0,
            "resolution": 0.5, "robot_radius": 1.2,
            "lookahead_distance": 3.0, "cruise_speed": 1.2,
            "max_speed": 1.8, "goal_tolerance": 0.7,
            "turn_preview_distance": 6.0, "turn_speed": 1.5,
            "turn_angle_threshold": 0.2,
            "max_planar_accel": 1.5,
            "publish_hz": 20.0, "pose_timeout_sec": 0.5,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)
        self.pose = None
        self.pose_time = 0.0
        self.path = []
        self.progress_index = 0
        self.goal_reached = False
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.cmd_pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("cmd_topic").value), 10
        )
        self.goal_pub = self.create_publisher(
            Bool, str(self.get_parameter("goal_reached_topic").value), 10
        )
        self.compute_time_pub = self.create_publisher(
            Float32, str(self.get_parameter("compute_time_topic").value), 10
        )
        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter("publish_hz").value), self.tick)
        self.occupied = self.build_occupancy()
        self.get_logger().info("A* map ready: occupied_cells={}".format(len(self.occupied)))

    def world_to_grid(self, x, y):
        resolution = float(self.get_parameter("resolution").value)
        return (
            int(round((x - float(self.get_parameter("map_min_x").value)) / resolution)),
            int(round((y - float(self.get_parameter("map_min_y").value)) / resolution)),
        )

    def grid_to_world(self, cell):
        resolution = float(self.get_parameter("resolution").value)
        return (
            float(self.get_parameter("map_min_x").value) + cell[0] * resolution,
            float(self.get_parameter("map_min_y").value) + cell[1] * resolution,
        )

    def build_occupancy(self):
        world = Path(str(self.get_parameter("world_path").value))
        root = ET.parse(str(world)).getroot()
        resolution = float(self.get_parameter("resolution").value)
        radius = float(self.get_parameter("robot_radius").value)
        min_x = float(self.get_parameter("map_min_x").value)
        max_x = float(self.get_parameter("map_max_x").value)
        min_y = float(self.get_parameter("map_min_y").value)
        max_y = float(self.get_parameter("map_max_y").value)
        nx = int(round((max_x - min_x) / resolution)) + 1
        ny = int(round((max_y - min_y) / resolution)) + 1
        occupied = set()
        rectangles = []
        circles = []
        for include in root.iter():
            if tag(include) != "include":
                continue
            name = text(include, "name", "")
            uri = text(include, "uri", "")
            if not name.startswith("cylinder_"):
                continue
            ix, iy, _iz, _ir, _ip, _iyaw = pose_values(include)
            match = re.search(r"_r(\d+)", uri)
            obstacle_radius = float(match.group(1)) / 10.0 if match else 0.5
            circles.append((ix, iy, obstacle_radius + radius))
        for model in root.iter():
            if tag(model) != "model" or text(model, "static", "false").lower() not in {"1", "true"}:
                continue
            mx, my, _mz, _r, _p, yaw = pose_values(model)
            for collision in model.iter():
                if tag(collision) != "collision":
                    continue
                geometry = child(collision, "geometry")
                box = child(geometry, "box") if geometry is not None else None
                if box is None:
                    continue
                size = [float(value) for value in text(box, "size").split()]
                if len(size) < 3 or (size[2] <= 0.1 and size[0] >= 5.0 and size[1] >= 5.0):
                    continue
                # Corridor-wide boundary walls are represented by grid bounds.
                # Rasterizing them would trap the local-pose origin inside an
                # inflated start boundary because PX4 local and world origins
                # differ by roughly half a metre.
                if max(size[0], size[1]) >= 28.0:
                    continue
                rectangles.append((mx, my, size[0] * 0.5 + radius, size[1] * 0.5 + radius, yaw))
        for ix in range(nx):
            for iy in range(ny):
                wx, wy = self.grid_to_world((ix, iy))
                for cx, cy, hx, hy, yaw in rectangles:
                    cosine, sine = math.cos(yaw), math.sin(yaw)
                    dx, dy = wx - cx, wy - cy
                    lx, ly = cosine * dx + sine * dy, -sine * dx + cosine * dy
                    if abs(lx) <= hx and abs(ly) <= hy:
                        occupied.add((ix, iy))
                        break
                if (ix, iy) in occupied:
                    continue
                for cx, cy, inflated_radius in circles:
                    if math.hypot(wx - cx, wy - cy) <= inflated_radius:
                        occupied.add((ix, iy))
                        break
        self.nx, self.ny = nx, ny
        return occupied

    def plan(self, start, goal):
        neighbors = [(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2))]
        queue = [(0.0, start)]
        came_from, g_score, closed = {}, {start: 0.0}, set()
        while queue:
            _score, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == goal:
                cells = [current]
                while current in came_from:
                    current = came_from[current]
                    cells.append(current)
                return [self.grid_to_world(cell) for cell in reversed(cells)]
            closed.add(current)
            for dx, dy, cost in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not (0 <= nxt[0] < self.nx and 0 <= nxt[1] < self.ny) or nxt in self.occupied:
                    continue
                candidate = g_score[current] + cost
                if candidate < g_score.get(nxt, float("inf")):
                    came_from[nxt] = current
                    g_score[nxt] = candidate
                    heuristic = math.hypot(nxt[0] - goal[0], nxt[1] - goal[1])
                    heapq.heappush(queue, (candidate + heuristic, nxt))
        return []

    def line_is_free(self, start, goal):
        """팽창된 점유격자에서 두 셀 사이 직선의 충돌 여부를 검사한다."""
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        steps = max(abs(dx), abs(dy)) * 2
        if steps == 0:
            return start not in self.occupied
        for index in range(steps + 1):
            ratio = index / steps
            cell = (
                int(round(start[0] + ratio * dx)),
                int(round(start[1] + ratio * dy)),
            )
            if cell in self.occupied:
                return False
        return True

    def shortcut_path(self, path):
        """시야가 트인 중간 격자점을 제거해 비행거리와 불필요한 회전을 줄인다."""
        if len(path) <= 2:
            return path
        cells = [self.world_to_grid(x, y) for x, y in path]
        result = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            candidate = len(path) - 1
            while candidate > anchor + 1 and not self.line_is_free(cells[anchor], cells[candidate]):
                candidate -= 1
            result.append(path[candidate])
            anchor = candidate
        return result

    @staticmethod
    def densify_path(path, spacing=0.5):
        """단순화된 선분을 추종기가 안정적으로 전진할 수 있게 재표본화한다."""
        if len(path) <= 1:
            return path
        result = [path[0]]
        for start, goal in zip(path, path[1:]):
            dx, dy = goal[0] - start[0], goal[1] - start[1]
            distance = math.hypot(dx, dy)
            steps = max(1, int(math.ceil(distance / spacing)))
            for index in range(1, steps + 1):
                ratio = index / steps
                result.append((start[0] + ratio * dx, start[1] + ratio * dy))
        return result

    def on_pose(self, message):
        self.pose = message
        self.pose_time = self.get_clock().now().nanoseconds * 1e-9
        if not self.path:
            p = message.pose.position
            start = self.world_to_grid(p.x, p.y)
            goal = self.world_to_grid(
                float(self.get_parameter("goal_x").value),
                float(self.get_parameter("goal_y").value),
            )
            started = time.perf_counter()
            raw_path = self.plan(start, goal)
            shortcut = self.shortcut_path(raw_path)
            self.path = self.densify_path(shortcut)
            compute_time_ms = (time.perf_counter() - started) * 1000.0
            self.compute_time_pub.publish(Float32(data=float(compute_time_ms)))
            self.get_logger().info(
                "A* path planned: raw_points={}, shortcut_vertices={}, "
                "tracking_points={}, compute_ms={:.3f}".format(
                    len(raw_path), len(shortcut), len(self.path), compute_time_ms
                )
            )

    def publish_command(self, vx, vy):
        max_accel = float(self.get_parameter("max_planar_accel").value)
        dt = 1.0 / float(self.get_parameter("publish_hz").value)
        dvx, dvy = vx - self.last_vx, vy - self.last_vy
        delta = math.hypot(dvx, dvy)
        max_delta = max_accel * dt
        if delta > max_delta:
            scale = max_delta / delta
            vx = self.last_vx + dvx * scale
            vy = self.last_vy + dvy * scale
        self.last_vx, self.last_vy = vx, vy
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        self.cmd_pub.publish(message)

    def tick(self):
        self.goal_pub.publish(Bool(data=self.goal_reached))
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.pose is None or now - self.pose_time > float(self.get_parameter("pose_timeout_sec").value) or not self.path:
            self.publish_command(0.0, 0.0)
            return
        p = self.pose.pose.position
        goal_x, goal_y = float(self.get_parameter("goal_x").value), float(self.get_parameter("goal_y").value)
        if math.hypot(goal_x - p.x, goal_y - p.y) <= float(self.get_parameter("goal_tolerance").value):
            self.goal_reached = True
            self.goal_pub.publish(Bool(data=True))
            self.publish_command(0.0, 0.0)
            return
        nearest = min(range(self.progress_index, len(self.path)), key=lambda i: math.hypot(self.path[i][0] - p.x, self.path[i][1] - p.y))
        self.progress_index = max(self.progress_index, nearest)
        lookahead = float(self.get_parameter("lookahead_distance").value)
        target_index = self.progress_index
        while target_index < len(self.path) - 1 and math.hypot(self.path[target_index][0] - p.x, self.path[target_index][1] - p.y) < lookahead:
            target_index += 1
        tx, ty = self.path[target_index]
        dx, dy = tx - p.x, ty - p.y
        distance = math.hypot(dx, dy) + 1e-6
        # `distance` is the distance to the short lookahead waypoint, not a
        # stopping distance. Capping by it unintentionally limited a 2.7 m/s
        # cruise command to roughly the 1 m lookahead distance.
        speed = min(float(self.get_parameter("max_speed").value), float(self.get_parameter("cruise_speed").value))
        preview_distance = float(self.get_parameter("turn_preview_distance").value)
        preview_index = target_index
        accumulated = 0.0
        while preview_index < len(self.path) - 1 and accumulated < preview_distance:
            ax, ay = self.path[preview_index]
            bx, by = self.path[preview_index + 1]
            accumulated += math.hypot(bx - ax, by - ay)
            preview_index += 1
        if preview_index > target_index:
            px, py = self.path[preview_index]
            approach_heading = math.atan2(dy, dx)
            preview_heading = math.atan2(py - ty, px - tx)
            turn_angle = abs(math.atan2(
                math.sin(preview_heading - approach_heading),
                math.cos(preview_heading - approach_heading),
            ))
            if turn_angle >= float(self.get_parameter("turn_angle_threshold").value):
                speed = min(speed, float(self.get_parameter("turn_speed").value))
        if math.hypot(goal_x - p.x, goal_y - p.y) < 2.0:
            speed = min(speed, 1.2)
        self.publish_command(speed * dx / distance, speed * dy / distance)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
