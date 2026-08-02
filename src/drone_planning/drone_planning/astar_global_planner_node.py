#!/usr/bin/env python3

import heapq
import math
import time
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

from mppi.world_geometry import load_world_geometry


class AstarGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("astar_global_planner")

        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("waypoint_topic", "/drone1/planner/astar/waypoint")
        self.declare_parameter(
            "compute_time_topic", "/drone1/planner/astar/compute_time_ms"
        )

        self.declare_parameter("goal_x", 140.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 3.0)

        self.declare_parameter("publish_hz", 5.0)

        # A* map 설정
        self.declare_parameter("map_min_x", -5.0)
        self.declare_parameter("map_max_x", 150.0)
        self.declare_parameter("map_min_y", -20.0)
        self.declare_parameter("map_max_y", 20.0)
        self.declare_parameter("resolution", 1.0)

        # 장애물 반경 + 드론 여유 반경
        self.declare_parameter("robot_radius", 1.2)
        self.declare_parameter("obstacle_default_radius", 0.5)

        # waypoint 전환 설정
        self.declare_parameter("waypoint_reach_dist", 2.0)
        self.declare_parameter("lookahead_waypoint_count", 5)

        # world 파일에서 장애물 읽기
        self.declare_parameter(
            "world_path",
            "/workspace/AV_Drone/sim_assets/worlds/obstacle_demo.world",
        )

        self.pose_topic = self.get_parameter("pose_topic").value
        self.waypoint_topic = self.get_parameter("waypoint_topic").value

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_z = float(self.get_parameter("goal_z").value)

        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.map_min_x = float(self.get_parameter("map_min_x").value)
        self.map_max_x = float(self.get_parameter("map_max_x").value)
        self.map_min_y = float(self.get_parameter("map_min_y").value)
        self.map_max_y = float(self.get_parameter("map_max_y").value)
        self.resolution = float(self.get_parameter("resolution").value)

        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.waypoint_reach_dist = float(
            self.get_parameter("waypoint_reach_dist").value
        )
        self.lookahead_waypoint_count = int(
            self.get_parameter("lookahead_waypoint_count").value
        )

        self.world_path = str(self.get_parameter("world_path").value)

        self.nx = int(round((self.map_max_x - self.map_min_x) / self.resolution)) + 1
        self.ny = int(round((self.map_max_y - self.map_min_y) / self.resolution)) + 1

        self.current_pose: Optional[PoseStamped] = None

        self.geometry = load_world_geometry(self.world_path)
        self.occupied_cells = self.build_occupancy_grid()

        self.path: List[Tuple[float, float]] = []
        self.path_index = 0
        self.path_initialized = False

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )

        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            self.waypoint_topic,
            10,
        )
        self.compute_time_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("compute_time_topic").value),
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_hz, 1.0),
            self.timer_callback,
        )

        self.get_logger().info("A* global planner node started")
        self.get_logger().info(f"pose_topic: {self.pose_topic}")
        self.get_logger().info(f"waypoint_topic: {self.waypoint_topic}")
        self.get_logger().info(
            f"goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}, z={self.goal_z:.2f}"
        )
        self.get_logger().info(
            f"map: x=[{self.map_min_x}, {self.map_max_x}], "
            f"y=[{self.map_min_y}, {self.map_max_y}], "
            f"resolution={self.resolution}, grid={self.nx}x{self.ny}"
        )
        self.get_logger().info(
            "collision geometry loaded: rectangles={}, circles={}".format(
                len(self.geometry.rectangles), len(self.geometry.circles)
            )
        )
        self.get_logger().info(f"occupied cells: {len(self.occupied_cells)}")

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def timer_callback(self):
        if self.current_pose is None:
            self.get_logger().warn(
                "Waiting for current pose...",
                throttle_duration_sec=2.0,
            )
            return

        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y

        # 처음 현재 위치를 받은 순간 A* 경로 계산
        if not self.path_initialized:
            started = time.perf_counter()
            self.path = self.plan_astar(x, y, self.goal_x, self.goal_y)
            compute_time_ms = (time.perf_counter() - started) * 1000.0
            self.compute_time_pub.publish(Float32(data=float(compute_time_ms)))

            if not self.path:
                self.get_logger().error("A* failed. Fallback to final goal only.")
                self.path = [(self.goal_x, self.goal_y)]

            self.path_index = self.find_nearest_path_index(x, y)
            self.path_initialized = True

            self.get_logger().info(
                f"A* path initialized: points={len(self.path)}, "
                f"start=({x:.2f},{y:.2f}), "
                f"goal=({self.goal_x:.2f},{self.goal_y:.2f}), "
                f"nearest_index={self.path_index}, compute_ms={compute_time_ms:.3f}"
            )

        # 현재 위치 기준 가장 가까운 path index로 진행도 갱신
        nearest_idx = self.find_nearest_path_index(x, y)
        if nearest_idx > self.path_index:
            self.path_index = nearest_idx

        # 현재 waypoint에 가까워지면 다음 index로 이동
        while self.path_index < len(self.path) - 1:
            wx, wy = self.path[self.path_index]
            dist_to_base_wp = math.hypot(wx - x, wy - y)
            if dist_to_base_wp < self.waypoint_reach_dist:
                self.path_index += 1
            else:
                break

        # MPPI에는 조금 앞쪽 waypoint를 줌
        target_index = min(
            self.path_index + self.lookahead_waypoint_count,
            len(self.path) - 1,
        )
        target_x, target_y = self.path[target_index]

        dist_to_target = math.hypot(target_x - x, target_y - y)

        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = "map"
        waypoint_msg.pose.position.x = target_x
        waypoint_msg.pose.position.y = target_y
        waypoint_msg.pose.position.z = self.goal_z
        waypoint_msg.pose.orientation.w = 1.0

        self.waypoint_pub.publish(waypoint_msg)

        self.get_logger().info(
            f"Published A* waypoint: base_idx={self.path_index}, "
            f"target_idx={target_index}, "
            f"target=({target_x:.2f}, {target_y:.2f}), "
            f"current=({x:.2f}, {y:.2f}), "
            f"dist_to_target={dist_to_target:.2f}",
            throttle_duration_sec=2.0,
        )

    def find_nearest_path_index(self, x: float, y: float) -> int:
        if not self.path:
            return 0

        min_dist = float("inf")
        min_idx = 0

        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - x, py - y)
            if d < min_dist:
                min_dist = d
                min_idx = i

        return min_idx

    def build_occupancy_grid(self) -> Set[Tuple[int, int]]:
        occupied = set()
        rectangles = [
            obstacle
            for obstacle in self.geometry.rectangles
            if max(obstacle.half_x, obstacle.half_y) * 2.0 < 28.0
        ]

        for ix in range(self.nx):
            for iy in range(self.ny):
                wx, wy = self.grid_to_world(ix, iy)
                for obstacle in rectangles:
                    cosine, sine = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
                    dx, dy = wx - obstacle.x, wy - obstacle.y
                    local_x = cosine * dx + sine * dy
                    local_y = -sine * dx + cosine * dy
                    if (
                        abs(local_x) <= obstacle.half_x + self.robot_radius
                        and abs(local_y) <= obstacle.half_y + self.robot_radius
                    ):
                        occupied.add((ix, iy))
                        break
                else:
                    for obstacle in self.geometry.circles:
                        if math.hypot(wx - obstacle.x, wy - obstacle.y) <= (
                            obstacle.radius + self.robot_radius
                        ):
                            occupied.add((ix, iy))
                            break

        return occupied

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        ix = int(round((x - self.map_min_x) / self.resolution))
        iy = int(round((y - self.map_min_y) / self.resolution))

        ix = max(0, min(self.nx - 1, ix))
        iy = max(0, min(self.ny - 1, iy))

        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        x = self.map_min_x + ix * self.resolution
        y = self.map_min_y + iy * self.resolution
        return x, y

    def is_in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def is_occupied(self, ix: int, iy: int) -> bool:
        return (ix, iy) in self.occupied_cells

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan_astar(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
    ) -> List[Tuple[float, float]]:
        start = self.world_to_grid(start_x, start_y)
        goal = self.world_to_grid(goal_x, goal_y)

        self.get_logger().info(f"A* start_grid={start}, goal_grid={goal}")

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {start: 0.0}

        closed = set()

        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            if current == goal:
                return self.reconstruct_path(came_from, current, start_x, start_y, goal_x, goal_y)

            closed.add(current)

            for dx, dy, step_cost in neighbors:
                nx = current[0] + dx
                ny = current[1] + dy

                if not self.is_in_bounds(nx, ny):
                    continue

                neighbor = (nx, ny)

                # 시작점과 목표점은 occupied여도 허용
                if neighbor != start and neighbor != goal:
                    if self.is_occupied(nx, ny):
                        continue

                tentative_g = g_score[current] + step_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_score, neighbor))

        self.get_logger().error("A* could not find path.")
        return []

    def reconstruct_path(
        self,
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        current: Tuple[int, int],
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
    ) -> List[Tuple[float, float]]:
        cells = [current]

        while current in came_from:
            current = came_from[current]
            cells.append(current)

        cells.reverse()

        path = [self.grid_to_world(ix, iy) for ix, iy in cells]

        if path:
            path[0] = (start_x, start_y)
            path[-1] = (goal_x, goal_y)

        return path


def main(args=None):
    rclpy.init(args=args)
    node = AstarGlobalPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
