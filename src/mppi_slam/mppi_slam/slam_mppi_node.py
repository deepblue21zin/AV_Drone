import math

import numpy as np
import rclpy

from mppi.mppi_node import MPPIPlannerNode, Obstacle2D
from .slam_map import SavedSlamMap


class SlamMPPIPlannerNode(MPPIPlannerNode):
    def _load_obstacles(self):
        grid_path = str(self.get_parameter("world_path").value)
        meta_path = grid_path.replace("_grid.npy", "_meta.json")
        slam_map = SavedSlamMap(grid_path, meta_path)
        occupied = slam_map.grid >= 50
        visited = np.zeros_like(occupied, dtype=bool)
        obstacles = []
        height, width = occupied.shape

        for row in range(height):
            for col in range(width):
                if not occupied[row, col] or visited[row, col]:
                    continue
                stack = [(col, row)]
                visited[row, col] = True
                component = []
                while stack:
                    x, y = stack.pop()
                    component.append((x, y))
                    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height and occupied[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = True
                            stack.append((nx, ny))
                if len(component) < 2:
                    continue
                points = np.asarray([slam_map.grid_to_world(cell) for cell in component], dtype=np.float64)
                center = np.mean(points, axis=0)
                centered = points - center
                covariance = centered.T @ centered / len(points)
                values, vectors = np.linalg.eigh(covariance)
                major = vectors[:, int(np.argmax(values))]
                minor = np.array([-major[1], major[0]])
                major_projection = centered @ major
                minor_projection = centered @ minor
                lo, hi = float(np.min(major_projection)), float(np.max(major_projection))
                thickness = 0.5 * float(np.max(minor_projection) - np.min(minor_projection))
                radius = max(0.12, thickness + 0.5 * slam_map.resolution)
                start = center + lo * major
                end = center + hi * major
                if math.hypot(*(end - start)) < 0.2:
                    obstacles.append(Obstacle2D(float(center[0]), float(center[1]), radius))
                else:
                    obstacles.append(Obstacle2D(float(start[0]), float(start[1]), radius, float(end[0]), float(end[1])))
        self.get_logger().info(
            f"SLAM obstacle source: {grid_path}, components={len(obstacles)}, occupied_cells={int(np.count_nonzero(occupied))}"
        )
        return obstacles


def main(args=None):
    rclpy.init(args=args)
    node = SlamMPPIPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
