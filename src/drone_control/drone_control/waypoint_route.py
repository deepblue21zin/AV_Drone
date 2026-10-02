"""Small, ROS-independent fixed-route sequencer for controlled experiments."""

import math


class WaypointRoute:
    def __init__(self, flattened, final_goal):
        values = list(flattened)
        if len(values) % 3:
            raise ValueError("outbound_waypoints must contain x,y,z triples")
        points = [tuple(float(v) for v in values[i:i+3]) for i in range(0, len(values), 3)]
        final_goal = tuple(float(v) for v in final_goal)
        if not points or points[-1] != final_goal:
            points.append(final_goal)
        if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError("All route goals must be finite x,y,z triples")
        if any(p[2] <= 0 for p in points):
            raise ValueError("Outbound route altitude must be positive")
        self.points = points
        self.index = 0

    @property
    def current(self):
        return self.points[self.index]

    def advance(self, x, y, reported_reached, tolerance=1.0):
        """Return changed/final/wait; ignore delayed success for a previous goal."""
        if not reported_reached or math.hypot(x-self.current[0], y-self.current[1]) > tolerance:
            return "wait"
        if self.index + 1 == len(self.points):
            return "final"
        self.index += 1
        return "changed"
