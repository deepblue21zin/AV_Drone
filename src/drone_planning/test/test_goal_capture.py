import math

from drone_planning.local_planner_node import goal_capture_world_velocity


def test_goal_capture_velocity_points_directly_to_goal():
    vx, vy = goal_capture_world_velocity(-3.0, 4.0, 0.30)

    assert math.isclose(vx, -0.18)
    assert math.isclose(vy, 0.24)
    assert math.isclose(math.hypot(vx, vy), 0.30)


def test_goal_capture_velocity_slows_inside_speed_limit():
    vx, vy = goal_capture_world_velocity(0.03, 0.04, 0.30)

    assert math.isclose(vx, 0.03)
    assert math.isclose(vy, 0.04)


def test_goal_capture_velocity_is_zero_at_goal():
    assert goal_capture_world_velocity(0.0, 0.0, 0.30) == (0.0, 0.0)
