import pytest

from drone_control.waypoint_route import WaypointRoute


def test_single_goal_preserves_old_behavior():
    route = WaypointRoute([], [10, 0, 3])
    assert route.current == (10, 0, 3)
    assert route.advance(0, 0, True) == "wait"
    assert route.advance(10, 0, False) == "wait"
    assert route.advance(10, 0, True) == "final"


def test_waypoints_only_finish_at_final_goal_and_reject_stale_success():
    route = WaypointRoute([10, 1, 3, 20, 2, 3], [30, 0, 3])
    assert route.advance(10, 1, True) == "changed"
    assert route.current == (20, 2, 3)
    assert route.advance(10, 1, True) == "wait"
    assert route.advance(20, 2, True) == "changed"
    assert route.advance(30, 0, True) == "final"


def test_duplicate_final_is_not_appended():
    assert len(WaypointRoute([10, 0, 3], [10, 0, 3]).points) == 1


@pytest.mark.parametrize("values", [[1, 2], [1, 2, -1], [float("nan"), 2, 3]])
def test_bad_route_rejected(values):
    with pytest.raises(ValueError):
        WaypointRoute(values, [20, 0, 3])
