from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped

from drone_control.autonomy_manager_node import AutonomyManagerNode


def test_existing_home_pose_is_not_overwritten():
    existing = PoseStamped()
    existing.pose.position.x = 3.0
    existing.pose.position.y = -7.5
    replacement = PoseStamped()
    replacement.pose.position.x = 120.0
    replacement.pose.position.y = 4.0

    fake = SimpleNamespace(
        _home_pose=existing,
        vehicle=SimpleNamespace(pose=replacement),
        _copy_current_pose=Mock(return_value=replacement),
        _publish_home_pose=Mock(),
        get_logger=Mock(return_value=Mock()),
    )

    AutonomyManagerNode._capture_home_pose(fake)

    assert fake._home_pose is existing
    fake._copy_current_pose.assert_not_called()
    fake._publish_home_pose.assert_called_once_with()
