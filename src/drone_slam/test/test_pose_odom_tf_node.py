import math
from types import SimpleNamespace
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from drone_slam.pose_odom_tf_node import PoseOdomTfNode


def _yaw_pose(yaw):
    pose = PoseStamped()
    pose.pose.orientation.z = math.sin(0.5 * yaw)
    pose.pose.orientation.w = math.cos(0.5 * yaw)
    return pose


def _fake_node():
    return SimpleNamespace(
        _odom_frame="drone1/odom",
        _base_frame="drone1/base_link",
        _last_tf_stamp_ns=None,
        _stamp_ns=PoseOdomTfNode._stamp_ns,
        _broadcaster=Mock(),
        get_logger=Mock(return_value=Mock()),
    )


def test_broadcast_pose_uses_requested_stamp_and_rejects_duplicates():
    fake = _fake_node()
    pose = PoseStamped()
    pose.pose.position.x = 4.0
    pose.pose.position.y = -2.0
    pose.pose.orientation.w = 1.0
    stamp = Time(sec=12, nanosec=300)

    PoseOdomTfNode._broadcast_pose(fake, pose, stamp)
    PoseOdomTfNode._broadcast_pose(fake, pose, stamp)

    fake._broadcaster.sendTransform.assert_called_once()
    transform = fake._broadcaster.sendTransform.call_args.args[0]
    assert transform.header.stamp == stamp
    assert transform.header.frame_id == "drone1/odom"
    assert transform.child_frame_id == "drone1/base_link"
    assert transform.transform.translation.x == 4.0


def test_scan_callback_uses_scan_timestamp_after_pose_is_available():
    pose = PoseStamped()
    scan = LaserScan()
    scan.header.stamp = Time(sec=8, nanosec=25)
    fake = SimpleNamespace(
        _latest_pose=pose,
        _broadcast_pose=Mock(),
        get_logger=Mock(return_value=Mock()),
    )

    PoseOdomTfNode._on_scan(fake, scan)

    fake._broadcast_pose.assert_called_once_with(pose, scan.header.stamp)


def test_initial_yaw_offset_is_removed_from_slam_pose():
    pose = _yaw_pose(0.35)

    corrected = PoseOdomTfNode._remove_yaw_offset(pose, 0.10)

    assert math.isclose(
        PoseOdomTfNode._yaw_from_pose(corrected), 0.25, abs_tol=1e-9
    )
