"""Publish Gazebo entity state as timestamped ground-truth odometry."""

from __future__ import annotations

from functools import partial

import rclpy
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class GazeboGroundTruthRecorder(Node):
    """Poll Gazebo state without feeding ground truth into the flight stack."""

    def __init__(self):
        super().__init__("gazebo_gt_recorder")
        self.declare_parameter("vehicle_names", ["drone1", "drone2"])
        self.declare_parameter(
            "entity_names", ["iris_rplidar_0", "iris_rplidar_1"]
        )
        self.declare_parameter("service_name", "/gazebo/get_entity_state")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("reference_frame", "world")

        self.vehicle_names = [
            str(value) for value in self.get_parameter("vehicle_names").value
        ]
        self.entity_names = [
            str(value) for value in self.get_parameter("entity_names").value
        ]
        if len(self.vehicle_names) != len(self.entity_names):
            raise ValueError("vehicle_names and entity_names must have equal length")
        if not self.vehicle_names:
            raise ValueError("at least one ground-truth entity is required")
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        service_name = str(self.get_parameter("service_name").value)
        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.client = self.create_client(GetEntityState, service_name)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._gt_publishers = {
            vehicle: self.create_publisher(
                Odometry, f"/{vehicle}/ground_truth/odom", qos
            )
            for vehicle in self.vehicle_names
        }
        self._pending = {vehicle: False for vehicle in self.vehicle_names}
        self._failure_counts = {vehicle: 0 for vehicle in self.vehicle_names}
        self._service_warning_emitted = False
        self._poll_timer = self.create_timer(1.0 / publish_rate_hz, self._poll)
        self.get_logger().info(
            f"Ground truth recorder waiting for {service_name}; "
            f"entities={dict(zip(self.vehicle_names, self.entity_names))}"
        )

    def _poll(self):
        if not self.client.service_is_ready():
            if not self._service_warning_emitted:
                self.get_logger().warning(
                    "Gazebo get_entity_state service is not ready; GT publication paused"
                )
                self._service_warning_emitted = True
            return
        if self._service_warning_emitted:
            self.get_logger().info("Gazebo ground-truth service is ready")
            self._service_warning_emitted = False
        for vehicle, entity in zip(self.vehicle_names, self.entity_names):
            if self._pending[vehicle]:
                continue
            request = GetEntityState.Request()
            request.name = entity
            request.reference_frame = self.reference_frame
            self._pending[vehicle] = True
            future = self.client.call_async(request)
            future.add_done_callback(partial(self._handle_response, vehicle))

    def _handle_response(self, vehicle: str, future):
        self._pending[vehicle] = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - requires a failed ROS service
            self._record_failure(vehicle, f"service exception: {exc}")
            return
        if response is None or not response.success:
            # Humble GetEntityState has success/state but no status_message.
            # Missing entities during Gazebo startup must be retried, not fatal.
            status = "empty response" if response is None else getattr(
                response, "status_message", "entity not ready or lookup failed"
            )
            self._record_failure(vehicle, status)
            return

        state = response.state
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.reference_frame
        message.child_frame_id = f"{vehicle}/base_link_gt"
        message.pose.pose = state.pose
        message.twist.twist = state.twist
        self._gt_publishers[vehicle].publish(message)

    def _record_failure(self, vehicle: str, status: str):
        self._failure_counts[vehicle] += 1
        count = self._failure_counts[vehicle]
        if count == 1 or count % 50 == 0:
            self.get_logger().error(
                f"GT query failed for {vehicle} ({count} failures): {status}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GazeboGroundTruthRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
