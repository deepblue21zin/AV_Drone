import time

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class VehicleInterface:
    def __init__(self, node: Node, mavros_namespace: str, pose_topic: str | None = None):
        self._node = node
        ns = mavros_namespace.rstrip("/")
        self.state = State()
        self.pose = None
        self.last_pose_t = 0.0

        self._node.create_subscription(State, f"{ns}/state", self._on_state, 10)
        pose_topic = pose_topic or f"{ns}/local_position/pose"
        self._node.create_subscription(
            PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data
        )
        self.cmd_pub = self._node.create_publisher(TwistStamped, f"{ns}/setpoint_velocity/cmd_vel", 10)
        self.arm_cli = self._node.create_client(CommandBool, f"{ns}/cmd/arming")
        self.mode_cli = self._node.create_client(SetMode, f"{ns}/set_mode")

    def _on_state(self, msg: State):
        self.state = msg

    def _on_pose(self, msg: PoseStamped):
        self.pose = msg
        self.last_pose_t = time.time()

    def publish_velocity(self, msg: TwistStamped):
        self.cmd_pub.publish(msg)

    def pose_age(self) -> float:
        if self.pose is None:
            return 1e9
        return time.time() - self.last_pose_t
