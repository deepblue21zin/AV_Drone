from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node


class VehicleInterface:
    def __init__(self, node: Node, mavros_namespace: str):
        self._node = node
        ns = mavros_namespace.rstrip("/")
        self.state = State()

        self._node.create_subscription(State, f"{ns}/state", self._on_state, 10)
        self.cmd_pub = self._node.create_publisher(TwistStamped, f"{ns}/setpoint_velocity/cmd_vel", 10)
        self.arm_cli = self._node.create_client(CommandBool, f"{ns}/cmd/arming")
        self.mode_cli = self._node.create_client(SetMode, f"{ns}/set_mode")

    def _on_state(self, msg: State):
        self.state = msg

    def publish_velocity(self, msg: TwistStamped):
        self.cmd_pub.publish(msg)

