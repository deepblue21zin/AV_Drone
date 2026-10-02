"""Harmless subprocesses for guard tests; never start PX4/Gazebo."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))


def main():
    mode = sys.argv[1]
    if mode == "supervise":
        import runtime_guard as guard
        guard.PROJECT = Path(os.environ["GUARD_TEST_PROJECT"])
        guard.DESCRIPTOR = guard.PROJECT / "descriptor.json"
        guard.load_limits = lambda: json.loads(os.environ["GUARD_TEST_LIMITS"])
        return guard.supervise(os.environ.get("GUARD_TEST_ROLE", "ros"), sys.argv[2:],
                               os.environ.get("GUARD_TEST_RUN"), os.environ.get("GUARD_TEST_MISSION"))
    target = Path(sys.argv[2])
    if mode == "mission_launch":
        import rclpy
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node("runtime_guard_test_publisher")
        one = node.create_publisher(String, "/guard_test/drone1/mission/phase", 10)
        two = node.create_publisher(String, "/guard_test/drone2/mission/phase", 10)
        child = subprocess.Popen(["ros2", "bag", "record", "--output", str(target),
                                  "/guard_test/drone1/mission/phase", "/guard_test/drone2/mission/phase"])
        def publish():
            one.publish(String(data="HOVER_AT_GOAL"))
            two.publish(String(data="DONE"))
        node.create_timer(0.2, publish)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            child.wait(timeout=3)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return 0
    elif mode == "launch":
        # Give this process the argv signature recognized as a recorder.
        child = subprocess.Popen([sys.executable, __file__, "bag", str(target),
                                  "ros2", "bag", "record", "--output", str(target)])
        def finish(_sig, _frame):
            (target / "ros_stopped").write_text(str(time.monotonic()))
            child.wait(timeout=3)
            sys.exit(0)
        signal.signal(signal.SIGINT, finish)
    elif mode == "bag":
        target.mkdir(parents=True, exist_ok=True)
        def finish(_sig, _frame):
            (target / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
            (target / "bag_stopped").write_text(str(time.monotonic()))
            sys.exit(0)
        signal.signal(signal.SIGINT, finish)
        (target / "bag_ready").touch()
    elif mode == "stubborn":
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        target.touch()
    elif mode == "hold":
        signal.signal(signal.SIGINT, lambda _sig, _frame: sys.exit(0))
        target.touch()
    else:
        raise ValueError(mode)
    while True:
        time.sleep(0.1)


if __name__ == "__main__":
    sys.exit(main())
