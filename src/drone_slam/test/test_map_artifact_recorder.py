import json
import math
import tempfile
from pathlib import Path

import numpy as np
from nav_msgs.msg import OccupancyGrid

from drone_slam.map_artifact_recorder_node import MapArtifactRecorderNode


def test_write_bundle_preserves_grid_pose_and_snapshot_metadata():
    message = OccupancyGrid()
    message.header.frame_id = "drone1/map"
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 345
    message.info.resolution = 0.5
    message.info.width = 2
    message.info.height = 2
    message.info.origin.position.x = -1.0
    message.info.origin.position.y = 2.0
    message.info.origin.orientation.z = math.sin(math.pi / 8.0)
    message.info.origin.orientation.w = math.cos(math.pi / 8.0)
    message.data = [-1, 0, 50, 100]

    with tempfile.TemporaryDirectory() as temp_dir:
        output = Path(temp_dir)
        MapArtifactRecorderNode._write_bundle(
            message,
            output,
            "slam_0001",
            "slam",
            {"snapshot_index": 1, "elapsed_sec": 30.0},
        )
        grid = np.load(output / "slam_0001_grid.npy", allow_pickle=False)
        meta = json.loads((output / "slam_0001_meta.json").read_text())
        yaml_text = (output / "slam_0001.yaml").read_text()

    assert grid.tolist() == [[-1, 0], [50, 100]]
    assert meta["source"] == "slam"
    assert meta["snapshot_index"] == 1
    assert meta["map_stamp_sec"] == 12
    assert math.isclose(meta["origin"]["yaw"], math.pi / 4.0)
    assert "image: slam_0001.pgm" in yaml_text
