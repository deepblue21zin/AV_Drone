import numpy as np

from drone_cslam.data import (
    ExperimentDataset,
    KeyframeTrack,
    SampledVehicle,
    ScanRecord,
)
from drone_cslam.mapping import (
    build_occupancy_grid,
    corrected_scan_poses,
    grid_spec_from_config,
)
from drone_cslam.metrics import map_metrics


def _dataset():
    scans = [
        ScanRecord(
            timestamp=0.0,
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=5.0,
            ranges=np.array([2.0], dtype=float),
        ),
        ScanRecord(
            timestamp=1.0,
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=5.0,
            ranges=np.array([2.0], dtype=float),
        ),
    ]
    poses = np.array([[1.0, 1.0, 0.0], [2.0, 1.0, 0.0]])
    vehicle = SampledVehicle(
        name="drone1",
        scans=scans,
        scan_times=np.array([0.0, 1.0]),
        raw_poses=poses.copy(),
        known_poses=poses.copy(),
        gt_poses=poses.copy(),
        valid_source_indices=np.array([0, 1]),
        dropped_scan_count=0,
    )
    track = KeyframeTrack(
        name="drone1",
        scan_indices=np.array([0, 1]),
        times=np.array([0.0, 1.0]),
        raw_poses=poses.copy(),
        known_poses=poses.copy(),
        gt_poses=poses.copy(),
        progress=np.array([0.0, 1.0]),
        node_indices=np.array([0, 1]),
        scan_to_keyframe=np.array([0, 1]),
    )
    return ExperimentDataset(vehicles={"drone1": vehicle}, keyframes={"drone1": track})


def _config():
    return {
        "sampling": {"map_scan_stride": 1, "map_beam_stride": 1},
        "map": {
            "resolution_m": 0.5,
            "bounds": [0.0, 6.0, 0.0, 4.0],
            "free_log_odds": -0.4,
            "occupied_log_odds": 2.0,
            "min_log_odds": -4.0,
            "max_log_odds": 4.0,
            "occupied_probability": 0.65,
        },
    }


def test_grid_reprojection_marks_rays_and_endpoints():
    dataset = _dataset()
    config = _config()
    poses = corrected_scan_poses(dataset, "GT-reference")
    grid = build_occupancy_grid(dataset, poses, config)
    assert np.count_nonzero(grid == 100) == 2
    assert np.count_nonzero(grid == 0) > 0
    metrics = map_metrics(grid, grid, grid_spec_from_config(config))
    assert metrics["occupied_f1"] == 1.0
    assert metrics["occupied_chamfer_m"] == 0.0


def test_corrected_scan_poses_preserve_local_scan_to_keyframe_geometry():
    dataset = _dataset()
    dataset.keyframes["drone1"].scan_to_keyframe[:] = 0
    optimized = np.array([[10.0, 3.0, 0.0], [11.0, 3.0, 0.0]])
    corrected = corrected_scan_poses(dataset, "O-periodic", optimized)["drone1"]
    np.testing.assert_allclose(corrected[0], [10.0, 3.0, 0.0])
    np.testing.assert_allclose(corrected[1], [11.0, 3.0, 0.0])
