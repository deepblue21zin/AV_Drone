import copy
import json
from pathlib import Path

import numpy as np
import yaml

from drone_cslam.bag_reader import load_bag
from drone_cslam.dataset import build_dataset
from drone_cslam.offline_lidar_eval import run
from drone_cslam.place_recognition import retrieve_candidates, shape_descriptor, correction_disagreement
from drone_cslam.se2 import transform_points, compose, inverse
from test_bag_pipeline_integration import _write_synthetic_bag, _config


def config():
    full = yaml.safe_load((Path(__file__).parents[1] / "config/lidar_no_gt_full.yaml").read_text())
    base = _config()
    full["vehicles"] = base["vehicles"]
    full["sampling"] = base["sampling"]
    full["map"] = base["map"]
    full["evaluation"].update(base["evaluation"])
    full["retrieval"].update(minimum_travel_m=0, anchor_spacing_m=2, maximum_candidates=4)
    full["registration"]["minimum_submap_points"] = 3
    return full


def test_descriptor_rigid_transform_invariance():
    points = np.random.default_rng(123).normal(size=(150, 2))
    moved = transform_points(np.array([4, -7, .6]), points)
    np.testing.assert_allclose(shape_descriptor(points), shape_descriptor(moved))


def test_consistency_is_independent_of_world_origin():
    one = np.array([.3, -.1, .02])
    two = np.array([.4, .2, .025])
    reference = np.array([70, 6])
    original = correction_disagreement(one, two, reference)
    change_frame = np.array([1000, -700, .7])
    transformed = correction_disagreement(
        compose(compose(change_frame, one), inverse(change_frame)),
        compose(compose(change_frame, two), inverse(change_frame)),
        transform_points(change_frame, reference.reshape(1, 2))[0])
    np.testing.assert_allclose(original, transformed, atol=1e-9)


def test_gt_not_loaded_and_missing_gt_does_not_change_sampling_or_candidates(tmp_path):
    path = tmp_path / "bag"
    _write_synthetic_bag(path)
    cfg = config()
    no_gt = load_bag(str(path), cfg, include_ground_truth=False)
    assert all(not v.ground_truth for v in no_gt.values())
    with_gt = load_bag(str(path), cfg)
    a = build_dataset(no_gt, cfg, use_ground_truth=False)
    for v in with_gt.values():
        # Break truth support entirely. Observation preparation must ignore it.
        v.ground_truth = v.ground_truth[:1]
    b = build_dataset(with_gt, cfg, use_ground_truth=False)
    for name in a.vehicles:
        np.testing.assert_array_equal(a.vehicles[name].scan_times, b.vehicles[name].scan_times)
        np.testing.assert_array_equal(a.vehicles[name].raw_poses, b.vehicles[name].raw_poses)
        np.testing.assert_array_equal(a.keyframes[name].progress, b.keyframes[name].progress)
        assert np.isnan(a.vehicles[name].gt_poses).all()
    assert retrieve_candidates(a, cfg)[1] == retrieve_candidates(b, cfg)[1]
    poisoned = copy.deepcopy(a)
    for track in poisoned.keyframes.values():
        track.gt_poses[:] = 123456789
    assert retrieve_candidates(a, cfg)[1] == retrieve_candidates(poisoned, cfg)[1]


def test_no_candidates_falls_back_to_b0_and_still_reports(tmp_path):
    path = tmp_path / "bag"
    _write_synthetic_bag(path)
    cfg = config()
    cfg["retrieval"]["minimum_travel_m"] = 1000
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(cfg))
    output = tmp_path / "output"
    result = run(str(path), str(config_path), str(output))
    assert not result["phase2_gate"]["pass"]
    frozen = json.loads((output / "inference_frozen.json").read_text())
    assert frozen["gt_topics_loaded"] is False
    assert frozen["factors"] == []
    poses = np.load(output / "inference_poses.npz")
    np.testing.assert_array_equal(poses["B0"], poses["N-single"])
    assert (output / "overview_map_errors.png").exists()
    assert (output / "manifest.json").exists()
