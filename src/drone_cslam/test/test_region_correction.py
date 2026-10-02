import copy
import json
from types import SimpleNamespace

import numpy as np
import yaml
import pytest

from drone_cslam import region_correction as module
from drone_cslam import place_recognition
from drone_cslam.offline_lidar_eval import run, complete_two_region_ablation
from test_no_gt_pipeline import config
from test_bag_pipeline_integration import _write_synthetic_bag


def fixture():
    cfg = config()
    cfg["experiment"] = {"correction_mode": "two_regions"}
    cfg["regions"] = {"minimum_progress_separation_m": 30., "minimum_world_separation_m": 25.}
    tracks = {}
    for n, item in enumerate(cfg["vehicles"]):
        poses = np.array([[20., n*5., 0], [26., n*5., 0],
                          [70., n*5., 0], [76., n*5., 0]])
        tracks[item["name"]] = SimpleNamespace(raw_poses=poses, node_indices=np.arange(4)+4*n,
                                              gt_poses=np.full((4, 3), np.nan))
    rows = []
    for i, progress in enumerate([20., 26., 70., 76.]):
        rows.append(dict(pair=i, source_keyframe=i, target_keyframe=i,
                         target_progress_m=progress, target_vehicle_progress_m=progress,
                         source_time=progress, target_time=progress+10,
                         geometry_pass=True, accepted=False, support_pairs=[i ^ 1],
                         overlap_ratio=.5 if i % 2 == 0 else .4, cost=.1,
                         estimated_pose=[0., 5., 0.], rejection_reasons=""))
    return cfg, SimpleNamespace(keyframes=tracks), rows


def test_independent_components_have_one_representative_and_ignore_gt():
    cfg, data, rows = fixture()
    selected, components = module.select_regions(rows, data, cfg)
    assert [c["representative"]["pair"] for c in selected] == [0, 2]
    assert len(components) == 2
    for track in data.keyframes.values():
        track.gt_poses[:] = 1e9
    again, _ = module.select_regions(list(reversed(rows)), data, cfg)
    assert [c["representative"]["pair"] for c in again] == [0, 2]


def test_same_place_revisited_does_not_count_as_spatially_separated_region():
    cfg, data, rows = fixture()
    for track in data.keyframes.values():
        track.raw_poses[2:, :2] = track.raw_poses[:2, :2]
    assert len(module.select_regions(rows, data, cfg)[0]) == 1


def test_two_ablation_factors_are_exactly_the_two_single_factors(monkeypatch):
    cfg, data, rows = fixture()
    monkeypatch.setattr(module, "estimate", lambda *a, **kw: dict(rows=copy.deepcopy(rows), results=[], consistent_pair_count=4))
    calls = []
    def optimize(initial, factors, priors, **kwargs):
        calls.append((initial.copy(), factors, priors, kwargs))
        return SimpleNamespace(poses=initial.copy(), success=True, evaluations=1, message="fixture")
    monkeypatch.setattr(module, "optimize_pose_graph", optimize)
    result = module.estimate_two_regions(data, cfg)
    opts = result["condition_optimizers"]
    assert [opts[c]["applied_factor_count"] for c in opts] == [0, 1, 1, 2]
    factors = result["condition_factors"]
    assert factors["N-double"][0] is factors["N-single-A"][0]
    assert factors["N-double"][1] is factors["N-single-B"][0]
    for initial, _, priors, kwargs in calls:
        np.testing.assert_array_equal(initial, calls[0][0])
        assert priors is calls[0][2]
        assert kwargs == calls[0][3]
    assert len([r for r in result["rows"] if r["accepted"]]) == 2


def test_missing_second_region_and_failed_optimizer_are_not_reported_as_two(monkeypatch):
    cfg, data, rows = fixture()
    monkeypatch.setattr(module, "estimate", lambda *a, **kw: dict(rows=copy.deepcopy(rows[:2]), results=[], consistent_pair_count=2))
    monkeypatch.setattr(module, "optimize_pose_graph", lambda initial, *a, **kw:
                        SimpleNamespace(poses=initial+100, success=False, evaluations=1, message="failed"))
    result = module.estimate_two_regions(data, cfg)
    opt = result["condition_optimizers"]
    assert not opt["N-double"]["available"]
    assert opt["N-double"]["accepted_factor_count"] == 1
    assert opt["N-double"]["applied_factor_count"] == 0
    assert not opt["N-single-B"]["available"]
    np.testing.assert_array_equal(result["poses"], module.initial_pose_array(data))


def test_two_region_report_preserves_missing_region_failure(tmp_path):
    bag = tmp_path / "bag"
    _write_synthetic_bag(bag)
    cfg, _, _ = fixture()
    cfg["retrieval"]["minimum_travel_m"] = 1000
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(cfg))
    output = tmp_path / "output"
    metrics = run(str(bag), str(path), str(output))
    assert not metrics["phase2_gate"]["pass"]
    assert not metrics["phase2_gate"]["checks"]["two_independent_regions_available"]
    frozen = json.loads((output / "inference_frozen.json").read_text())
    assert not frozen["gt_topics_loaded"]
    assert frozen["factors"] == []
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["primary_condition"] == "N-double"
    assert len(metrics["conditions"]) == 6
    assert (output / "maps/N-single-B.npy").exists()


def test_optional_spatial_retrieval_adds_missing_nearby_candidate_without_duplicates(monkeypatch):
    cfg = config()
    cfg["retrieval"].update(minimum_travel_m=0, anchor_spacing_m=1,
                            top_k_per_anchor=2, maximum_candidates=20)
    cfg["registration"]["minimum_submap_points"] = 1
    data = SimpleNamespace(keyframes={
        "a": SimpleNamespace(progress=np.array([0.]), raw_poses=np.array([[0., 0., 0.]])),
        "b": SimpleNamespace(progress=np.arange(3.), raw_poses=np.array([[0., 5., 0.], [0., -5., 0.], [0., 1., 0.]]))})
    monkeypatch.setattr(place_recognition, "build_local_submap", lambda data, name, i, cfg:
                        SimpleNamespace(points=np.array([[1. if name == "a" else [1., 1.1, 4.][i], 0.]])))
    monkeypatch.setattr(place_recognition, "shape_descriptor", lambda points: points[0])
    base = place_recognition.retrieve_candidates(data, cfg)[1]
    assert {r["target_keyframe"] for r in base} == {0, 1}
    cfg["retrieval"]["spatial_top_k_per_anchor"] = 2
    expanded = place_recognition.retrieve_candidates(data, cfg)[1]
    assert {r["target_keyframe"] for r in expanded} == {0, 1, 2}
    assert len(expanded) == 3
    assert set((r["source_keyframe"], r["target_keyframe"]) for r in base).issubset(
        {(r["source_keyframe"], r["target_keyframe"]) for r in expanded})


def test_complete_ablation_requires_both_single_runs_and_two_applied_factors():
    good = {name: dict(available=True, success=True, applied_factor_count=count)
            for name, count in [("N-single-A", 1), ("N-single-B", 1), ("N-double", 2)]}
    assert complete_two_region_ablation(good)
    for name in good:
        failed = copy.deepcopy(good)
        failed[name]["success"] = False
        assert not complete_two_region_ablation(failed)
    good["N-double"].update(applied_factor_count=1, accepted_factor_count=2)
    assert not complete_two_region_ablation(good)


def test_strict_ablation_stops_before_map_generation_and_gt_read(tmp_path, monkeypatch):
    from drone_cslam import offline_lidar_eval
    bag = tmp_path / "bag"
    _write_synthetic_bag(bag)
    cfg, _, _ = fixture()
    cfg["retrieval"]["minimum_travel_m"] = 1000
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(cfg))
    output = tmp_path / "output"
    original = offline_lidar_eval.load_bag
    def no_truth(*args, **kwargs):
        assert kwargs.get("include_ground_truth") is False
        return original(*args, **kwargs)
    def no_maps(*args, **kwargs):
        raise AssertionError("Incomplete strict ablation must not create maps")
    monkeypatch.setattr(offline_lidar_eval, "load_bag", no_truth)
    monkeypatch.setattr(offline_lidar_eval, "build_occupancy_grid", no_maps)
    with pytest.raises(RuntimeError, match="Two-region ablation incomplete"):
        run(str(bag), str(path), str(output), require_complete_ablation=True)
    assert not json.loads((output / "ablation_status.json").read_text())["complete"]
    assert (output / "inference_frozen.json").exists()
    assert (output / "source_snapshots/place_recognition.py").exists()
    assert not (output / "manifest.json").exists()
