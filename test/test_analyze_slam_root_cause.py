import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "analyze_slam_root_cause.py"
)
SPEC = importlib.util.spec_from_file_location("analyze_slam_root_cause", MODULE_PATH)
analyzer = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(analyzer)


def test_alignment_metrics_reports_thresholds_and_percentiles():
    reference = np.asarray([[0.0, 0.0], [1.0, 0.0]])
    query = np.asarray([[0.1, 0.0], [1.6, 0.0]])

    result = analyzer.alignment_metrics(query, reference)

    assert result["within_0p3_pct"] == 50.0
    assert result["within_0p5_pct"] == 50.0
    assert result["within_1p0_pct"] == 100.0
    assert np.isclose(result["median_distance_m"], 0.35)


def test_nearest_distances_handles_empty_query():
    result = analyzer.nearest_distances(
        np.empty((0, 2)), np.asarray([[0.0, 0.0]])
    )

    assert result.size == 0
