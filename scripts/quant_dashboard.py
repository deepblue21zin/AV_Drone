#!/usr/bin/env python3
"""Read-only Streamlit dashboard for A* vs MPPI path-planning quantification."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any


DEFAULT_METRICS = [
    "success",
    "mission_time_s",
    "actual_path_length_m",
    "path_efficiency",
    "min_obstacle_distance_m",
    "safety_intervention_count",
    "control_effort",
    "planning_time_ms_p95",
    "replan_count",
]

KPI_GUIDE = [
    {
        "kpi": "success",
        "direction": "higher is better",
        "paper_use": "Goal reach success rate by planner condition.",
        "target_note": "Primary KPI. Failed runs remain in the dataset.",
    },
    {
        "kpi": "mission_time_s",
        "direction": "lower is better",
        "paper_use": "Elapsed time from mission start to goal arrival.",
        "target_note": "Compare only under same world, start, goal, and seed group.",
    },
    {
        "kpi": "actual_path_length_m",
        "direction": "lower is better",
        "paper_use": "Measured flown trajectory length.",
        "target_note": "Computed from pose trajectory; includes real vehicle motion.",
    },
    {
        "kpi": "path_efficiency",
        "direction": "higher is better",
        "paper_use": "straight_line_distance_m / actual_path_length_m.",
        "target_note": "Use alongside safety; a short unsafe path is not a win.",
    },
    {
        "kpi": "min_obstacle_distance_m",
        "direction": "higher is better",
        "paper_use": "Closest obstacle distance observed during a run.",
        "target_note": "Safety KPI. Must be interpreted with LiDAR false positives.",
    },
    {
        "kpi": "safety_intervention_count",
        "direction": "lower is better",
        "paper_use": "Number of non-benign safety events.",
        "target_note": "High count indicates planner/safety disagreement.",
    },
    {
        "kpi": "control_effort",
        "direction": "lower is better",
        "paper_use": "Integrated command energy proxy.",
        "target_note": "Useful for stability/smoothness comparison.",
    },
    {
        "kpi": "planning_time_ms_p95",
        "direction": "lower is better",
        "paper_use": "Planner compute latency tail.",
        "target_note": "A* and MPPI should publish this once implemented.",
    },
    {
        "kpi": "replan_count",
        "direction": "context dependent",
        "paper_use": "How often the planner rebuilt the path/control plan.",
        "target_note": "Not always bad; interpret with safety and time.",
    },
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", default=".", help="Repository root directory.")
    parser.add_argument(
        "--check-data",
        action="store_true",
        help="Validate and summarize available quantification data without importing Streamlit.",
    )
    return parser.parse_args()


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", newline="") as handle:
        return list(csv.DictReader(handle))


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def as_float(value: Any) -> float | None:
    if value in {None, ""}:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if value in {None, ""}:
        return None
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "pass", "success"}:
        return True
    if text in {"false", "0", "no", "fail", "failure"}:
        return False
    return None


def first_non_empty(*values: Any, default: str = "") -> Any:
    for value in values:
        if value not in {None, "", "unknown"}:
            return value
    return default


def artifact_dirs(repo_root: Path) -> list[Path]:
    artifacts_root = repo_root / "artifacts"
    if not artifacts_root.exists():
        return []
    return sorted(
        [path for path in artifacts_root.glob("*_drone*") if path.is_dir()],
        key=lambda item: item.name,
    )


def load_artifact_records(repo_root: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for artifact_dir in artifact_dirs(repo_root):
        metadata = read_json(artifact_dir / "metadata.json")
        summary = read_json(artifact_dir / "summary.json")
        paper = read_json(artifact_dir / "paper_metrics.json")
        phase = read_json(artifact_dir / "phase_summary.json")
        slam = read_json(artifact_dir / "slam_summary.json")

        run_id = first_non_empty(
            paper.get("run_id"),
            metadata.get("run_id"),
            summary.get("run_id"),
            artifact_dir.name,
        )
        condition_id = first_non_empty(
            paper.get("condition_id"),
            paper.get("condition"),
            metadata.get("condition_id"),
            metadata.get("experiment_condition"),
            summary.get("condition_id"),
            summary.get("experiment_condition"),
            default="unknown",
        )
        scenario_id = first_non_empty(
            paper.get("scenario_id"),
            paper.get("scenario"),
            metadata.get("scenario_id"),
            metadata.get("scenario_name"),
            summary.get("scenario_id"),
            summary.get("scenario_name"),
            default="unknown",
        )
        success = as_bool(first_non_empty(paper.get("success"), paper.get("return_success")))
        if success is None:
            success = as_bool(summary.get("goal_reached"))

        mission_time = as_float(
            first_non_empty(
                paper.get("mission_time_s"),
                paper.get("outbound_time_s"),
                summary.get("outbound_time_s"),
                paper.get("runtime_s"),
                summary.get("runtime_s"),
            )
        )
        actual_path_length = as_float(
            first_non_empty(
                paper.get("actual_path_length_m"),
                paper.get("outbound_path_length_m"),
                paper.get("total_path_length_m"),
                summary.get("outbound_path_length_m"),
                summary.get("total_path_length_m"),
            )
        )
        straight_line_distance = as_float(paper.get("straight_line_distance_m"))
        planned_path_length = as_float(paper.get("planned_path_length_m"))
        path_efficiency = as_float(paper.get("path_efficiency"))
        if path_efficiency is None and actual_path_length:
            if planned_path_length:
                path_efficiency = planned_path_length / actual_path_length
            elif straight_line_distance:
                path_efficiency = straight_line_distance / actual_path_length

        return_path_length = as_float(paper.get("return_path_length_m"))
        straight_home_distance = as_float(paper.get("straight_line_home_distance_m"))
        return_efficiency = as_float(paper.get("return_path_efficiency"))
        if return_efficiency is None and return_path_length and straight_home_distance:
            return_efficiency = straight_home_distance / return_path_length

        record = {
            "run_id": run_id,
            "artifact_path": str(artifact_dir),
            "condition_id": condition_id,
            "scenario_id": scenario_id,
            "world_name": first_non_empty(
                paper.get("world_name"),
                metadata.get("world_name"),
                metadata.get("px4_gz_world"),
                summary.get("px4_gz_world"),
                default="unknown",
            ),
            "planner_name": first_non_empty(
                paper.get("planner_name"),
                metadata.get("planner_name"),
                summary.get("planner_name"),
                default="unknown",
            ),
            "planner_family": first_non_empty(
                paper.get("planner_family"),
                metadata.get("planner_family"),
                summary.get("planner_family"),
                default="unknown",
            ),
            "map_source": first_non_empty(
                paper.get("map_source"),
                metadata.get("map_source"),
                summary.get("map_source"),
                default="unknown",
            ),
            "seed": first_non_empty(
                paper.get("seed"),
                paper.get("experiment_seed"),
                metadata.get("experiment_seed"),
                summary.get("experiment_seed"),
                default="",
            ),
            "return_mode": first_non_empty(
                paper.get("return_mode"),
                metadata.get("return_mode"),
                summary.get("return_mode"),
                default="unknown",
            ),
            "mapping_enabled": first_non_empty(
                paper.get("mapping_enabled"),
                metadata.get("mapping_enabled"),
                default="",
            ),
            "started_at": first_non_empty(metadata.get("started_at"), summary.get("started_at")),
            "git_commit": first_non_empty(metadata.get("git_commit"), summary.get("git_commit")),
            "git_branch": first_non_empty(metadata.get("git_branch"), summary.get("git_branch")),
            "git_dirty": first_non_empty(metadata.get("git_dirty"), summary.get("git_dirty")),
            "success": success,
            "result": "pass" if success else "fail",
            "success_code": first_non_empty(paper.get("success_code"), summary.get("failure_code")),
            "failure_code": first_non_empty(paper.get("failure_code"), summary.get("failure_code")),
            "runtime_s": as_float(first_non_empty(paper.get("runtime_s"), summary.get("runtime_s"))),
            "mission_time_s": mission_time,
            "actual_path_length_m": actual_path_length,
            "straight_line_distance_m": straight_line_distance,
            "planned_path_length_m": planned_path_length,
            "path_efficiency": path_efficiency,
            "return_time_s": as_float(first_non_empty(paper.get("return_time_s"), summary.get("return_time_s"))),
            "return_path_length_m": return_path_length,
            "return_path_efficiency": return_efficiency,
            "total_path_length_m": as_float(
                first_non_empty(paper.get("total_path_length_m"), summary.get("total_path_length_m"))
            ),
            "min_obstacle_distance_m": as_float(
                first_non_empty(
                    paper.get("return_min_obstacle_distance_m"),
                    paper.get("min_obstacle_distance_m"),
                    summary.get("closest_obstacle_m"),
                )
            ),
            "safety_intervention_count": as_float(
                first_non_empty(
                    paper.get("safety_intervention_count"),
                    summary.get("safety_intervention_count"),
                    summary.get("safety_event_count"),
                )
            ),
            "control_effort": as_float(first_non_empty(paper.get("control_effort"), summary.get("control_effort"))),
            "planning_time_ms_p50": as_float(paper.get("planning_time_ms_p50")),
            "planning_time_ms_p95": as_float(paper.get("planning_time_ms_p95")),
            "replan_count": as_float(paper.get("replan_count")),
            "escape_count": as_float(first_non_empty(paper.get("escape_count"), summary.get("escape_count"))),
            "map_coverage": as_float(
                first_non_empty(
                    paper.get("map_coverage"),
                    paper.get("map_coverage_final"),
                    paper.get("slam_coverage"),
                    summary.get("slam_coverage"),
                    slam.get("coverage"),
                )
            ),
            "localization_ok_rate": as_float(paper.get("localization_ok_rate")),
            "mission_phase": first_non_empty(summary.get("mission_phase"), paper.get("mission_phase")),
            "rosbag_path": first_non_empty(paper.get("rosbag_path"), metadata.get("rosbag_path")),
            "paper_metrics_path": str(artifact_dir / "paper_metrics.json"),
            "summary_path": str(artifact_dir / "summary.json"),
            "trajectory_path": str(artifact_dir / "trajectory.csv"),
            "phase_summary_path": str(artifact_dir / "phase_summary.json"),
            "phase_count": len(phase.get("phase_timeline") or []),
        }
        records.append(record)
    return records


def load_registry_records(repo_root: Path) -> dict[str, list[dict[str, str]]]:
    experiments_dir = repo_root / "experiments"
    return {
        "index": read_csv_rows(experiments_dir / "index.csv"),
        "ledger": read_csv_rows(experiments_dir / "ledger.csv"),
        "scenario_table": read_csv_rows(experiments_dir / "scenario_table.csv"),
        "summary_table": read_csv_rows(experiments_dir / "paper_outputs" / "summary_table.csv"),
    }


def metric_summary(records: list[dict[str, Any]], metric_names: list[str]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        grouped[str(record.get("condition_id") or "unknown")].append(record)

    rows: list[dict[str, Any]] = []
    for condition_id, condition_records in sorted(grouped.items()):
        for metric in metric_names:
            if metric == "success":
                values = [1.0 if record.get("success") else 0.0 for record in condition_records]
            else:
                values = [
                    as_float(record.get(metric))
                    for record in condition_records
                    if as_float(record.get(metric)) is not None
                ]
            clean = [value for value in values if value is not None]
            if not clean:
                rows.append({
                    "condition_id": condition_id,
                    "metric": metric,
                    "runs": len(condition_records),
                    "valid": 0,
                    "mean": "",
                    "std": "",
                    "min": "",
                    "max": "",
                })
                continue
            mean = sum(clean) / len(clean)
            variance = sum((value - mean) ** 2 for value in clean) / len(clean)
            rows.append({
                "condition_id": condition_id,
                "metric": metric,
                "runs": len(condition_records),
                "valid": len(clean),
                "mean": mean,
                "std": math.sqrt(variance),
                "min": min(clean),
                "max": max(clean),
            })
    return rows


def check_data(repo_root: Path) -> int:
    artifacts = load_artifact_records(repo_root)
    registry = load_registry_records(repo_root)
    print(f"repo_root: {repo_root}")
    print(f"artifact_records: {len(artifacts)}")
    print(f"registry_index_rows: {len(registry['index'])}")
    print(f"ledger_rows: {len(registry['ledger'])}")
    print(f"scenario_table_rows: {len(registry['scenario_table'])}")
    print(f"summary_table_rows: {len(registry['summary_table'])}")
    conditions = sorted({str(record.get("condition_id")) for record in artifacts})
    scenarios = sorted({str(record.get("scenario_id")) for record in artifacts})
    unknown_conditions = sum(1 for record in artifacts if record.get("condition_id") == "unknown")
    unknown_scenarios = sum(1 for record in artifacts if record.get("scenario_id") == "unknown")
    print(f"conditions: {', '.join(conditions) if conditions else '-'}")
    print(f"scenarios: {', '.join(scenarios) if scenarios else '-'}")
    print(f"unknown_condition_records: {unknown_conditions}")
    print(f"unknown_scenario_records: {unknown_scenarios}")
    return 0


def render_dashboard(repo_root: Path) -> None:
    try:
        import pandas as pd
        import streamlit as st
    except Exception as exc:
        raise SystemExit(
            "Streamlit dashboard dependencies are missing. Install with: "
            "python3 -m pip install -r requirements-dashboard.txt"
        ) from exc

    st.set_page_config(page_title="AV_Drone A* vs MPPI Quantification", layout="wide")
    st.title("AV_Drone A* vs MPPI Path Planning Quantification")
    st.caption(
        "Read-only dashboard for comparing graph-search shortest-path planning "
        "and sampling-based MPPI control under matched world, start, goal, and seed conditions."
    )

    artifacts = load_artifact_records(repo_root)
    registry = load_registry_records(repo_root)
    df = pd.DataFrame(artifacts)

    with st.sidebar:
        st.header("Data")
        st.write(f"Repo root: `{repo_root}`")
        st.write(f"Artifact runs: `{len(artifacts)}`")
        if st.button("Refresh"):
            st.rerun()

        if df.empty:
            st.warning("No artifact records found.")
            return

        scenarios = sorted(df["scenario_id"].dropna().unique().tolist())
        conditions = sorted(df["condition_id"].dropna().unique().tolist())
        results = sorted(df["result"].dropna().unique().tolist())
        selected_scenarios = st.multiselect("Scenario", scenarios, default=scenarios)
        selected_conditions = st.multiselect("Condition", conditions, default=conditions)
        selected_results = st.multiselect("Result", results, default=results)
        run_query = st.text_input("Run ID contains", "")

    filtered = df[
        df["scenario_id"].isin(selected_scenarios)
        & df["condition_id"].isin(selected_conditions)
        & df["result"].isin(selected_results)
    ].copy()
    if run_query:
        filtered = filtered[filtered["run_id"].astype(str).str.contains(run_query, case=False, na=False)]

    total_runs = len(filtered)
    success_rate = float(filtered["success"].fillna(False).mean() * 100.0) if total_runs else 0.0
    mean_mission_time = filtered["mission_time_s"].dropna().mean() if "mission_time_s" in filtered else math.nan
    mean_path_length = filtered["actual_path_length_m"].dropna().mean() if "actual_path_length_m" in filtered else math.nan

    kpi1, kpi2, kpi3, kpi4 = st.columns(4)
    kpi1.metric("Runs", total_runs)
    kpi2.metric("Success rate", f"{success_rate:.1f}%")
    kpi3.metric("Mean mission time", "-" if math.isnan(mean_mission_time) else f"{mean_mission_time:.2f}s")
    kpi4.metric("Mean path length", "-" if math.isnan(mean_path_length) else f"{mean_path_length:.2f}m")

    unknown_conditions = int((df["condition_id"] == "unknown").sum())
    unknown_scenarios = int((df["scenario_id"] == "unknown").sum())
    if unknown_conditions or unknown_scenarios:
        st.warning(
            "Some historical artifacts do not contain quantification schema fields: "
            f"unknown condition={unknown_conditions}, unknown scenario={unknown_scenarios}. "
            "Future A*-vs-MPPI runs should write condition_id/scenario_id into metadata and paper_metrics."
        )

    tabs = st.tabs(["Overview", "Compare", "Trajectory Overlay", "Runs", "Ledger", "Artifacts"])

    with tabs[0]:
        st.subheader("KPI 기준")
        st.dataframe(pd.DataFrame(KPI_GUIDE), use_container_width=True)
        st.subheader("Condition counts")
        if not filtered.empty:
            counts = filtered.groupby(["scenario_id", "condition_id", "result"]).size().reset_index(name="runs")
            st.dataframe(counts, use_container_width=True)
        st.subheader("Existing registry tables")
        c1, c2 = st.columns(2)
        c1.write("experiments/index.csv")
        c1.dataframe(pd.DataFrame(registry["index"]), use_container_width=True)
        c2.write("experiments/scenario_table.csv")
        c2.dataframe(pd.DataFrame(registry["scenario_table"]), use_container_width=True)

    with tabs[1]:
        st.subheader("Metric summary by condition")
        metrics = st.multiselect("Metrics", DEFAULT_METRICS, default=DEFAULT_METRICS[:6])
        summary_rows = metric_summary(filtered.to_dict("records"), metrics)
        summary_df = pd.DataFrame(summary_rows)
        st.dataframe(summary_df, use_container_width=True)
        numeric_summary = summary_df[summary_df["mean"] != ""].copy()
        if not numeric_summary.empty:
            selected_metric = st.selectbox("Chart metric", sorted(numeric_summary["metric"].unique()))
            chart_df = numeric_summary[numeric_summary["metric"] == selected_metric][["condition_id", "mean"]]
            chart_df = chart_df.set_index("condition_id")
            st.bar_chart(chart_df)

    with tabs[2]:
        st.subheader("Trajectory overlay")
        if filtered.empty:
            st.info("No runs match the current filter.")
        else:
            default_runs = filtered["run_id"].astype(str).tail(6).tolist()
            selected_runs = st.multiselect(
                "Runs to overlay",
                filtered["run_id"].astype(str).tolist(),
                default=default_runs,
            )
            trajectory_frames = []
            for _, record in filtered[filtered["run_id"].astype(str).isin(selected_runs)].iterrows():
                trajectory_path = Path(str(record.get("trajectory_path", "")))
                if not trajectory_path.exists():
                    continue
                try:
                    frame = pd.read_csv(trajectory_path)
                except Exception:
                    continue
                if frame.empty or not {"x", "y"}.issubset(frame.columns):
                    continue
                frame = frame.copy()
                frame["run_id"] = str(record["run_id"])
                frame["condition_id"] = str(record["condition_id"])
                frame["planner_name"] = str(record.get("planner_name", "unknown"))
                trajectory_frames.append(frame)

            if not trajectory_frames:
                st.warning("Selected runs do not have readable trajectory.csv files.")
            else:
                trajectory_df = pd.concat(trajectory_frames, ignore_index=True)
                max_points = 5000
                if len(trajectory_df) > max_points:
                    stride = max(1, len(trajectory_df) // max_points)
                    trajectory_df = trajectory_df.iloc[::stride].copy()

                chart_df = trajectory_df[["x", "y", "condition_id", "run_id"]].dropna()
                try:
                    import altair as alt

                    chart = (
                        alt.Chart(chart_df)
                        .mark_line(point=False)
                        .encode(
                            x=alt.X("x:Q", title="x [m]"),
                            y=alt.Y("y:Q", title="y [m]"),
                            color=alt.Color("condition_id:N", title="Condition"),
                            detail="run_id:N",
                            tooltip=["run_id", "condition_id", "x", "y"],
                        )
                        .properties(height=520)
                        .interactive()
                    )
                    st.altair_chart(chart, use_container_width=True)
                except Exception:
                    st.scatter_chart(chart_df, x="x", y="y", color="condition_id")

                st.dataframe(
                    trajectory_df[
                        [
                            col
                            for col in [
                                "run_id",
                                "condition_id",
                                "mission_phase",
                                "t_sec",
                                "x",
                                "y",
                                "z",
                                "speed_mps",
                                "nearest_obstacle_m",
                            ]
                            if col in trajectory_df.columns
                        ]
                    ].tail(500),
                    use_container_width=True,
                )

    with tabs[3]:
        st.subheader("Runs")
        display_cols = [
            "run_id",
            "scenario_id",
            "condition_id",
            "planner_name",
            "planner_family",
            "map_source",
            "result",
            "seed",
            "mission_time_s",
            "actual_path_length_m",
            "planned_path_length_m",
            "path_efficiency",
            "min_obstacle_distance_m",
            "safety_intervention_count",
            "control_effort",
            "planning_time_ms_p95",
            "replan_count",
            "artifact_path",
        ]
        st.dataframe(filtered[[col for col in display_cols if col in filtered.columns]], use_container_width=True)

    with tabs[4]:
        st.subheader("Ledger")
        st.dataframe(pd.DataFrame(registry["ledger"]), use_container_width=True)

    with tabs[5]:
        st.subheader("Artifact drill-down")
        if filtered.empty:
            st.info("No runs match the current filter.")
            return
        selected_run = st.selectbox("Run", filtered["run_id"].astype(str).tolist())
        record = filtered[filtered["run_id"].astype(str) == selected_run].iloc[0].to_dict()
        st.json(record)
        artifact_path = Path(str(record["artifact_path"]))
        for label, filename in [
            ("paper_metrics.json", "paper_metrics.json"),
            ("summary.json", "summary.json"),
            ("metadata.json", "metadata.json"),
            ("phase_summary.json", "phase_summary.json"),
        ]:
            with st.expander(label):
                st.json(read_json(artifact_path / filename))


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    if args.check_data:
        return check_data(repo_root)
    render_dashboard(repo_root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
