#!/usr/bin/env python3

import argparse
import csv
import json
import subprocess
import time
from collections import defaultdict
from pathlib import Path


INDEX_HEADERS = [
    "logged_at",
    "run_id",
    "started_at",
    "git_commit",
    "git_branch",
    "git_dirty",
    "scenario_name",
    "result",
    "runner",
    "artifact_path",
    "plot_dir",
    "runtime_s",
    "mission_phase",
    "goal_reached",
    "connected",
    "armed",
    "mode",
    "pose_count",
    "scan_count",
    "planner_cmd_count",
    "safe_cmd_count",
    "safety_event_count",
    "current_obstacle_m",
    "closest_obstacle_m",
    "pose_period_p99_s",
    "scan_period_p99_s",
    "issue",
    "fix",
    "notes",
]

LEDGER_HEADERS = [
    "logged_at",
    "run_id",
    "git_commit",
    "scenario_name",
    "result",
    "artifact_path",
    "mission_phase",
    "goal_reached",
    "issue",
    "fix",
    "notes",
]

SCENARIO_HEADERS = [
    "scenario_name",
    "total_runs",
    "pass_runs",
    "fail_runs",
    "other_runs",
    "pass_rate_pct",
    "latest_logged_at",
    "latest_result",
    "latest_commit",
    "latest_artifact",
]


def git_context(repo_root: Path):
    try:
        commit = subprocess.run(
            ["git", "-c", f"safe.directory={repo_root}", "rev-parse", "HEAD"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        branch = subprocess.run(
            ["git", "-c", f"safe.directory={repo_root}", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "-c", f"safe.directory={repo_root}", "status", "--porcelain"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
        return commit, branch, bool(dirty)
    except Exception:
        return "unknown", "unknown", True


def load_json(path: Path):
    if not path.exists():
        return {}
    return json.loads(path.read_text())


def read_csv(path: Path):
    if not path.exists():
        return []
    with path.open("r", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, headers, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        for row in rows:
            writer.writerow({header: row.get(header, "") for header in headers})


def markdown_table(headers, rows):
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(header, "")) for header in headers) + " |")
    return "\n".join(lines) + "\n"


def relative_to_repo(path: Path, repo_root: Path) -> str:
    try:
        return str(path.relative_to(repo_root))
    except ValueError:
        return str(path)


def infer_result(summary):
    if summary.get("goal_reached") is True and summary.get("mission_phase") == "HOVER_AT_GOAL":
        return "pass"
    if summary:
        return "artifact_only"
    return "unknown"


def build_row(repo_root: Path, artifact_path: Path | None, result: str, runner: str, scenario_override: str, issue: str, fix: str, notes: str):
    now = time.strftime("%Y-%m-%dT%H:%M:%S")
    git_commit, git_branch, git_dirty = git_context(repo_root)
    metadata = {}
    summary = {}
    if artifact_path:
        metadata = load_json(artifact_path / "metadata.json")
        summary = load_json(artifact_path / "summary.json")

    scenario_name = scenario_override or metadata.get("scenario_name") or summary.get("scenario_name") or "unknown"
    run_id = metadata.get("run_id") or summary.get("run_id") or (artifact_path.name if artifact_path else "")
    plot_dir = ""
    if artifact_path and (repo_root / "experiments" / "plots" / artifact_path.name).exists():
        plot_dir = relative_to_repo(repo_root / "experiments" / "plots" / artifact_path.name, repo_root)

    metadata_commit = metadata.get("git_commit", "")
    metadata_branch = metadata.get("git_branch", "")
    metadata_dirty = metadata.get("git_dirty", "")
    summary_commit = summary.get("git_commit", "")
    summary_branch = summary.get("git_branch", "")
    summary_dirty = summary.get("git_dirty", "")

    effective_commit = metadata_commit if metadata_commit not in {"", "unknown"} else summary_commit
    if effective_commit in {"", "unknown"}:
        effective_commit = git_commit

    effective_branch = metadata_branch if metadata_branch not in {"", "unknown"} else summary_branch
    if effective_branch in {"", "unknown"}:
        effective_branch = git_branch

    effective_dirty = metadata_dirty if metadata_dirty not in {"", "unknown"} else summary_dirty
    if effective_dirty in {"", "unknown"}:
        effective_dirty = git_dirty

    return {
        "logged_at": now,
        "run_id": run_id,
        "started_at": metadata.get("started_at", ""),
        "git_commit": effective_commit,
        "git_branch": effective_branch,
        "git_dirty": effective_dirty,
        "scenario_name": scenario_name,
        "result": result or infer_result(summary),
        "runner": runner,
        "artifact_path": relative_to_repo(artifact_path, repo_root) if artifact_path else "",
        "plot_dir": plot_dir,
        "runtime_s": summary.get("runtime_s", ""),
        "mission_phase": summary.get("mission_phase", ""),
        "goal_reached": summary.get("goal_reached", ""),
        "connected": summary.get("connected", ""),
        "armed": summary.get("armed", ""),
        "mode": summary.get("mode", ""),
        "pose_count": summary.get("pose_count", ""),
        "scan_count": summary.get("scan_count", ""),
        "planner_cmd_count": summary.get("planner_cmd_count", ""),
        "safe_cmd_count": summary.get("safe_cmd_count", ""),
        "safety_event_count": summary.get("safety_event_count", ""),
        "current_obstacle_m": summary.get("current_obstacle_m", ""),
        "closest_obstacle_m": summary.get("closest_obstacle_m", ""),
        "pose_period_p99_s": summary.get("pose_period_p99_s", ""),
        "scan_period_p99_s": summary.get("scan_period_p99_s", ""),
        "issue": issue,
        "fix": fix,
        "notes": notes,
    }


def upsert_index(rows, new_row, preserve_existing=False):
    artifact_path = new_row.get("artifact_path", "")
    if artifact_path:
        for idx, row in enumerate(rows):
            if row.get("artifact_path", "") == artifact_path:
                if preserve_existing:
                    merged = new_row.copy()
                    for key, old_value in row.items():
                        if old_value not in {"", "unknown"}:
                            merged[key] = old_value
                    rows[idx] = merged
                else:
                    rows[idx] = new_row
                return rows
    rows.append(new_row)
    rows.sort(key=lambda row: row.get("logged_at", ""))
    return rows


def build_scenario_rows(index_rows):
    groups = defaultdict(list)
    for row in index_rows:
        groups[row.get("scenario_name", "unknown")].append(row)

    scenario_rows = []
    for scenario_name, rows in sorted(groups.items()):
        rows = sorted(rows, key=lambda row: row.get("logged_at", ""))
        latest = rows[-1]
        pass_runs = sum(1 for row in rows if row.get("result") == "pass")
        fail_runs = sum(1 for row in rows if row.get("result") == "fail")
        other_runs = len(rows) - pass_runs - fail_runs
        scenario_rows.append({
            "scenario_name": scenario_name,
            "total_runs": len(rows),
            "pass_runs": pass_runs,
            "fail_runs": fail_runs,
            "other_runs": other_runs,
            "pass_rate_pct": round((pass_runs / len(rows)) * 100.0, 1) if rows else 0.0,
            "latest_logged_at": latest.get("logged_at", ""),
            "latest_result": latest.get("result", ""),
            "latest_commit": latest.get("git_commit", ""),
            "latest_artifact": latest.get("artifact_path", ""),
        })
    return scenario_rows


def write_markdown_outputs(experiments_dir: Path, index_rows, scenario_rows, ledger_rows):
    (experiments_dir / "index.md").write_text(markdown_table(INDEX_HEADERS, index_rows))
    (experiments_dir / "scenario_table.md").write_text(markdown_table(SCENARIO_HEADERS, scenario_rows))
    (experiments_dir / "ledger.md").write_text(markdown_table(LEDGER_HEADERS, ledger_rows))


def scan_artifacts(artifacts_root: Path):
    return sorted([path for path in artifacts_root.glob("*_drone1") if path.is_dir()])


def main():
    parser = argparse.ArgumentParser(description="Update experiment registry and ledger from artifacts or smoke test results.")
    parser.add_argument("--artifact", default="", help="Artifact directory to register.")
    parser.add_argument("--scan-artifacts", default="", help="Artifacts root to scan and rebuild from.")
    parser.add_argument("--result", default="", help="Run result, e.g. pass/fail/artifact_only.")
    parser.add_argument("--runner", default="manual", help="Originating runner, e.g. smoke_test_single_drone.")
    parser.add_argument("--scenario", default="", help="Override scenario name.")
    parser.add_argument("--issue", default="", help="Issue summary for the ledger.")
    parser.add_argument("--fix", default="", help="Fix summary for the ledger.")
    parser.add_argument("--notes", default="", help="Free-form notes.")
    parser.add_argument("--allow-missing-artifact", action="store_true", help="Allow registration without a valid artifact path.")
    args = parser.parse_args()

    repo_root = Path.cwd()
    experiments_dir = repo_root / "experiments"
    index_path = experiments_dir / "index.csv"
    scenario_path = experiments_dir / "scenario_table.csv"
    ledger_path = experiments_dir / "ledger.csv"

    index_rows = read_csv(index_path)
    ledger_rows = read_csv(ledger_path)

    if args.scan_artifacts:
        for artifact_dir in scan_artifacts(Path(args.scan_artifacts)):
            summary = load_json(artifact_dir / "summary.json")
            row = build_row(
                repo_root,
                artifact_dir,
                args.result or infer_result(summary),
                "artifact_scan",
                args.scenario,
                "",
                "",
                "rebuild from artifacts",
            )
            index_rows = upsert_index(index_rows, row, preserve_existing=True)
    else:
        artifact_path = Path(args.artifact) if args.artifact else None
        if artifact_path and not artifact_path.exists():
            if not args.allow_missing_artifact:
                raise FileNotFoundError(f"Artifact path does not exist: {artifact_path}")
            artifact_path = None
        if artifact_path is None and not args.allow_missing_artifact:
            raise ValueError("Either --artifact must exist or --allow-missing-artifact must be set")

        row = build_row(
            repo_root,
            artifact_path,
            args.result,
            args.runner,
            args.scenario,
            args.issue,
            args.fix,
            args.notes,
        )
        index_rows = upsert_index(index_rows, row)
        ledger_rows.append({
            "logged_at": row["logged_at"],
            "run_id": row["run_id"],
            "git_commit": row["git_commit"],
            "scenario_name": row["scenario_name"],
            "result": row["result"],
            "artifact_path": row["artifact_path"],
            "mission_phase": row["mission_phase"],
            "goal_reached": row["goal_reached"],
            "issue": args.issue,
            "fix": args.fix,
            "notes": args.notes,
        })

    scenario_rows = build_scenario_rows(index_rows)
    write_csv(index_path, INDEX_HEADERS, index_rows)
    write_csv(scenario_path, SCENARIO_HEADERS, scenario_rows)
    write_csv(ledger_path, LEDGER_HEADERS, ledger_rows)
    write_markdown_outputs(experiments_dir, index_rows, scenario_rows, ledger_rows)

    print(index_path)
    print(scenario_path)
    print(ledger_path)


if __name__ == "__main__":
    main()
