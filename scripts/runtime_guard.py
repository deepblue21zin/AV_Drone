#!/usr/bin/env python3
"""Local SITL/ROS safeguards. Never delete logs or modify PX4 parameters."""

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

PROJECT = Path(__file__).resolve().parents[1]
GIB = 1024 ** 3
LIMITS = PROJECT / "docker/runtime/limits.json"
DESCRIPTOR = Path("/run/av_drone_runtime.json")


def load_limits(path=LIMITS):
    values = json.loads(Path(path).read_text())
    required = {"root_min_free_gib", "project_min_free_gib", "max_log_gib",
                "max_runtime_sec", "poll_sec", "bag_grace_sec", "ros_grace_sec",
                "sim_grace_sec", "term_grace_sec"}
    if set(values) != required:
        raise ValueError("limits.json has missing or unknown settings")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
           not math.isfinite(v) or v <= 0 for v in values.values()):
        raise ValueError("All limits must be finite positive numbers")
    return values


def write_json(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    temporary.replace(path)


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except FileNotFoundError:
        return {}


def storage_problem(project, limits, disk_usage=shutil.disk_usage):
    for path, key in ((Path("/"), "root_min_free_gib"),
                      (project, "project_min_free_gib")):
        free = disk_usage(path).free / GIB
        if free < limits[key]:
            return f"low_space:{path}:free={free:.2f}GiB:min={limits[key]}GiB"
    return None


def tree_bytes(root):
    """Only this run's regular files; do not traverse links to other runs."""
    total = 0
    for directory, _, files in os.walk(root, followlinks=False):
        for name in files:
            try:
                path = Path(directory) / name
                if not path.is_symlink():
                    total += path.stat().st_size
            except FileNotFoundError:
                pass  # A logger may rotate a file during the scan.
    return total


def limit_problem(project, run_dir, limits, elapsed):
    problem = storage_problem(project, limits)
    if problem:
        return problem
    if elapsed >= limits["max_runtime_sec"]:
        return "max_runtime"
    if tree_bytes(run_dir) >= limits["max_log_gib"] * GIB:
        return "max_log_size"
    return None


def shared_directory(path, project=PROJECT):
    """Use the bind mount owner's group so host user can manage new logs."""
    path.mkdir(parents=True, exist_ok=True)
    if os.geteuid() == 0:
        owner = project.stat()
        os.chown(path, owner.st_uid, owner.st_gid)
    path.chmod(0o2770)
    return path


def new_run(project=PROJECT):
    # Fail closed if the project is accidentally in the Docker writable layer.
    if project.stat().st_dev == Path("/").stat().st_dev:
        raise RuntimeError("Project must be mounted on a separate data filesystem")
    parent = shared_directory(project / "runtime", project)
    parent = shared_directory(parent / "sim", project)
    name = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "_" + uuid.uuid4().hex[:12]
    run_dir = shared_directory(parent / name, project)
    for child in ("px4", "ros", "gazebo"):
        shared_directory(run_dir / child, project)
    return run_dir


def validated_run(path, project=PROJECT):
    run_dir = Path(path).resolve(strict=True)
    parent = (project / "runtime/sim").resolve(strict=True)
    if run_dir.parent != parent or not (run_dir / "session.json").is_file():
        raise ValueError("Not an AV_Drone runtime session directory")
    return run_dir


def redirect_path(source, destination):
    """Rename old data in place, never overwrite it or follow an old symlink."""
    source = Path(source)
    destination = Path(destination)
    source.parent.mkdir(parents=True, exist_ok=True)
    if source.is_symlink() and source.resolve() == destination.resolve():
        return
    if os.path.lexists(source):
        archived = source.with_name(source.name + ".pre_guard_" + uuid.uuid4().hex)
        source.rename(archived)
        print(f"[runtime guard] preserved {source} as {archived}", flush=True)
    source.symlink_to(destination, target_is_directory=destination.is_dir())


def prepare_px4(working_dir, instance, run_dir):
    destination = shared_directory(run_dir / "px4" / f"instance_{instance}")
    shared_directory(destination / "log")
    redirect_path(working_dir / "log", destination / "log")
    for name in ("px4_stdout.log", "px4_stderr.log"):
        redirect_path(working_dir / name, destination / name)


def processes():
    result = {}
    for path in Path("/proc").iterdir():
        if not path.name.isdigit():
            continue
        try:
            fields = (path / "stat").read_text().rsplit(")", 1)[1].split()
            argv = (path / "cmdline").read_bytes().decode(errors="replace").split("\0")
            result[int(path.name)] = {"state": fields[0], "ppid": int(fields[1]),
                                      "group": int(fields[2]), "argv": argv}
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            pass
    return result


def live_group(group):
    return [pid for pid, info in processes().items()
            if info["group"] == group and info["state"] != "Z"]


def send_group(group, sig):
    try:
        os.killpg(group, sig)
    except ProcessLookupError:
        pass


def stop_group(child, grace, term_grace):
    forced = False
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, term_grace),
                      (signal.SIGKILL, 2)):
        child.poll()
        if not live_group(child.pid):
            break
        forced |= sig != signal.SIGINT
        send_group(child.pid, sig)
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline and live_group(child.pid):
            child.poll()
            time.sleep(0.1)
    child.poll()
    return {"forced": forced, "remaining_pids": live_group(child.pid)}


def bag_recorders(group):
    bags = []
    for pid, info in processes().items():
        args = [arg for arg in info["argv"] if arg]
        if info["group"] != group or info["state"] == "Z":
            continue
        is_recorder = any(Path(arg).name == "ros2" and args[i+1:i+3] == ["bag", "record"]
                          for i, arg in enumerate(args))
        if not is_recorder:
            continue
        output = None
        for i, arg in enumerate(args):
            if arg in ("-o", "--output") and i + 1 < len(args):
                output = args[i+1]
            elif arg.startswith("--output="):
                output = arg.split("=", 1)[1]
        if output and not Path(output).is_absolute():
            output = str(Path(f"/proc/{pid}/cwd").resolve() / output)
        bags.append({"pid": pid, "output": output})
    return bags


def close_bags(group, grace):
    bags = bag_recorders(group)
    for bag in bags:
        try:
            os.kill(bag["pid"], signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline:
        alive = set(live_group(group))
        if not any(b["pid"] in alive for b in bags):
            break
        time.sleep(0.1)
    alive = set(live_group(group))
    for bag in bags:
        bag["closed_before_ros_stop"] = bag["pid"] not in alive
    return bags


def supervise(role, command, run_dir=None, mission=None):
    os.umask(0o002)
    limits = load_limits()
    problem = storage_problem(PROJECT, limits)
    if problem:
        raise RuntimeError(problem)
    started = time.monotonic()
    if role == "sim":
        run_dir = new_run(PROJECT)
        write_json(run_dir / "session.json", {"started_at": time.time(), "run_dir": str(run_dir),
                   "limits": limits, "pid": os.getpid()})
        write_json(DESCRIPTOR, read_json(run_dir / "session.json"))
    else:
        run_dir = validated_run(run_dir, PROJECT)
        if read_json(run_dir / "ros.state.json").get("status") == "running":
            raise RuntimeError("This session already has a managed ROS process")
        if (run_dir / "stop.request.json").exists():
            raise RuntimeError("Simulation session is already stopping")
    env = dict(os.environ, AV_RUNTIME_GUARDED="1", AV_RUNTIME_RUN_DIR=str(run_dir),
               ROS_LOG_DIR=str(run_dir / "ros"), GAZEBO_LOG_PATH=str(run_dir / "gazebo"))
    requested = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda number, frame: requested.append(f"signal:{number}"))
    child = None
    watcher = None
    reason = "startup_failure"
    clean = True
    bags = []
    details = {}
    print(f"[runtime guard] {role}: {run_dir}; limit={limits['max_runtime_sec']}s", flush=True)
    write_json(run_dir / f"{role}.state.json", {"status": "running", "pid": os.getpid()})
    try:
        if mission:
            watcher = subprocess.Popen([sys.executable, str(PROJECT / "scripts/watch_mission_completion.py"),
                                        "--policy", mission], env=env, start_new_session=True)
        child = subprocess.Popen(command, env=env, start_new_session=True)
        while True:
            if requested:
                reason = requested[0]
                break
            stop = read_json(run_dir / "stop.request.json")
            if stop:
                reason = stop["reason"]
                break
            if child.poll() is not None:
                reason = f"command_exit:{child.returncode}"
                break
            if watcher is not None and watcher.poll() is not None:
                reason = "mission_complete" if watcher.returncode == 0 else "mission_watcher_failed"
                break
            problem = limit_problem(PROJECT, run_dir, limits, time.monotonic() - started)
            if problem:
                reason = problem
                break
            time.sleep(limits["poll_sec"])
    except Exception as exc:
        reason = f"supervisor_error:{exc}"
        clean = False
    finally:
        print(f"[runtime guard] {role}: stopping ({reason})", flush=True)
        try:
            write_json(run_dir / "stop.request.json", {"reason": reason, "source": role})
        except OSError as exc:
            print(f"[runtime guard] cannot persist stop request: {exc}", file=sys.stderr)
            clean = False
        if role == "sim":
            # Let a managed recorder close before taking away its data sources.
            deadline = time.monotonic() + limits["bag_grace_sec"] + limits["ros_grace_sec"] + limits["term_grace_sec"] + 10
            while read_json(run_dir / "ros.state.json").get("status") == "running":
                if time.monotonic() >= deadline:
                    clean = False
                    details["ros_wait_timed_out"] = True
                    break
                time.sleep(0.2)
        if child is not None:
            if role == "ros":
                try:
                    bags = close_bags(child.pid, limits["bag_grace_sec"])
                except OSError as exc:
                    # Always attempt ROS teardown, even if /proc or storage fails.
                    details["bag_close_error"] = str(exc)
                    clean = False
            stopped = stop_group(child, limits[f"{role}_grace_sec"], limits["term_grace_sec"])
            clean &= not stopped["forced"] and not stopped["remaining_pids"]
            details.update(stopped)
        if watcher is not None:
            stop_group(watcher, 2, 2)
        for bag in bags:
            bag["metadata_present"] = bool(bag["output"] and (Path(bag["output"]) / "metadata.yaml").is_file())
            clean &= bag["closed_before_ros_stop"] and bag["metadata_present"]
        result = {"reason": reason, "clean_shutdown": bool(clean), "bags": bags,
                  "elapsed_sec": time.monotonic() - started, **details}
        # A failed report must not prevent the actual process shutdown above.
        write_json(run_dir / f"{role}.result.json", result)
        write_json(run_dir / f"{role}.state.json", {"status": "stopped", **result})
    return 0 if clean and reason in ("mission_complete", "command_exit:0") else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("check")
    prep = sub.add_parser("prepare-px4")
    prep.add_argument("--working-dir", required=True, type=Path)
    prep.add_argument("--instance", required=True, type=int, choices=range(10))
    for role in ("sim", "ros"):
        item = sub.add_parser(role)
        item.add_argument("--run-dir")
        item.add_argument("--mission")
        item.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.action == "check":
        problem = storage_problem(PROJECT, load_limits())
        print(problem or "Disk preflight OK")
        return 1 if problem else 0
    if args.action == "prepare-px4":
        run_dir = validated_run(os.environ["AV_RUNTIME_RUN_DIR"])
        rootfs = Path("/opt/PX4-Autopilot/build/px4_sitl_default/rootfs")
        work = args.working_dir.resolve()
        if work not in (rootfs, rootfs / str(args.instance)):
            raise ValueError("Unexpected PX4 working directory")
        prepare_px4(work, args.instance, run_dir)
        return 0
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a command after -- is required")
    return supervise(args.action, command, args.run_dir, args.mission)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"[runtime guard] REFUSED: {exc}", file=sys.stderr)
        sys.exit(2)
