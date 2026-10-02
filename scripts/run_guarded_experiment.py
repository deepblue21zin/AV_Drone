#!/usr/bin/env python3
"""Run one existing, stopped AV_Drone sim/ROS pair; never recreate or delete it."""

import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

from runtime_guard import PROJECT, load_limits, read_json, storage_problem, write_json

CONTAINER_PROJECT = Path("/workspace/AV_Drone")


def docker(*args, check=True, timeout=30):
    return subprocess.run(["docker", *args], check=check, timeout=timeout,
                          text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def environment(container):
    return dict(item.split("=", 1) for item in container["Config"]["Env"] if "=" in item)


def validate_container(container):
    mounts = [m for m in container["Mounts"] if m["Destination"] == str(CONTAINER_PROJECT)]
    if len(mounts) != 1 or mounts[0]["Type"] != "bind" or not mounts[0]["RW"] or Path(mounts[0]["Source"]).resolve() != PROJECT:
        raise ValueError("Container does not have this project's writable bind mount")
    if container["State"]["Status"] not in ("exited", "created"):
        raise ValueError(f"{container['Name']} is not stopped; refusing to interrupt existing work")
    if container.get("HostConfig", {}).get("AutoRemove"):
        raise ValueError("AutoRemove containers are not supported (preserve existing data)")


def scenario(args, sim):
    env = environment(sim)
    if args.single:
        if int(env.get("VEHICLE_COUNT", "1")) != 1 or int(env.get("PX4_INSTANCE", "0")) != 0:
            raise ValueError("Single launch requires existing sim VEHICLE_COUNT=1, PX4_INSTANCE=0")
        # Use installed config, exactly as single_drone_autonomy.launch.py does.
        config = PROJECT / "install/drone_bringup/share/drone_bringup/config/drone1_autonomy.yaml"
        params = yaml.safe_load(config.read_text())["/**"]["ros__parameters"]
        phase = "DONE" if params.get("return_home_enabled", True) else "HOVER_AT_GOAL"
        targets = {params.get("mission_phase_topic", "/drone1/mission/phase"): phase}
        return targets, ["ros2", "launch", "drone_bringup", "single_drone_autonomy.launch.py"]
    manifest = Path(args.manifest).resolve(strict=True)
    relative = manifest.relative_to(PROJECT)
    data = yaml.safe_load(manifest.read_text())
    vehicles = data["vehicles"]
    if len(vehicles) != 2 or data.get("vehicle_count") != 2 or int(env.get("VEHICLE_COUNT", "1")) != 2:
        raise ValueError("Multi launch requires exactly two manifest/simulator vehicles")
    if data["world_name"] != env.get("PX4_SITL_WORLD", "obstacle_demo"):
        raise ValueError("Manifest world differs from the existing sim container; review configuration first")
    base = int(env.get("PX4_INSTANCE_BASE", "2"))
    if [v["px4_instance"] for v in vehicles] != [base, base + 1]:
        raise ValueError("Manifest PX4 instances differ from the existing sim container")
    targets = {}
    for i, vehicle in enumerate(vehicles):
        if vehicle["name"] != f"drone{i+1}":
            raise ValueError("Existing multi simulator publishes drone1/drone2 only")
        actual_xy = [float(env.get("SWARM_SPAWN_X", "3.0")),
                     float(env.get(f"SWARM_SPAWN_Y_DRONE{i+1}", "-7.5" if i == 0 else "7.5"))]
        if any(abs(a - b) > 1e-6 for a, b in zip(actual_xy, vehicle["spawn"][:2])):
            raise ValueError("Manifest spawn differs from the existing sim container")
        returning = vehicle.get("return_home_enabled", data.get("return_home_enabled", True))
        targets[f"/{vehicle['name']}/mission/phase"] = "DONE" if returning else "HOVER_AT_GOAL"
    command = ["ros2", "launch", "drone_bringup", "multi_drone_slam_fusion.launch.py",
               f"manifest:={CONTAINER_PROJECT / relative}"]
    if args.fusion_source:
        command.append(f"fusion_source:={args.fusion_source}")
    return targets, command


def wait_ros_closed(run_dir, wait_sec):
    deadline = time.monotonic() + wait_sec
    while time.monotonic() < deadline:
        if (run_dir / "ros.result.json").exists():
            return True
        time.sleep(0.2)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--single", action="store_true")
    mode.add_argument("--manifest")
    parser.add_argument("--fusion-source", choices=("known_pose", "slam"))
    parser.add_argument("--sim-container", default="av_drone-sim-1")
    parser.add_argument("--ros-container", default="av_drone-ros-1")
    parser.add_argument("--check", action="store_true", help="Read-only preflight; do not start containers")
    args = parser.parse_args()
    limits = load_limits()
    problem = storage_problem(PROJECT, limits)
    if problem:
        raise RuntimeError(problem)
    if PROJECT.stat().st_dev == Path("/").stat().st_dev:
        raise RuntimeError("Project must be on the separate data filesystem (/home3 here)")
    sim, ros = json.loads(docker("inspect", args.sim_container, args.ros_container).stdout)
    for container in (sim, ros):
        validate_container(container)
    if sim["Id"] == ros["Id"]:
        raise ValueError("Sim and ROS must be different containers")
    if sim["Config"]["Cmd"] != ["/workspace/AV_Drone/docker/sim/entrypoint.sh"]:
        raise ValueError("Sim container must use this project's guarded entrypoint")
    if ros["Config"]["Cmd"] != ["sleep", "infinity"]:
        raise ValueError("ROS container must be the idle workspace container")
    for key, default in (("ROS_DOMAIN_ID", "42"), ("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")):
        if environment(sim).get(key, default) != environment(ros).get(key, default):
            raise ValueError(f"Container {key} values differ")
    targets, launch = scenario(args, sim)
    print(json.dumps({"preflight": "OK", "limits": limits, "terminal_phases": targets,
                      "launch": launch}, indent=2), flush=True)
    if args.check:
        return 0

    requested = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda number, frame: requested.append(f"host_signal:{number}"))
    started_ids = []
    run_dir = None
    ros_client = None
    output = None
    reason = "host_startup_failure"
    exit_code = 1
    cleanup_ok = True
    try:
        started_at = time.time()
        # Track exact IDs, not names that could later refer to another container.
        started_ids.append(sim["Id"])
        docker("start", sim["Id"])
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and not requested:
            response = docker("exec", sim["Id"], "cat", "/run/av_drone_runtime.json", check=False)
            if response.returncode == 0:
                session = json.loads(response.stdout)
                if session["started_at"] >= started_at:
                    container_run = Path(session["run_dir"])
                    relative = container_run.relative_to(CONTAINER_PROJECT / "runtime/sim")
                    if len(relative.parts) != 1:
                        raise ValueError("Unsafe session path in sim descriptor")
                    run_dir = PROJECT / "runtime/sim" / relative
                    if not (run_dir / "session.json").is_file():
                        raise RuntimeError("Run directory is not shared with host")
                    break
            time.sleep(0.5)
        if run_dir is None or requested:
            raise RuntimeError("Simulator guard did not become available or start was cancelled")
        write_json(run_dir / "mission.policy.json", {"targets": targets})
        # Create the new artifact root as the host user, with inherited group
        # ownership, before root-owned ROS nodes create their output children.
        artifact_run = PROJECT / "artifacts" / run_dir.name
        artifact_run.mkdir(mode=0o2770, parents=True, exist_ok=False)
        artifact_run.chmod(0o2770)
        if args.manifest:
            launch.append(f"run_id:={run_dir.name}")
        started_ids.append(ros["Id"])
        docker("start", ros["Id"])
        # Source ROS in a shell, pass all user/config arguments as argv (not interpolation).
        inner = ["python3", str(CONTAINER_PROJECT / "scripts/runtime_guard.py"), "ros",
                 "--run-dir", str(container_run), "--mission", str(container_run / "mission.policy.json"),
                 "--", *launch]
        output = (run_dir / "ros_console.log").open("w")
        ros_client = subprocess.Popen(
            ["docker", "exec", "--workdir", str(CONTAINER_PROJECT), ros["Id"], "bash", "-c",
             'set -e; source /opt/ros/humble/setup.bash; source install/setup.bash; exec "$@"',
             "guarded_ros", *inner], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        print(f"Run directory: {run_dir}\nROS console: {run_dir / 'ros_console.log'}", flush=True)
        while True:
            if requested:
                reason = requested[0]
                break
            result = read_json(run_dir / "ros.result.json")
            if result:
                reason = result["reason"]
                exit_code = 0 if result["clean_shutdown"] and reason == "mission_complete" else 1
                break
            if ros_client.poll() is not None:
                reason = f"ros_exec_exit:{ros_client.returncode}"
                break
            if (run_dir / "stop.request.json").exists():
                reason = read_json(run_dir / "stop.request.json")["reason"]
                break
            problem = storage_problem(PROJECT, limits)
            if problem:
                reason = problem
                break
            if time.time() - started_at >= limits["max_runtime_sec"]:
                reason = "max_runtime"
                break
            time.sleep(0.5)
    finally:
        print(f"[experiment] shutdown: {reason}", flush=True)
        if run_dir is not None:
            try:
                write_json(run_dir / "stop.request.json", {"reason": reason, "source": "host"})
                if ros_client is not None and ros_client.poll() is None:
                    grace = limits["bag_grace_sec"] + limits["ros_grace_sec"] + limits["term_grace_sec"] + 10
                    if not wait_ros_closed(run_dir, grace):
                        print("WARNING: ROS shutdown report missing; bag closure is NOT confirmed", file=sys.stderr)
                        cleanup_ok = False
                        exit_code = 1
            except OSError as exc:
                print(f"WARNING: shutdown request could not be saved: {exc}", file=sys.stderr)
                cleanup_ok = False
                exit_code = 1
        # No docker rm, compose down, prune, or container recreation here.
        for container in (ros, sim):
            if container["Id"] not in started_ids:
                continue
            try:
                grace = 60 if container is ros else 90
                result = docker("stop", "--time", str(grace), container["Id"], check=False, timeout=grace + 15)
                if result.returncode:
                    print(f"WARNING: stop failed: {result.stderr}", file=sys.stderr)
                    cleanup_ok = False
                    exit_code = 1
            except (OSError, subprocess.TimeoutExpired) as exc:
                print(f"WARNING: container cleanup needs attention: {exc}", file=sys.stderr)
                cleanup_ok = False
                exit_code = 1
        if ros_client is not None:
            try:
                ros_client.wait(timeout=5)
            except subprocess.TimeoutExpired:
                ros_client.terminate()  # Docker client only, after container stop was attempted.
                cleanup_ok = False
                exit_code = 1
        if output is not None:
            output.close()
        if run_dir is not None:
            sim_result = read_json(run_dir / "sim.result.json")
            ros_result = read_json(run_dir / "ros.result.json")
            # A stop request can precede the result while the bag is flushing.
            if cleanup_ok and reason == "mission_complete" and ros_result.get("clean_shutdown") and sim_result.get("clean_shutdown"):
                exit_code = 0
            if not sim_result.get("clean_shutdown") or not ros_result.get("clean_shutdown"):
                exit_code = 1
            write_json(run_dir / "host.result.json", {"reason": reason, "exit_code": exit_code,
                       "container_cleanup_ok": cleanup_ok,
                       "sim_shutdown_confirmed": bool(sim_result.get("clean_shutdown")),
                       "ros_result": ros_result})
    return exit_code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"[experiment] REFUSED/FAILED: {exc}", file=sys.stderr)
        if isinstance(exc, subprocess.CalledProcessError):
            print(exc.stderr, file=sys.stderr)
        sys.exit(2)
