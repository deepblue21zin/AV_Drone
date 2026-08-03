#!/usr/bin/env python3
import argparse
import datetime as dt
import json
import signal
import subprocess
import sys
import time
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]


def log(message):
    print(f"[{dt.datetime.now().strftime('%H:%M:%S')}] {message}", flush=True)


def run(command, **kwargs):
    return subprocess.run(command, check=True, universal_newlines=True, **kwargs)


def docker_exec(container, shell_command, **kwargs):
    return run(["docker", "exec", container, "bash", "-lc", shell_command], **kwargs)


def cleanup_ros_processes(container):
    subprocess.run(
        [
            "docker",
            "exec",
            container,
            "bash",
            "-lc",
            "pkill -INT -f '[r]os2 bag record' 2>/dev/null || true; "
            "pkill -INT -f '[r]os2 launch mppi_lidar' 2>/dev/null || true",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def tail(path, lines=40):
    if not path.exists():
        return f"(missing: {path})"
    return "\n".join(path.read_text(errors="replace").splitlines()[-lines:])


def stop_process(process, name):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        log(f"{name}: SIGINT timeout; sending SIGTERM")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def wait_for_sim(container, vehicle_count, timeout):
    expected = [f"iris_rplidar_{index}" for index in range(vehicle_count)]
    deadline = time.monotonic() + timeout
    next_report = 0.0
    while time.monotonic() < deadline:
        found = []
        for model_name in expected:
            result = subprocess.run(
                [
                    "docker",
                    "exec",
                    container,
                    "gz",
                    "model",
                    "-m",
                    model_name,
                    "-i",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            if result.returncode == 0:
                found.append(model_name)
        if len(found) == vehicle_count:
            log(f"시뮬레이터 준비 완료: {', '.join(expected)}")
            return
        now = time.monotonic()
        if now >= next_report:
            log(f"시뮬레이터 준비 대기: {len(found)}/{vehicle_count} 기체 확인")
            next_report = now + 10.0
        time.sleep(2.0)
    raise RuntimeError(f"simulator did not become ready within {timeout:.0f}s")


def container_process(container, command, output_file):
    return subprocess.Popen(
        ["docker", "exec", container, "bash", "-lc", command],
        stdout=output_file,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )


def analyze_run(args, experiment_dir, run_dir, run_number):
    command = (
        "source /opt/ros/humble/setup.bash; "
        f"cd {args.container_repo}; "
        "python3 scripts/plot_multi_mppi_trajectory.py "
        f"--world sim_assets/worlds/{args.world}.world "
        f"--bag {args.container_repo}/{run_dir.relative_to(REPO)}/rosbag "
        f"--output {args.container_repo}/{run_dir.relative_to(REPO)}/trajectory.png "
        f"--run-label run_{run_number:02d}"
    )
    result = docker_exec(
        args.ros_container,
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    (run_dir / "trajectory_metrics.txt").write_text(result.stdout)
    log(f"경로 이미지 생성: {run_dir / 'trajectory.png'}")


def run_once(args, experiment_dir, run_number):
    run_dir = experiment_dir / f"run_{run_number:02d}"
    run_dir.mkdir(parents=True, exist_ok=False)
    launch_log = run_dir / "launch.log"
    bag_log = run_dir / "rosbag.log"
    result_path = run_dir / "result.json"
    launch_process = None
    bag_process = None

    log(f"실험 {run_number}/{args.runs}: 시뮬레이터 초기화")
    cleanup_ros_processes(args.ros_container)
    run(
        ["docker", "restart", args.ros_container, args.sim_container],
        stdout=subprocess.DEVNULL,
    )
    wait_for_sim(args.sim_container, args.vehicle_count, args.sim_timeout)

    try:
        with launch_log.open("w") as launch_output, bag_log.open("w") as bag_output:
            launch_command = (
                "source /opt/ros/humble/setup.bash; "
                f"cd {args.container_repo}; source install/setup.bash; "
                "exec ros2 launch mppi_lidar multi_mppi_lidar.launch.py "
                f"vehicle_count:={args.vehicle_count}"
            )
            launch_process = container_process(
                args.ros_container, launch_command, launch_output
            )
            log(f"실험 {run_number}/{args.runs}: ROS launch 시작")
            time.sleep(args.launch_wait)
            if launch_process.poll() is not None:
                raise RuntimeError("ROS launch exited during startup")

            bag_command = (
                "source /opt/ros/humble/setup.bash; "
                f"cd {args.container_repo}; source install/setup.bash; "
                f"exec ros2 bag record -a -x '{args.exclude_topics}' "
                f"-o {args.container_repo}/{run_dir.relative_to(REPO)}/rosbag"
            )
            bag_process = container_process(args.ros_container, bag_command, bag_output)

            waiter_command = (
                "source /opt/ros/humble/setup.bash; "
                f"cd {args.container_repo}; source install/setup.bash; "
                "python3 scripts/wait_multi_flight_success.py "
                f"--timeout {args.timeout} --vehicle-count {args.vehicle_count} "
                f"--progress-interval {args.progress_interval}"
            )
            log(f"실험 {run_number}/{args.runs}: 비행 및 rosbag 기록 시작")
            waiter = subprocess.Popen(
                ["docker", "exec", args.ros_container, "bash", "-lc", waiter_command],
                stdout=subprocess.PIPE,
                stderr=None,
                universal_newlines=True,
            )
            stdout, _ = waiter.communicate()
            result_path.write_text(stdout)
            if waiter.returncode != 0:
                raise RuntimeError(f"flight waiter failed with exit code {waiter.returncode}")
            result = json.loads(stdout)
            if not result.get("success"):
                raise RuntimeError("flight result reported success=false")
    finally:
        cleanup_ros_processes(args.ros_container)
        stop_process(bag_process, "rosbag")
        stop_process(launch_process, "ROS launch")

    analyze_run(args, experiment_dir, run_dir, run_number)
    log(f"실험 {run_number}/{args.runs}: 성공 ({result['elapsed_sec']:.1f}s)")


def create_overlay(args, experiment_dir):
    output = experiment_dir / "trajectory_overlay.png"
    command = (
        "source /opt/ros/humble/setup.bash; "
        f"cd {args.container_repo}; "
        "python3 scripts/plot_multi_mppi_four_run_overlay.py "
        f"--world sim_assets/worlds/{args.world}.world "
        f"--experiment-dir {args.container_repo}/{experiment_dir.relative_to(REPO)} "
        f"--runs {args.runs} --output {args.container_repo}/{output.relative_to(REPO)}"
    )
    result = docker_exec(
        args.ros_container,
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    log(result.stdout.strip())
    log(f"중첩 경로 이미지 생성: {output}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run repeatable multi-UAV MPPI LiDAR experiments"
    )
    parser.add_argument("--runs", type=int, default=1, help="number of runs")
    parser.add_argument("--vehicle-count", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--progress-interval", type=float, default=10.0)
    parser.add_argument("--sim-timeout", type=float, default=240.0)
    parser.add_argument("--launch-wait", type=float, default=8.0)
    parser.add_argument("--world", default="random_cylinders_double_178640653")
    parser.add_argument("--sim-container", default="av_drone-sim-1")
    parser.add_argument("--ros-container", default="av_drone-ros-1")
    parser.add_argument("--container-repo", default="/workspace/AV_Drone")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--exclude-topics", default=".*(camera|image|video).*")
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be positive")
    if args.vehicle_count < 1 or args.vehicle_count > 3:
        parser.error("--vehicle-count must be between 1 and 3")
    return args


def main():
    args = parse_args()
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    experiment_dir = (
        args.output_dir
        if args.output_dir is not None
        else REPO / "experiments" / f"{stamp}_multi_mppi_{args.world}"
    ).resolve()
    try:
        experiment_dir.relative_to(REPO)
    except ValueError:
        raise SystemExit("--output-dir must be inside the repository")
    if experiment_dir.exists():
        raise SystemExit(f"output directory already exists: {experiment_dir}")
    experiment_dir.mkdir(parents=True)

    try:
        run(["docker", "inspect", args.sim_container], stdout=subprocess.DEVNULL)
        run(["docker", "inspect", args.ros_container], stdout=subprocess.DEVNULL)
        configured_world = run(
            ["docker", "inspect", "--format", "{{range .Config.Env}}{{println .}}{{end}}", args.sim_container],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        ).stdout.splitlines()
        world_values = [
            value.split("=", 1)[1]
            for value in configured_world
            if value.startswith("PX4_SITL_WORLD=")
        ]
        if not world_values or world_values[-1] != args.world:
            actual = world_values[-1] if world_values else "(unset)"
            raise RuntimeError(
                f"sim container world is {actual}, expected {args.world}; "
                "recreate it with docker-compose.multi-mppi.yml first"
            )
        for run_number in range(1, args.runs + 1):
            run_once(args, experiment_dir, run_number)
        create_overlay(args, experiment_dir)
    except Exception as exc:
        print(f"\n[FAILED] {exc}", file=sys.stderr)
        run_dirs = sorted(experiment_dir.glob("run_*"))
        if run_dirs:
            failed_dir = run_dirs[-1]
            print("\n--- launch.log tail ---", file=sys.stderr)
            print(tail(failed_dir / "launch.log"), file=sys.stderr)
            print("\n--- rosbag.log tail ---", file=sys.stderr)
            print(tail(failed_dir / "rosbag.log"), file=sys.stderr)
        return 1

    log(f"전체 {args.runs}회 실험 완료: {experiment_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
