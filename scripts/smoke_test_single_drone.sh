#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

EXPERIMENT_SCENARIO=""
EXPERIMENT_ISSUE=""
EXPERIMENT_FIX=""
EXPERIMENT_NOTES=""
RUNNER_NAME="smoke_test_single_drone"

while (($#)); do
  case "$1" in
    --scenario)
      EXPERIMENT_SCENARIO="$2"
      shift 2
      ;;
    --issue)
      EXPERIMENT_ISSUE="$2"
      shift 2
      ;;
    --fix)
      EXPERIMENT_FIX="$2"
      shift 2
      ;;
    --notes)
      EXPERIMENT_NOTES="$2"
      shift 2
      ;;
    *)
      echo "[FAIL] unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

fail() {
  local message="$1"
  record_result fail "${message}" || true
  echo "[FAIL] $1" >&2
  exit 1
}

pass() {
  echo "[PASS] $1"
}

info() {
  echo "[INFO] $1"
}

require_service_running() {
  local service="$1"
  if ! docker compose ps --services --status running | grep -qx "${service}"; then
    fail "docker compose service '${service}' is not running"
  fi
}

ros_exec() {
  docker compose exec -T ros bash -lc "source /opt/ros/humble/setup.bash && cd /workspace/AV_Drone && source install/setup.bash && $*"
}

latest_artifact_dir() {
  ls -1dt artifacts/*_drone1 2>/dev/null | head -n1
}

record_result() {
  local result="$1"
  local reason="${2:-}"
  local artifact=""
  local notes="${EXPERIMENT_NOTES}"

  artifact="$(latest_artifact_dir || true)"
  if [ -n "${reason}" ]; then
    if [ -n "${notes}" ]; then
      notes="${notes} | "
    fi
    notes="${notes}smoke_result_detail=${reason}"
  fi

  if [ -n "${artifact}" ] && [ -f "${artifact}/summary.json" ]; then
    python3 scripts/generate_artifact_plots.py --artifact "${artifact}" >/dev/null 2>&1 || true
  fi

  local registry_cmd=(
    python3 scripts/update_experiment_registry.py
    --runner "${RUNNER_NAME}"
    --result "${result}"
    --issue "${EXPERIMENT_ISSUE}"
    --fix "${EXPERIMENT_FIX}"
    --notes "${notes}"
    --allow-missing-artifact
  )

  if [ -n "${EXPERIMENT_SCENARIO}" ]; then
    registry_cmd+=(--scenario "${EXPERIMENT_SCENARIO}")
  fi

  if [ -n "${artifact}" ]; then
    registry_cmd+=(--artifact "${artifact}")
  fi

  "${registry_cmd[@]}" >/dev/null 2>&1 || true
}

wait_for_sim_ready() {
  local timeout_sec="${1:-300}"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if docker compose logs --tail=200 sim 2>/dev/null | grep -q "Startup script returned successfully"; then
      pass "sim runtime is ready"
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      fail "sim did not reach runtime readiness within ${timeout_sec}s. Check 'docker compose logs -f sim'"
    fi
    sleep 2
  done
}

wait_for_node() {
  local node_name="$1"
  local timeout_sec="${2:-120}"
  local start_ts
  start_ts="$(date +%s)"
  while true; do
    if ros_exec "timeout 5 ros2 node list" 2>/dev/null | grep -qx "${node_name}"; then
      pass "ROS node is running: ${node_name}"
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      fail "ROS node '${node_name}' was not found within ${timeout_sec}s. Did you run ros2 launch drone_bringup single_drone_autonomy.launch.py?"
    fi
    sleep 2
  done
}

wait_for_topic_sample() {
  local topic="$1"
  local timeout_sec="${2:-120}"
  local start_ts
  local sample
  start_ts="$(date +%s)"
  while true; do
    sample="$(ros_exec "timeout 8 ros2 topic echo ${topic} --once" 2>/dev/null || true)"
    if [ -n "${sample}" ]; then
      printf '%s' "${sample}"
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      fail "did not receive sample from ${topic} within ${timeout_sec}s"
    fi
    sleep 2
  done
}

wait_for_topic_value() {
  local topic="$1"
  local expected="$2"
  local timeout_sec="${3:-180}"
  local start_ts
  local sample
  local value
  start_ts="$(date +%s)"
  while true; do
    sample="$(wait_for_topic_sample "${topic}" 20)"
    value="$(echo "${sample}" | awk '/^data:/{print $2; exit}')"
    if [ "${value}" = "${expected}" ]; then
      printf '%s' "${sample}"
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      fail "${topic} did not reach expected value '${expected}' within ${timeout_sec}s (last='${value}')"
    fi
    sleep 2
  done
}

require_service_running sim
require_service_running ros
pass "sim and ros containers are running"
info "waiting for sim runtime readiness"
wait_for_sim_ready 300
info "waiting for autonomy stack nodes"
wait_for_node "/autonomy_manager" 180
wait_for_node "/metrics_logger" 180

SCAN_SAMPLE="$(wait_for_topic_sample /drone1/scan 180)"
echo "${SCAN_SAMPLE}" | grep -q "frame_id: rplidar_link" || fail "did not receive /drone1/scan LaserScan sample"
pass "/drone1/scan sample received"

PHASE_SAMPLE="$(wait_for_topic_sample /drone1/mission/phase 120)"
PHASE_VALUE="$(echo "${PHASE_SAMPLE}" | awk '/^data:/{print $2; exit}')"
[ -n "${PHASE_VALUE}" ] || fail "did not receive /drone1/mission/phase"
pass "mission phase sample received: ${PHASE_VALUE}"

FINAL_PHASE_SAMPLE="$(wait_for_topic_value /drone1/mission/phase HOVER_AT_GOAL 180)"
FINAL_PHASE_VALUE="$(echo "${FINAL_PHASE_SAMPLE}" | awk '/^data:/{print $2; exit}')"
pass "mission reached final phase: ${FINAL_PHASE_VALUE}"

GOAL_REACHED_SAMPLE="$(wait_for_topic_value /drone1/mission/goal_reached true 180)"
GOAL_REACHED_VALUE="$(echo "${GOAL_REACHED_SAMPLE}" | awk '/^data:/{print $2; exit}')"
pass "goal_reached topic reported: ${GOAL_REACHED_VALUE}"

POSE_SAMPLE="$(wait_for_topic_sample /mavros/local_position/pose 120)"
echo "${POSE_SAMPLE}" | grep -q "position:" || fail "did not receive /mavros/local_position/pose"
pass "/mavros/local_position/pose sample received"

OBSTACLE_SAMPLE="$(wait_for_topic_sample /drone1/perception/nearest_obstacle_distance 120)"
OBSTACLE_VALUE="$(echo "${OBSTACLE_SAMPLE}" | awk '/^data:/{print $2; exit}')"
[ -n "${OBSTACLE_VALUE}" ] || fail "did not receive /drone1/perception/nearest_obstacle_distance"
pass "nearest obstacle topic is active: ${OBSTACLE_VALUE} m"

LATEST_ARTIFACT="$(latest_artifact_dir)"
[ -n "${LATEST_ARTIFACT}" ] || fail "no artifact directory found under artifacts/"
[ -f "${LATEST_ARTIFACT}/summary.json" ] || fail "latest artifact has no summary.json"

SCAN_COUNT="$(awk -F': ' '/"scan_count"/ {gsub(/,/, "", $2); print $2; exit}' "${LATEST_ARTIFACT}/summary.json")"
POSE_COUNT="$(awk -F': ' '/"pose_count"/ {gsub(/,/, "", $2); print $2; exit}' "${LATEST_ARTIFACT}/summary.json")"
MISSION_PHASE="$(awk -F': ' '/"mission_phase"/ {gsub(/[",]/, "", $2); print $2; exit}' "${LATEST_ARTIFACT}/summary.json")"
GOAL_REACHED="$(awk -F': ' '/"goal_reached"/ {gsub(/,/, "", $2); print $2; exit}' "${LATEST_ARTIFACT}/summary.json")"
CURRENT_OBSTACLE="$(awk -F': ' '/"current_obstacle_m"/ {gsub(/,/, "", $2); print $2; exit}' "${LATEST_ARTIFACT}/summary.json")"
CLOSEST_OBSTACLE="$(awk -F': ' '/"closest_obstacle_m"/ {gsub(/,/, "", $2); print $2; exit}' "${LATEST_ARTIFACT}/summary.json")"

[ -n "${SCAN_COUNT}" ] || fail "scan_count missing from summary.json"
[ -n "${POSE_COUNT}" ] || fail "pose_count missing from summary.json"
[ -n "${MISSION_PHASE}" ] || fail "mission_phase missing from summary.json"
[ -n "${GOAL_REACHED}" ] || fail "goal_reached missing from summary.json"
[ "${SCAN_COUNT}" -gt 0 ] || fail "scan_count is zero in summary.json"
[ "${POSE_COUNT}" -gt 0 ] || fail "pose_count is zero in summary.json"
[ "${MISSION_PHASE}" = "HOVER_AT_GOAL" ] || fail "mission_phase is '${MISSION_PHASE}', expected HOVER_AT_GOAL"
[ "${GOAL_REACHED}" = "true" ] || fail "goal_reached is '${GOAL_REACHED}', expected true"

if case "${CURRENT_OBSTACLE}" in
  ""|inf|.inf|Infinity) true ;;
  *) false ;;
esac; then
  case "${CLOSEST_OBSTACLE}" in
    ""|inf|.inf|Infinity)
      fail "neither current_obstacle_m nor closest_obstacle_m is finite in summary.json"
      ;;
  esac
fi

pass "artifact summary looks valid: phase=${MISSION_PHASE}, goal_reached=${GOAL_REACHED}, pose_count=${POSE_COUNT}, scan_count=${SCAN_COUNT}, closest_obstacle_m=${CLOSEST_OBSTACLE}, current_obstacle_m=${CURRENT_OBSTACLE}"

record_result pass ""

echo
echo "Smoke test completed successfully."
echo "Latest artifact: ${LATEST_ARTIFACT}"
echo "Observed mission phase: ${PHASE_VALUE}"
echo "Observed nearest obstacle topic sample: ${OBSTACLE_VALUE} m"
