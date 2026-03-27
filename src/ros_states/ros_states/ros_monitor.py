"""ROS2 State Monitor with AV_Drone-specific flight debug helpers."""

import json
import math
import os
import re
import subprocess
import threading
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ros_states.debug_report import generate_session_report
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

try:
    from geometry_msgs.msg import PoseStamped, TwistStamped
except Exception:
    PoseStamped = None
    TwistStamped = None

try:
    from mavros_msgs.msg import State as MavrosState
except Exception:
    MavrosState = None

try:
    from sensor_msgs.msg import LaserScan
except Exception:
    LaserScan = None

try:
    from std_msgs.msg import Bool, Float32, String
except Exception:
    Bool = None
    Float32 = None
    String = None

# Node names to filter out from counts
_INTERNAL_NODES = {'ros_web_monitor'}
_INTERNAL_PREFIX = '_ros2cli_'
_SEVERITY_ORDER = {'ok': 0, 'info': 1, 'warn': 2, 'error': 3}


def _is_internal(node_name):
    return node_name in _INTERNAL_NODES or node_name.startswith(_INTERNAL_PREFIX)


def _safe_float(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _json_safe_value(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe_value(val) for key, val in value.items()}
    if isinstance(value, list):
        return [_json_safe_value(item) for item in value]
    if isinstance(value, tuple):
        return [_json_safe_value(item) for item in value]
    return value


@dataclass
class AvDroneProfile:
    drone_name: str = 'drone1'
    mavros_namespace: str = '/mavros'
    artifacts_root: str = '/workspace/AV_Drone/artifacts'
    pose_timeout_sec: float = 0.5
    scan_timeout_sec: float = 0.5
    planner_cmd_timeout_sec: float = 0.5
    startup_grace_sec: float = 3.0
    emergency_stop_distance: float = 1.0
    obstacle_stop_distance: float = 2.0

    @classmethod
    def from_dict(cls, data=None):
        data = data or {}
        return cls(
            drone_name=str(data.get('drone_name') or 'drone1').strip('/') or 'drone1',
            mavros_namespace=str(data.get('mavros_namespace') or '/mavros').strip() or '/mavros',
            artifacts_root=str(data.get('artifacts_root') or '/workspace/AV_Drone/artifacts').strip() or '/workspace/AV_Drone/artifacts',
            pose_timeout_sec=_safe_float(data.get('pose_timeout_sec'), 0.5),
            scan_timeout_sec=_safe_float(data.get('scan_timeout_sec'), 0.5),
            planner_cmd_timeout_sec=_safe_float(data.get('planner_cmd_timeout_sec'), 0.5),
            startup_grace_sec=_safe_float(data.get('startup_grace_sec'), 3.0),
            emergency_stop_distance=_safe_float(data.get('emergency_stop_distance'), 1.0),
            obstacle_stop_distance=_safe_float(data.get('obstacle_stop_distance'), 2.0),
        )

    def drone_namespace(self):
        return '/' + self.drone_name.strip('/')

    def mavros_ns(self):
        ns = self.mavros_namespace.strip()
        if not ns.startswith('/'):
            ns = '/' + ns
        return ns.rstrip('/') or '/mavros'

    def topic_map(self):
        drone_ns = self.drone_namespace()
        mavros_ns = self.mavros_ns()
        return {
            'state': f'{mavros_ns}/state',
            'pose': f'{mavros_ns}/local_position/pose',
            'setpoint_cmd': f'{mavros_ns}/setpoint_velocity/cmd_vel',
            'scan': f'{drone_ns}/scan',
            'autonomy_cmd': f'{drone_ns}/autonomy/cmd_vel',
            'safe_cmd': f'{drone_ns}/safety/cmd_vel',
            'mission_phase': f'{drone_ns}/mission/phase',
            'nearest_obstacle': f'{drone_ns}/perception/nearest_obstacle_distance',
            'safety_event': f'{drone_ns}/safety/event',
            'goal_reached': f'{drone_ns}/mission/goal_reached',
        }

    def to_dict(self):
        data = {
            'drone_name': self.drone_name,
            'drone_namespace': self.drone_namespace(),
            'mavros_namespace': self.mavros_ns(),
            'artifacts_root': self.artifacts_root,
            'pose_timeout_sec': self.pose_timeout_sec,
            'scan_timeout_sec': self.scan_timeout_sec,
            'planner_cmd_timeout_sec': self.planner_cmd_timeout_sec,
            'startup_grace_sec': self.startup_grace_sec,
            'emergency_stop_distance': self.emergency_stop_distance,
            'obstacle_stop_distance': self.obstacle_stop_distance,
        }
        data.update(self.topic_map())
        return data


class RosMonitor:
    def __init__(self):
        self._node = None
        self._spin_thread = None
        self._active = False
        self._domain_id = 0
        self._activated_at = None
        self._profile = AvDroneProfile()

        # TF data
        self._tf_frames = {}
        self._tf_lock = threading.Lock()

        # Hz cache
        self._hz_cache = {}
        self._hz_lock = threading.Lock()
        self._hz_thread = None

        # AV_Drone flight debug cache
        self._flight_cache = {}
        self._flight_lock = threading.Lock()
        self._flight_subscriptions = []
        self._subscription_errors = []

        # ros_states debug recorder
        self._debug_lock = threading.Lock()
        self._debug_thread = None
        self._debug_stop_event = threading.Event()
        self._debug_state = {
            'active': False,
            'started_at': None,
            'stopped_at': None,
            'interval_sec': None,
            'session_dir': None,
            'snapshots_dir': None,
            'timeline_path': None,
            'snapshot_count': 0,
            'last_snapshot_path': None,
            'last_snapshot_at': None,
            'last_error': None,
            'report_path': None,
            'report_summary_path': None,
            'report_generated_at': None,
        }

    @property
    def active(self):
        return self._active

    @property
    def domain_id(self):
        return self._domain_id

    @property
    def profile(self):
        return self._profile.to_dict()

    def activate(self, domain_id, profile=None):
        if self._active:
            self.deactivate()

        self._profile = AvDroneProfile.from_dict(profile)
        self._domain_id = int(domain_id)
        self._activated_at = time.time()
        os.environ['ROS_DOMAIN_ID'] = str(self._domain_id)

        try:
            rclpy.init()
        except RuntimeError:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            rclpy.init()

        self._node = rclpy.create_node('ros_web_monitor')

        qos_tf = QoSProfile(depth=100)
        qos_tf_static = QoSProfile(
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._tf_frames = {}
        self._hz_cache = {}
        self._reset_flight_cache()

        self._node.create_subscription(TFMessage, '/tf', self._tf_cb, qos_tf)
        self._node.create_subscription(TFMessage, '/tf_static', self._tf_static_cb, qos_tf_static)

        self._active = True
        self._configure_flight_subscriptions()

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self._hz_thread = threading.Thread(target=self._hz_loop, daemon=True)
        self._hz_thread.start()

    def deactivate(self):
        self.stop_debug_recording(reason='monitor_deactivate')
        self._active = False
        self._activated_at = None
        self._flight_subscriptions = []

        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

        try:
            rclpy.shutdown()
        except Exception:
            pass

        with self._tf_lock:
            self._tf_frames.clear()
        with self._hz_lock:
            self._hz_cache.clear()
        self._reset_flight_cache()

    def _reset_flight_cache(self):
        with self._flight_lock:
            self._flight_cache = {}
            self._subscription_errors = []

    def _spin(self):
        while self._active and self._node:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception:
                break

    def _tf_cb(self, msg):
        with self._tf_lock:
            for t in msg.transforms:
                self._tf_frames[t.child_frame_id] = t.header.frame_id

    def _tf_static_cb(self, msg):
        with self._tf_lock:
            for t in msg.transforms:
                self._tf_frames[t.child_frame_id] = t.header.frame_id

    def _configure_flight_subscriptions(self):
        self._flight_subscriptions = []
        with self._flight_lock:
            self._subscription_errors = []

        topics = self._profile.topic_map()

        best_effort_qos = QoSProfile(depth=20)
        best_effort_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        reliable_qos = QoSProfile(depth=20)

        self._create_optional_subscription('state', topics['state'], MavrosState, self._mavros_state_cb, best_effort_qos)
        self._create_optional_subscription('pose', topics['pose'], PoseStamped, self._pose_cb, best_effort_qos)
        self._create_optional_subscription('scan', topics['scan'], LaserScan, self._scan_cb, best_effort_qos)
        self._create_optional_subscription('nearest_obstacle', topics['nearest_obstacle'], Float32, self._nearest_obstacle_cb, reliable_qos)
        self._create_optional_subscription('mission_phase', topics['mission_phase'], String, self._mission_phase_cb, reliable_qos)
        self._create_optional_subscription('goal_reached', topics['goal_reached'], Bool, self._goal_reached_cb, reliable_qos)
        self._create_optional_subscription('safety_event', topics['safety_event'], String, self._safety_event_cb, reliable_qos)
        self._create_optional_subscription('autonomy_cmd', topics['autonomy_cmd'], TwistStamped, self._autonomy_cmd_cb, reliable_qos)
        self._create_optional_subscription('safe_cmd', topics['safe_cmd'], TwistStamped, self._safe_cmd_cb, reliable_qos)
        self._create_optional_subscription('setpoint_cmd', topics['setpoint_cmd'], TwistStamped, self._setpoint_cmd_cb, reliable_qos)

    def _create_optional_subscription(self, key, topic_name, msg_type, callback, qos):
        if not self._node:
            return
        if msg_type is None:
            with self._flight_lock:
                self._subscription_errors.append({
                    'key': key,
                    'topic': topic_name,
                    'message': f'Message type for {key} is unavailable in this ROS environment',
                })
            return

        try:
            sub = self._node.create_subscription(msg_type, topic_name, callback, qos)
            self._flight_subscriptions.append(sub)
        except Exception as exc:
            with self._flight_lock:
                self._subscription_errors.append({
                    'key': key,
                    'topic': topic_name,
                    'message': str(exc),
                })

    def _record_flight_value(self, key, payload):
        with self._flight_lock:
            self._flight_cache[key] = {
                'received_at': time.time(),
                **payload,
            }

    def _flight_entry(self, key):
        with self._flight_lock:
            entry = self._flight_cache.get(key)
            return dict(entry) if entry else None

    def _mavros_state_cb(self, msg):
        self._record_flight_value('state', {
            'connected': bool(getattr(msg, 'connected', False)),
            'armed': bool(getattr(msg, 'armed', False)),
            'guided': bool(getattr(msg, 'guided', False)),
            'mode': str(getattr(msg, 'mode', '')),
            'system_status': int(getattr(msg, 'system_status', 0)),
        })

    def _pose_cb(self, msg):
        pos = msg.pose.position
        self._record_flight_value('pose', {
            'x': round(pos.x, 3),
            'y': round(pos.y, 3),
            'z': round(pos.z, 3),
        })

    def _scan_cb(self, msg):
        finite_ranges = [r for r in msg.ranges if math.isfinite(r)]
        min_range = min(finite_ranges) if finite_ranges else None
        self._record_flight_value('scan', {
            'min_range': round(min_range, 3) if min_range is not None else None,
            'finite_count': len(finite_ranges),
            'range_count': len(msg.ranges),
        })

    def _nearest_obstacle_cb(self, msg):
        distance = float(msg.data)
        self._record_flight_value('nearest_obstacle', {
            'distance': round(distance, 3) if math.isfinite(distance) else None,
        })

    def _mission_phase_cb(self, msg):
        self._record_flight_value('mission_phase', {'text': str(msg.data)})

    def _goal_reached_cb(self, msg):
        self._record_flight_value('goal_reached', {'value': bool(msg.data)})

    def _safety_event_cb(self, msg):
        self._record_flight_value('safety_event', {'text': str(msg.data)})

    def _autonomy_cmd_cb(self, msg):
        self._record_flight_value('autonomy_cmd', self._twist_payload(msg))

    def _safe_cmd_cb(self, msg):
        self._record_flight_value('safe_cmd', self._twist_payload(msg))

    def _setpoint_cmd_cb(self, msg):
        self._record_flight_value('setpoint_cmd', self._twist_payload(msg))

    def _twist_payload(self, msg):
        twist = msg.twist
        linear = twist.linear
        angular = twist.angular
        speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
        return {
            'vx': round(linear.x, 3),
            'vy': round(linear.y, 3),
            'vz': round(linear.z, 3),
            'yaw_rate': round(angular.z, 3),
            'speed': round(speed, 3),
            'is_zero': speed < 0.05 and abs(angular.z) < 0.05,
        }

    # --- Hz measurement (sequential subprocess, one topic at a time) ---

    def _hz_loop(self):
        """Continuously measure Hz for active topics, one at a time."""
        while self._active:
            topics = self._get_publishable_topics()
            for topic_name in topics:
                if not self._active:
                    break
                self._measure_hz_single(topic_name)
            time.sleep(2.0)

    def _get_publishable_topics(self):
        """Get list of topics that have external publishers."""
        if not self._node:
            return []
        try:
            names_and_types = self._node.get_topic_names_and_types()
        except Exception:
            return []

        result = []
        for name, _ in names_and_types:
            try:
                pubs = self._node.get_publishers_info_by_topic(name)
                if any(not _is_internal(p.node_name) for p in pubs):
                    result.append(name)
            except Exception:
                pass

        result = sorted(set(result))
        watch_names = [topic for topic in self._profile.topic_map().values() if topic in result]
        watch_set = set(watch_names)
        return watch_names + [name for name in result if name not in watch_set]

    def _measure_hz_single(self, topic_name):
        """Measure Hz for a single topic using ros2 topic hz subprocess."""
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(self._domain_id)

        try:
            proc = subprocess.Popen(
                ['ros2', 'topic', 'hz', topic_name, '--window', '5'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env,
            )
            try:
                output, _ = proc.communicate(timeout=4)
            except subprocess.TimeoutExpired:
                proc.kill()
                output, _ = proc.communicate()

            if output:
                matches = re.findall(r'average rate:\s+([\d.]+)', output)
                if matches:
                    with self._hz_lock:
                        self._hz_cache[topic_name] = float(matches[-1])
        except Exception:
            pass

    # --- Node map builder ---

    def _build_node_maps(self):
        """Build mappings: service_name -> server nodes, service_name -> client nodes."""
        server_map = defaultdict(set)
        client_map = defaultdict(set)

        if not self._node:
            return server_map, client_map

        try:
            nodes = self._node.get_node_names_and_namespaces()
        except Exception:
            return server_map, client_map

        for node_name, namespace in nodes:
            if _is_internal(node_name):
                continue
            full_name = namespace.rstrip('/') + '/' + node_name

            try:
                svcs = self._node.get_service_names_and_types_by_node(node_name, namespace)
                for svc_name, _ in svcs:
                    server_map[svc_name].add(full_name)
            except Exception:
                pass

            try:
                clients = self._node.get_client_names_and_types_by_node(node_name, namespace)
                for svc_name, _ in clients:
                    client_map[svc_name].add(full_name)
            except Exception:
                pass

        return server_map, client_map

    def _topic_graph_snapshot(self, topic_name):
        snapshot = {
            'name': topic_name,
            'type': 'unknown',
            'publishers': 0,
            'subscribers': 0,
            'pub_nodes': [],
            'sub_nodes': [],
            'hz': None,
        }
        if not self._active or not self._node:
            return snapshot

        try:
            names_and_types = dict(self._node.get_topic_names_and_types())
            types = names_and_types.get(topic_name)
            if types:
                snapshot['type'] = types[0]
        except Exception:
            pass

        try:
            pubs = self._node.get_publishers_info_by_topic(topic_name)
            subs = self._node.get_subscriptions_info_by_topic(topic_name)
        except Exception:
            pubs, subs = [], []

        ext_pubs = [p for p in pubs if not _is_internal(p.node_name)]
        ext_subs = [s for s in subs if not _is_internal(s.node_name)]
        snapshot['publishers'] = len(ext_pubs)
        snapshot['subscribers'] = len(ext_subs)
        snapshot['pub_nodes'] = sorted({
            p.node_namespace.rstrip('/') + '/' + p.node_name
            for p in ext_pubs
        })
        snapshot['sub_nodes'] = sorted({
            s.node_namespace.rstrip('/') + '/' + s.node_name
            for s in ext_subs
        })

        with self._hz_lock:
            hz = self._hz_cache.get(topic_name)
        snapshot['hz'] = round(hz, 1) if hz is not None else None
        return snapshot

    def _format_age(self, age_sec):
        if age_sec is None:
            return 'never'
        if age_sec < 1.0:
            return f'{age_sec:.2f}s ago'
        if age_sec < 10.0:
            return f'{age_sec:.1f}s ago'
        return f'{age_sec:.0f}s ago'

    def _entry_age(self, entry, now):
        if not entry:
            return None
        return max(0.0, now - float(entry.get('received_at', now)))

    def _vector_text(self, entry):
        if not entry:
            return '-'
        return (
            f"vx {entry.get('vx', 0.0):.2f} | "
            f"vy {entry.get('vy', 0.0):.2f} | "
            f"vz {entry.get('vz', 0.0):.2f} | "
            f"yaw {entry.get('yaw_rate', 0.0):.2f}"
        )

    def _missing_status(self, in_grace):
        return 'warn' if in_grace else 'error'

    def _expected_nodes(self):
        nodes = self.get_node_list()
        mavros_ns = self._profile.mavros_ns()
        core = [
            ('MAVROS namespace', mavros_ns, any(n == mavros_ns or n.startswith(mavros_ns + '/') for n in nodes)),
            ('autonomy_manager', '/autonomy_manager', '/autonomy_manager' in nodes),
            ('lidar_obstacle', '/lidar_obstacle', '/lidar_obstacle' in nodes),
            ('local_planner', '/local_planner', '/local_planner' in nodes),
            ('safety_monitor', '/safety_monitor', '/safety_monitor' in nodes),
            ('metrics_logger', '/metrics_logger', '/metrics_logger' in nodes),
        ]

        result = []
        for label, expected, present in core:
            result.append({
                'label': label,
                'expected': expected,
                'status': 'ok' if present else 'error',
                'detail': 'Detected' if present else 'Missing from ROS graph',
            })
        return result

    def _read_json_file(self, path):
        try:
            with path.open('r', encoding='utf-8') as handle:
                return json.load(handle)
        except Exception:
            return None

    def get_latest_artifact_summary(self):
        profile = self._profile
        root = Path(profile.artifacts_root)
        result = {
            'status': 'warn',
            'available': False,
            'root': str(root),
            'artifact_dir': None,
            'summary': {},
            'metadata': {},
            'recent_events': [],
            'message': '',
        }

        if not root.exists():
            result['message'] = 'Artifact root does not exist'
            return result
        if not root.is_dir():
            result['message'] = 'Artifact root is not a directory'
            return result

        suffix = f'_{profile.drone_name}'
        candidates = [p for p in root.iterdir() if p.is_dir() and p.name.endswith(suffix)]
        if not candidates:
            candidates = [p for p in root.iterdir() if p.is_dir()]
        if not candidates:
            result['message'] = 'No artifact directories found'
            return result

        candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        artifact_dir = candidates[0]
        summary_path = artifact_dir / 'summary.json'
        metadata_path = artifact_dir / 'metadata.json'
        events_path = artifact_dir / 'events.log'

        summary = self._read_json_file(summary_path) if summary_path.exists() else {}
        metadata = self._read_json_file(metadata_path) if metadata_path.exists() else {}
        events = []
        if events_path.exists():
            try:
                with events_path.open('r', encoding='utf-8') as handle:
                    lines = [line.rstrip() for line in handle.readlines() if line.strip()]
                events = lines[-8:]
            except Exception:
                events = []

        result.update({
            'status': 'ok' if summary else 'warn',
            'available': bool(summary or metadata or events),
            'artifact_dir': str(artifact_dir),
            'summary': summary or {},
            'metadata': metadata or {},
            'recent_events': events,
            'message': 'Loaded latest artifact' if summary else 'Artifact folder found, but summary.json is missing',
        })
        return _json_safe_value(result)

    def get_flight_debug_snapshot(self):
        if not self._active or not self._node:
            return _json_safe_value({
                'active': False,
                'domain_id': self._domain_id,
                'profile': self._profile.to_dict(),
                'overall_status': 'info',
                'summary': 'Activate the dashboard to inspect the AV_Drone pipeline',
                'checks': [],
                'watch_topics': [],
                'expected_nodes': [],
                'artifact': self.get_latest_artifact_summary(),
                'hints': [],
                'subscription_errors': list(self._subscription_errors),
            })

        now = time.time()
        in_grace = (now - (self._activated_at or now)) < self._profile.startup_grace_sec
        topics = self._profile.topic_map()
        entries = {key: self._flight_entry(key) for key in topics}
        checks = []

        def add_check(key, label, status, headline, detail, topic_name):
            checks.append({
                'id': key,
                'label': label,
                'status': status,
                'headline': headline,
                'detail': detail,
                'topic': topic_name,
            })

        state = entries['state']
        state_age = self._entry_age(state, now)
        if not state or state_age is None:
            add_check('fcu', 'FCU Link', self._missing_status(in_grace), 'No MAVROS state yet', f'Waiting on {topics["state"]}', topics['state'])
            add_check('offboard', 'OFFBOARD / ARM', self._missing_status(in_grace), 'No FCU mode yet', 'MAVROS state has not arrived', topics['state'])
        else:
            if state_age > 2.0:
                fcu_status = self._missing_status(in_grace)
                fcu_headline = 'MAVROS state is stale'
            elif not state.get('connected'):
                fcu_status = 'error'
                fcu_headline = 'FCU disconnected'
            else:
                fcu_status = 'ok'
                fcu_headline = 'FCU connected'
            add_check(
                'fcu',
                'FCU Link',
                fcu_status,
                fcu_headline,
                f"{('armed' if state.get('armed') else 'disarmed')} | mode {state.get('mode') or '-'} | {self._format_age(state_age)}",
                topics['state'],
            )

            if state.get('connected') and state.get('armed') and state.get('mode') == 'OFFBOARD':
                offboard_status = 'ok'
                offboard_headline = 'OFFBOARD and armed'
                offboard_detail = self._format_age(state_age)
            elif state.get('connected'):
                offboard_status = 'warn'
                offboard_headline = f"mode {state.get('mode') or '-'}"
                offboard_detail = 'Waiting for OFFBOARD + armed'
            else:
                offboard_status = 'error'
                offboard_headline = 'No FCU control link'
                offboard_detail = 'PX4 is not connected through MAVROS'
            add_check('offboard', 'OFFBOARD / ARM', offboard_status, offboard_headline, offboard_detail, topics['state'])

        pose = entries['pose']
        pose_age = self._entry_age(pose, now)
        if not pose or pose_age is None or pose_age > self._profile.pose_timeout_sec:
            status = self._missing_status(in_grace)
            headline = 'Pose stream stale'
            detail = f'Expected fresh pose within {self._profile.pose_timeout_sec:.1f}s'
        else:
            status = 'ok'
            headline = f"x {pose['x']:.2f} | y {pose['y']:.2f} | z {pose['z']:.2f}"
            detail = self._format_age(pose_age)
        add_check('pose', 'Vehicle Pose', status, headline, detail, topics['pose'])

        scan = entries['scan']
        scan_age = self._entry_age(scan, now)
        if not scan or scan_age is None or scan_age > self._profile.scan_timeout_sec:
            status = self._missing_status(in_grace)
            headline = 'LiDAR scan stale'
            detail = f'Expected fresh scan within {self._profile.scan_timeout_sec:.1f}s'
        else:
            min_text = f"min {scan['min_range']:.2f}m" if scan.get('min_range') is not None else 'no finite ranges'
            status = 'ok' if scan.get('finite_count', 0) > 0 else 'warn'
            headline = min_text
            detail = f"{scan.get('finite_count', 0)} finite samples | {self._format_age(scan_age)}"
        add_check('scan', 'LiDAR Scan', status, headline, detail, topics['scan'])

        obstacle = entries['nearest_obstacle']
        obstacle_age = self._entry_age(obstacle, now)
        if not obstacle or obstacle_age is None:
            status = self._missing_status(in_grace)
            headline = 'No obstacle metric yet'
            detail = f'Waiting on {topics["nearest_obstacle"]}'
        else:
            distance = obstacle.get('distance')
            if distance is None:
                status = 'warn'
                headline = 'Obstacle distance unavailable'
                detail = self._format_age(obstacle_age)
            elif distance <= self._profile.emergency_stop_distance:
                status = 'error'
                headline = f'{distance:.2f}m'
                detail = f'At or below emergency stop distance ({self._profile.emergency_stop_distance:.2f}m)'
            elif distance <= self._profile.obstacle_stop_distance:
                status = 'warn'
                headline = f'{distance:.2f}m'
                detail = f'Inside planner stop band ({self._profile.obstacle_stop_distance:.2f}m)'
            else:
                status = 'ok'
                headline = f'{distance:.2f}m'
                detail = self._format_age(obstacle_age)
        add_check('obstacle', 'Nearest Obstacle', status, headline, detail, topics['nearest_obstacle'])

        autonomy_cmd = entries['autonomy_cmd']
        autonomy_age = self._entry_age(autonomy_cmd, now)
        if not autonomy_cmd or autonomy_age is None or autonomy_age > self._profile.planner_cmd_timeout_sec:
            status = self._missing_status(in_grace)
            headline = 'Planner command stale'
            detail = f'Expected fresh planner cmd within {self._profile.planner_cmd_timeout_sec:.1f}s'
        else:
            status = 'ok'
            headline = self._vector_text(autonomy_cmd)
            detail = self._format_age(autonomy_age)
        add_check('autonomy_cmd', 'Planner Cmd', status, headline, detail, topics['autonomy_cmd'])

        safe_cmd = entries['safe_cmd']
        safe_age = self._entry_age(safe_cmd, now)
        if not safe_cmd or safe_age is None or safe_age > self._profile.planner_cmd_timeout_sec:
            status = self._missing_status(in_grace)
            headline = 'Safety output stale'
            detail = f'Expected fresh safe cmd within {self._profile.planner_cmd_timeout_sec:.1f}s'
        elif autonomy_cmd and not autonomy_cmd.get('is_zero') and safe_cmd.get('is_zero'):
            status = 'warn'
            headline = 'Safety is clamping velocity'
            detail = self._vector_text(safe_cmd)
        else:
            status = 'ok'
            headline = self._vector_text(safe_cmd)
            detail = self._format_age(safe_age)
        add_check('safe_cmd', 'Safety Cmd', status, headline, detail, topics['safe_cmd'])

        setpoint_cmd = entries['setpoint_cmd']
        setpoint_age = self._entry_age(setpoint_cmd, now)
        if not setpoint_cmd or setpoint_age is None or setpoint_age > self._profile.planner_cmd_timeout_sec:
            status = self._missing_status(in_grace)
            headline = 'FCU setpoint stale'
            detail = f'Waiting on {topics["setpoint_cmd"]}'
        else:
            status = 'ok'
            headline = self._vector_text(setpoint_cmd)
            detail = self._format_age(setpoint_age)
        add_check('setpoint_cmd', 'PX4 Setpoint', status, headline, detail, topics['setpoint_cmd'])

        mission_phase = entries['mission_phase']
        phase_age = self._entry_age(mission_phase, now)
        if not mission_phase or phase_age is None:
            status = self._missing_status(in_grace)
            headline = 'No mission phase yet'
            detail = f'Waiting on {topics["mission_phase"]}'
        else:
            phase_text = mission_phase.get('text') or '(empty)'
            status = 'ok' if phase_text == 'HOVER_AT_GOAL' else 'info'
            headline = phase_text
            detail = self._format_age(phase_age)
        add_check('mission_phase', 'Mission Phase', status, headline, detail, topics['mission_phase'])

        goal_reached = entries['goal_reached']
        goal_age = self._entry_age(goal_reached, now)
        if not goal_reached or goal_age is None:
            status = 'info'
            headline = 'Goal status unknown'
            detail = f'Waiting on {topics["goal_reached"]}'
        elif goal_reached.get('value'):
            status = 'ok'
            headline = 'Goal reached'
            detail = self._format_age(goal_age)
        else:
            status = 'info'
            headline = 'Goal not reached yet'
            detail = self._format_age(goal_age)
        add_check('goal_reached', 'Goal Reached', status, headline, detail, topics['goal_reached'])

        safety_event = entries['safety_event']
        event_age = self._entry_age(safety_event, now)
        if not safety_event or event_age is None:
            status = 'info'
            headline = 'No safety event yet'
            detail = f'Waiting on {topics["safety_event"]}'
        else:
            text = (safety_event.get('text') or '').strip()
            lowered = text.lower()
            if any(token in lowered for token in ('emergency', 'timeout', 'stop', 'collision')):
                status = 'error'
            elif any(token in lowered for token in ('startup', 'grace', 'hold', 'slow')):
                status = 'warn'
            else:
                status = 'ok'
            headline = text or '(empty)'
            detail = self._format_age(event_age)
        add_check('safety_event', 'Safety Event', status, headline, detail, topics['safety_event'])

        check_map = {check['id']: check for check in checks}
        watch_specs = [
            ('state', 'MAVROS State', check_map['fcu']),
            ('pose', 'Vehicle Pose', check_map['pose']),
            ('scan', 'LiDAR Scan', check_map['scan']),
            ('nearest_obstacle', 'Nearest Obstacle', check_map['obstacle']),
            ('mission_phase', 'Mission Phase', check_map['mission_phase']),
            ('goal_reached', 'Goal Reached', check_map['goal_reached']),
            ('autonomy_cmd', 'Planner Cmd', check_map['autonomy_cmd']),
            ('safe_cmd', 'Safety Cmd', check_map['safe_cmd']),
            ('setpoint_cmd', 'PX4 Setpoint', check_map['setpoint_cmd']),
            ('safety_event', 'Safety Event', check_map['safety_event']),
        ]
        watch_topics = []
        for key, label, check in watch_specs:
            graph = self._topic_graph_snapshot(topics[key])
            watch_topics.append({
                'key': key,
                'label': label,
                'status': check['status'],
                'topic': topics[key],
                'type': graph['type'],
                'publishers': graph['publishers'],
                'subscribers': graph['subscribers'],
                'hz': graph['hz'],
                'headline': check['headline'],
                'detail': check['detail'],
            })

        expected_nodes = self._expected_nodes()
        artifact = self.get_latest_artifact_summary()
        hints = []

        def add_hint(level, title, detail, commands):
            hints.append({
                'level': level,
                'title': title,
                'detail': detail,
                'commands': commands,
            })

        if check_map['fcu']['status'] == 'error':
            add_hint(
                'error',
                'MAVROS / PX4 link is unhealthy',
                'FCU state is missing or disconnected. Start by checking whether the ros container can see MAVROS and whether PX4 SITL is up.',
                [
                    f'ros2 topic echo {topics["state"]} --once',
                    'ros2 node list | grep mavros',
                    'docker compose ps',
                ],
            )
        if check_map['pose']['status'] in ('warn', 'error'):
            add_hint(
                'warn',
                'Pose stream is stale',
                'Without a fresh local pose, the autonomy pipeline will not progress safely.',
                [
                    f'ros2 topic hz {topics["pose"]}',
                    f'ros2 topic echo {topics["pose"]} --once',
                ],
            )
        if check_map['scan']['status'] in ('warn', 'error'):
            add_hint(
                'warn',
                'LiDAR data is missing or stale',
                'The obstacle pipeline depends on /scan. Check Gazebo sensor output before debugging planner logic.',
                [
                    f'ros2 topic hz {topics["scan"]}',
                    f'ros2 topic echo {topics["scan"]} --once',
                ],
            )
        if check_map['autonomy_cmd']['status'] in ('warn', 'error'):
            add_hint(
                'warn',
                'Planner is not producing commands',
                'When pose and scan are healthy but planner cmd is stale, focus on local_planner and mission phase logic.',
                [
                    f'ros2 topic echo {topics["mission_phase"]} --once',
                    f'ros2 topic echo {topics["autonomy_cmd"]} --once',
                    'ros2 node list | grep local_planner',
                ],
            )
        if check_map['safe_cmd']['status'] == 'warn':
            add_hint(
                'warn',
                'Safety layer is overriding planner velocity',
                'Planner output exists, but safety output is clamped. Inspect the latest safety event and obstacle distance.',
                [
                    f'ros2 topic echo {topics["safety_event"]} --once',
                    f'ros2 topic echo {topics["nearest_obstacle"]} --once',
                    f'ros2 topic echo {topics["safe_cmd"]} --once',
                ],
            )
        if check_map['setpoint_cmd']['status'] in ('warn', 'error'):
            add_hint(
                'warn',
                'Control output is not reaching MAVROS setpoint',
                'If planner and safety topics are healthy but MAVROS setpoints are stale, inspect autonomy_manager forwarding.',
                [
                    f'ros2 topic echo {topics["safe_cmd"]} --once',
                    f'ros2 topic echo {topics["setpoint_cmd"]} --once',
                    'ros2 node list | grep autonomy_manager',
                ],
            )
        if artifact['status'] != 'ok':
            add_hint(
                'info',
                'Artifact summary is unavailable',
                'Runtime is healthy enough to inspect, but the latest metrics artifact could not be read from the configured path.',
                [
                    f'ls -lah {self._profile.artifacts_root}',
                    'ros2 node list | grep metrics_logger',
                ],
            )
        if not hints:
            add_hint(
                'ok',
                'Pipeline looks healthy',
                'Core flight topics, node graph, and artifact output all look reasonable for the current mission state.',
                [
                    f'ros2 topic echo {topics["mission_phase"]} --once',
                    f'ros2 topic echo {topics["goal_reached"]} --once',
                ],
            )

        overall_status = 'ok'
        for check in checks:
            if _SEVERITY_ORDER[check['status']] > _SEVERITY_ORDER[overall_status]:
                overall_status = check['status']

        phase_text = mission_phase.get('text') if mission_phase else 'phase unknown'
        goal_text = 'goal reached' if goal_reached and goal_reached.get('value') else 'goal pending'
        obstacle_text = '-'
        if obstacle and obstacle.get('distance') is not None:
            obstacle_text = f"{obstacle['distance']:.2f}m obstacle"
        state_text = 'FCU disconnected'
        if state and state.get('connected'):
            state_text = f"{state.get('mode') or '-'} | {'armed' if state.get('armed') else 'disarmed'}"
        summary = f'{phase_text} | {goal_text} | {obstacle_text} | {state_text}'

        with self._flight_lock:
            subscription_errors = list(self._subscription_errors)

        return _json_safe_value({
            'active': True,
            'domain_id': self._domain_id,
            'profile': self._profile.to_dict(),
            'overall_status': overall_status,
            'summary': summary,
            'checks': checks,
            'watch_topics': watch_topics,
            'expected_nodes': expected_nodes,
            'artifact': artifact,
            'hints': hints,
            'subscription_errors': subscription_errors,
        })


    # --- Debug recorder helpers ---

    def _debug_root_dir(self):
        return Path(self._profile.artifacts_root) / '_ros_states_debug'

    def _debug_timestamp(self, ts=None):
        ts = time.time() if ts is None else ts
        return time.strftime('%Y%m%d_%H%M%S', time.localtime(ts))

    def _debug_iso(self, ts=None):
        ts = time.time() if ts is None else ts
        return time.strftime('%Y-%m-%dT%H:%M:%S%z', time.localtime(ts))

    def _build_debug_payload(self, reason='manual_snapshot', include_graph=True):
        payload = {
            'captured_at': self._debug_iso(),
            'captured_at_epoch': time.time(),
            'reason': reason,
            'active': self._active,
            'domain_id': self._domain_id,
            'profile': self._profile.to_dict(),
            'flight_debug': self.get_flight_debug_snapshot(),
        }
        if include_graph:
            payload.update({
                'topics': self.get_topics(),
                'services': self.get_services(),
                'actions': self.get_actions(),
                'tf': self.get_tf_tree(),
                'nodes': self.get_node_list(),
            })
        return _json_safe_value(payload)

    def _write_debug_manifest(self):
        with self._debug_lock:
            session_dir = self._debug_state.get('session_dir')
            manifest_data = dict(self._debug_state)
        if not session_dir:
            return
        manifest = {
            'recording_active': bool(manifest_data.get('active')),
            'started_at': manifest_data.get('started_at'),
            'stopped_at': manifest_data.get('stopped_at'),
            'interval_sec': manifest_data.get('interval_sec'),
            'session_dir': manifest_data.get('session_dir'),
            'snapshots_dir': manifest_data.get('snapshots_dir'),
            'timeline_path': manifest_data.get('timeline_path'),
            'snapshot_count': manifest_data.get('snapshot_count', 0),
            'last_snapshot_path': manifest_data.get('last_snapshot_path'),
            'last_snapshot_at': manifest_data.get('last_snapshot_at'),
            'last_error': manifest_data.get('last_error'),
            'report_path': manifest_data.get('report_path'),
            'report_summary_path': manifest_data.get('report_summary_path'),
            'report_generated_at': manifest_data.get('report_generated_at'),
            'domain_id': self._domain_id,
            'profile': self._profile.to_dict(),
        }
        manifest_path = Path(session_dir) / 'session_manifest.json'
        manifest_path.write_text(json.dumps(_json_safe_value(manifest), indent=2), encoding='utf-8')

    def _ensure_debug_session(self, origin='manual', interval_sec=None):
        root_dir = self._debug_root_dir()
        root_dir.mkdir(parents=True, exist_ok=True)
        started_at = time.time()
        session_name = f"{self._debug_timestamp(started_at)}_{self._profile.drone_name}_{origin}"
        session_dir = root_dir / session_name
        snapshots_dir = session_dir / 'snapshots'
        snapshots_dir.mkdir(parents=True, exist_ok=True)
        timeline_path = session_dir / 'timeline.jsonl'
        with self._debug_lock:
            self._debug_state.update({
                'active': False,
                'started_at': self._debug_iso(started_at),
                'stopped_at': None,
                'interval_sec': interval_sec,
                'session_dir': str(session_dir),
                'snapshots_dir': str(snapshots_dir),
                'timeline_path': str(timeline_path),
                'snapshot_count': 0,
                'last_snapshot_path': None,
                'last_snapshot_at': None,
                'last_error': None,
                'report_path': None,
                'report_summary_path': None,
                'report_generated_at': None,
            })
        self._write_debug_manifest()
        return session_dir, snapshots_dir, timeline_path

    def _register_debug_capture(self, path_value, captured_at, count_delta=1):
        with self._debug_lock:
            self._debug_state['snapshot_count'] = int(self._debug_state.get('snapshot_count', 0)) + count_delta
            self._debug_state['last_snapshot_path'] = str(path_value)
            self._debug_state['last_snapshot_at'] = captured_at
        self._write_debug_manifest()

    def _append_timeline_entry(self, reason='interval'):
        with self._debug_lock:
            timeline_path = self._debug_state.get('timeline_path')
        if not timeline_path:
            return None
        payload = self._build_debug_payload(reason=reason, include_graph=False)
        path_obj = Path(timeline_path)
        with path_obj.open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(payload, ensure_ascii=False) + '\n')
        self._register_debug_capture(path_obj, payload['captured_at'])
        return {
            'timeline_path': str(path_obj),
            'captured_at': payload['captured_at'],
        }

    def _debug_recording_loop(self):
        while not self._debug_stop_event.is_set():
            try:
                self._append_timeline_entry(reason='interval')
            except Exception as exc:
                with self._debug_lock:
                    self._debug_state['last_error'] = str(exc)
                self._write_debug_manifest()
            with self._debug_lock:
                interval_sec = float(self._debug_state.get('interval_sec') or 5.0)
            if self._debug_stop_event.wait(max(interval_sec, 0.5)):
                break

    def get_debug_recording_status(self):
        with self._debug_lock:
            data = dict(self._debug_state)
        data['recording_active'] = bool(data.get('active'))
        data['root_dir'] = str(self._debug_root_dir())
        data['domain_id'] = self._domain_id
        data['profile'] = self._profile.to_dict()
        data['report_url'] = '/debug/report/current' if data.get('report_path') else None
        return _json_safe_value(data)

    def _set_debug_error(self, message):
        with self._debug_lock:
            self._debug_state['last_error'] = message
        self._write_debug_manifest()

    def get_debug_report_path(self):
        with self._debug_lock:
            return self._debug_state.get('report_path')

    def generate_debug_report(self, reason='manual_generate'):
        with self._debug_lock:
            session_dir = self._debug_state.get('session_dir')
        if not session_dir:
            raise RuntimeError('Save a snapshot or start a recording before generating a report.')

        result = generate_session_report(session_dir)
        with self._debug_lock:
            self._debug_state['report_path'] = result.get('report_path')
            self._debug_state['report_summary_path'] = result.get('report_summary_path')
            self._debug_state['report_generated_at'] = result.get('report_generated_at')
            self._debug_state['last_error'] = None
        self._write_debug_manifest()

        result = dict(result)
        result['report_url'] = '/debug/report/current'
        result['reason'] = reason
        result['recording'] = self.get_debug_recording_status()
        return _json_safe_value(result)

    def save_debug_snapshot(self, reason='manual_snapshot', include_graph=True):
        with self._debug_lock:
            session_dir = self._debug_state.get('session_dir')
            snapshots_dir = self._debug_state.get('snapshots_dir')
        if not session_dir or not snapshots_dir:
            _, snapshots_path, _ = self._ensure_debug_session(origin='manual', interval_sec=None)
            snapshots_dir = str(snapshots_path)
        payload = self._build_debug_payload(reason=reason, include_graph=include_graph)
        file_name = f"snapshot_{self._debug_timestamp()}_{reason}.json"
        snapshot_path = Path(snapshots_dir) / file_name
        snapshot_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding='utf-8')
        self._register_debug_capture(snapshot_path, payload['captured_at'])
        report = None
        try:
            report = self.generate_debug_report(reason=reason)
        except Exception as exc:
            self._set_debug_error(f'debug report generation failed: {exc}')
        return {
            'snapshot_path': str(snapshot_path),
            'captured_at': payload['captured_at'],
            'report': report,
            'recording': self.get_debug_recording_status(),
        }

    def start_debug_recording(self, interval_sec=5.0):
        interval_sec = max(float(interval_sec), 0.5)
        if self._debug_state.get('active'):
            return self.get_debug_recording_status()
        self._ensure_debug_session(origin='recording', interval_sec=interval_sec)
        with self._debug_lock:
            self._debug_state['active'] = True
            self._debug_state['stopped_at'] = None
            self._debug_state['interval_sec'] = interval_sec
            self._debug_state['last_error'] = None
        self._debug_stop_event.clear()
        self.save_debug_snapshot(reason='recording_start', include_graph=True)
        self._debug_thread = threading.Thread(target=self._debug_recording_loop, daemon=True)
        self._debug_thread.start()
        self._write_debug_manifest()
        return self.get_debug_recording_status()

    def stop_debug_recording(self, reason='manual_stop'):
        was_active = bool(self._debug_state.get('active'))
        if was_active:
            self._debug_stop_event.set()
            with self._debug_lock:
                self._debug_state['active'] = False
                self._debug_state['stopped_at'] = self._debug_iso()
            self._write_debug_manifest()
            self.save_debug_snapshot(reason=reason, include_graph=True)
        return self.get_debug_recording_status()

    # --- API methods ---

    def get_topics(self):
        if not self._active or not self._node:
            return []

        try:
            names_and_types = self._node.get_topic_names_and_types()
        except Exception:
            return []

        topics = []
        watch_names = set(self._profile.topic_map().values())
        for name, types in names_and_types:
            if '/_action/' in name:
                continue
            type_str = types[0] if types else 'unknown'

            try:
                pubs = self._node.get_publishers_info_by_topic(name)
                subs = self._node.get_subscriptions_info_by_topic(name)
            except Exception:
                pubs, subs = [], []

            ext_pubs = [p for p in pubs if not _is_internal(p.node_name)]
            ext_subs = [s for s in subs if not _is_internal(s.node_name)]

            pub_nodes = sorted({
                p.node_namespace.rstrip('/') + '/' + p.node_name
                for p in ext_pubs
            })
            sub_nodes = sorted({
                s.node_namespace.rstrip('/') + '/' + s.node_name
                for s in ext_subs
            })

            with self._hz_lock:
                hz = self._hz_cache.get(name)

            topics.append({
                'name': name,
                'type': type_str,
                'publishers': len(ext_pubs),
                'subscribers': len(ext_subs),
                'hz': round(hz, 1) if hz is not None else None,
                'pub_nodes': pub_nodes,
                'sub_nodes': sub_nodes,
                'is_watch_topic': name in watch_names,
            })

        topics.sort(key=lambda item: (not item['is_watch_topic'], item['name']))
        return topics

    def get_services(self):
        if not self._active or not self._node:
            return []

        try:
            names_and_types = self._node.get_service_names_and_types()
        except Exception:
            return []

        server_map, client_map = self._build_node_maps()

        services = []
        for name, types in names_and_types:
            if '/ros_web_monitor/' in name or '/_ros2cli_' in name:
                continue
            if '/_action/' in name:
                continue
            type_str = types[0] if types else 'unknown'
            services.append({
                'name': name,
                'type': type_str,
                'server_nodes': sorted(server_map.get(name, [])),
                'client_nodes': sorted(client_map.get(name, [])),
            })

        services.sort(key=lambda item: item['name'])
        return services

    def get_actions(self):
        """Detect actions by finding _action/send_goal services."""
        if not self._active or not self._node:
            return []

        try:
            names_and_types = self._node.get_service_names_and_types()
        except Exception:
            return []

        server_map, client_map = self._build_node_maps()

        actions = []
        for name, types in names_and_types:
            if name.endswith('/_action/send_goal'):
                action_name = name.rsplit('/_action/send_goal', 1)[0]
                type_str = types[0] if types else 'unknown'
                type_str = re.sub(r'_SendGoal$', '', type_str)
                actions.append({
                    'name': action_name,
                    'type': type_str,
                    'server_nodes': sorted(server_map.get(name, [])),
                    'client_nodes': sorted(client_map.get(name, [])),
                })

        actions.sort(key=lambda item: item['name'])
        return actions

    def get_tf_tree(self):
        with self._tf_lock:
            frames = dict(self._tf_frames)

        if not frames:
            return {'frames': [], 'tree': []}

        tree = defaultdict(list)
        all_children = set()
        all_parents = set()

        for child, parent in frames.items():
            tree[parent].append(child)
            all_children.add(child)
            all_parents.add(parent)

        roots = all_parents - all_children

        def build_subtree(node):
            children = sorted(tree.get(node, []))
            return {'name': node, 'children': [build_subtree(c) for c in children]}

        return {
            'frames': [{'child': c, 'parent': p} for c, p in sorted(frames.items())],
            'tree': [build_subtree(r) for r in sorted(roots)],
        }

    # --- Parameter API ---

    def get_node_list(self):
        """Get list of all external nodes."""
        if not self._active or not self._node:
            return []
        try:
            nodes = self._node.get_node_names_and_namespaces()
        except Exception:
            return []

        result = []
        for node_name, namespace in nodes:
            if _is_internal(node_name):
                continue
            full_name = namespace.rstrip('/') + '/' + node_name
            result.append(full_name)
        return sorted(result)

    def _call_service(self, srv_type, srv_name, request, timeout=5.0):
        """Call a ROS2 service synchronously with timeout."""
        if not self._node:
            return None
        client = self._node.create_client(srv_type, srv_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(request)
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if future.done():
                return future.result()
            return None
        finally:
            try:
                self._node.destroy_client(client)
            except Exception:
                pass

    def get_node_parameters(self, node_full_name):
        """Get all parameters for a given node using ListParameters service."""
        if not self._active or not self._node:
            return []

        try:
            srv_name = node_full_name + '/list_parameters'
            req = ListParameters.Request()
            resp = self._call_service(ListParameters, srv_name, req)
            if resp:
                return sorted(resp.result.names)
        except Exception:
            pass
        return []

    _TYPE_NAMES = {
        ParameterType.PARAMETER_BOOL: 'Boolean',
        ParameterType.PARAMETER_INTEGER: 'Integer',
        ParameterType.PARAMETER_DOUBLE: 'Double',
        ParameterType.PARAMETER_STRING: 'String',
        ParameterType.PARAMETER_BYTE_ARRAY: 'Byte array',
        ParameterType.PARAMETER_BOOL_ARRAY: 'Boolean array',
        ParameterType.PARAMETER_INTEGER_ARRAY: 'Integer array',
        ParameterType.PARAMETER_DOUBLE_ARRAY: 'Double array',
        ParameterType.PARAMETER_STRING_ARRAY: 'String array',
    }

    def get_parameter_value(self, node_full_name, param_name):
        """Get a single parameter value using GetParameters service."""
        if not self._active or not self._node:
            return None

        try:
            srv_name = node_full_name + '/get_parameters'
            req = GetParameters.Request()
            req.names = [param_name]
            resp = self._call_service(GetParameters, srv_name, req)
            if resp and resp.values:
                val = resp.values[0]
                ptype = val.type
                type_name = self._TYPE_NAMES.get(ptype, 'Unknown')

                if ptype == ParameterType.PARAMETER_BOOL:
                    value_str = str(val.bool_value)
                elif ptype == ParameterType.PARAMETER_INTEGER:
                    value_str = str(val.integer_value)
                elif ptype == ParameterType.PARAMETER_DOUBLE:
                    value_str = str(val.double_value)
                elif ptype == ParameterType.PARAMETER_STRING:
                    value_str = val.string_value
                elif ptype == ParameterType.PARAMETER_BYTE_ARRAY:
                    value_str = str(list(val.byte_array_value))
                elif ptype == ParameterType.PARAMETER_BOOL_ARRAY:
                    value_str = str(list(val.bool_array_value))
                elif ptype == ParameterType.PARAMETER_INTEGER_ARRAY:
                    value_str = str(list(val.integer_array_value))
                elif ptype == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    value_str = str(list(val.double_array_value))
                elif ptype == ParameterType.PARAMETER_STRING_ARRAY:
                    value_str = str(list(val.string_array_value))
                else:
                    value_str = '(not set)'

                return {'type': type_name, 'value': value_str}
        except Exception:
            pass
        return None

    def set_parameter_value(self, node_full_name, param_name, value):
        """Set a parameter value using SetParameters service."""
        if not self._active or not self._node:
            return {'success': False, 'message': 'Not active'}

        from rcl_interfaces.msg import Parameter, ParameterValue
        from rcl_interfaces.srv import SetParameters

        try:
            current = self.get_parameter_value(node_full_name, param_name)
            param_val = ParameterValue()

            if current and current['type'] == 'Integer':
                param_val.type = ParameterType.PARAMETER_INTEGER
                param_val.integer_value = int(value)
            elif current and current['type'] == 'Double':
                param_val.type = ParameterType.PARAMETER_DOUBLE
                param_val.double_value = float(value)
            elif current and current['type'] == 'Boolean':
                param_val.type = ParameterType.PARAMETER_BOOL
                param_val.bool_value = str(value).lower() in ('true', '1', 'yes')
            else:
                param_val.type = ParameterType.PARAMETER_STRING
                param_val.string_value = str(value)

            srv_name = node_full_name + '/set_parameters'
            req = SetParameters.Request()
            param = Parameter()
            param.name = param_name
            param.value = param_val
            req.parameters = [param]

            resp = self._call_service(SetParameters, srv_name, req)
            if resp and resp.results:
                result = resp.results[0]
                if result.successful:
                    return {'success': True, 'message': 'Parameter set successfully'}
                return {'success': False, 'message': result.reason or 'Failed'}
            return {'success': False, 'message': 'No response from service'}
        except Exception as exc:
            return {'success': False, 'message': str(exc)}
