"""Flask app for ROS2 State Observer."""

import argparse
import os
import threading
import webbrowser

from pathlib import Path

from flask import Flask, jsonify, render_template, request, send_file
from ros_states.ros_monitor import RosMonitor

monitor = RosMonitor()

_PROFILE_KEYS = [
    'drone_name',
    'mavros_namespace',
    'artifacts_root',
    'pose_timeout_sec',
    'scan_timeout_sec',
    'planner_cmd_timeout_sec',
    'startup_grace_sec',
    'emergency_stop_distance',
    'obstacle_stop_distance',
]

# Default config
_config = {
    'port': 5050,
    'update_interval': 5000,
    'drone_name': 'drone1',
    'mavros_namespace': '/mavros',
    'artifacts_root': '/workspace/AV_Drone/artifacts',
    'pose_timeout_sec': 0.5,
    'scan_timeout_sec': 0.5,
    'planner_cmd_timeout_sec': 0.5,
    'startup_grace_sec': 3.0,
    'emergency_stop_distance': 1.0,
    'obstacle_stop_distance': 2.0,
}


def _profile_from_payload(payload=None):
    payload = payload or {}
    return {
        key: payload.get(key, _config[key])
        for key in _PROFILE_KEYS
    }


def create_app():
    template_dir = None
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('ros_states')
        share_templates = os.path.join(pkg_share, 'templates')
        if os.path.isdir(share_templates):
            template_dir = share_templates
    except Exception:
        pass

    if template_dir is None:
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        local_templates = os.path.join(project_root, 'templates')
        if os.path.isdir(local_templates):
            template_dir = local_templates

    app = Flask(__name__, template_folder=template_dir)

    @app.route('/')
    def index():
        return render_template('index.html', update_interval=_config['update_interval'])

    @app.route('/api/activate', methods=['POST'])
    def activate():
        data = request.get_json(silent=True) or {}
        domain_id = data.get('domain_id', 0)
        try:
            monitor.activate(domain_id, profile=_profile_from_payload(data))
            return jsonify({
                'status': 'ok',
                'domain_id': monitor.domain_id,
                'profile': monitor.profile,
            })
        except Exception as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500

    @app.route('/api/deactivate', methods=['POST'])
    def deactivate():
        try:
            monitor.deactivate()
            return jsonify({'status': 'ok'})
        except Exception as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500

    @app.route('/api/status')
    def status():
        return jsonify({
            'active': monitor.active,
            'domain_id': monitor.domain_id,
            'profile': monitor.profile,
        })

    @app.route('/api/config')
    def config():
        return jsonify(_config)

    @app.route('/api/profile')
    def profile():
        return jsonify(monitor.profile)

    @app.route('/api/flight_debug')
    def flight_debug():
        return jsonify(monitor.get_flight_debug_snapshot())

    @app.route('/api/artifacts/latest')
    def latest_artifact():
        return jsonify(monitor.get_latest_artifact_summary())

    @app.route('/api/debug/status')
    def debug_status():
        return jsonify(monitor.get_debug_recording_status())

    @app.route('/api/debug/snapshot', methods=['POST'])
    def debug_snapshot():
        data = request.get_json(silent=True) or {}
        try:
            result = monitor.save_debug_snapshot(
                reason=str(data.get('reason') or 'manual_snapshot'),
                include_graph=bool(data.get('include_graph', True)),
            )
            return jsonify({'status': 'ok', **result})
        except Exception as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500

    @app.route('/api/debug/start', methods=['POST'])
    def debug_start():
        data = request.get_json(silent=True) or {}
        try:
            interval_sec = float(data.get('interval_sec', max(_config['update_interval'] / 1000.0, 1.0)))
            result = monitor.start_debug_recording(interval_sec=interval_sec)
            return jsonify({'status': 'ok', **result})
        except Exception as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500

    @app.route('/api/debug/stop', methods=['POST'])
    def debug_stop():
        data = request.get_json(silent=True) or {}
        try:
            result = monitor.stop_debug_recording(reason=str(data.get('reason') or 'manual_stop'))
            return jsonify({'status': 'ok', **result})
        except Exception as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500

    @app.route('/api/debug/report/generate', methods=['POST'])
    def debug_report_generate():
        data = request.get_json(silent=True) or {}
        try:
            result = monitor.generate_debug_report(reason=str(data.get('reason') or 'manual_generate'))
            return jsonify({'status': 'ok', **result})
        except Exception as exc:
            return jsonify({'status': 'error', 'message': str(exc)}), 500

    @app.route('/debug/report/current')
    def debug_report_current():
        report_path = monitor.get_debug_report_path()
        if not report_path:
            return 'No debug report has been generated yet.', 404
        report_file = Path(report_path)
        if not report_file.exists():
            return f'Report file not found: {report_path}', 404
        return send_file(report_file)

    @app.route('/api/topics')
    def topics():
        return jsonify(monitor.get_topics())

    @app.route('/api/services')
    def services():
        return jsonify(monitor.get_services())

    @app.route('/api/actions')
    def actions():
        return jsonify(monitor.get_actions())

    @app.route('/api/tf')
    def tf():
        return jsonify(monitor.get_tf_tree())

    @app.route('/api/nodes')
    def nodes():
        return jsonify(monitor.get_node_list())

    @app.route('/api/params/list')
    def param_list():
        node_name = request.args.get('node', '')
        if not node_name:
            return jsonify([])
        return jsonify(monitor.get_node_parameters(node_name))

    @app.route('/api/params/get')
    def param_get():
        node_name = request.args.get('node', '')
        param_name = request.args.get('param', '')
        if not node_name or not param_name:
            return jsonify({'error': 'Missing node or param'}), 400
        result = monitor.get_parameter_value(node_name, param_name)
        if result is None:
            return jsonify({'error': 'Could not get parameter'}), 404
        return jsonify(result)

    @app.route('/api/params/set', methods=['POST'])
    def param_set():
        data = request.get_json(silent=True) or {}
        node_name = data.get('node', '')
        param_name = data.get('param', '')
        value = data.get('value', '')
        if not node_name or not param_name:
            return jsonify({'success': False, 'message': 'Missing node or param'}), 400
        return jsonify(monitor.set_parameter_value(node_name, param_name, value))

    return app


def main():
    parser = argparse.ArgumentParser(description='ROS2 State Observer Web Server')
    parser.add_argument('--port', type=int, default=5050, help='Web server port (default: 5050)')
    parser.add_argument('--update-interval', type=int, default=5000,
                        help='Dashboard update interval in ms (default: 5000)')
    parser.add_argument('--open-browser', action='store_true', default=False,
                        help='Open web browser on startup')
    parser.add_argument('--domain-id', type=int, default=0,
                        help='Initial ROS_DOMAIN_ID (default: 0)')
    parser.add_argument('--drone-name', type=str, default='drone1',
                        help='Drone name / namespace suffix (default: drone1)')
    parser.add_argument('--mavros-namespace', type=str, default='/mavros',
                        help='MAVROS namespace (default: /mavros)')
    parser.add_argument('--artifacts-root', type=str, default='/workspace/AV_Drone/artifacts',
                        help='Artifact root directory (default: /workspace/AV_Drone/artifacts)')
    parser.add_argument('--pose-timeout-sec', type=float, default=0.5,
                        help='Freshness threshold for pose topic')
    parser.add_argument('--scan-timeout-sec', type=float, default=0.5,
                        help='Freshness threshold for scan topic')
    parser.add_argument('--planner-cmd-timeout-sec', type=float, default=0.5,
                        help='Freshness threshold for cmd velocity topics')
    parser.add_argument('--startup-grace-sec', type=float, default=3.0,
                        help='Grace period before missing data becomes an error')
    parser.add_argument('--emergency-stop-distance', type=float, default=1.0,
                        help='Critical obstacle distance in meters')
    parser.add_argument('--obstacle-stop-distance', type=float, default=2.0,
                        help='Planner stop distance in meters')
    args = parser.parse_args()

    _config['port'] = args.port
    _config['update_interval'] = args.update_interval
    _config['drone_name'] = args.drone_name
    _config['mavros_namespace'] = args.mavros_namespace
    _config['artifacts_root'] = args.artifacts_root
    _config['pose_timeout_sec'] = args.pose_timeout_sec
    _config['scan_timeout_sec'] = args.scan_timeout_sec
    _config['planner_cmd_timeout_sec'] = args.planner_cmd_timeout_sec
    _config['startup_grace_sec'] = args.startup_grace_sec
    _config['emergency_stop_distance'] = args.emergency_stop_distance
    _config['obstacle_stop_distance'] = args.obstacle_stop_distance

    app = create_app()

    if args.open_browser:
        def _delayed_open():
            import time
            time.sleep(1.5)
            webbrowser.open(f'http://localhost:{args.port}')
        threading.Thread(target=_delayed_open, daemon=True).start()

    print(f'Starting ROS2 State Observer on port {args.port}')
    print(f'Update interval: {args.update_interval}ms')
    print(f'Drone profile: {args.drone_name} via {args.mavros_namespace}')
    print(f'Artifact root: {args.artifacts_root}')
    print(f'Open http://localhost:{args.port} in your browser')
    app.run(host='0.0.0.0', port=args.port, debug=False)


if __name__ == '__main__':
    main()
