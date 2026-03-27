"""Utilities for turning ros_states debug sessions into readable HTML reports."""

from __future__ import annotations

import html
import json
import math
import re
import time
from collections import Counter
from pathlib import Path

_STATUS_ORDER = {'ok': 0, 'info': 1, 'warn': 2, 'error': 3}
_STATUS_COLORS = {
    'ok': '#1d7f5f',
    'info': '#345f9e',
    'warn': '#c78015',
    'error': '#c64642',
}
_PHASE_COLORS = {
    'WAIT_STREAM': '#e6edf7',
    'TAKEOFF': '#d8f0e5',
    'FOLLOW_PLAN': '#d4e4ff',
    'HOVER_AT_GOAL': '#d9f4e3',
    'RETURN_HOME': '#f1e2ff',
}
_VECTOR_RE = re.compile(
    r'vx\s+(?P<vx>-?\d+(?:\.\d+)?)\s*\|\s*vy\s+(?P<vy>-?\d+(?:\.\d+)?)\s*\|\s*vz\s+(?P<vz>-?\d+(?:\.\d+)?)',
    re.IGNORECASE,
)
_FLOAT_RE = re.compile(r'-?\d+(?:\.\d+)?')


def _read_json(path: Path, default=None):
    if not path.exists():
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return default


def _read_jsonl(path: Path):
    if not path.exists():
        return []
    records = []
    for line in path.read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            records.append(json.loads(line))
        except Exception:
            continue
    return records


def _safe_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _parse_number(text):
    if not text:
        return None
    match = _FLOAT_RE.search(str(text))
    return _safe_float(match.group(0)) if match else None


def _parse_speed(text):
    if not text:
        return None
    match = _VECTOR_RE.search(str(text))
    if not match:
        return None
    vx = _safe_float(match.group('vx')) or 0.0
    vy = _safe_float(match.group('vy')) or 0.0
    vz = _safe_float(match.group('vz')) or 0.0
    return math.sqrt(vx * vx + vy * vy + vz * vz)


def _check_map(flight_debug):
    return {
        check.get('id'): check
        for check in (flight_debug or {}).get('checks', [])
        if isinstance(check, dict) and check.get('id')
    }


def _fmt_value(value, suffix=''):
    if value is None:
        return '-'
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float)):
        return f'{value:.2f}{suffix}'
    return str(value)


def _status_badge(status):
    color = _STATUS_COLORS.get(status, '#345f9e')
    status_text = (status or 'info').upper()
    return (
        f'<span class="badge" style="background:{color}22;color:{color};border-color:{color}33;">'
        f'{html.escape(status_text)}</span>'
    )


def _timeline_samples(records):
    if not records:
        return []
    sorted_records = sorted(records, key=lambda item: item.get('captured_at_epoch') or 0.0)
    base_time = sorted_records[0].get('captured_at_epoch') or 0.0
    samples = []
    for record in sorted_records:
        flight_debug = record.get('flight_debug') or {}
        checks = list(flight_debug.get('checks', []))
        check_map = _check_map(flight_debug)
        obstacle = check_map.get('obstacle', {})
        planner = check_map.get('autonomy_cmd', {})
        safe_cmd = check_map.get('safe_cmd', {})
        mission = check_map.get('mission_phase', {})
        goal = check_map.get('goal_reached', {})
        samples.append({
            'captured_at': record.get('captured_at'),
            'captured_at_epoch': record.get('captured_at_epoch') or 0.0,
            'elapsed_sec': max((record.get('captured_at_epoch') or 0.0) - base_time, 0.0),
            'overall_status': flight_debug.get('overall_status') or 'info',
            'summary': flight_debug.get('summary') or '-',
            'warn_count': sum(1 for check in checks if check.get('status') == 'warn'),
            'error_count': sum(1 for check in checks if check.get('status') == 'error'),
            'nearest_obstacle_m': _parse_number(obstacle.get('headline')),
            'planner_speed_mps': _parse_speed(planner.get('headline')),
            'safe_speed_mps': _parse_speed(safe_cmd.get('headline')),
            'phase': (mission.get('headline') or '').strip() or 'UNKNOWN',
            'goal_reached': (goal.get('headline') or '').strip().lower() == 'goal reached',
        })
    return samples


def _svg_line_chart(title, samples, series_defs, threshold_defs=None, y_unit=''):
    values = []
    for sample in samples:
        for series in series_defs:
            value = sample.get(series['key'])
            if value is not None:
                values.append(float(value))
    threshold_defs = threshold_defs or []
    for threshold in threshold_defs:
        value = threshold.get('value')
        if value is not None:
            values.append(float(value))
    if not values:
        return '<div class="empty">No numeric data was captured for this graph yet.</div>'

    width = 880
    height = 280
    left = 54
    right = 18
    top = 24
    bottom = 36
    plot_w = width - left - right
    plot_h = height - top - bottom

    min_v = min(values)
    max_v = max(values)
    if math.isclose(min_v, max_v):
        min_v -= 1.0
        max_v += 1.0
    padding = max((max_v - min_v) * 0.12, 0.2)
    min_v -= padding
    max_v += padding

    total_t = samples[-1]['elapsed_sec'] if len(samples) > 1 else 1.0
    total_t = max(total_t, 1.0)

    def x_pos(sample):
        return left + (sample['elapsed_sec'] / total_t) * plot_w

    def y_pos(value):
        return top + (1.0 - ((value - min_v) / (max_v - min_v))) * plot_h

    grid_lines = []
    for idx in range(5):
        ratio = idx / 4 if 4 else 0.0
        value = max_v - ratio * (max_v - min_v)
        y = top + ratio * plot_h
        grid_lines.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" stroke="#d9e4f2" stroke-width="1" />'
            f'<text x="{left - 8}" y="{y + 4:.1f}" text-anchor="end" class="axis-label">{value:.2f}{html.escape(y_unit)}</text>'
        )

    x_labels = []
    for idx in range(5):
        ratio = idx / 4 if 4 else 0.0
        sec = ratio * total_t
        x = left + ratio * plot_w
        x_labels.append(
            f'<line x1="{x:.1f}" y1="{top + plot_h}" x2="{x:.1f}" y2="{top + plot_h + 5}" stroke="#9eb2cc" stroke-width="1" />'
            f'<text x="{x:.1f}" y="{height - 10}" text-anchor="middle" class="axis-label">{sec:.0f}s</text>'
        )

    threshold_lines = []
    for threshold in threshold_defs:
        value = threshold.get('value')
        if value is None:
            continue
        y = y_pos(float(value))
        color = threshold.get('color', '#9b6bff')
        label = threshold.get('label', 'threshold')
        threshold_lines.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" stroke="{color}" stroke-width="1.2" stroke-dasharray="6 4" />'
            f'<text x="{left + plot_w - 4}" y="{y - 6:.1f}" text-anchor="end" class="axis-label" fill="{color}">{html.escape(label)}</text>'
        )

    series_lines = []
    legend_items = []
    for series in series_defs:
        color = series.get('color', '#345f9e')
        points = []
        for sample in samples:
            value = sample.get(series['key'])
            if value is None:
                continue
            points.append((x_pos(sample), y_pos(float(value))))
        if not points:
            continue
        point_text = ' '.join(f'{x:.1f},{y:.1f}' for x, y in points)
        circles = ''.join(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.8" fill="{color}" />'
            for x, y in points
        )
        series_lines.append(
            f'<polyline points="{point_text}" fill="none" stroke="{color}" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" />{circles}'
        )
        legend_items.append(
            f'<div class="legend-item"><span class="legend-swatch" style="background:{color};"></span>{html.escape(series.get("label", series["key"]))}</div>'
        )

    return f'''<div class="chart-card">
      <div class="chart-head">
        <h4>{html.escape(title)}</h4>
        <div class="legend">{''.join(legend_items)}</div>
      </div>
      <svg viewBox="0 0 {width} {height}" class="chart-svg" role="img" aria-label="{html.escape(title)}">
        <rect x="0" y="0" width="{width}" height="{height}" rx="14" fill="#fbfdff" />
        <line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" stroke="#9eb2cc" stroke-width="1.2" />
        <line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" stroke="#9eb2cc" stroke-width="1.2" />
        {''.join(grid_lines)}
        {''.join(x_labels)}
        {''.join(threshold_lines)}
        {''.join(series_lines)}
      </svg>
    </div>'''


def _svg_band_timeline(title, samples, key, color_map, default_color='#9eb2cc'):
    if not samples:
        return '<div class="empty">No timeline data is available yet.</div>'

    width = 880
    height = 110
    left = 18
    right = 18
    top = 30
    band_h = 28
    plot_w = width - left - right
    total_t = samples[-1]['elapsed_sec'] if len(samples) > 1 else 1.0
    total_t = max(total_t, 1.0)

    segments = []
    current_value = samples[0].get(key) or 'UNKNOWN'
    start_t = 0.0
    for sample in samples[1:]:
        value = sample.get(key) or 'UNKNOWN'
        if value != current_value:
            segments.append((current_value, start_t, sample['elapsed_sec']))
            current_value = value
            start_t = sample['elapsed_sec']
    segments.append((current_value, start_t, total_t))

    rects = []
    labels = []
    for value, seg_start, seg_end in segments:
        x = left + (seg_start / total_t) * plot_w
        end_x = left + (seg_end / total_t) * plot_w
        width_value = max(end_x - x, 8.0)
        color = color_map.get(value, default_color)
        rects.append(f'<rect x="{x:.1f}" y="{top}" width="{width_value:.1f}" height="{band_h}" rx="10" fill="{color}" opacity="0.85" />')
        labels.append(f'<text x="{x + (width_value / 2):.1f}" y="{top + 18:.1f}" text-anchor="middle" class="band-label">{html.escape(value)}</text>')

    ticks = []
    for idx in range(5):
        ratio = idx / 4 if 4 else 0.0
        sec = ratio * total_t
        x = left + ratio * plot_w
        ticks.append(
            f'<line x1="{x:.1f}" y1="{top + band_h + 6}" x2="{x:.1f}" y2="{top + band_h + 11}" stroke="#8ca3bf" stroke-width="1" />'
            f'<text x="{x:.1f}" y="{top + band_h + 26:.1f}" text-anchor="middle" class="axis-label">{sec:.0f}s</text>'
        )

    return f'''<div class="chart-card">
      <div class="chart-head"><h4>{html.escape(title)}</h4></div>
      <svg viewBox="0 0 {width} {height}" class="chart-svg" role="img" aria-label="{html.escape(title)}">
        <rect x="0" y="0" width="{width}" height="{height}" rx="14" fill="#fbfdff" />
        {''.join(rects)}
        {''.join(labels)}
        {''.join(ticks)}
      </svg>
    </div>'''


def _latest_full_snapshot(snapshots):
    for snapshot in reversed(snapshots):
        if any(key in snapshot for key in ('topics', 'services', 'actions', 'nodes')):
            return snapshot
    return snapshots[-1] if snapshots else {}


def generate_session_report(session_dir):
    session_path = Path(session_dir)
    manifest = _read_json(session_path / 'session_manifest.json', default={}) or {}
    timeline = _read_jsonl(session_path / 'timeline.jsonl')
    snapshots_dir = session_path / 'snapshots'
    snapshots = []
    if snapshots_dir.exists():
        for snapshot_path in sorted(snapshots_dir.glob('*.json')):
            payload = _read_json(snapshot_path, default={}) or {}
            payload['_snapshot_path'] = str(snapshot_path)
            snapshots.append(payload)

    records = timeline or snapshots
    samples = _timeline_samples(records)
    latest_payload = _latest_full_snapshot(snapshots) if snapshots else (records[-1] if records else {})
    latest_flight = (latest_payload or {}).get('flight_debug') or {}
    latest_checks = list(latest_flight.get('checks', []))
    latest_sample = samples[-1] if samples else {}
    profile = (latest_payload or {}).get('profile') or manifest.get('profile') or {}
    generated_at = time.strftime('%Y-%m-%dT%H:%M:%S%z', time.localtime())

    report_path = session_path / 'report.html'
    summary_path = session_path / 'report_summary.json'

    latest_obstacle = latest_sample.get('nearest_obstacle_m')
    latest_planner_speed = latest_sample.get('planner_speed_mps')
    latest_safe_speed = latest_sample.get('safe_speed_mps')
    latest_status = latest_flight.get('overall_status') or 'info'
    latest_phase = latest_sample.get('phase') or '-'
    latest_summary = latest_flight.get('summary') or '-'
    status_counts = Counter(sample.get('overall_status') or 'info' for sample in samples)

    overview_cards = [
        ('Latest Status', _status_badge(latest_status)),
        ('Latest Phase', html.escape(latest_phase)),
        ('Goal Reached', html.escape('Yes' if latest_sample.get('goal_reached') else 'No')),
        ('Latest Obstacle', html.escape(_fmt_value(latest_obstacle, ' m'))),
        ('Planner Speed', html.escape(_fmt_value(latest_planner_speed, ' m/s'))),
        ('Safety Speed', html.escape(_fmt_value(latest_safe_speed, ' m/s'))),
        ('Timeline Points', html.escape(str(len(timeline)))),
        ('Snapshots', html.escape(str(len(snapshots)))),
    ]

    inventory_cards = [
        ('Session Dir', html.escape(str(session_path))),
        ('Started At', html.escape(str(manifest.get('started_at') or '-'))),
        ('Stopped At', html.escape(str(manifest.get('stopped_at') or '-'))),
        ('Interval', html.escape(_fmt_value(manifest.get('interval_sec'), ' s'))),
        ('Drone', html.escape(str(profile.get('drone_name') or '-'))),
        ('MAVROS', html.escape(str(profile.get('mavros_namespace') or '-'))),
        ('Topics in Latest Snapshot', html.escape(str(len((latest_payload or {}).get('topics', []))))),
        ('Nodes in Latest Snapshot', html.escape(str(len((latest_payload or {}).get('nodes', []))))),
    ]

    latest_artifact = latest_flight.get('artifact') or {}
    hints = list(latest_flight.get('hints', []))
    topic_rows = list(latest_flight.get('watch_topics', []))
    subscription_errors = list(latest_flight.get('subscription_errors', []))

    charts = [
        _svg_band_timeline('Overall Health Timeline', samples, 'overall_status', _STATUS_COLORS, '#9eb2cc'),
        _svg_band_timeline('Mission Phase Timeline', samples, 'phase', _PHASE_COLORS, '#9eb2cc'),
        _svg_line_chart(
            'Nearest Obstacle Distance',
            samples,
            [{'key': 'nearest_obstacle_m', 'label': 'Obstacle distance', 'color': '#345f9e'}],
            threshold_defs=[
                {'value': _safe_float(profile.get('obstacle_stop_distance')), 'label': 'Planner stop band', 'color': '#c78015'},
                {'value': _safe_float(profile.get('emergency_stop_distance')), 'label': 'Emergency stop', 'color': '#c64642'},
            ],
            y_unit=' m',
        ),
        _svg_line_chart(
            'Planner vs Safety Speed',
            samples,
            [
                {'key': 'planner_speed_mps', 'label': 'Planner speed', 'color': '#1d7f5f'},
                {'key': 'safe_speed_mps', 'label': 'Safety speed', 'color': '#345f9e'},
            ],
            y_unit=' m/s',
        ),
        _svg_line_chart(
            'Warn / Error Check Count',
            samples,
            [
                {'key': 'warn_count', 'label': 'Warn count', 'color': '#c78015'},
                {'key': 'error_count', 'label': 'Error count', 'color': '#c64642'},
            ],
        ),
    ]

    check_rows = ''.join(
        f'<tr><td>{html.escape(check.get("label") or "-")}</td><td>{_status_badge(check.get("status"))}</td><td>{html.escape(check.get("headline") or "-")}</td><td>{html.escape(check.get("detail") or "-")}</td></tr>'
        for check in latest_checks
    ) or '<tr><td colspan="4" class="empty-cell">No health checks were captured.</td></tr>'

    watch_rows = ''.join(
        f'<tr><td>{html.escape(item.get("label") or "-")}</td><td>{_status_badge(item.get("status"))}</td><td>{html.escape(item.get("topic") or "-")}</td><td>{html.escape(item.get("headline") or "-")}</td><td>{html.escape(item.get("detail") or "-")}</td></tr>'
        for item in topic_rows
    ) or '<tr><td colspan="5" class="empty-cell">No watch topic snapshot is available.</td></tr>'

    hint_rows = ''.join(
        f'<div class="hint-card"><div class="hint-head">{_status_badge(item.get("level"))}<strong>{html.escape(item.get("title") or "-")}</strong></div><p>{html.escape(item.get("detail") or "-")}</p><ul>{"".join(f"<li><code>{html.escape(command)}</code></li>" for command in item.get("commands", []))}</ul></div>'
        for item in hints
    ) or '<div class="empty">No troubleshooting hints were captured.</div>'

    status_counter_html = ''.join(
        f'<div class="mini-stat"><div class="mini-label">{html.escape(key.upper())}</div><div class="mini-value" style="color:{_STATUS_COLORS.get(key, "#345f9e")};">{value}</div></div>'
        for key, value in sorted(status_counts.items(), key=lambda item: _STATUS_ORDER.get(item[0], 99))
    ) or '<div class="empty">No status summary yet.</div>'

    if latest_artifact:
        artifact_html = f'''
        <div class="artifact-box">
          <div class="artifact-grid">
            <div class="mini-stat"><div class="mini-label">Artifact Status</div><div class="mini-value">{html.escape(str(latest_artifact.get('status') or '-'))}</div></div>
            <div class="mini-stat"><div class="mini-label">Run ID</div><div class="mini-value">{html.escape(str(latest_artifact.get('run_id') or '-'))}</div></div>
            <div class="mini-stat"><div class="mini-label">Summary</div><div class="mini-value">{html.escape(str(latest_artifact.get('summary_line') or '-'))}</div></div>
          </div>
        </div>
        '''
    else:
        artifact_html = '<div class="empty">No artifact summary was available in the latest snapshot.</div>'

    subscription_error_html = ''.join(f'<li><code>{html.escape(str(item))}</code></li>' for item in subscription_errors)
    if not subscription_error_html:
        subscription_error_html = '<li>No subscription errors were recorded.</li>'

    note_html = '''
    <div class="note-grid">
      <div class="note-card">
        <h3>이 보고서가 저장하는 것</h3>
        <p>이건 브라우저 화면의 픽셀 스크린샷이 아니라, <strong>ros_states가 브라우저에 보여주던 백엔드 상태</strong>를 구조화해서 저장한 것이다.</p>
        <ul>
          <li>Mission Health 카드 상태</li>
          <li>현재 핵심 토픽/노드/서비스 그래프 정보</li>
          <li>artifact 요약, 힌트, subscription error</li>
          <li>시간에 따라 누적된 timeline.jsonl 기록</li>
        </ul>
      </div>
      <div class="note-card">
        <h3>사용자가 직접 해야 하는 것</h3>
        <p><strong>항상 자동 저장되는 구조는 아니다.</strong> 아래 동작 중 하나를 눌러야 기록이 남는다.</p>
        <ul>
          <li><code>Save Snapshot</code> : 지금 상태를 한 번 저장</li>
          <li><code>Start Recording</code> : 일정 간격으로 timeline 기록 시작</li>
          <li><code>Stop Recording</code> : recording 종료 + 마지막 snapshot 저장</li>
          <li><code>Generate Report</code> : 저장된 JSON을 이 HTML 보고서와 그래프로 다시 정리</li>
        </ul>
      </div>
    </div>
    '''

    cards_html = ''.join(
        f'<div class="metric-card"><div class="metric-label">{label}</div><div class="metric-value">{value}</div></div>'
        for label, value in overview_cards
    )
    inventory_html = ''.join(
        f'<div class="metric-card compact"><div class="metric-label">{label}</div><div class="metric-value">{value}</div></div>'
        for label, value in inventory_cards
    )

    html_doc = f'''<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ros_states Debug Report</title>
  <style>
    :root {{
      --bg: #eef3fb;
      --panel: #ffffff;
      --line: #dbe5f0;
      --text: #17324d;
      --muted: #5b738d;
      --shadow: 0 20px 40px rgba(17, 45, 78, 0.08);
    }}
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; font-family: "Noto Sans KR", "Segoe UI", sans-serif; background: linear-gradient(180deg, #f4f8fd 0%, var(--bg) 100%); color: var(--text); }}
    .page {{ max-width: 1320px; margin: 0 auto; padding: 28px 22px 46px; }}
    .hero {{ background: linear-gradient(135deg, #113f6d 0%, #1d568f 100%); color: #fff; border-radius: 24px; padding: 28px; box-shadow: 0 20px 40px rgba(17, 45, 78, 0.08); }}
    .hero h1 {{ margin: 0 0 10px; font-size: 30px; }}
    .hero p {{ margin: 0; color: rgba(255,255,255,0.88); line-height: 1.6; }}
    .hero-meta {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; margin-top: 22px; }}
    .hero-chip {{ background: rgba(255,255,255,0.12); border: 1px solid rgba(255,255,255,0.18); border-radius: 16px; padding: 12px 14px; }}
    .hero-chip .label {{ display: block; font-size: 12px; color: rgba(255,255,255,0.68); text-transform: uppercase; letter-spacing: 0.08em; margin-bottom: 6px; }}
    .hero-chip .value {{ font-size: 15px; font-weight: 700; }}
    .section {{ margin-top: 22px; background: var(--panel); border: 1px solid var(--line); border-radius: 22px; padding: 22px; box-shadow: 0 20px 40px rgba(17, 45, 78, 0.08); }}
    .section h2 {{ margin: 0 0 12px; font-size: 22px; }}
    .section p {{ color: var(--muted); line-height: 1.7; }}
    .metric-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; }}
    .metric-card {{ border: 1px solid var(--line); border-radius: 18px; padding: 16px; background: #fbfdff; min-height: 96px; }}
    .metric-card.compact {{ min-height: 84px; }}
    .metric-label {{ font-size: 12px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; margin-bottom: 10px; }}
    .metric-value {{ font-size: 20px; font-weight: 800; line-height: 1.4; word-break: break-word; }}
    .badge {{ display: inline-flex; align-items: center; padding: 6px 10px; border: 1px solid transparent; border-radius: 999px; font-size: 12px; font-weight: 800; letter-spacing: 0.04em; }}
    .note-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 14px; }}
    .note-card {{ background: #f8fbff; border: 1px solid var(--line); border-radius: 18px; padding: 18px; }}
    .note-card h3 {{ margin-top: 0; margin-bottom: 8px; }}
    .note-card ul {{ margin: 12px 0 0 18px; color: var(--muted); line-height: 1.7; }}
    .charts {{ display: grid; grid-template-columns: 1fr; gap: 14px; }}
    .chart-card {{ border: 1px solid var(--line); border-radius: 18px; padding: 16px; background: #fff; }}
    .chart-head {{ display: flex; justify-content: space-between; align-items: center; gap: 12px; margin-bottom: 10px; flex-wrap: wrap; }}
    .chart-head h4 {{ margin: 0; font-size: 18px; }}
    .chart-svg {{ width: 100%; height: auto; display: block; }}
    .axis-label {{ font-size: 11px; fill: #657d97; font-family: "Noto Sans KR", "Segoe UI", sans-serif; }}
    .band-label {{ font-size: 11px; fill: #17324d; font-family: "Noto Sans KR", "Segoe UI", sans-serif; font-weight: 700; }}
    .legend {{ display: flex; flex-wrap: wrap; gap: 10px; }}
    .legend-item {{ display: inline-flex; align-items: center; gap: 6px; color: var(--muted); font-size: 13px; }}
    .legend-swatch {{ width: 12px; height: 12px; border-radius: 999px; display: inline-block; }}
    table {{ width: 100%; border-collapse: collapse; margin-top: 10px; }}
    th, td {{ padding: 12px 10px; border-bottom: 1px solid var(--line); text-align: left; vertical-align: top; }}
    th {{ font-size: 12px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; }}
    td {{ font-size: 14px; line-height: 1.5; }}
    .empty-cell, .empty {{ color: var(--muted); }}
    .two-col {{ display: grid; grid-template-columns: 1.1fr 0.9fr; gap: 18px; }}
    .mini-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 10px; }}
    .mini-stat {{ background: #f8fbff; border: 1px solid var(--line); border-radius: 16px; padding: 14px; }}
    .mini-label {{ font-size: 12px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; margin-bottom: 8px; }}
    .mini-value {{ font-size: 18px; font-weight: 800; }}
    .hint-stack {{ display: grid; gap: 12px; }}
    .hint-card {{ border: 1px solid var(--line); background: #fbfdff; border-radius: 18px; padding: 16px; }}
    .hint-card p {{ margin: 10px 0; }}
    .hint-card ul {{ margin: 0 0 0 18px; color: var(--muted); }}
    .hint-head {{ display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }}
    .artifact-box {{ border: 1px solid var(--line); border-radius: 18px; padding: 14px; background: #fbfdff; }}
    .artifact-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 10px; }}
    code {{ background: #eef4fb; color: #21476c; padding: 2px 6px; border-radius: 8px; font-size: 0.94em; }}
    .footer-note {{ color: var(--muted); font-size: 13px; margin-top: 10px; }}
    @media (max-width: 980px) {{
      .two-col {{ grid-template-columns: 1fr; }}
      .page {{ padding: 18px 14px 32px; }}
      .hero {{ padding: 22px; }}
    }}
  </style>
</head>
<body>
  <div class="page">
    <section class="hero">
      <h1>ros_states Debug Report</h1>
      <p>저장된 ros_states 세션을 사람이 읽기 쉬운 요약, 상태 해석, 타임라인 그래프 형태로 다시 정리한 보고서다. 즉, “브라우저에서 무엇을 보고 있었는지”를 픽셀 스크린샷이 아니라 <strong>구조화된 상태 기록</strong>으로 복원한 결과다.</p>
      <div class="hero-meta">
        <div class="hero-chip"><span class="label">Generated At</span><span class="value">{html.escape(generated_at)}</span></div>
        <div class="hero-chip"><span class="label">Session</span><span class="value">{html.escape(session_path.name)}</span></div>
        <div class="hero-chip"><span class="label">Latest Summary</span><span class="value">{html.escape(latest_summary)}</span></div>
        <div class="hero-chip"><span class="label">Recording Active</span><span class="value">{html.escape('Yes' if manifest.get('recording_active') else 'No')}</span></div>
      </div>
    </section>

    <section class="section">
      <h2>1. 이 기록이 의미하는 것</h2>
      {note_html}
    </section>

    <section class="section">
      <h2>2. 세션 한눈에 보기</h2>
      <div class="metric-grid">{cards_html}</div>
    </section>

    <section class="section">
      <h2>3. 저장 파일과 환경 정보</h2>
      <div class="metric-grid">{inventory_html}</div>
      <p class="footer-note">원본 파일은 <code>session_manifest.json</code>, <code>timeline.jsonl</code>, <code>snapshots/</code>에 그대로 남아 있고, 이 보고서는 그 위에 설명과 그래프를 추가한 2차 가공 결과다.</p>
    </section>

    <section class="section">
      <h2>4. 그래프로 보는 디버깅 흐름</h2>
      <div class="charts">{''.join(charts)}</div>
    </section>

    <section class="section two-col">
      <div>
        <h2>5. Latest Mission Health</h2>
        <table>
          <thead><tr><th>Check</th><th>Status</th><th>Headline</th><th>Detail</th></tr></thead>
          <tbody>{check_rows}</tbody>
        </table>
      </div>
      <div>
        <h2>6. Latest Watch Topics</h2>
        <table>
          <thead><tr><th>Signal</th><th>Status</th><th>Topic</th><th>Headline</th><th>Detail</th></tr></thead>
          <tbody>{watch_rows}</tbody>
        </table>
      </div>
    </section>

    <section class="section two-col">
      <div>
        <h2>7. Artifact / Status Summary</h2>
        <div class="mini-grid">{status_counter_html}</div>
        <div style="margin-top:14px;">{artifact_html}</div>
      </div>
      <div>
        <h2>8. Troubleshooting Hints</h2>
        <div class="hint-stack">{hint_rows}</div>
      </div>
    </section>

    <section class="section">
      <h2>9. Subscription Errors</h2>
      <ul>{subscription_error_html}</ul>
    </section>
  </div>
</body>
</html>
'''

    report_summary = {
        'generated_at': generated_at,
        'session_dir': str(session_path),
        'recording_active': bool(manifest.get('recording_active')),
        'snapshot_count': len(snapshots),
        'timeline_count': len(timeline),
        'latest_overall_status': latest_status,
        'latest_phase': latest_phase,
        'latest_summary': latest_summary,
        'goal_reached': bool(latest_sample.get('goal_reached')),
        'latest_nearest_obstacle_m': latest_obstacle,
        'latest_planner_speed_mps': latest_planner_speed,
        'latest_safe_speed_mps': latest_safe_speed,
        'latest_topic_count': len((latest_payload or {}).get('topics', [])),
        'latest_node_count': len((latest_payload or {}).get('nodes', [])),
        'latest_snapshot_path': (latest_payload or {}).get('_snapshot_path'),
        'profile': profile,
        'status_counts': dict(status_counts),
    }

    report_path.write_text(html_doc, encoding='utf-8')
    summary_path.write_text(json.dumps(report_summary, indent=2, ensure_ascii=False), encoding='utf-8')

    return {
        'report_path': str(report_path),
        'report_summary_path': str(summary_path),
        'report_generated_at': generated_at,
    }
