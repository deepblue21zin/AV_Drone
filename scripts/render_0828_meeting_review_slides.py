#!/usr/bin/env python3
"""Render revised and added image slides for the 2026-08-28 meeting deck."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw

from render_oracle_lidar_70m_slides import (
    BLUE,
    GREEN,
    HEIGHT,
    INK,
    LINE,
    MUTED,
    NAVY,
    OFF_WHITE,
    ORANGE,
    RED,
    RESAMPLE,
    TEAL,
    WIDTH,
    add_card,
    draw_bullets,
    fit_background,
    font,
    header,
    metric_card,
    paragraph,
    paste_contain,
    rounded_rect,
)


def arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    color: str = TEAL,
    width: int = 7,
) -> None:
    draw.line((start[0], start[1], end[0], end[1]), fill=color, width=width)
    ex, ey = end
    draw.polygon(
        [(ex, ey), (ex - 18, ey - 12), (ex - 18, ey + 12)],
        fill=color,
    )


def slide_cover(repo: Path, output: Path) -> None:
    root = repo / "docs/presentation/0828_meeting_review/revised"
    canvas = fit_background(root / "backgrounds/01_cover.png")
    overlay = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    overlay_draw = ImageDraw.Draw(overlay)
    overlay_draw.rectangle((0, 0, 1060, HEIGHT), fill=(250, 249, 245, 220))
    canvas.alpha_composite(overlay)
    draw = ImageDraw.Draw(canvas)

    rounded_rect(draw, (82, 78, 376, 125), radius=15, fill=(223, 243, 240, 245))
    draw.text((108, 86), "2026.08.28 · 1차 미팅", font=font(21, "bold"), fill=TEAL)
    draw.text((82, 205), "ONE-WAY MULTI-UAV", font=font(24, "bold"), fill=TEAL)
    draw.text((82, 258), "LiDAR SLAM", font=font(66, "bold"), fill=NAVY)
    draw.text((82, 342), "Drift 보정 연구", font=font(66, "bold"), fill=NAVY)
    draw.line((82, 455, 818, 455), fill=TEAL, width=5)
    paragraph(
        draw,
        (82, 494),
        "Oracle feasibility에서 LiDAR 상대 pose 추정,\n그리고 GT-free association으로",
        font(27, "medium"),
        INK,
        760,
        line_gap=13,
    )

    stages = [
        ("01", "Relative drift 검증", RED),
        ("02", "LiDAR measurement", TEAL),
        ("03", "Adaptive scheduling", GREEN),
    ]
    for index, (number, label, color) in enumerate(stages):
        x0 = 82 + index * 260
        rounded_rect(draw, (x0, 660, x0 + 238, 742), radius=18, fill=(255, 255, 255, 236), outline=LINE, width=2)
        draw.text((x0 + 18, 674), number, font=font(18, "bold"), fill=color)
        draw.text((x0 + 18, 706), label, font=font(17, "medium"), fill=INK)

    draw.text((82, 910), "유병욱 · 정진수 · 조성준", font=font(25, "bold"), fill=NAVY)
    draw.text((82, 954), "자율주행 군집 드론 연구", font=font(20), fill=MUTED)
    canvas.convert("RGB").save(output / "01_cover_revised.png", quality=95)


def slide_problem(repo: Path, output: Path) -> None:
    root = repo / "docs/presentation/0828_meeting_review/revised"
    artifact = repo / "artifacts/2026-08-27_lidar_phase2_70m_codex_v1"
    canvas = fit_background(root / "backgrounds/02_drift.png")
    header(
        canvas,
        "RESULT 01 · PROBLEM DEFINITION",
        "70 m One-Way 비행에서 Raw SLAM Drift가 누적됨",
        "동일 rosbag · 2 UAV · loop closure OFF · 공통 진행 거리 74.7 m",
        "02",
    )
    chart_box = (62, 205, 1402, 963)
    add_card(canvas, chart_box)
    paste_contain(
        canvas,
        artifact / "oracle_correction/presentation/02_trajectories.png",
        chart_box,
        crop=(35, 105, 2645, 1150),
        padding=22,
    )
    metric_card(canvas, (1432, 220, 1848, 364), "B0 endpoint relative drift", "1.067 m", "두 UAV 사이 translation", RED)
    metric_card(canvas, (1432, 384, 1848, 528), "B0 map Chamfer", "1.318 m", "GT 장애물과 평균 거리", RED)
    metric_card(canvas, (1432, 548, 1848, 692), "B0 occupied F1", "0.180", "장애물 셀 일치도", RED)
    add_card(canvas, (1432, 716, 1848, 963), radius=22, fill=(255, 255, 255, 247))
    draw = ImageDraw.Draw(canvas)
    draw.text((1462, 742), "수정된 발표 메시지", font=font(21, "bold"), fill=RED)
    draw_bullets(
        draw,
        [
            "UAV2의 B0 궤적이 x≈25 m 이후 GT에서 크게 이탈",
            "self loop closure가 없는 one-way에서는 drift를 되돌릴 기회가 부족",
        ],
        1462,
        784,
        350,
        bullet_color=RED,
        face=font(18),
        gap=10,
    )
    draw.text((72, 1015), "B0 = raw SLAM · B1 = known spawn + PX4/MAVROS odometry (Gazebo GT 아님)", font=font(18), fill=MUTED)
    canvas.convert("RGB").save(output / "02_problem_revised.png", quality=95)


def slide_oracle(repo: Path, output: Path) -> None:
    root = repo / "docs/presentation/0828_meeting_review/revised"
    artifact = repo / "artifacts/2026-08-27_lidar_phase2_70m_codex_v1"
    canvas = fit_background(root / "backgrounds/03_oracle.png")
    header(
        canvas,
        "RESULT 02 · ORACLE FEASIBILITY",
        "주기적 Inter-UAV Constraint는 상대 Drift를 91.7% 감소",
        "정확한 상대 pose가 주어진다는 상한선 실험 · O-single 1개 vs O-periodic 7개",
        "03",
    )
    chart_box = (62, 205, 1410, 955)
    add_card(canvas, chart_box)
    paste_contain(
        canvas,
        artifact / "oracle_correction/presentation/03_differential_error.png",
        chart_box,
        crop=(35, 95, 2645, 1165),
        padding=22,
    )
    add_card(canvas, (1440, 220, 1850, 442), radius=22, fill=(255, 255, 255, 248))
    draw = ImageDraw.Draw(canvas)
    draw.text((1470, 246), "Endpoint translation", font=font(20, "medium"), fill=MUTED)
    draw.text((1470, 286), "1.067 → 0.089 m", font=font(36, "bold"), fill=GREEN)
    rounded_rect(draw, (1470, 350, 1665, 403), radius=16, fill=(224, 247, 232, 255))
    draw.text((1494, 358), "−91.7%", font=font(25, "bold"), fill=GREEN)
    draw.text((1684, 365), "O-periodic", font=font(18, "medium"), fill=MUTED)
    metric_card(canvas, (1440, 468, 1850, 612), "O-single endpoint", "0.610 m", "중간 연결 후 후반 drift 재성장", ORANGE)
    metric_card(canvas, (1440, 632, 1850, 776), "O-periodic factors", "7 개", "약 10 m 간격", GREEN)
    add_card(canvas, (1440, 802, 1850, 955), radius=22, fill=(255, 248, 235, 250), outline="#f3c56c")
    draw.text((1470, 823), "필수 한계 문구", font=font(20, "bold"), fill=ORANGE)
    paragraph(draw, (1470, 858), "GT 상대 pose(+noise)를 factor로 넣은 상한선이다. 자동 장소 인식 결과가 아니다.", font(18), INK, 350, line_gap=7)
    draw.text((72, 1015), "한 번의 연결보다 반복되는 cross-UAV 연결이 one-way relative drift 억제에 효과적", font=font(18), fill=MUTED)
    canvas.convert("RGB").save(output / "03_oracle_revised.png", quality=95)


def slide_map(repo: Path, output: Path) -> None:
    root = repo / "docs/presentation/0828_meeting_review/revised"
    artifact = repo / "artifacts/2026-08-27_lidar_phase2_70m_codex_v1"
    canvas = fit_background(root / "backgrounds/04_map.png")
    header(
        canvas,
        "RESULT 03 · MAP QUALITY",
        "상대 일관성 개선이 곧 절대 지도 최적화를 의미하지는 않음",
        "모든 조건에서 동일 LiDAR scan을 pose만 바꾸어 재투영 · 청록 outline = GT occupied",
        "04",
    )
    map_box = (62, 205, 1420, 960)
    add_card(canvas, map_box)
    paste_contain(
        canvas,
        artifact / "oracle_correction/presentation/05_map_comparison.png",
        map_box,
        crop=(40, 95, 2640, 1530),
        padding=20,
    )
    add_card(canvas, (1450, 220, 1855, 600), radius=22, fill=(255, 255, 255, 248))
    draw = ImageDraw.Draw(canvas)
    draw.text((1480, 246), "정량 비교", font=font(23, "bold"), fill=NAVY)
    draw.text((1480, 294), "Condition", font=font(17, "medium"), fill=MUTED)
    draw.text((1663, 294), "Chamfer↓", font=font(17, "medium"), fill=MUTED)
    draw.text((1782, 294), "F1↑", font=font(17, "medium"), fill=MUTED)
    rows = [
        ("B0", "1.318", "0.180", RED),
        ("O-periodic", "0.935", "0.398", GREEN),
        ("B1 odometry", "0.441", "0.511", BLUE),
    ]
    for index, (name, chamfer, f1, color) in enumerate(rows):
        y = 340 + index * 77
        draw.line((1480, y - 12, 1825, y - 12), fill=LINE, width=2)
        draw.ellipse((1480, y + 8, 1492, y + 20), fill=color)
        draw.text((1508, y), name, font=font(18, "medium"), fill=INK)
        draw.text((1670, y), chamfer, font=font(18, "bold"), fill=color)
        draw.text((1788, y), f1, font=font(18, "bold"), fill=color)
    metric_card(canvas, (1450, 626, 1855, 770), "O-periodic vs B0", "Chamfer −29.1%", "F1 +0.218", GREEN)
    add_card(canvas, (1450, 796, 1855, 960), radius=22, fill=(244, 248, 252, 250))
    draw.text((1480, 818), "결론", font=font(21, "bold"), fill=NAVY)
    paragraph(draw, (1480, 855), "Oracle은 B0보다 개선됐지만, 강한 odometry baseline인 B1을 아직 넘지 못했다.", font(18), INK, 345, line_gap=7)
    draw.text((72, 1015), "relative drift · yaw · trajectory · Chamfer · F1을 함께 보고해야 함", font=font(18), fill=MUTED)
    canvas.convert("RGB").save(output / "04_map_revised.png", quality=95)


def flow_box(
    canvas: Image.Image,
    box: tuple[int, int, int, int],
    step: str,
    title: str,
    detail: str,
    color: str,
) -> None:
    add_card(canvas, box, radius=22, fill=(255, 255, 255, 248))
    draw = ImageDraw.Draw(canvas)
    x0, y0, _, _ = box
    rounded_rect(draw, (x0 + 18, y0 + 18, x0 + 73, y0 + 69), radius=15, fill=color)
    draw.text((x0 + 34, y0 + 27), step, font=font(20, "bold"), fill="#ffffff")
    draw.text((x0 + 91, y0 + 22), title, font=font(21, "bold"), fill=NAVY)
    paragraph(draw, (x0 + 26, y0 + 84), detail, font(18), MUTED, box[2] - x0 - 52, line_gap=8)


def slide_gt_usage(repo: Path, output: Path) -> None:
    root = repo / "docs/presentation/0828_meeting_review/revised"
    canvas = fit_background(root / "backgrounds/05_gt_usage.png")
    header(
        canvas,
        "METHOD · ORACLE DEFINITION",
        "Oracle에서 GT는 어디에 사용되었나?",
        "전체 trajectory를 GT로 덮어쓴 것이 아니라, 선택된 keyframe 사이 상대 pose factor를 생성",
        "05",
    )
    draw = ImageDraw.Draw(canvas)
    boxes = [
        ((62, 235, 330, 440), "1", "GT pose 기록", "Gazebo에서\nT1_GT(t), T2_GT(t)", NAVY),
        ((372, 235, 640, 440), "2", "Pair 선택", "같은 GT 진행거리의\ntwo-UAV keyframe", TEAL),
        ((682, 235, 1000, 440), "3", "상대 pose 생성", "z12 = inv(T1_GT) T2_GT\n+ translation/yaw noise", ORANGE),
        ((1042, 235, 1308, 440), "4", "PGO factor", "두 raw SLAM graph를\nrelative constraint로 연결", GREEN),
    ]
    for box, step, title, detail, color in boxes:
        flow_box(canvas, box, step, title, detail, color)
    for first, second in zip(boxes, boxes[1:]):
        arrow(draw, (first[0][2] + 8, 338), (second[0][0] - 12, 338), TEAL, 6)

    add_card(canvas, (62, 485, 650, 660), radius=22, fill=(255, 255, 255, 248))
    draw.text((92, 510), "O-single", font=font(22, "bold"), fill=ORANGE)
    draw.text((92, 553), "공통 경로 중앙 ≈ 37.3 m에서 1 factor", font=font(20), fill=INK)
    draw.text((92, 596), "→ 이후 구간에서 drift 재성장", font=font(18), fill=MUTED)
    add_card(canvas, (680, 485, 1308, 660), radius=22, fill=(255, 255, 255, 248))
    draw.text((710, 510), "O-periodic", font=font(22, "bold"), fill=GREEN)
    draw.text((710, 553), "10–70 m, 약 10 m 간격으로 7 factors", font=font(20), fill=INK)
    draw.text((710, 596), "→ 상대 위치를 반복적으로 다시 연결", font=font(18), fill=MUTED)

    add_card(canvas, (62, 700, 1308, 950), radius=24, fill=(246, 250, 252, 249))
    draw.text((92, 727), "핵심 구분", font=font(23, "bold"), fill=NAVY)
    draw_bullets(
        draw,
        [
            "GT는 absolute pose 정답을 매 시점 optimizer에 고정하는 데 사용하지 않음",
            "GT는 pair association과 relative measurement 생성, 그리고 결과 평가에 사용",
            "따라서 O 조건은 최종 방법이 아니라 ‘상대 pose가 있다면 보정 가능한가?’를 묻는 상한선",
        ],
        96,
        775,
        1160,
        bullet_color=TEAL,
        face=font(19),
        gap=10,
    )

    add_card(canvas, (1342, 235, 1857, 950), radius=26, fill=(255, 255, 255, 247))
    draw.text((1375, 265), "단계별 GT 의존성", font=font(25, "bold"), fill=NAVY)
    conditions = [
        ("Oracle O", "Pair: GT\nMeasurement: GT + noise\nEvaluation: GT", ORANGE),
        ("LiDAR R", "Pair: GT\nMeasurement: LiDAR\nEvaluation: GT", TEAL),
        ("Final P", "Pair: descriptor\nMeasurement: LiDAR\nEvaluation만 GT", GREEN),
    ]
    for index, (name, detail, color) in enumerate(conditions):
        y = 325 + index * 190
        rounded_rect(draw, (1375, y, 1825, y + 160), radius=20, fill=(248, 250, 252, 250), outline=LINE, width=2)
        draw.rectangle((1375, y, 1385, y + 160), fill=color)
        draw.text((1410, y + 18), name, font=font(22, "bold"), fill=color)
        paragraph(draw, (1410, y + 58), detail, font(18), INK, 370, line_gap=6)
    draw.text((72, 1015), "발표에서는 O와 R을 명확히 분리하고, GT-free 완성으로 과장하지 않기", font=font(18), fill=MUTED)
    canvas.convert("RGB").save(output / "05_gt_usage_added.png", quality=95)


def slide_lidar(repo: Path, output: Path) -> None:
    root = repo / "docs/presentation/0828_meeting_review/revised"
    artifact = repo / "artifacts/2026-08-27_lidar_phase2_70m_codex_v1"
    canvas = fit_background(root / "backgrounds/06_lidar.png")
    header(
        canvas,
        "RESULT 04 · CONTROLLED LIDAR REGISTRATION",
        "LiDAR 상대 Pose 추정은 효과 확인 — GT-free Association은 아직 미완성",
        "GT는 비교할 keyframe pair만 선택 · 실제 relative-pose measurement에는 GT transform 미사용",
        "06",
    )
    accepted_box = (58, 210, 500, 820)
    rejected_box = (520, 210, 962, 820)
    add_card(canvas, accepted_box)
    add_card(canvas, rejected_box)
    paste_contain(canvas, artifact / "lidar_registration/debug_plots/R-periodic_02_0030.0m.png", accepted_box, crop=(55, 0, 1165, 1175), padding=14)
    paste_contain(canvas, artifact / "lidar_registration/debug_plots/R-periodic_05_0060.0m.png", rejected_box, crop=(55, 0, 1165, 1175), padding=14)
    draw = ImageDraw.Draw(canvas)
    rounded_rect(draw, (76, 232, 237, 276), radius=14, fill=(224, 247, 232, 245))
    draw.text((98, 238), "30 m · ACCEPT", font=font(18, "bold"), fill=GREEN)
    rounded_rect(draw, (538, 232, 700, 276), radius=14, fill=(254, 232, 232, 245))
    draw.text((558, 238), "60 m · REJECT", font=font(18, "bold"), fill=RED)
    add_card(canvas, (58, 842, 500, 970), radius=20, fill=(255, 255, 255, 248))
    draw.text((82, 862), "overlap 0.371", font=font(21, "bold"), fill=GREEN)
    draw.text((82, 903), "B0 0.779 → LiDAR 0.440 m", font=font(20, "medium"), fill=INK)
    add_card(canvas, (520, 842, 962, 970), radius=20, fill=(255, 255, 255, 248))
    draw.text((544, 862), "overlap 0.000", font=font(21, "bold"), fill=RED)
    draw.text((544, 903), "저중첩·고잔차·모호성 gate로 거절", font=font(19, "medium"), fill=INK)
    add_card(canvas, (1000, 210, 1857, 970), radius=28, fill=(255, 255, 255, 246))
    draw.text((1040, 244), "Controlled Phase-2 결과", font=font(27, "bold"), fill=NAVY)
    metric_card(canvas, (1040, 300, 1415, 448), "R-periodic accepted", "2 / 7", "잘못된 factor보다 거절 우선", GREEN)
    metric_card(canvas, (1440, 300, 1815, 448), "Endpoint relative drift", "0.635 m", "B0 대비 −40.5%", TEAL)
    metric_card(canvas, (1040, 472, 1415, 620), "Map Chamfer", "0.774 m", "B0 1.318 m에서 개선", TEAL)
    metric_card(canvas, (1440, 472, 1815, 620), "Occupied F1", "0.421", "B0 0.180에서 개선", TEAL)
    draw.text((1040, 668), "개발 상태", font=font(23, "bold"), fill=NAVY)
    draw_bullets(
        draw,
        [
            "완료: LiDAR submap 정합 · confidence gate · pose graph 보정",
            "남음: GT 없이 동일 장소 후보를 찾는 association front-end",
            "다음: cylinder/scan descriptor → top-K 후보 → 정합 → 시간 일관성 검증",
        ],
        1042,
        715,
        750,
        bullet_color=TEAL,
        face=font(20),
        gap=10,
    )
    draw.text((72, 1015), "measurement backend의 가능성은 확인 · 완전 GT-free 제안은 association 구현 후 평가", font=font(18), fill=MUTED)
    canvas.convert("RGB").save(output / "06_lidar_added.png", quality=95)


def strategy_card(
    canvas: Image.Image,
    box: tuple[int, int, int, int],
    label: str,
    title: str,
    detail: str,
    color: str,
) -> None:
    add_card(canvas, box, radius=22, fill=(255, 255, 255, 248))
    draw = ImageDraw.Draw(canvas)
    x0, y0, _, _ = box
    rounded_rect(draw, (x0 + 22, y0 + 20, x0 + 136, y0 + 57), radius=12, fill=color)
    draw.text((x0 + 35, y0 + 25), label, font=font(15, "bold"), fill="#ffffff")
    draw.text((x0 + 22, y0 + 76), title, font=font(21, "bold"), fill=NAVY)
    paragraph(draw, (x0 + 22, y0 + 116), detail, font(17), MUTED, box[2] - x0 - 44, line_gap=7)


def slide_adaptive(repo: Path, output: Path) -> None:
    root = repo / "docs/presentation/0828_meeting_review/revised"
    canvas = fit_background(root / "backgrounds/07_adaptive.png")
    footer_overlay = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    ImageDraw.Draw(footer_overlay).rectangle((0, 990, WIDTH, HEIGHT), fill=(250, 249, 245, 225))
    canvas.alpha_composite(footer_overlay)
    header(
        canvas,
        "PROPOSED EXTENSION · EFFICIENCY",
        "얼마나 자주 연결해야 가장 효과적인가?",
        "고정 간격이 아니라 drift 위험 · 예상 overlap · 정보 이득을 기준으로 constraint 생성",
        "07",
    )
    draw = ImageDraw.Draw(canvas)
    add_card(canvas, (62, 220, 875, 640), radius=26, fill=(255, 255, 255, 248))
    draw.text((92, 248), "현재 70 m run의 구간별 실제 overlap", font=font(24, "bold"), fill=NAVY)
    labels = ["0–20", "20–40", "40–60", "60–78.9"]
    values = [0.027, 0.250, 0.103, 0.000]
    colors = [ORANGE, GREEN, TEAL, RED]
    chart_left, chart_top, chart_bottom = 130, 330, 560
    draw.line((chart_left, chart_top, chart_left, chart_bottom), fill=MUTED, width=3)
    draw.line((chart_left, chart_bottom, 820, chart_bottom), fill=MUTED, width=3)
    max_value = 0.30
    for index, (label, value, color) in enumerate(zip(labels, values, colors)):
        x0 = 190 + index * 155
        height = int((value / max_value) * 180)
        rounded_rect(draw, (x0, chart_bottom - height, x0 + 90, chart_bottom), radius=12, fill=color)
        draw.text((x0 + 10, chart_bottom - height - 37), f"{value:.3f}", font=font(18, "bold"), fill=color)
        draw.text((x0 + 7, chart_bottom + 16), label, font=font(16, "medium"), fill=INK)
    draw.text((116, 594), "LiDAR factor ACCEPT: 30 m, 40 m · 후반 구간은 overlap 부족으로 안전하게 거절", font=font(17), fill=MUTED)

    strategy_card(canvas, (915, 220, 1215, 530), "BASELINE", "Fixed 10 m", "간단하지만 overlap이 없는 10·60·70 m에서도 불필요한 후보 계산", ORANGE)
    strategy_card(canvas, (1240, 220, 1538, 530), "PROPOSED", "Adaptive passive", "불확실성이 크고 descriptor overlap이 충분할 때만 submap 교환·정합", GREEN)
    strategy_card(canvas, (1563, 220, 1860, 530), "FUTURE", "Active rendezvous", "drift가 위험하지만 overlap이 없을 때만 최소 경로 수정으로 관측 중첩 생성", TEAL)

    add_card(canvas, (915, 560, 1860, 775), radius=24, fill=(255, 255, 255, 248))
    draw.text((945, 586), "Runtime trigger", font=font(23, "bold"), fill=NAVY)
    rounded_rect(draw, (945, 635, 1818, 706), radius=18, fill=(237, 247, 247, 250), outline="#b9d8d8", width=2)
    draw.text((976, 650), "uncertainty ↑  ∧  predicted overlap ↑  ∧  unique geometry", font=font(23, "bold"), fill=TEAL)
    draw.text((1230, 724), "→ relative-pose constraint 생성", font=font(20, "medium"), fill=GREEN)

    add_card(canvas, (62, 680, 875, 950), radius=25, fill=(255, 255, 255, 248))
    draw.text((92, 708), "효율 목적함수", font=font(23, "bold"), fill=NAVY)
    rounded_rect(draw, (92, 758, 840, 827), radius=18, fill=(245, 247, 250, 255), outline=LINE, width=2)
    draw.text((116, 775), "J = SLAM error + λd·detour + λc·bytes + λt·delay", font=font(22, "bold"), fill=INK)
    draw.text((92, 854), "비교: no constraint / fixed 5·10·20·30 m / adaptive passive / adaptive active", font=font(18), fill=MUTED)
    draw.text((92, 894), "지표: relative drift · Chamfer/F1 · factor 수 · 통신량 · 추가 거리 · 임무 시간", font=font(18), fill=MUTED)

    add_card(canvas, (915, 805, 1860, 950), radius=24, fill=(240, 250, 244, 250), outline="#b9dfc5")
    draw.text((945, 832), "논문 확장 메시지", font=font(22, "bold"), fill=GREEN)
    paragraph(draw, (945, 872), "최소한의 통신·비행 비용으로 최대 drift 감소를 얻는 uncertainty/overlap-aware scheduling", font(20, "medium"), INK, 840, line_gap=8)
    draw.text((72, 1015), "첫 구현은 offline fixed-spacing sweep → adaptive passive → 필요한 경우 active rendezvous 순서", font=font(18), fill=MUTED)
    canvas.convert("RGB").save(output / "07_adaptive_scheduling_added.png", quality=95)


def contact_sheet(slides: list[Path], output: Path) -> None:
    thumb_w, thumb_h = 960, 540
    rows = (len(slides) + 1) // 2
    sheet = Image.new("RGB", (thumb_w * 2, thumb_h * rows), OFF_WHITE)
    for index, path in enumerate(slides):
        image = Image.open(path).convert("RGB").resize((thumb_w, thumb_h), RESAMPLE)
        sheet.paste(image, ((index % 2) * thumb_w, (index // 2) * thumb_h))
    sheet.save(output / "00_contact_sheet.png", quality=95)


def slide_pdf(slides: list[Path], output: Path) -> None:
    pages = [Image.open(path).convert("RGB") for path in slides]
    pages[0].save(
        output / "0828_meeting_revised_and_added.pdf",
        "PDF",
        resolution=150.0,
        save_all=True,
        append_images=pages[1:],
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    repo = args.repo_root.resolve()
    output = args.output.resolve() if args.output else repo / "docs/presentation/0828_meeting_review/revised/slides"
    output.mkdir(parents=True, exist_ok=True)

    slide_cover(repo, output)
    slide_problem(repo, output)
    slide_oracle(repo, output)
    slide_map(repo, output)
    slide_gt_usage(repo, output)
    slide_lidar(repo, output)
    slide_adaptive(repo, output)
    slides = sorted(output.glob("0[1-7]_*.png"))
    contact_sheet(slides, output)
    slide_pdf(slides, output)
    print(f"Rendered {len(slides)} revised/added slides to {output}")


if __name__ == "__main__":
    main()
