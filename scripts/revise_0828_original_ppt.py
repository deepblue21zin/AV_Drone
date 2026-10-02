#!/usr/bin/env python3
"""Revise the user's 0828 PPTX in place stylistically, while preserving the original."""

from __future__ import annotations

import argparse
from io import BytesIO
from pathlib import Path
import re
import zipfile
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont


P = "http://schemas.openxmlformats.org/presentationml/2006/main"
A = "http://schemas.openxmlformats.org/drawingml/2006/main"
R = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
REL = "http://schemas.openxmlformats.org/package/2006/relationships"
CT = "http://schemas.openxmlformats.org/package/2006/content-types"
EP = "http://schemas.openxmlformats.org/officeDocument/2006/extended-properties"
VT = "http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes"
MC = "http://schemas.openxmlformats.org/markup-compatibility/2006"
P14 = "http://schemas.microsoft.com/office/powerpoint/2010/main"

for prefix, uri in (
    ("p", P),
    ("a", A),
    ("r", R),
    ("", REL),
    ("mc", MC),
    ("p14", P14),
):
    ET.register_namespace(prefix, uri)

NS = {"p": P, "a": A, "r": R}

WIDTH = 1920
HEIGHT = 1080
GREEN = "#50B26B"
ACCENT_GREEN = "#9BBB59"
LIGHT_GREEN = "#D7E4BD"
OLIVE = "#4E5E2D"
BLUE = "#4F81BD"
RED = "#C0504D"
ORANGE = "#F79646"
INK = "#171A18"
MUTED = "#59615D"
WHITE = "#FFFFFF"
BG = "#EEEEEE"
RESAMPLE = getattr(Image, "Resampling", Image).LANCZOS

FONT_REGULAR = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FONT_MEDIUM = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD if bold else (FONT_MEDIUM if FONT_MEDIUM.exists() else FONT_REGULAR)
    return ImageFont.truetype(str(path), size=size)


def text_width(draw: ImageDraw.ImageDraw, text: str, face: ImageFont.FreeTypeFont) -> int:
    if hasattr(draw, "textbbox"):
        box = draw.textbbox((0, 0), text, font=face)
        return box[2] - box[0]
    return draw.textsize(text, font=face)[0]


def draw_fit(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    max_width: int,
    start_size: int,
    fill: str,
    bold: bool = False,
    min_size: int = 22,
) -> None:
    size = start_size
    face = font(size, bold)
    while size > min_size and text_width(draw, text, face) > max_width:
        size -= 2
        face = font(size, bold)
    draw.text(xy, text, font=face, fill=fill)


def wrap_by_width(
    draw: ImageDraw.ImageDraw,
    text: str,
    face: ImageFont.FreeTypeFont,
    max_width: int,
) -> list[str]:
    lines: list[str] = []
    for explicit in text.split("\n"):
        words = explicit.split()
        if not words:
            lines.append("")
            continue
        current = words[0]
        for word in words[1:]:
            candidate = f"{current} {word}"
            if text_width(draw, candidate, face) <= max_width:
                current = candidate
            else:
                lines.append(current)
                current = word
        lines.append(current)
    return lines


def paragraph(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    face: ImageFont.FreeTypeFont,
    fill: str,
    max_width: int,
    line_gap: int = 8,
) -> int:
    x, y = xy
    line_height = face.size + line_gap
    for line in wrap_by_width(draw, text, face, max_width):
        draw.text((x, y), line, font=face, fill=fill)
        y += line_height
    return y


def rounded(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    fill: str,
    outline: str | None = None,
    width: int = 2,
    radius: int = 18,
) -> None:
    if hasattr(draw, "rounded_rectangle"):
        draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)
        return

    def fill_round(target: tuple[int, int, int, int], target_radius: int, color: str) -> None:
        x0, y0, x1, y1 = target
        target_radius = max(1, min(target_radius, (x1 - x0) // 2, (y1 - y0) // 2))
        draw.rectangle((x0 + target_radius, y0, x1 - target_radius, y1), fill=color)
        draw.rectangle((x0, y0 + target_radius, x1, y1 - target_radius), fill=color)
        draw.pieslice((x0, y0, x0 + target_radius * 2, y0 + target_radius * 2), 180, 270, fill=color)
        draw.pieslice((x1 - target_radius * 2, y0, x1, y0 + target_radius * 2), 270, 360, fill=color)
        draw.pieslice((x0, y1 - target_radius * 2, x0 + target_radius * 2, y1), 90, 180, fill=color)
        draw.pieslice((x1 - target_radius * 2, y1 - target_radius * 2, x1, y1), 0, 90, fill=color)

    if outline is not None and width > 0:
        fill_round(box, radius, outline)
        x0, y0, x1, y1 = box
        fill_round((x0 + width, y0 + width, x1 - width, y1 - width), radius - width, fill)
    else:
        fill_round(box, radius, fill)


def paste_contain(
    canvas: Image.Image,
    source: Path,
    box: tuple[int, int, int, int],
    crop: tuple[int, int, int, int] | None = None,
    padding: int = 10,
) -> None:
    image = Image.open(source).convert("RGBA")
    if crop is not None:
        image = image.crop(crop)
    x0, y0, x1, y1 = box
    max_width = x1 - x0 - padding * 2
    max_height = y1 - y0 - padding * 2
    scale = min(max_width / image.width, max_height / image.height)
    resized = image.resize((int(image.width * scale), int(image.height * scale)), RESAMPLE)
    x = x0 + (x1 - x0 - resized.width) // 2
    y = y0 + (y1 - y0 - resized.height) // 2
    canvas.alpha_composite(resized, (x, y))


def original_style_base(background: Image.Image, title: str, section: str, page: int) -> Image.Image:
    canvas = background.convert("RGBA").resize((WIDTH, HEIGHT), RESAMPLE)
    draw = ImageDraw.Draw(canvas)
    draw_fit(draw, (35, 42), title, 1810, 66, GREEN, bold=True, min_size=46)
    draw.rectangle((78, 161, 1845, 953), fill=WHITE, outline=ACCENT_GREEN, width=2)
    rounded(draw, (98, 178, 1826, 250), LIGHT_GREEN, OLIVE, width=2, radius=15)
    draw.text((122, 190), section, font=font(36, True), fill=INK)
    draw.text((1820, 1025), str(page), font=font(20), fill=GREEN)
    return canvas


def bottom_takeaway(canvas: Image.Image, text: str, color: str = INK) -> None:
    draw = ImageDraw.Draw(canvas)
    arrow_box = [(90, 995), (145, 995), (145, 980), (188, 1023), (145, 1066), (145, 1050), (90, 1050)]
    draw.polygon(arrow_box, fill=BLUE, outline="#345F8E")
    draw_fit(draw, (215, 997), text, 1580, 31, color, bold=True, min_size=22)


def flow_card(
    canvas: Image.Image,
    box: tuple[int, int, int, int],
    number: str,
    title: str,
    detail: str,
    color: str,
) -> None:
    draw = ImageDraw.Draw(canvas)
    rounded(draw, box, "#F8FAF8", ACCENT_GREEN, width=2, radius=17)
    x0, y0, _, _ = box
    rounded(draw, (x0 + 18, y0 + 18, x0 + 69, y0 + 69), color, radius=13)
    draw.text((x0 + 34, y0 + 24), number, font=font(20, True), fill=WHITE)
    draw.text((x0 + 87, y0 + 22), title, font=font(24, True), fill=INK)
    paragraph(draw, (x0 + 24, y0 + 88), detail, font(18), MUTED, box[2] - x0 - 48, 7)


def situation_step(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    number: str,
    title: str,
    detail: str,
    color: str,
) -> None:
    rounded(draw, box, "#F8FAF8", ACCENT_GREEN, width=2, radius=16)
    x0, y0, _, _ = box
    rounded(draw, (x0 + 20, y0 + 20, x0 + 75, y0 + 75), color, radius=14)
    draw.text((x0 + 37, y0 + 29), number, font=font(21, True), fill=WHITE)
    draw.text((x0 + 96, y0 + 20), title, font=font(23, True), fill=INK)
    paragraph(draw, (x0 + 96, y0 + 64), detail, font(18), MUTED, box[2] - x0 - 120, 7)


def build_intro_situation(
    background: Image.Image,
    context_asset: Path,
    output: Path,
    page_number: int = 2,
) -> None:
    canvas = original_style_base(
        background,
        "연구 상황: One-Way 비행에서는 Drift 보정 기회가 부족",
        "왜 이 실험이 필요한가?",
        page_number,
    )
    draw = ImageDraw.Draw(canvas)
    situation_step(
        draw,
        (112, 292, 690, 438),
        "1",
        "70 m 편도 비행",
        "두 UAV가 같은 방향으로 이동하고 출발 지점으로 돌아오지 않음",
        BLUE,
    )
    situation_step(
        draw,
        (112, 466, 690, 612),
        "2",
        "Self loop closure 부재",
        "각 UAV의 raw SLAM drift를 다시 관측해 스스로 줄일 기회가 부족",
        RED,
    )
    situation_step(
        draw,
        (112, 640, 690, 786),
        "3",
        "Inter-UAV 연결 가능성",
        "두 UAV가 같은 구조를 관측하면 상대 pose factor로 graph를 연결 가능",
        GREEN,
    )
    rounded(draw, (112, 820, 690, 910), "#EFF6E5", ACCENT_GREEN, width=2, radius=15)
    draw.text((137, 839), "핵심 문제", font=font(20, True), fill=OLIVE)
    draw.text((137, 874), "정확한 상대 pose를 어떻게 얻고, 언제 연결할 것인가?", font=font(19, True), fill=INK)

    visual_box = (735, 292, 1792, 784)
    rounded(draw, visual_box, WHITE, ACCENT_GREEN, width=2, radius=16)
    paste_contain(canvas, context_asset, visual_box, padding=14)
    rounded(draw, (735, 820, 1792, 910), "#F7F7F2", ACCENT_GREEN, width=2, radius=15)
    draw.text((760, 839), "이번 단계에서 확인할 질문", font=font(20, True), fill=OLIVE)
    draw_fit(
        draw,
        (760, 873),
        "정확한 상대 pose가 주어진다면, 1회와 주기적 연결 중 어느 쪽이 drift를 더 줄이는가?",
        990,
        20,
        INK,
        bold=True,
        min_size=16,
    )
    bottom_takeaway(canvas, "먼저 상대 pose constraint 자체의 보정 효과를 Oracle 실험으로 분리하여 검증")
    canvas.convert("RGB").save(output, quality=95)


def condition_card(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    name: str,
    headline: str,
    detail: str,
    color: str,
) -> None:
    rounded(draw, box, "#F8FAF8", ACCENT_GREEN, width=2, radius=15)
    x0, y0, _, _ = box
    draw.rectangle((x0, y0, x0 + 10, box[3]), fill=color)
    draw.text((x0 + 25, y0 + 18), name, font=font(25, True), fill=color)
    draw.text((x0 + 25, y0 + 61), headline, font=font(18, True), fill=INK)
    paragraph(draw, (x0 + 25, y0 + 97), detail, font(16), MUTED, box[2] - x0 - 48, 5)


def build_intro_setup(background: Image.Image, output: Path, page_number: int = 3) -> None:
    canvas = original_style_base(
        background,
        "실험 구성: 상대 Pose Constraint 효과를 분리하여 검증",
        "70 m Oracle 실험 설계",
        page_number,
    )
    draw = ImageDraw.Draw(canvas)
    chips = ["2 UAV", "공통 진행거리 74.7 m", "Loop closure OFF", "동일 rosbag · 동일 LiDAR scan"]
    chip_widths = [190, 340, 300, 560]
    chip_colors = [BLUE, GREEN, RED, OLIVE]
    x = 112
    for label, width, color in zip(chips, chip_widths, chip_colors):
        rounded(draw, (x, 282, x + width, 338), "#F7F7F2", color, width=2, radius=14)
        label_face = font(18, True)
        label_width = text_width(draw, label, label_face)
        draw.text((x + (width - label_width) // 2, 294), label, font=label_face, fill=color)
        x += width + 22

    card_width = 314
    gap = 17
    card_x = 112
    cards = [
        ("GT", "Gazebo 실제 위치", "결과 평가의 정답\n현실에서는 알 수 없음", "#111827"),
        ("B0", "Raw SLAM", "드론 간 보정 없음\ndrift 누적 기준", "#D62728"),
        ("B1", "Odometry baseline", "known spawn + odometry\nGazebo GT 아님", "#2567D8"),
        ("O-single", "Oracle 1회 연결", "GT 상대 pose factor\n중앙 ≈ 37.3 m", ORANGE),
        ("O-periodic", "Oracle 주기 연결", "GT 상대 pose factors\n10–70 m, 총 7회", "#16A34A"),
    ]
    for name, headline, detail, color in cards:
        condition_card(draw, (card_x, 372, card_x + card_width, 565), name, headline, detail, color)
        card_x += card_width + gap

    rounded(draw, (112, 610, 1770, 875), "#F7F7F2", ACCENT_GREEN, width=2, radius=16)
    draw.text((140, 634), "실험 순서", font=font(23, True), fill=OLIVE)
    steps = [
        ("1", "동일 rosbag", "두 UAV의 raw SLAM·odometry·GT 기록"),
        ("2", "조건별 trajectory", "B0/B1/O-single/O-periodic 생성"),
        ("3", "동일 scan 재투영", "pose만 바꾸어 지도 비교"),
        ("4", "정량 평가", "relative drift · yaw · Chamfer · F1"),
    ]
    step_x = 140
    for index, (number, title, detail) in enumerate(steps):
        rounded(draw, (step_x, 692, step_x + 350, 838), WHITE, ACCENT_GREEN, width=2, radius=14)
        rounded(draw, (step_x + 18, 710, step_x + 66, 758), GREEN, radius=12)
        draw.text((step_x + 33, 718), number, font=font(18, True), fill=WHITE)
        draw.text((step_x + 82, 708), title, font=font(20, True), fill=INK)
        paragraph(draw, (step_x + 22, 770), detail, font(16), MUTED, 305, 5)
        if index < len(steps) - 1:
            arrow_x0 = step_x + 354
            arrow_x1 = step_x + 391
            draw.line((arrow_x0, 765, arrow_x1, 765), fill=GREEN, width=5)
            draw.polygon([(arrow_x1, 765), (arrow_x1 - 13, 756), (arrow_x1 - 13, 774)], fill=GREEN)
        step_x += 405
    bottom_takeaway(canvas, "Oracle로 보정 가능성 확인 → 다음 단계에서 GT 상대 pose를 LiDAR 측정으로 교체")
    canvas.convert("RGB").save(output, quality=95)


def build_slide_5(background: Image.Image, output: Path, page_number: int = 5) -> None:
    canvas = original_style_base(background, "Oracle 실험에서 GT는 어디에 사용했는가?", "실험 가정", page_number)
    draw = ImageDraw.Draw(canvas)
    boxes = [
        ((115, 296, 480, 500), "1", "GT pose 기록", "Gazebo에서 두 UAV의\n동일 진행거리 pose 선택", OLIVE),
        ((535, 296, 900, 500), "2", "상대 pose factor", "선택된 keyframe 사이\nrelative transform 생성", ORANGE),
        ((955, 296, 1320, 500), "3", "Pose graph 최적화", "두 raw SLAM graph를\nInter-UAV factor로 연결", GREEN),
        ((1375, 296, 1740, 500), "4", "결과 평가", "relative drift와 지도 품질을\nGT 기준으로 정량 평가", BLUE),
    ]
    for box, number, title, detail, color in boxes:
        flow_card(canvas, box, number, title, detail, color)
    for left, right in zip(boxes, boxes[1:]):
        x0 = left[0][2] + 10
        x1 = right[0][0] - 10
        draw.line((x0, 398, x1, 398), fill=GREEN, width=6)
        draw.polygon([(x1, 398), (x1 - 16, 388), (x1 - 16, 408)], fill=GREEN)

    rounded(draw, (115, 555, 900, 690), "#F7F7F2", ACCENT_GREEN, width=2, radius=16)
    draw.text((142, 579), "O-single", font=font(25, True), fill=ORANGE)
    draw.text((142, 622), "공통 진행거리 중앙 ≈ 37.3 m에서 1회 연결", font=font(21), fill=INK)
    rounded(draw, (955, 555, 1740, 690), "#F7F7F2", ACCENT_GREEN, width=2, radius=16)
    draw.text((982, 579), "O-periodic", font=font(25, True), fill=GREEN)
    draw.text((982, 622), "10–70 m 구간, 약 10 m 간격으로 7회 연결", font=font(21), fill=INK)

    rounded(draw, (115, 735, 1740, 908), "#EFF6E5", ACCENT_GREEN, width=2, radius=16)
    draw.text((145, 758), "중요한 해석", font=font(25, True), fill=OLIVE)
    draw.text((145, 806), "GT로 전체 trajectory를 강제로 맞춘 실험이 아님", font=font(27, True), fill=INK)
    draw.text((145, 856), "GT가 상대 pose 측정값을 대신했을 때 보정 가능한지를 확인한 Oracle upper bound", font=font(21), fill=MUTED)
    bottom_takeaway(canvas, "Oracle 결과는 최종 방법이 아니라 LiDAR 상대 pose 보정의 가능성 검증")
    canvas.convert("RGB").save(output, quality=95)


def metric_box(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    label: str,
    value: str,
    detail: str,
    color: str,
) -> None:
    rounded(draw, box, "#F8FAF8", ACCENT_GREEN, width=2, radius=16)
    x0, y0, _, _ = box
    draw.text((x0 + 22, y0 + 18), label, font=font(18), fill=MUTED)
    draw.text((x0 + 22, y0 + 54), value, font=font(31, True), fill=color)
    draw.text((x0 + 22, y0 + 103), detail, font=font(17), fill=INK)


def build_slide_6(background: Image.Image, artifact: Path, output: Path, page_number: int = 6) -> None:
    canvas = original_style_base(background, "LiDAR 상대 Pose 추정: 가능 구간과 한계", "현재 구현 결과", page_number)
    draw = ImageDraw.Draw(canvas)
    left = (112, 290, 565, 805)
    center = (590, 290, 1043, 805)
    rounded(draw, left, WHITE, ACCENT_GREEN, width=2, radius=14)
    rounded(draw, center, WHITE, ACCENT_GREEN, width=2, radius=14)
    paste_contain(
        canvas,
        artifact / "lidar_registration/debug_plots/R-periodic_02_0030.0m.png",
        left,
        crop=(55, 0, 1165, 1175),
        padding=10,
    )
    paste_contain(
        canvas,
        artifact / "lidar_registration/debug_plots/R-periodic_05_0060.0m.png",
        center,
        crop=(55, 0, 1165, 1175),
        padding=10,
    )
    rounded(draw, (130, 305, 315, 348), "#E3F2D9", radius=12)
    draw.text((149, 312), "30 m · ACCEPT", font=font(17, True), fill=OLIVE)
    rounded(draw, (608, 305, 792, 348), "#F8DEDC", radius=12)
    draw.text((628, 312), "60 m · REJECT", font=font(17, True), fill=RED)
    draw.text((130, 824), "overlap 0.371 · B0 0.779 → LiDAR 0.440 m", font=font(18, True), fill=GREEN)
    draw.text((608, 824), "overlap 0.000 · 잘못된 factor를 넣지 않고 거절", font=font(18, True), fill=RED)

    metric_box(draw, (1080, 300, 1400, 452), "Accepted", "2 / 7", "confidence gate 통과", GREEN)
    metric_box(draw, (1425, 300, 1745, 452), "끝점 상대오차", "0.635 m", "B0 대비 −40.5%", GREEN)
    metric_box(draw, (1080, 480, 1400, 632), "Map Chamfer", "0.774 m", "B0 1.318 m에서 개선", OLIVE)
    metric_box(draw, (1425, 480, 1745, 632), "Occupied F1", "0.421", "B0 0.180에서 개선", OLIVE)
    rounded(draw, (1080, 678, 1745, 888), "#EFF6E5", ACCENT_GREEN, width=2, radius=16)
    draw.text((1105, 700), "남은 핵심 문제", font=font(24, True), fill=OLIVE)
    paragraph(
        draw,
        (1105, 750),
        "LiDAR 정합·gate·PGO는 구현됨\nGT 없이 ‘같은 장소’를 찾는 association front-end는 미완성",
        font(21),
        INK,
        600,
        9,
    )
    bottom_takeaway(canvas, "정합 계산보다 GT 없이 같은 장소 후보를 찾는 Association이 핵심 난제")
    canvas.convert("RGB").save(output, quality=95)


def strategy_box(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    badge: str,
    title: str,
    detail: str,
    color: str,
) -> None:
    rounded(draw, box, "#F8FAF8", ACCENT_GREEN, width=2, radius=16)
    x0, y0, _, _ = box
    rounded(draw, (x0 + 20, y0 + 18, x0 + 154, y0 + 57), color, radius=11)
    draw.text((x0 + 35, y0 + 23), badge, font=font(15, True), fill=WHITE)
    draw.text((x0 + 20, y0 + 78), title, font=font(23, True), fill=INK)
    paragraph(draw, (x0 + 20, y0 + 120), detail, font(18), MUTED, box[2] - x0 - 40, 7)


def build_slide_7(background: Image.Image, output: Path, page_number: int = 7) -> None:
    canvas = original_style_base(background, "고정 주기 대신 Adaptive Inter-UAV Constraint", "논문 확장 방향", page_number)
    draw = ImageDraw.Draw(canvas)
    rounded(draw, (110, 290, 830, 700), "#F8FAF8", ACCENT_GREEN, width=2, radius=16)
    draw.text((140, 318), "70 m run의 구간별 실제 overlap", font=font(24, True), fill=INK)
    left, top, bottom = 175, 420, 625
    draw.line((left, top, left, bottom), fill=MUTED, width=3)
    draw.line((left, bottom, 780, bottom), fill=MUTED, width=3)
    labels = ["0–20", "20–40", "40–60", "60–78.9"]
    values = [0.027, 0.250, 0.103, 0.000]
    colors = [ORANGE, GREEN, "#3C8D96", RED]
    for index, (label, value, color) in enumerate(zip(labels, values, colors)):
        x0 = 220 + index * 140
        height = int(value / 0.30 * 165)
        if height > 0:
            rounded(draw, (x0, bottom - height, x0 + 78, bottom), color, radius=9)
        draw.text((x0 + 6, bottom - height - 32), f"{value:.3f}", font=font(16, True), fill=color)
        draw.text((x0 + 5, bottom + 15), label, font=font(15), fill=INK)

    strategy_box(draw, (875, 290, 1160, 548), "BASELINE", "Fixed 10 m", "간단하지만 overlap이 없는 구간에서도 통신·정합 시도", ORANGE)
    strategy_box(draw, (1190, 290, 1475, 548), "PROPOSED", "Adaptive passive", "불확실성과 예상 overlap이 충분할 때만 constraint 생성", GREEN)
    strategy_box(draw, (1505, 290, 1790, 548), "FUTURE", "Active rendezvous", "overlap이 없고 drift가 위험할 때 최소 비행으로 중첩 생성", BLUE)

    rounded(draw, (875, 590, 1790, 720), "#EFF6E5", ACCENT_GREEN, width=2, radius=16)
    draw.text((905, 610), "Runtime trigger", font=font(22, True), fill=OLIVE)
    draw_fit(draw, (905, 661), "Drift 불확실성↑  ∧  예상 overlap↑  ∧  고유 geometry", 840, 25, GREEN, bold=True, min_size=20)

    rounded(draw, (110, 755, 1790, 910), "#F7F7F2", ACCENT_GREEN, width=2, radius=16)
    draw.text((140, 778), "실험 비교", font=font(23, True), fill=INK)
    draw.text((140, 824), "No constraint / Fixed 5·10·20·30 m / Adaptive passive / Adaptive active", font=font(21), fill=MUTED)
    draw.text((140, 865), "지표: relative drift · Chamfer/F1 · factor 수 · 통신량 · 추가 거리 · 임무 시간", font=font(20), fill=MUTED)
    bottom_takeaway(canvas, "정확도–우회거리–통신량 사이에서 가장 효율적인 연결 시점을 결정")
    canvas.convert("RGB").save(output, quality=95)


def find_shape(root: ET.Element, shape_id: int) -> ET.Element:
    for shape in root.findall(".//p:sp", NS):
        marker = shape.find("./p:nvSpPr/p:cNvPr", NS)
        if marker is not None and marker.get("id") == str(shape_id):
            return shape
    raise KeyError(f"shape id {shape_id} not found")


def set_shape_text(shape: ET.Element, text: str) -> None:
    nodes = shape.findall(".//a:t", NS)
    if not nodes:
        paragraph_node = shape.find("./p:txBody/a:p", NS)
        if paragraph_node is None:
            raise ValueError("shape has no text body")
        for child in list(paragraph_node):
            if child.tag != f"{{{A}}}pPr":
                paragraph_node.remove(child)
        run = ET.SubElement(paragraph_node, f"{{{A}}}r")
        run_props = ET.SubElement(
            run,
            f"{{{A}}}rPr",
            {
                "lang": "ko-KR",
                "altLang": "en-US",
                "sz": "3000",
                "b": "0",
                "dirty": "0",
            },
        )
        fill = ET.SubElement(run_props, f"{{{A}}}solidFill")
        ET.SubElement(fill, f"{{{A}}}srgbClr", {"val": "3D4B44"})
        for tag in ("latin", "ea", "cs"):
            ET.SubElement(run_props, f"{{{A}}}{tag}", {"typeface": "KoPubWorldDotum_Pro Medium"})
        node = ET.SubElement(run, f"{{{A}}}t")
        node.text = text
        ET.SubElement(paragraph_node, f"{{{A}}}endParaRPr", {"lang": "ko-KR", "sz": "3000"})
        return
    nodes[0].text = text
    for node in nodes[1:]:
        node.text = ""


def set_font_size(shape: ET.Element, points100: int) -> None:
    for tag in ("rPr", "endParaRPr", "defRPr"):
        for props in shape.findall(f".//a:{tag}", NS):
            props.set("sz", str(points100))


def set_text_color(shape: ET.Element, rgb: str) -> None:
    for props in shape.findall(".//a:rPr", NS) + shape.findall(".//a:endParaRPr", NS):
        solid = props.find("./a:solidFill", NS)
        if solid is None:
            solid = ET.SubElement(props, f"{{{A}}}solidFill")
        for child in list(solid):
            solid.remove(child)
        ET.SubElement(solid, f"{{{A}}}srgbClr", {"val": rgb})


def set_shape_geometry(shape: ET.Element, x: int, y: int, width: int, height: int) -> None:
    xfrm = shape.find("./p:spPr/a:xfrm", NS)
    if xfrm is None:
        raise ValueError("shape has no transform")
    off = xfrm.find("./a:off", NS)
    ext = xfrm.find("./a:ext", NS)
    if off is None or ext is None:
        raise ValueError("shape transform incomplete")
    off.set("x", str(x))
    off.set("y", str(y))
    ext.set("cx", str(width))
    ext.set("cy", str(height))


def set_alignment(shape: ET.Element, alignment: str) -> None:
    for props in shape.findall(".//a:pPr", NS):
        props.set("algn", alignment)


def update_existing_slides(entries: dict[str, bytes], page_offset: int = 0) -> None:
    slides = {index: ET.fromstring(entries[f"ppt/slides/slide{index}.xml"]) for index in range(1, 5)}

    cover_subtitle = find_shape(slides[1], 9)
    set_shape_text(cover_subtitle, "70 m One-Way Multi-UAV LiDAR SLAM Drift 보정 연구")
    set_shape_geometry(cover_subtitle, 1181100, 7750000, 14200000, 850000)
    set_alignment(cover_subtitle, "l")
    set_font_size(cover_subtitle, 3000)
    set_text_color(cover_subtitle, "3D4B44")

    slide2 = slides[2]
    set_shape_text(find_shape(slide2, 49), "70 m One-Way 비행에서 SLAM Drift 누적")
    set_shape_text(find_shape(slide2, 47), "Baseline 결과")
    conclusion2 = find_shape(slide2, 38)
    set_shape_text(
        conclusion2,
        "B0 끝점 상대오차 1.067 m · One-Way에서는 Loop Closure가 없어 drift 누적 · B1 = odometry baseline (GT 아님)",
    )
    set_font_size(conclusion2, 2700)
    set_shape_text(find_shape(slide2, 13), str(2 + page_offset))

    slide3 = slides[3]
    title3 = find_shape(slide3, 49)
    set_shape_text(title3, "주기적 Inter-UAV Constraint로 상대 Drift 91.7% 감소")
    set_font_size(title3, 5600)
    set_shape_geometry(title3, 403997, 367594, 23200000, 1200000)
    set_shape_text(find_shape(slide3, 47), "Oracle 보정 결과")
    conclusion3 = find_shape(slide3, 38)
    set_shape_text(
        conclusion3,
        "끝점 상대오차: B0 1.067 m → O-periodic 0.089 m (7회) · GT 상대 pose 기반 Oracle upper bound",
    )
    set_font_size(conclusion3, 2850)
    set_shape_geometry(conclusion3, 2535767, 12425000, 20500000, 750000)
    set_shape_text(find_shape(slide3, 13), str(3 + page_offset))

    slide4 = slides[4]
    title4 = find_shape(slide4, 49)
    set_shape_text(title4, "상대 Drift 감소 ≠ 절대 지도 정확도 보장")
    set_font_size(title4, 5800)
    set_shape_geometry(title4, 403997, 367594, 23200000, 1200000)
    set_shape_text(find_shape(slide4, 47), "지도 품질 비교")
    map_labels = {
        20: ("B0", 1850000, 3250000, 1800000, 520000),
        23: ("O-single", 13500000, 3180000, 3000000, 520000),
        33: ("O-periodic", 1850000, 7500000, 3400000, 520000),
        35: ("GT", 13500000, 7650000, 1800000, 520000),
    }
    for shape_id, (label, x, y, width, height) in map_labels.items():
        shape = find_shape(slide4, shape_id)
        set_shape_text(shape, label)
        set_font_size(shape, 2600)
        set_shape_geometry(shape, x, y, width, height)
        set_alignment(shape, "l")
    conclusion4 = find_shape(slide4, 38)
    set_shape_text(
        conclusion4,
        "O-periodic은 B0보다 개선되지만 B1보다 절대 지도 정확도는 낮음 (Chamfer 0.935 vs 0.441)",
    )
    set_font_size(conclusion4, 2850)
    set_text_color(conclusion4, "171A18")
    set_shape_geometry(conclusion4, 2535767, 12425000, 20500000, 750000)
    set_shape_text(find_shape(slide4, 13), str(4 + page_offset))

    for index, root in slides.items():
        entries[f"ppt/slides/slide{index}.xml"] = ET.tostring(
            root, encoding="utf-8", xml_declaration=True
        )


def full_slide_xml() -> bytes:
    xml = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sld xmlns:a="{A}" xmlns:r="{R}" xmlns:p="{P}"><p:cSld><p:spTree><p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr><p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr><p:pic><p:nvPicPr><p:cNvPr id="2" name="Full-slide added content"/><p:cNvPicPr><a:picLocks noChangeAspect="1"/></p:cNvPicPr><p:nvPr/></p:nvPicPr><p:blipFill><a:blip r:embed="rId2"/><a:stretch><a:fillRect/></a:stretch></p:blipFill><p:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="24384000" cy="13716000"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr></p:pic></p:spTree></p:cSld><p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr></p:sld>'''
    return xml.encode("utf-8")


def slide_relationships(image_name: str) -> bytes:
    xml = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="{REL}"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout7.xml"/><Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/{image_name}"/></Relationships>'''
    return xml.encode("utf-8")


def add_slides(entries: dict[str, bytes], slide_images: list[Path], intro_count: int = 0) -> None:
    presentation_additions: list[bytes] = []
    relationship_additions: list[bytes] = []
    content_type_additions: list[bytes] = []
    slide_template = entries["ppt/slides/slide1.xml"]
    rels_template = entries["ppt/slides/_rels/slide1.xml.rels"]
    for offset, image_path in enumerate(slide_images, start=5):
        relationship_id = f"rId{offset + 6}"
        image_name = f"image{offset + 8}.png"
        slide_id = 292 + offset
        presentation_additions.append(
            f'<p:sldId id="{slide_id}" r:id="{relationship_id}"/>'.encode("utf-8")
        )
        relationship_additions.append(
            (
                f'<Relationship Id="{relationship_id}" '
                'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" '
                f'Target="slides/slide{offset}.xml"/>'
            ).encode("utf-8")
        )
        content_type_additions.append(
            (
                f'<Override PartName="/ppt/slides/slide{offset}.xml" '
                'ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml"/>'
            ).encode("utf-8")
        )

        overlay = (
            f'<p:pic><p:nvPicPr><p:cNvPr id="99" name="Added slide {offset}"/>'
            '<p:cNvPicPr><a:picLocks noChangeAspect="1"/></p:cNvPicPr><p:nvPr/>'
            '</p:nvPicPr><p:blipFill><a:blip r:embed="rId6"/><a:stretch><a:fillRect/>'
            '</a:stretch></p:blipFill><p:spPr><a:xfrm><a:off x="0" y="0"/>'
            '<a:ext cx="24384000" cy="13716000"/></a:xfrm><a:prstGeom prst="rect">'
            '<a:avLst/></a:prstGeom></p:spPr></p:pic>'
        ).encode("utf-8")
        entries[f"ppt/slides/slide{offset}.xml"] = slide_template.replace(
            b"</p:spTree>", overlay + b"</p:spTree>", 1
        )
        image_relationship = (
            f'<Relationship Id="rId6" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" '
            f'Target="../media/{image_name}"/>'
        ).encode("utf-8")
        entries[f"ppt/slides/_rels/slide{offset}.xml.rels"] = rels_template.replace(
            b"</Relationships>", image_relationship + b"</Relationships>", 1
        )
        entries[f"ppt/media/{image_name}"] = image_path.read_bytes()

    presentation_xml = entries["ppt/presentation.xml"]
    slide_list_match = re.search(rb"<p:sldIdLst>(.*?)</p:sldIdLst>", presentation_xml)
    if slide_list_match is None:
        raise ValueError("presentation is missing slide list")
    original_slide_ids = re.findall(rb"<p:sldId\b[^>]*/>", slide_list_match.group(1))
    ordered_slide_ids = (
        original_slide_ids[:1]
        + presentation_additions[:intro_count]
        + original_slide_ids[1:]
        + presentation_additions[intro_count:]
    )
    new_slide_list = b"<p:sldIdLst>" + b"".join(ordered_slide_ids) + b"</p:sldIdLst>"
    entries["ppt/presentation.xml"] = (
        presentation_xml[: slide_list_match.start()]
        + new_slide_list
        + presentation_xml[slide_list_match.end() :]
    )
    entries["ppt/_rels/presentation.xml.rels"] = entries[
        "ppt/_rels/presentation.xml.rels"
    ].replace(b"</Relationships>", b"".join(relationship_additions) + b"</Relationships>", 1)
    entries["[Content_Types].xml"] = entries["[Content_Types].xml"].replace(
        b"</Types>", b"".join(content_type_additions) + b"</Types>", 1
    )
    entries["docProps/app.xml"] = entries["docProps/app.xml"].replace(
        b"<Slides>4</Slides>", f"<Slides>{4 + len(slide_images)}</Slides>".encode("utf-8"), 1
    )


def prepare_cover_asset(source: Path) -> bytes:
    image = Image.open(source).convert("RGB")
    cleaned = Image.new("RGB", image.size)
    cleaned.putdata(
        [
            (238, 238, 238)
            if min(pixel) > 205 and max(pixel) - min(pixel) < 25
            else pixel
            for pixel in image.getdata()
        ]
    )
    image = cleaned
    target_aspect = 6515100 / 2997200
    target_height = int(round(image.width / target_aspect))
    if target_height < image.height:
        crop_top = min(130, image.height - target_height)
        image = image.crop((0, crop_top, image.width, crop_top + target_height))
    target_width = int(round(image.height * target_aspect))
    canvas = Image.new("RGB", (target_width, image.height), BG)
    canvas.paste(image, ((target_width - image.width) // 2, 0))
    buffer = BytesIO()
    canvas.save(buffer, "PNG")
    return buffer.getvalue()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("자율주행군집드론_0828미팅1차.pptx"))
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/presentation/0828_meeting_review/original_based/자율주행군집드론_0828미팅1차_원본기반수정.pptx"),
    )
    parser.add_argument(
        "--cover-asset",
        type=Path,
        default=Path("docs/presentation/0828_meeting_review/original_based/assets/multi_uav_lidar_cover.png"),
    )
    parser.add_argument(
        "--artifact",
        type=Path,
        default=Path("artifacts/2026-08-27_lidar_phase2_70m_codex_v1"),
    )
    parser.add_argument(
        "--context-asset",
        type=Path,
        default=Path("docs/presentation/0828_meeting_review/original_based/assets/experiment_context_visual.png"),
    )
    parser.add_argument(
        "--include-intro",
        action="store_true",
        help="insert research-situation and experiment-setup slides after the cover",
    )
    parser.add_argument("--skip-existing", action="store_true", help="debug: do not patch slides 1–4")
    parser.add_argument("--skip-added", action="store_true", help="debug: do not add image slides")
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    slide_dir = args.output.parent / (
        "added_slide_images_with_intro" if args.include_intro else "added_slide_images"
    )
    slide_dir.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(args.input, "r") as archive:
        entries = {name: archive.read(name) for name in archive.namelist()}
    background = Image.open(BytesIO(entries["ppt/media/image1.png"])).convert("RGB")

    intro_images: list[Path] = []
    page_offset = 2 if args.include_intro else 0
    if args.include_intro:
        intro_images = [slide_dir / "slide-2-situation.png", slide_dir / "slide-3-setup.png"]
        build_intro_situation(background, args.context_asset, intro_images[0], page_number=2)
        build_intro_setup(background, intro_images[1], page_number=3)

    extra_pages = [5 + page_offset, 6 + page_offset, 7 + page_offset]
    extra_images = [slide_dir / f"slide-{page}.png" for page in extra_pages]
    build_slide_5(background, extra_images[0], page_number=extra_pages[0])
    build_slide_6(background, args.artifact, extra_images[1], page_number=extra_pages[1])
    build_slide_7(background, extra_images[2], page_number=extra_pages[2])
    slide_images = intro_images + extra_images

    if not args.skip_existing:
        update_existing_slides(entries, page_offset=page_offset)
        entries["ppt/media/image4.png"] = prepare_cover_asset(args.cover_asset)
    if not args.skip_added:
        add_slides(entries, slide_images, intro_count=len(intro_images))

    with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in entries.items():
            archive.writestr(name, data)
    print(f"Wrote original-based {4 + (0 if args.skip_added else len(slide_images))}-slide deck: {args.output}")


if __name__ == "__main__":
    main()
