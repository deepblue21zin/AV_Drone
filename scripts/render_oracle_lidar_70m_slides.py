#!/usr/bin/env python3
"""Render four Korean result slides from the 70 m cSLAM artifacts.

The charts and maps are frozen experiment outputs.  Image-generation assets are
used only as restrained slide backgrounds so that labels and metrics remain
deterministic and reproducible.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

from PIL import Image, ImageDraw, ImageFont


WIDTH = 1920
HEIGHT = 1080
NAVY = "#0b1f33"
TEAL = "#0f7c86"
GREEN = "#16a34a"
RED = "#dc2626"
ORANGE = "#f59e0b"
BLUE = "#2563eb"
INK = "#172033"
MUTED = "#5f6b7a"
LINE = "#d9e1e8"
WHITE = "#ffffff"
OFF_WHITE = "#faf9f5"
RESAMPLE = getattr(Image, "Resampling", Image).LANCZOS

FONT_REGULAR = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FONT_MEDIUM = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")


def font(size: int, weight: str = "regular") -> ImageFont.FreeTypeFont:
    path = {
        "regular": FONT_REGULAR,
        "medium": FONT_MEDIUM,
        "bold": FONT_BOLD,
    }[weight]
    return ImageFont.truetype(str(path), size=size)


def fit_background(path: Path) -> Image.Image:
    image = Image.open(path).convert("RGB")
    target_ratio = WIDTH / HEIGHT
    ratio = image.width / image.height
    if ratio > target_ratio:
        crop_width = round(image.height * target_ratio)
        left = (image.width - crop_width) // 2
        image = image.crop((left, 0, left + crop_width, image.height))
    elif ratio < target_ratio:
        crop_height = round(image.width / target_ratio)
        top = (image.height - crop_height) // 2
        image = image.crop((0, top, image.width, top + crop_height))
    return image.resize((WIDTH, HEIGHT), RESAMPLE).convert("RGBA")


def rounded_rect(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    *,
    radius: int,
    fill,
    outline=None,
    width: int = 1,
) -> None:
    """Pillow-7-compatible rounded rectangle."""
    x0, y0, x1, y1 = [int(value) for value in box]
    radius = max(1, min(int(radius), (x1 - x0) // 2, (y1 - y0) // 2))
    draw.rectangle((x0 + radius, y0, x1 - radius, y1), fill=fill)
    draw.rectangle((x0, y0 + radius, x1, y1 - radius), fill=fill)
    draw.pieslice((x0, y0, x0 + 2 * radius, y0 + 2 * radius), 180, 270, fill=fill)
    draw.pieslice((x1 - 2 * radius, y0, x1, y0 + 2 * radius), 270, 360, fill=fill)
    draw.pieslice((x1 - 2 * radius, y1 - 2 * radius, x1, y1), 0, 90, fill=fill)
    draw.pieslice((x0, y1 - 2 * radius, x0 + 2 * radius, y1), 90, 180, fill=fill)
    if outline is None:
        return
    for offset in range(max(1, int(width))):
        draw.line((x0 + radius, y0 + offset, x1 - radius, y0 + offset), fill=outline)
        draw.line((x0 + radius, y1 - offset, x1 - radius, y1 - offset), fill=outline)
        draw.line((x0 + offset, y0 + radius, x0 + offset, y1 - radius), fill=outline)
        draw.line((x1 - offset, y0 + radius, x1 - offset, y1 - radius), fill=outline)
        draw.arc((x0 + offset, y0 + offset, x0 + 2 * radius, y0 + 2 * radius), 180, 270, fill=outline)
        draw.arc((x1 - 2 * radius, y0 + offset, x1 - offset, y0 + 2 * radius), 270, 360, fill=outline)
        draw.arc((x1 - 2 * radius, y1 - 2 * radius, x1 - offset, y1 - offset), 0, 90, fill=outline)
        draw.arc((x0 + offset, y1 - 2 * radius, x0 + 2 * radius, y1 - offset), 90, 180, fill=outline)


def add_card(
    canvas: Image.Image,
    box: tuple[int, int, int, int],
    *,
    radius: int = 28,
    fill: tuple[int, int, int, int] = (255, 255, 255, 244),
    outline: str = LINE,
) -> None:
    shadow = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    shadow_draw = ImageDraw.Draw(shadow)
    x0, y0, x1, y1 = box
    rounded_rect(
        shadow_draw,
        (x0 + 5, y0 + 9, x1 + 5, y1 + 9),
        radius=radius,
        fill=(11, 31, 51, 24),
    )
    canvas.alpha_composite(shadow)
    draw = ImageDraw.Draw(canvas)
    rounded_rect(draw, box, radius=radius, fill=fill, outline=outline, width=2)


def paste_contain(
    canvas: Image.Image,
    path: Path,
    box: tuple[int, int, int, int],
    *,
    crop: tuple[int, int, int, int] | None = None,
    padding: int = 12,
) -> None:
    image = Image.open(path).convert("RGBA")
    if crop is not None:
        image = image.crop(crop)
    x0, y0, x1, y1 = box
    available = (max(1, x1 - x0 - padding * 2), max(1, y1 - y0 - padding * 2))
    image.thumbnail(available, RESAMPLE)
    x = x0 + (x1 - x0 - image.width) // 2
    y = y0 + (y1 - y0 - image.height) // 2
    canvas.alpha_composite(image, (x, y))


def text_width(draw: ImageDraw.ImageDraw, value: str, face: ImageFont.FreeTypeFont) -> int:
    if hasattr(draw, "textbbox"):
        bounds = draw.textbbox((0, 0), value, font=face)
        return bounds[2] - bounds[0]
    return int(draw.textsize(value, font=face)[0])


def wrap_text(
    draw: ImageDraw.ImageDraw,
    value: str,
    face: ImageFont.FreeTypeFont,
    max_width: int,
) -> list[str]:
    lines: list[str] = []
    for paragraph in value.split("\n"):
        if not paragraph:
            lines.append("")
            continue
        current = ""
        for word in paragraph.split(" "):
            candidate = word if not current else f"{current} {word}"
            if text_width(draw, candidate, face) <= max_width:
                current = candidate
            else:
                if current:
                    lines.append(current)
                current = word
        if current:
            lines.append(current)
    return lines


def paragraph(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    value: str,
    face: ImageFont.FreeTypeFont,
    fill: str,
    max_width: int,
    *,
    line_gap: int = 10,
) -> int:
    x, y = xy
    lines = wrap_text(draw, value, face, max_width)
    line_height = face.size + line_gap
    for index, line in enumerate(lines):
        draw.text((x, y + index * line_height), line, font=face, fill=fill)
    return y + len(lines) * line_height


def header(
    canvas: Image.Image,
    section: str,
    title: str,
    subtitle: str,
    number: str,
) -> None:
    draw = ImageDraw.Draw(canvas)
    draw.text((82, 48), section, font=font(20, "bold"), fill=TEAL)
    draw.text((82, 82), title, font=font(48, "bold"), fill=NAVY)
    draw.text((84, 145), subtitle, font=font(23), fill=MUTED)
    draw.text((1810, 56), number, font=font(25, "bold"), fill=NAVY)


def metric_card(
    canvas: Image.Image,
    box: tuple[int, int, int, int],
    label: str,
    value: str,
    note: str,
    color: str,
) -> None:
    add_card(canvas, box, radius=22, fill=(255, 255, 255, 247))
    draw = ImageDraw.Draw(canvas)
    x0, y0, x1, _ = box
    rounded_rect(draw, (x0, y0, x0 + 9, box[3]), radius=5, fill=color)
    draw.text((x0 + 28, y0 + 20), label, font=font(19, "medium"), fill=MUTED)
    draw.text((x0 + 28, y0 + 50), value, font=font(37, "bold"), fill=color)
    draw.text((x0 + 28, y0 + 99), note, font=font(17), fill=MUTED)


def draw_bullets(
    draw: ImageDraw.ImageDraw,
    items: Iterable[str],
    x: int,
    y: int,
    width: int,
    *,
    bullet_color: str = TEAL,
    face: ImageFont.FreeTypeFont | None = None,
    gap: int = 18,
) -> int:
    face = face or font(22)
    current_y = y
    for item in items:
        draw.ellipse((x, current_y + 11, x + 9, current_y + 20), fill=bullet_color)
        current_y = paragraph(
            draw,
            (x + 24, current_y),
            item,
            face,
            INK,
            width - 24,
            line_gap=8,
        ) + gap
    return current_y


def slide_one(repo: Path, output: Path) -> None:
    base = repo / "docs/presentation/oracle_lidar_70m"
    artifact = repo / "artifacts/2026-08-27_lidar_phase2_70m_codex_v1"
    canvas = fit_background(base / "backgrounds/slide_01_drift.png")
    header(
        canvas,
        "RESULT 01 · PROBLEM OBSERVATION",
        "70 m One-Way 비행에서 Raw SLAM Drift가 누적됨",
        "동일 rosbag · 2 UAV · loop closure OFF · 공통 진행 거리 74.7 m",
        "01",
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
    draw.text((1462, 742), "관찰", font=font(22, "bold"), fill=RED)
    draw_bullets(
        draw,
        [
            "UAV2의 B0 궤적이 x≈25 m 이후 GT에서 크게 이탈",
            "반복 원통 환경의 scan matching 오류가 누적된 것으로 해석",
        ],
        1462,
        784,
        350,
        bullet_color=RED,
        face=font(19),
        gap=12,
    )
    draw.text(
        (72, 1015),
        "B0 = raw slam_toolbox · B1 = known spawn + PX4/MAVROS odometry (Gazebo GT 아님)",
        font=font(18),
        fill=MUTED,
    )
    canvas.convert("RGB").save(output / "01_raw_slam_drift.png", quality=95)


def slide_two(repo: Path, output: Path) -> None:
    base = repo / "docs/presentation/oracle_lidar_70m"
    artifact = repo / "artifacts/2026-08-27_lidar_phase2_70m_codex_v1"
    canvas = fit_background(base / "backgrounds/slide_02_oracle.png")
    header(
        canvas,
        "RESULT 02 · ORACLE FEASIBILITY",
        "주기적인 Inter-UAV Constraint는 상대 Drift를 91.7% 감소",
        "정확한 상대 pose가 주어진다는 상한선 실험 · O-single 1개 vs O-periodic 7개",
        "02",
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
    draw.text((1470, 823), "해석 범위", font=font(20, "bold"), fill=ORANGE)
    paragraph(
        draw,
        (1470, 858),
        "GT 상대 pose(+측정 노이즈)를 factor로 사용한 가능성 검증이다. 자동 장소 인식 성공을 뜻하지 않는다.",
        font(18),
        INK,
        350,
        line_gap=7,
    )
    draw.text(
        (72, 1015),
        "핵심: one-way에서는 self loop closure가 없으므로, 한 번의 연결보다 반복되는 cross-UAV 연결이 필요",
        font=font(18),
        fill=MUTED,
    )
    canvas.convert("RGB").save(output / "02_oracle_relative_drift.png", quality=95)


def slide_three(repo: Path, output: Path) -> None:
    base = repo / "docs/presentation/oracle_lidar_70m"
    artifact = repo / "artifacts/2026-08-27_lidar_phase2_70m_codex_v1"
    canvas = fit_background(base / "backgrounds/slide_03_map.png")
    header(
        canvas,
        "RESULT 03 · MAP QUALITY",
        "상대 일관성 개선이 곧 절대 지도 최적화를 의미하지는 않음",
        "모든 조건에서 동일 LiDAR scan을 pose만 바꾸어 재투영 · 청록 outline = GT occupied",
        "03",
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
    paragraph(
        draw,
        (1480, 855),
        "Oracle 보정은 B0보다 지도 품질도 개선했지만, 강한 odometry baseline인 B1을 아직 넘지 못했다.",
        font(18),
        INK,
        345,
        line_gap=7,
    )
    draw.text(
        (72, 1015),
        "따라서 논문 평가는 relative drift · yaw · trajectory · Chamfer · F1을 함께 보고해야 함",
        font=font(18),
        fill=MUTED,
    )
    canvas.convert("RGB").save(output / "03_map_quality_analysis.png", quality=95)


def slide_four(repo: Path, output: Path) -> None:
    base = repo / "docs/presentation/oracle_lidar_70m"
    artifact = repo / "artifacts/2026-08-27_lidar_phase2_70m_codex_v1"
    canvas = fit_background(base / "backgrounds/slide_04_lidar.png")
    header(
        canvas,
        "RESULT 04 · CONTROLLED LIDAR REGISTRATION",
        "LiDAR 상대 Pose 추정은 효과 확인 — GT-free Association은 아직 미완성",
        "GT는 비교할 keyframe pair만 선택 · 실제 relative-pose measurement에는 GT transform 미사용",
        "04",
    )

    accepted_box = (58, 210, 500, 820)
    rejected_box = (520, 210, 962, 820)
    add_card(canvas, accepted_box)
    add_card(canvas, rejected_box)
    paste_contain(
        canvas,
        artifact / "lidar_registration/debug_plots/R-periodic_02_0030.0m.png",
        accepted_box,
        crop=(55, 0, 1165, 1175),
        padding=14,
    )
    paste_contain(
        canvas,
        artifact / "lidar_registration/debug_plots/R-periodic_05_0060.0m.png",
        rejected_box,
        crop=(55, 0, 1165, 1175),
        padding=14,
    )

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
    draw.text(
        (72, 1015),
        "논문 메시지: measurement backend의 가능성은 확인했으며, 완전 GT-free 제안은 place association 구현 후 평가",
        font=font(18),
        fill=MUTED,
    )
    canvas.convert("RGB").save(output / "04_lidar_registration_and_gap.png", quality=95)


def contact_sheet(slides: list[Path], output: Path) -> None:
    thumb_width = 960
    thumb_height = 540
    sheet = Image.new("RGB", (thumb_width * 2, thumb_height * 2), OFF_WHITE)
    for index, path in enumerate(slides):
        image = Image.open(path).convert("RGB").resize(
            (thumb_width, thumb_height), RESAMPLE
        )
        sheet.paste(image, ((index % 2) * thumb_width, (index // 2) * thumb_height))
    sheet.save(output / "00_contact_sheet.png", quality=95)


def slide_pdf(slides: list[Path], output: Path) -> None:
    pages = [Image.open(path).convert("RGB") for path in slides]
    if not pages:
        return
    pages[0].save(
        output / "oracle_lidar_70m_results.pdf",
        "PDF",
        resolution=150.0,
        save_all=True,
        append_images=pages[1:],
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Default: docs/presentation/oracle_lidar_70m/slides",
    )
    args = parser.parse_args()
    repo = args.repo_root.resolve()
    output = (
        args.output.resolve()
        if args.output is not None
        else repo / "docs/presentation/oracle_lidar_70m/slides"
    )
    output.mkdir(parents=True, exist_ok=True)

    slide_one(repo, output)
    slide_two(repo, output)
    slide_three(repo, output)
    slide_four(repo, output)
    slides = sorted(output.glob("0[1-4]_*.png"))
    contact_sheet(slides, output)
    slide_pdf(slides, output)
    print(f"Rendered {len(slides)} slides to {output}")


if __name__ == "__main__":
    main()
