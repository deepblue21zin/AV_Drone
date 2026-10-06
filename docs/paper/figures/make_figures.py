"""Draw the paper figures as SVG from the small JSON files in ./data.

  python3 make_figures.py        # standard library only; writes fig1..fig5 .svg next to this file

World layouts come from sim_assets/worlds/*_layout.json. Colours are paired with marker shapes
(filled circle / cross / hollow circle) so the figures survive greyscale printing.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, "../../.."))
INK, MUTED, RULE, BLUE, RED, ORANGE, AMBER = "#13202A", "#6B7883", "#D9DFE3", "#1F5FAF", "#B42335", "#C0572B", "#9A6A12"
TINT = {"RU": BLUE, "RR": RED, "SU": MUTED, "SR": AMBER}
NAME = {"RU": "많음·유일", "RR": "많음·반복", "SU": "적음·유일", "SR": "적음·반복"}
FONT = "'Noto Sans KR','IBM Plex Sans KR','Malgun Gothic',sans-serif"


def data(name):
    return json.load(open(os.path.join(HERE, "data", name)))


def svg(width, height, body):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
            f'font-family="{FONT}" font-size="12" fill="{INK}">\n<rect width="{width}" height="{height}" fill="#FFFFFF"/>\n'
            + "\n".join(body) + "\n</svg>\n")


def text(x, y, s, anchor="start", size=12, fill=INK, weight="normal"):
    return f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" font-size="{size}" fill="{fill}" font-weight="{weight}">{s}</text>'


def cross(x, y, r=5, colour=RED, width=2.2):
    return (f'<path d="M{x - r:.1f} {y - r:.1f}L{x + r:.1f} {y + r:.1f}M{x + r:.1f} {y - r:.1f}L{x - r:.1f} {y + r:.1f}" '
            f'stroke="{colour}" stroke-width="{width}" stroke-linecap="round" fill="none"/>')


def dot(x, y, r=4, colour=BLUE):
    return f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="{colour}" stroke="#FFFFFF" stroke-width="1"/>'


def ring(x, y, r=4, colour=MUTED, width=1.5):
    return f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="#FFFFFF" stroke="{colour}" stroke-width="{width}"/>'


def corridor(g, x0, y0, scale, zones, cylinders, tint_from_y=15.0, tint_to_y=-15.0):
    """Top view of the 150 m x 30 m corridor; returns world->pixel mappers."""
    X = lambda x: x0 + x * scale
    Y = lambda y: y0 + (15.0 - y) * scale
    g.append(f'<rect x="{X(0):.1f}" y="{Y(15):.1f}" width="{150 * scale:.1f}" height="{30 * scale:.1f}" fill="#F7F9FA" stroke="{INK}" stroke-width="1.2"/>')
    for a, b, kind in zones:
        g.append(f'<rect x="{X(a):.1f}" y="{Y(tint_from_y):.1f}" width="{(b - a) * scale:.1f}" height="{(tint_from_y - tint_to_y) * scale:.1f}" fill="{TINT[kind]}" fill-opacity="0.13"/>')
    for c in cylinders:
        g.append(f'<circle cx="{X(c[0]):.1f}" cy="{Y(c[1]):.1f}" r="{max(0.5 * scale, 1.8):.1f}" fill="#FFFFFF" stroke="{INK}" stroke-width="0.9"/>')
    return X, Y


def path(points, X, Y, colour, width=2, dash=None):
    d = " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in points)
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<polyline points="{d}" fill="none" stroke="{colour}" stroke-width="{width}"{extra}/>'


def fig1():
    layout = json.load(open(os.path.join(PROJECT, "sim_assets/worlds/zoned_corridor_w1_layout.json")))
    zones = [[z["x_min"], z["x_max"], z["type"]] for z in layout["zones"]]
    cyl = [[c["x"], c["y"]] for c in layout["cylinders"]]
    g = [text(10, 16, "(a) 구간형 월드 w1 (위에서 본 모습)", weight="bold")]
    X, Y = corridor(g, 30, 44, 4.5, zones, cyl, tint_from_y=-1.5)
    for a, b, kind in zones:
        g.append(text(X((a + b) / 2), 38, NAME[kind], "middle", 11.5, TINT[kind], "bold"))
    g.append(f'<line x1="{X(3)}" y1="{Y(-7.5)}" x2="{X(142)}" y2="{Y(-7.5)}" stroke="{BLUE}" stroke-width="2"/>')
    g.append(text(X(146), Y(-7.5) + 4, "기체 1 차선", "start", 10.5, BLUE))
    for v in (0, 50, 100, 150):
        g.append(text(X(v), Y(-15) + 15, f"{v} m", "middle", 10.5, MUTED))
    # (b) error strip
    top = 228
    g.append(text(10, top - 14, "(b) 같은 장소 후보의 정합 병진 오차", weight="bold"))
    EX = lambda v: 150 + min(v, 6.0) * 88
    for v in range(7):
        g.append(f'<line x1="{EX(v)}" y1="{top}" x2="{EX(v)}" y2="{top + 122}" stroke="{RULE}"/>')
        g.append(text(EX(v), top + 138, f"{v} m", "middle", 10.5, MUTED))
    errors = data("fig1_errors.json")
    for row, kind in enumerate(("RU", "RR", "SU", "SR")):
        y = top + 16 + row * 30
        sel = [e for e in errors if e["zone"] == kind]
        wrong = sum(not e["correct"] for e in sel)
        g.append(text(140, y + 4, f"{NAME[kind]} ({len(sel)})", "end", 11.5))
        for i, e in enumerate(sel):
            jitter = ((i * 37) % 17) - 8
            if e["correct"]:
                g.append(ring(EX(e["err"]), y + jitter * 0.9, 3.2, BLUE, 1.2))
            else:
                g.append(cross(EX(e["err"]), y + jitter * 0.9, 4, RED, 1.8))
        g.append(text(EX(6) + 10, y + 4, f"오정합 {wrong}", "start", 11.5, RED if wrong else MUTED, "bold" if wrong else "normal"))
    g.append(ring(150, top + 158, 3.2, BLUE, 1.2) + text(160, top + 162, "정답", size=11))
    g.append(cross(210, top + 158, 4, RED, 1.8) + text(220, top + 162, "오정합 (복사 간격 4 m, 5 m에 집중)", size=11))
    return svg(760, 400, g)


def zoom(g, case, x_lo, x_hi, y_lo, y_hi, x0, y0, scale, estimate_colour=ORANGE):
    X = lambda x: x0 + (x - x_lo) * scale
    Y = lambda y: y0 + (y_hi - y) * scale
    w, h = (x_hi - x_lo) * scale, (y_hi - y_lo) * scale
    g.append(f'<clipPath id="clip{int(x0)}{int(y0)}"><rect x="{x0}" y="{y0}" width="{w:.1f}" height="{h:.1f}"/></clipPath>')
    g.append(f'<rect x="{x0}" y="{y0}" width="{w:.1f}" height="{h:.1f}" fill="#F7F9FA" stroke="{INK}"/>')
    inner = []
    for a, b, kind in case["zones"]:
        lo, hi = max(a, x_lo), min(b, x_hi)
        if hi > lo:
            inner.append(f'<rect x="{X(lo):.1f}" y="{y0}" width="{(hi - lo) * scale:.1f}" height="{h:.1f}" fill="{TINT[kind]}" fill-opacity="0.10"/>')
    for cx, cy, _ in case["cyl"]:
        if x_lo - 1 < cx < x_hi + 1 and y_lo - 1 < cy < y_hi + 1:
            inner.append(f'<circle cx="{X(cx):.1f}" cy="{Y(cy):.1f}" r="{0.5 * scale:.1f}" fill="none" stroke="{INK}" stroke-width="1" opacity="0.55"/>')
    for x, y in case["src"]:
        inner.append(f'<circle cx="{X(x):.1f}" cy="{Y(y):.1f}" r="1.9" fill="{BLUE}"/>')
    for x, y in case["tgt_true"][::2]:
        inner.append(f'<circle cx="{X(x):.1f}" cy="{Y(y):.1f}" r="2.8" fill="none" stroke="{MUTED}" stroke-width="1.1"/>')
    for x, y in case["tgt_est"][::2]:
        inner.append(f'<circle cx="{X(x):.1f}" cy="{Y(y):.1f}" r="2" fill="{estimate_colour}"/>')
    g.append(f'<g clip-path="url(#clip{int(x0)}{int(y0)})">' + "".join(inner) + "</g>")
    for v in range(int(x_lo // 4 * 4 + 4), int(x_hi) + 1, 4):
        g.append(text(X(v), y0 + h + 14, f"{v} m", "middle", 10.5, MUTED))
    return X, Y


def legend_points(g, x, y):
    g.append(f'<circle cx="{x}" cy="{y - 4}" r="2.4" fill="{BLUE}"/>' + text(x + 8, y, "기체 1 submap", size=11))
    g.append(f'<circle cx="{x + 110}" cy="{y - 4}" r="3" fill="none" stroke="{MUTED}" stroke-width="1.2"/>' + text(x + 118, y, "기체 2 submap · 원래 위치", size=11))
    g.append(f'<circle cx="{x + 290}" cy="{y - 4}" r="2.4" fill="{ORANGE}"/>' + text(x + 298, y, "기체 2 submap · 정합이 맞춘 위치", size=11))


def shift_arrow(g, case, X, y):
    mean = lambda pts: sum(p[0] for p in pts) / len(pts)
    a, b = X(mean(case["tgt_true"])), X(mean(case["tgt_est"]))
    head = 7 if b < a else -7
    g.append(f'<line x1="{a:.1f}" y1="{y}" x2="{b:.1f}" y2="{y}" stroke="{INK}" stroke-width="2"/>')
    g.append(f'<path d="M{b:.1f} {y}l{head} -5v10z" fill="{INK}"/>')
    g.append(text((a + b) / 2, y - 7, f"{case['err_m']:.1f} m 어긋남", "middle", 12, INK, "bold"))


def fig2():
    case = data("fig2_case.json")
    g = [text(10, 16, "따라가기 비행의 오정합 사례 (많음·반복 구간, 복사 간격 4 m)", weight="bold")]
    cx = case["d1"][0]
    # only the band between the lane and the corridor centre holds cylinders in this zone
    X, Y = zoom(g, case, cx - 12, cx + 12, -7.0, -1.0, 20, 50, 30)
    shift_arrow(g, case, X, 40)
    legend_points(g, 24, 266)
    return svg(760, 278, g)


def panel_axes(g, x0, y0, w, h, x_ticks, y_ticks, x_map, y_map, x_label, y_format):
    for v in y_ticks:
        g.append(f'<line x1="{x0}" y1="{y_map(v):.1f}" x2="{x0 + w}" y2="{y_map(v):.1f}" stroke="{RULE}"/>')
        g.append(text(x0 - 6, y_map(v) + 4, y_format(v), "end", 10.5, MUTED))
    for v, label in x_ticks:
        g.append(text(x_map(v), y0 + h + 15, label, "middle", 10.5, MUTED))
    g.append(text(x0 + w / 2, y0 + h + 32, x_label, "middle", 11, MUTED))


def fig3():
    rates = data("fig3_rates.json")
    g = []
    styles = {"fixed6": (RED, None, "고정 ±6 m"), "k3": (BLUE, None, "3σ 범위"), "k2": (MUTED, "5 4", "2σ 범위")}
    for col, (group, title) in enumerate((("repeated", "(a) 반복 구간 (많음·반복 + 적음·반복)"), ("unique", "(b) 유일 구간 (많음·유일 + 적음·유일)"))):
        x0, y0, w, h = 60 + col * 370, 40, 280, 170
        g.append(text(x0 - 40, 20, title, weight="bold"))
        XM = lambda i, x0=x0: x0 + 25 + i * 77
        YM = lambda v, y0=y0, h=h: y0 + h - v / 0.45 * h
        panel_axes(g, x0, y0, w, h, [(i, f"{s:g} m") for i, s in enumerate((0.5, 1, 2, 4))], (0, 0.1, 0.2, 0.3, 0.4), XM, YM, "초기 오차 σ", lambda v: f"{v:.1f}")
        for policy in ("fixed6", "k2", "k3"):
            colour, dash, label = styles[policy]
            pts = [(XM(i), YM(r["accepted_wrong"])) for i, r in enumerate(rates[f"{group}|{policy}"])]
            extra = f' stroke-dasharray="{dash}"' if dash else ""
            g.append('<polyline points="' + " ".join(f"{x:.1f},{y:.1f}" for x, y in pts) + f'" fill="none" stroke="{colour}" stroke-width="2"{extra}/>')
            for x, y in pts:
                g.append(cross(x, y, 4, colour, 2) if policy == "fixed6" else (dot(x, y, 4, colour) if policy == "k3" else ring(x, y, 3.6, colour)))
        if col == 0:
            g.append(text(XM(0) + 8, YM(0.40), "고정 ±6 m", size=11.5, fill=RED, weight="bold"))
            g.append(text(XM(0) + 8, YM(0.06), "3σ 범위", size=11.5, fill=BLUE, weight="bold"))
            g.append(text(XM(2) + 10, YM(0.13), "2σ 범위", size=11.5, fill=MUTED, weight="bold"))
        else:
            g.append(text(XM(0), YM(0.05), "세 방식 모두 0에 가깝다", size=11, fill=MUTED))
    g.append(text(14, 130, "사후 검사를 통과한 오정합 비율", "middle", 11, MUTED).replace("<text ", '<text transform="rotate(-90 14 130)" '))
    return svg(760, 255, g)


def fig4():
    flights = data("fig4_flights.json")
    g = []
    for col, (key, title, lo, hi, threshold) in enumerate((("predAlias", "(a) 착각 위험 점수 (기체 1 submap만으로 계산)", 0.5, 1.0, 0.79),
                                                             ("predAlpha", "(b) 퇴화 지표 α", 0.1, 0.5, None))):
        x0, y0, w, h = 60 + col * 370, 44, 280, 170
        g.append(text(x0 - 40, 20, title, weight="bold"))
        XM = lambda v, x0=x0, lo=lo, hi=hi: x0 + (v - lo) / (hi - lo) * 280
        YM = lambda v, y0=y0, h=h: y0 + h - v * h
        panel_axes(g, x0, y0, w, h, [(lo + (hi - lo) * i / 4, f"{lo + (hi - lo) * i / 4:.2f}") for i in range(5)], (0, 0.25, 0.5, 0.75, 1.0), XM, YM,
                   "점수", lambda v: f"{int(v * 100)}%")
        if threshold:
            g.append(f'<line x1="{XM(threshold):.1f}" y1="{y0 - 6}" x2="{XM(threshold):.1f}" y2="{y0 + h}" stroke="{MUTED}" stroke-dasharray="4 4"/>')
            g.append(text(XM(threshold) + 5, y0 - 8, "임계값 0.79", size=10.5, fill=MUTED))
        for f in flights:
            x, y = XM(f[key]), YM(f["usable"] / f["same"])
            colour = RED if f["type"] == "RR" else BLUE
            g.append(dot(x, y, 5.5, colour) if f["selected_correct"] else cross(x, y, 5.5, colour, 2.8))
    g.append(text(14, 130, "쓸 수 있는 제약 비율", "middle", 11, MUTED).replace("<text ", '<text transform="rotate(-90 14 130)" '))
    y = 268
    g.append(dot(70, y - 4, 5, BLUE) + text(80, y, "유일 구간 접근", size=11) + dot(180, y - 4, 5, RED) + text(190, y, "반복 구간 접근", size=11))
    g.append(text(300, y, "●: 채택된 제약이 정답", size=11) + cross(460, y - 4, 5, RED, 2.6) + text(470, y, ": 채택된 제약이 오정합", size=11))
    return svg(760, 280, g)


def fig5():
    case = data("fig5_case.json")
    g = [text(10, 16, "(a) 접근 비행 (월드 a1, 구간 2 = 많음·반복에서 접근)", weight="bold")]
    cyl = [[c[0], c[1]] for c in case["cyl"]]
    X, Y = corridor(g, 30, 30, 4.5, case["zones"], cyl, tint_from_y=-1.5)
    a, b = case["approach_x"]
    g.append(f'<rect x="{X(a):.1f}" y="{Y(15):.1f}" width="{(b - a) * 4.5:.1f}" height="{30 * 4.5:.1f}" fill="none" stroke="{INK}" stroke-dasharray="4 3" stroke-width="1.3"/>')
    g.append(text(X((a + b) / 2), Y(15) + 13, "접근 구간", "middle", 10.5, INK, "bold"))
    g.append(path(case["traj2"], X, Y, ORANGE, 1.8, "5 3"))
    g.append(path(case["traj1"], X, Y, BLUE, 2))
    g.append(text(X(147), Y(7.5) + 4, "기체 2", size=10.5, fill=ORANGE) + text(X(147), Y(-7.5) + 4, "기체 1", size=10.5, fill=BLUE))
    for v in (0, 50, 100, 150):
        g.append(text(X(v), Y(-15) + 15, f"{v} m", "middle", 10.5, MUTED))
    top = 222
    g.append(text(10, top - 6, "(b) 보정에 채택된 제약: 기체 2 submap이 한 묶음 옆 원통에 맞음", weight="bold"))
    cx = (case["d1"][0] + case["d2"][0]) / 2
    ZX, ZY = zoom(g, case, cx - 12, cx + 12, -9.0, 1.5, 20, top + 30, 30)
    shift_arrow(g, case, ZX, top + 22)
    dx, dy = ZX(case["d2"][0]), ZY(case["d2"][1])
    g.append(f'<path d="M{dx:.1f} {dy - 7:.1f}l7 7l-7 7l-7 -7z" fill="#FFFFFF" stroke="{INK}" stroke-width="1.6"/>' + text(dx + 11, dy + 4, "기체 2 위치", size=10.5))
    g.append(f'<line x1="20" y1="{ZY(-7.5):.1f}" x2="740" y2="{ZY(-7.5):.1f}" stroke="{BLUE}" stroke-dasharray="6 5" opacity="0.6"/>' + text(26, ZY(-7.5) - 5, "기체 1 차선", size=10.5, fill=BLUE))
    legend_points(g, 24, top + 30 + 10.5 * 30 + 34)
    return svg(760, top + 30 + 10.5 * 30 + 46, g)


def main():
    for name, draw in (("fig1_world_and_errors", fig1), ("fig2_alias_case", fig2), ("fig3_search_window", fig3),
                       ("fig4_approach_prediction", fig4), ("fig5_approach_case", fig5)):
        open(os.path.join(HERE, name + ".svg"), "w").write(draw())
        print("written", name + ".svg")


if __name__ == "__main__":
    main()
