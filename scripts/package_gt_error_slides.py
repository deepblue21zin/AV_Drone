#!/usr/bin/env python3
"""Package two imagegen designs with the unchanged Streamlit error figure.

No bitmap editing: select each figure row with native PowerPoint cropping.
Reuse the previous meeting deck's document masters and packaging helpers.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import posixpath
from xml.etree import ElementTree as ET
from zipfile import ZIP_DEFLATED, ZipFile

from package_two_region_slides import (
    ROOT, RUN, ANALYSIS, W, H, P, R, GROUP,
    body_shape, contain, picture, rels, xml,
)

OUT = ROOT / "docs/presentation/gt_error_20260912"
TEMPLATE = ROOT / "docs/presentation/two_region_20260912/two_region_results_4slides.pptx"
CT = "http://schemas.openxmlformats.org/package/2006/content-types"
MIME = "application/vnd.openxmlformats-officedocument.presentationml."
ET.register_namespace("p", P)
ET.register_namespace("r", R)
ET.register_namespace("", CT)

NOTES = [
    "Streamlit의 절대 위치·yaw 오차 그림 중 위쪽 위치 오차 두 패널을 원본 그대로 보여준다. "
    "각 시점의 추정 x,y와 GT x,y 사이 2차원 거리이다. 가로축은 각 드론 raw SLAM 누적거리이며 "
    "시간 또는 world x가 아니다. 왼쪽 drone1, 오른쪽 drone2이다. 두 그래프의 축 범위는 원본처럼 "
    "서로 다르므로 실제 눈금을 읽는다. GT를 별도 곡선으로 표시하지 않으며 오차 0이 GT 일치다. "
    "B0와 B1은 동일한 PX4 odometry 입력과 알려진 초기 정렬을 사용한다. B0에는 개별 SLAM의 "
    "map-to-odom 보정이 추가된다. B0에서 위치 오차가 더 커진 사실은 관측됐지만 정합 자체, "
    "좌표 변환, 시간 정렬 또는 센서 설정 중 원인을 이 그래프만으로 확정하지 않는다. "
    "N-double은 B0보다 좋아졌으나 B1보다 절대 위치 RMSE가 크다. 카드의 수치는 종단 오차가 "
    "아니라 GT 지원 keyframe 전체의 RMSE이다. 한 번의 탐색적 개발 비행 결과다.",
    "동일 원본 그림의 아래쪽 yaw 오차 두 패널이다. yaw는 수평면 방향각이다. 추정 yaw-GT yaw를 "
    "[-pi,pi)로 감싼 후 절댓값을 취해 도로 변환한다. 부호 있는 yaw나 드론 간 상대 yaw가 아니다. "
    "양수만 나오며 0도에 가까울수록 정확하다. 위치 RMSE에서는 B1이 유리하지만 yaw RMSE에서는 "
    "N-double이 B0와 B1보다 낮다. N-single-B의 drone1 yaw RMSE는 약 0.412도로 N-double의 "
    "0.444도보다 조금 작으므로 모든 조건에서 두 제약이 최선이라고 주장하지 않는다. "
    "drone2 후반 등 급등 구간은 원본 그대로 남겼고, 원인은 별도 검증이 필요하다. "
    "N 계열 곡선은 비행 후 pose graph 최적화 결과다. 초반부터 낮다고 초반에 실제 통신하거나 "
    "실시간 보정한 것이 아니다. GT는 추론 pose와 지도를 고정한 이후 평가에만 사용했다.",
]


def verify_metrics():
    """Cross-check displayed values against JSON and all six CSV trajectories."""
    metrics = json.loads((ANALYSIS / "metrics.json").read_text())["conditions"]
    displayed = {
        "B0": ((1.493, 1.891), (1.367, 1.952)),
        "B1": ((0.233, 0.258), (1.103, 1.740)),
        "N-double": ((0.305, 0.331), (0.444, 0.641)),
    }
    for name, values in metrics.items():
        with (ANALYSIS / "trajectories" / (name + ".csv")).open() as handle:
            rows = list(csv.DictReader(handle))
        for index, vehicle in enumerate(("drone1", "drone2")):
            selected = [row for row in rows if row["vehicle"] == vehicle]
            pos_sq, yaw_sq = [], []
            for row in selected:
                x, y, yaw = (float(row[key]) for key in ("x", "y", "yaw"))
                gx, gy, gyaw = (float(row[key]) for key in ("gt_x", "gt_y", "gt_yaw"))
                pos_sq.append((x-gx)**2 + (y-gy)**2)
                angle = (yaw-gyaw+math.pi) % (2*math.pi)-math.pi
                yaw_sq.append(math.degrees(angle)**2)
            for kind, samples, which in (("ate_translation_m", pos_sq, 0), ("ate_yaw_deg", yaw_sq, 1)):
                recorded = values["vehicles"][vehicle][kind]["rmse"]
                recomputed = math.sqrt(sum(samples)/len(samples))
                assert math.isclose(recorded, recomputed, abs_tol=1e-9), (name, vehicle, kind)
                if name in displayed:
                    assert round(recorded, 3) == displayed[name][which][index]
    return displayed


def build(output):
    if output.exists():
        raise FileExistsError("Preserve the existing deck; choose a new --output")
    displayed = verify_metrics()
    with ZipFile(TEMPLATE) as archive:
        entries = {name: archive.read(name) for name in archive.namelist()
                   if not name.startswith(("ppt/slides/", "ppt/notesSlides/", "ppt/media/"))}
    presentation = ET.fromstring(entries["ppt/presentation.xml"])
    slide_ids = presentation.find("{%s}sldIdLst" % P)
    slide_ids.clear()
    for i in (1, 2):
        ET.SubElement(slide_ids, "{%s}sldId" % P, {"id": str(255+i), "{%s}id" % R: "rId%d" % (i+2)})
    entries["ppt/presentation.xml"] = ET.tostring(presentation, encoding="utf-8", xml_declaration=True)
    entries["ppt/_rels/presentation.xml.rels"] = rels([
        ("rId1", "slideMaster", "slideMasters/slideMaster1.xml"),
        ("rId2", "notesMaster", "notesMasters/notesMaster1.xml"),
        ("rId3", "slide", "slides/slide1.xml"),
        ("rId4", "slide", "slides/slide2.xml"),
    ]).encode()
    content_types = ET.fromstring(entries["[Content_Types].xml"])
    for entry in list(content_types):
        if entry.attrib.get("PartName", "").startswith(("/ppt/slides/", "/ppt/notesSlides/")):
            content_types.remove(entry)
    graph = ANALYSIS / "overview_errors.png"
    entries["ppt/media/overview_errors.png"] = graph.read_bytes()
    designs = [OUT / "generated/01_absolute_position.png", OUT / "generated/02_absolute_yaw.png"]
    for i, design in enumerate(designs, 1):
        entries["ppt/media/design%d.png" % i] = design.read_bytes()
        # Preserve every original curve, legend and axis in the selected row.
        crop = (0, 0, 0, 0.5) if i == 1 else (0, 0.5, 0, 0)
        shapes = picture(2, "rId2", "Imagegen slide design", (0, 0, W, H))
        shapes += picture(3, "rId4", "Unchanged Streamlit GT error figure, row %d" % i,
                          contain(graph, (40, 181, 1592, 373), crop), crop)
        slide_path = "ppt/slides/slide%d.xml" % i
        notes_path = "ppt/notesSlides/notesSlide%d.xml" % i
        entries[slide_path] = xml("sld", '<p:cSld><p:spTree>%s%s</p:spTree></p:cSld>'
                                  '<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>' % (GROUP, shapes)).encode()
        entries["ppt/slides/_rels/slide%d.xml.rels" % i] = rels([
            ("rId1", "slideLayout", "../slideLayouts/slideLayout1.xml"),
            ("rId2", "image", "../media/design%d.png" % i),
            ("rId3", "notesSlide", "../notesSlides/notesSlide%d.xml" % i),
            ("rId4", "image", "../media/overview_errors.png"),
        ]).encode()
        notes = NOTES[i-1] + "\nSource run: " + RUN + "\nSource figure: " + str(graph.relative_to(ROOT))
        entries[notes_path] = xml("notes", '<p:cSld><p:spTree>%s%s</p:spTree></p:cSld>'
                                  '<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>' % (GROUP, body_shape(notes))).encode()
        entries["ppt/notesSlides/_rels/notesSlide%d.xml.rels" % i] = rels([
            ("rId1", "notesMaster", "../notesMasters/notesMaster1.xml"),
            ("rId2", "slide", "../slides/slide%d.xml" % i),
        ]).encode()
        for path, kind in ((slide_path, "slide+xml"), (notes_path, "notesSlide+xml")):
            ET.SubElement(content_types, "{%s}Override" % CT,
                          {"PartName": "/"+path, "ContentType": MIME+kind})
    entries["[Content_Types].xml"] = ET.tostring(content_types, encoding="utf-8", xml_declaration=True)
    for path, data in entries.items():
        if path.endswith((".xml", ".rels")):
            tree = ET.fromstring(data)
            if path.endswith(".rels"):
                base = "" if path == "_rels/.rels" else str(Path(path).parent.parent)
                for relation in tree:
                    target = posixpath.normpath(posixpath.join(base, relation.attrib["Target"]))
                    assert target in entries, (path, target)
    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, "w", ZIP_DEFLATED) as archive:
        for name, data in entries.items():
            archive.writestr(name, data)
    with ZipFile(output) as archive:
        assert archive.testzip() is None
        assert archive.read("ppt/media/overview_errors.png") == graph.read_bytes()
    print(json.dumps({"output": str(output), "slides": 2, "csv_rmse_verified": True,
                      "original_graph_sha256": hashlib.sha256(graph.read_bytes()).hexdigest(),
                      "displayed_rmse": displayed}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUT / "gt_error_2slides.pptx")
    build(parser.parse_args().output)
