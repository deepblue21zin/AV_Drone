#!/usr/bin/env python3
"""Compose an imagegen frame and a CSV-derived four-panel plot as native PPT pictures."""

import json
from pathlib import Path
import posixpath
from xml.etree import ElementTree as ET
from zipfile import ZipFile, ZIP_DEFLATED

from PIL import Image  # Image dimensions only; never edits pixels.
from package_two_region_slides import ROOT, W, P, R, GROUP, body_shape, picture, rels, xml

OUT = ROOT / "docs/presentation/gt_error_no_b1_20260912"
TEMPLATE = ROOT / "docs/presentation/gt_error_20260912/gt_error_2slides.pptx"
CT = "http://schemas.openxmlformats.org/package/2006/content-types"
MIME = "application/vnd.openxmlformats-officedocument.presentationml."
ET.register_namespace("p", P)
ET.register_namespace("r", R)
ET.register_namespace("", CT)


def main():
    output = OUT / "composition.pptx"
    if output.exists():
        raise FileExistsError(output)
    audit = json.loads((OUT / "data/data_audit.json").read_text())
    assert audit["omitted_condition"] == "B1"
    assert audit["minimum_rmse_panels_for_n_double"] == 3
    design, graph = OUT / "imagegen_design.png", OUT / "data/overview_errors_no_b1_highlight.png"
    with Image.open(design) as im:
        dw, dh = im.size
    with Image.open(graph) as im:
        gw, gh = im.size
    height = round(W * dh / dw)
    with ZipFile(TEMPLATE) as archive:
        entries = {name: archive.read(name) for name in archive.namelist()
                   if not name.startswith(("ppt/slides/", "ppt/notesSlides/", "ppt/media/"))}
    presentation = ET.fromstring(entries["ppt/presentation.xml"])
    ids = presentation.find("{%s}sldIdLst" % P)
    ids.clear()
    ET.SubElement(ids, "{%s}sldId" % P, {"id": "256", "{%s}id" % R: "rId3"})
    size = presentation.find("{%s}sldSz" % P)
    size.attrib.clear()
    size.attrib.update({"cx": str(W), "cy": str(height)})
    entries["ppt/presentation.xml"] = ET.tostring(presentation, encoding="utf-8", xml_declaration=True)
    entries["ppt/_rels/presentation.xml.rels"] = rels([
        ("rId1", "slideMaster", "slideMasters/slideMaster1.xml"),
        ("rId2", "notesMaster", "notesMasters/notesMaster1.xml"),
        ("rId3", "slide", "slides/slide1.xml"),
    ]).encode()
    entries["ppt/media/design.png"] = design.read_bytes()
    entries["ppt/media/plot.png"] = graph.read_bytes()
    # Fit the whole plot inside the inspected imagegen design's white opening.
    x, y, pw, ph = 35/1484*W, 160/1060*height, 1414/1484*W, 675/1060*height
    scale = min(pw/gw, ph/gh)
    gw, gh = gw*scale, gh*scale
    plot_box = x+(pw-gw)/2, y+(ph-gh)/2, gw, gh
    shapes = picture(2, "rId2", "Imagegen title and truthful N-double highlight", (0, 0, W, height))
    shapes += picture(3, "rId4", "B1 omitted; unmodified recorded values and original axis limits", plot_box)
    entries["ppt/slides/slide1.xml"] = xml("sld", '<p:cSld><p:spTree>%s%s</p:spTree></p:cSld>'
        '<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>' % (GROUP, shapes)).encode()
    entries["ppt/slides/_rels/slide1.xml.rels"] = rels([
        ("rId1", "slideLayout", "../slideLayouts/slideLayout1.xml"),
        ("rId2", "image", "../media/design.png"),
        ("rId3", "notesSlide", "../notesSlides/notesSlide1.xml"),
        ("rId4", "image", "../media/plot.png"),
    ]).encode()
    notes = ("원본 CSV에서 B1만 표시 제외하고 나머지 5조건의 좌표·오차·축 범위를 유지했다. "
             "N-double은 표시된 조건의 전 구간 RMSE 기준으로 drone1/drone2 위치 및 drone2 yaw에서 최소다. "
             "drone1 yaw는 N-single-B 0.412도가 N-double 0.444도보다 작다. 모든 순간의 최솟값이나 "
             "B1까지 포함한 전체 기준선 우위를 주장하지 않는다. 기존 Streamlit과 기록은 변경하지 않았다. "
             "이미지 스킬은 제목·강조·해석 디자인에 사용했고, 실제 곡선은 AI가 다시 그리지 않았다.")
    entries["ppt/notesSlides/notesSlide1.xml"] = xml("notes", '<p:cSld><p:spTree>%s%s</p:spTree></p:cSld>'
        '<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>' % (GROUP, body_shape(notes))).encode()
    entries["ppt/notesSlides/_rels/notesSlide1.xml.rels"] = rels([
        ("rId1", "notesMaster", "../notesMasters/notesMaster1.xml"),
        ("rId2", "slide", "../slides/slide1.xml"),
    ]).encode()
    content_types = ET.fromstring(entries["[Content_Types].xml"])
    for item in list(content_types):
        if item.attrib.get("PartName", "").startswith(("/ppt/slides/", "/ppt/notesSlides/")):
            content_types.remove(item)
    for path, kind in (("/ppt/slides/slide1.xml", "slide+xml"), ("/ppt/notesSlides/notesSlide1.xml", "notesSlide+xml")):
        ET.SubElement(content_types, "{%s}Override" % CT, {"PartName": path, "ContentType": MIME+kind})
    entries["[Content_Types].xml"] = ET.tostring(content_types, encoding="utf-8", xml_declaration=True)
    for path, data in entries.items():
        if path.endswith((".xml", ".rels")):
            tree = ET.fromstring(data)
            if path.endswith(".rels"):
                base = "" if path == "_rels/.rels" else str(Path(path).parent.parent)
                for item in tree:
                    target = posixpath.normpath(posixpath.join(base, item.attrib["Target"]))
                    assert target in entries, (path, target)
    with ZipFile(output, "w", ZIP_DEFLATED) as archive:
        for name, data in entries.items():
            archive.writestr(name, data)
    with ZipFile(output) as archive:
        assert archive.testzip() is None
        assert archive.read("ppt/media/plot.png") == graph.read_bytes()
    print(output)
    print("Verified one page, all XML relationships, unchanged scientific plot bytes.")


if __name__ == "__main__":
    main()
