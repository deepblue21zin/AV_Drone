#!/usr/bin/env python3
"""Package imagegen slide designs and unchanged experiment plots in PowerPoint.

No bitmap is edited or redrawn: measured plots are native picture objects with
DrawingML cropping. LibreOffice/PowerPoint renders the final document to PNG.
"""

import argparse
import hashlib
import json
from pathlib import Path
import posixpath
from xml.etree import ElementTree as ET
from xml.sax.saxutils import escape
from zipfile import ZipFile, ZIP_DEFLATED

from PIL import Image  # Dimensions only; no bitmap manipulation.


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/presentation/two_region_20260912"
RUN = "20260911T161139Z_9abd773a46f1"
ANALYSIS = ROOT / "artifacts" / RUN / "lidar_registration"
W, H = 12192000, 6858000
P = "http://schemas.openxmlformats.org/presentationml/2006/main"
A = "http://schemas.openxmlformats.org/drawingml/2006/main"
R = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
PKG = "http://schemas.openxmlformats.org/package/2006/relationships"
HEADER = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
GROUP = ('<p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>'
         '<p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/>'
         '<a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>')
CMAP = ('<p:clrMap accent1="accent1" accent2="accent2" accent3="accent3" '
        'accent4="accent4" accent5="accent5" accent6="accent6" bg1="lt1" bg2="lt2" '
        'folHlink="folHlink" hlink="hlink" tx1="dk1" tx2="dk2"/>')

NOTES = [
    "기존 2구간 경로에서는 실제 정합 제약이 하나뿐이었다. 이번에는 후반 공통 통로를 확보해 "
    "두 독립 제약을 얻었다. 동시 만남이 아니라 A 55.7초, B 99.9초 차이의 공통 관측이다. "
    "원본 궤적의 GT는 사후 시각화 전용이다. 고정 경로, 초기 정렬 기지, 오프라인 처리를 가정한다.",
    "같은 bag, 같은 raw pose와 graph 가중치에서 A만 1개, B만 1개, A+B 2개를 적용했다. "
    "형상 및 raw SLAM 공간 prior로 후보를 검색하고 이웃 일관성으로 검증한다. GT는 추론과 지도 "
    "저장이 끝난 다음 평가에만 읽는다. B0는 raw SLAM, B1은 외부 odometry, N-prior는 시작 prior만의 대조군이다. "
    "도식은 설명용 개념도이며 실제 관측 그래프가 아니다.",
    "B0 대비 N-double은 종단 상대 위치 오차를 94.9%, 지도 Chamfer를 70.4% 줄였다. "
    "다만 B만의 종단 오차 0.281m는 두 구간의 0.332m보다 작다. 모든 지표에서 두 구간이 "
    "우수하다는 뜻은 아니다. B1의 절대 위치와 지도 정확도는 여전히 더 좋다. "
    "종단 상대 pose는 각 기체 마지막 GT 지원 keyframe 사이 변환이고 동시각 거리가 아니다.",
    "위 그림 B0와 아래 N-double은 원본 지도 오차 패널이다. 초록은 GT에만 점유, 빨강은 추정에만 점유, "
    "검정은 점유 일치, 회색은 둘 다 미관측이다. 상대 RMSE와 Chamfer가 개선됐지만 A만의 지도 F1은 "
    "0.557로 두 구간의 0.547보다 조금 높다. 한 번의 탐색적 개발 비행이므로 코드와 설정을 고정한 "
    "반복 검증 후 겹침 위치·품질·제약 개수와 공유 비용의 관계로 확장한다. GT-reference는 동일 LiDAR를 "
    "GT pose로 투영한 지도이며 공통 관측 영역에서 지도 지표를 계산한다.",
]


def xml(root, body, attrs=""):
    return f'{HEADER}<p:{root} xmlns:p="{P}" xmlns:a="{A}" xmlns:r="{R}" {attrs}>{body}</p:{root}>'


def rels(items):
    return (f'{HEADER}<Relationships xmlns="{PKG}">' + ''.join(
        f'<Relationship Id="{rid}" Type="{R}/{kind}" Target="{target}"/>'
        for rid, kind, target in items) + '</Relationships>')


def picture(number, rid, name, box, crop=(0, 0, 0, 0)):
    x, y, width, height = [round(v) for v in box]
    left, top, right, bottom = [round(v * 100000) for v in crop]
    return (
        f'<p:pic><p:nvPicPr><p:cNvPr id="{number}" name="{escape(name)}" descr="{escape(name)}"/>'
        '<p:cNvPicPr><a:picLocks noChangeAspect="1"/></p:cNvPicPr><p:nvPr/></p:nvPicPr>'
        f'<p:blipFill><a:blip r:embed="{rid}"/><a:srcRect l="{left}" t="{top}" r="{right}" b="{bottom}"/>'
        '<a:stretch><a:fillRect/></a:stretch></p:blipFill><p:spPr>'
        f'<a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{width}" cy="{height}"/></a:xfrm>'
        '<a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr></p:pic>'
    )


def contain(path, panel, crop=(0, 0, 0, 0)):
    """Fit a native picture inside a panel measured on the 1672x941 AI design."""
    with Image.open(path) as im:
        iw, ih = im.size
    left, top, right, bottom = crop
    iw *= 1-left-right
    ih *= 1-top-bottom
    x, y, pw, ph = panel
    x, pw = x * W / 1672, pw * W / 1672
    y, ph = y * H / 941, ph * H / 941
    ratio = min(pw / iw, ph / ih)
    width, height = iw * ratio, ih * ratio
    return (x+(pw-width)/2, y+(ph-height)/2, width, height)


def body_shape(text):
    paragraphs = ''.join(f'<a:p><a:r><a:rPr lang="ko-KR"/><a:t>{escape(line)}</a:t></a:r></a:p>'
                         for line in text.split('\n'))
    return ('<p:sp><p:nvSpPr><p:cNvPr id="2" name="Speaker notes"/><p:cNvSpPr/>'
            '<p:nvPr><p:ph type="body" idx="1"/></p:nvPr></p:nvSpPr><p:spPr/>'
            f'<p:txBody><a:bodyPr/><a:lstStyle/>{paragraphs}</p:txBody></p:sp>')


def build(output, slide4):
    names = ["slide_01_setup.png", "slide_02_method.png", "slide_03_results.png", slide4]
    designs = [OUT / "generated" / name for name in names]
    for path in designs:
        if not path.is_file():
            raise FileNotFoundError(path)
    if output.exists():
        raise FileExistsError(f"Preserve existing deck; choose another --output: {output}")
    status = json.loads((ANALYSIS / "ablation_status.json").read_text())
    assert status["complete"] and status["actual_applied_counts"]["N-double"] == 2
    evidence = {"trajectory.png": ANALYSIS / "overview_shared_regions.png",
                "map_errors.png": ANALYSIS / "overview_map_errors.png"}
    entries = {}
    overrides = []
    def put(path, data, content_type=None):
        entries[path] = data.encode() if isinstance(data, str) else data
        if content_type:
            overrides.append((path, content_type))
    mime = "application/vnd.openxmlformats-officedocument.presentationml."
    theme_path = "ppt/theme/theme1.xml"
    with ZipFile(ROOT / "자율주행군집드론_0828미팅1차.pptx") as template:
        put(theme_path, template.read(theme_path), "application/vnd.openxmlformats-officedocument.theme+xml")
    put("_rels/.rels", rels([("rId1", "officeDocument", "ppt/presentation.xml")]))
    ids = ''.join(f'<p:sldId id="{255+i}" r:id="rId{i+2}"/>' for i in range(1, 5))
    put("ppt/presentation.xml", xml("presentation",
        '<p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId1"/></p:sldMasterIdLst>'
        '<p:notesMasterIdLst><p:notesMasterId r:id="rId2"/></p:notesMasterIdLst>'
        f'<p:sldIdLst>{ids}</p:sldIdLst><p:sldSz cx="{W}" cy="{H}" type="screen16x9"/>'
        '<p:notesSz cx="6858000" cy="9144000"/>'), mime + "presentation.main+xml")
    put("ppt/_rels/presentation.xml.rels", rels([
        ("rId1", "slideMaster", "slideMasters/slideMaster1.xml"),
        ("rId2", "notesMaster", "notesMasters/notesMaster1.xml"),
        *[(f"rId{i+2}", "slide", f"slides/slide{i}.xml") for i in range(1, 5)]]))
    put("ppt/slideMasters/slideMaster1.xml", xml("sldMaster",
        f'<p:cSld><p:spTree>{GROUP}</p:spTree></p:cSld>{CMAP}'
        '<p:sldLayoutIdLst><p:sldLayoutId id="2147483649" r:id="rId1"/></p:sldLayoutIdLst>'
        '<p:txStyles><p:titleStyle/><p:bodyStyle/><p:otherStyle/></p:txStyles>'), mime + "slideMaster+xml")
    put("ppt/slideMasters/_rels/slideMaster1.xml.rels", rels([
        ("rId1", "slideLayout", "../slideLayouts/slideLayout1.xml"),
        ("rId2", "theme", "../theme/theme1.xml")]))
    put("ppt/slideLayouts/slideLayout1.xml", xml("sldLayout",
        f'<p:cSld name="Blank"><p:spTree>{GROUP}</p:spTree></p:cSld>'
        '<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>', 'type="blank" preserve="1"'), mime + "slideLayout+xml")
    put("ppt/slideLayouts/_rels/slideLayout1.xml.rels", rels([
        ("rId1", "slideMaster", "../slideMasters/slideMaster1.xml")]))
    put("ppt/notesMasters/notesMaster1.xml", xml("notesMaster",
        f'<p:cSld><p:spTree>{GROUP}</p:spTree></p:cSld>{CMAP}'), mime + "notesMaster+xml")
    put("ppt/notesMasters/_rels/notesMaster1.xml.rels", rels([
        ("rId1", "theme", "../theme/theme1.xml")]))
    for name, path in evidence.items():
        put("ppt/media/" + name, path.read_bytes())
    for i, design in enumerate(designs, 1):
        put(f"ppt/media/design{i}.png", design.read_bytes())
        shapes = picture(2, "rId2", f"Slide {i}: imagegen design", (0, 0, W, H))
        relations = [("rId1", "slideLayout", "../slideLayouts/slideLayout1.xml"),
                     ("rId2", "image", f"../media/design{i}.png"),
                     ("rId3", "notesSlide", f"../notesSlides/notesSlide{i}.xml")]
        if i == 1:
            path = evidence["trajectory.png"]
            shapes += picture(3, "rId4", "Unchanged recorded trajectories and GT-only overlap audit",
                              contain(path, (57, 267, 1056, 464)))
            relations.append(("rId4", "image", "../media/trajectory.png"))
        if i == 4:
            path = evidence["map_errors.png"]
            # Equal-height plot areas from first and sixth panels. Their
            # English titles are redundant with the Korean slide labels.
            # Preserve all axes, ticks and data; only native PowerPoint crop.
            for number, crop, panel, name in [
                (3, (0, 51/2880, 0, 1-475/2880), (69, 247, 1008, 167), "B0: original first map panel"),
                (4, (0, 2404/2880, 0, 1-2828/2880), (69, 498, 1008, 167), "N-double: original sixth map panel"),
            ]:
                shapes += picture(number, "rId4", name, contain(path, panel, crop), crop)
            relations.append(("rId4", "image", "../media/map_errors.png"))
        put(f"ppt/slides/slide{i}.xml", xml("sld",
            f'<p:cSld name="Slide {i}"><p:spTree>{GROUP}{shapes}</p:spTree></p:cSld>'
            '<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>'), mime + "slide+xml")
        put(f"ppt/slides/_rels/slide{i}.xml.rels", rels(relations))
        source_note = f"\nSource run: {RUN}\nData: {ANALYSIS.relative_to(ROOT)}/metrics.json\nFull notes: docs/presentation/two_region_20260912/README.md"
        put(f"ppt/notesSlides/notesSlide{i}.xml", xml("notes",
            f'<p:cSld><p:spTree>{GROUP}{body_shape(NOTES[i-1]+source_note)}</p:spTree></p:cSld>'
            '<p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>'), mime + "notesSlide+xml")
        put(f"ppt/notesSlides/_rels/notesSlide{i}.xml.rels", rels([
            ("rId1", "notesMaster", "../notesMasters/notesMaster1.xml"),
            ("rId2", "slide", f"../slides/slide{i}.xml")]))
    put("[Content_Types].xml", HEADER +
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>'
        '<Default Extension="png" ContentType="image/png"/>' + ''.join(
            f'<Override PartName="/{path}" ContentType="{kind}"/>' for path, kind in overrides) + '</Types>')
    # Verify XML and all internal relationships before writing a new package.
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
        for path, data in entries.items():
            archive.writestr(path, data)
    with ZipFile(output) as archive:
        assert archive.testzip() is None
        for name, source in evidence.items():
            assert archive.read("ppt/media/"+name) == source.read_bytes()
    print(output)
    print("4 slides, valid XML and relationships, unchanged measured image bytes verified.")
    print(json.dumps({name: hashlib.sha256(path.read_bytes()).hexdigest()
                      for name, path in evidence.items()}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUT / "two_region_results_4slides.pptx")
    parser.add_argument("--slide4", default="slide_04_map_and_next_v2.png")
    args = parser.parse_args()
    build(args.output, args.slide4)
