"""Editable presentation metadata. Never renames or deletes experiment data."""

import fcntl
import hashlib
import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path


GROUPS = ("SLAM 보정 본실험", "지도·융합 기준", "SLAM 원인 진단", "경로계획 비교", "준비·점검", "분류 대기")
CATALOG_RELATIVE = Path("experiments/dashboard_catalog.json")


class CatalogError(ValueError):
    pass


class CatalogConflict(CatalogError):
    pass


def catalog_exists(root):
    return (Path(root) / CATALOG_RELATIVE).is_file()


def validate_run_id(run_id):
    if (not isinstance(run_id, str) or not run_id or len(run_id) > 200
            or run_id in (".", "..") or "/" in run_id or "\\" in run_id
            or any(ord(ch) < 32 for ch in run_id)):
        raise CatalogError("올바르지 않은 실험 ID입니다.")
    return run_id


def load_catalog(root):
    path = Path(root) / CATALOG_RELATIVE
    if not path.exists():
        return {"schema_version": 1, "revision": 0, "runs": {}}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or data.get("schema_version") != 1:
            raise ValueError("unsupported schema")
        if not isinstance(data.get("revision"), int) or not isinstance(data.get("runs"), dict):
            raise ValueError("invalid catalog")
        for run_id, entry in data["runs"].items():
            validate_run_id(run_id)
            if not isinstance(entry, dict) or entry.get("visibility", "active") not in ("active", "archive"):
                raise ValueError("invalid entry")
            for field in ("label", "purpose", "group", "notes", "archive_reason"):
                if field in entry and not isinstance(entry[field], str):
                    raise ValueError("invalid text field")
        return data
    except (ValueError, OSError, TypeError) as exc:
        raise CatalogError(f"실험 이름 파일을 읽을 수 없습니다. 원본은 덮어쓰지 않습니다: {path}") from exc


def discover_experiments(root):
    """Bounded scan: no recursion into backups, build trees or rosbag contents."""
    root = Path(root)
    artifacts = root / "artifacts"
    result = {}
    evidence = ("metadata.json", "summary.json", "paper_metrics.json", "rosbag_metrics.json", "trajectory.csv")
    if not artifacts.exists():
        return result
    for path in sorted(artifacts.iterdir()):
        if not path.is_dir() or path.is_symlink():
            continue
        vehicles = [child for child in path.iterdir()
                    if child.is_dir() and not child.is_symlink() and child.name.startswith("drone")
                    and any((child / filename).is_file() for filename in evidence)]
        direct = any((path / filename).is_file() for filename in evidence)
        analyses = [kind for kind in ("lidar_registration", "oracle_correction")
                    if (path / kind / "metrics.json").is_file()
                    and (path / kind / "manifest.json").is_file()]
        if not direct and not vehicles and not analyses:
            continue
        result[path.name] = {"run_id": path.name, "path": path, "analyses": analyses,
                             "vehicles": [p.name for p in vehicles], "modified": path.stat().st_mtime}
    return result


def entry_for(catalog, run_id):
    return {"label": run_id, "purpose": "실험 목적을 입력해 주세요.", "group": "분류 대기",
            "visibility": "active", "notes": "", "archive_reason": "",
            **catalog["runs"].get(run_id, {})}


def display_label(catalog, run_id):
    entry = entry_for(catalog, run_id)
    label = entry["label"].strip() or run_id
    duplicates = sum(1 for other in catalog["runs"].values() if other.get("label", "").strip() == label)
    if duplicates > 1:
        label += " · " + hashlib.sha256(run_id.encode()).hexdigest()[:6]
    return label


def active_ids(root):
    catalog = load_catalog(root)
    return {run_id for run_id in discover_experiments(root)
            if entry_for(catalog, run_id)["visibility"] == "active"}


def _atomic_json(path, data):
    """Replace only our small catalog file; the previous revision is backed up."""
    descriptor, temporary = tempfile.mkstemp(prefix=".catalog-", suffix=".json", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)  # Only this call's own incomplete temporary file.


def save_entries(root, updates, expected_revision):
    """Lock + optimistic revision check avoids losing edits from another browser."""
    root = Path(root)
    path = root / CATALOG_RELATIVE
    path.parent.mkdir(parents=True, exist_ok=True)
    if not updates:
        raise CatalogError("변경할 실험이 없습니다.")
    known = discover_experiments(root)
    clean = {}
    limits = {"label": 120, "purpose": 2000, "notes": 4000, "archive_reason": 1000}
    for run_id, entry in updates.items():
        validate_run_id(run_id)
        if run_id not in known:
            raise CatalogError("실제 데이터가 있는 실험만 편집할 수 있습니다.")
        if not isinstance(entry, dict):
            raise CatalogError("수정 내용이 올바르지 않습니다.")
        entry = dict(entry)
        allowed = set(limits) | {"group", "visibility"}
        if set(entry) - allowed:
            raise CatalogError("편집할 수 없는 필드가 포함됐습니다.")
        for field, limit in limits.items():
            if field in entry:
                if not isinstance(entry[field], str) or len(entry[field]) > limit:
                    raise CatalogError(f"{field}: 최대 {limit}자까지 입력할 수 있습니다.")
                entry[field] = entry[field].strip()
        if "label" in entry and not entry["label"]:
            raise CatalogError("실험 이름은 비워둘 수 없습니다.")
        if "group" in entry and entry["group"] not in GROUPS:
            raise CatalogError("실험 분류를 선택해 주세요.")
        if "visibility" in entry and entry["visibility"] not in ("active", "archive"):
            raise CatalogError("표시 상태가 올바르지 않습니다.")
        clean[run_id] = entry
    with path.with_suffix(".lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        catalog = load_catalog(root)
        if catalog["revision"] != expected_revision:
            raise CatalogConflict("다른 화면에서 이름/보관 상태가 변경됐습니다. 새로고침 후 다시 저장해 주세요.")
        stamp = datetime.now(timezone.utc).isoformat()
        for run_id, changes in clean.items():
            entry = {**entry_for(catalog, run_id), **changes, "updated_at": stamp}
            if entry["visibility"] == "archive" and not entry["archive_reason"]:
                raise CatalogError("보관함으로 옮기는 이유를 입력해 주세요.")
            catalog["runs"][run_id] = entry
        if path.exists():
            history = path.parent / "dashboard_catalog_history"
            history.mkdir(exist_ok=True)
            previous = load_catalog(root)
            # Exclusive creation: no earlier history snapshot is overwritten.
            backup_path = history / f"revision_{previous['revision']:06d}.json"
            try:
                with backup_path.open("x", encoding="utf-8") as stream:
                    json.dump(previous, stream, ensure_ascii=False, indent=2)
                    stream.write("\n")
            except FileExistsError:
                # A prior atomic replacement may have failed after backup.
                # Retry only if that immutable backup is still exactly right.
                if json.loads(backup_path.read_text(encoding="utf-8")) != previous:
                    raise CatalogError("기존 라벨 백업과 현재 revision이 다릅니다. 백업을 덮어쓰지 않습니다.")
        catalog["revision"] += 1
        catalog["updated_at"] = stamp
        _atomic_json(path, catalog)
        return catalog
