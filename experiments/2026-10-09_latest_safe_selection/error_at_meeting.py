"""Post-hoc: relative position error right after the meeting, for all 34 approach flights.

  python3 error_at_meeting.py        # standard library; reads curves.json of both selection experiments

The terminal error mixes the constraint's own error with the drift accumulated after the meeting.
Here the corrected relative error is read (a) at the drone2 keyframe closest to the meeting and
(b) averaged over the 30 m after it. Defined on 2026-10-09 after the pre-registered results were known.
"""
import json
import os
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCES = [("a", os.path.join(HERE, "../2026-10-09_zone_selection")), ("b", HERE)]
AFTER_M = 30.0


def rows():
    out = []
    for series, folder in SOURCES:
        curves = json.load(open(os.path.join(folder, "curves.json")))
        for run, c in curves.items():
            if not c["meeting"]:
                out.append({"world": c["world"], "zone": c["approach_zone_index"] + 1, "type": c["zone_type"], "state": "none"})
                continue
            m = c["meeting"][0]
            x = m["drone2"][0]
            pts = c["points"]
            at = min(pts, key=lambda p: abs(p["x"] - x))
            after = [p for p in pts if x <= p["x"] <= x + AFTER_M]
            out.append({"world": c["world"], "zone": c["approach_zone_index"] + 1, "type": c["zone_type"],
                        "state": "correct" if m["correct"] else "alias", "meeting_x": x, "constraint_err_m": m["err_m"],
                        "before_m": at["b0"], "at_meeting_m": at["corrected"],
                        "after30_mean_m": sum(p["corrected"] for p in after) / len(after),
                        "after30_before_mean_m": sum(p["b0"] for p in after) / len(after),
                        "terminal_m": pts[-1]["corrected"]})
    return out


def summary(group):
    keys = ("before_m", "at_meeting_m", "after30_before_mean_m", "after30_mean_m", "terminal_m")
    return {"n": len(group), **{k: round(st.median(r[k] for r in group), 2) for k in keys},
            "at_meeting_range_m": [round(min(r["at_meeting_m"] for r in group), 2), round(max(r["at_meeting_m"] for r in group), 2)],
            "worse_than_before": sum(r["at_meeting_m"] > r["before_m"] for r in group)}


def main():
    data = rows()
    with_c = [r for r in data if r["state"] != "none"]
    res = {"flights": len(data), "no_constraint": [f'{r["world"]} z{r["zone"]}' for r in data if r["state"] == "none"],
           "by_constraint": {s: summary([r for r in with_c if r["state"] == s]) for s in ("correct", "alias")},
           "by_zone_type": {t: summary([r for r in with_c if r["type"] == t]) for t in ("RU", "RR")}, "rows": data}
    json.dump(res, open(os.path.join(HERE, "error_at_meeting.json"), "w"), indent=1)
    for k in ("by_constraint", "by_zone_type"):
        for name, s in res[k].items():
            print(k, name, s)
    print("no constraint:", res["no_constraint"])
    for r in sorted(with_c, key=lambda r: (r["state"], r["world"], r["zone"])):
        print(r["world"], "z%d" % r["zone"], r["type"], r["state"], "x %.0f" % r["meeting_x"], "constraint err %.2f" % r["constraint_err_m"],
              "| before %.2f -> at meeting %.2f | next 30 m %.2f -> %.2f | terminal %.1f" % (r["before_m"], r["at_meeting_m"], r["after30_before_mean_m"], r["after30_mean_m"], r["terminal_m"]))


if __name__ == "__main__":
    main()
