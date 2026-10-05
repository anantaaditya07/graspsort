#!/usr/bin/env python3
"""Summarise a GraspSort Phase 6 evaluation run into a Markdown report (architecture 8, D-17).

Usage:
    python3 scripts/eval_summary.py data/eval/<run_id> [--out docs/results.md]
        [--targets success_low_clutter=0.9 --targets plan_p95_s=2.0 ...]

Inputs (schema: Phase 6 contract):
    <run_dir>/trials.jsonl        one JSON object per trial (written by scripts/eval_run.py)
    <run_dir>/node_metrics.jsonl  one JSON line per pick attempt (sort_task_node metrics_log_path)
Missing files or fields are reported as "n/a"; no number is produced without data.

Metric definitions
    pick success rate     objects with "correct": true / objects spawned
    trials fully cleared  trials in which every spawned object is correct
    detected              objects with localization.matched true / objects spawned (snapshot
                          before the goal)
    3D error              localization.err_3d of matched objects, per class, in mm
    bottle yaw error      localization.yaw_err_deg of bottles (non-null), folded into [0, 90]
                          deg (180 deg footprint symmetry; a no-op if already folded)
    planning time         node_metrics planning_time of executed attempts (stage != "reach_check"
                          and planning_time > 0), in s
    cycle time            node_metrics cycle_time of successful attempts, in s
    attempts per object   executed attempts (stage != "reach_check") / objects spawned, over the
                          trials that have a goal_id
    percentiles           linear interpolation between closest ranks (Hyndman-Fan type 7, the
                          numpy default): rank h = (n - 1) * p / 100, value = x[floor h] +
                          (h - floor h) * (x[floor h + 1] - x[floor h]) on the sorted sample

Failure label precedence (exactly one primary label per object that is not correct)
    1. the action failure reason attributed to the object ("<object> (<class>): <category>: ..."),
       label = its category (unparsable strings -> "unparsed")
    2. wrong_bin       final_bin is set (but not the expected bin)
    3. off_table       final z below table_top_z - off_table_drop
    4. not_detected    localization.matched is false
    5. still_on_table  anything else with a final pose
    6. no_final_pose   no final pose recorded
Attribution of a failure reason to a ground-truth (GT) object, among the not-correct objects of
the same class that have no reason yet: (a) the reason's "object_<id>" equals localization.id,
else (b) the GT spawn nearest (xy) to the last node_metrics target_pose logged for that label in
the same goal, else (c) the first such object in trial order. A reason that cannot be attributed
(e.g. no not-correct object of that class is left) is counted separately as "unattributed".
"""

import argparse
import datetime
import json
import math
import os
import re
import subprocess
import sys

CLASSES = ("sports ball", "bottle")
ACTION_CATEGORIES = ("no_ik", "plan_failed", "grasp_slipped", "execution_failed",
                     "misdetection", "unreachable", "no_grasp_candidate", "other")
OUTCOME_CATEGORIES = ("wrong_bin", "off_table", "not_detected", "still_on_table",
                      "no_final_pose")
ALL_CATEGORIES = ACTION_CATEGORIES + ("unparsed",) + OUTCOME_CATEGORIES

# Architecture 8 targets; every value can be overridden with --targets KEY=VALUE.
DEFAULT_TARGETS = {
    "success_low_clutter": 0.90,   # pick success rate >= this on low clutter
    "low_clutter_gap_m": 0.08,     # min_gap that defines "low clutter"
    "err3d_median_mm": 15.0,       # median 3D error < this, per class
    "yaw_median_deg": 10.0,        # bottle yaw error median < this
    "plan_p95_s": 2.0,             # planning time p95 < this
}

GAP_TOL = 1e-6
REASON_RE = re.compile(r"^(?P<object>.+?) \((?P<cls>[^()]*)\): (?P<category>[a-z_]+):\s?"
                       r"(?P<detail>.*)$", re.DOTALL)
LABEL_ID_RE = re.compile(r"(\d+)$")


# ----------------------------------------------------------------------------- statistics

def percentile(values, p):
    """Percentile p (0..100) by linear interpolation (type 7). None for an empty sample."""
    xs = sorted(v for v in values if v is not None)
    if not xs:
        return None
    if not 0.0 <= p <= 100.0:
        raise ValueError("percentile p must be in [0, 100]")
    h = (len(xs) - 1) * p / 100.0
    lo = int(math.floor(h))
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (h - lo) * (xs[hi] - xs[lo])


def median(values):
    return percentile(values, 50.0)


def mean(values):
    xs = [v for v in values if v is not None]
    return sum(xs) / len(xs) if xs else None


def fold_yaw_deg(e):
    """Bottle footprint is symmetric under 180 deg: fold an error into [0, 90] deg."""
    e = abs(e) % 180.0
    return min(e, 180.0 - e)


def rate(k, n):
    """(k, n, k/n) with None as the fraction when n == 0."""
    return (k, n, (k / n) if n else None)


# ----------------------------------------------------------------------------- parsing

def load_jsonl(path):
    """Returns (records, n_bad_lines, exists). Blank lines are ignored."""
    if not os.path.isfile(path):
        return [], 0, False
    records, bad = [], 0
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                bad += 1
                continue
            if isinstance(rec, dict):
                records.append(rec)
            else:
                bad += 1
    return records, bad, True


def parse_failure_reason(s):
    """Parses "<object> (<class>): <category>: <detail>". None if the format does not match."""
    if not isinstance(s, str):
        return None
    m = REASON_RE.match(s.strip())
    if not m:
        return None
    return {"object": m.group("object"), "class": m.group("cls"),
            "category": m.group("category"), "detail": m.group("detail")}


def label_id(label):
    """Trailing integer of a perception label such as "object_12", else None."""
    if not isinstance(label, str):
        return None
    m = LABEL_ID_RE.search(label)
    return int(m.group(1)) if m else None


def config_key(trial):
    cfg = trial.get("config") or {}
    n = cfg.get("n_objects")
    gap = cfg.get("min_gap")
    return (n, round(gap, 4) if isinstance(gap, (int, float)) else gap)


def config_name(key):
    n, gap = key
    gap_s = "%g cm" % (gap * 100.0) if isinstance(gap, (int, float)) else "gap n/a"
    return "%s obj, gap %s" % (n, gap_s)


def group_metrics_by_goal(trials, lines):
    """Returns ({goal_id: [lines]} for goal_ids of trials, number of lines not joined)."""
    goals = {t.get("goal_id") for t in trials if t.get("goal_id")}
    by_goal = {g: [] for g in goals}
    unjoined = 0
    for ln in lines:
        g = ln.get("goal_id")
        if g in by_goal:
            by_goal[g].append(ln)
        else:
            unjoined += 1
    return by_goal, unjoined


def is_executed_attempt(line):
    return line.get("stage") != "reach_check"


# ----------------------------------------------------------------------------- failure labels

def _xy_dist(a, b):
    try:
        return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))
    except (KeyError, TypeError, ValueError):
        return None


def assign_failure_labels(trial, metric_lines=(), table_top_z=0.75, off_table_drop=0.05):
    """Primary failure label for every not-correct object of a trial (see module docstring).

    Returns (labels, unattributed): labels maps object index -> category; unattributed lists
    the failure-reason categories that could not be tied to a not-correct GT object.
    """
    objects = trial.get("objects") or []
    failed = [i for i, o in enumerate(objects) if not o.get("correct")]
    reason_for = {}
    unattributed = []
    last_pose = {}
    for ln in metric_lines:
        if ln.get("object") is not None and isinstance(ln.get("target_pose"), dict):
            last_pose[ln["object"]] = ln["target_pose"]

    reasons = (trial.get("action") or {}).get("failure_reasons") or []
    for raw in reasons:
        parsed = parse_failure_reason(raw)
        if parsed is None:
            # Unparsable: cannot be tied to a class; attribute to nothing, count separately.
            unattributed.append("unparsed")
            continue
        cat = parsed["category"]
        if cat not in ACTION_CATEGORIES:
            cat = "unparsed"
        cands = [i for i in failed if i not in reason_for
                 and objects[i].get("class") == parsed["class"]]
        if not cands:
            unattributed.append(cat)
            continue
        rid = label_id(parsed["object"])
        chosen = None
        if rid is not None:
            for i in cands:
                if ((objects[i].get("localization") or {}).get("id")) == rid:
                    chosen = i
                    break
        if chosen is None and parsed["object"] in last_pose:
            pose = last_pose[parsed["object"]]
            dists = [(_xy_dist(objects[i].get("spawn") or {}, pose), i) for i in cands]
            dists = [d for d in dists if d[0] is not None]
            if dists:
                chosen = min(dists)[1]
        if chosen is None:
            chosen = cands[0]
        reason_for[chosen] = cat

    labels = {}
    for i in failed:
        o = objects[i]
        final = o.get("final")
        if i in reason_for:
            labels[i] = reason_for[i]
        elif o.get("final_bin"):
            labels[i] = "wrong_bin"
        elif isinstance(final, dict) and isinstance(final.get("z"), (int, float)) \
                and final["z"] < table_top_z - off_table_drop:
            labels[i] = "off_table"
        elif not (o.get("localization") or {}).get("matched"):
            labels[i] = "not_detected"
        elif isinstance(final, dict) and final:
            labels[i] = "still_on_table"
        else:
            labels[i] = "no_final_pose"
    return labels, unattributed


# ----------------------------------------------------------------------------- metrics

def compute_metrics(trials, by_goal, table_top_z=0.75, off_table_drop=0.05):
    """Metrics of a group of trials. by_goal: {goal_id: [node metric lines]}."""
    m = {"n_trials": len(trials), "status": {}, "objects": 0, "correct": 0, "cleared": 0,
         "matched": 0, "err3d_mm": {c: [] for c in CLASSES}, "yaw_deg": [],
         "planning_s": [], "cycle_s": [], "attempts": 0, "attempt_objects": 0,
         "attempt_trials": 0, "failures": {}, "unattributed": {}, "attempt_failures": {}}
    for t in trials:
        status = (t.get("action") or {}).get("status") or "n/a"
        m["status"][status] = m["status"].get(status, 0) + 1
        objs = t.get("objects") or []
        m["objects"] += len(objs)
        n_ok = sum(1 for o in objs if o.get("correct"))
        m["correct"] += n_ok
        if objs and n_ok == len(objs):
            m["cleared"] += 1
        for o in objs:
            loc = o.get("localization") or {}
            if loc.get("matched"):
                m["matched"] += 1
                e = loc.get("err_3d")
                if isinstance(e, (int, float)):
                    m["err3d_mm"].setdefault(o.get("class"), []).append(e * 1000.0)
            y = loc.get("yaw_err_deg")
            if o.get("class") == "bottle" and isinstance(y, (int, float)):
                m["yaw_deg"].append(fold_yaw_deg(y))
        lines = by_goal.get(t.get("goal_id"), []) if t.get("goal_id") else []
        if t.get("goal_id"):
            m["attempt_trials"] += 1
            m["attempt_objects"] += len(objs)
            m["attempts"] += sum(1 for ln in lines if is_executed_attempt(ln))
        for ln in lines:
            pt = ln.get("planning_time")
            if is_executed_attempt(ln) and isinstance(pt, (int, float)) and pt > 0:
                m["planning_s"].append(pt)
            ct = ln.get("cycle_time")
            if ln.get("success") is True and isinstance(ct, (int, float)):
                m["cycle_s"].append(ct)
            if ln.get("success") is False:
                c = ln.get("category") or "n/a"
                m["attempt_failures"][c] = m["attempt_failures"].get(c, 0) + 1
        labels, unattr = assign_failure_labels(t, lines, table_top_z, off_table_drop)
        for c in labels.values():
            m["failures"][c] = m["failures"].get(c, 0) + 1
        for c in unattr:
            m["unattributed"][c] = m["unattributed"].get(c, 0) + 1
    return m


def is_low_clutter(key, gap):
    return isinstance(key[1], (int, float)) and abs(key[1] - gap) < GAP_TOL


def evaluate_targets(groups, overall, targets):
    """Rows (name, target text, measured text, verdict). Verdict n/a when data is missing."""
    rows = []
    low = [k for k in groups if is_low_clutter(k, targets["low_clutter_gap_m"])]
    k = sum(groups[g]["correct"] for g in low)
    n = sum(groups[g]["objects"] for g in low)
    frac = rate(k, n)[2]
    rows.append(("Pick success, low clutter (gap %g cm)" % (targets["low_clutter_gap_m"] * 100),
                 ">= %.0f %%" % (targets["success_low_clutter"] * 100),
                 "n/a (n=0)" if frac is None else "%.1f %% (%d/%d)" % (frac * 100, k, n),
                 "n/a" if frac is None else
                 ("PASS" if frac >= targets["success_low_clutter"] else "FAIL")))
    for c in CLASSES:
        xs = overall["err3d_mm"].get(c, [])
        med = median(xs)
        rows.append(("3D error median, %s" % c, "< %g mm" % targets["err3d_median_mm"],
                     "n/a (n=0)" if med is None else "%.1f mm (n=%d)" % (med, len(xs)),
                     "n/a" if med is None else
                     ("PASS" if med < targets["err3d_median_mm"] else "FAIL")))
    med = median(overall["yaw_deg"])
    rows.append(("Bottle yaw error median", "< %g deg" % targets["yaw_median_deg"],
                 "n/a (n=0)" if med is None else
                 "%.1f deg (n=%d)" % (med, len(overall["yaw_deg"])),
                 "n/a" if med is None else
                 ("PASS" if med < targets["yaw_median_deg"] else "FAIL")))
    p95 = percentile(overall["planning_s"], 95)
    rows.append(("Planning time p95", "< %g s" % targets["plan_p95_s"],
                 "n/a (n=0)" if p95 is None else
                 "%.3f s (n=%d)" % (p95, len(overall["planning_s"])),
                 "n/a" if p95 is None else ("PASS" if p95 < targets["plan_p95_s"] else "FAIL")))
    return rows


# ----------------------------------------------------------------------------- rendering

def _fmt(v, digits):
    return "n/a" if v is None else "%.*f" % (digits, v)


def _rate_text(k, n):
    frac = rate(k, n)[2]
    return "n/a (0/0)" if frac is None else "%.1f %% (%d/%d)" % (frac * 100, k, n)


def _dist_text(xs, digits, with_mean=False):
    if not xs:
        return "n/a (n=0)"
    parts = [_fmt(mean(xs), digits)] if with_mean else []
    parts += [_fmt(median(xs), digits), _fmt(percentile(xs, 95), digits)]
    return "%s (n=%d)" % (" / ".join(parts), len(xs))


def _table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return out


def render_report(info, groups, overall, targets, table_top_z, off_table_drop):
    keys = sorted(groups, key=lambda k: (k[0] if isinstance(k[0], int) else 1 << 30,
                                         -(k[1] if isinstance(k[1], (int, float)) else 0)))
    cols = [(config_name(k), groups[k]) for k in keys] + [("overall", overall)]
    L = ["# GraspSort evaluation results", ""]
    L += ["- Generated: %s" % info["date"],
          "- Git commit: %s" % info["commit"],
          "- run_id: %s" % (", ".join(info["run_ids"]) or "n/a"),
          "- Scenario: D-17 (n_objects 2 or 4 x minimum footprint gap 8 or 3 cm, 10 trials per "
          "config planned; 2 objects = 1 ball + 1 bottle, 4 = 2 + 2; random positions and bottle "
          "yaw, fixed seeds)",
          "- Data: %d trials, %d objects, %d node metric lines (%d not joined to a trial)"
          % (overall["n_trials"], overall["objects"], info["n_lines"], info["unjoined"])]
    for w in info["warnings"]:
        L.append("- Warning: %s" % w)
    L += ["", "## Results per configuration", ""]
    rows = []

    def row(name, fn):
        rows.append([name] + [fn(g) for _, g in cols])

    row("Trials", lambda g: str(g["n_trials"]))
    row("Action status", lambda g: ", ".join("%s %d" % kv for kv in sorted(g["status"].items()))
        or "n/a")
    row("Objects spawned", lambda g: str(g["objects"]))
    row("Pick success rate", lambda g: _rate_text(g["correct"], g["objects"]))
    row("Trials fully cleared", lambda g: _rate_text(g["cleared"], g["n_trials"]))
    row("Detected at snapshot", lambda g: _rate_text(g["matched"], g["objects"]))
    for c in CLASSES:
        row("3D error %s median / p95 (mm)" % c,
            lambda g, c=c: _dist_text(g["err3d_mm"].get(c, []), 1))
    row("Bottle yaw error median / p95 (deg)", lambda g: _dist_text(g["yaw_deg"], 1))
    row("Planning time p50 / p95 (s)", lambda g: _dist_text(g["planning_s"], 3))
    row("Cycle time mean / p50 / p95 (s)", lambda g: _dist_text(g["cycle_s"], 2, True))
    row("Attempts per object", lambda g: "n/a (0 objects)" if not g["attempt_objects"] else
        "%.2f (%d/%d, %d trials)" % (g["attempts"] / g["attempt_objects"], g["attempts"],
                                     g["attempt_objects"], g["attempt_trials"]))
    L += _table(["Metric"] + [c for c, _ in cols], rows)

    L += ["", "## Targets (architecture 8)", ""]
    L += _table(["Target", "Required", "Measured", "Result"],
                evaluate_targets(groups, overall, targets))

    L += ["", "## Failure breakdown (one primary label per object not in its correct bin)", ""]
    frows = []
    for c in ALL_CATEGORIES:
        counts = [g["failures"].get(c, 0) for _, g in cols]
        if any(counts):
            frows.append([c] + [str(x) for x in counts])
    extra = sorted({c for _, g in cols for c in g["failures"]} - set(ALL_CATEGORIES))
    for c in extra:
        frows.append([c] + [str(g["failures"].get(c, 0)) for _, g in cols])
    frows.append(["**total not correct**"] + [str(g["objects"] - g["correct"]) for _, g in cols])
    L += _table(["Category"] + [c for c, _ in cols], frows)
    un = overall["unattributed"]
    L += ["", "Action failure reasons not attributable to a not-correct object: %s."
          % (", ".join("%s %d" % kv for kv in sorted(un.items())) if un else "none")]
    L += ["", "Failed attempts by category (node metrics, per attempt incl. retries and "
          "reach-check give-ups):", ""]
    arows = [[c] + [str(g["attempt_failures"].get(c, 0)) for _, g in cols]
             for c in sorted({c for _, g in cols for c in g["attempt_failures"]})]
    L += _table(["Category"] + [c for c, _ in cols], arows) if arows else ["none"]
    L += ["", "Label precedence: action failure reason attributed to the object, else wrong_bin, "
          "off_table (final z < %g m), not_detected, still_on_table, no_final_pose."
          % (table_top_z - off_table_drop)]

    L += ["", "## Notes / limits", "",
          "- Simulation only (Gazebo Classic); ground truth from Gazebo is used only by the "
          "evaluation scripts.",
          "- One fixed oblique RGB-D camera (D-05), no hand-eye loop.",
          "- Two object classes, sports ball and bottle (D-13); 2 and 4 objects instead of the "
          "PDF's 3/5/7, gaps 8 / 3 cm instead of 8 / 2 cm (D-17).",
          "- 3D and yaw errors are from one snapshot before each goal (nearest same-class "
          "estimate within 0.10 m); unmatched objects count as not detected and have no error.",
          "- Planning time is the planning_time field of sort_task_node per executed attempt "
          "(reach-check give-ups excluded); cycle time covers successful attempts only.",
          "- Percentiles use linear interpolation (type 7); n is shown for every value.", ""]
    return "\n".join(L)


# ----------------------------------------------------------------------------- main

def git_commit(repo_dir):
    try:
        sha = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=repo_dir,
                             capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"],
                               cwd=repo_dir, capture_output=True, text=True,
                               check=True).stdout.strip()
        return sha + (" (tracked files modified)" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "n/a"


def parse_targets(items):
    targets = dict(DEFAULT_TARGETS)
    for item in items or []:
        for part in item.split(","):
            if not part.strip():
                continue
            key, sep, val = part.partition("=")
            key = key.strip()
            if not sep or key not in targets:
                raise ValueError("bad target %r; keys: %s" % (part, ", ".join(sorted(targets))))
            targets[key] = float(val)
    return targets


def summarise(run_dir, targets, table_top_z, off_table_drop, date=None, commit=None):
    trials, bad_t, has_t = load_jsonl(os.path.join(run_dir, "trials.jsonl"))
    lines, bad_m, has_m = load_jsonl(os.path.join(run_dir, "node_metrics.jsonl"))
    warnings = []
    if not has_t:
        warnings.append("trials.jsonl missing")
    if not has_m:
        warnings.append("node_metrics.jsonl missing")
    if bad_t or bad_m:
        warnings.append("unparsable JSON lines: trials %d, node_metrics %d" % (bad_t, bad_m))
    no_goal = sum(1 for t in trials if not t.get("goal_id"))
    if no_goal:
        warnings.append("%d trials without goal_id (excluded from attempts per object)" % no_goal)
    by_goal, unjoined = group_metrics_by_goal(trials, lines)
    by_cfg = {}
    for t in trials:
        by_cfg.setdefault(config_key(t), []).append(t)
    groups = {k: compute_metrics(v, by_goal, table_top_z, off_table_drop)
              for k, v in by_cfg.items()}
    overall = compute_metrics(trials, by_goal, table_top_z, off_table_drop)
    info = {"date": date or datetime.date.today().isoformat(),
            "commit": commit if commit is not None else
            git_commit(os.path.dirname(os.path.abspath(__file__))),
            "run_ids": sorted({str(t.get("run_id")) for t in trials if t.get("run_id")}),
            "n_lines": len(lines), "unjoined": unjoined, "warnings": warnings}
    return render_report(info, groups, overall, targets, table_top_z, off_table_drop)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir", help="directory with trials.jsonl and node_metrics.jsonl")
    ap.add_argument("--out", default="docs/results.md", help="Markdown output path")
    ap.add_argument("--targets", action="append", default=[],
                    help="KEY=VALUE[,KEY=VALUE] overrides of %s" % ", ".join(DEFAULT_TARGETS))
    ap.add_argument("--table-top-z", type=float, default=0.75,
                    help="table top height in world (m), world_layout.yaml table.z")
    ap.add_argument("--off-table-drop", type=float, default=0.05,
                    help="final z below table_top_z minus this (m) counts as off_table")
    args = ap.parse_args(argv)
    try:
        targets = parse_targets(args.targets)
    except ValueError as e:
        ap.error(str(e))
    report = summarise(args.run_dir, targets, args.table_top_z, args.off_table_drop)
    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(report)
    print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
