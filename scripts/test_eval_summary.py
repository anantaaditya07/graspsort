"""Unit tests for scripts/eval_summary.py on synthetic data. Run: python3 -m pytest scripts -q"""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_summary as es  # noqa: E402


def obj(name, cls, correct, matched=True, loc_id=None, err=0.01, yaw=None, final_bin=None,
        final_z=0.75, spawn=(0.5, 0.0)):
    return {"name": name, "class": cls, "model": "m",
            "spawn": {"x": spawn[0], "y": spawn[1], "yaw": 0.0},
            "localization": {"matched": matched, "id": loc_id, "err_3d": err if matched else None,
                             "err_xy": None, "err_z": None, "yaw_err_deg": yaw},
            "final": {"x": 0.5, "y": 0.0, "z": final_z},
            "expected_bin": "bin_ball" if cls == "sports ball" else "bin_bottle",
            "final_bin": final_bin, "correct": correct}


def trial(objects, n=2, gap=0.08, reasons=(), goal="g1", status="SUCCEEDED", idx=0):
    return {"run_id": "r1", "config": {"n_objects": n, "min_gap": gap}, "trial": idx,
            "seed": 2026 + idx, "objects": objects,
            "action": {"status": status, "picked": 0, "failed": len(reasons),
                       "failure_reasons": list(reasons), "duration": 1.0},
            "goal_id": goal, "notes": []}


def line(goal, label, success, cat="none", stage="", plan=0.1, cycle=12.0, pose=(0.5, 0.0),
         attempt=1):
    return {"stamp": 1.0, "goal_id": goal, "object": label, "class": "bottle",
            "attempt": attempt, "success": success, "category": cat, "stage": stage,
            "stage_times": {}, "planning_time": plan, "cycle_time": cycle,
            "target_pose": {"x": pose[0], "y": pose[1], "z": 0.8, "yaw": 0.0}}


# ---------------------------------------------------------------- statistics

def test_percentile_linear_interpolation_matches_type7():
    xs = [1, 2, 3, 4]
    assert es.percentile(xs, 0) == 1
    assert es.percentile(xs, 100) == 4
    assert es.percentile(xs, 50) == pytest.approx(2.5)
    assert es.percentile(xs, 95) == pytest.approx(3.85)  # h = 2.85
    assert es.percentile([4, 1, 3, 2], 25) == pytest.approx(1.75)  # unsorted input
    assert es.percentile([7.0], 95) == 7.0


def test_percentile_matches_numpy_when_available():
    np = pytest.importorskip("numpy")
    xs = [0.03, 0.2, 0.07, 1.9, 0.5, 0.11, 2.4]
    for p in (5, 50, 90, 95, 99):
        assert es.percentile(xs, p) == pytest.approx(float(np.percentile(xs, p)))


def test_percentile_empty_and_invalid():
    assert es.percentile([], 50) is None
    assert es.median([None]) is None
    assert es.mean([]) is None
    with pytest.raises(ValueError):
        es.percentile([1], 101)


def test_rate_and_yaw_fold():
    assert es.rate(9, 10) == (9, 10, 0.9)
    assert es.rate(0, 0) == (0, 0, None)
    assert es.fold_yaw_deg(175.0) == pytest.approx(5.0)
    assert es.fold_yaw_deg(-8.0) == pytest.approx(8.0)
    assert es.fold_yaw_deg(90.0) == pytest.approx(90.0)


# ---------------------------------------------------------------- parsing

def test_parse_failure_reason():
    r = es.parse_failure_reason("object_6 (sports ball): plan_failed: pregrasp: no plan: x")
    assert r == {"object": "object_6", "class": "sports ball", "category": "plan_failed",
                 "detail": "pregrasp: no plan: x"}
    assert es.parse_failure_reason("garbage") is None
    assert es.parse_failure_reason(None) is None
    assert es.label_id("object_12") == 12
    assert es.label_id("ball") is None


def test_load_jsonl_missing_and_bad_lines(tmp_path):
    assert es.load_jsonl(str(tmp_path / "nope.jsonl")) == ([], 0, False)
    p = tmp_path / "x.jsonl"
    p.write_text('{"a": 1}\n\nnot json\n[1]\n')
    recs, bad, exists = es.load_jsonl(str(p))
    assert recs == [{"a": 1}] and bad == 2 and exists


# ---------------------------------------------------------------- failure labels

def test_success_rate_and_cleared():
    t1 = trial([obj("b1", "sports ball", True), obj("o1", "bottle", True)], goal="g1")
    t2 = trial([obj("b2", "sports ball", True), obj("o2", "bottle", False)], goal="g2", idx=1)
    m = es.compute_metrics([t1, t2], {})
    assert (m["correct"], m["objects"], m["cleared"], m["n_trials"]) == (3, 4, 1, 2)


def test_label_precedence_reason_beats_outcome_no_double_count():
    # Bottle not detected, in the wrong bin, with an action reason: exactly one label (reason).
    o = obj("o1", "bottle", False, matched=False, loc_id=None, final_bin="bin_ball")
    t = trial([o], reasons=["object_3 (bottle): grasp_slipped: lift"])
    labels, un = es.assign_failure_labels(t)
    assert labels == {0: "grasp_slipped"} and un == []


def test_label_precedence_outcomes():
    objs = [obj("a", "bottle", False, final_bin="bin_ball"),            # wrong bin
            obj("b", "bottle", False, matched=False, final_z=0.05),     # fell: off_table
            obj("c", "sports ball", False, matched=False),              # not detected
            obj("d", "sports ball", False),                             # still on table
            obj("e", "sports ball", True)]
    objs[3]["final"] = {"x": 0.5, "y": 0.0, "z": 0.75}
    labels, _ = es.assign_failure_labels(trial(objs))
    assert labels == {0: "wrong_bin", 1: "off_table", 2: "not_detected", 3: "still_on_table"}
    objs[3]["final"] = None
    assert es.assign_failure_labels(trial(objs))[0][3] == "no_final_pose"


def test_reason_attribution_by_id_pose_and_class():
    objs = [obj("o1", "bottle", False, loc_id=10, spawn=(0.4, 0.2)),
            obj("o2", "bottle", False, loc_id=11, spawn=(0.6, -0.2)),
            obj("b1", "sports ball", False, loc_id=12)]
    # id match -> o2; pose match (label 99 last seen near o1) -> o1; ball: no candidate ball
    # reason left -> b1 by class; extra bottle reason -> unattributed.
    reasons = ["object_11 (bottle): no_ik: a", "object_99 (bottle): plan_failed: b",
               "object_50 (sports ball): unreachable: c", "object_7 (bottle): other: d",
               "not a reason"]
    lines = [line("g1", "object_99", False, "plan_failed", pose=(0.41, 0.19))]
    labels, un = es.assign_failure_labels(trial(objs, reasons=reasons), lines)
    assert labels == {0: "plan_failed", 1: "no_ik", 2: "unreachable"}
    assert sorted(un) == ["other", "unparsed"]


def test_failure_total_equals_not_correct_objects():
    objs = [obj("o1", "bottle", False, matched=False, final_bin="bin_ball"),
            obj("b1", "sports ball", False, matched=False), obj("b2", "sports ball", True)]
    t = trial(objs, reasons=["object_1 (bottle): execution_failed: x"])
    m = es.compute_metrics([t], {})
    assert sum(m["failures"].values()) == m["objects"] - m["correct"] == 2
    assert m["failures"] == {"execution_failed": 1, "not_detected": 1}


# ---------------------------------------------------------------- node metrics join

def test_join_node_metrics_by_goal_id():
    t1 = trial([obj("o1", "bottle", True)], goal="g1")
    t2 = trial([obj("o2", "bottle", True), obj("o3", "bottle", True)], goal="g2", idx=1)
    t3 = trial([obj("o4", "bottle", False)], goal=None, idx=2, status="TIMEOUT")
    lines = [line("g1", "object_1", True, plan=0.5, cycle=10.0),
             line("g2", "object_2", False, "plan_failed", "pregrasp", plan=0.2),
             line("g2", "object_2", True, plan=0.3, cycle=14.0, attempt=2),
             line("g2", "object_3", False, "unreachable", "reach_check", plan=0.0, attempt=0),
             line("g2", "object_4", True, plan=0.4, cycle=11.0),
             line("zz", "object_9", True, plan=9.0, cycle=99.0)]
    by_goal, unjoined = es.group_metrics_by_goal([t1, t2, t3], lines)
    assert unjoined == 1 and len(by_goal["g1"]) == 1 and len(by_goal["g2"]) == 4
    m = es.compute_metrics([t1, t2, t3], by_goal)
    assert sorted(m["planning_s"]) == [0.2, 0.3, 0.4, 0.5]   # reach_check excluded
    assert sorted(m["cycle_s"]) == [10.0, 11.0, 14.0]        # successful attempts only
    assert (m["attempts"], m["attempt_objects"], m["attempt_trials"]) == (4, 3, 2)
    assert m["attempt_failures"] == {"plan_failed": 1, "unreachable": 1}


# ---------------------------------------------------------------- targets

def _groups(correct_low, n_low, err_mm, yaw, plan):
    g = es.compute_metrics([], {})
    g.update(correct=correct_low, objects=n_low)
    overall = es.compute_metrics([], {})
    overall["err3d_mm"] = {"sports ball": err_mm, "bottle": err_mm}
    overall["yaw_deg"] = yaw
    overall["planning_s"] = plan
    return {(2, 0.08): g, (2, 0.03): dict(g, correct=0)}, overall


def test_targets_pass_fail_and_boundaries():
    t = dict(es.DEFAULT_TARGETS)
    groups, overall = _groups(9, 10, [10.0, 14.9], [9.0], [0.1, 1.9])
    rows = es.evaluate_targets(groups, overall, t)
    assert [r[3] for r in rows] == ["PASS"] * 5      # 90 % is >= 90 %: PASS; 3 cm excluded
    assert "90.0 % (9/10)" in rows[0][2]
    groups, overall = _groups(8, 10, [15.0], [10.0], [2.0] * 3)
    rows = es.evaluate_targets(groups, overall, t)
    assert [r[3] for r in rows] == ["FAIL"] * 5      # strict "<" at the boundaries


def test_targets_missing_data_is_na():
    rows = es.evaluate_targets({}, es.compute_metrics([], {}), dict(es.DEFAULT_TARGETS))
    assert all(r[3] == "n/a" for r in rows)
    assert all(r[2].startswith("n/a") for r in rows)


def test_parse_targets_overrides():
    t = es.parse_targets(["plan_p95_s=1.5,success_low_clutter=0.8"])
    assert t["plan_p95_s"] == 1.5 and t["success_low_clutter"] == 0.8
    with pytest.raises(ValueError):
        es.parse_targets(["nope=1"])


# ---------------------------------------------------------------- end to end

def test_summarise_empty_run_dir(tmp_path):
    md = es.summarise(str(tmp_path), dict(es.DEFAULT_TARGETS), 0.75, 0.05, date="2026-01-01",
                      commit="abc1234")
    assert "trials.jsonl missing" in md and "node_metrics.jsonl missing" in md
    assert "| Pick success rate | n/a (0/0) |" in md
    assert "PASS" not in md and "FAIL" not in md


def test_main_writes_report(tmp_path, capsys):
    t = trial([obj("b1", "sports ball", True, loc_id=1, err=0.004),
               obj("o1", "bottle", False, loc_id=2, yaw=176.0)],
              reasons=["object_2 (bottle): no_ik: x"])
    (tmp_path / "trials.jsonl").write_text(json.dumps(t) + "\n")
    (tmp_path / "node_metrics.jsonl").write_text(json.dumps(line("g1", "object_1", True)) + "\n")
    out = tmp_path / "out" / "results.md"
    assert es.main([str(tmp_path), "--out", str(out)]) == 0
    md = out.read_text()
    assert md in capsys.readouterr().out
    assert "50.0 % (1/2)" in md and "| no_ik | 1 | 1 |" in md
    assert "4.0 / 4.0 (n=1)" in md      # ball 3D error in mm
    assert "4.0 / 4.0 (n=1)" in md.split("yaw")[1]  # bottle yaw 176 deg folded to 4 deg
