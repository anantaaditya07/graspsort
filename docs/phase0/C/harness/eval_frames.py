#!/usr/bin/env python3
"""Phase 0 / C: YOLOv8n (ONNX Runtime, CPU) detection rate on the frames saved by grab_objects.py.

Tooling venv python (no ROS):
    <venv>/bin/python eval_frames.py --frames $C_SCRATCH/frames --model $C_SCRATCH/models/yolov8n.onnx \
        --out-csv results_frames.csv --summary-csv results_summary.csv --annot-dir <dir>

Pre/post-processing is imported from check_yolo_on_frames.py (verbatim copy of the SemNav
script that mirrors the C++ detector node: letterbox 114, RGB /255, decode [1,84,8400], NMS).

Ground truth: tight box around the pixels whose depth differs from the empty-table background
frame by more than --gt-depth-diff metres (largest connected component). Depth and colour come
from the same Gazebo depth sensor, so the box is pixel-aligned with the colour image.
"""
import argparse
import ast
import csv
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_yolo_on_frames import decode, letterbox, nms, preprocess, unletterbox_boxes  # noqa

EXPECTED = {"cup": ["cup"], "bottle": ["bottle"], "ball": ["sports ball"],
            "can": None, "box": None}  # None -> report the best-matching class that fires


def gt_box(depth, bg, thr):
    d = np.nan_to_num(depth, nan=10.0, posinf=10.0, neginf=10.0)
    b = np.nan_to_num(bg, nan=10.0, posinf=10.0, neginf=10.0)
    mask = (np.abs(d - b) > thr).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None, 0
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h, area = stats[i]
    return (float(x), float(y), float(x + w - 1), float(y + h - 1)), int(area)


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--out-csv", required=True)
    ap.add_argument("--summary-csv", required=True)
    ap.add_argument("--annot-dir", default=None)
    ap.add_argument("--min-conf", type=float, default=0.05, help="decode threshold (to log weak hits)")
    ap.add_argument("--nms-iou", type=float, default=0.45)
    ap.add_argument("--match-iou", type=float, default=0.5)
    ap.add_argument("--conf-pass", type=float, default=0.35)
    ap.add_argument("--conf-low", type=float, default=0.25)
    ap.add_argument("--gt-depth-diff", type=float, default=0.004)
    ap.add_argument("--threads", type=int, default=4)
    a = ap.parse_args()

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = a.threads
    sess = ort.InferenceSession(a.model, sess_options=opts, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    size = int(inp.shape[2])
    names = ast.literal_eval(sess.get_modelmeta().custom_metadata_map["names"])
    frames = Path(a.frames)
    rows = list(csv.DictReader((frames / "manifest.csv").open()))
    if a.annot_dir:
        Path(a.annot_dir).mkdir(parents=True, exist_ok=True)

    def run(img):
        h, w = img.shape[:2]
        padded, scale, pad = letterbox(img, size)
        t0 = time.perf_counter()
        out = sess.run(None, {inp.name: preprocess(padded)})[0]
        ms = (time.perf_counter() - t0) * 1000.0
        boxes, scores, cids = decode(out, a.min_conf)
        keep = nms(boxes, scores, cids, a.min_conf, a.nms_iou)
        xyxy = unletterbox_boxes(boxes[keep], scale, pad, w, h)
        return [(tuple(map(float, bb)), names[int(c)], float(s)) for bb, s, c in
                zip(xyxy, scores[keep], cids[keep])], ms

    times = []
    per_frame = []
    for cam in sorted(d.name for d in frames.iterdir() if d.is_dir()):
        bg = np.load(frames / cam / "background_depth.npy")
        for r in rows:
            img = cv2.imread(str(frames / cam / f"{r['tag']}.png"))
            depth = np.load(frames / cam / f"{r['tag']}_depth.npy")
            gt, area = gt_box(depth, bg, a.gt_depth_diff)
            dets, ms = run(img)
            times.append(ms)
            # best conf per class among detections overlapping the GT box
            best = {}
            for bb, cname, s in dets:
                if gt is not None and iou(bb, gt) >= a.match_iou:
                    best[cname] = max(best.get(cname, 0.0), s)
            top = max(best.items(), key=lambda kv: kv[1]) if best else ("none", 0.0)
            per_frame.append(dict(cam=cam, tag=r["tag"], model=r["model"], cls=r["cls"],
                                  gt=gt, gt_area=area, best=best, top=top,
                                  all=[(c, round(s, 3)) for _, c, s in dets if s >= a.conf_low]))
            if a.annot_dir:
                vis = img.copy()
                if gt is not None:
                    cv2.rectangle(vis, (int(gt[0]), int(gt[1])), (int(gt[2]), int(gt[3])), (0, 255, 0), 1)
                for bb, cname, s in dets:
                    if s < a.conf_low:
                        continue
                    p1, p2 = (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3]))
                    cv2.rectangle(vis, p1, p2, (0, 0, 255), 2)
                    cv2.putText(vis, f"{cname} {s:.2f}", (p1[0], max(12, p1[1] - 3)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.imwrite(str(Path(a.annot_dir) / f"{cam}_{r['tag']}.png"), vis)

    with open(a.out_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["viewpoint", "tag", "model", "intended_class", "gt_box", "gt_area_px",
                    "top_class_iou50", "top_conf", "matched_classes_iou50", "all_dets_ge_0.25"])
        for f in per_frame:
            w.writerow([f["cam"], f["tag"], f["model"], f["cls"],
                        "" if f["gt"] is None else "%d %d %d %d" % tuple(f["gt"]), f["gt_area"],
                        f["top"][0], f"{f['top'][1]:.3f}",
                        ";".join(f"{k}:{v:.3f}" for k, v in sorted(f["best"].items(), key=lambda kv: -kv[1])),
                        ";".join(f"{c}:{s}" for c, s in f["all"])])

    groups = defaultdict(list)
    for f in per_frame:
        groups[(f["cls"], f["model"], f["cam"])].append(f)
        groups[(f["cls"], "ALL", f["cam"])].append(f)
    summ = []
    for (cls, model, cam), fs in sorted(groups.items()):
        n = len(fs)
        cands = EXPECTED[cls]
        if cands is None:  # best-matching class that fires (by frames >= conf_low)
            counts = defaultdict(int)
            for f in fs:
                for k, v in f["best"].items():
                    if v >= a.conf_low:
                        counts[k] += 1
            cands = [max(counts, key=counts.get)] if counts else ["none"]
        c = cands[0]
        confs = [f["best"].get(c, 0.0) for f in fs]
        r35 = sum(x >= a.conf_pass for x in confs) / n
        r25 = sum(x >= a.conf_low for x in confs) / n
        tops = defaultdict(int)
        for f in fs:
            tops[f["top"][0] if f["top"][1] >= a.conf_low else "none"] += 1
        summ.append([cls, model, cam, c, n, sum(f["gt"] is None for f in fs), f"{r35:.2f}",
                     f"{r25:.2f}", f"{max(confs):.3f}", f"{np.mean(confs):.3f}",
                     " ".join(f"{k}:{v}" for k, v in sorted(tops.items(), key=lambda kv: -kv[1])),
                     "PASS" if r35 >= 0.8 else "FAIL"])
    with open(a.summary_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["intended_class", "model", "viewpoint", "scored_coco_class", "frames", "gt_missing",
                    "det_rate_0.35_iou0.5", "det_rate_0.25_iou0.5", "max_conf", "mean_conf",
                    "top_class_fired_iou50_ge0.25 (frames)", "pass_0.8"])
        w.writerows(summ)
    for s in summ:
        print(",".join(map(str, s)))
    t = np.array(times[1:])  # drop first (warm-up)
    print(f"inference ms over {len(t)} frames (ORT CPU, {a.threads} threads, 640x640): "
          f"mean {t.mean():.1f} p50 {np.percentile(t, 50):.1f} p95 {np.percentile(t, 95):.1f}")


if __name__ == "__main__":
    main()
