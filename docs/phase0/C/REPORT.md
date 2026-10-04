# Phase 0 / C: YOLOv8n (COCO, ONNX Runtime) on Gazebo Classic sim images + RGB-D topic check

Feeds **D-04** (object set, class mapping, camera viewpoint, fine-tune or not). Evidence only; no
project code, nothing committed. Date 2026-10-04, ~35 min of the 1.5 h time box.

## 1. Summary

* **COCO YOLOv8n is viewpoint-limited.** From a pure **top-down** camera nothing passes except balls:
  cup 0/20, bottle 0/20, can and box ≈0 at both 0.95 m and 0.65 m height. From an **oblique**
  camera (50° pitch) cups/bottles/balls are detected; the closer one (0.7 m) works best for cups.
* **Passing (≥ 80 % frames, conf ≥ 0.35, IoU ≥ 0.5)** per model x viewpoint:
  `cricket_ball` → sports ball (oblique 1.00, oblique_near 1.00, top_near 1.00, top 0.80);
  `mustard_bottle` → bottle (oblique 0.80, oblique_near 0.80); `robocup_3Dsim_ball` (top 0.90,
  top_near 0.80); `robocup_spl_ball` (top 0.80); `master_chef_can` → *bottle* (oblique_near 0.90).
* **Per class (all models pooled):** only **ball** passes (top 0.83, top_near 0.80). Bottle best
  0.70 (oblique), cup best 0.65 (oblique_near), can and box never reach 0.80 for any class.
* **can and box have no COCO class and nothing stable fires:** cans → bottle / cup / cell phone /
  nothing (oblique), clock / frisbee / sports ball (top); boxes → book (max 0.44) or nothing.
* **Fewer than 3 classes pass → the pre-recorded fallback applies:** `make_dataset.py`
  auto-labelled sim images + short YOLOv8n fine-tune (to be decided in D-04, not done now).
* Misses are **per pose and deterministic** (re-running the capture gave identical numbers for the oblique and top views),
  so temporal averaging in the localizer will NOT recover a pose YOLO misses on a static table.
* **RGB-D: all checks pass.** color + depth 10.0 Hz, 0 frames lost (CycloneDDS, SensorData QoS sub),
  depth `32FC1` in metres (z-depth, not Euclidean range), CameraInfo present, color and depth
  share `camera_color_optical_frame` (optical convention verified by projection), depth error at
  4 known points ≤ 3.1 mm (oblique) and 0.0 mm (top-down).
* Inference (ORT 1.23 CPU, 4 threads, 640x640): mean 47.8 ms, p50 47.2, p95 53.3 (479 frames).

## 2. Headline table (best model per class and viewpoint)

| class | model | viewpoint | det rate @0.35 | det @0.25 | max conf | class that fired |
|---|---|---|---|---|---|---|
| ball | cricket_ball | oblique / oblique_near / top / top_near | 1.00 / 1.00 / 0.80 / 1.00 | 1.00 / 1.00 / 0.80 / 1.00 | 0.83 / 0.88 / 0.72 / 0.96 | sports ball |
| ball | robocup_spl_ball (orange-red) | oblique_near | 0.00 | 0.00 | – | **orange** 10/10 |
| bottle | mustard_bottle | oblique / oblique_near | 0.80 / 0.80 | 1.00 / 0.80 | 0.73 / 0.81 | bottle |
| bottle | mustard_bottle | top / top_near | 0.00 / 0.00 | 0.00 / 0.00 | 0 | none / sports ball / remote |
| bottle | beer (osrf, textured cylinder) | oblique / oblique_near | 0.60 / 0.20 | 0.70 / 0.40 | 0.72 / 0.42 | bottle, vase, cup |
| cup | plastic_cup (osrf, grey transparent) | oblique / oblique_near | 0.30 / 0.70 | 0.50 / 0.80 | 0.84 / 0.81 | cup |
| cup | pitcher_base (YCB pitcher) | oblique / oblique_near | 0.50 / 0.60 | 0.70 / 0.70 | 0.70 / 0.83 | cup |
| cup | both | top / top_near | 0.00 / 0.00 | 0.00 / 0.00 | 0 | none / mouse |
| can | master_chef_can (YCB) | oblique_near | 0.90 as *bottle* | 1.00 | 0.76 | bottle 9, cup 1 |
| can | coke_can (osrf) | oblique_near | 0.40 as *cup* | 0.50 | 0.53 | cup 5, bottle 2, none 3 |
| can | potted_meat_can (YCB) | oblique_near | 0.10 as bottle | 0.10 | 0.36 | none 9 |
| can | any | top / top_near | ≤ 0.20 | ≤ 0.30 | ≤ 0.78 | clock, frisbee, sports ball, none |
| box | cracker_box (YCB) | oblique / oblique_near | 0.20 / 0.20 as *book* | 0.40 / 0.30 | 0.44 | book, none |
| box | cardboard_box (osrf, 0.5x0.4x0.3 m) | all | ≤ 0.10 | ≤ 0.10 | ≤ 0.38 | none (umbrella/chair once) |

Full table (every model x viewpoint, plus pooled "ALL" rows): section 9 and `results_summary.csv`;
per-frame detail (GT box, matched classes, every detection ≥ 0.25): `results_frames.csv`.

## 3. Method

* **World** (`harness/yolo_check.world`): ground plane, sun + a ceiling point light + a directional
  fill light from the camera side, a 1.0 x 0.8 m wooden table with top at 0.75 m, four static
  Gazebo Classic `depth` sensors (640x480, 10 Hz, HFOV 1.2112 rad = 69.4°, like a D435 colour
  imager) with `libgazebo_ros_camera.so`:

  | viewpoint | namespace | link pose (x y z, pitch) | distance to table centre |
  |---|---|---|---|
  | oblique | /camera | -0.643 0 1.516, 50° down | 1.0 m along the optical axis |
  | oblique_near | /camera_near | -0.450 0 1.286, 50° down | 0.7 m |
  | top | /camera_top | 0 0 1.70, 90° down | 0.95 m above table top |
  | top_near | /camera_top_near | 0 0 1.40, 90° down | 0.65 m above table top |

  The two "near" viewpoints were added after the first pass showed small objects (cans ≈ 30-40 px)
  at 1 m; they test size vs viewpoint.
* **Objects** spawned via `/spawn_entity` as **static** models resting on the table, one at a time,
  5 positions ((0,0), (±0.15, ±0.12), (-0.12,-0.15)) x 2 yaws (0, 2.0 rad) = 10 frames per model and
  viewpoint; ≥ 20 frames per class per viewpoint (cup 20, bottle 20, can 30, box 20, ball 30).
  Plus one 5-object clutter frame per viewpoint (bonus, qualitative).
* **Ground truth box:** tight box around the pixels whose depth differs from the empty-table
  depth frame by > 4 mm (largest connected component, 3x3 opening). Depth and colour come from the
  same Gazebo sensor, so the mask is pixel-aligned with the colour image. This is the tight box of
  the *visible* object (what YOLO is trained to output) instead of projected 3D-box corners, which
  over-size cylinders in oblique views. Spot-checked visually on ~15 annotated frames (green boxes
  in `images/`); 0 frames without GT.
* **Detector:** `yolov8n.onnx` re-exported from SemNav's `models/yolov8n.pt` with SemNav's
  `export_yolo.py` defaults (static 1x3x640x640, opset 12, simplified, output 1x84x8400) into
  scratch. Pre/post-processing imported from a verbatim copy of SemNav `check_yolo_on_frames.py`
  (letterbox 114, RGB/255, decode, per-class NMS 0.45) which mirrors the C++ node.
* **Metric:** a frame counts if a detection of the scored COCO class has conf ≥ 0.35 and IoU ≥ 0.5
  with the GT box (also reported at 0.25). Scored class: cup→cup, bottle→bottle, ball→sports ball;
  can/box → the class that fires most often (≥ 0.25, IoU ≥ 0.5) for that model/viewpoint.
  Mean conf = mean over frames of the best matching conf of that class (0 when absent).

## 4. Exact commands (trimmed real output in `logs/`)

```bash
source docs/phase0/C/harness/env.sh   # ROS_DOMAIN_ID=43, GAZEBO_MASTER_URI=:11353, cyclonedds, GAZEBO_MODEL_PATH
docs/phase0/C/harness/fetch_candidates.sh $C_SCRATCH/gzmodels          # models + MTL fix (sec. 5, 7)
~/robotics-project-01/.venv/bin/python $C_SCRATCH/scripts/export_yolo.py \
    --weights $C_SCRATCH/models/yolov8n.pt --output-dir $C_SCRATCH/models
#  -> input images [1,3,640,640], output0 [1,84,8400], opset 12
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so docs/phase0/C/harness/yolo_check.world &
/usr/bin/python3 docs/phase0/C/harness/grab_objects.py --out $C_SCRATCH/frames --clutter   # 120 poses + clutter, 4 cams
~/robotics-project-01/.venv/bin/python docs/phase0/C/harness/eval_frames.py --frames $C_SCRATCH/frames \
    --model $C_SCRATCH/models/yolov8n.onnx --out-csv docs/phase0/C/results_frames.csv \
    --summary-csv docs/phase0/C/results_summary.csv --annot-dir $C_SCRATCH/annot
~/robotics-project-01/.venv/bin/python docs/phase0/C/harness/check_yolo_on_frames.py \
    --model $C_SCRATCH/models/yolov8n.onnx --input-dir $C_SCRATCH/clutter_in --output-dir $C_SCRATCH/clutter_out --conf-threshold 0.25
ros2 topic list -t; ros2 topic info -v /camera/depth/image_raw; ros2 topic hz /camera/{color,depth}/image_raw
ros2 run gazebo_ros spawn_entity.py -entity beer_rgbd -file beer_static.sdf -x 0.1 -y -0.05 -z 0.75
/usr/bin/python3 docs/phase0/C/harness/rgbd_check.py --ns /camera     --secs 10 --points '0,0,0.75;0.2,0.15,0.75;-0.15,-0.1,0.75;0.1,-0.05,0.98'
/usr/bin/python3 docs/phase0/C/harness/rgbd_check.py --ns /camera_top --secs 10 --points (same)
```

Clutter frame (`logs/clutter_check.log`, conf 0.25; objects: plastic_cup, mustard_bottle, coke_can,
cracker_box, cricket_ball):

```
clutter_oblique.png:      cup 0.837, sports ball 0.754
clutter_oblique_near.png: sports ball 0.889, bottle 0.611, cup 0.331, dining table 0.250
clutter_top.png:          remote 0.316, sports ball 0.266
clutter_top_near.png:     sports ball 0.747
```
Coke can and cracker box are never detected in clutter; cup/bottle only from oblique views.

## 5. Model sources, versions, licences (downloaded into scratch only, not in the repo)

| model | source | pin | licence |
|---|---|---|---|
| coke_can, beer, plastic_cup, cardboard_box, cricket_ball, robocup_spl_ball, robocup_3Dsim_ball | https://github.com/osrf/gazebo_models/tree/8163eb4b5e7e21985c6591d1c0bfb56468c0093f/<name> | commit 8163eb4b5e7e21985c6591d1c0bfb56468c0093f | CC BY 3.0 Unported (repo LICENSE) |
| Mustard Bottle (YCB 006) | https://fuel.gazebosim.org/1.0/Gambit/models/Mustard%20Bottle | version 2, zip sha256 e3862191…c8380 | CC BY 4.0 |
| Potted Meat Can (YCB 010) | https://fuel.gazebosim.org/1.0/Gambit/models/Potted%20Meat%20Can | version 2, sha256 6b45a5fb…bd173 | CC BY 4.0 |
| Cracker Box (YCB 003) | https://fuel.gazebosim.org/1.0/Gambit/models/Cracker%20Box | version 1, sha256 a01b03a4…69111 | CC BY 4.0 |
| Pitcher Base (YCB 019) | https://fuel.gazebosim.org/1.0/Gambit/models/Pitcher%20Base | version 2, sha256 c4746920…c41d | CC BY 4.0 |
| Master Chef Can (YCB 002) | https://fuel.gazebosim.org/1.0/petermitrano/models/Master%20Chef%20Can | version 2, sha256 2eb8dc0f…3618a | CC BY 4.0 |

Full zip SHA-256s: Cracker_Box a01b03a42e236f3a1cf249108ca70570c5b3efbe68a8470934e93118b8769111,
Master_Chef_Can 2eb8dc0f32e1b00dceff155aac2e5c1a38901d47647fa36a9139d89406e3618a,
Mustard_Bottle e3862191a0a9530afa584716f192f0be01be1d5a8d32db9312109fda1b7c8380,
Pitcher_Base c47469200c61717552719ae697fbde5b9a2f2c78a3579004a040d2bff0f6c41d,
Potted_Meat_Can 6b45a5fb0adf31c2065732e7921c1bd6c45d8e79243f3f828dc63ad8637bd173.
YCB meshes originate from the YCB Object and Model Set (ycbbenchmarks.com, CC BY 4.0).
No YCB **mug**, bleach bottle, soup can or tennis ball exists on Fuel in Gazebo form (searched
"ycb", "mug", "bottle", "soup", "tennis"); the only mugs are Google Scanned Objects
(GoogleResearch, CC BY 4.0), not in the approved source list, so not used. The OBJ meshes (not the
Y-up DAE copies) were used; all load in Gazebo Classic 11. `bowl` was downloaded but not tested.
YOLO weights: Ultralytics yolov8n.pt (AGPL-3.0), sha256 f59b3d83…fc83b36; re-exported ONNX
sha256 838d33325a01bc180a7a9fc4534e078139e8e3d38fee09ba673e8b01aa870f72 (SemNav's existing .onnx
c7d670c3… was not used).

## 6. RGB-D check (`logs/rgbd_topics.log`, `logs/rgbd_check.log`)

| check | result |
|---|---|
| topics | `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/depth/image_raw`, `/camera/depth/camera_info`, `/camera/depth/points` (needs 3 `<remapping>` rules, see 7.4) |
| rate (`ros2 topic hz`) | color 9.98 Hz, depth 9.98 Hz (window 104) |
| frame loss | 10 s window, RTF 1.00: /camera color/depth/infos 100/100 received (0 lost); /camera_top 99/100 (1 lost) with 4 cameras rendering and 2 other agents' gzservers running |
| QoS | publisher is RELIABLE, KEEP_LAST 5, VOLATILE; a SensorData (BEST_EFFORT) subscriber is compatible. `/clock` is BEST_EFFORT, so a RELIABLE /clock subscriber gets nothing (hit once in my script) |
| encodings | color `rgb8` (detector must convert or use cv_bridge `bgr8`), depth `32FC1`, 640x480, step 2560 |
| CameraInfo | K = [462.168 0 320.5; 0 462.168 240.5; 0 0 1], D = 0, plumb_bob; depth camera_info identical |
| frame_id | color and depth both `camera_color_optical_frame` (set by `<frame_name>`; the plugin default would be the link name, cf. SemNav D-05). Pixel data follows the optical convention (x right, y down, z forward): projecting world points with optical axes derived from the link pose hits the expected pixels (beer top found at the predicted pixel) |
| stamps | per sensor, colour and depth carry the same stamp (equal on /camera_top last pair; /camera last pair differed only because the two callbacks were sampled at different moments) |
| depth semantics | **z-depth along the optical axis, not Euclidean range** (top view: 0.9500 m at every table pixel, Euclidean would be 0.982 m at the 2nd point) |

Depth accuracy (camera link pose from the world file; optical axes x=right, y=down, z=forward;
Z = (p - c)·z_opt, u = fx·X/Z + cx, v = fy·Y/Z + cy):

```
/camera (oblique, c = (-0.643, 0, 1.516), pitch 50 deg)
point [0,0,0.75] table     -> pixel (320.5,240.4) GT Z 1.0001 m, measured 0.9990 m, error -0.0011 m
point [0.2,0.15,0.75]      -> pixel (259.1,177.7) GT Z 1.1287 m, measured 1.1256 m, error -0.0031 m
point [-0.15,-0.1,0.75]    -> pixel (371.6,299.2) GT Z 0.9037 m, measured 0.9024 m, error -0.0013 m
point [0.1,-0.05,0.98] beer top (r 0.055, h 0.23) -> pixel (346.5,123.6) GT Z 0.8882, measured 0.8853, error -0.0029 m
/camera_top (c = (0,0,1.70), pitch 90 deg)
table points: GT 0.9500, measured 0.9500 (error 0.0000); beer top GT 0.7200, measured 0.7200 (error 0.0000)
```
All within 1 cm (the oblique residual of 1-3 mm is consistent with rounding to the nearest pixel on
a surface whose depth changes ~3-5 mm/pixel at that slant). No depth noise is simulated.

## 7. Problems and fixes

1. **Gambit YCB models rendered black**: their MTL has only `map_Kd`, no `Kd`, and Gazebo Classic
   multiplies the texture by Kd = 0. Fix: rewrite the MTL with `Ka/Kd 1.0` (original kept as
   `textured.mtl.orig`); done in `fetch_candidates.sh`. Phase 1 must ship patched MTLs.
2. **~16 % of spawns missing from the frame** (first full pass: 19 / 120 poses had no object in
   either camera). Cause: re-using the same entity name right after `delete_entity` lets the
   rendering drop the new visual. Fix: unique entity name per spawn + check that the depth differs
   from the empty table before saving. After the fix 0 / 120 missing, no extra waits needed. The
   first-pass numbers were discarded. Relevant for eval_runner (Phase 6): never reuse names.
3. **Render lag after spawn**: frames 0.3-1.0 s (sim) after a spawn sometimes still showed the old
   scene; 2.0 s settle used.
4. **Topic names**: `gazebo_ros_camera` with `type="depth"` publishes `<ns>/<camera_name>/depth/...`
   (`/camera/color/depth/image_raw`). Matching architecture section 6 (`/camera/depth/image_raw`)
   needs `<remapping>color/depth/image_raw:=depth/image_raw</remapping>` (+ camera_info, points);
   verified working.
5. **Static osrf cricket_ball invisible in the quick pass**: same render-lag/name issue as 2/3, fine
   afterwards.
6. Fuel YCB SDFs have nonsense inertials (mass 0.6-2.5 g) and hard-coded model poses; harness wraps
   the OBJ in its own static SDF. Phase 1 needs real inertials for grasping.
7. `gzserver` ignores SIGTERM here; stopped with SIGINT (my own PIDs only).
8. Rendering worked headless with DISPLAY=:0 on the Intel iGPU; no software GL / xvfb needed.

## 8. Recommendation and options for D-04

**Viewpoint:** use an **oblique** fixed camera (~50° pitch, 0.7-1.0 m from the table centre,
beside the arm), not a pure top-down one. Top-down fails for every non-ball class with stock
COCO weights. This is a deviation from the PDF wording "fixed overhead camera" (scope bullet,
section 1) and should be recorded in DECISIONS.md. Grasps stay top-down; the localizer only needs
the camera pose from TF. The 0.7 m mount gives better cup confidence but sees ~0.9 m of table width
at the centre line; 1.0 m sees the whole 1 m table.

**Object set with stock COCO weights (oblique):** `cricket_ball` → sports ball (1.00),
`mustard_bottle` → bottle (0.80); third class `plastic_cup` → cup is only 0.70 (0.80 at conf 0.25).
That is **2 classes passing, < 3**, so the pre-recorded fallback (auto-labelled dataset + short
fine-tune) is triggered. Avoid `robocup_spl_ball` (detected as orange), `cardboard_box` (0.5 m,
ungraspable and undetected), `master_chef_can` (~10.3 cm diameter > 85 mm 2F-85 stroke, and it
fires as "bottle").

Options:
* **A. COCO only, 3 classes, accept a marginal cup**: ball, bottle, cup at oblique_near; cup
  0.70 per frame (deterministic per pose, so a missed pose stays missed). Cheapest; cans and boxes
  dropped; pick success will be capped by cup recall.
* **B. Fine-tune fallback now-ish (architecture section 9)**: `make_dataset.py` renders randomized
  layouts and labels them from depth masks / ground truth (the depth-difference box used here
  already gives exact labels for free), then a short YOLOv8n fine-tune on 5 classes
  (cup, bottle, can, box, ball) using these YCB/osrf models. Allows the PDF's intended classes and
  even top-down views. Cost: dataset tool + 1-2 h training + re-export; adds class-name changes to
  the detector `class_filter` only.
* **C. Hybrid (recommended)**: build Phases 1-5 with option A's set (ball, bottle, cup; oblique
  camera) so nothing blocks, and schedule B in Phase 3/6 to add can and box and to lift cup recall;
  B is already in the cut order as the last item, so A is the safe floor.
* **D. Try more viewpoints/objects before deciding** (e.g. 35-40° pitch, more cup models such as
  Google Scanned Objects mugs, which would need an approval to download). Not done; time-box
  candidate only.

## 9. Full results table (`results_summary.csv`)

| class | model | viewpoint | scored COCO class | n | det @0.35 | det @0.25 | max conf | mean conf | top class fired (IoU>=0.5, >=0.25), frames | >=80% |
|---|---|---|---|---|---|---|---|---|---|---|
| ball | cricket_ball | oblique | sports ball | 10 | 1.00 | 1.00 | 0.830 | 0.759 | sports ball:10 | PASS |
| ball | cricket_ball | oblique_near | sports ball | 10 | 1.00 | 1.00 | 0.877 | 0.671 | sports ball:10 | PASS |
| ball | cricket_ball | top | sports ball | 10 | 0.80 | 0.80 | 0.721 | 0.463 | sports ball:8 none:2 | PASS |
| ball | cricket_ball | top_near | sports ball | 10 | 1.00 | 1.00 | 0.961 | 0.866 | sports ball:10 | PASS |
| ball | robocup_3Dsim_ball | oblique | sports ball | 10 | 0.50 | 0.70 | 0.726 | 0.394 | sports ball:7 none:3 | FAIL |
| ball | robocup_3Dsim_ball | oblique_near | sports ball | 10 | 0.70 | 0.90 | 0.613 | 0.427 | sports ball:9 none:1 | FAIL |
| ball | robocup_3Dsim_ball | top | sports ball | 10 | 0.90 | 1.00 | 0.799 | 0.686 | sports ball:10 | PASS |
| ball | robocup_3Dsim_ball | top_near | sports ball | 10 | 0.80 | 0.90 | 0.727 | 0.487 | sports ball:9 none:1 | PASS |
| ball | robocup_spl_ball | oblique | sports ball | 10 | 0.20 | 0.20 | 0.510 | 0.100 | orange:8 sports ball:2 | FAIL |
| ball | robocup_spl_ball | oblique_near | sports ball | 10 | 0.00 | 0.00 | 0.000 | 0.000 | orange:10 | FAIL |
| ball | robocup_spl_ball | top | sports ball | 10 | 0.80 | 0.80 | 0.642 | 0.373 | sports ball:6 orange:4 | PASS |
| ball | robocup_spl_ball | top_near | sports ball | 10 | 0.60 | 0.60 | 0.667 | 0.408 | sports ball:6 orange:4 | FAIL |
| ball | **ALL** | oblique | sports ball | 30 | 0.57 | 0.63 | 0.830 | 0.418 | sports ball:19 orange:8 none:3 | FAIL |
| ball | **ALL** | oblique_near | sports ball | 30 | 0.57 | 0.63 | 0.877 | 0.366 | sports ball:19 orange:10 none:1 | FAIL |
| ball | **ALL** | top | sports ball | 30 | 0.83 | 0.87 | 0.799 | 0.507 | sports ball:24 orange:4 none:2 | PASS |
| ball | **ALL** | top_near | sports ball | 30 | 0.80 | 0.83 | 0.961 | 0.587 | sports ball:25 orange:4 none:1 | PASS |
| bottle | beer | oblique | bottle | 10 | 0.60 | 0.70 | 0.722 | 0.416 | bottle:7 none:2 parking meter:1 | FAIL |
| bottle | beer | oblique_near | bottle | 10 | 0.20 | 0.40 | 0.421 | 0.183 | bottle:4 vase:3 cup:2 none:1 | FAIL |
| bottle | beer | top | bottle | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:10 | FAIL |
| bottle | beer | top_near | bottle | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:8 sports ball:1 kite:1 | FAIL |
| bottle | mustard_bottle | oblique | bottle | 10 | 0.80 | 1.00 | 0.731 | 0.542 | bottle:10 | PASS |
| bottle | mustard_bottle | oblique_near | bottle | 10 | 0.80 | 0.80 | 0.814 | 0.546 | bottle:8 vase:1 banana:1 | PASS |
| bottle | mustard_bottle | top | bottle | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:9 frisbee:1 | FAIL |
| bottle | mustard_bottle | top_near | bottle | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:5 sports ball:3 frisbee:1 remote:1 | FAIL |
| bottle | **ALL** | oblique | bottle | 20 | 0.70 | 0.85 | 0.731 | 0.479 | bottle:17 none:2 parking meter:1 | FAIL |
| bottle | **ALL** | oblique_near | bottle | 20 | 0.50 | 0.60 | 0.814 | 0.365 | bottle:12 vase:4 cup:2 banana:1 none:1 | FAIL |
| bottle | **ALL** | top | bottle | 20 | 0.00 | 0.00 | 0.000 | 0.000 | none:19 frisbee:1 | FAIL |
| bottle | **ALL** | top_near | bottle | 20 | 0.00 | 0.00 | 0.000 | 0.000 | none:13 sports ball:4 frisbee:1 remote:1 kite:1 | FAIL |
| box | cardboard_box | oblique | none | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:10 | FAIL |
| box | cardboard_box | oblique_near | chair | 10 | 0.10 | 0.10 | 0.384 | 0.059 | none:9 chair:1 | FAIL |
| box | cardboard_box | top | book | 10 | 0.00 | 0.10 | 0.256 | 0.026 | none:9 book:1 | FAIL |
| box | cardboard_box | top_near | none | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:10 | FAIL |
| box | cracker_box | oblique | book | 10 | 0.20 | 0.40 | 0.441 | 0.171 | none:6 book:4 | FAIL |
| box | cracker_box | oblique_near | book | 10 | 0.20 | 0.30 | 0.434 | 0.179 | none:6 book:3 remote:1 | FAIL |
| box | cracker_box | top | none | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:10 | FAIL |
| box | cracker_box | top_near | book | 10 | 0.00 | 0.10 | 0.340 | 0.112 | none:9 book:1 | FAIL |
| box | **ALL** | oblique | book | 20 | 0.10 | 0.20 | 0.441 | 0.086 | none:16 book:4 | FAIL |
| box | **ALL** | oblique_near | book | 20 | 0.10 | 0.15 | 0.434 | 0.089 | none:15 book:3 remote:1 chair:1 | FAIL |
| box | **ALL** | top | book | 20 | 0.00 | 0.05 | 0.256 | 0.016 | none:19 book:1 | FAIL |
| box | **ALL** | top_near | book | 20 | 0.00 | 0.05 | 0.340 | 0.056 | none:19 book:1 | FAIL |
| can | coke_can | oblique | bottle | 10 | 0.10 | 0.20 | 0.382 | 0.154 | none:8 bottle:2 | FAIL |
| can | coke_can | oblique_near | cup | 10 | 0.40 | 0.50 | 0.532 | 0.278 | cup:5 none:3 bottle:2 | FAIL |
| can | coke_can | top | none | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:10 | FAIL |
| can | coke_can | top_near | sports ball | 10 | 0.20 | 0.20 | 0.450 | 0.126 | none:8 sports ball:2 | FAIL |
| can | master_chef_can | oblique | bottle | 10 | 0.50 | 0.60 | 0.646 | 0.353 | bottle:6 none:3 cup:1 | FAIL |
| can | master_chef_can | oblique_near | bottle | 10 | 0.90 | 1.00 | 0.763 | 0.580 | bottle:9 cup:1 | PASS |
| can | master_chef_can | top | clock | 10 | 0.10 | 0.30 | 0.351 | 0.132 | none:7 clock:3 | FAIL |
| can | master_chef_can | top_near | sports ball | 10 | 0.40 | 0.60 | 0.780 | 0.375 | sports ball:6 none:3 frisbee:1 | FAIL |
| can | potted_meat_can | oblique | cell phone | 10 | 0.20 | 0.20 | 0.660 | 0.215 | none:7 cell phone:2 bottle:1 | FAIL |
| can | potted_meat_can | oblique_near | bottle | 10 | 0.10 | 0.10 | 0.358 | 0.075 | none:9 bottle:1 | FAIL |
| can | potted_meat_can | top | remote | 10 | 0.10 | 0.10 | 0.373 | 0.068 | none:9 remote:1 | FAIL |
| can | potted_meat_can | top_near | remote | 10 | 0.10 | 0.20 | 0.560 | 0.130 | none:8 remote:2 | FAIL |
| can | **ALL** | oblique | bottle | 30 | 0.20 | 0.30 | 0.646 | 0.193 | none:18 bottle:9 cell phone:2 cup:1 | FAIL |
| can | **ALL** | oblique_near | bottle | 30 | 0.40 | 0.47 | 0.763 | 0.280 | none:12 bottle:12 cup:6 | FAIL |
| can | **ALL** | top | clock | 30 | 0.03 | 0.10 | 0.351 | 0.047 | none:26 clock:3 remote:1 | FAIL |
| can | **ALL** | top_near | sports ball | 30 | 0.20 | 0.27 | 0.780 | 0.169 | none:19 sports ball:8 remote:2 frisbee:1 | FAIL |
| cup | pitcher_base | oblique | cup | 10 | 0.50 | 0.70 | 0.702 | 0.396 | cup:7 none:2 mouse:1 | FAIL |
| cup | pitcher_base | oblique_near | cup | 10 | 0.60 | 0.70 | 0.829 | 0.416 | cup:7 none:3 | FAIL |
| cup | pitcher_base | top | cup | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:5 mouse:2 sports ball:2 scissors:1 | FAIL |
| cup | pitcher_base | top_near | cup | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:6 mouse:4 | FAIL |
| cup | plastic_cup | oblique | cup | 10 | 0.30 | 0.50 | 0.839 | 0.333 | none:5 cup:5 | FAIL |
| cup | plastic_cup | oblique_near | cup | 10 | 0.70 | 0.80 | 0.809 | 0.469 | cup:8 none:1 mouse:1 | FAIL |
| cup | plastic_cup | top | cup | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:10 | FAIL |
| cup | plastic_cup | top_near | cup | 10 | 0.00 | 0.00 | 0.000 | 0.000 | none:10 | FAIL |
| cup | **ALL** | oblique | cup | 20 | 0.40 | 0.60 | 0.839 | 0.364 | cup:12 none:7 mouse:1 | FAIL |
| cup | **ALL** | oblique_near | cup | 20 | 0.65 | 0.75 | 0.829 | 0.443 | cup:15 none:4 mouse:1 | FAIL |
| cup | **ALL** | top | cup | 20 | 0.00 | 0.00 | 0.000 | 0.000 | none:15 mouse:2 sports ball:2 scissors:1 | FAIL |
| cup | **ALL** | top_near | cup | 20 | 0.00 | 0.00 | 0.000 | 0.000 | none:16 mouse:4 | FAIL |

## 10. Files

* `REPORT.md` (this), `results_summary.csv`, `results_frames.csv` (480 rows)
* `logs/`: `grab_objects.log`, `eval_frames.log`, `clutter_check.log`, `rgbd_topics.log`,
  `rgbd_check.log`, `gzserver_tail.log`
* `images/` (10 PNGs, 480x360; green = depth GT box, red = detections ≥ 0.25): clutter (3 views),
  cup oblique_near vs top_near, coke can, cracker box, SPL ball (orange), mustard top (miss),
  master chef can (bottle)
* `harness/`: `yolo_check.world`, `env.sh`, `fetch_candidates.sh`, `grab_objects.py`,
  `eval_frames.py`, `rgbd_check.py`, `check_yolo_on_frames.py` (verbatim SemNav copy)
* Scratch only (not in repo): models, ONNX, 480 frames + depth .npy, annotated frames.

## 11. Time

Started 11:46, finished ~12:25 (≈ 40 min of the 1.5 h box). Cleanup: my gzserver stopped
(SIGINT), no processes of mine left; other agents' gzservers untouched.
