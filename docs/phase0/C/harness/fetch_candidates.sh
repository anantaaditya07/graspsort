#!/usr/bin/env bash
# Phase 0 / C: download the candidate object models used for the YOLO check into $1 (scratch).
# Not for the repo. Usage: fetch_candidates.sh <dest_dir>
set -euo pipefail
DEST="${1:?dest dir}"
OSRF_SHA="8163eb4b5e7e21985c6591d1c0bfb56468c0093f"   # osrf/gazebo_models, CC BY 3.0 (per model.config / SemNav D-14)
mkdir -p "${DEST}/fuel"
cd "${DEST}"
curl -sf "https://api.github.com/repos/osrf/gazebo_models/git/trees/${OSRF_SHA}?recursive=1" > tree.json
python3 - > files.txt <<'PY'
import json
want = {'coke_can','beer','plastic_cup','cardboard_box','cricket_ball','robocup_spl_ball','robocup_3Dsim_ball'}
for e in json.load(open('tree.json'))['tree']:
    if e['type'] == 'blob' and e['path'].split('/')[0] in want and 'thumbnails' not in e['path']:
        print(e['path'])
PY
while read -r p; do
  mkdir -p "$(dirname "$p")"
  curl -sf --retry 3 -o "$p" "https://raw.githubusercontent.com/osrf/gazebo_models/${OSRF_SHA}/$p"
done < files.txt
# Gazebo Fuel YCB-derived models (CC BY 4.0), pinned versions.
for spec in "Gambit/Mustard Bottle/2" "Gambit/Potted Meat Can/2" "Gambit/Cracker Box/1" \
            "Gambit/Pitcher Base/2" "petermitrano/Master Chef Can/2"; do
  IFS=/ read -r owner name ver <<< "${spec}"
  enc="${name// /%20}"; dir="${name// /_}"
  curl -sfL -o "fuel/${dir}.zip" "https://fuel.gazebosim.org/1.0/${owner}/models/${enc}/${ver}/${enc}.zip"
  mkdir -p "fuel/${dir}" && unzip -q -o "fuel/${dir}.zip" -d "fuel/${dir}"
done
# Fix: the Gambit MTLs have only map_Kd and no Kd -> Gazebo Classic renders them black.
for d in Cracker_Box Mustard_Bottle Pitcher_Base Potted_Meat_Can; do
  cp "fuel/$d/textured.mtl" "fuel/$d/textured.mtl.orig"
  printf 'newmtl material_0\nKa 1.000000 1.000000 1.000000\nKd 1.000000 1.000000 1.000000\nKs 0.100000 0.100000 0.100000\nd 1.000000\nillum 2\nmap_Kd texture_map.png\n' > "fuel/$d/textured.mtl"
done
echo "done: ${DEST}"
