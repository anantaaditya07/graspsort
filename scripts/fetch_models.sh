#!/usr/bin/env bash
# fetch_models.sh - download the pinned GraspSort object models into
# src/graspsort_gazebo/models_external/ (gitignored, docs/DECISIONS.md D-04 and D-08)
# and patch them for simulation (mass, inertia, collision, material, pose).
#
# Usage:
#   scripts/fetch_models.sh          # install missing or outdated models; no-op otherwise
#   scripts/fetch_models.sh --force  # re-download and re-patch every model
#   scripts/fetch_models.sh --help
#
# Every downloaded file is checked against a pinned SHA-256 (mismatch = abort, nothing installed).
# Each installed model is a Gazebo Classic model directory (model.config + model.sdf) with one link
# named "link" (the attach plugin uses it as child_link) and an ATTRIBUTION.txt.
# Model origin = bottom centre of the object, so place it at the table-top height.
# Needs network access, curl, sha256sum, unzip, awk. No sudo.
set -euo pipefail

# ------------------------------------------------------------------------------------------------
# PHYSICAL PARAMETERS (edit here; a change re-patches the affected model on the next run)
#
# Inertia is NOT typed in by hand: it is computed below from mass + collision primitive with the
# solid-body formulas (sphere 2/5 m r^2, cylinder m(3r^2+h^2)/12 and m r^2/2, box m(b^2+c^2)/12),
# so the tensor always matches the mass and the collision shape. Inertial origin = shape centre.
# Collision uses a primitive fitted to the measured mesh bounding box (mesh collision is replaced:
# the 16k-face YCB trimesh and the open osrf cup trimesh rest poorly in ODE and are slow).
#
# model          shape     mass [kg]  size [m]                         source / reasoning
# cricket_ball   sphere    0.160      r 0.0375                          ICC Law 4 ball is 155.9-163.0 g;
#                                                                     osrf had 0.1467 kg (light). Radius
#                                                                     kept = osrf visual/collision sphere.
# plastic_cup    cylinder  0.060      r 0.0324, h 0.1301               osrf mass 0.0599 kg kept (realistic
#                                                                     rigid PP tumbler). osrf inertia was
#                                                                     ~3x too large for that mass/size.
#                                                                     Mesh is a frustum, r 0.0275 (bottom)
#                                                                     to 0.03725 (top), z -0.0726..0.0575,
#                                                                     axis at y +0.00785 in mesh frame;
#                                                                     cylinder r = mean radius (exact at
#                                                                     mid height, where it is grasped).
# mustard_bottle box       0.300      0.0972 x 0.0666 x 0.1913         YCB 006 measured full mass is
#                                                                     0.603 kg. Chosen 0.300 kg (a half
#                                                                     used bottle): the attach joint was
#                                                                     validated with 0.1 kg (D-03), so
#                                                                     stay within ~3x of that. Box = OBJ
#                                                                     vertex bbox x -0.063938..0.033260,
#                                                                     y -0.056809..0.009812,
#                                                                     z -0.003153..0.188148.
# ------------------------------------------------------------------------------------------------
BALL_MASS="0.160"
BALL_RADIUS="0.0375"

CUP_MASS="0.060"
CUP_RADIUS="0.0324"
CUP_HEIGHT="0.1301"
# Mesh-frame bottom centre of the cup mesh (moved to the model origin).
CUP_MESH_BOTTOM_X="0.0"
CUP_MESH_BOTTOM_Y="0.00785"
CUP_MESH_BOTTOM_Z="-0.0726"

BOTTLE_MASS="0.300"
BOTTLE_SIZE_X="0.0972"
BOTTLE_SIZE_Y="0.0666"
BOTTLE_SIZE_Z="0.1913"
# Mesh-frame bottom centre of the OBJ bounding box (moved to the model origin).
BOTTLE_MESH_BOTTOM_X="-0.015339"
BOTTLE_MESH_BOTTOM_Y="-0.0234985"
BOTTLE_MESH_BOTTOM_Z="-0.003153"

# ODE contact for all three models (osrf cricket_ball/plastic_cup values, reused for the bottle).
CONTACT_KP="100000"
CONTACT_KD="100"
CONTACT_MAX_VEL="100.0"
CONTACT_MIN_DEPTH="0.001"
FRICTION_MU="1.0"
TORSIONAL_COEFF="1.0"
TORSIONAL_SURFACE_RADIUS="0.01"
# Rolling resistance of the ball (osrf value).
BALL_ANGULAR_DECAY="0.005"

# Bump when the patch logic below changes (forces a re-patch of every model).
PATCH_REV="1"

# ------------------------------------------------------------------------------------------------
# PINNED SOURCES
# osrf/gazebo_models, CC BY 3.0 Unported (repo LICENSE), master as of 2023-07-15.
OSRF_SHA="8163eb4b5e7e21985c6591d1c0bfb56468c0093f"
OSRF_RAW="https://raw.githubusercontent.com/osrf/gazebo_models/${OSRF_SHA}"
OSRF_TREE="https://github.com/osrf/gazebo_models/tree/${OSRF_SHA}"
# SHA-256 of each file at that commit (computed 2026-10-04 from the URLs above).
declare -A OSRF_FILES_SHA256=(
  [cricket_ball/model.config]="ca467cca35565beaa8012530100d76434c78fc712b68e5905a01a6e664fed34d"
  [cricket_ball/model.sdf]="c4b1b6eb61bc7551c56b01df0f1004febb39f70b58d042648184dc31645871be"
  [plastic_cup/model.config]="b8894a070b710588d16e10f2edb350be0e1381e9ea7bb4e908e396690bc5c605"
  [plastic_cup/model.sdf]="3bd06e128d87f48769ee43e3a2a28be11251b605e3b5c365170a3a4903ad686a"
  [plastic_cup/meshes/plastic_cup.dae]="a245b42d8a228d158b6bf587ae55ca3da220b2cad9b2084c756d7e521e27909c"
)

# Gazebo Fuel Gambit/Mustard Bottle version 2 (YCB 006), CC BY 4.0.
# Zip SHA-256 from docs/phase0/C/REPORT.md section 5; re-downloaded and matched on 2026-10-04.
BOTTLE_FUEL_PAGE="https://fuel.gazebosim.org/1.0/Gambit/models/Mustard%20Bottle"
BOTTLE_FUEL_VERSION="2"
BOTTLE_ZIP_URL="${BOTTLE_FUEL_PAGE}/${BOTTLE_FUEL_VERSION}/Mustard%20Bottle.zip"
BOTTLE_ZIP_SHA256="e3862191a0a9530afa584716f192f0be01be1d5a8d32db9312109fda1b7c8380"

# ------------------------------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEST="${REPO_ROOT}/src/graspsort_gazebo/models_external"
STAMP_FILE=".fetch_models.stamp"

FORCE=0
for arg in "$@"; do
  case "${arg}" in
    --force) FORCE=1 ;;
    -h | --help)
      sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "error: unknown argument '${arg}' (use --force or --help)" >&2
      exit 2
      ;;
  esac
done

for tool in curl sha256sum unzip awk; do
  command -v "${tool}" >/dev/null 2>&1 || {
    echo "error: ${tool} not found" >&2
    exit 1
  }
done

TMP_DIR="$(mktemp -d)"
cleanup() {
  rm -rf "${TMP_DIR}"
  rm -rf "${DEST}"/.*.partial
}
trap cleanup EXIT

# download <url> <out> <sha256>
download() {
  local url="$1" out="$2" want="$3" got
  mkdir -p "$(dirname "${out}")"
  curl --fail --silent --show-error --location --retry 3 -o "${out}" "${url}"
  got="$(sha256sum "${out}" | cut -d' ' -f1)"
  if [[ "${got}" != "${want}" ]]; then
    echo "error: SHA-256 mismatch for ${url}" >&2
    echo "       expected ${want}" >&2
    echo "       got      ${got}" >&2
    exit 1
  fi
  echo "       ok sha256 ${got:0:16}...  $(basename "${out}")"
}

# Inertia diagonal "ixx iyy izz" (products are zero for these symmetric shapes about the centre).
inertia_sphere() { awk -v m="$1" -v r="$2" 'BEGIN{i=0.4*m*r*r; printf "%.6e %.6e %.6e", i, i, i}'; }
inertia_cylinder() {
  awk -v m="$1" -v r="$2" -v h="$3" \
    'BEGIN{t=m*(3*r*r+h*h)/12; printf "%.6e %.6e %.6e", t, t, 0.5*m*r*r}'
}
inertia_box() {
  awk -v m="$1" -v x="$2" -v y="$3" -v z="$4" \
    'BEGIN{printf "%.6e %.6e %.6e", m*(y*y+z*z)/12, m*(x*x+z*z)/12, m*(x*x+y*y)/12}'
}
half() { awk -v a="$1" 'BEGIN{printf "%.6f", a/2}'; }
# Offset that moves mesh point <bottom> to (0,0,-half_height) in the link frame (link = shape centre).
mesh_offset() {
  awk -v x="$1" -v y="$2" -v z="$3" -v hz="$4" 'function n(v) { return v == 0 ? 0 : v }
    BEGIN{printf "%.6f %.6f %.6f", n(-x), n(-y), n(-hz-z)}'
}

# surface_xml: ODE contact + friction block shared by all models.
surface_xml() {
  cat <<EOF
        <surface>
          <contact>
            <ode>
              <kp>${CONTACT_KP}</kp>
              <kd>${CONTACT_KD}</kd>
              <max_vel>${CONTACT_MAX_VEL}</max_vel>
              <min_depth>${CONTACT_MIN_DEPTH}</min_depth>
            </ode>
          </contact>
          <friction>
            <ode>
              <mu>${FRICTION_MU}</mu>
              <mu2>${FRICTION_MU}</mu2>
            </ode>
            <torsional>
              <coefficient>${TORSIONAL_COEFF}</coefficient>
              <use_patch_radius>0</use_patch_radius>
              <surface_radius>${TORSIONAL_SURFACE_RADIUS}</surface_radius>
            </torsional>
          </friction>
        </surface>
EOF
}

# inertial_xml <mass> <ixx iyy izz>
inertial_xml() {
  local m="$1" ixx iyy izz
  read -r ixx iyy izz <<<"$2"
  cat <<EOF
      <inertial>
        <mass>${m}</mass>
        <inertia>
          <ixx>${ixx}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>${iyy}</iyy>
          <iyz>0</iyz>
          <izz>${izz}</izz>
        </inertia>
      </inertial>
EOF
}

# up_to_date <name> <recipe>: true if the model is installed with the same recipe.
up_to_date() {
  local target="${DEST}/$1"
  [[ "${FORCE}" -eq 0 && -f "${target}/model.sdf" && -f "${target}/model.config" &&
    -f "${target}/${STAMP_FILE}" && "$(cat "${target}/${STAMP_FILE}")" == "$2" ]]
}

# commit <name> <recipe>: atomically replace the installed model by the staged one.
commit() {
  local name="$1" recipe="$2"
  printf '%s' "${recipe}" >"${DEST}/.${name}.partial/${STAMP_FILE}"
  rm -rf "${DEST:?}/${name}"
  mv "${DEST}/.${name}.partial" "${DEST}/${name}"
  echo "[done] ${name}"
}

osrf_attribution() {
  local name="$1" author="$2" changes="$3"
  cat <<EOF
${name} (Gazebo Classic model, fetched by scripts/fetch_models.sh; not tracked by git)

Source:   ${OSRF_TREE}/${name}
Pinned:   osrf/gazebo_models commit ${OSRF_SHA}
Author:   ${author}
Licence:  Creative Commons Attribution 3.0 Unported (CC BY 3.0),
          https://creativecommons.org/licenses/by/3.0/
          (osrf/gazebo_models repository LICENSE)

Changes made by GraspSort (docs/DECISIONS.md D-08):
${changes}
The original file is kept as model.sdf.orig.
EOF
}

# ------------------------------------------------------------------------------------------------
install_cricket_ball() {
  local name="cricket_ball" recipe stage inertia r
  recipe="rev=${PATCH_REV} osrf=${OSRF_SHA} m=${BALL_MASS} r=${BALL_RADIUS} decay=${BALL_ANGULAR_DECAY}"
  recipe+=" contact=${CONTACT_KP},${CONTACT_KD},${CONTACT_MAX_VEL},${CONTACT_MIN_DEPTH},${FRICTION_MU}"
  recipe+=",${TORSIONAL_COEFF},${TORSIONAL_SURFACE_RADIUS}"
  if up_to_date "${name}" "${recipe}"; then
    echo "[skip] ${name} up to date (use --force to re-download)"
    return 0
  fi
  echo "[get ] ${name}"
  stage="${DEST}/.${name}.partial"
  rm -rf "${stage}"
  local f
  for f in model.config model.sdf; do
    download "${OSRF_RAW}/${name}/${f}" "${stage}/${f}" "${OSRF_FILES_SHA256[${name}/${f}]}"
  done
  mv "${stage}/model.sdf" "${stage}/model.sdf.orig"
  r="${BALL_RADIUS}"
  inertia="$(inertia_sphere "${BALL_MASS}" "${r}")"
  cat >"${stage}/model.sdf" <<EOF
<?xml version="1.0" ?>
<!-- Generated by GraspSort scripts/fetch_models.sh from osrf/gazebo_models@${OSRF_SHA}
     cricket_ball (CC BY 3.0, see ATTRIBUTION.txt). Mass/inertia patched; do not edit by hand. -->
<sdf version="1.6">
  <model name="${name}">
    <link name="link">
      <pose>0 0 ${r} 0 0 0</pose>
$(inertial_xml "${BALL_MASS}" "${inertia}")
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>${r}</radius>
          </sphere>
        </geometry>
$(surface_xml)
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>${r}</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0.0</linear>
        <angular>${BALL_ANGULAR_DECAY}</angular>
      </velocity_decay>
    </link>
  </model>
</sdf>
EOF
  osrf_attribution "${name}" "Nate Koenig (OSRF)" \
    "- mass 0.1467 -> ${BALL_MASS} kg, inertia recomputed (solid sphere r ${r} m)
- SDF 1.5 -> 1.6, same sphere visual/collision, ODE contact and rolling decay as the original" \
    >"${stage}/ATTRIBUTION.txt"
  commit "${name}" "${recipe}"
}

install_plastic_cup() {
  local name="plastic_cup" recipe stage inertia hz off
  recipe="rev=${PATCH_REV} osrf=${OSRF_SHA} m=${CUP_MASS} r=${CUP_RADIUS} h=${CUP_HEIGHT}"
  recipe+=" bottom=${CUP_MESH_BOTTOM_X},${CUP_MESH_BOTTOM_Y},${CUP_MESH_BOTTOM_Z}"
  recipe+=" contact=${CONTACT_KP},${CONTACT_KD},${CONTACT_MAX_VEL},${CONTACT_MIN_DEPTH},${FRICTION_MU}"
  recipe+=",${TORSIONAL_COEFF},${TORSIONAL_SURFACE_RADIUS}"
  if up_to_date "${name}" "${recipe}"; then
    echo "[skip] ${name} up to date (use --force to re-download)"
    return 0
  fi
  echo "[get ] ${name}"
  stage="${DEST}/.${name}.partial"
  rm -rf "${stage}"
  local f
  for f in model.config model.sdf meshes/plastic_cup.dae; do
    download "${OSRF_RAW}/${name}/${f}" "${stage}/${f}" "${OSRF_FILES_SHA256[${name}/${f}]}"
  done
  mv "${stage}/model.sdf" "${stage}/model.sdf.orig"
  hz="$(half "${CUP_HEIGHT}")"
  off="$(mesh_offset "${CUP_MESH_BOTTOM_X}" "${CUP_MESH_BOTTOM_Y}" "${CUP_MESH_BOTTOM_Z}" "${hz}")"
  inertia="$(inertia_cylinder "${CUP_MASS}" "${CUP_RADIUS}" "${CUP_HEIGHT}")"
  cat >"${stage}/model.sdf" <<EOF
<?xml version="1.0" ?>
<!-- Generated by GraspSort scripts/fetch_models.sh from osrf/gazebo_models@${OSRF_SHA}
     plastic_cup (CC BY 3.0, see ATTRIBUTION.txt). Inertia/collision/pose patched; do not edit. -->
<sdf version="1.6">
  <model name="${name}">
    <link name="link">
      <pose>0 0 ${hz} 0 0 0</pose>
$(inertial_xml "${CUP_MASS}" "${inertia}")
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>${CUP_RADIUS}</radius>
            <length>${CUP_HEIGHT}</length>
          </cylinder>
        </geometry>
$(surface_xml)
      </collision>
      <visual name="visual">
        <pose>${off} 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://${name}/meshes/plastic_cup.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/GreyTransparent</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF
  osrf_attribution "${name}" "Jackie Kay (OSRF)" \
    "- mass kept ${CUP_MASS} kg; inertia recomputed (solid cylinder r ${CUP_RADIUS} m, h ${CUP_HEIGHT} m)
- mesh collision -> cylinder primitive (mean radius of the frustum mesh)
- visual offset so the mesh bottom sits at the model origin (the original sank 7.6 mm
  below it and was 7.85 mm off-axis in y)" \
    >"${stage}/ATTRIBUTION.txt"
  commit "${name}" "${recipe}"
}

install_mustard_bottle() {
  local name="mustard_bottle" recipe stage zip inertia hz off
  recipe="rev=${PATCH_REV} fuel=${BOTTLE_ZIP_SHA256} m=${BOTTLE_MASS}"
  recipe+=" box=${BOTTLE_SIZE_X},${BOTTLE_SIZE_Y},${BOTTLE_SIZE_Z}"
  recipe+=" bottom=${BOTTLE_MESH_BOTTOM_X},${BOTTLE_MESH_BOTTOM_Y},${BOTTLE_MESH_BOTTOM_Z}"
  recipe+=" contact=${CONTACT_KP},${CONTACT_KD},${CONTACT_MAX_VEL},${CONTACT_MIN_DEPTH},${FRICTION_MU}"
  recipe+=",${TORSIONAL_COEFF},${TORSIONAL_SURFACE_RADIUS}"
  if up_to_date "${name}" "${recipe}"; then
    echo "[skip] ${name} up to date (use --force to re-download)"
    return 0
  fi
  echo "[get ] ${name}"
  stage="${DEST}/.${name}.partial"
  rm -rf "${stage}"
  zip="${TMP_DIR}/mustard_bottle.zip"
  download "${BOTTLE_ZIP_URL}" "${zip}" "${BOTTLE_ZIP_SHA256}"
  mkdir -p "${TMP_DIR}/mustard" "${stage}/meshes" "${stage}/orig"
  unzip -q -o "${zip}" -d "${TMP_DIR}/mustard"
  # The Z-up OBJ is used (the DAE copy is Y-up and needed a hard-coded roll in the original SDF).
  cp "${TMP_DIR}/mustard/textured.obj" "${TMP_DIR}/mustard/texture_map.png" "${stage}/meshes/"
  cp "${TMP_DIR}/mustard/model.config" "${TMP_DIR}/mustard/model.sdf" \
    "${TMP_DIR}/mustard/textured.mtl" "${stage}/orig/"
  # Fix: the original MTL has map_Kd but no Kd, which Gazebo Classic renders black.
  cat >"${stage}/meshes/textured.mtl" <<'EOF'
newmtl material_0
Ka 1.000000 1.000000 1.000000
Kd 1.000000 1.000000 1.000000
Ks 0.100000 0.100000 0.100000
d 1.000000
illum 2
map_Kd texture_map.png
EOF
  # Fix: the original model.config points at a missing mustard_bottle.sdf.
  cat >"${stage}/model.config" <<EOF
<?xml version="1.0" ?>
<model>
  <name>mustard_bottle</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Adwait Naik (Gazebo Fuel Gambit); YCB Object and Model Set</name>
  </author>
  <description>
    YCB 006 mustard bottle, Gazebo Fuel Gambit/Mustard Bottle v${BOTTLE_FUEL_VERSION} (CC BY 4.0),
    patched by GraspSort scripts/fetch_models.sh. See ATTRIBUTION.txt.
  </description>
</model>
EOF
  hz="$(half "${BOTTLE_SIZE_Z}")"
  off="$(mesh_offset "${BOTTLE_MESH_BOTTOM_X}" "${BOTTLE_MESH_BOTTOM_Y}" "${BOTTLE_MESH_BOTTOM_Z}" "${hz}")"
  inertia="$(inertia_box "${BOTTLE_MASS}" "${BOTTLE_SIZE_X}" "${BOTTLE_SIZE_Y}" "${BOTTLE_SIZE_Z}")"
  cat >"${stage}/model.sdf" <<EOF
<?xml version="1.0" ?>
<!-- Generated by GraspSort scripts/fetch_models.sh from Gazebo Fuel Gambit/Mustard Bottle
     v${BOTTLE_FUEL_VERSION} (YCB 006, CC BY 4.0, see ATTRIBUTION.txt). Do not edit by hand. -->
<sdf version="1.6">
  <model name="${name}">
    <link name="link">
      <pose>0 0 ${hz} 0 0 0</pose>
$(inertial_xml "${BOTTLE_MASS}" "${inertia}")
      <collision name="collision">
        <geometry>
          <box>
            <size>${BOTTLE_SIZE_X} ${BOTTLE_SIZE_Y} ${BOTTLE_SIZE_Z}</size>
          </box>
        </geometry>
$(surface_xml)
      </collision>
      <visual name="visual">
        <pose>${off} 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://${name}/meshes/textured.obj</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
EOF
  cat >"${stage}/ATTRIBUTION.txt" <<EOF
mustard_bottle (Gazebo Classic model, fetched by scripts/fetch_models.sh; not tracked by git)

Source:   ${BOTTLE_FUEL_PAGE}  (version ${BOTTLE_FUEL_VERSION})
Download: ${BOTTLE_ZIP_URL}
Pinned:   zip SHA-256 ${BOTTLE_ZIP_SHA256}
Author:   Gazebo Fuel model by Adwait Naik (Gambit). Mesh and texture from the
          YCB Object and Model Set, object 006 mustard bottle (https://www.ycbbenchmarks.com/),
          Calli et al., "The YCB object and model set", 2015.
Licence:  Creative Commons Attribution 4.0 International (CC BY 4.0),
          https://creativecommons.org/licenses/by/4.0/

Changes made by GraspSort (docs/DECISIONS.md D-08):
- model.config rewritten (the original pointed at a missing mustard_bottle.sdf)
- model.sdf rewritten: hard-coded model pose (1 2 3 -1 0 0) removed; Z-up OBJ used instead of
  the Y-up DAE; visual offset so the mesh bounding-box bottom centre is the model origin
- mass 0.000612 -> ${BOTTLE_MASS} kg (YCB full bottle 0.603 kg; lighter value chosen, see the
  script table), inertia = solid box ${BOTTLE_SIZE_X} x ${BOTTLE_SIZE_Y} x ${BOTTLE_SIZE_Z} m
- mesh collision -> box primitive fitted to the mesh bounding box
- textured.mtl: added Ka/Kd/Ks (map_Kd alone renders black in Gazebo Classic)
Original model.config, model.sdf and textured.mtl are kept in orig/. Thumbnails and the DAE
copy are not installed.
EOF
  commit "${name}" "${recipe}"
}

mkdir -p "${DEST}"
install_cricket_ball
install_plastic_cup
install_mustard_bottle
echo "[ok  ] models in ${DEST}"
