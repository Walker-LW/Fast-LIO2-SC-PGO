#!/usr/bin/env bash
# Preparation only; never starts a container or replay.
set -euo pipefail
base=/home/lw/slam-research
here=$(cd -- "$(dirname -- "$0")" && pwd)
id=${1:?Usage: bash prepare.sh corridor02-full-b1-YYYYMMDD-NNN}
[[ "$id" =~ ^corridor02-full-b1-[0-9]{8}-[0-9]{3}$ ]] || { echo 'Invalid ID'; exit 1; }
src=$base/fast_lio_ws/src/M3DGR
pkg=$src/baseline_systems/Fast_LIO2_M3DGR/src/fast_lio2
expected_commit=e0cf7d59c9a5a3df515624034698d976abc26549
expected_image=sha256:fcfce4546a744e9e76b02e79485ac60f40cc54522362cc8ad7654871c98c2f7f
[[ "$(git -C "$src" rev-parse HEAD)" == "$expected_commit" ]] || { echo 'Commit changed; review first'; exit 1; }
[[ -z "$(git -C "$src" status --porcelain)" ]] || { echo 'Source tree changed; review first'; exit 1; }
[[ "$(docker image inspect slam-research-fastlio:noetic --format '{{.Id}}')" == "$expected_image" ]] || { echo 'Image changed; review first'; exit 1; }
if docker container inspect "$id-container" >/dev/null 2>&1; then echo 'Container ID already used'; exit 1; fi
run=$base/results/baseline/$id
mkdir -- "$run"
mkdir -- "$run/raw" "$run/derived" "$run/checks" "$run/config" "$run/Log" "$run/PCD" "$run/ros_logs" "$run/screenshots"
exec > >(tee "$run/checks/prepare.log") 2>&1
date -Is
cp -- "$here/run.sh" "$here/verify.py" "$here/watch.py" "$run/"
cp -- "$here/export_pcd.py" "$run/"
cp -- "$here/../corridor02_p0/convert.py" "$run/"
cp -- "$here/view.rviz" "$run/config/"
cp -- "$here/README.md" "$run/config/protocol.md"
cp -- "$here/environment-audit-20261102-003.log" "$run/checks/"
cp -- "$pkg/config/avia.yaml" "$pkg/launch/mapping_avia.launch" "$run/config/"
cp -- "$base/compose.yaml" "$run/config/reference-compose.yaml"
cp -- "$base/datasets/GTCorridor02.txt" "$run/config/"
printf '%s\n' "$expected_image" > "$run/config/image.txt"
git -C "$src" rev-parse HEAD > "$run/config/commit.txt"
git -C "$src" status --short > "$run/config/git-status.txt"
git -C "$src" diff > "$run/config/source.diff"
sha256sum "$base/datasets/Corridor02.bag" > "$run/config/input.sha256"
[[ "$(cut -d ' ' -f 1 "$run/config/input.sha256")" == f6ac8ab3e82fa80bab74f20eaeb437bbe68cb2e2e0f4c90ba25248d724743452 ]] || { echo 'Input changed; retain this directory, review before continuing'; exit 1; }
sha256sum "$base/fast_lio_ws/devel/lib/fast_lio/fastlio_mapping" > "$run/config/executable.sha256"
sha256sum "$pkg/src/laserMapping.cpp" "$pkg/src/preprocess.cpp" > "$run/config/source.sha256"
cat > "$run/config/full.launch" <<'XML'
<launch>
  <param name="use_sim_time" value="true" />
  <include file="$(find fast_lio)/launch/mapping_avia.launch">
    <arg name="rviz" value="false" />
  </include>
  <param name="preprocess/blind" type="double" value="1.0" />
  <param name="publish/path_en" value="true" />
  <param name="pcd_save/pcd_save_en" value="false" />
</launch>
XML
find "$run/config" -maxdepth 1 -type f ! -name prepared-files.sha256 -print0 | sort -z | xargs -0 sha256sum > "$run/config/prepared-files.sha256"
sha256sum "$run/run.sh" "$run/convert.py" "$run/verify.py" "$run/watch.py" > "$run/config/tools.sha256"
sha256sum "$run/export_pcd.py" >> "$run/config/tools.sha256"
printf 'Prepared %s\nNo experiment has started. Retain failed preparations and use a new ID.\n' "$run"
