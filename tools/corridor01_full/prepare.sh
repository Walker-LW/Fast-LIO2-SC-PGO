#!/usr/bin/env bash
# Preparation only; never starts a container or replay.
set -euo pipefail
base=/home/lw/slam-research
here=$(cd -- "$(dirname -- "$0")" && pwd)
id=${1:?Usage: bash prepare.sh corridor01-full-b1-YYYYMMDD-NNN or corridor01-full-b4-YYYYMMDD-NNN}
online_pcd=false
case ${2:-} in
  '') ;;
  --online-pcd) online_pcd=true ;;
  *) echo 'Unknown option; use --online-pcd or no option'; exit 1 ;;
esac
[[ "$id" =~ ^corridor01-full-b([14])-[0-9]{8}-[0-9]{3}$ ]] || { echo 'Invalid ID'; exit 1; }
blind=${BASH_REMATCH[1]}
if [[ "$blind" == 4 && "$online_pcd" != true ]]; then
  echo 'The planned Corridor01 blind=4 comparison requires --online-pcd to match blind=1'; exit 1
fi
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
cp -- "$here/export_pcd.py" "$here/downsample_pcd.py" "$here/trajectory.py" "$here/ready.py" "$here/end_status.py" "$run/"
cp -- "$here/compare_pcd.py" "$run/"
cp -- "$here/../corridor02_p0/convert.py" "$run/"
cp -- "$here/view.rviz" "$run/config/"
cp -- "$here/README.md" "$run/config/protocol.md"
cp -- "$here/input-audit-20261103-001.json" "$run/checks/"
cp -- "$pkg/config/avia.yaml" "$pkg/launch/mapping_avia.launch" "$run/config/"
cp -- "$base/compose.yaml" "$run/config/reference-compose.yaml"
cp -- "$base/datasets/GTCorridor01.txt" "$run/config/"
printf '{"online_pcd":%s,"pcd_interval":-1,"blind":%s}\n' "$online_pcd" "$blind" > "$run/config/output-policy.json"
if [[ "$blind" == 4 ]]; then
  cp -- "$base/results/baseline/corridor01-full-b1-20261103-001/config/runtime.yaml" "$run/config/reference-blind1-runtime.yaml"
  cp -- "$here/BLIND4.md" "$run/config/protocol-blind4.md"
fi
printf '%s\n' "$expected_image" > "$run/config/image.txt"
git -C "$src" rev-parse HEAD > "$run/config/commit.txt"
git -C "$src" status --short > "$run/config/git-status.txt"
git -C "$src" diff > "$run/config/source.diff"
sha256sum "$base/datasets/Corridor01.bag" > "$run/config/input.sha256"
[[ "$(cut -d ' ' -f 1 "$run/config/input.sha256")" == "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["sha256"])' "$here/input-audit-20261103-001.json")" ]] || { echo 'Input changed; retain this directory, review before continuing'; exit 1; }
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
sed -i "s@name=\"preprocess/blind\" type=\"double\" value=\"1.0\"@name=\"preprocess/blind\" type=\"double\" value=\"$blind.0\"@" "$run/config/full.launch"
if [[ "$online_pcd" == true ]]; then
  sed -i 's@name="pcd_save/pcd_save_en" value="false"@name="pcd_save/pcd_save_en" value="true"@' "$run/config/full.launch"
fi
find "$run/config" -maxdepth 1 -type f ! -name prepared-files.sha256 -print0 | sort -z | xargs -0 sha256sum > "$run/config/prepared-files.sha256"
sha256sum "$run/run.sh" "$run/convert.py" "$run/verify.py" "$run/watch.py" > "$run/config/tools.sha256"
sha256sum "$run/export_pcd.py" >> "$run/config/tools.sha256"
sha256sum "$run/downsample_pcd.py" "$run/trajectory.py" "$run/ready.py" "$run/end_status.py" >> "$run/config/tools.sha256"
sha256sum "$run/compare_pcd.py" >> "$run/config/tools.sha256"
printf 'Prepared %s\nNo experiment has started. Retain failed preparations and use a new ID.\n' "$run"
