#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")/.." && pwd)
base_image=slam-research-fastlio:baseline-pre-scpgo-20261105
expected=sha256:fcfce4546a744e9e76b02e79485ac60f40cc54522362cc8ad7654871c98c2f7f
new_image=slam-research-fastlio-scpgo:noetic-gtsam403-v1
verify_baseline() {
  [[ "$(docker image inspect "$base_image" --format '{{.Id}}')" == "$expected" ]] \
    && [[ "$(docker image inspect slam-research-fastlio:noetic --format '{{.Id}}')" == "$expected" ]]
}
finish() {
  build_rc=$?
  trap - EXIT
  if ! verify_baseline; then
    echo 'Baseline image identity verification failed.' >&2
    exit 2
  fi
  exit "$build_rc"
}
trap finish EXIT
[[ "$(docker image inspect "$base_image" --format '{{.Id}}')" == "$expected" ]]
[[ "$(docker image inspect slam-research-fastlio:noetic --format '{{.Id}}')" == "$expected" ]]
if docker image inspect "$new_image" >/dev/null 2>&1; then
  echo "Target image already exists; inspect it before choosing another version." >&2
  exit 1
fi
source_dir=$here/.build-context/gtsam
[[ "$(git -C "$source_dir" rev-parse HEAD)" == 6425000775da29c93719f37dce3f2de38a0064ec ]]
[[ -z "$(git -C "$source_dir" status --porcelain)" ]]
docker build --pull=false --progress=plain \
  --build-arg "BASE_IMAGE=$base_image" \
  --tag "$new_image" --file "$here/Dockerfile.scpgo" "$here"
[[ "$(docker image inspect "$base_image" --format '{{.Id}}')" == "$expected" ]]
[[ "$(docker image inspect slam-research-fastlio:noetic --format '{{.Id}}')" == "$expected" ]]
