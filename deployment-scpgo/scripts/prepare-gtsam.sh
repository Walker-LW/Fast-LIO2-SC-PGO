#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")/.." && pwd)
source_dir=$here/.build-context/gtsam
expected=6425000775da29c93719f37dce3f2de38a0064ec
mkdir -p -- "$here/.build-context"
if [[ ! -e "$source_dir" ]]; then
  git clone --depth 1 --branch 4.0.3 https://github.com/borglab/gtsam.git "$source_dir"
fi
[[ "$(git -C "$source_dir" rev-parse HEAD)" == "$expected" ]]
[[ -z "$(git -C "$source_dir" status --porcelain)" ]]
printf 'Verified GTSAM source: %s\n' "$expected"
