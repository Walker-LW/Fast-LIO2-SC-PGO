#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
for setup in /ws/devel/setup.bash /ws/devel_scpgo/setup.bash; do
  if [[ ! -f "$setup" ]]; then
    printf 'Required workspace environment missing: %s\n' "$setup" >&2
    exit 2
  fi
  source "$setup"
done
exec "$@"
