#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
source /ws/devel_scpgo_geometry/setup.bash
python3 /fix/check_backend.py
