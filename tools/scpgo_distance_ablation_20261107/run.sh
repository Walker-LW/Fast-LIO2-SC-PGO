#!/usr/bin/env bash
set -euo pipefail
python3 /out/prepare.py
g++ -std=c++14 -O1 -g0 -I/usr/include/pcl-1.10 -I/usr/include/eigen3 \
 /out/probe.cpp -o /out/distance_probe \
 -lpcl_registration -lpcl_io -lpcl_common -lpcl_search -lpcl_kdtree -lpcl_filters -lpcl_sample_consensus -lpcl_features -lflann_cpp -llz4 -lpthread \
 > /out/build.log 2>&1
/out/distance_probe > /out/console.log 2>&1
cat /out/console.log
