#!/usr/bin/env bash
set -euo pipefail
g++ -std=c++14 -O1 -g0 -I/usr/include/pcl-1.10 -I/usr/include/eigen3 \
  /tools/icp_evidence_probe.cpp -o /out/icp_evidence_probe \
  -lpcl_registration -lpcl_io -lpcl_common -lpcl_search -lpcl_kdtree \
  -lpcl_filters -lpcl_sample_consensus -lpcl_features -lflann_cpp -llz4 -lpthread \
  > /out/probe-build-003.log 2>&1
