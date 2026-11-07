#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
set -u
python3 /fix/prepare_test.py
g++ -std=c++17 -O1 -g0 -fopenmp -I/source/include -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -I/opt/ros/noetic/include -I/usr/local/include \
 /out/harness.cpp -o /out/geometry_probe -L/opt/ros/noetic/lib -L/usr/local/lib \
 -Wl,-rpath,/usr/local/lib -lgtsam -lpcl_registration -lpcl_common -lpcl_search -lpcl_kdtree -lpcl_filters -lpcl_sample_consensus -lpcl_features -lflann_cpp -llz4 -lroscpp -lroscpp_serialization -lrostime -lcpp_common -lpthread \
 > /out/build.log 2>&1
/out/geometry_probe > /out/console.log 2>&1
cat /out/console.log
