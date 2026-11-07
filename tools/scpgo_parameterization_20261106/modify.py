from pathlib import Path
import json

root = Path(__file__).resolve().parent
src = root/'source'
out = root/'modified'
out.mkdir(exist_ok=True)
spec = {
 'sc_voxel_size': .4, 'icp_voxel_size': .4,
 'icp_submap_half_frames': 25, 'icp_max_correspondence_distance': 150.,
 'icp_max_iterations': 100, 'icp_transformation_epsilon': 1e-6,
 'icp_fitness_epsilon': 1e-6, 'icp_ransac_iterations': 0,
 'icp_fitness_threshold': .3, 'transform_threads': 8,
 'sc_lidar_height': 2., 'sc_num_rings': 20, 'sc_num_sectors': 60,
 'sc_exclude_recent': 30, 'sc_candidates': 3, 'sc_tree_period': 30,
 'sc_tree_leaf_size': 10, 'sc_search_ratio': .1,
 'isam_relinearize_threshold': .01, 'isam_relinearize_skip': 1,
 'prior_variance': 1e-12, 'odom_rotation_variance': 1e-6,
 'odom_translation_variance': 1e-4, 'loop_variance': .5,
 'loop_cauchy_scale': 1., 'gps_xy_variance': 1e9,
 'gps_z_variance': 250., 'gps_cauchy_scale': 1.,
 'loop_frequency': 1., 'isam_frequency': 1., 'path_frequency': 10.,
 'map_frequency': .1, 'map_skip_frames': 2, 'subscriber_queue_size': 100,
 'publisher_queue_size': 100, 'icp_backlog_warning': 30,
 'worker_sleep_ms': 2, 'pose_log_interval': 100,
}
header = ['#pragma once', '#include <ros/ros.h>', '#include <cmath>', '#include <stdexcept>',
          '// Startup-only configuration. Defaults preserve the imported backend.',
          'struct BackendParameters {']
for name, value in spec.items():
    header.append(f'    {"int" if type(value) is int else "double"} {name} = {repr(value)};')
header.append('    void load(ros::NodeHandle& nh) {')
for name, value in spec.items():
    typ = 'int' if type(value) is int else 'double'
    check = f'{name} >= 0' if name in ('icp_ransac_iterations','icp_submap_half_frames','sc_exclude_recent') else f'{name} > 0'
    if name == 'sc_lidar_height': check = 'true'
    if name == 'sc_search_ratio': check += f' && {name} <= 1'
    header += [f'        nh.param<{typ}>("{name}", {name}, {repr(value)});',
               f'        if (!std::isfinite(double({name})) || !({check})) throw std::invalid_argument("Invalid {name}");',
               f'        ROS_INFO_STREAM("backend parameter {name}=" << {name});']
header += ['    }', '};', '']
(out/'BackendParameters.h').write_text('\n'.join(header))
c = (src/'laserPosegraphOptimization.cpp').read_text()
def replace(a,b):
    global c
    assert a in c, a
    c = c.replace(a,b)
replace('#include "scancontext/Scancontext.h"', '#include "scancontext/Scancontext.h"\n#include "aloam_velodyne/BackendParameters.h"\nBackendParameters backendParameters;')
replacements = {
 'priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;': 'priorNoiseVector6.setConstant(backendParameters.prior_variance);',
 'odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;': 'odomNoiseVector6 << backendParameters.odom_rotation_variance, backendParameters.odom_rotation_variance, backendParameters.odom_rotation_variance, backendParameters.odom_translation_variance, backendParameters.odom_translation_variance, backendParameters.odom_translation_variance;',
 'double loopNoiseScore = 0.5;': 'double loopNoiseScore = backendParameters.loop_variance;',
 'double bigNoiseTolerentToXY = 1000000000.0;': 'double bigNoiseTolerentToXY = backendParameters.gps_xy_variance;',
 'double gpsAltitudeNoiseScore = 250.0;': 'double gpsAltitudeNoiseScore = backendParameters.gps_z_variance;',
 'int numberOfCores = 8;': 'int numberOfCores = backendParameters.transform_threads;',
 'int historyKeyframeSearchNum = 25;': 'int historyKeyframeSearchNum = backendParameters.icp_submap_half_frames;',
 'icp.setMaxCorrespondenceDistance(150)': 'icp.setMaxCorrespondenceDistance(backendParameters.icp_max_correspondence_distance)',
 'icp.setMaximumIterations(100)': 'icp.setMaximumIterations(backendParameters.icp_max_iterations)',
 'icp.setTransformationEpsilon(1e-6)': 'icp.setTransformationEpsilon(backendParameters.icp_transformation_epsilon)',
 'icp.setEuclideanFitnessEpsilon(1e-6)': 'icp.setEuclideanFitnessEpsilon(backendParameters.icp_fitness_epsilon)',
 'icp.setRANSACIterations(0)': 'icp.setRANSACIterations(backendParameters.icp_ransac_iterations)',
 'float loopFitnessScoreThreshold = 0.3;': 'float loopFitnessScoreThreshold = backendParameters.icp_fitness_threshold;',
 'float loopClosureFrequency = 1.0;': 'float loopClosureFrequency = backendParameters.loop_frequency;',
 'float hz = 10.0;': 'float hz = backendParameters.path_frequency;',
 'float hz = 1;': 'float hz = backendParameters.isam_frequency;',
 'float vizmapFrequency = 0.1;': 'float vizmapFrequency = backendParameters.map_frequency;',
 'int SKIP_FRAMES = 2;': 'int SKIP_FRAMES = backendParameters.map_skip_frames;',
 'scLoopICPBuf.size() > 30': 'scLoopICPBuf.size() > size_t(backendParameters.icp_backlog_warning)',
 'std::chrono::milliseconds dura(2)': 'std::chrono::milliseconds dura(backendParameters.worker_sleep_ms)',
 'curr_node_idx % 100': 'curr_node_idx % backendParameters.pose_log_interval',
 'parameters.relinearizeThreshold = 0.01': 'parameters.relinearizeThreshold = backendParameters.isam_relinearize_threshold',
 'parameters.relinearizeSkip = 1': 'parameters.relinearizeSkip = backendParameters.isam_relinearize_skip',
 'float filter_size = 0.4;': '// Independent filter scales, preserving the original defaults.',
 'setLeafSize(filter_size, filter_size, filter_size)': 'setLeafSize(backendParameters.sc_voxel_size, backendParameters.sc_voxel_size, backendParameters.sc_voxel_size)',
}
for a,b in replacements.items(): replace(a,b)
replace('downSizeFilterICP.setLeafSize(backendParameters.sc_voxel_size, backendParameters.sc_voxel_size, backendParameters.sc_voxel_size)', 'downSizeFilterICP.setLeafSize(backendParameters.icp_voxel_size, backendParameters.icp_voxel_size, backendParameters.icp_voxel_size)')
assert c.count('mEstimator::Cauchy::Create(1)') == 2
c = c.replace('mEstimator::Cauchy::Create(1)', 'mEstimator::Cauchy::Create(backendParameters.loop_cauchy_scale)',1).replace('mEstimator::Cauchy::Create(1)', 'mEstimator::Cauchy::Create(backendParameters.gps_cauchy_scale)',1)
c = c.replace(', 100, laser', ', backendParameters.subscriber_queue_size, laser').replace(', 100, gpsHandler', ', backendParameters.subscriber_queue_size, gpsHandler')
import re
c = re.sub(r'(nh.advertise<[^\n]+>\("[^\"]+", )100(\))', r'\1backendParameters.publisher_queue_size\2', c)
# Read and validate everything before the upstream output-directory cleanup.
marker = '\tnh.param<std::string>("save_directory"'
pos = c.index(marker)
startup = '''    try {
        backendParameters.load(nh);
        double meter, degree, threshold, radius, viz;
        nh.param<double>("keyframe_meter_gap", meter, 2.0);
        nh.param<double>("keyframe_deg_gap", degree, 10.0);
        nh.param<double>("sc_dist_thres", threshold, 0.2);
        nh.param<double>("sc_max_radius", radius, 80.0);
        nh.param<double>("mapviz_filter_size", viz, 0.4);
        if (!std::isfinite(meter) || meter <= 0 || !std::isfinite(degree) || degree <= 0 ||
            !std::isfinite(threshold) || threshold <= 0 || threshold > 2 ||
            !std::isfinite(radius) || radius <= 0 || !std::isfinite(viz) || viz <= 0)
            throw std::invalid_argument("Invalid keyframe/SC/map visualization parameter");
        scManager.configure(backendParameters.sc_lidar_height, backendParameters.sc_num_rings,
            backendParameters.sc_num_sectors, backendParameters.sc_exclude_recent,
            backendParameters.sc_candidates, backendParameters.sc_tree_period,
            backendParameters.sc_search_ratio, backendParameters.sc_tree_leaf_size);
        scManager.setMaximumRadius(radius);
        scManager.setSCdistThres(threshold);
        ROS_INFO_STREAM("effective profile: keyframe_meter_gap=" << meter << " keyframe_deg_gap=" << degree
            << " sc_dist_thres=" << threshold << " sc_max_radius=" << radius << " mapviz_filter_size=" << viz);
        bool validateOnly = false;
        nh.param<bool>("validate_config_only", validateOnly, false);
        if (validateOnly) { ROS_INFO("Configuration valid; no output files or worker threads created."); return 0; }
    } catch (const std::exception& e) { ROS_FATAL_STREAM(e.what()); return 2; }

'''
c = c[:pos] + startup + c[pos:]
(out/'laserPosegraphOptimization.cpp').write_text(c)
h = (src/'Scancontext.h').read_text()
for field in ('LIDAR_HEIGHT', 'PC_NUM_RING', 'PC_NUM_SECTOR', 'PC_UNIT_SECTORANGLE', 'PC_UNIT_RINGGAP', 'NUM_EXCLUDE_RECENT', 'NUM_CANDIDATES_FROM_TREE', 'SEARCH_RATIO', 'TREE_MAKING_PERIOD_'):
    h = re.sub(r'const (double|int)(\s+'+field+r'\b)', r'\1\2', h)
h = h.replace('    // setter', '''    // Startup-only; configure before storing any descriptors.
    int TREE_LEAF_SIZE = 10;
    void configure(double height, int rings, int sectors, int exclude, int candidates,
                   int period, double ratio, int leaf) {
        if (!polarcontexts_.empty()) throw std::logic_error("SC configuration is startup-only");
        LIDAR_HEIGHT = height; PC_NUM_RING = rings; PC_NUM_SECTOR = sectors;
        NUM_EXCLUDE_RECENT = exclude; NUM_CANDIDATES_FROM_TREE = candidates;
        TREE_MAKING_PERIOD_ = period; SEARCH_RATIO = ratio; TREE_LEAF_SIZE = leaf;
        PC_UNIT_SECTORANGLE = 360.0 / PC_NUM_SECTOR;
        PC_UNIT_RINGGAP = PC_MAX_RADIUS / PC_NUM_RING;
    }
    // setter''')
h = h.replace('#include <ctime>', '#include <ctime>\n#include <stdexcept>')
(out/'Scancontext.h').write_text(h)
sc = (src/'Scancontext.cpp').read_text().replace('    PC_MAX_RADIUS = _max_r;', '    PC_MAX_RADIUS = _max_r;\n    PC_UNIT_RINGGAP = PC_MAX_RADIUS / PC_NUM_RING;')
sc = sc.replace('10 /* max leaf */', 'TREE_LEAF_SIZE /* max leaf */')
(out/'Scancontext.cpp').write_text(sc)
profile = '# User-approved Corridor01 profile. Other settings retain imported defaults.\nkeyframe_meter_gap: 1.0\nkeyframe_deg_gap: 10.0\nsc_dist_thres: 0.2\nsc_max_radius: 40.0\nmapviz_filter_size: 0.2\n'
profile += '\n# Startup-only parameters; no additional tuning approved.\n'
profile += ''.join(f'{name}: {repr(value)}\n' for name,value in spec.items())
(root/'corridor01-indoor.yaml').write_text(profile)
(root/'parameter-defaults.json').write_text(json.dumps(spec,indent=2)+'\n')
for name in ('laserPosegraphOptimization.cpp','Scancontext.h','Scancontext.cpp'):
    if b'\r\n' in (src/name).read_bytes():
        (out/name).write_bytes((out/name).read_text().replace('\n','\r\n').encode())
print('Generated',len(spec),'parameters and approved profile')
