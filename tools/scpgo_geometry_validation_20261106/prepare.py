"""Extract current functions verbatim; production source is read-only."""
from pathlib import Path
import hashlib, json

src = Path('/source')
out = Path('/out')
text = (src/'src/laserPosegraphOptimization.cpp').read_text()
def extract(signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]

names = ['pcl::PointCloud<PointType>::Ptr local2global(',
         'void loopFindNearKeyframesCloud(',
         'std::optional<gtsam::Pose3> doICPVirtualRelative(']
functions = '\n\n'.join(extract(n) for n in names)
(out/'extracted-functions.cpp').write_text(functions)
(out/'harness.cpp').write_text((out/'prefix.cpp').read_text()+'\n'+functions+'\n'+(out/'cases.cpp').read_text())
manifest = {
    'question': 'Do unchanged backend geometry functions preserve known relative poses and align multi-frame local scans?',
    'scope': 'Deterministic synthetic correctness tests, not Corridor01 accuracy evaluation.',
    'source_sha256': hashlib.sha256((src/'src/laserPosegraphOptimization.cpp').read_bytes()).hexdigest(),
    'extracted_functions_sha256': hashlib.sha256(functions.encode()).hexdigest(),
    'functions': names,
    'adaptations': 'ROS publishers replaced by captures; no ROS master, workers or optimizer. Extracted functions unchanged.',
    'parameters': 'BackendParameters defaults unchanged, ICP voxel 0.4 m, half-submap 25. SC input voxel 0.4 m.',
    'design': '300 asymmetric separated 3D points, fixed seed 20261106, full overlap, noiseless. 52 slots, current=51, previous=1. Empty unused slots isolate single/multi-frame behavior without changing half-submap.',
    'cases': ['identity', 'translation', 'mixed', 'mixed_changed_world', 'translation_changed_world', 'three_frame_submap'],
    'metrics': 'relative measurement translation error m, rotation error deg; actual GTSAM unwhitened residual; target-to-known-world NN RMS m',
    'pass_limits': {'translation_m': 0.001, 'rotation_deg': 0.01, 'submap_nn_rms_m': 0.001},
    'limits_rationale': 'Noiseless well-separated points with no intra-scan voxel merging; tolerances exceed float roundoff and remain far below imposed motions.',
    'stopping': 'Run all six predefined cases once; record all results including rejected ICP. No parameter sweep.',
}
(out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
