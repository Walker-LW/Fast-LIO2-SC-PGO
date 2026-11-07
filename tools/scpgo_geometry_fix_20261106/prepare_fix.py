from pathlib import Path
import difflib, hashlib, json

out=Path(__file__).resolve().parent
baseline=out.parent/'scpgo_parameterization_20261106/modified/laserPosegraphOptimization.cpp'
raw=baseline.read_bytes()
assert hashlib.sha256(raw).hexdigest()=='1bd79b0f80fa5432b0a1c7262ac2a3e1cef8f06c138b5a9bc84dc4aabd066e8f'
old=raw.decode().replace('\r\n','\n')
start=old.index('void loopFindNearKeyframesCloud(')
end=old.index('} // loopFindNearKeyframesCloud',start)+len('} // loopFindNearKeyframesCloud')
replacement='''void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx, Pose6D* root_pose = nullptr)
{
    // Each stored scan is in its own body frame. Snapshot poses together so
    // all members of this submap use the same optimization state.
    nearKeyframes->clear();
    Pose6D root;
    std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, Pose6D>> frames;
    {
        std::lock_guard<std::mutex> lock(mKF);
        root = keyframePosesUpdated.at(root_idx);
        if (root_pose) *root_pose = root;
        for (int i = -submap_size; i <= submap_size; ++i) {
            const int keyNear = key + i;
            if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size())) continue;
            frames.emplace_back(keyframeLaserClouds[keyNear], keyframePosesUpdated.at(keyNear));
        }
    }
    const gtsam::Pose3 worldFromRoot = Pose6DtoGTSAMPose3(root);
    for (const auto& frame : frames) {
        const gtsam::Pose3 rootFromFrame = worldFromRoot.between(Pose6DtoGTSAMPose3(frame.second));
        const auto t = rootFromFrame.translation();
        const auto rpy = rootFromFrame.rotation().rpy();
        const Pose6D relative {t.x(), t.y(), t.z(), rpy.x(), rpy.y(), rpy.z()};
        *nearKeyframes += *local2global(frame.first, relative);
    }
    if (nearKeyframes->empty()) return;
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud'''
new=old[:start]+replacement+old[end:]
needle='''    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx 
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx); '''
replace='''    Pose6D sourceRoot, targetRoot;
    // ICP source: current body frame; target: historical root body frame.
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _curr_kf_idx, &sourceRoot);
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx, &targetRoot);'''
assert new.count(needle)==1
new=new.replace(needle,replace)
new=new.replace('    // loop verification \n','    // Publish world-coordinate copies for inspection; ICP inputs stay local.\n')
new=new.replace('pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);','pcl::toROSMsg(*local2global(cureKeyframeCloud, sourceRoot), cureKeyframeCloudMsg);')
new=new.replace('pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);','pcl::toROSMsg(*local2global(targetKeyframeCloud, targetRoot), targetKeyframeCloudMsg);')
needle='''    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    return poseFrom.between(poseTo);'''
assert new.count(needle)==1
new=new.replace(needle,'''    // ICP maps current-body points into historical-body coordinates:
    // Z = inverse(T_world_previous) * T_world_current, as BetweenFactor(p,c) expects.
    return Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));''')
(out/'laserPosegraphOptimization.cpp').write_bytes(new.replace('\n','\r\n').encode())
(out/'geometry-fix.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='before/laserPosegraphOptimization.cpp',tofile='after/laserPosegraphOptimization.cpp')))
(out/'change-manifest.json').write_text(json.dumps({'baseline_sha256':hashlib.sha256(raw).hexdigest(),'candidate_sha256':hashlib.sha256((out/'laserPosegraphOptimization.cpp').read_bytes()).hexdigest(),'scope':['historical-root local submap using per-frame poses','current-local source and direct relative measurement','world-coordinate debug copies with corresponding root snapshots'],'numeric_parameters_changed':False},indent=2)+'\n')

# Reuse the previous six fixed cases, with the target visualization still in world coordinates.
prior=out.parent/'scpgo_geometry_validation_20261106'
test=out/'synthetic';test.mkdir(exist_ok=True)
(test/'prefix.cpp').write_bytes((prior/'prefix.cpp').read_bytes())
cases=(prior/'cases.cpp').read_text()
# The old recovery expression diagnoses the old bug; it is not a valid metric after repair.
a=cases.index('            // Diagnostic only:')
b=cases.index('            f<<',a)
cases=cases[:a]+cases[b:]
cases=cases.replace('f<<"]"\n             <<",\\\"diagnostic_recovery_translation_error_m\\\":"<<recoveryError.translation().norm()\n             <<",\\\"diagnostic_recovery_rotation_error_deg\\\":"<<Rot3::Logmap(recoveryError.rotation()).norm()*180/M_PI;', 'f<<"]";')
assert 'recoveryError' not in cases
(test/'cases.cpp').write_text(cases)
