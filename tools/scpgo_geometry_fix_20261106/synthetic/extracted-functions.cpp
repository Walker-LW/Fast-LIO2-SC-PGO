gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
}

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx, Pose6D* root_pose = nullptr)
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
}

std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    int historyKeyframeSearchNum = backendParameters.icp_submap_half_frames; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    Pose6D sourceRoot, targetRoot;
    // ICP source: current body frame; target: historical root body frame.
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _curr_kf_idx, &sourceRoot);
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx, &targetRoot);

    // Publish world-coordinate copies for inspection; ICP inputs stay local.
    sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*local2global(cureKeyframeCloud, sourceRoot), cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopScanLocal.publish(cureKeyframeCloudMsg);

    sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*local2global(targetKeyframeCloud, targetRoot), targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(backendParameters.icp_max_correspondence_distance); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter 
    icp.setMaximumIterations(backendParameters.icp_max_iterations);
    icp.setTransformationEpsilon(backendParameters.icp_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(backendParameters.icp_fitness_epsilon);
    icp.setRANSACIterations(backendParameters.icp_ransac_iterations);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
 
    float loopFitnessScoreThreshold = backendParameters.icp_fitness_threshold; // user parameter but fixed low value is safe. 
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    } else {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    // ICP maps current-body points into historical-body coordinates:
    // Z = inverse(T_world_previous) * T_world_current, as BetweenFactor(p,c) expects.
    return Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
}