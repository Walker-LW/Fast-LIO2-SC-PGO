#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <optional>
#include <mutex>
#include <random>
#include <fstream>
#include <iomanip>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/BackendParameters.h"
using namespace gtsam;
using Cloud = pcl::PointCloud<PointType>;
BackendParameters backendParameters;
std::vector<Cloud::Ptr> keyframeLaserClouds;
std::vector<Pose6D> keyframePosesUpdated;
std::mutex mKF;
pcl::VoxelGrid<PointType> downSizeFilterICP;
struct Capture {
    Cloud cloud;
    void publish(const sensor_msgs::PointCloud2& msg) { pcl::fromROSMsg(msg, cloud); }
};
Capture pubLoopScanLocal, pubLoopSubmapLocal;

#include <pcl/io/pcd_io.h>

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

void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock(); 
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[root_idx]);
        mKF.unlock(); 
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
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
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx 
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx); 

    // loop verification 
    sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopScanLocal.publish(cureKeyframeCloudMsg);

    sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
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
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    return poseFrom.between(poseTo);
}
void matrix(std::ostream& f,const Pose3& p) {
    auto m=p.matrix();f<<"[";for(int i=0;i<16;++i){if(i)f<<",";f<<m(i/4,i%4);}f<<"]";
}
int main(int argc,char** argv) {
    if(argc!=3)return 2;
    std::cout<<std::setprecision(12);ros::Time::init();
    const std::string mode=argv[1],variant=argv[2];
    std::ifstream poses("/run/scpgo/backend/"+mode+"_poses.txt");
    Eigen::Matrix4d m=Eigen::Matrix4d::Identity();
    while(poses>>m(0,0)){
        for(int i=1;i<12;++i)poses>>m(i/4,i%4);
        Pose3 p(m);auto r=p.rotation().rpy();auto t=p.translation();
        keyframePosesUpdated.push_back({t.x(),t.y(),t.z(),r.x(),r.y(),r.z()});
    }
    if(keyframePosesUpdated.size()!=383)return 3;
    pcl::VoxelGrid<PointType> sc;sc.setLeafSize(.4f,.4f,.4f);
    for(size_t i=0;i<keyframePosesUpdated.size();++i){
        std::ostringstream name;name<<"/run/scpgo/backend/Scans/"<<std::setw(6)<<std::setfill('0')<<i<<".pcd";
        Cloud::Ptr raw(new Cloud),ds(new Cloud);
        if(pcl::io::loadPCDFile<PointType>(name.str(),*raw)<0)return 4;
        sc.setInputCloud(raw);sc.filter(*ds);keyframeLaserClouds.push_back(ds);
    }
    downSizeFilterICP.setLeafSize(.4f,.4f,.4f);
    std::ifstream pairs("/fix/real/pairs.txt");int p,c;
    std::ofstream f("/fix/real/"+variant+"-"+mode+".jsonl");f<<std::setprecision(12);
    while(pairs>>p>>c){
        std::cout<<"PAIR "<<p<<" "<<c<<std::endl;
        auto z=doICPVirtualRelative(p,c);
        f<<"{\"previous\":"<<p<<",\"current\":"<<c<<",\"accepted\":"<<(z?"true":"false")
         <<",\"source_points\":"<<pubLoopScanLocal.cloud.size()<<",\"target_points\":"<<pubLoopSubmapLocal.cloud.size();
        if(z){
            const Pose3 a=Pose6DtoGTSAMPose3(keyframePosesUpdated[p]),b=Pose6DtoGTSAMPose3(keyframePosesUpdated[c]);
            auto delta=a.between(b).between(*z);
            f<<",\"measurement\":";matrix(f,*z);
            f<<",\"measurement_translation_norm_m\":"<<z->translation().norm()
             <<",\"disagreement_with_saved_poses_translation_m\":"<<delta.translation().norm()
             <<",\"disagreement_with_saved_poses_rotation_deg\":"<<Rot3::Logmap(delta.rotation()).norm()*180/M_PI;
        }
        f<<"}"<<std::endl;
    }
}
