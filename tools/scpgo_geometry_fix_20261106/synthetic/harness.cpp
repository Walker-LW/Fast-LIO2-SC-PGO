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
Pose6D pose6(const Pose3& p) {
    auto r=p.rotation().rpy(); auto t=p.translation();
    return {t.x(),t.y(),t.z(),r.x(),r.y(),r.z()};
}
Cloud::Ptr makeLocal(const Cloud& world, const Pose3& pose) {
    Cloud::Ptr c(new Cloud);
    for (const auto& p:world) {
        auto q=pose.transformTo(Point3(p.x,p.y,p.z));
        PointType a; a.x=q.x();a.y=q.y();a.z=q.z();a.intensity=p.intensity;c->push_back(a);
    }
    // Same input downsampling used by process_pg before keyframeLaserClouds insertion.
    pcl::VoxelGrid<PointType> v; v.setLeafSize(.4f,.4f,.4f);v.setInputCloud(c);
    Cloud::Ptr d(new Cloud);v.filter(*d);return d;
}
void matrix(std::ostream& f,const Pose3& p) {
    auto m=p.matrix(); f<<"[";for(int i=0;i<16;++i){if(i)f<<",";f<<m(i/4,i%4);}f<<"]";
}
double rmsNN(const Cloud& points,const Cloud& world) {
    double sum=0;
    for(auto& p:points){double best=1e100;for(auto& q:world){
        double d=std::pow(double(p.x)-q.x,2)+std::pow(double(p.y)-q.y,2)+std::pow(double(p.z)-q.z,2);best=std::min(best,d);
    } sum+=best;}return std::sqrt(sum/points.size());
}
int main() {
    std::cout<<std::setprecision(12);
    ros::Time::init();
    Cloud base;
    std::mt19937 rng(20261106);std::uniform_real_distribution<double> u(-6,6);
    while(base.size()<300){PointType p;p.x=u(rng);p.y=u(rng);p.z=u(rng);p.intensity=base.size();
        bool far=true;for(auto& q:base)if((p.getVector3fMap()-q.getVector3fMap()).norm()<1.0f){far=false;break;}
        if(far)base.push_back(p);
    }
    Pose3 I, translation(Rot3(),Point3(.18,-.12,.08));
    Pose3 mixed(Rot3::RzRyRx(.006,-.009,.012),Point3(.18,-.12,.08));
    Pose3 G(Rot3::RzRyRx(.2,-.15,.7),Point3(3,-2,1));
    struct Case { const char* name; Pose3 root,relative;bool multi; };
    std::vector<Case> cases={{"identity",I,I,false},{"translation",I,translation,false},
        {"mixed",I,mixed,false},{"mixed_changed_world",G,mixed,false},
        {"translation_changed_world",G,translation,false},{"three_frame_submap",I,I,true}};
    std::ofstream f("/out/results.json");f<<std::setprecision(12)<<"[\n";
    bool first=true;
    for(auto& c:cases){
        keyframeLaserClouds.clear();keyframePosesUpdated.assign(52,pose6(c.root));
        for(int i=0;i<52;++i)keyframeLaserClouds.emplace_back(new Cloud);
        Cloud world;
        for(auto& p:base){auto q=c.root.transformFrom(Point3(p.x,p.y,p.z));PointType a=p;a.x=q.x();a.y=q.y();a.z=q.z();world.push_back(a);}
        auto current=c.root.compose(c.relative);
        keyframeLaserClouds[1]=makeLocal(world,c.root);
        keyframeLaserClouds[51]=makeLocal(world,current);
        keyframePosesUpdated[51]=pose6(current);
        if(c.multi){
            Pose3 a=c.root.compose(Pose3(Rot3::RzRyRx(.015,-.01,.02),Point3(.65,-.3,.2)));
            Pose3 b=c.root.compose(Pose3(Rot3::RzRyRx(-.02,.01,-.015),Point3(-.55,.25,-.15)));
            keyframeLaserClouds[0]=makeLocal(world,a);keyframePosesUpdated[0]=pose6(a);
            keyframeLaserClouds[2]=makeLocal(world,b);keyframePosesUpdated[2]=pose6(b);
        }
        downSizeFilterICP.setLeafSize(.4f,.4f,.4f);
        std::cout<<"CASE "<<c.name<<std::endl;
        auto measurement=doICPVirtualRelative(1,51);
        Cloud independentlyComposed, originalBeforeVoxel;
        for(int i=0;i<=26;++i) {
            auto p=keyframePosesUpdated[i];Pose3 pose(Rot3::RzRyRx(p.roll,p.pitch,p.yaw),Point3(p.x,p.y,p.z));
            for(auto& q:*keyframeLaserClouds[i]){auto t=pose.transformFrom(Point3(q.x,q.y,q.z));PointType a=q;a.x=t.x();a.y=t.y();a.z=t.z();independentlyComposed.push_back(a);}
            originalBeforeVoxel += *local2global(keyframeLaserClouds[i],keyframePosesUpdated[1]);
        }
        pcl::VoxelGrid<PointType> controlVoxel;controlVoxel.setLeafSize(.4f,.4f,.4f);
        controlVoxel.setInputCloud(independentlyComposed.makeShared());Cloud controlAfterVoxel;controlVoxel.filter(controlAfterVoxel);
        if(!first)f<<",\n";first=false;
        f<<"{\"case\":\""<<c.name<<"\",\"accepted\":"<<(measurement?"true":"false")
          <<",\"source_points\":"<<pubLoopScanLocal.cloud.size()<<",\"target_points\":"<<pubLoopSubmapLocal.cloud.size()
          <<",\"target_world_nn_rms_m\":"<<rmsNN(pubLoopSubmapLocal.cloud,world)
          <<",\"original_before_voxel_nn_rms_m\":"<<rmsNN(originalBeforeVoxel,world)
          <<",\"independent_composition_after_voxel_nn_rms_m\":"<<rmsNN(controlAfterVoxel,world)
          <<",\"independent_composition_after_voxel_points\":"<<controlAfterVoxel.size()
          <<",\"independent_composition_nn_rms_m\":"<<rmsNN(independentlyComposed,world)<<",\"expected\":";
        matrix(f,c.relative);
        if(measurement){
            auto delta=c.relative.between(*measurement);
            BetweenFactor<Pose3> factor(1,51,*measurement,noiseModel::Unit::Create(6));
            auto residual=factor.evaluateError(c.root,current);
            f<<",\"measured\":";matrix(f,*measurement);
            f<<",\"translation_error_m\":"<<delta.translation().norm()
             <<",\"rotation_error_deg\":"<<Rot3::Logmap(delta.rotation()).norm()*180/M_PI
             <<",\"factor_residual\":[";
            for(int i=0;i<6;++i){if(i)f<<",";f<<residual(i);}f<<"]";
        }
        f<<"}";
    }
    f<<"\n]\n";
}
