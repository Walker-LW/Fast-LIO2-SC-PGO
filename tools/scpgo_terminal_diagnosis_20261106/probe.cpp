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
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "scancontext/Scancontext.h"
#include <algorithm>

void matrix(std::ostream& f,const Eigen::Matrix4f& m){f<<"[";for(int i=0;i<16;++i){if(i)f<<",";f<<m(i/4,i%4);}f<<"]";}
void distances(std::ostream& f,const Cloud::Ptr& source,const Cloud::Ptr& target,const Eigen::Matrix4f& transform){
    Cloud aligned;pcl::transformPointCloud(*source,aligned,transform);
    pcl::KdTreeFLANN<PointType> tree;tree.setInputCloud(target);
    std::vector<int> ids(1);std::vector<float> ds(1);
    f<<"[";
    for(size_t i=0;i<aligned.size();++i){tree.nearestKSearch(aligned[i],1,ids,ds);if(i)f<<",";f<<std::sqrt(ds[0]);}
    f<<"]";
}
int main(){
    std::cout<<std::setprecision(12);ros::Time::init();
    SCManager sc;sc.setMaximumRadius(40.0);
    Cloud signPrevious,signCurrent;
    std::mt19937 rng(20261106);std::uniform_real_distribution<float> height(.1f,4.f);
    for(int r=0;r<20;++r)for(int s=0;s<60;++s){
        PointType p;float radius=(r+.5f)*2.f,angle=(s+.5f)*6.f*M_PI/180.;p.x=radius*cos(angle);p.y=radius*sin(angle);p.z=height(rng);p.intensity=0;signPrevious.push_back(p);
    }
    Eigen::Matrix4f known=pcl::getTransformation(0,0,0,0,0,float(M_PI/6)).matrix();
    pcl::transformPointCloud(signPrevious,signCurrent,known.inverse());
    auto sp=sc.makeScancontext(signPrevious),sn=sc.makeScancontext(signCurrent);auto sign=sc.distanceBtnScanContext(sn,sp);
    Eigen::Matrix4f recovered=pcl::getTransformation(0,0,0,0,0,float(-sign.second*6*M_PI/180.)).matrix();
    std::ofstream sf("/out/yaw-sign-test.json");sf<<std::setprecision(12)<<"{\"known_pose_yaw_deg\":30,\"returned_shift_deg\":"<<sign.second*6<<",\"negative_yaw_matrix_error\":"<<(known-recovered).norm()<<",\"score\":"<<sign.first<<"}";
    if((known-recovered).norm()>1e-4)return 10;
    std::ifstream poses("/run/scpgo/backend/odom_poses.txt");Eigen::Matrix4d m=Eigen::Matrix4d::Identity();
    while(poses>>m(0,0)){for(int i=1;i<12;++i)poses>>m(i/4,i%4);Pose3 p(m);auto r=p.rotation().rpy();auto t=p.translation();keyframePosesUpdated.push_back({t.x(),t.y(),t.z(),r.x(),r.y(),r.z()});}
    if(keyframePosesUpdated.size()!=383)return 3;
    std::vector<Cloud::Ptr> raw;
    pcl::VoxelGrid<PointType> voxel;voxel.setLeafSize(.4f,.4f,.4f);
    for(int i=0;i<383;++i){std::ostringstream name;name<<"/run/scpgo/backend/Scans/"<<std::setw(6)<<std::setfill('0')<<i<<".pcd";
        Cloud::Ptr a(new Cloud),b(new Cloud);if(pcl::io::loadPCDFile(name.str(),*a)<0)return 4;raw.push_back(a);voxel.setInputCloud(a);voxel.filter(*b);keyframeLaserClouds.push_back(b);
    }
    downSizeFilterICP.setLeafSize(.4f,.4f,.4f);
    std::ifstream pairfile("/out/pairs.txt");std::ofstream f("/out/diagnostics.jsonl");f<<std::setprecision(12);int p,c;
    while(pairfile>>p>>c){
        std::string tag=std::to_string(p)+"-"+std::to_string(c);std::cout<<"PAIR "<<tag<<std::endl;
        Cloud::Ptr source(new Cloud),target(new Cloud),previous(new Cloud);
        loopFindNearKeyframesCloud(source,c,0,c);loopFindNearKeyframesCloud(target,p,25,p);loopFindNearKeyframesCloud(previous,p,0,p);
        pcl::io::savePCDFileBinary("/out/"+tag+"-source.pcd",*source);pcl::io::savePCDFileBinary("/out/"+tag+"-target.pcd",*target);
        pcl::io::savePCDFileBinary("/out/"+tag+"-previous.pcd",*previous);
        pcl::io::savePCDFileBinary("/out/"+tag+"-raw-current.pcd",*raw[c]);pcl::io::savePCDFileBinary("/out/"+tag+"-raw-previous.pcd",*raw[p]);
        auto currdesc=sc.makeScancontext(*keyframeLaserClouds[c]),prevdesc=sc.makeScancontext(*keyframeLaserClouds[p]);
        auto result=sc.distanceBtnScanContext(currdesc,prevdesc);
        const Pose3 odom=Pose6DtoGTSAMPose3(keyframePosesUpdated[p]).between(Pose6DtoGTSAMPose3(keyframePosesUpdated[c]));
        Eigen::Matrix4f odo=odom.matrix().cast<float>(),rot=odo;rot.block<3,1>(0,3).setZero();
        Eigen::Matrix4f syaw=pcl::getTransformation(0,0,0,0,0,float(-result.second*6*M_PI/180.)).matrix();
        std::vector<std::pair<std::string,Eigen::Matrix4f>> initials={{"identity",Eigen::Matrix4f::Identity()},{"sc_yaw",syaw},{"odom_rotation",rot},{"odom_full",odo}};
        f<<"{\"previous\":"<<p<<",\"current\":"<<c<<",\"source_points\":"<<source->size()<<",\"target_points\":"<<target->size()<<",\"sc_score\":"<<result.first<<",\"sc_returned_yaw_deg\":"<<result.second*6<<",\"odom_yaw_deg\":"<<odom.rotation().yaw()*180/M_PI<<",\"odom_translation_m\":"<<odom.translation().norm()<<",\"sc_distance_by_shift\":[";
        for(int shift=0;shift<60;++shift){auto shifted=circshift(prevdesc,shift);double d=sc.distDirectSC(currdesc,shifted);if(shift)f<<",";if(std::isfinite(d))f<<d;else f<<"null";}f<<"],\"source_ranges\":[";
        for(size_t i=0;i<source->size();++i){if(i)f<<",";f<<(*source)[i].getVector3fMap().norm();}f<<"],\"initials\":[";
        bool first=true;
        for(auto& v:initials){
            pcl::IterativeClosestPoint<PointType,PointType> icp;
            icp.setMaxCorrespondenceDistance(150);icp.setMaximumIterations(100);icp.setTransformationEpsilon(1e-6);icp.setEuclideanFitnessEpsilon(1e-6);icp.setRANSACIterations(0);
            icp.setInputSource(source);icp.setInputTarget(target);Cloud aligned;icp.align(aligned,v.second);
            auto final=icp.getFinalTransformation();double fitness=icp.getFitnessScore();
            pcl::io::savePCDFileBinary("/out/"+tag+"-"+v.first+"-aligned.pcd",aligned);
            if(!first)f<<",";first=false;
            f<<"{\"name\":\""<<v.first<<"\",\"converged\":"<<(icp.hasConverged()?"true":"false")<<",\"fitness_m2\":"<<fitness<<",\"initial_matrix\":";matrix(f,v.second);
            f<<",\"final_matrix\":";matrix(f,final);
            f<<",\"initial_nn_distances_m\":";distances(f,source,target,v.second);
            f<<",\"final_nn_distances_m\":";distances(f,source,target,final);
            f<<",\"final_nn_to_single_previous_m\":";distances(f,source,previous,final);
            f<<"}";
        }
        f<<"]}"<<std::endl;
    }
}
