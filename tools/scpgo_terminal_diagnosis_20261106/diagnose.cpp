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
