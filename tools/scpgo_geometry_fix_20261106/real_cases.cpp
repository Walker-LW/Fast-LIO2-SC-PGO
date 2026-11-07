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
