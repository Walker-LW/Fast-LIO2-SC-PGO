// Standalone diagnostic; never modifies or links the original SLAM program.
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
int main(int argc,char** argv) {
  if(argc!=3) return 2;
  using Point=pcl::PointXYZI;
  pcl::PointCloud<Point>::Ptr source(new pcl::PointCloud<Point>),target(new pcl::PointCloud<Point>);
  if(pcl::io::loadPCDFile<Point>(argv[1],*source)<0 || pcl::io::loadPCDFile<Point>(argv[2],*target)<0) return 3;
  pcl::IterativeClosestPoint<Point,Point> icp;
  icp.setMaxCorrespondenceDistance(150);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  icp.setInputSource(source); icp.setInputTarget(target);
  pcl::PointCloud<Point> unused; icp.align(unused);
  Eigen::Matrix4f transform=icp.getFinalTransformation();
  Eigen::Matrix4f measurement=transform.inverse();
  std::cout<<std::setprecision(12)<<"{\"converged\":"<<(icp.hasConverged()?"true":"false")
    <<",\"fitness\":"<<icp.getFitnessScore()<<",\"source_points\":"<<source->size()
    <<",\"target_points\":"<<target->size()<<",\"icp_transform\":[";
  for(int i=0;i<16;i++) { if(i) std::cout<<","; std::cout<<transform(i/4,i%4); }
  std::cout<<"],\"original_backend_between_measurement\":[";
  for(int i=0;i<16;i++) { if(i) std::cout<<","; std::cout<<measurement(i/4,i%4); }
  std::cout<<"]}"<<std::endl;
}
