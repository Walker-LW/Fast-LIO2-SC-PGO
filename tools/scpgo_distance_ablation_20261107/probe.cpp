#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
using Point=pcl::PointXYZI;
using Cloud=pcl::PointCloud<Point>;
class DiagnosticICP:public pcl::IterativeClosestPoint<Point,Point>{
 public:int iterations()const{return nr_iterations_;}
};
int main(){
 std::ifstream pairs("/out/pairs.txt");std::ofstream f("/out/results.jsonl");f<<std::setprecision(12);int p,c;
 while(pairs>>p>>c){
  std::string tag=std::to_string(p)+"-"+std::to_string(c);
  Cloud::Ptr source(new Cloud),target(new Cloud);
  if(pcl::io::loadPCDFile<Point>("/previous/"+tag+"-source.pcd",*source)<0 || pcl::io::loadPCDFile<Point>("/previous/"+tag+"-target.pcd",*target)<0)return 2;
  pcl::KdTreeFLANN<Point> targetTree;targetTree.setInputCloud(target);
  for(double gate:{150.,5.,2.,1.}){
   DiagnosticICP icp;icp.setMaxCorrespondenceDistance(gate);icp.setMaximumIterations(100);icp.setTransformationEpsilon(1e-6);icp.setEuclideanFitnessEpsilon(1e-6);icp.setRANSACIterations(0);
   icp.setInputSource(source);icp.setInputTarget(target);Cloud::Ptr aligned(new Cloud);icp.align(*aligned);
   auto transform=icp.getFinalTransformation();double score=icp.getFitnessScore();
   if(!transform.allFinite() || !std::isfinite(score))return 3;
   pcl::io::savePCDFileBinary("/out/"+tag+"-gate"+std::to_string(int(gate))+"-aligned.pcd",*aligned);
   pcl::KdTreeFLANN<Point> sourceTree;sourceTree.setInputCloud(aligned);
   std::vector<int> ids(1),reverse(1),targets,mutual;
   std::vector<float> dist(1),revDist(1),distances;
   for(size_t i=0;i<aligned->size();++i){
    targetTree.nearestKSearch((*aligned)[i],1,ids,dist);targets.push_back(ids[0]);distances.push_back(std::sqrt(dist[0]));
    sourceTree.nearestKSearch((*target)[ids[0]],1,reverse,revDist);mutual.push_back(reverse[0]==int(i));
   }
   f<<"{\"previous\":"<<p<<",\"current\":"<<c<<",\"gate_m\":"<<gate<<",\"source_points\":"<<source->size()<<",\"target_points\":"<<target->size()
    <<",\"converged\":"<<(icp.hasConverged()?"true":"false")<<",\"iterations\":"<<icp.iterations()<<",\"convergence_state\":"<<int(icp.getConvergeCriteria()->getConvergenceState())
    <<",\"full_fitness_m2\":"<<score<<",\"final_matrix\":[";
   for(int i=0;i<16;++i){if(i)f<<",";f<<transform(i/4,i%4);}f<<"],\"nn_distances_m\":[";
   for(size_t i=0;i<distances.size();++i){if(i)f<<",";f<<distances[i];}f<<"],\"target_indices\":[";
   for(size_t i=0;i<targets.size();++i){if(i)f<<",";f<<targets[i];}f<<"],\"reciprocal\":[";
   for(size_t i=0;i<mutual.size();++i){if(i)f<<",";f<<mutual[i];}f<<"]}"<<std::endl;
   std::cout<<tag<<" gate="<<gate<<" converged="<<icp.hasConverged()<<" fitness="<<score<<std::endl;
  }
 }
}
