#include <iostream>
#include <cmath>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <ceres/ceres.h>
#include <ceres/version.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
int main() {
  gtsam::Vector variances(6); variances.setConstant(1e-4);
  auto noise = gtsam::noiseModel::Diagonal::Variances(variances);
  const gtsam::Pose3 origin;
  const gtsam::Pose3 delta(gtsam::Rot3::RzRyRx(0.02,-0.01,0.1), gtsam::Point3(1,2,0.3));
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, origin, noise));
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(0,1,delta,noise));
  gtsam::Values initial;
  initial.insert(0,gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(0.01,0,0)));
  initial.insert(1,gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(0.8,1.8,0.2)));
  gtsam::ISAM2 optimizer;
  optimizer.update(graph,initial);
  for(int i=0;i<10;++i) optimizer.update();
  const auto result=optimizer.calculateEstimate().at<gtsam::Pose3>(1);
  const double error=gtsam::Pose3::Logmap(delta.between(result)).norm();
  if(!std::isfinite(error) || error>1e-5) { std::cerr << "GTSAM pose-graph check failed: " << error << std::endl; return 1; }
  ceres::Problem problem;
  cv_bridge::CvImage cv_image(std_msgs::Header(),"bgr8",cv::Mat::zeros(2,2,CV_8UC3));
  const auto message=cv_image.toImageMsg();
  if(message->width!=2 || message->height!=2) return 2;
  std::cout << "PASS: GTSAM ISAM2 two-pose graph; error=" << error << std::endl;
  std::cout << "PASS: Ceres " << CERES_VERSION_STRING << "; OpenCV " << CV_VERSION << "; cv_bridge image conversion" << std::endl;
  std::cout << "PASS: Eigen " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
  return 0;
}
