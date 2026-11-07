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
