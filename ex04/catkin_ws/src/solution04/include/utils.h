#ifndef SOLUTION04_INCLUDE_UTILS_H
#define SOLUTION04_INCLUDE_UTILS_H

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <solution04/ImgPts.h>
#include <solution04/MyImu.h>
#include <solution04/MyPose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen3/Eigen/Dense>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

namespace Utils {

constexpr bool kDebug = 0;

struct Imu {
    using Ptr = std::shared_ptr<Imu>;
    double t;
    Eigen::Vector3d w;
    Eigen::Vector3d v;
};

struct Pose {
    using Ptr = std::shared_ptr<Pose>;
    double t;
    Eigen::Vector3d theta;
    Eigen::Vector3d r;
};

struct ImgPts {
    using Ptr = std::shared_ptr<ImgPts>;
    double t;
    Eigen::Matrix<double, 20, 4> pts;
};

using Landmark3DPts = Eigen::Matrix<double, 20, 3>;

void fromROSMsg(const solution04::MyImu::ConstPtr &msg, Imu::Ptr imu);
void fromROSMsg(const solution04::MyPose::ConstPtr &msg, Pose::Ptr pose);
void fromROSMsg(const solution04::ImgPts::ConstPtr &msg, ImgPts::Ptr imgPts);

}  // namespace Utils
#endif  // SOLUTION04_INCLUDE_UTILS_H