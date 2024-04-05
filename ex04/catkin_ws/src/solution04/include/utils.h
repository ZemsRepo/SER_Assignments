#ifndef SOLUTION04_INCLUDE_UTILS_H
#define SOLUTION04_INCLUDE_UTILS_H

#include <ceres/ceres.h>
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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <thread>
#include <vector>

namespace Utils {

constexpr bool kDebug = 0;

constexpr int imgWidth = 640;
constexpr int imgHeight = 480;

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

bool imgPtIsVisible(const Eigen::Vector2d &imgPt);

void publish_trajectory(const ros::Publisher &pub, nav_msgs::Path &traj, const Eigen::Matrix3d &C,
                        const Eigen::Vector3d &r, const ros::Time &time);

void publishPointCloud(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 3> &landmarks,
                       const ros::Time &time, const std::string &frame_id);

void publishPointCloud(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 3> &landmarks,
                       const Eigen::Matrix<double, 20, 2> &imgPtsNomo, const ros::Time &time,
                       const std::string &frame_id);

void publishImage(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 2> &imgPtsMono,
                  const cv::Scalar &color, const ros::Time &time, const std::string &frame_id);

void publishImage(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 2> &imgPtsMono,
                  const cv::Scalar &color, const Eigen::Matrix<double, 20, 2> &imgPtsObsModel,
                  const cv::Scalar &colorObsModel, const ros::Time &time,
                  const std::string &frame_id);

void publishMarkerArray(const ros::Publisher &pub, const Eigen::Matrix<double, 20, 3> &landmarks,
                        const ros::Time &time, const std::string &frame_id);

void publishMarker(const ros::Publisher &pub, const ros::Time &time, const int frameIdx,
                   const std::string &frame_id);

void broadcastWorld2VehTF(tf2_ros::TransformBroadcaster &br, const Eigen::Matrix3d &C,
                          const Eigen::Vector3d &r, const ros::Time &time,
                          const std::string &child_frame_id);

void broadcastStaticVeh2CamTF(tf2_ros::StaticTransformBroadcaster &staticBr,
                              const Eigen::Matrix3d &C_c_v, const Eigen::Vector3d &rho_v_c_v,
                              const ros::Time &time);

}  // namespace Utils
#endif  // SOLUTION04_INCLUDE_UTILS_H