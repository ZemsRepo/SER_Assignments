#ifndef SOLUTION04_INCLUDE_UTILS_H
#define SOLUTION04_INCLUDE_UTILS_H

#include <ros/ros.h>

#include <memory>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "solution04/ImgPts.h"
#include "solution04/MyImu.h"
#include "solution04/MyPose.h"

namespace Utils {

constexpr bool kDebug = true;

struct MyImu {
    Eigen::Vector3d w;
    Eigen::Vector3d v;
};

struct MyPose {
    Eigen::Vector3d theta;
    Eigen::Vector3d r;
};

using ImgPts = Eigen::Matrix<double, 20, 4>;
using Landmark3DPts = Eigen::Matrix<double, 20, 3>;
using ImuPtr = std::shared_ptr<MyImu>;
using PosePtr = std::shared_ptr<MyPose>;
using ImgPtsPtr = std::shared_ptr<ImgPts>;

void fromROSMsg(const solution04::MyImu::ConstPtr &msg, ImuPtr imu);
void fromROSMsg(const solution04::MyPose::ConstPtr &msg, PosePtr pose);
void fromROSMsg(const solution04::ImgPts::ConstPtr &msg, ImgPtsPtr imgPts);

void threeDPt2ImgPt();
void motionModel();
void measurementModel();
void vec2Pose();
void pose2Vec();

void montionError();
void measurementError();
void error();

}  // namespace Utils
#endif  // SOLUTION04_INCLUDE_UTILS_H