#include "utils.h"

namespace Utils {

void fromROSMsg(const solution04::MyImu::ConstPtr &msg, Imu::Ptr imu) {}

void fromROSMsg(const solution04::MyPose::ConstPtr &msg, Pose::Ptr pose) {}

void fromROSMsg(const solution04::ImgPts::ConstPtr &msg, ImgPts::Ptr imgPts) {}

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skewSymmetric;
    skewSymmetric << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return skewSymmetric;
}

void vec2rotMat(const Eigen::Vector3d &v, Eigen::Matrix3d &rotMat) {
    auto &&v_norm = v.norm();
    auto &I = Eigen::Matrix3d::Identity();
    rotMat = cos(v_norm) * I + (1 - cos(v_norm)) * (v / v_norm) * (v / v_norm).transpose() -
             sin(v_norm) * skewSymmetric(v / v_norm);
}
}  // namespace Utils