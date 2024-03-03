#include "utils.h"

namespace Utils {

void fromROSMsg(const solution04::MyImu::ConstPtr &msg, ImuPtr imu) {
    imu->w(0) = msg->angular_velocity.x;
    imu->w(1) = msg->angular_velocity.y;
    imu->w(2) = msg->angular_velocity.z;

    imu->v(0) = msg->linear_velocity.x;
    imu->v(1) = msg->linear_velocity.y;
    imu->v(2) = msg->linear_velocity.z;
}

void fromROSMsg(const solution04::MyPose::ConstPtr &msg, PosePtr pose) {
    pose->theta(0) = msg->theta.x;
    pose->theta(1) = msg->theta.y;
    pose->theta(2) = msg->theta.z;

    pose->r(0) = msg->r.x;
    pose->r(1) = msg->r.y;
    pose->r(2) = msg->r.z;
}

void fromROSMsg(const solution04::ImgPts::ConstPtr &msg, ImgPtsPtr imgPts) {
    for (int i = 0; i < 20; i++) {
        (*imgPts)(i, 0) = msg->left[i].x;
        (*imgPts)(i, 1) = msg->left[i].y;
        (*imgPts)(i, 2) = msg->right[i].x;
        (*imgPts)(i, 3) = msg->right[i].y;
    }
}
}  // namespace Utils