#include "estimator.h"

Estimator::Estimator() : ParamServer() {
    imuSub_ = nh_.subscribe("/imu", 100, &Estimator::imuCallBack, this);
    gtPoseSub_ = nh_.subscribe("/gt_pose", 100, &Estimator::gtPoseCallBack, this);
    imgPtsSub_ = nh_.subscribe("/imgPts", 100, &Estimator::imgPtsCallBack, this);
}
Estimator::~Estimator() = default;

void Estimator::imuCallBack(const solution04::MyImu::ConstPtr &msg) {

    if (Utils::kDebug) {
        // ROS_INFO("imu:   angular: [%f %f %f]  linear: [%f %f %f]",
        //          msg->angular_velocity.x, msg->angular_velocity.y,
        //          msg->angular_velocity.z, msg->linear_velocity.x,
        //          msg->linear_velocity.y, msg->linear_velocity.z);
    }
}

void Estimator::imgPtsCallBack(const solution04::ImgPts::ConstPtr &msg) {
    if (Utils::kDebug) {
    }
}

void Estimator::gtPoseCallBack(const solution04::MyPose::ConstPtr &msg) {
    if (Utils::kDebug) {
        // ROS_INFO("gtPose:   theta: [%f %f %f]  r: [%f %f %f]",
        //          msg->theta.x, msg->theta.y, msg->theta.z, msg->r.x, msg->r.y,
        //          msg->r.z);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "estimator");

    Estimator estimator;

    ros::spin();
    return 0;
}