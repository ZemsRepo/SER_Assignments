#include "estimator.h"


Estimator::Estimator():ParamServer() {}
Estimator::~Estimator() = default;

void printCallback(const solution04::MyImu::ConstPtr &msg) {
    // ROS_INFO("IMU:   angular: [%f %f %f]  linear: [%f %f %f]",
    //          msg->angular_velocity.x, msg->angular_velocity.y,
    //          msg->angular_velocity.z, msg->linear_velocity.x,
    //          msg->linear_velocity.y, msg->linear_velocity.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "estimator");

    Estimator estimator;
    
    ros::spin();
    return 0;
}