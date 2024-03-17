#include "estimator.h"

Estimator::Estimator() : ParamServer(), cloudToPub_(new pcl::PointCloud<pcl::PointXYZ>) {
    imuSub_ = nh_.subscribe("/imu", 100, &Estimator::imuCallBack, this);
    gtPoseSub_ = nh_.subscribe("/gt_pose", 100, &Estimator::gtPoseCallBack, this);
    imgPtsSub_ = nh_.subscribe("/img_points", 100, &Estimator::imgPtsCallBack, this);
    pclPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud", 2);

    imgPubLeft_ = nh_.advertise<sensor_msgs::Image>("/img_left", 2);
    imgPubRight_ = nh_.advertise<sensor_msgs::Image>("/img_right", 2);

    cloudToPub_->width = 20;
    cloudToPub_->height = 1;
    cloudToPub_->is_dense = false;
    cloudToPub_->points.resize(cloudToPub_->width * cloudToPub_->height);
    cloudToPub_->header.frame_id = "camera";

    minIdx_ = 1215 - 1;
    maxIdx_ = 1714 - 1;
    processInterval_ = 50;
    frame_ = 0;

    lastImuFlag_ = false;
    lastImgPtsFlag_ = false;
    lastGtPoseFlag_ = false;
}
Estimator::~Estimator() = default;

void Estimator::imuCallBack(const solution04::MyImu::ConstPtr &msg) {
    Imu::Ptr imu = std::make_shared<Imu>();
    imu->t = msg->header.stamp.toSec();
    imu->v(0) = msg->linear_velocity.x;
    imu->v(1) = msg->linear_velocity.y;
    imu->v(2) = msg->linear_velocity.z;
    imu->w(0) = msg->angular_velocity.x;
    imu->w(1) = msg->angular_velocity.y;
    imu->w(2) = msg->angular_velocity.z;
    imuArray_.push_back(imu);
    lastImuFlag_ = true;
    if (Utils::kDebug)
        ROS_INFO_STREAM("imu: \n"
                        << "t: " << imu->t << "\nv:\n"
                        << imu->v << "\nw:\n"
                        << imu->w);
}

void Estimator::imgPtsCallBack(const solution04::ImgPts::ConstPtr &msg) {
    ImgPts::Ptr imgPts = std::make_shared<ImgPts>();
    imgPts->t = msg->header.stamp.toSec();
    auto *pt = &imgPts->pts;
    for (int i = 0; i < 20; i++) {
        (*pt)(i, 0) = msg->left[i].x;
        (*pt)(i, 1) = msg->left[i].y;
        (*pt)(i, 2) = msg->right[i].x;
        (*pt)(i, 3) = msg->right[i].y;
    }
    imgPtsArray_.push_back(imgPts);
    lastImgPtsFlag_ = true;
    if (Utils::kDebug) ROS_INFO_STREAM("imgPts: \n" << *pt);

    // visualize the image points
    cv::Mat imgLeft = cv::Mat::zeros(cv::Size(imgWidth_, imgHeight_), CV_8UC3);
    cv::Mat imgRight = cv::Mat::zeros(cv::Size(imgWidth_, imgHeight_), CV_8UC3);
    for (int i = 0; i < 20; i++) {
        if (msg->left[i].x != -1)
            cv::circle(imgLeft, cv::Point(msg->left[i].x, msg->left[i].y), 5, cv::Scalar(255, 255, 255), -1);
        if (msg->right[i].x != -1)
            cv::circle(imgRight, cv::Point(msg->right[i].x, msg->right[i].y), 5, cv::Scalar(255, 255, 255), -1);
    }
    sensor_msgs::ImagePtr imgMsgLeft = cv_bridge::CvImage(msg->header, "bgr8", imgLeft).toImageMsg();
    sensor_msgs::ImagePtr imgMsgRight = cv_bridge::CvImage(msg->header, "bgr8", imgRight).toImageMsg();
    imgPubLeft_.publish(imgMsgLeft);
    imgPubRight_.publish(imgMsgRight);
}

void Estimator::gtPoseCallBack(const solution04::MyPose::ConstPtr &msg) {
    Pose::Ptr pose = std::make_shared<Pose>();
    pose->t = msg->header.stamp.toSec();
    pose->theta(0) = msg->theta.x;
    pose->theta(1) = msg->theta.y;
    pose->theta(2) = msg->theta.z;
    pose->r(0) = msg->r.x;
    pose->r(1) = msg->r.y;
    pose->r(2) = msg->r.z;
    gtPoseArray_.push_back(pose);
    lastGtPoseFlag_ = true;
    if (Utils::kDebug)
        ROS_INFO_STREAM("gtPose:\n"
                        << "t: " << pose->t << "\ntheta:\n"
                        << pose->theta << "\nr:\n"
                        << pose->r);
}

void Estimator::run() {
    if (lastImuFlag_ && lastImgPtsFlag_ && lastGtPoseFlag_) {
        lastImuFlag_ = false;
        lastImgPtsFlag_ = false;
        lastGtPoseFlag_ = false;
        int size = std::min({imuArray_.size(), imgPtsArray_.size(), gtPoseArray_.size()});
        // TODO: visualize the GT pcl (camera frame)

        Eigen::Vector3d r_i_vk_i = gtPoseArray_[frame_]->r;

        double roll = gtPoseArray_[frame_]->theta(0);
        double pitch = gtPoseArray_[frame_]->theta(1);
        double yaw = gtPoseArray_[frame_]->theta(2);

        Eigen::Matrix3d C_vk_i;
        C_vk_i = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        for (int i = 0; i < 20; i++) {
            Eigen::Vector3d p_ck_pj_ck;
            Eigen::Vector3d rho_i_pj_i = landmarks3dPts_.row(i);
            p_ck_pj_ck = C_c_v_ * C_vk_i * (rho_i_pj_i - r_i_vk_i) - rho_v_c_v_;
            cloudToPub_->points[i].x = p_ck_pj_ck.x();
            cloudToPub_->points[i].y = p_ck_pj_ck.y();
            cloudToPub_->points[i].z = p_ck_pj_ck.z();
        }
        sensor_msgs::PointCloud2 pclMsg;
        pcl::toROSMsg(*cloudToPub_, pclMsg);
        ros::Time time(imgPtsArray_[frame_]->t);
        pclMsg.header.stamp = time;
        pclPub_.publish(pclMsg);

        ++frame_;
        // TODO: visualize the GT trajectory
        if (size - 1 != maxIdx_) return;
        // TODO: process data
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "estimator");
    Estimator estimator;

    ros::Rate rate(200);
    while (ros::ok()) {
        ros::spinOnce();
        estimator.run();
        rate.sleep();
    }

    return 0;
}