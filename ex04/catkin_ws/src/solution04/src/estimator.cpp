#include "estimator.h"

Estimator::Estimator() : ParamServer() {
    imuSub_ = nh_.subscribe("/imu", 100, &Estimator::imuCallBack, this);
    gtPoseSub_ = nh_.subscribe("/gt_pose", 100, &Estimator::gtPoseCallBack, this);
    imgPtsSub_ = nh_.subscribe("/img_points", 100, &Estimator::imgPtsCallBack, this);

    gtPclCamPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gt_point_cloud_cam", 2);
    gtPclWorldPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gt_point_cloud_world", 2);
    gtTrajPub_ = nh_.advertise<nav_msgs::Path>("/gt_trajectory", 2);
    gtPosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("/gt_pose1", 2);

    imgPubLeft_ = nh_.advertise<sensor_msgs::Image>("/img_left", 2);
    imgPubRight_ = nh_.advertise<sensor_msgs::Image>("/img_right", 2);

    // buffer
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
    for (int i = 0; i < 20; i++) {
        imgPts->pts(i, 0) = msg->left[i].x;
        imgPts->pts(i, 1) = msg->left[i].y;
        imgPts->pts(i, 2) = msg->right[i].x;
        imgPts->pts(i, 3) = msg->right[i].y;
    }
    imgPtsArray_.push_back(imgPts);
    lastImgPtsFlag_ = true;
    if (Utils::kDebug) ROS_INFO_STREAM("imgPts: \n" << imgPts->pts);
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

        auto thisGtPose = gtPoseArray_[frame_];
        Eigen::Vector3d r_i_vk_i = thisGtPose->r;
        ros::Time timeGtPose(thisGtPose->t);
        
        Eigen::Matrix3d C_vk_i;
        Utils::vec2rotMat(thisGtPose->theta, C_vk_i);

        Eigen::Quaterniond Q_vk_i(C_vk_i);
        Eigen::Quaterniond Q_vk_i_inv = Q_vk_i.inverse();

        // TODO: publish GT trajectory
        static nav_msgs::Path gtTraj;
        gtTraj.header.frame_id = "world";

        geometry_msgs::PoseStamped gtPoseMsg;
        gtPoseMsg.header.stamp = timeGtPose;
        gtPoseMsg.header.frame_id = "world";
        gtPoseMsg.pose.position.x = r_i_vk_i.x();
        gtPoseMsg.pose.position.y = r_i_vk_i.y();
        gtPoseMsg.pose.position.z = r_i_vk_i.z();
        gtPoseMsg.pose.orientation.x = Q_vk_i_inv.x();
        gtPoseMsg.pose.orientation.y = Q_vk_i_inv.y();
        gtPoseMsg.pose.orientation.z = Q_vk_i_inv.z();
        gtPoseMsg.pose.orientation.w = Q_vk_i_inv.w();
        gtTraj.poses.push_back(gtPoseMsg);
        gtTrajPub_.publish(gtTraj);
        gtPosePub_.publish(gtPoseMsg);

        // TODO: broadcast tf (world -> vehicle)
        geometry_msgs::TransformStamped world2VehTrans;
        world2VehTrans.header.stamp = timeGtPose;
        world2VehTrans.header.frame_id = "world";
        world2VehTrans.child_frame_id = "vehicle";
        world2VehTrans.transform.translation.x = r_i_vk_i.x();
        world2VehTrans.transform.translation.y = r_i_vk_i.y();
        world2VehTrans.transform.translation.z = r_i_vk_i.z();
        world2VehTrans.transform.rotation.x = Q_vk_i_inv.x();
        world2VehTrans.transform.rotation.y = Q_vk_i_inv.y();
        world2VehTrans.transform.rotation.z = Q_vk_i_inv.z();
        world2VehTrans.transform.rotation.w = Q_vk_i_inv.w();
        br_.sendTransform(world2VehTrans);

        // TODO: broadcast static tf (vehicle -> camera)
        static Eigen::Quaterniond Q_c_v(C_c_v_);
        static Eigen::Quaterniond Q_c_v_inv = Q_c_v.inverse();
        static geometry_msgs::TransformStamped veh2CamTrans;
        veh2CamTrans.header.stamp = timeGtPose;
        veh2CamTrans.header.frame_id = "vehicle";
        veh2CamTrans.child_frame_id = "camera";
        veh2CamTrans.transform.translation.x = rho_v_c_v_.x();
        veh2CamTrans.transform.translation.y = rho_v_c_v_.y();
        veh2CamTrans.transform.translation.z = rho_v_c_v_.z();
        veh2CamTrans.transform.rotation.x = Q_c_v_inv.x();
        veh2CamTrans.transform.rotation.y = Q_c_v_inv.y();
        veh2CamTrans.transform.rotation.z = Q_c_v_inv.z();
        veh2CamTrans.transform.rotation.w = Q_c_v_inv.w();
        staticBr_.sendTransform(veh2CamTrans);

        // TODO: visualize the GT pcl (world frame)
        static pcl::PointCloud<pcl::PointXYZ>::Ptr gtCloudWorld(new pcl::PointCloud<pcl::PointXYZ>);
        gtCloudWorld->width = 20;
        gtCloudWorld->height = 1;
        gtCloudWorld->is_dense = false;
        gtCloudWorld->points.resize(gtCloudWorld->width * gtCloudWorld->height);
        gtCloudWorld->header.frame_id = "world";

        for (int i = 0; i < 20; i++) {
            gtCloudWorld->points[i].x = landmarks3dPts_.row(i).x();
            gtCloudWorld->points[i].y = landmarks3dPts_.row(i).y();
            gtCloudWorld->points[i].z = landmarks3dPts_.row(i).z();
        }

        sensor_msgs::PointCloud2 pclWorldMsg;
        pcl::toROSMsg(*gtCloudWorld, pclWorldMsg);
        pclWorldMsg.header.stamp = timeGtPose;
        gtPclWorldPub_.publish(pclWorldMsg);

        // TODO: visualize the GT pcl (camera frame)
        static pcl::PointCloud<pcl::PointXYZ>::Ptr gtCloudCam(new pcl::PointCloud<pcl::PointXYZ>);
        gtCloudCam->width = 20;
        gtCloudCam->height = 1;
        gtCloudCam->is_dense = false;
        gtCloudCam->points.resize(gtCloudCam->width * gtCloudCam->height);
        gtCloudCam->header.frame_id = "camera";

        for (int i = 0; i < 20; i++) {
            Eigen::Vector3d p_ck_pj_ck;
            Eigen::Vector3d rho_i_pj_i = landmarks3dPts_.row(i);
            p_ck_pj_ck = C_c_v_ * (C_vk_i * (rho_i_pj_i - r_i_vk_i) - rho_v_c_v_);
            gtCloudCam->points[i].x = p_ck_pj_ck.x();
            gtCloudCam->points[i].y = p_ck_pj_ck.y();
            gtCloudCam->points[i].z = p_ck_pj_ck.z();
        }

        sensor_msgs::PointCloud2 pclCamMsg;
        pcl::toROSMsg(*gtCloudCam, pclCamMsg);
        pclCamMsg.header.stamp = timeGtPose;
        gtPclCamPub_.publish(pclCamMsg);

        // TODO: visualize image points
        auto thisImgPts = imgPtsArray_[frame_];
        cv::Mat imgLeft = cv::Mat::zeros(cv::Size(imgWidth_, imgHeight_), CV_8UC3);
        cv::Mat imgRight = cv::Mat::zeros(cv::Size(imgWidth_, imgHeight_), CV_8UC3);
        for (int i = 0; i < 20; i++) {
            if (thisImgPts->pts(i, 0) != -1)
                cv::circle(imgLeft, cv::Point(thisImgPts->pts(i, 0), thisImgPts->pts(i, 1)), 5,
                           cv::Scalar(255, 255, 255), -1);
            if (thisImgPts->pts(i, 2) != -1)
                cv::circle(imgRight, cv::Point(thisImgPts->pts(i, 2), thisImgPts->pts(i, 3)), 5,
                           cv::Scalar(255, 255, 255), -1);
        }
        auto header = std_msgs::Header();
        header.stamp = ros::Time(thisImgPts->t);
        header.frame_id = "camera";
        sensor_msgs::ImagePtr imgMsgLeft = cv_bridge::CvImage(header, "bgr8", imgLeft).toImageMsg();
        sensor_msgs::ImagePtr imgMsgRight =
            cv_bridge::CvImage(header, "bgr8", imgRight).toImageMsg();
        imgPubLeft_.publish(imgMsgLeft);
        imgPubRight_.publish(imgMsgRight);

        ++frame_;

        if (size - 1 != maxIdx_) return;
        // TODO: process data
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "estimator");
    Estimator estimator;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        estimator.run();
        rate.sleep();
    }

    return 0;
}