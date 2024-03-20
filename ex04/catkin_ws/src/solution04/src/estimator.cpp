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

    // initialize the path
    gtTraj_.header.frame_id = "world";

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
        ros::Time timePose(thisGtPose->t);

        double roll = thisGtPose->theta(0);
        double pitch = thisGtPose->theta(1);
        double yaw = thisGtPose->theta(2);
        Eigen::Matrix3d C_vk_i;
        C_vk_i = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond quaternion(C_vk_i);

        Eigen::Vector3d euler_angles = C_c_v_.eulerAngles(2, 1, 0);
        ROS_INFO_STREAM_ONCE("roll: " << euler_angles(2) * 180 / M_PI
                                      << " pitch: " << euler_angles(1) * 180 / M_PI
                                      << " yaw: " << euler_angles(0) * 180 / M_PI);

        // TODO: publish GT trajectory
        geometry_msgs::PoseStamped gtPoseMsg;
        gtPoseMsg.header.stamp = timePose;
        gtPoseMsg.header.frame_id = "world";
        gtPoseMsg.pose.position.x = r_i_vk_i.x();
        gtPoseMsg.pose.position.y = r_i_vk_i.y();
        gtPoseMsg.pose.position.z = r_i_vk_i.z();
        gtPoseMsg.pose.orientation.x = quaternion.x();
        gtPoseMsg.pose.orientation.y = quaternion.y();
        gtPoseMsg.pose.orientation.z = quaternion.z();
        gtPoseMsg.pose.orientation.w = quaternion.w();
        gtTraj_.poses.push_back(gtPoseMsg);
        gtTrajPub_.publish(gtTraj_);
        gtPosePub_.publish(gtPoseMsg);

        // TODO: broadcast tf (world -> vehicle)
        geometry_msgs::TransformStamped world2VehTrans;
        world2VehTrans.header.stamp = timePose;
        world2VehTrans.header.frame_id = "world";
        world2VehTrans.child_frame_id = "vehicle";
        world2VehTrans.transform.translation.x = r_i_vk_i.x();
        world2VehTrans.transform.translation.y = r_i_vk_i.y();
        world2VehTrans.transform.translation.z = r_i_vk_i.z();
        world2VehTrans.transform.rotation.x = quaternion.x();
        world2VehTrans.transform.rotation.y = quaternion.y();
        world2VehTrans.transform.rotation.z = quaternion.z();
        world2VehTrans.transform.rotation.w = quaternion.w();
        br_.sendTransform(world2VehTrans);

        // TODO: broadcast static tf (vehicle -> camera)
        geometry_msgs::TransformStamped veh2CamTrans;
        veh2CamTrans.header.stamp = timePose;
        veh2CamTrans.header.frame_id = "vehicle";
        veh2CamTrans.child_frame_id = "camera";
        veh2CamTrans.transform.translation.x = rho_v_c_v_.x();
        veh2CamTrans.transform.translation.y = rho_v_c_v_.y();
        veh2CamTrans.transform.translation.z = rho_v_c_v_.z();
        static Eigen::Quaterniond q(C_c_v_);
        veh2CamTrans.transform.rotation.x = q.x();
        veh2CamTrans.transform.rotation.y = q.y();
        veh2CamTrans.transform.rotation.z = q.z();
        veh2CamTrans.transform.rotation.w = q.w();
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
        pclWorldMsg.header.stamp = timePose;
        gtPclWorldPub_.publish(pclWorldMsg);

        
        // FIXME: visualize the GT pcl (camera frame)
        static pcl::PointCloud<pcl::PointXYZ>::Ptr gtCloudCam(new pcl::PointCloud<pcl::PointXYZ>);
        gtCloudCam->width = 20;
        gtCloudCam->height = 1;
        gtCloudCam->is_dense = false;
        gtCloudCam->points.resize(gtCloudCam->width * gtCloudCam->height);
        gtCloudCam->header.frame_id = "camera";

        for (int i = 0; i < 20; i++) {
            Eigen::Vector3d p_ck_pj_ck;
            Eigen::Vector3d rho_i_pj_i = landmarks3dPts_.row(i);
            p_ck_pj_ck = C_c_v_ * C_vk_i * (rho_i_pj_i - r_i_vk_i) - rho_v_c_v_;
            gtCloudCam->points[i].x = p_ck_pj_ck.x();
            gtCloudCam->points[i].y = p_ck_pj_ck.y();
            gtCloudCam->points[i].z = p_ck_pj_ck.z();
        }

        sensor_msgs::PointCloud2 pclCamMsg;
        pcl::toROSMsg(*gtCloudCam, pclCamMsg);
        pclCamMsg.header.stamp = timePose;
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