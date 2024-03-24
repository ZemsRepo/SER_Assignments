#include "estimator.h"

Estimator::Estimator() : ParamServer() {
    imuSub_ = nh_.subscribe("/imu", 100, &Estimator::imuCallBack, this);
    gtPoseSub_ = nh_.subscribe("/gt_pose", 100, &Estimator::gtPoseCallBack, this);
    imgPtsSub_ = nh_.subscribe("/img_points", 100, &Estimator::imgPtsCallBack, this);

    leftCamVisibleGtPclPub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/gt_point_cloud_left_vis", 2);
    rightCamVisibleGtPclPub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/gt_point_cloud_right_vis", 2);

    leftCamVisibleGtPclMarkerPub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/gt_point_cloud_left_vis_markers", 2);
    rightCamVisibleGtPclMarkerPub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/gt_point_cloud_right_vis_markers", 2);

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
        auto thisImgPts = imgPtsArray_[frame_];

        ros::Time timeGtPose(thisGtPose->t);
        ros::Time timeImgPts(thisImgPts->t);

        Eigen::Vector3d r_i_vk_i = thisGtPose->r;
        Eigen::Matrix3d C_vk_i;
        Utils::vec2rotMat(thisGtPose->theta, C_vk_i);

        Utils::Landmark3DPts landmarks3dPtsCam;
        for (int i = 0; i < 20; i++) {
            Eigen::Vector3d p_ck_pj_ck;
            Eigen::Vector3d rho_i_pj_i = landmarks3dPts_.row(i);
            p_ck_pj_ck = C_c_v_ * (C_vk_i * (rho_i_pj_i - r_i_vk_i) - rho_v_c_v_);
            landmarks3dPtsCam.row(i) = p_ck_pj_ck;
        }

        // TODO: publish visible landmarks (for both left and right cameras)

        // publish ground truth
        Utils::broadcastWorld2VehTF(br_, C_vk_i, r_i_vk_i, timeGtPose);
        Utils::broadcastStaticVeh2CamTF(staticBr_, C_c_v_, rho_v_c_v_, timeGtPose);

        Utils::publish_trajectory(gtTrajPub_, C_vk_i, r_i_vk_i, timeGtPose);
        Utils::publishPointCloud(gtPclWorldPub_, landmarks3dPts_, timeGtPose, "world");
        Utils::publishPointCloud(leftCamVisibleGtPclPub_, rightCamVisibleGtPclPub_,
                                 landmarks3dPtsCam, thisImgPts->pts, timeGtPose, "camera");

        Utils::publishMarkerArray(leftCamVisibleGtPclMarkerPub_, landmarks3dPts_, timeGtPose,
                                  "world");
        Utils::publishImage(imgPubLeft_, imgPubRight_, thisImgPts->pts, timeImgPts);

        ROS_INFO("frame: %d", frame_);

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