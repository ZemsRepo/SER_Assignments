#include "estimator.h"

Estimator::Estimator() : ParamServer() {
    imuSub_ = nh_.subscribe("/imu", 100, &Estimator::imuCallBack, this);
    gtPoseSub_ = nh_.subscribe("/gt_pose", 100, &Estimator::gtPoseCallBack, this);
    imgPtsSub_ = nh_.subscribe("/img_points", 100, &Estimator::imgPtsCallBack, this);

    leftCamVisibleGtPclPub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/gt_point_cloud_left_vis", 2);
    rightCamVisibleGtPclPub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/gt_point_cloud_right_vis", 2);

    gtPclWorldPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gt_point_cloud_world", 2);
    gtPclWorldMarkerPub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/gt_point_cloud_world_markers", 2);

    estTrajPub_ = nh_.advertise<nav_msgs::Path>("/est_trajectory", 2);
    deadReckoningTrajPub_ = nh_.advertise<nav_msgs::Path>("/dead_reckoning_trajectory", 2);
    gtTrajPub_ = nh_.advertise<nav_msgs::Path>("/gt_trajectory", 2);

    imgPubLeft_ = nh_.advertise<sensor_msgs::Image>("/img_left", 2);
    imgPubRight_ = nh_.advertise<sensor_msgs::Image>("/img_right", 2);

    frameMarkerPub_ = nh_.advertise<visualization_msgs::Marker>("/frame_marker", 2);
    lastImuFlag_ = false;
    lastImgPtsFlag_ = false;
    lastGtPoseFlag_ = false;
    estimatorFlag_ = true;
    vizFlag_ = false;
    frame_ = 0;
    vizFrame_ = 0;
}
Estimator::~Estimator() {
    for (auto poseSE3 : deadReckoningPoseArraySE3_) delete poseSE3;
    for (auto poseSE3 : estPoseArraySE3_) delete poseSE3;
    for (auto imgPts : deadReckoningImgPtsArray_) delete imgPts;
};

void Estimator::imuCallBack(const solution04::MyImu::ConstPtr& msg) {
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

void Estimator::imgPtsCallBack(const solution04::ImgPts::ConstPtr& msg) {
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

void Estimator::gtPoseCallBack(const solution04::MyPose::ConstPtr& msg) {
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

Eigen::Matrix3d Estimator::skewSymmetric(const Eigen::Vector3d& v) {
    return (Eigen::Matrix3d() << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0).finished();
}

Eigen::Matrix3d Estimator::expMap(const Eigen::Vector3d& v) {
    const auto& v_norm = v.norm();
    const auto& I = Eigen::Matrix3d::Identity();
    return cos(v_norm) * I + (1 - cos(v_norm)) * (v / v_norm) * (v / v_norm).transpose() -
           sin(v_norm) * skewSymmetric(v / v_norm);
}

Sophus::SE3d Estimator::vecToSE3(const Eigen::Vector3d& theta, const Eigen::Vector3d& r) {
    Eigen::Matrix3d&& C = expMap(theta);
    // Sophus::SO3d&& sophus_C = Sophus::SO3d::exp(theta); // sophus_C equals C.inverse()
    return Sophus::SE3d(C, -C * r);
}

void Estimator::motionModel(const double& delta_t, const Imu::Ptr imu,
                            const Eigen::Matrix3d& C_vk_1_i, const Eigen::Vector3d& r_i_vk_1_i,
                            Eigen::Matrix3d& C_vk_i, Eigen::Vector3d& r_i_vk_i) {
    const auto& w = imu->w;
    const auto& v = imu->v;
    const auto& d = v * delta_t;
    const auto& psi = w * delta_t;
    C_vk_i = expMap(psi) * C_vk_1_i;
    r_i_vk_i = r_i_vk_1_i + C_vk_1_i.transpose() * d;
}

Sophus::SE3d Estimator::motionModel(const double& delta_t, const Imu::Ptr imu,
                                    const Sophus::SE3d& T_vk_1_i) {
    const auto& w = imu->w;
    const auto& v = imu->v;
    const auto& d = v * delta_t;
    const auto& psi = w * delta_t;
    auto&& ksaiUpper = vecToSE3(psi, d);
    return ksaiUpper * T_vk_1_i;
}

Eigen::Matrix<double, 20, 3> Estimator::camera3dPts(const Eigen::Matrix3d& C_vk_i,
                                                    const Eigen::Vector3d& r_i_vk_i) {
    Eigen::Matrix<double, 20, 3> landmarks3dPtsCam;
    for (int i = 0; i < 20; i++) {
        Eigen::Vector3d p_ck_pj_ck;
        Eigen::Vector3d rho_i_pj_i = landmarks3dPts_.row(i);
        p_ck_pj_ck = C_c_v_ * (C_vk_i * (rho_i_pj_i - r_i_vk_i) - rho_v_c_v_);
        landmarks3dPtsCam.row(i) = p_ck_pj_ck;
    }
    return landmarks3dPtsCam;
}

Eigen::Matrix<double, 20, 3> Estimator::camera3dPts(const Sophus::SE3d& T_vk_i) {
    Eigen::Matrix<double, 20, 3> landmarks3dPtsCam;
    static Sophus::SE3d T_c_v(C_c_v_, -C_c_v_ * rho_v_c_v_);
    for (int i = 0; i < 20; i++) {
        Eigen::Vector3d p_ck_pj_ck;
        Eigen::Vector3d rho_i_pj_i = landmarks3dPts_.row(i);
        p_ck_pj_ck = T_c_v * T_vk_i * rho_i_pj_i;
        landmarks3dPtsCam.row(i) = p_ck_pj_ck;
    }
    return landmarks3dPtsCam;
}

Eigen::Matrix<double, 20, 4> Estimator::observationModel(const Sophus::SE3d& T_vk_i) {
    const auto& landmarks3dPtsCam = camera3dPts(T_vk_i);
    Eigen::Matrix<double, 20, 4> imgPts;
    for (int i = 0; i < 20; i++) {
        double x = landmarks3dPtsCam(i, 0);
        double y = landmarks3dPtsCam(i, 1);
        double z = landmarks3dPtsCam(i, 2);
        imgPts(i, 0) = fu_ * x / z + cu_;
        imgPts(i, 1) = fv_ * y / z + cv_;
        imgPts(i, 2) = fu_ * (x - b_) / z + cu_;
        imgPts(i, 3) = fv_ * y / z + cv_;
    }
    return imgPts;
}

void Estimator::visualize() {
    int baseRate = 10;
    ros::Rate rate(baseRate * vizSpeed_);
    while (ros::ok()) {
        rate.sleep();
        if (vizFlag_ == true && stateBegin_ + vizFrame_ <= stateEnd_) {
            const auto thisImu = imuArray_[stateBegin_ + vizFrame_];
            const auto thisImgPts = imgPtsArray_[stateBegin_ + vizFrame_];
            const auto thisGtPose = gtPoseArray_[stateBegin_ + vizFrame_];

            const auto thisDeadReckoningPose = deadReckoningPoseArraySE3_[vizFrame_];
            const auto C = thisDeadReckoningPose->rotationMatrix();
            const auto r = thisDeadReckoningPose->inverse().translation();

            const auto thisDeadReckoningImgPts = deadReckoningImgPtsArray_[vizFrame_];

            ros::Time timeImu(thisImu->t);
            ros::Time timeImgPts(thisImgPts->t);
            ros::Time timeGtPose(thisGtPose->t);

            const auto& r_i_vk_i_gt = thisGtPose->r;
            const auto& C_vk_i_gt = expMap(thisGtPose->theta);
            Sophus::SE3d T_vk_i_gt(C_vk_i_gt, -C_vk_i_gt * r_i_vk_i_gt);
            Eigen::Matrix<double, 20, 3> landmarks3dPtsCam = camera3dPts(T_vk_i_gt);

            // publish
            Utils::broadcastWorld2VehTF(br_, C_vk_i_gt, r_i_vk_i_gt, timeGtPose, "vehicle");

            Utils::broadcastWorld2VehTF(br_, C, r, timeGtPose, "vehicle_dead_reckoning");

            Utils::broadcastStaticVeh2CamTF(staticBr_, C_c_v_, rho_v_c_v_, timeGtPose);

            Utils::publish_trajectory(gtTrajPub_, gtTraj_, C_vk_i_gt, r_i_vk_i_gt, timeGtPose);

            Utils::publish_trajectory(deadReckoningTrajPub_, deadReckoningTraj_, C, r, timeGtPose);

            Utils::publishPointCloud(gtPclWorldPub_, landmarks3dPts_, timeGtPose, "world");

            Utils::publishPointCloud(leftCamVisibleGtPclPub_, landmarks3dPtsCam,
                                     thisImgPts->pts.block(0, 0, 20, 2), timeGtPose, "camera");

            Utils::publishPointCloud(rightCamVisibleGtPclPub_, landmarks3dPtsCam,
                                     thisImgPts->pts.block(0, 2, 20, 2), timeGtPose, "camera");

            Utils::publishImage(imgPubLeft_, thisImgPts->pts.block(0, 0, 20, 2),
                                cv::Scalar(0, 0, 164), thisDeadReckoningImgPts->block(0, 0, 20, 2),
                                cv::Scalar(0, 255, 0), timeImgPts, "camera");

            Utils::publishImage(imgPubRight_, thisImgPts->pts.block(0, 2, 20, 2),
                                cv::Scalar(135, 74, 32),
                                thisDeadReckoningImgPts->block(0, 2, 20, 2), cv::Scalar(0, 255, 0),
                                timeImgPts, "camera");

            Utils::publishMarkerArray(gtPclWorldMarkerPub_, landmarks3dPts_, timeGtPose, "world");

            Utils::publishMarker(frameMarkerPub_, timeGtPose, stateBegin_ + vizFrame_, "world");

            ++vizFrame_;
        }
    }
}

void Estimator::run() {
    if (estimatorFlag_ == false || gtPoseArray_.size() != imuArray_.size() ||
        gtPoseArray_.size() != imgPtsArray_.size() || gtPoseArray_.size() < stateEnd_ + 1)
        return;

    // Estimator implementation
    const auto gt_pose_begin = gtPoseArray_[stateBegin_];
    auto&& T_vk_begin_i = vecToSE3(gt_pose_begin->theta, gt_pose_begin->r);
    deadReckoningPoseArraySE3_.push_back(new Sophus::SE3d(T_vk_begin_i));

    for (int i = stateBegin_ + 1; i < stateEnd_ + 1; i++) {
        double delta_t = gtPoseArray_[i]->t - gtPoseArray_[i - 1]->t;
        const auto& T_vk_1_i = *deadReckoningPoseArraySE3_.back();
        auto&& T_vk_i = motionModel(delta_t, imuArray_[i], T_vk_1_i);
        deadReckoningPoseArraySE3_.push_back(new Sophus::SE3d(T_vk_i));
    }

    for (const auto T_vk_i : deadReckoningPoseArraySE3_) {
        auto&& imgPts = observationModel(*T_vk_i);
        deadReckoningImgPtsArray_.push_back(new Eigen::Matrix<double, 20, 4>(imgPts));
    }

    estimatorFlag_ = false;
    vizFlag_ = true;
}

// void Estimator::run() {
// if (lastImuFlag_ && lastImgPtsFlag_ && lastGtPoseFlag_) {
//     lastImuFlag_ = false;
//     lastImgPtsFlag_ = false;
//     lastGtPoseFlag_ = false;
//     int size = std::min({imuArray_.size(), imgPtsArray_.size(), gtPoseArray_.size()});

//     const auto thisImu = imuArray_[frame_];
//     const auto thisImgPts = imgPtsArray_[frame_];
//     const auto thisGtPose = gtPoseArray_[frame_];

//     ros::Time timeImu(thisImu->t);
//     ros::Time timeImgPts(thisImgPts->t);
//     ros::Time timeGtPose(thisGtPose->t);

//     // gt pose
//     const auto& r_i_vk_i_gt = thisGtPose->r;
//     const auto& C_vk_i_gt = expMap(thisGtPose->theta);

//     Utils::Eigen::Matrix<double, 20, 3> landmarks3dPtsCam;
//     for (int i = 0; i < 20; i++) {
//         Eigen::Vector3d p_ck_pj_ck;
//         Eigen::Vector3d rho_i_pj_i = landmarks3dPts_.row(i);
//         p_ck_pj_ck = C_c_v_ * (C_vk_i_gt * (rho_i_pj_i - r_i_vk_i_gt) - rho_v_c_v_);
//         landmarks3dPtsCam.row(i) = p_ck_pj_ck;
//     }

//     static auto r = r_i_vk_i_gt;
//     static auto C = C_vk_i_gt;
//     if (frame_ != 0)
//         motionModel(imuArray_[frame_]->t - imuArray_[frame_ - 1]->t, imuArray_[frame_], C, r,
//         C,
//                     r);
//     Utils::publish_trajectory(estTrajPub_, estTraj_, C, r, timeImu);

//     // publish ground truth
//     Utils::broadcastWorld2VehTF(br_, C_vk_i_gt, r_i_vk_i_gt, timeGtPose);

//     Utils::broadcastStaticVeh2CamTF(staticBr_, C_c_v_, rho_v_c_v_, timeGtPose);

//     Utils::publish_trajectory(gtTrajPub_, gtTraj_, C_vk_i_gt, r_i_vk_i_gt, timeGtPose);

//     Utils::publish_trajectory(deadReckoningTrajPub_, deadReckoningTraj_, C, r, timeImu);

//     Utils::publishPointCloud(gtPclWorldPub_, landmarks3dPts_, timeGtPose, "world");

//     Utils::publishPointCloud(leftCamVisibleGtPclPub_, landmarks3dPtsCam,
//                              thisImgPts->pts.block(0, 0, 20, 2), timeGtPose, "camera");

//     Utils::publishPointCloud(rightCamVisibleGtPclPub_, landmarks3dPtsCam,
//                              thisImgPts->pts.block(0, 2, 20, 2), timeGtPose, "camera");

//     Utils::publishImage(imgPubLeft_, thisImgPts->pts.block(0, 0, 20, 2), cv::Scalar(0, 0,
//     164),
//                         timeImgPts, "camera");

//     Utils::publishImage(imgPubRight_, thisImgPts->pts.block(0, 2, 20, 2),
//                         cv::Scalar(135, 74, 32), timeImgPts, "camera");

//     Utils::publishMarkerArray(gtPclWorldMarkerPub_, landmarks3dPts_, timeGtPose, "world");

//     ROS_INFO("frame: %d", frame_ + 1);

//     ++frame_;
// }
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "estimator");
    Estimator estimator;

    std::thread visThread(&Estimator::visualize, &estimator);
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        estimator.run();
        rate.sleep();
    }
    visThread.join();
    return 0;
}