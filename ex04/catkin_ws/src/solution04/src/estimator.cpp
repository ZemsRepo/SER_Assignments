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

Sophus::SE3d Estimator::incrementalPose(const double& delta_t, const Imu::Ptr imu) {
    const auto& w = imu->w;
    const auto& v = imu->v;
    const auto& d = v * delta_t;
    const auto& psi = w * delta_t;
    auto&& ksaiUpper = vecToSE3(psi, d);
    return ksaiUpper;
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
    return incrementalPose(delta_t, imu) * T_vk_1_i;
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
    for (int i = 0; i < 20; i++)
        landmarks3dPtsCam.row(i) = transformToCameraFrame(landmarks3dPts_.row(i), T_vk_i);

    return landmarks3dPtsCam;
}

Eigen::Vector3d Estimator::transformToCameraFrame(const Eigen::Vector3d& landmark,
                                                  const Sophus::SE3d& T_vk_i) {
    static Sophus::SE3d T_c_v(C_c_v_, -C_c_v_ * rho_v_c_v_);
    Eigen::Vector3d p_ck_pj_ck;
    const auto& rho_i_pj_i = landmark;

    p_ck_pj_ck = T_c_v * T_vk_i * rho_i_pj_i;
    return p_ck_pj_ck;
}

Eigen::Vector4d Estimator::transformToImgPts(const Eigen::Vector3d& p_ck_pj_ck) {
    double x = p_ck_pj_ck(0);
    double y = p_ck_pj_ck(1);
    double z = p_ck_pj_ck(2);

    Eigen::Vector4d imgPts;
    imgPts(0) = fu_ * x / z + cu_;
    imgPts(1) = fv_ * y / z + cv_;
    imgPts(2) = fu_ * (x - b_) / z + cu_;
    imgPts(3) = fv_ * y / z + cv_;
    return imgPts;
}

Eigen::Matrix<double, 20, 4> Estimator::observationModel(const Sophus::SE3d& T_vk_i) {
    const auto& landmarks3dPtsCam = camera3dPts(T_vk_i);
    Eigen::Matrix<double, 20, 4> imgPts;
    for (int i = 0; i < 20; i++) imgPts.row(i) = transformToImgPts(landmarks3dPtsCam.row(i));
    return imgPts;
}

Eigen::Vector4d Estimator::observationModel(const Sophus::SE3d& T_vk_i,
                                            const Eigen::Vector3d rho_i_pj_i) {
    Eigen::Vector3d p_ck_pj_ck = transformToCameraFrame(rho_i_pj_i, T_vk_i);
    return transformToImgPts(p_ck_pj_ck);
}

Sophus::Vector6d Estimator::error_op_vk(const Sophus::SE3d& ksaiUpper_k,
                                        const Sophus::SE3d& T_op_k_1, const Sophus::SE3d& T_op_k) {
    return (ksaiUpper_k * T_op_k_1 * T_op_k.inverse()).log();
}

Sophus::Matrix6d Estimator::F_k_1(const Sophus::SE3d& T_op_k_1, const Sophus::SE3d& T_op_k) {
    return (T_op_k * T_op_k_1.inverse()).Adj();
}

Sophus::Vector4d Estimator::error_op_y_jk(const Eigen::Vector4d& y_jk, const Sophus::SE3d& T_op_k,
                                          const Eigen::Vector3d p_ck_pj_ck) {
    return y_jk - observationModel(T_op_k, p_ck_pj_ck);
}

Eigen::Matrix<double, 80, 1> Estimator::error_op_y_k(const Eigen::Matrix<double, 20, 4>& y_k,
                                                     const Sophus::SE3d& T_op_k) {
    Eigen::Matrix<double, 80, 1> error_op_y_k;
    for (int i = 0; i < 20; i++) {
        const auto& y_jk = y_k.row(i);
        bool y_jk_is_visible = y_jk(0) != -1.0;
        error_op_y_k.segment<4>(i * 4) = y_jk_is_visible
                                             ? error_op_y_jk(y_jk, T_op_k, landmarks3dPts_.row(i))
                                             : Eigen::Vector4d::Zero();
    }
    return error_op_y_k;
}

Sophus::Matrix6d Estimator::Q_k() {
    Eigen::DiagonalMatrix<double, 6> Q_k;
    Q_k.diagonal() << v_var_, w_var_;
    return Q_k;
}

Sophus::Matrix4d Estimator::R_jk() { return y_var_.asDiagonal(); }

Eigen::Matrix<double, 80, 80> Estimator::R_k() {
    Eigen::DiagonalMatrix<double, 80> R_k;
    R_k.diagonal() << R_jk().diagonal().replicate<20, 1>();
    return R_k;
}

Eigen::Matrix<double, 4, 6> Estimator::G_jk(const Eigen::Vector3d& p_ck_pj_ck) {
    Eigen::Matrix4d S_jk;
    double x = p_ck_pj_ck.x();
    double y = p_ck_pj_ck.y();
    double z = p_ck_pj_ck.z();

    S_jk << /*r1*/ -fu_ / z, 0, -fu_ * x / (z * z), 0, /*r2*/ 0, fv_ / z, -fv_ * y / (z * z), 0,
        /*r3*/ fu_ / z, 0 - fu_ * (x - b_) / (z * z), 0, /*r4*/ 0, fv_ / z, -fv_ * y / (z * z), 0;

    auto&& Z_jk = circleDot(p_ck_pj_ck);
    return S_jk * Z_jk;
}

Eigen::Matrix<double, 4, 6> Estimator::circleDot(const Eigen::Vector3d& p) {
    Eigen::Matrix<double, 4, 6> p_circleDot;
    p_circleDot.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    p_circleDot.block<3, 3>(0, 3) = -skewSymmetric(p);
    p_circleDot.block<1, 6>(3, 0) = Eigen::Matrix<double, 1, 6>::Zero();
    return p_circleDot;
}

// template <typename T>
// bool Estimator::ObjectiveFunction::operator()(const T* const T_vk_i_log, T* residuals) const {
//     Sophus::Vector6d T_vk_i_log_vec;

//     for (int i = 0; i < 6; i++) {
//         T_vk_i_log_vec[i] = *T_vk_i_log;
//         T_vk_i_log++;
//     }

//     const Sophus::SE3d& T_vk_i = T_vk_i_log_vec.exp();
//     const auto& e_vk_i = error_op_vk(ksaiUpper_k, T_vk_1_i, T_vk_i);
//     const auto& e_y_k = error_op_y_k(y_k, T_vk_i);
//     const auto& e_vk_i_weighted = Q_k.diagonal().cwiseSqrt().cwiseInverse() * e_vk_i;
//     const auto& e_y_k_weighted = R_k.diagonal().cwiseSqrt().cwiseInverse() * e_y_k;

//     for (int i = 0; i < 6; i++) residuals[i] = e_vk_i_weighted(i);
//     for (int i = 0; i < 80; i++) residuals[i + 6] = e_y_k_weighted(i);

//     return true;
// }

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
    incrementalPoseArraySE3_.push_back(new Sophus::SE3d(Eigen::Matrix4d::Identity()));
    deadReckoningPoseArraySE3_.push_back(new Sophus::SE3d(T_vk_begin_i));

    for (int i = stateBegin_ + 1; i < stateEnd_ + 1; i++) {
        double delta_t = imuArray_[i]->t - imuArray_[i - 1]->t;
        const auto& T_vk_1_i = *deadReckoningPoseArraySE3_.back();
        const auto& ksaiUpper_k = incrementalPose(delta_t, imuArray_[i]);
        const auto& T_vk_i = ksaiUpper_k * T_vk_1_i;

        incrementalPoseArraySE3_.push_back(new Sophus::SE3d(ksaiUpper_k));
        deadReckoningPoseArraySE3_.push_back(new Sophus::SE3d(T_vk_i));
    }

    for (const auto T_vk_i : deadReckoningPoseArraySE3_) {
        auto&& imgPts = observationModel(*T_vk_i);
        deadReckoningImgPtsArray_.push_back(new Eigen::Matrix<double, 20, 4>(imgPts));
    }

    // double T_log_array[6 * deadReckoningPoseArraySE3_.size()];
    // for (int i = 0; i < deadReckoningPoseArraySE3_.size(); i++)
    //     for (int j = 0; j < 6; j++)
    //         T_log_array[6 * i + j] = deadReckoningPoseArraySE3_[i]->log()[j];

    // ceres::Problem problem;
    // static const Sophus::Matrix6d Q = Q_k();
    // static const Eigen::Matrix<double, 80, 80> R = R_k();

    // for (int i = 1; i < deadReckoningPoseArraySE3_.size(); i++) {
    //     const auto& ksaiUpper_k = *deadReckoningPoseArraySE3_[i];
    //     const auto& T_vk_1_i = *estPoseArraySE3_[i - 1];
    //     const auto& y_k = imgPtsArray_[stateBegin_ + i]->pts;

    //     ceres::CostFunction* cost_function =
    //         new ceres::AutoDiffCostFunction<ObjectiveFunction, 86, 6>(
    //             new ObjectiveFunction(ksaiUpper_k, T_vk_1_i, y_k, Q, R));
    //     problem.AddResidualBlock(cost_function, nullptr, T_log_array + 6 * i);
    // }

    // ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_type = ceres::TRUST_REGION;
    // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.use_nonmonotonic_steps = true;
    // options.max_num_iterations = 100;
    // options.minimizer_progress_to_stdout = true;

    // ceres::Solver::Summary summary;
    // ceres::Solve(options, &problem, &summary);


    // ROS_INFO_STREAM(summary.BriefReport() << "\n");

    // for (int i = 0; i < stateEnd_ - stateBegin_ + 1; i++) {
    //     const auto& T_vk_i = *deadReckoningPoseArraySE3_[i];
    //     auto&& imgPts = imgPtsArray_[i + stateBegin_]->pts;

    //     Sophus::Vector6d e_vk_i;
    //     Eigen::Matrix<double, 80, 1> e_y_k;
    //     if (i == 0) {
    //         e_vk_i.setZero();
    //         e_y_k.setZero();
    //     } else {
    //         const auto& T_vk_i_1 = *deadReckoningPoseArraySE3_[i - 1];
    //         const auto& ksaiUpper_k = *incrementalPoseArraySE3_[i];
    //         e_vk_i = error_op_vk(ksaiUpper_k, T_vk_i_1, T_vk_i);
    //         e_y_k = error_op_y_k(imgPts, T_vk_i);
    //     }

    //     // ROS_INFO_STREAM("e_vk_" << i + stateBegin_ + 1 << ":\n" << e_vk_i);
    //     // ROS_INFO_STREAM("e_y_" << i + stateBegin_ + 1 << ":\n" << e_y_k);
    // }

    estimatorFlag_ = false;
    vizFlag_ = true;
}

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