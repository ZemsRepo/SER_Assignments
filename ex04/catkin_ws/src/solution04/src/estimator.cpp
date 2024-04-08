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
        /*r3*/ fu_ / z, 0, -fu_ * (x - b_) / (z * z), 0, /*r4*/ 0, fv_ / z, -fv_ * y / (z * z), 0;

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

Eigen::Matrix<double, 80, 6> Estimator::G_k(const Eigen::Matrix<double, 20, 4> y_k,
                                            const Sophus::SE3d& T_vi_k) {
    Eigen::Matrix<double, 80, 6> G_k;
    for (int i = 0; i < 20; i++) {
        const auto& p_ck_pj_ck = transformToCameraFrame(landmarks3dPts_.row(i), T_vi_k);
        const auto& y_jk = y_k.row(i);
        bool y_jk_is_visible = y_jk(0) != -1.0;
        G_k.block<4, 6>(i * 4, 0) =
            y_jk_is_visible ? G_jk(p_ck_pj_ck) : Eigen::Matrix<double, 4, 6>::Zero();
    }
    return G_k;
}

void Estimator::insertSparseBlock(Eigen::SparseMatrix<double>& largeMatrix,
                                  const Eigen::SparseMatrix<double>& block, int startRow,
                                  int startCol) {
    std::vector<Eigen::Triplet<double>> triplets;

    for (int k = 0; k < block.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(block, k); it; ++it) {
            triplets.emplace_back(it.row() + startRow, it.col() + startCol, it.value());
        }
    }

    largeMatrix.setFromTriplets(triplets.begin(), triplets.end());
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
    incrementalPoseArraySE3_.push_back(new Sophus::SE3d(Eigen::Matrix4d::Identity()));
    deadReckoningPoseArraySE3_.push_back(new Sophus::SE3d(T_vk_begin_i));
    estPoseArraySE3_.push_back(new Sophus::SE3d(T_vk_begin_i));

    for (int i = stateBegin_ + 1; i < stateEnd_ + 1; i++) {
        double delta_t = imuArray_[i]->t - imuArray_[i - 1]->t;
        const auto& T_vk_1_i = *deadReckoningPoseArraySE3_.back();
        const auto& ksaiUpper_k = incrementalPose(delta_t, imuArray_[i]);
        const auto& T_vk_i = ksaiUpper_k * T_vk_1_i;

        incrementalPoseArraySE3_.push_back(new Sophus::SE3d(ksaiUpper_k));
        deadReckoningPoseArraySE3_.push_back(new Sophus::SE3d(T_vk_i));
        estPoseArraySE3_.push_back(new Sophus::SE3d(T_vk_i));
    }

    for (const auto T_vk_i : deadReckoningPoseArraySE3_) {
        const auto& imgPts = observationModel(*T_vk_i);
        deadReckoningImgPtsArray_.push_back(new Eigen::Matrix<double, 20, 4>(imgPts));
    }

    // FIXME:
    int batchSize = stateEnd_ - stateBegin_ + 1;
    int maxIterations = 100;
    int minIterations = 10;
    for (int i = 0; i < maxIterations; i++) {
        Eigen::VectorXf e = Eigen::VectorXf::Zero(86 * batchSize);
        Eigen::VectorXf e_v = Eigen::VectorXf::Zero(6 * batchSize);
        Eigen::VectorXf e_y = Eigen::VectorXf::Zero(80 * batchSize);
        Eigen::MatrixXf H = Eigen::MatrixXf::Zero(86 * batchSize, 6 * batchSize);
        Eigen::MatrixXf H_F = Eigen::MatrixXf::Zero(6 * batchSize, 6 * batchSize);
        Eigen::MatrixXf H_G = Eigen::MatrixXf::Zero(80 * batchSize, 6 * batchSize);
        Eigen::MatrixXf W_Q_inv = Eigen::MatrixXf::Zero(6 * batchSize, 6 * batchSize);
        Eigen::MatrixXf W_R_inv = Eigen::MatrixXf::Zero(80 * batchSize, 80 * batchSize);
        Eigen::MatrixXf W_inv = Eigen::MatrixXf::Zero(86 * batchSize, 86 * batchSize);

        Eigen::MatrixXf Q_k_inv = Q_k().inverse().cast<float>();
        Eigen::MatrixXf R_k_inv = R_k().inverse().cast<float>();

        for (int k = 0; k < batchSize; k++) {
            if (k == 0) {
                auto T_k = *estPoseArraySE3_[k];
                auto y_k = imgPtsArray_[k + stateBegin_]->pts;

                e_v.segment<6>(6 * k) = Eigen::VectorXf::Zero(6);
                e_y.segment<80>(80 * k) = Eigen::VectorXf::Zero(80);
                H_F.block<6, 6>(6 * k, 6 * k) = Eigen::MatrixXf::Identity(6, 6);
                H_G.block<80, 6>(80 * k, 6 * k) = G_k(y_k, T_k).cast<float>();
                W_Q_inv.block<6, 6>(6 * k, 6 * k) = 1000.0 * Eigen::MatrixXf::Identity(6, 6);
                W_R_inv.block<80, 80>(80 * k, 80 * k) = R_k_inv;
            } else {
                double delta_t = imuArray_[k + stateBegin_]->t - imuArray_[k + stateBegin_ - 1]->t;
                auto ksaiUpper_k = *incrementalPoseArraySE3_[k];
                auto T_k = *estPoseArraySE3_[k];
                auto T_k_1 = *estPoseArraySE3_[k - 1];
                auto y_k = imgPtsArray_[k + stateBegin_]->pts;

                e_v.segment<6>(6 * k) = error_op_vk(ksaiUpper_k, T_k_1, T_k).cast<float>();
                e_y.segment<80>(80 * k) = error_op_y_k(y_k, T_k).cast<float>();
                H_F.block<6, 6>(6 * k, 6 * (k - 1)) = F_k_1(T_k_1, T_k).cast<float>();
                H_F.block<6, 6>(6 * k, 6 * k) = Eigen::MatrixXf::Identity(6, 6);
                H_G.block<80, 6>(80 * k, 6 * k) = G_k(y_k, T_k).cast<float>();
                W_Q_inv.block<6, 6>(6 * k, 6 * k) = Q_k_inv / (delta_t * delta_t);
                W_R_inv.block<80, 80>(80 * k, 80 * k) = R_k_inv;
            }
        }

        W_inv.block(0, 0, 6 * batchSize, 6 * batchSize) = W_Q_inv;
        W_inv.block(6 * batchSize, 6 * batchSize, 80 * batchSize, 80 * batchSize) = W_R_inv;
        H.block(0, 0, 6 * batchSize, 6 * batchSize) = H_F;
        H.block(6 * batchSize, 0, 80 * batchSize, 6 * batchSize) = H_G;

        Eigen::MatrixXf A = H.transpose() * W_inv * H;
        Eigen::MatrixXf b = H.transpose() * W_inv * e;

        Eigen::VectorXf x = A.ldlt().solve(b);

        std::ofstream debugLog("/home/zeming/Repos/SER_Assignments/ex04/catkin_ws/log.txt");

        if (!debugLog.is_open()) std::cerr << "Unable to open file" << std::endl;
        debugLog << "iteration: " << i + 1 << std::endl;
        debugLog << "e_v: \n" << e_v << std::endl;
        debugLog << "e_y: \n" << e_y << std::endl;
        debugLog << "W_Q_inv: \n" << W_Q_inv << std::endl;
        debugLog << "W_R_inv: \n" << W_R_inv << std::endl;

        debugLog << "Q_k_inv: \n" << Q_k_inv << std::endl;
        debugLog << "R_k_inv: \n" << R_k_inv << std::endl;
        debugLog.close();

        Eigen::MatrixXf motionError = 1 / 2 * e_v.transpose() * W_Q_inv * e_v;
        Eigen::MatrixXf measurementError = 1 / 2 * e_y.transpose() * W_R_inv * e_y;

        for (int k = 0; k < batchSize; k++) {
            Sophus::SE3d T_k_star = Sophus::SE3d::exp(x.segment<6>(6 * k).cast<double>());
            *estPoseArraySE3_[k] = T_k_star * (*estPoseArraySE3_[k]);
        }

        ROS_INFO_STREAM("optimize iteration: " << i + 1 << "  motion error: " << motionError
                                               << "  measurement error: " << measurementError);
    }

    // for (int i = 0; i < maxIterations; i++) {
    //     Eigen::SparseMatrix<double> e(86 * batchSize, 1);
    //     Eigen::SparseMatrix<double> e_v_i(6 * batchSize, 1);
    //     Eigen::SparseMatrix<double> e_y_i(80 * batchSize, 1);

    //     Eigen::SparseMatrix<double> H(86 * batchSize, 6 * batchSize);
    //     Eigen::SparseMatrix<double> H_F(6 * batchSize, 6 * batchSize);
    //     Eigen::SparseMatrix<double> H_G(80 * batchSize, 6 * batchSize);

    //     Eigen::SparseMatrix<double> W_Q_inv(6 * batchSize, 6 * batchSize);
    //     Eigen::SparseMatrix<double> W_R_inv(80 * batchSize, 80 * batchSize);
    //     Eigen::SparseMatrix<double> W_inv(86 * batchSize, 86 * batchSize);

    //     Eigen::SparseMatrix<double> Q_k_inv = Q_k().inverse().sparseView();
    //     Eigen::SparseMatrix<double> R_k_inv = R_k().inverse().sparseView();

    //     // make w
    //     for (int k = 0; k < batchSize; k++) {
    //         double delta_t = timeDiffArray[i];
    //         if (k == 0) {
    //             Eigen::MatrixXd W_Q_inv_begin(6, 6);
    //             W_Q_inv_begin.setIdentity();
    //             W_Q_inv_begin *= 1000;
    //             insertSparseBlock(W_Q_inv, W_Q_inv_begin.sparseView(), 6 * k, 6 * k);
    //             insertSparseBlock(W_R_inv, R_k_inv / (delta_t * delta_t), 80 * k, 80 * k);
    //         } else {
    //             insertSparseBlock(W_Q_inv, Q_k_inv / (delta_t * delta_t), 6 * k, 6 * k);
    //             insertSparseBlock(W_R_inv, R_k_inv / (delta_t * delta_t), 80 * k, 80 * k);
    //         }
    //     }
    //     insertSparseBlock(W_inv, W_Q_inv, 0, 0);
    //     insertSparseBlock(W_inv, W_R_inv, 6 * batchSize, 6 * batchSize);

    //     // make e
    //     for (int k = 0; k < batchSize; k++) {
    //         const auto& T_vk_i = *estPoseArraySE3_[k];
    //         const auto& y_k = imgPtsArray_[k + stateBegin_]->pts;

    //         if (k == 0) {
    //             insertSparseBlock(e_v_i, Eigen::Matrix<double, 6, 1>::Zero().sparseView(), 0,
    //             0); insertSparseBlock(e_y_i, Eigen::Matrix<double, 80,
    //             1>::Zero().sparseView(), 0, 0); insertSparseBlock(H_F,
    //             Sophus::Matrix6d::Identity().sparseView(), 0, 0); insertSparseBlock(H_G,
    //             G_k(y_k, T_vk_i).sparseView(), 0, 0);

    //         } else {
    //             const auto& T_vk_i_1 = *deadReckoningPoseArraySE3_[k - 1];
    //             const auto& ksaiUpper_k = *incrementalPoseArraySE3_[k];

    //             insertSparseBlock(e_v_i, error_op_vk(ksaiUpper_k, T_vk_i_1,
    //             T_vk_i).sparseView(),
    //                               6 * k, 0);
    //             insertSparseBlock(e_y_i, error_op_y_k(y_k, T_vk_i).sparseView(), 80 * k, 0);
    //             insertSparseBlock(H_F, F_k_1(T_vk_i_1, T_vk_i).sparseView(), 6 * k, 6 * (k -
    //             1)); insertSparseBlock(H_F, Sophus::Matrix6d::Identity().sparseView(), 6 * k,
    //             6 * k); insertSparseBlock(H_G, G_k(y_k, T_vk_i).sparseView(), 80 * k, 6 * k);
    //         }
    //     }
    //     insertSparseBlock(e, e_v_i, 0, 0);
    //     insertSparseBlock(e, e_y_i, 6 * batchSize, 0);
    //     insertSparseBlock(H, H_F, 0, 0);
    //     insertSparseBlock(H, H_G, 6 * batchSize, 0);

    //     Eigen::SparseMatrix<double> A = H.transpose() * W_inv * H;
    //     Eigen::SparseMatrix<double> b = H.transpose() * W_inv * e;

    //     Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    //     solver.analyzePattern(A);
    //     solver.factorize(A);
    //     solver.compute(A);
    //     Eigen::VectorXd x = solver.solve(b);

    //     for (int k = 0; k < batchSize; k++) {
    //         Sophus::SE3d T_k_star = Sophus::SE3d::exp(x.segment<6>(6 * k));
    //         *estPoseArraySE3_[k] = T_k_star * (*estPoseArraySE3_[k]);
    //     }

    //     Eigen::MatrixXd motionError = 1 / 2 * e_v_i.transpose() * W_Q_inv * e_v_i;
    //     Eigen::MatrixXd measurementError = 1 / 2 * e_y_i.transpose() * W_R_inv * e_y_i;

    //     ROS_INFO_STREAM("optimize iteration: " << i + 1 << "  motion error: " <<
    //     motionError.norm()
    //                                            << "  measurement error: "
    //                                            << measurementError.norm());
    // }

    // // build H
    // Eigen::MatrixXd H;
    // H << H_F, H_G;

    // // build W
    // int rows = W_Q.rows() + W_R.rows();
    // int cols = W_Q.cols() + W_R.cols();
    // Eigen::MatrixXd W = Eigen::MatrixXd::Zero(rows, cols);
    // W.topLeftCorner(W_Q.rows(), W_Q.cols()) = W_Q;
    // W.bottomRightCorner(W_R.rows(), W_R.cols()) = W_R;

    // const auto& A = H.transpose() * W.inverse() * H;
    // const auto& b = H.transpose() * W.inverse() * e_op;

    // ROS_INFO_STREAM("e_op: \n" << e_op);
    // ROS_INFO_STREAM("H: \n" << H);
    // ROS_INFO_STREAM("W: \n" << W);

    // Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > qr;
    // qr.compute(A.sparseView());
    // const auto& x = qr.solve(b);

    // for (int i = 0; i < batchSize; i++) {
    //     Sophus::SE3d T_k_star = Sophus::SE3d::exp(x.segment<6>(6 * i));
    //     *estPoseArraySE3_[i] = T_k_star * (*estPoseArraySE3_[i]);
    // }

    // double T_log_array [6 * deadReckoningPoseArraySE3_.size()];
    // for (int i = 0; i < deadReckoningPoseArraySE3_.size(); i++)
    //     for (int j = 0; j < 6; j++)
    //         T_log_array[6 * i + j] = deadReckoningPoseArraySE3_[i]->log()[j];

    // ceres::Problem problem;
    // static const Sophus::Matrix6d Q = Q_k();
    // static const Eigen::Matrix<double, 80, 80> R = R_k();

    // ceres::CostFunction* cost_function =
    //     new ceres::AutoDiffCostFunction<ObjectiveFunction, Utils::batchSize, 6 *
    //     Utils::batchSize>(
    //         new ObjectiveFunction(Utils::batchSize, incrementalPoseArraySE3_, imgPtsArray_,
    //         Q, R));
    // problem.AddResidualBlock(cost_function, nullptr, T_log_array);

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