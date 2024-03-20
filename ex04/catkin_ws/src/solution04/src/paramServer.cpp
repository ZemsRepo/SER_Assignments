#include "paramServer.h"

ParamServer::ParamServer() {
    std::vector<double> landmarks3dPts_v;
    std::vector<double> C_c_v_v;
    std::vector<double> rho_v_c_v_v;
    std::vector<double> v_var_v;
    std::vector<double> w_var_v;
    std::vector<double> y_var_v;

    // landmarks
    nh_.param("rho_i_pj_i", landmarks3dPts_v, std::vector<double>(20 * 3, 0.0));
    // vehicle to camera
    nh_.param("extrinsics/C_c_v", C_c_v_v, std::vector<double>(9, 0.0));
    nh_.param("extrinsics/rho_v_c_v", rho_v_c_v_v, std::vector<double>(3, 0.0));
    // stero camera intrinsics
    nh_.param("intrinsics/b", b_, 0.0);
    nh_.param("intrinsics/cu", cu_, 0.0);
    nh_.param("intrinsics/cv", cv_, 0.0);
    nh_.param("intrinsics/fu", fu_, 0.0);
    nh_.param("intrinsics/fv", fv_, 0.0);

    imgHeight_ = 480;
    imgWidth_ = 640;

    // covariance
    nh_.param("covariance/v_var", v_var_v, std::vector<double>(3, 0.0));
    nh_.param("covariance/w_var", w_var_v, std::vector<double>(3, 0.0));
    nh_.param("covariance/y_var", y_var_v, std::vector<double>(4, 0.0));

    // std::vector to eigen
    landmarks3dPts_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        landmarks3dPts_v.data(), 20, 3);
    C_c_v_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(C_c_v_v.data(), 3, 3);
    rho_v_c_v_ =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(rho_v_c_v_v.data(), 3, 1);
    v_var_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(v_var_v.data(), 3, 1);
    w_var_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(w_var_v.data(), 3, 1);
    y_var_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(y_var_v.data(), 4, 1);

    if (Utils::kDebug) {
        ROS_INFO_STREAM("C_c_v: \n" << C_c_v_);
        ROS_INFO_STREAM("rho_v_c_v: \n" << rho_v_c_v_);
        ROS_INFO_STREAM("v_var: \n" << v_var_);
        ROS_INFO_STREAM("w_var: \n" << w_var_);
        ROS_INFO_STREAM("y_var: \n" << y_var_);
        ROS_INFO_STREAM("b: " << b_);
        ROS_INFO_STREAM("cu: " << cu_);
        ROS_INFO_STREAM("cv: " << cv_);
        ROS_INFO_STREAM("fu: " << fu_);
        ROS_INFO_STREAM("fv: " << fv_);
        ROS_INFO_STREAM("landmarks3dPts: \n" << landmarks3dPts_);
    }
}

ParamServer::~ParamServer() = default;
