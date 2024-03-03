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
    // vehicle_frame to camera_frame
    nh_.param("extrinsics/C_c_v", C_c_v_v, std::vector<double>(9, 0.0));
    nh_.param("extrinsics/rho_v_c_v", rho_v_c_v_v, std::vector<double>(3, 0.0));
    // stero camera intrinsics
    nh_.param("intrinsics/b", b_, 0.0);
    nh_.param("intrinsics/cu", cu_, 0.0);
    nh_.param("intrinsics/cv", cv_, 0.0);
    nh_.param("intrinsics/fu", fu_, 0.0);
    nh_.param("intrinsics/fv", fv_, 0.0);
    // covariance
    nh_.param("covariance/v_var", v_var_v, std::vector<double>(3, 0.0));
    nh_.param("covariance/w_var", w_var_v, std::vector<double>(3, 0.0));
    nh_.param("covariance/y_var", y_var_v, std::vector<double>(4, 0.0));

    // std::vector to eigen
    landmarks3dPts_ =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            landmarks3dPts_v.data(), 20, 3);
    C_c_v_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        C_c_v_v.data(), 3, 3);
    rho_v_c_v_ =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            rho_v_c_v_v.data(), 3, 1);
    v_var_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        v_var_v.data(), 3, 1);
    w_var_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        w_var_v.data(), 3, 1);
    y_var_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        y_var_v.data(), 4, 1);

    if (Utils::kDebug) {
        std::cout << "C_c_v: \n" << C_c_v_ << std::endl;
        std::cout << "rho_v_c_v: \n" << rho_v_c_v_ << std::endl;
        std::cout << "v_var: \n" << v_var_ << std::endl;
        std::cout << "w_var: \n" << w_var_ << std::endl;
        std::cout << "y_var: \n" << y_var_ << std::endl;
        std::cout << "b: " << b_ << std::endl;
        std::cout << "cu: " << cu_ << std::endl;
        std::cout << "cv: " << cv_ << std::endl;
        std::cout << "fu: " << fu_ << std::endl;
        std::cout << "fv: " << fv_ << std::endl;
        std::cout << "landmarks3dPts: \n" << landmarks3dPts_ << std::endl;
    }
}

ParamServer::~ParamServer() = default;
