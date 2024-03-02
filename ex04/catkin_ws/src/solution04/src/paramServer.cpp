#include "paramServer.h"

ParamServer::ParamServer() {
    std::vector<double> landmarks3dPts_v;
    std::vector<double> C_c_v_v;
    std::vector<double> rho_v_c_v_v;
    std::vector<double> v_var_v;
    std::vector<double> w_var_v;
    std::vector<double> y_var_v;

    // landmarks
    _nh.param("rho_i_pj_i", landmarks3dPts_v, std::vector<double>(20 * 3, 0.0));
    // vehicle_frame to camera_frame
    _nh.param("extrinsics/C_c_v", C_c_v_v, std::vector<double>(9, 0.0));
    _nh.param("extrinsics/rho_v_c_v", rho_v_c_v_v, std::vector<double>(3, 0.0));
    // stero camera intrinsics
    _nh.param("intrinsics/b", _b, 0.0);
    _nh.param("intrinsics/cu", _cu, 0.0);
    _nh.param("intrinsics/cv", _cv, 0.0);
    _nh.param("intrinsics/fu", _fu, 0.0);
    _nh.param("intrinsics/fv", _fv, 0.0);
    // covariance
    _nh.param("covariance/v_var", v_var_v, std::vector<double>(3, 0.0));
    _nh.param("covariance/w_var", w_var_v, std::vector<double>(3, 0.0));
    _nh.param("covariance/y_var", y_var_v, std::vector<double>(4, 0.0));

    // std::vector to eigen
    _landmarks3dPts =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            landmarks3dPts_v.data(), 20, 3);
    _C_c_v = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        C_c_v_v.data(), 3, 3);
    _rho_v_c_v =
        Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
            rho_v_c_v_v.data(), 3, 1);
    _v_var = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        v_var_v.data(), 3, 1);
    _w_var = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        w_var_v.data(), 3, 1);
    _y_var = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        y_var_v.data(), 4, 1);

    std::cout << "C_c_v: \n" << _C_c_v << std::endl;
    std::cout << "rho_v_c_v: \n" << _rho_v_c_v << std::endl;
    std::cout << "v_var: \n" << _v_var << std::endl;
    std::cout << "w_var: \n" << _w_var << std::endl;
    std::cout << "y_var: \n" << _y_var << std::endl;
    std::cout << "b: " << _b << std::endl;
    std::cout << "cu: " << _cu << std::endl;
    std::cout << "cv: " << _cv << std::endl;
    std::cout << "fu: " << _fu << std::endl;
    std::cout << "fv: " << _fv << std::endl;
    std::cout << "landmarks3dPts: \n" << _landmarks3dPts << std::endl;
}

ParamServer::~ParamServer() = default;
