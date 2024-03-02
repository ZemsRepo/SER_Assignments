#ifndef SOLUTION04_INCLUDE_PARAMSERVER_H
#define SOLUTION04_INCLUDE_PARAMSERVER_H

#include "utilities.h"

class ParamServer {
   public:
    ParamServer();
    ~ParamServer();

   protected:
    ros::NodeHandle _nh;

    // vehicle_frame to camera_frame
    Eigen::Matrix3d _C_c_v;
    Eigen::Vector3d _rho_v_c_v;

    // stero camera intrinsics
    double _b;
    double _cu;
    double _cv;
    double _fu;
    double _fv;

    // covariance
    Eigen::Vector3d _v_var;
    Eigen::Vector3d _w_var;
    Eigen::Vector4d _y_var;

    // landmarks
    Eigen::Matrix<double, 20, 3> _landmarks3dPts;
};

#endif  // SOLUTION04_INCLUDE_PARAMSERVER_H