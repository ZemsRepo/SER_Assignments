#ifndef SOLUTION04_INCLUDE_ESTIMATOR_H
#define SOLUTION04_INCLUDE_ESTIMATOR_H

#include "paramServer.h"
#include "utils.h"

class Estimator : ParamServer {
    using Imu = Utils::Imu;
    using ImgPts = Utils::ImgPts;
    using Pose = Utils::Pose;

   public:
    Estimator();
    ~Estimator();
    void run();
    void visualize();
    
   private:
    void imuCallBack(const solution04::MyImu::ConstPtr& msg);
    void imgPtsCallBack(const solution04::ImgPts::ConstPtr& msg);
    void gtPoseCallBack(const solution04::MyPose::ConstPtr& msg);

    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
    Eigen::Matrix3d rotVecToRotMat(const Eigen::Vector3d& v);
    Eigen::Vector3d rotMatToRotVec(const Eigen::Matrix3d& C);

    Sophus::SE3d computeIncrementalSE3Pose(const double& delta_t, const Imu::Ptr imu);

    void motionModel(const double& delta_t, const Imu::Ptr imu, const Eigen::Matrix3d& C_vk_i_1,
                     const Eigen::Vector3d& r_i_vk_i_1, Eigen::Matrix3d& C_vk_i,
                     Eigen::Vector3d& r_i_vk_i);

    Sophus::SE3d motionModel(const double& delta_t, const Imu::Ptr imu,
                             const Sophus::SE3d& T_vk_1_i);

    Sophus::SE3d poseVecToSE3Pose(const Eigen::Vector3d& theta, const Eigen::Vector3d& r);

    Eigen::Matrix<double, 20, 3> cameraFrameLandmarks(const Eigen::Matrix3d& C_vk_i,
                                                      const Eigen::Vector3d& r_i_vk_i);

    Eigen::Matrix<double, 20, 3> cameraFrameLandmarks(const Sophus::SE3d& T_k);

    Eigen::Vector3d transformWorldFrameToCameraFrame(const Eigen::Vector3d& worldFrameLandmark,
                                                     const Sophus::SE3d& T_k);

    Eigen::Vector4d transformCamreaFramePtToImgPt(const Eigen::Vector3d& p_ck_pj_ck);

    Eigen::Matrix<double, 20, 4> observationModel(const Sophus::SE3d& T_k);

    Eigen::Vector4d observationModel(const Sophus::SE3d& T_k, const Eigen::Vector3d& rho_i_pj_i);

    Sophus::Vector6d computeMotionError(const Sophus::SE3d& ksaiUpper_k, const Sophus::SE3d& T_k_1,
                                        const Sophus::SE3d& T_k);

    Eigen::Vector4d computeSingleLandmarkObservationError(
        const Eigen::Vector4d& y_jk, const Sophus::SE3d& T_k,
        const Eigen::Vector3d& cameraFrameLandmark);

    Eigen::VectorXd computeObservationError(const Eigen::Matrix<double, 20, 4>& y_k,
                                            const Sophus::SE3d& T_k);

    Sophus::Matrix6d F_k_1(const Sophus::SE3d& T_k, const Sophus::SE3d& T_k_1);

    Sophus::Matrix6d adjoint(const Sophus::SE3d& T_k);

    Eigen::DiagonalMatrix<double, 6> Q_k_inv(double delta_t);

    Eigen::DiagonalMatrix<double, 4> R_jk_inv();

    Eigen::DiagonalMatrix<double, -1> R_k_inv(const Eigen::Matrix<double, 20, 4>& y_k);

    Eigen::Matrix<double, 4, 6> G_jk(const Sophus::SE3d& T_k,
                                     const Eigen::Vector3d& worldFrameLandmark);

    Eigen::Matrix<double, -1, 6> G_k(const Eigen::Matrix<double, 20, 4>& y_k,
                                     const Sophus::SE3d& T_k);

    bool thisImgPtIsObservable(const Eigen::Vector4d& imgPt);

    std::vector<uint16_t> countObservableImgPtsIdx(const Eigen::Matrix<double, 20, 4>& imgPts);

    void insertSparseBlock(Eigen::SparseMatrix<double>& largeMatrix,
                           const Eigen::SparseMatrix<double>& block, int startRow, int startCol);

    // subscriber
    ros::Subscriber imuSub_;
    ros::Subscriber gtPoseSub_;
    ros::Subscriber imgPtsSub_;

    // publisher
    ros::Publisher imgPubLeft_;
    ros::Publisher imgPubRight_;
    ros::Publisher leftCamVisibleGtPclPub_;
    ros::Publisher rightCamVisibleGtPclPub_;
    ros::Publisher gtPclWorldMarkerPub_;
    ros::Publisher gtPclWorldPub_;
    ros::Publisher gtTrajPub_;
    ros::Publisher deadReckoningTrajPub_;
    ros::Publisher estTrajPub_;
    ros::Publisher frameMarkerPub_;

    // tf broadcaster
    tf2_ros::TransformBroadcaster br_;
    tf2_ros::StaticTransformBroadcaster staticBr_;

    // data array
    std::vector<Imu::Ptr> imuArray_;
    std::vector<ImgPts::Ptr> imgPtsArray_;
    std::vector<Pose::Ptr> gtPoseArray_;
    std::vector<const Sophus::SE3d*> incrementalPoseArraySE3_;
    std::vector<const Sophus::SE3d*> deadReckonPoseArraySE3_;
    std::vector<Sophus::SE3d*> estPoseArraySE3_;
    std::vector<Eigen::Matrix<double, 20, 4>*> deadReckonImgPtsArray_;

    bool lastImuFlag_;
    bool lastImgPtsFlag_;
    bool lastGtPoseFlag_;
    bool estimatorFlag_;
    bool vizFlag_;
    int frame_;
    int vizFrame_;

    nav_msgs::Path gtTraj_;
    nav_msgs::Path estTraj_;
    nav_msgs::Path deadReckoningTraj_;
};

#endif  // SOLUTION04_INCLUDE_ESTIMATOR_H
