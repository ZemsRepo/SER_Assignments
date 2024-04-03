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
    struct ObjectiveFunction {
        ObjectiveFunction(int batchSize, const std::vector<const Sophus::SE3d*>& motionArraySE3)
            : batchSize_(batchSize), motionArraySE3_(motionArraySE3) {}
        int batchSize_;
        std::vector<const Sophus::SE3d*> motionArraySE3_;
        template <typename T>
        bool operator()(const T* const T_v_i, const T* const y, T* residuals) const;
    };

   private:
    void imuCallBack(const solution04::MyImu::ConstPtr& msg);
    void imgPtsCallBack(const solution04::ImgPts::ConstPtr& msg);
    void gtPoseCallBack(const solution04::MyPose::ConstPtr& msg);

    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);
    Eigen::Matrix3d expMap(const Eigen::Vector3d& v);
    void motionModel(const double& delta_t, const Imu::Ptr imu, const Eigen::Matrix3d& C_vk_i_1,
                     const Eigen::Vector3d& r_i_vk_i_1, Eigen::Matrix3d& C_vk_i,
                     Eigen::Vector3d& r_i_vk_i);

    Sophus::SE3d motionModel(const double& delta_t, const Imu::Ptr imu,
                             const Sophus::SE3d& T_vk_1_i);

    Sophus::SE3d vecToSE3(const Eigen::Vector3d& theta, const Eigen::Vector3d& r);

    Eigen::Matrix<double, 20, 3> camera3dPts(const Eigen::Matrix3d& C_vk_i,
                                             const Eigen::Vector3d& r_i_vk_i);

    Eigen::Matrix<double, 20, 3> camera3dPts(const Sophus::SE3d& T_vk_i);

    Eigen::Vector3d transformToCameraFrame(const Eigen::Vector3d& landmark,
                                           const Sophus::SE3d& T_vk_i);

    Eigen::Vector4d transformToImgPts(const Eigen::Vector3d& p_ck_pj_ck);

    Eigen::Matrix<double, 20, 4> observationModel(const Sophus::SE3d& T_vk_i);

    Eigen::Vector4d observationModel(const Sophus::SE3d& T_vk_i, const Eigen::Vector3d rho_i_pj_i);

    Sophus::Vector6d error_op_vk(const Sophus::SE3d& ksaiUpper_k, const Sophus::SE3d& T_op_k_1,
                                 const Sophus::SE3d& T_op_k);

    Sophus::Matrix6d F_k_1(const Sophus::SE3d& T_op_k_1, const Sophus::SE3d& T_op_k);

    Sophus::Vector4d error_op_y_jk(const Eigen::Vector4d& y_jk, const Sophus::SE3d& T_op_k,
                                   const Eigen::Vector3d p_ck_pj_ck);

    Eigen::Matrix<double, 80, 1> error_op_y_k(const Eigen::Matrix<double, 20, 4>& y_k,
                                              const Sophus::SE3d& T_op_k);

    Sophus::Matrix6d Q_k();

    Sophus::Matrix4d R_jk();

    Eigen::Matrix<double, 80, 80> R_k();

    Eigen::Matrix<double, 4, 6> G_jk(const Eigen::Vector3d& p_ck_pj_ck);

    Eigen::Matrix<double, 4, 6> circleDot(const Eigen::Vector3d& p);

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
    std::vector<const Sophus::SE3d*> deadReckoningPoseArraySE3_;
    std::vector<const Sophus::SE3d*> estPoseArraySE3_;
    std::vector<Eigen::Matrix<double, 20, 4>*> deadReckoningImgPtsArray_;

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
