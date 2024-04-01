#ifndef SOLUTION04_INCLUDE_ESTIMATOR_H
#define SOLUTION04_INCLUDE_ESTIMATOR_H

#include "paramServer.h"
#include "utils.h"

class Estimator : ParamServer {
    using Imu = Utils::Imu;
    using ImgPts = Utils::ImgPts;
    using Pose = Utils::Pose;
    using SE3d_Ptr = std::shared_ptr<Sophus::SE3d>;
    using SO3d_Ptr = std::shared_ptr<Sophus::SO3d>;

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
    Eigen::Matrix3d expMap(const Eigen::Vector3d& v);
    void motionModel(const double& delta_t, const Imu::Ptr imu, const Eigen::Matrix3d& C_vk_i_1,
                     const Eigen::Vector3d& r_i_vk_i_1, Eigen::Matrix3d& C_vk_i,
                     Eigen::Vector3d& r_i_vk_i);

    Sophus::SE3d motionModel(const double& delta_t, const Imu::Ptr imu,
                             const Sophus::SE3d& T_vk_1_i);

    Sophus::SE3d vecToSE3(const Eigen::Vector3d& theta, const Eigen::Vector3d& r);

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
    std::vector<Sophus::SE3d*> deadReckoningPoseArraySE3_;
    std::vector<Sophus::SE3d*> estPoseArraySE3_;

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
