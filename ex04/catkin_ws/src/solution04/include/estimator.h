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

   private:
    void imuCallBack(const solution04::MyImu::ConstPtr &msg);
    void imgPtsCallBack(const solution04::ImgPts::ConstPtr &msg);
    void gtPoseCallBack(const solution04::MyPose::ConstPtr &msg);

    // subscriber
    ros::Subscriber imuSub_;
    ros::Subscriber gtPoseSub_;
    ros::Subscriber imgPtsSub_;

    // publisher
    ros::Publisher imgPubLeft_;
    ros::Publisher imgPubRight_;
    ros::Publisher pclPub_;

    // data array
    std::vector<Imu::Ptr> imuArray_;
    std::vector<ImgPts::Ptr> imgPtsArray_;
    std::vector<Pose::Ptr> estPoseArray_;
    std::vector<Pose::Ptr> gtPoseArray_;

    int minIdx_;
    int maxIdx_;
    int processInterval_;
    int frame_;

    bool lastImuFlag_;
    bool lastImgPtsFlag_;
    bool lastGtPoseFlag_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToPub_;
};

#endif  // SOLUTION04_INCLUDE_ESTIMATOR_H
