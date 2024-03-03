#ifndef SOLUTION04_INCLUDE_ESTIMATOR_H
#define SOLUTION04_INCLUDE_ESTIMATOR_H

#include "paramServer.h"
#include "utils.h"

class Estimator : ParamServer {
   public:
    Estimator();
    ~Estimator();

   private:
    void imuCallBack(const solution04::MyImu::ConstPtr &msg);
    void imgPtsCallBack(const solution04::ImgPts::ConstPtr &msg);
    void gtPoseCallBack(const solution04::MyPose::ConstPtr &msg);

    // subscriber
    ros::Subscriber imuSub_;
    ros::Subscriber gtPoseSub_;
    ros::Subscriber imgPtsSub_;

    // data array
    std::vector<Utils::ImuPtr> imuArray_;
    std::vector<Utils::ImgPtsPtr> imgPtsArray_;
    std::vector<Utils::PosePtr> estPoseArray_;
    std::vector<Utils::PosePtr> gtPoseArray_;

};

#endif  // SOLUTION04_INCLUDE_ESTIMATOR_H
