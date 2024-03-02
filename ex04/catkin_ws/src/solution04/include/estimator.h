#ifndef SOLUTION04_INCLUDE_ESTIMATOR_H
#define SOLUTION04_INCLUDE_ESTIMATOR_H

#include "paramServer.h"
#include "utilities.h"

class Estimator : ParamServer {
   public:
    Estimator();
    ~Estimator();

   private:
    // subscriber
    // ros::Subscriber _imuSub;
    // ros::Subscriber _gtPoseSub;
    // ros::Subscriber _imgPtsSub;
};

#endif  // SOLUTION04_INCLUDE_ESTIMATOR_H
