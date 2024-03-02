#ifndef SOLUTION04_INCLUDE_UTILITIES_H
#define SOLUTION04_INCLUDE_UTILITIES_H

#include <ros/ros.h>

#include <vector>

#include "eigen3/Eigen/Dense"
#include "solution04/ImgPts.h"
#include "solution04/MyImu.h"
#include "solution04/MyPose.h"
#include "solution04/Pixel.h"

class Utilities {
   public:
    Utilities();
    ~Utilities();

    static void threeDPt2ImgPt();
    static void motionModel();
    static void measurementModel();
    static void vec2Pose();
    static void pose2Vec();

    static void montionError();
    static void measurementError();
    static void error();
};

#endif  // SOLUTION04_INCLUDE_UTILITIES_H