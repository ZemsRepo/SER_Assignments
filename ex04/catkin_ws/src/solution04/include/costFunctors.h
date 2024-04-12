#ifndef SOLUTION04_INCLUDE_COSTFUNCTORS_H
#define SOLUTION04_INCLUDE_COSTFUNCTORS_H

#include "utils.h"

struct MotionErrorCostFunctor {
    MotionErrorCostFunctor(const Sophus::SE3d incrementalPose,
                           const Eigen::DiagonalMatrix<double, 6> motion_noise_sigma_inv)
        : incrementalPose_(incrementalPose), motionNoiseSigmaInv_(motion_noise_sigma_inv) {}

    template <typename T>
    bool operator()(const T* const pose_se3_k, const T* const pose_se3_k_1, T* residuals) const {
        const Eigen::Quaternion<T> q_k(pose_se3_k[0], pose_se3_k[1], pose_se3_k[2], pose_se3_k[3]);
        const Eigen::Matrix<T, 3, 1> t_k(pose_se3_k[4], pose_se3_k[5], pose_se3_k[6]);
        Sophus::SE3<T> T_k(q_k, t_k);

        const Eigen::Quaternion<T> q_k_1(pose_se3_k_1[0], pose_se3_k_1[1], pose_se3_k_1[2],
                                         pose_se3_k_1[3]);
        const Eigen::Matrix<T, 3, 1> t_k_1(pose_se3_k_1[4], pose_se3_k_1[5], pose_se3_k_1[6]);
        Sophus::SE3<T> T_k_1(q_k_1, t_k_1);

        Eigen::Matrix<T, 6, 1> e_v_k_weighted;

        e_v_k_weighted = motionNoiseSigmaInv_ * (incrementalPose_ * T_k_1 * T_k.inverse()).log();

        for (int i = 0; i < 6; i++) {
            residuals[i] = e_v_k_weighted(i);
        }

        return true;
    }

    const Sophus::SE3d incrementalPose_;
    const Eigen::DiagonalMatrix<double, 6> motionNoiseSigmaInv_;
};

struct ObservationErrorCostFunctor {
    ObservationErrorCostFunctor(const Eigen::VectorXd y_k,
                                const Eigen::Matrix<double, -1, 3> observedWorldFrameLandmarks,
                                const Eigen::DiagonalMatrix<double, -1> observation_noise_sigma_inv,
                                const Sophus::SE3d T_c_v, const double fu, const double fv,
                                const double cu, const double cv, const double b)
        : y_k_(y_k),
          observedWorldFrameLandmarks_(observedWorldFrameLandmarks),
          observationNoiseSigmaInv_(observation_noise_sigma_inv),
          T_c_v_(T_c_v),
          fu_(fu),
          fv_(fv),
          cu_(cu),
          cv_(cv),
          b_(b) {}

    template <typename T>
    bool operator()(const T* const pose_se3_k, T* residuals) const {
        const Eigen::Quaternion<T> q_k(pose_se3_k[0], pose_se3_k[1], pose_se3_k[2], pose_se3_k[3]);
        const Eigen::Matrix<T, 3, 1> t_k(pose_se3_k[4], pose_se3_k[5], pose_se3_k[6]);
        Sophus::SE3<T> T_k(q_k, t_k);

        int observedLandmarksNum = observedWorldFrameLandmarks_.rows();

        Eigen::Matrix<T, -1, 1> e_y_k_weighted(observedLandmarksNum * 4);
        Eigen::Matrix<T, -1, 1> y_k_pred(observedLandmarksNum * 4);

        int counter = 0;
        for (int i = 0; i < observedLandmarksNum; i++) {
            // tramsform world frame landmark to camera frame
            Eigen::Vector3d p = observedWorldFrameLandmarks_.row(i);
            Eigen::Matrix<T, -1, 1> cameraFrameLandmark = T_c_v_ * T_k * p;

            // project camera frame landmark to image plane
            T x = cameraFrameLandmark(0);
            T y = cameraFrameLandmark(1);
            T z = cameraFrameLandmark(2);

            Eigen::Matrix<T, 4, 1> imgPt;
            imgPt(0) = fu_ * x / z + cu_;
            imgPt(1) = fv_ * y / z + cv_;
            imgPt(2) = fu_ * (x - b_) / z + cu_;
            imgPt(3) = fv_ * y / z + cv_;

            y_k_pred(counter * 4 + 0, 0) = imgPt(0);
            y_k_pred(counter * 4 + 1, 0) = imgPt(1);
            y_k_pred(counter * 4 + 2, 0) = imgPt(2);
            y_k_pred(counter * 4 + 3, 0) = imgPt(3);

            counter++;
        }

        e_y_k_weighted = observationNoiseSigmaInv_ * (y_k_ - y_k_pred);

        for (int i = 0; i < observedLandmarksNum * 4; i++) {
            residuals[i] = e_y_k_weighted(i);
        }

        return true;
    }
    const Sophus::SE3d T_c_v_;
    const double fu_;
    const double fv_;
    const double cu_;
    const double cv_;
    const double b_;
    const Eigen::VectorXd y_k_;
    const Eigen::Matrix<double, -1, 3> observedWorldFrameLandmarks_;
    const Eigen::DiagonalMatrix<double, -1> observationNoiseSigmaInv_;
};

#endif