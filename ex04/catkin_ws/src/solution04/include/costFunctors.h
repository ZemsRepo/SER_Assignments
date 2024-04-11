#ifndef SOLUTION04_INCLUDE_COSTFUNCTORS_H
#define SOLUTION04_INCLUDE_COSTFUNCTORS_H

#include "utils.h"

struct MotionErrorCostFunctor {
    MotionErrorCostFunctor(const Sophus::SE3d* incrementalPose,
                           const Eigen::DiagonalMatrix<double, 6> motion_noise_sigma_inv)
        : incrementalPose_(incrementalPose), motionNoiseSigmaInv_(motion_noise_sigma_inv) {}

    template <typename T>
    bool operator()(const T* const pose_se3_k, const T* const pose_se3_k_1, T* residuals) const {
        Eigen::Map<const Eigen::Quaternion<T>> q_k(pose_se3_k[0], pose_se3_k[1], pose_se3_k[2],
                                                   pose_se3_k[3]);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_k(pose_se3_k[4], pose_se3_k[5], pose_se3_k[6]);
        Sophus::SE3<T> T_k(q_k, t_k);

        Eigen::Map<const Eigen::Quaternion<T>> q_K_1(pose_se3_k_1[0], pose_se3_k_1[1],
                                                     pose_se3_k_1[2], pose_se3_k_1[3]);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_K_1(pose_se3_k_1[4], pose_se3_k_1[5],
                                                       pose_se3_k_1[6]);
        Sophus::SE3<T> T_K_1(q_K_1, t_K_1);

        Eigen::Map<Eigen::Matrix<T, 6, 1>> e_v_k_weighted(residuals);

        e_v_k_weighted = motionNoiseSigmaInv_ * (*incrementalPose_ * T_k_1 * T_k.inverse()).log();

        return true;
    }

    const Sophus::SE3d* incrementalPose_;
    const Eigen::DiagonalMatrix<double, 6> motionNoiseSigmaInv_;
};

struct ObservationErrorCostFunctor {
    ObservationErrorCostFunctor(const Eigen::VectorXd* y_k,
                                const Eigen::Matrix<double, -1, 3> observedWorldFrameLandmarks,
                                const Eigen::DiagonalMatrix<double, -1> observation_noise_sigma_inv,
                                const Sophus::SE3d* T_c_v, const double fu, const double fv,
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
        Eigen::Map<const Eigen::Quaternion<T>> q_k(pose_se3_k[0], pose_se3_k[1], pose_se3_k[2],
                                                   pose_se3_k[3]);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_k(pose_se3_k[4], pose_se3_k[5], pose_se3_k[6]);
        Sophus::SE3<T> T_k(q_k, t_k);
        Eigen::Map<Eigen::Matrix<T, 6, 1>> e_y_k_weighted(residuals);

        Eigen::VectorXd y_k_pred(observedWorldFrameLandmarks_.rows() * 4);

        int counter = 0;
        for (const auto& p : observedWorldFrameLandmarks_) {
            // tramsform world frame landmark to camera frame
            Eigen::Vector3d cameraFrameLandmark = (*T_c_v_) * T_k * p;

            // project camera frame landmark to image plane
            double x = cameraFrameLandmark(0);
            double y = cameraFrameLandmark(1);
            double z = cameraFrameLandmark(2);

            Eigen::Vector4d imgPt;
            imgPt(0) = fu_ * x / z + cu_;
            imgPt(1) = fv_ * y / z + cv_;
            imgPt(2) = fu_ * (x - b_) / z + cu_;
            imgPt(3) = fv_ * y / z + cv_;

            y_k_pred.segment<4>(counter * 4) = imgPt;
            counter++;
        }

        e_y_k_weighted = observationNoiseSigmaInv_ * ((*y_k_).cast<T>() - y_k_pred.cast<T>());
    }
    const Sophus::SE3d* T_c_v_;
    const double fu_;
    const double fv_;
    const double cu_;
    const double cv_;
    const double b_;
    const Eigen::VectorXd* y_k_;
    const Eigen::Matrix<double, -1, 3> observedWorldFrameLandmarks_;
    const Eigen::DiagonalMatrix<double, -1> observationNoiseSigmaInv_;
};

#endif