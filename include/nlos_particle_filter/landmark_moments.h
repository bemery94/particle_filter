#ifndef PROJECT_LANDMARK_MOMENTS_H
#define PROJECT_LANDMARK_MOMENTS_H

#include "Eigen/Dense"

//! Struct for storing the sample mean and covariance (first and second statistical moments) of the
//! converged particles.
//!
//! @author Brendan Emery
//! @date Oct 2016
//! @version 1.0.0
//! @bug Currently no known bugs.
//! @todo Currently no todos
//! @nosubgrouping
struct LandmarkMoments {
    Eigen::Matrix<double,3,1> landmarkMean;
    Eigen::Matrix<double,3,3> landmarkCovariance;
};

#endif //PROJECT_LANDMARK_MOMENTS_H
