#ifndef PROJECT_ROBOT_POSE_H
#define PROJECT_ROBOT_POSE_H

#include "Eigen/Dense"

//! Class for storing the robot pose.
//!
//! @author Brendan Emery
//! @date Oct 2016
//! @version 1.0.0
//! @bug Currently no known bugs.
//! @todo Currently no todos
//! @nosubgrouping
class RobotPose {
public:
    RobotPose() : orientationQuaternion_(1, 0, 0, 0), x_(0), y_(0), z_(0) {};
    void setPosition(double x, double y, double z);
    void setOrientation(double w, double x, double y, double z);
    void setOrientation(double z, double y, double x);
    double getX() const;
    double getY() const;
    double getZ() const;

    Eigen::Quaternion<double> getOrientationQuaternion() const;

    //! @return Vector of ZYX euler angles.
    Eigen::Vector3d getOrientationZyx() const;

private:
    double x_;
    double y_;
    double z_;

    inline static Eigen::Vector3d convertRotationMatToZyx(Eigen::Matrix3d rotationMatrix);
    Eigen::Quaternion<double> orientationQuaternion_;
};



#endif //PROJECT_ROBOT_POSE_H