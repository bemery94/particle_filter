#include "nlos_particle_filter/robot_pose.h"

void RobotPose::setPosition(double x, double y, double z)
{
    x_ = x;
    y_ = y;
    z_ = z;
};

void RobotPose::setOrientation(double w, double x, double y, double z)
{
    orientationQuaternion_.w() = w;
    orientationQuaternion_.x() = x;
    orientationQuaternion_.y() = y;
    orientationQuaternion_.z() = z;

    orientationQuaternion_.normalize();
};

void RobotPose::setOrientation(double z, double y, double x)
{
    Eigen::Matrix3d orientationMatrixIn;
    orientationMatrixIn = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ())
                          * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX());

    orientationQuaternion_ = Eigen::Quaternion<double>(orientationMatrixIn);
};

Eigen::Quaternion<double> RobotPose::getOrientationQuaternion() const
{
    return orientationQuaternion_;
};

Eigen::Vector3d RobotPose::getOrientationZyx() const
{
    Eigen::Matrix3d rotationMatrix = orientationQuaternion_.toRotationMatrix();
    Eigen::Vector3d zyxOut = RobotPose::convertRotationMatToZyx(rotationMatrix);

    return zyxOut;
};

double RobotPose::getX() const
{
    return x_;
};

double RobotPose::getY() const
{
    return y_;
};

double RobotPose::getZ() const
{
    return z_;
};

inline Eigen::Vector3d RobotPose::convertRotationMatToZyx(Eigen::Matrix3d rotationMatrix)
{
    Eigen::Vector3d zyxOut;
    zyxOut << atan2(rotationMatrix(1,0), rotationMatrix(0,0)),
            atan2(-rotationMatrix(2,0), sqrt(pow(rotationMatrix(2,1),2) +
                                                pow(rotationMatrix(2,2),2))),
            atan2(rotationMatrix(2,1), rotationMatrix(2,2));

    return zyxOut;
};