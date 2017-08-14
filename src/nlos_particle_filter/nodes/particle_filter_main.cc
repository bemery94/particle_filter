#include "ros/ros.h"
#include "sensor_msg/SensorMsg.h"
#include "nlos_particle_filter/robot_pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "nlos_particle_filter/particle_filter.h"
#include "nlos_particle_filter/particle_parameters.h"


void observationCallback(const sensor_msg::SensorMsg::ConstPtr& msg);
void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg);


std::array<double, 2> sensorObservations;
RobotPose robotPose;
ParticleFilter* particleFilter = NULL;


// Particle filter to localise sound sources in the environment. Should subscribe to sound 
// source measurements and robot pose from a ros bag. The parameters for the particle filter 
// need to be tuned (params are set in the main inside particleParameters. For info on each 
// values meaning, check the ParticleParameters class).
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nlos_particle_filter");
    ros::NodeHandle n;

    ros::Subscriber sensorObsSub = n.subscribe<sensor_msg::SensorMsg>("/sensorMsg", 10,
                                                                        observationCallback);

    ros::Subscriber robotPoseSub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>
            ("/poseupdate", 10, robotPoseCallback);

    // Can create a launch file to set these values. Currently they're set below 
    // in particleParameters()
    n.getParam("/azimuthStdDevRadians", azimuthStdDev);
    n.getParam("/elevStdDevRadians", elevStdDev);
    n.getParam("/robotMotionNoiseIn", robotMotionNoiseIn);
    n.getParam("/numOfParticles", numOfParticles);
    n.getParam("/mapSize", mapSize);
    n.getParam("/kLDistanceThresh", kLDistanceThresh);
    n.getParam("/particleFilterNoise", particleFilterNoise);
    n.getParam("/numberOfEffectiveParticles", numberOfEffectiveParticles);


    std::array<double,3> initPositionEstimate = {0, 1, 0};
    ParticleParameters particleParameters(0.7, 0.0473, 0.0473, 800, 0.04, 1000, 5);
    particleFilter = new ParticleFilter(n, initPositionEstimate, particleParameters);

    ros::spin();
}


void observationCallback(const sensor_msg::SensorMsg::ConstPtr& msg)
{
    if(msg->exist_src_num == 1)
    {
        sensorObservations = {msg->src[0].azimuth * M_PI / 180,
                              msg->src[0].elevation * M_PI / 180};
    }
    else
    {
        std::cerr << "Should only pass in a single observation" << std::endl;
    }
}

void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
{
    robotPose.setPosition(poseMsg->pose.pose.position.x, 
                          poseMsg->pose.pose.position.y,
                          poseMsg->pose.pose.position.z + 0.329);

    robotPose.setOrientation(poseMsg->pose.pose.orientation.w, 
                             poseMsg->pose.pose.orientation.x,
                             poseMsg->pose.pose.orientation.y, 
                             poseMsg->pose.pose.orientation.z);

    particleFilter->updateParticleFilter(sensorObservations, robotPose);
}
