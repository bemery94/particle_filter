#ifndef PROJECT_PARTICLE_FILTER_H
#define PROJECT_PARTICLE_FILTER_H

#include "ros/ros.h"

#include "nlos_particle_filter/landmark_moments.h"
#include "nlos_particle_filter/particles_class.h"
#include "nlos_particle_filter/particle_parameters.h"
#include "nlos_particle_filter/robot_pose.h"

#include "gtest/gtest_prod.h"
#include <chrono>

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32MultiArray.h"

#include <math.h>
#include <iostream>
#include "Eigen/Dense"

//! Class for generating, storing and resampling particles.
//!
//! @author Brendan Emery
//! @date Oct 2016
//! @version 1.0.0
//! @bug Currently no known bugs.
//! @todo Currently no todos
//! @nosubgrouping
class ParticleFilter {
public:
    // Member functions
    //! Constructor to initialise particle filter. The constructor instantiates the an object of
    //! the Particles class which generates a random sample of particles evenly in a cube around the
    //! robot in the range of -mapSize to mapSize.
    ParticleFilter(ros::NodeHandle nodeHandle, std::array<double, 3> initPositionEstimate,
                   ParticleParameters particleParameters) :
            nodeHandle_(nodeHandle),
            particles_(initPositionEstimate, particleParameters.numberOfParticles,
                       particleParameters.mapSize),
            particleParameters_(particleParameters),
            particleFilterNoise_(particleParameters.particleFilterNoise)
    {
        std::string topicName = "particle_cloud";
        particleCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud>(topicName, 100);
    };

    //! Function to update a single iteration of the particle filter.
    //!
    //! @param landmark
    //! @param sensorObservations is a single observation for the landmark being tracked in the
    //!        particle filter.
    //! @param robotPose
    //! @param systemParameters
    void updateParticleFilter(const std::array<double,2>& sensorObservations,
                              const RobotPose& robotPose);

    bool checkParticleConvergence() const;
    LandmarkMoments calculateMoments() const;

    //! @name Getting the object containing the particles.
    //!@{

    //! Returns a constant object that cannot be modified.
    const Particles& getConstParticles() const;

    //! Returns a non-constant object that can be modified.
    Particles& getParticles();
    //!@}

    //! Stochastic universal low variance resampling.
    //! Resamples the particles with replacement with the probability of drawing a particles (0
    //! to multiple times) is proportional to its weight. This method runs in O(M) time where M
    //! is the number of particles to be resampled and all particles are evenly weighted, it will
    //! return an unchanged list of particles.
    void runUniversalStochasticResampling();

    void runSystematicSampling();

private:
    static inline double normalizeAngleMinusPiToPi(double x)
    {
        x = fmod(x + M_PI, 2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }

    // Member functions
    void runMotionModel();
    double fRand(double fMin, double fMax);
    void runObservationModel(const std::array<double, 2> &sensorObservations, const RobotPose &robotPose);

    Eigen::Vector2d calcExpectedBearings(unsigned int particleId, double robotX, double robotY,
                                         double robotZ, double robotAzimuth);
    void publishParticles() const;
    static inline int signOfInputDifference(double input1, double input2);
    void runResampling();

    inline double calcNormalDistribution(double xValue, double mean, double standardDeviation)
    {
        return invSqrt2Pi_ * exp(-0.5 * pow(((xValue - mean) / standardDeviation),2)) / standardDeviation;
    }
    // Allow unit test for private member function
    FRIEND_TEST(ExpectedResults,
            calcNormalDistribution_ShouldReturnGoodValues_WhenGoodValuesPassedIn);

    // Data members
    Particles particles_;
    ParticleParameters particleParameters_;
    const double pi_ = 3.1415926535897;
    const double invSqrt2Pi_ = 0.3989422804014327;
    ros::NodeHandle nodeHandle_;
    ros::Publisher particleCloudPublisher_;
    double particleFilterNoise_;
};

#endif //PROJECT_PARTICLE_FILTER_H
