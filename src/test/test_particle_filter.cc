#include <gtest/gtest.h>
#include "ros/ros.h"
#include "Eigen/Dense"

#include "particle_filter/particle_filter.h"

// Declare a test
TEST(ExpectedResults, calculateMoments_ShouldReturnGoodValues_WhenGoodValuesPassedIn)
{
    ros::NodeHandle n;
    SystemParameters systemParameters(1, 1, 1, 1, 1, 0.95, 4, 2, 1, 1, 0);
    ParticleFilter particleFilter(n, 0.0, 0.0, 0.0, 0, systemParameters);

    Particles& particles = particleFilter.getParticles();

    particles.x_ << 10,4,6,2;
    particles.y_ << 4,1,3,6;
    particles.z_ << 3,6,2,2;

    LandmarkMoments landmarkMoments;
    landmarkMoments = particleFilter.calculateMoments();

    EXPECT_EQ(landmarkMoments.landmarkMean[0], 5.5);
    EXPECT_EQ(landmarkMoments.landmarkMean[1], 3.5);
    EXPECT_EQ(landmarkMoments.landmarkMean[2], 3.25);

    Eigen::Matrix<double,3,3> expectedCovariance;
    expectedCovariance << 11.666666666666666, -1.0, -0.5, -1.0, 4.333333333333333,
                          -3.166666666666667, -0.5, -3.166666666666667, 3.583333333333333;

    EXPECT_TRUE(landmarkMoments.landmarkCovariance.isApprox(expectedCovariance));
}

TEST(ExpectedResults, calcNormalDistribution_ShouldReturnGoodValues_WhenGoodValuesPassedIn)
{
    ros::NodeHandle n;
    SystemParameters systemParameters(1, 1, 1, 1, 1, 0.95, 4, 2, 1, 1, 0);
    ParticleFilter particleFilter(n, 0.0, 0.0, 0.0, 0, systemParameters);

    double x = 1;
    double mean = 1;
    double variance = 1;

    EXPECT_NEAR(particleFilter.calcNormalDistribution(x, mean, variance), 0.398942, 0.000001);

    double x2 = 4.23;
    double mean2 = -1.32;
    double variance2 = sqrt(7.1);

    EXPECT_NEAR(particleFilter.calcNormalDistribution(x2, mean2, variance2), 0.017108564959013, 0.001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "particle_filter_tester");

    return RUN_ALL_TESTS();
}