#include "nlos_particle_filter/particle_filter.h"

#if TIMER_MODE
#define TIME(x, y){std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();\
x;\
std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();\
auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();\
std::cout << y << " duration = " << duration * pow(10,-6) << std::endl;\
}
#else
#define TIME(x,y) x;
#endif


void ParticleFilter::updateParticleFilter(const std::array<double,2>& sensorObservation,
                                          const RobotPose& robotPose)
{
    runMotionModel();
    runObservationModel(sensorObservation, robotPose);
    publishParticles();
    runResampling();
}


bool ParticleFilter::checkParticleConvergence() const
{
    // Particle convergence calculated by comparing the Kullback–Leibler divergence with a user
    // defined threshold such that we classify the particles as having converged if
    // D_{KL}(p || q) < threshold_{KLD}.
    //
    // D_{KL}(p || q) =
    // n/2*ln(2*pi) + 1/2*ln|cov| + 1/2*sum_{i=1_to_n}(w_i * MD(x_i)) + sum_{i=1_to_n}(w_i * ln(w_i)
    //
    // where,
    //         - MD(x_i) is the mahalanobis distance between the ith particle and the sample mean
    //      and covariance:     MD(x_i) = (x - mu)^T * cov^(-1) * (x - mu),
    //         - mu is the sample mean of all particles
    //         - cov is the sample covariance of all particles
    //         - n is the number of particles.
    //         - x_i is a [3x1] vector containing the x, y and z coordinates of the ith particle
    //         - w_i is the weight of the ith particle. Since this step is performed after
    //      resampling, all particles are weighted equally with w_i = 1 / n.
    //
    unsigned int n = particles_.getNumberOfParticles();
    LandmarkMoments landmarkMoments = calculateMoments();
    Eigen::Matrix<double,3,1> mean = landmarkMoments.landmarkMean;
    Eigen::Matrix<double,3,3> covariance = landmarkMoments.landmarkCovariance;
    double w_i = 1 / n;

    double equationConstant = n/2 * log(2*pi_) + 1/2*log(covariance.determinant());
    double equationSum1 = 0;
    double equationSum2 = 0;

    for(unsigned int i=0; i<n; ++i)
    {
        Eigen::Matrix<double,3,1> particleCoordinates;
        particleCoordinates << particles_.x_[i], particles_.y_[i], particles_.z_[i];

        equationSum1 += w_i * (particleCoordinates - mean).transpose() * covariance.inverse() *
                        (particleCoordinates - mean);

        equationSum2 += w_i * log(w_i);
    }

    double klDivergence = equationConstant + equationSum1 + equationSum2;

    return klDivergence > particleParameters_.kLDistanceThresh;
}


void ParticleFilter::runMotionModel()
{
    unsigned int n = particles_.getNumberOfParticles();

    // Create normal distribution and random number generator
    std::random_device rd;
    std::mt19937 e2(rd());
    std::normal_distribution<> dist(0, 1);

    for(size_t i=0; i < n; ++i)
    {
        double particleX = particles_.x_[i];
        double particleY = particles_.y_[i];
        double particleZ = particles_.z_[i];

        // Add zero mean gaussian noise with variance of 1 to particle coordinates
        double randX = dist(e2);
        particleX += randX * particleFilterNoise_;

        double randY = dist(e2);
        particleY += randY * particleFilterNoise_;

        double randZ = dist(e2);
        particleZ += randZ * particleFilterNoise_;

        particles_.x_[i] = particleX;
        particles_.y_[i] = particleY;
        particles_.z_[i] = particleZ;
    }
}


double ParticleFilter::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}


void ParticleFilter::runObservationModel(const std::array<double, 2> &sensorObservation,
                                         const RobotPose &robotPose)
{
    unsigned int n = particles_.getNumberOfParticles();

    double robotAzimuth = robotPose.getOrientationZyx()[0];
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double robotZ = robotPose.getZ();

    // The new weight for each particle is calculated according to:
    //      newWeight = oldWeight * p(z_t|x_t)
    //                = oldWeight * N(mu_{az}, stdDev_{az}) * N(mu_{elev}, stdDev_{elev})
    TIME(for(size_t i=0; i < n; ++i)
    {
        Eigen::Vector2d expectedObservation = calcExpectedBearings(i, robotX, robotY, robotZ, robotAzimuth);
        double azimuthLikelihood = calcNormalDistribution(sensorObservation[0], expectedObservation[0],
                                       particleParameters_.azStdDev);

        double elevationLikelihood = calcNormalDistribution(sensorObservation[1], expectedObservation[1],
                                                            particleParameters_.elevStdDev);

        particles_.weight_[i] *= azimuthLikelihood * elevationLikelihood;
    }, "LOOP");

    TIME(particles_.normaliseWeights(), "NORMALISE");
}


//! We don't resample every iteration, instead we resample based on the effective number of samples.
//!     effectiveNumberOfSamples = 1 / sum((p.w ./ sum(p.w)).^2);
//!
void ParticleFilter::runResampling()
{
    unsigned int n = particles_.getNumberOfParticles();

    particles_.normaliseWeights();
    double sumOfWeights = particles_.weight_.sum();

    Eigen::VectorXd squaredWeights;
    squaredWeights.resize(n);

    for(size_t i=0; i<n; ++i)
    {
        squaredWeights[i] = pow(particles_.weight_[i], 2);
    }

    double effectiveNumberOfSamples = 1 / squaredWeights.sum();

    if(effectiveNumberOfSamples < particleParameters_.numOfEffectiveParticles)
    {
        runUniversalStochasticResampling();
    }
}


void ParticleFilter::runUniversalStochasticResampling()
{
    unsigned int n = particles_.getNumberOfParticles();
    particles_.normaliseWeights();

    // Since process is done without replacement, we need to assign the new values after the
    // resampling is complete to be later transferred into the particles_ data member.
    std::vector<double> newX;
    std::vector<double> newY;
    std::vector<double> newZ;
    std::vector<double> newWeight;

    newX.reserve(n);
    newY.reserve(n);
    newZ.reserve(n);
    newWeight.reserve(n);

    // Generate a single random double in the range 0 => (number of particles)^-1
    double randomValue = fRand(0, 1.0 / (double)n);

    double weight = particles_.weight_[0];
    int i = 0;
    double U = 0;

    for(int m=0; m < n; ++m)
    {
        U = randomValue + (double)m / (double)n;
        while(U > weight)
        {
            i = i + 1;
            weight = weight + particles_.weight_[i];
        }
        newX.push_back(particles_.x_[i]);
        newY.push_back(particles_.y_[i]);
        newZ.push_back(particles_.z_[i]);
    }

    for(size_t i=0; i < n; ++i)
    {
        particles_.x_[i] = newX[i];
        particles_.y_[i] = newY[i];
        particles_.z_[i] = newZ[i];
        particles_.weight_[i] = 1 / (double)n;
    }
}


void ParticleFilter::runSystematicSampling()
{
    unsigned int n = particles_.getNumberOfParticles();

    particles_.normaliseWeights();
    std::vector<double> cumulativeSummedWeights;
    cumulativeSummedWeights.reserve(n);

    std::vector<double> weightVector(particles_.weight_.data(),
                                     particles_.weight_.data() + particles_.weight_.rows() * particles_.weight_.cols());
    std::partial_sum(weightVector.begin(), weightVector.end(), cumulativeSummedWeights.begin());

    // Since process is done without replacement, we need to assign the new values after the
    // resampling is complete to be later transferred into the particles_ data member.
    Eigen::VectorXd newX;
    Eigen::VectorXd newY;
    Eigen::VectorXd newZ;
    Eigen::VectorXd newWeight;

    newX.resize(n);
    newY.resize(n);
    newZ.resize(n);
    newWeight.resize(n);

    for(size_t i=0; i<n; ++i)
    {
        // Generate a single random double in the range 0 => (number of particles)^-1
        std::uniform_real_distribution<double> dist(0, 1.0);
        std::mt19937 rng;
        rng.seed(std::random_device{}());
        double randomValue = dist(rng);

        std::vector<int> signVector;
        signVector.reserve(n);
        std::transform(cumulativeSummedWeights.begin(), cumulativeSummedWeights.end(), signVector.begin(),
                       std::bind(signOfInputDifference, std::placeholders::_1, randomValue));

        int id = n - 1;
        for(size_t j=0; j<n; ++j)
        {
            if(signVector[j] == 1)
            {
                id = j;
                break;
            }
        }

        newX[i] = particles_.x_(id);
        newY[i] = particles_.y_(id);
        newZ[i] = particles_.z_(id);
    }

    for(size_t i=0; i<n; ++i)
    {
        newWeight[i] = 1 / n;
    }

    particles_.x_ = newX;
    particles_.y_ = newY;
    particles_.z_ = newZ;
    particles_.weight_ = newWeight;
}


inline int ParticleFilter::signOfInputDifference(double input1, double input2)
{
    if(input1 > input2)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}


LandmarkMoments ParticleFilter::calculateMoments() const
// Sample mean and covariance calculated according to
// https://en.wikipedia.org/wiki/Sample_mean_and_covariance
{
    double n = particles_.getNumberOfParticles();

    double meanX = particles_.x_.sum() / n;
    double meanY = particles_.y_.sum() / n;
    double meanZ = particles_.z_.sum() / n;

    Eigen::Matrix<double,1,3> landmarkMean;
    landmarkMean << meanX, meanY, meanZ;

    Eigen::MatrixXd particleCoordinateMatrix;
    particleCoordinateMatrix.resize(n, 3);
    particleCoordinateMatrix.col(0) = particles_.x_;
    particleCoordinateMatrix.col(1) = particles_.y_;
    particleCoordinateMatrix.col(2) = particles_.z_;

    Eigen::VectorXd onesVector;
    onesVector.resize(n,1);
    onesVector << Eigen::VectorXd::Ones(n, 1);

    Eigen::Matrix<double,3,3> landmarkCovariance;
    landmarkCovariance = (particleCoordinateMatrix - onesVector * landmarkMean).transpose() *
                         (particleCoordinateMatrix - onesVector * landmarkMean) / (n - 1);

    LandmarkMoments landmarkMoments;
    landmarkMoments.landmarkMean = landmarkMean.transpose();
    landmarkMoments.landmarkCovariance = landmarkCovariance;

    return landmarkMoments;
}


Eigen::Vector2d ParticleFilter::calcExpectedBearings(unsigned int particleId, double robotX, double robotY,
                                                     double robotZ, double robotAzimuth)
{
    Eigen::Vector2d bearings;
    bearings[0] = normalizeAngleMinusPiToPi(atan2(particles_.y_[particleId] -  robotY,
                                                  particles_.x_[particleId] - robotX)
                                            - robotAzimuth);

    bearings[1] = atan2(particles_.z_[particleId] - robotZ,
                        sqrt(pow(particles_.x_[particleId] - robotX, 2)
                             + pow(particles_.y_[particleId] -  robotY, 2)
                            )
                       );

    return bearings;
}


const Particles& ParticleFilter::getConstParticles() const
{
    return particles_;
}


Particles& ParticleFilter::getParticles()
{
    return particles_;
}


void ParticleFilter::publishParticles() const
{
    sensor_msgs::PointCloud pointCloud;

    sensor_msgs::ChannelFloat32 channel;
    std_msgs::Float32MultiArray weights;

    pointCloud.header.frame_id = "map";

    for(size_t i=0; i<particles_.getNumberOfParticles(); ++i)
    {
        geometry_msgs::Point32 point;
        point.x = particles_.x_[i];
        point.y = particles_.y_[i];
        point.z = particles_.z_[i];

        channel.values.push_back(particles_.weight_[i]);

        pointCloud.points.push_back(point);
    }

    channel.name = "weights";
    pointCloud.channels.push_back(channel);
    particleCloudPublisher_.publish(pointCloud);
}
