#include "nlos_particle_filter/particles_class.h"

Particles::Particles(std::array<double, 3> initPositionEstimate, unsigned int numberOfParticles,
                     double mapSize) :
        numberOfParticles_(numberOfParticles)
{
    x_.resize(numberOfParticles);
    y_.resize(numberOfParticles);
    z_.resize(numberOfParticles);

    velX_.resize(numberOfParticles);
    velY_.resize(numberOfParticles);
    velZ_.resize(numberOfParticles);

    weight_.resize(numberOfParticles);

    double normalisedWeight = 1 / (double)numberOfParticles;

    // Randomly generate particlesNo number of particles evenly in a cube around the robot.
    for(unsigned int i=0; i < numberOfParticles; ++i)
    {
        // Generate a random double
        std::uniform_real_distribution<double> dist(-1, 1);
        std::mt19937 rng;

        double randX;
        double randY;
        double randZ;

        // Calculate random coordinates for the particles centered around the landmark estimate in
        // range -mapSize to mapSize.
        rng.seed(std::random_device{}());
        randX = dist(rng) * mapSize / 2 + initPositionEstimate[0];

        rng.seed(std::random_device{}());
        randY = dist(rng) * mapSize / 2 + initPositionEstimate[1];

        rng.seed(std::random_device{}());
        randZ = dist(rng) * mapSize / 2 + initPositionEstimate[2];

        x_[i] = randX;
        y_[i] = randY;
        z_[i] = randZ;
        weight_[i] = normalisedWeight;
    }
};

void Particles::normaliseWeights()
{
    unsigned int n = getNumberOfParticles();
    double sumOfWeight = weight_.sum();

    for(unsigned int i=0; i < n; ++i)
    {
        weight_[i] /= sumOfWeight;
    }
};

unsigned int Particles::getNumberOfParticles() const
{
    return numberOfParticles_;
};
