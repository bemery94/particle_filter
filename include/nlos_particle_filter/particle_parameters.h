#ifndef NLOS_PARTICLE_FILTER_PARTICLE_PARAMETERS_H
#define NLOS_PARTICLE_FILTER_PARTICLE_PARAMETERS_H

//! Struct for storing parameters related to particle filter.
//!
//! @author Brendan Emery
//! @date Jan 2017
//! @version 1.0.0
//! @bug Currently no known bugs.
//! @todo Currently no todos
//! @nosubgrouping
struct ParticleParameters {
        ParticleParameters(double kLDistanceThreshIn, double azStdDevIn, double elevStdDevIn,
                           double numOfEffectiveParticlesIn, double particleFilterNoiseIn,
                           unsigned int numberOfParticlesIn, double mapSizeIn) :
                kLDistanceThresh(kLDistanceThreshIn), azStdDev(azStdDevIn),
                elevStdDev(elevStdDevIn), numOfEffectiveParticles(numOfEffectiveParticlesIn),
                particleFilterNoise(particleFilterNoiseIn), numberOfParticles(numberOfParticlesIn),
                                    mapSize(mapSizeIn){};

    double kLDistanceThresh;
    double azStdDev;
    double elevStdDev;
    double numOfEffectiveParticles;
    double particleFilterNoise;
    unsigned int numberOfParticles;
    double mapSize;
};

#endif //NLOS_PARTICLE_FILTER_PARTICLE_PARAMETERS_H
