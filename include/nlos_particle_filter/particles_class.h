#ifndef PROJECT_PARTICLES_H
#define PROJECT_PARTICLES_H

#include <vector>
#include <iostream>
#include <random>
#include <ctime>
#include "Eigen/Dense"

//! Class containing particles
//!
//! @author Brendan Emery
//! @date Oct 2016
//! @version 1.0.0
//! @bug Currently no known bugs.
//! @todo Currently no todos
//! @nosubgrouping
class Particles {
    public:
        // Member functions
        //! Constructor to initialise particle vectors and generate a random sample of
        //! particles evenly in a cube around the robot in the range of -mapSize to mapSize.
        //!
        //! @param initPositionEstimate is the coordinates of the landmark estimate's position
        //! in the world.
        //! @param numberOfParticles is the number of particles to be initialized
        //! @param mapSize is the dimensions of the cubic map (in metres) in which to initialize the
        //! particles in.
        Particles(std::array<double, 3> initPositionEstimate, unsigned int numberOfParticles,
                  double mapSize);

        //! Normalise particle weights so that all weight sum to 1.
        void normaliseWeights();

        //! Get the number of particles
        unsigned int getNumberOfParticles() const;


        // Data members
        Eigen::VectorXd x_;
        Eigen::VectorXd y_;
        Eigen::VectorXd z_;

        Eigen::VectorXd velX_;
        Eigen::VectorXd velY_;
        Eigen::VectorXd velZ_;

        Eigen::VectorXd weight_;
    private:
        unsigned int numberOfParticles_;
};

#endif //PROJECT_PARTICLES_H
