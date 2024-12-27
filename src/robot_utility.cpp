#include <random>
#include "EKF.h"

// Method to generate a Gaussian random number with mean 0 and standard deviation for noise
double RobotUtility::randomGaussian(double mean, double std) {
    std::random_device rd;  // Random device to seed the generator
    std::mt19937 gen(rd()); // Mersenne Twister random number generator
    std::normal_distribution<> dist(mean, std); // Gaussian distribution with mean=noise, stddev=noise
    return dist(gen); // Generate and return the random value
}

// Function to calculate distance
double RobotUtility::distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
}