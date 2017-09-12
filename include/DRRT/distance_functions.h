#ifndef DISTANCE_FUNCTIONS_H
#define DISTANCE_FUNCTIONS_H

#include <Eigen/Eigen>
#include <chrono>

double GetTimeNs(std::chrono::time_point<std::chrono::high_resolution_clock> start);

double DubinsDistance(Eigen::Vector3d a, Eigen::Vector3d b);

Eigen::Vector3d SaturateDubins(Eigen::Vector3d closest_point, double delta, double dist);

#endif // DISTANCE_FUNCTIONS_H
