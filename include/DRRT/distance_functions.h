#ifndef DISTANCE_FUNCTIONS_H
#define DISTANCE_FUNCTIONS_H

#include <Eigen/Eigen>
#include <chrono>

class Region2D;
typedef Region2D Region;

double GetTimeNs(std::chrono::time_point<std::chrono::high_resolution_clock> start);

double DubinsDistance(Eigen::Vector3d a, Eigen::Vector3d b);

// Set the Saturate function in kdnode.h
Eigen::Vector3d SaturateDubins(Eigen::Vector3d closest_point, double delta, double dist);

double DistToPolygonSqrd(Eigen::VectorXd point, Region polygon);

double DistPointToSegmentSqrd(Eigen::VectorXd point, Eigen::VectorXd start, Eigen::VectorXd end);

double DistSegmentSqrd(Eigen::VectorXd PA, Eigen::VectorXd PB,
                       Eigen::VectorXd QA, Eigen::VectorXd QB);

double EuclideanDistance2D(Eigen::Vector2d a, Eigen::Vector2d b);

#endif // DISTANCE_FUNCTIONS_H
