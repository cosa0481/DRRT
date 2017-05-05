#ifndef DRRT_DISTANCE_FUNCTIONS_H
#define DRRT_DISTANCE_FUNCTIONS_H

#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>
#include <iomanip>
#include <thread>
#include <mutex>
#include <string>
#include <random>
#include <algorithm>
#include <chrono>
#include <fstream>

#define PI 3.1415926536
#define INF 1000000000000 // Infinity value for distance
#define MICROSECOND 1000000 // for timing
#define MAXPATHNODES 1000000
#define DELTA 10 // should be changed if delta is changed in executable

/* Returns the distance between two points in the projection of a Dubin's space
 * that contains [X Y T] depending on if time is being used or not.
 * This is useful for calculating the distance between two points that are close
 * to each other on the same Dubin's path. e.g. when change in theta is small
 * and so the theta component can be ignored
 */
double DubinsDistAlongTimePath(Eigen::VectorXd x, Eigen::VectorXd y);

/* Returns the distance between two points in the projection of a Dubin's space
 * that contains [X Y] depending on if time is being used or not.
 * This is useful for calculating the distance between two points that are close
 * to each other on the same Dubin's path. e.g. when change in theta is small
 * and so the theta component can be ignored
 */
double DubinsDistAlongPath(Eigen::VectorXd x, Eigen::VectorXd y);

/* Helps with Dubin's car
 * Returns the distance that the car travels to get from
 * pointA to pointB, where both are assumed to be on the
 * circle of radius r that is centered at circleCenter
 */
double RightTurnDist(Eigen::VectorXd point_a, Eigen::VectorXd point_b,
                     Eigen::VectorXd circle_center, double r);

/* Helps with Dubin's car
 * Returns the distance that the car travels to get from
 * pointA to pointB, where both are assumed to be on the
 * circle of radius r that is centered at circleCenter
 */
double LeftTurnDist(Eigen::VectorXd point_a, Eigen::VectorXd point_b,
                    Eigen::VectorXd circle_center, double r);

/////////////////////// Geometric Functions ///////////////////////

// Returns the min distance squared between the point and the segment
// [startPoint, endPoint] assumes a 2D space
double DistanceSqrdPointToSegment(Eigen::VectorXd point,
                                  Eigen::Vector2d start_point,
                                  Eigen::Vector2d end_point);

// This returns the distance of the closest point on the boundary
// of the polygon to the point (assumes 2D space)
double DistToPolygonSqrd(Eigen::VectorXd point, Eigen::MatrixX2d polygon);

// All input args represent points, this returns the minimum distance
// between line segments [PA PB] and [QA QB] and assumes 2D space
double SegmentDistSqrd(Eigen::VectorXd PA, Eigen::VectorXd PB,
                       Eigen::VectorXd QA, Eigen::VectorXd QB);


// Returns true if the point is in the polygon (open set of it anyway)
// each row in polygon is a vertex and subsequent vertices define edges
// Top and bottom rows of polygon also form an edge
// Polygon does not have to be convex but should be simple
bool PointInPolygon(Eigen::VectorXd this_point, Eigen::MatrixX2d polygon);

#endif // DRRT_DISTANCE_FUNCTIONS_H
