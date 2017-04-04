#ifndef DRRT_DISTANCE_FUNCTIONS_H
#define DRRT_DISTANCE_FUNCTIONS_H

#include <vector>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>

/* Returns the distance between two points in the projection of a Dubin's space
 * that contains [X Y T] depending on if time is being used or not.
 * This is useful for calculating the distance between two points that are close
 * to each other on the same Dubin's path. e.g. when change in theta is small
 * and so the theta component can be ignored
 */
double DubinsDistAlongTimePath( Eigen::VectorXd x, Eigen::VectorXd y );

/* Returns the distance between two points in the projection of a Dubin's space
 * that contains [X Y] depending on if time is being used or not.
 * This is useful for calculating the distance between two points that are close
 * to each other on the same Dubin's path. e.g. when change in theta is small
 * and so the theta component can be ignored
 */
double DubinsDistAlongPath( Eigen::VectorXd x, Eigen::VectorXd y );

/* Helps with Dubin's car
 * Returns the distance that the car travels to get from
 * pointA to pointB, where both are assumed to be on the
 * circle of radius r that is centered at circleCenter
 */
double RightTurnDist( Eigen::VectorXd point_a, Eigen::VectorXd point_b,
                      Eigen::VectorXd circle_center, double r );

/* Helps with Dubin's car
 * Returns the distance that the car travels to get from
 * pointA to pointB, where both are assumed to be on the
 * circle of radius r that is centered at circleCenter
 */
double LeftTurnDist(Eigen::VectorXd point_a, Eigen::VectorXd point_b,
                     Eigen::VectorXd circle_center, double r );

#endif // DRRT_DISTANCE_FUNCTIONS_H
