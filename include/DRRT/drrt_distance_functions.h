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
double dubinsDistAlongTimePath( Eigen::VectorXd x, Eigen::VectorXd y );

/* Returns the distance between two points in the projection of a Dubin's space
 * that contains [X Y] depending on if time is being used or not.
 * This is useful for calculating the distance between two points that are close
 * to each other on the same Dubin's path. e.g. when change in theta is small
 * and so the theta component can be ignored
 */
double dubinsDistAlongPath( Eigen::VectorXd x, Eigen::VectorXd y );

/* Helps with Dubin's car
 * Returns the distance that the car travels to get from
 * pointA to pointB, where both are assumed to be on the
 * circle of radius r that is centered at circleCenter
 */
double rightTurnDist( Eigen::VectorXd pointA, Eigen::VectorXd pointB,
                      Eigen::VectorXd circleCenter, double r );

/* Helps with Dubin's car
 * Returns the distance that the car travels to get from
 * pointA to pointB, where both are assumed to be on the
 * circle of radius r that is centered at circleCenter
 */
double leftTurnDist( Eigen::VectorXd pointA, Eigen::VectorXd pointB,
                     Eigen::VectorXd circleCenter, double r );

#endif // DRRT_DISTANCE_FUNCTIONS_H
