#ifndef DRRT_DISTANCE_FUNCTIONS_H
#define DRRT_DISTANCE_FUNCTIONS_H

#include <vector>
#include <math.h>
#include <eigen3/Eigen/Eigen>

#define PI 3.1415926536

// Returns the Euclidian distance
double EuclideanDist( Eigen::VectorXd x, Eigen::VectorXd y )
{
//sqrt(sum((x-y).^2))
    Eigen::ArrayXd temp = x - y;
    temp = temp*temp;
    return sqrt(temp.sum());
}

// Returns the 'straight-line' distance in a space containing [X Y Time Theta]
// where theta exists on [0 2pi] and wraps around, i.e. 0 == 2pi
double R3SDist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    Eigen::ArrayXd temp = x.head(3) - y.head(3);
    temp = temp*temp;
    return sqrt( temp.sum() + pow( std::min( std::abs(x(3)-y(3)), std::min(x(3),y(3)) + 2.0*PI - std::max(x(3),y(3)) ), 2 ) );
}

/* Returns the distance between two points in the projection of a Dubin's space
 * that contains [X Y T] depending on if time is being used or not.
 * This is useful for calculating the distance between two points that are close
 * to each other on the same Dubin's path. e.g. when change in theta is small
 * and so the theta component can be ignored
 */
double dubinsDistAlongTimePath( Eigen::VectorXd x, Eigen::VectorXd y )
{
    Eigen::ArrayXd temp = x.head(3) - y.head(3);
    temp = temp*temp;
    return sqrt(temp.sum());
}

/* Returns the distance between two points in the projection of a Dubin's space
 * that contains [X Y] depending on if time is being used or not.
 * This is useful for calculating the distance between two points that are close
 * to each other on the same Dubin's path. e.g. when change in theta is small
 * and so the theta component can be ignored
 */
double dubinsDistAlongPath( Eigen::VectorXd x, Eigen::VectorXd y )
{
    Eigen::ArrayXd temp = x.head(2) - y.head(2);
    temp = temp*temp;
    return sqrt(temp.sum());
}

/* Helps with Dubin's car
 * Returns the distance that the car travels to get from
 * pointA to pointB, where both are assumed to be on the
 * circle of radius r that is centered at circleCenter
 */
double rightTurnDist( Eigen::VectorXd pointA, Eigen::VectorXd pointB,
                      Eigen::VectorXd circleCenter, double r )
{
    double theta = atan2(pointA(1)-circleCenter(1), pointA(0)-circleCenter(0))
            - atan2(pointB(1)-circleCenter(1), pointB(0)-circleCenter(0));
    if( theta < 0 ) {
        theta += 2*PI;
    }
    return theta*r;
}

/* Helps with Dubin's car
 * Returns the distance that the car travels to get from
 * pointA to pointB, where both are assumed to be on the
 * circle of radius r that is centered at circleCenter
 */
double leftTurnDist( Eigen::VectorXd pointA, Eigen::VectorXd pointB,
                     Eigen::VectorXd circleCenter, double r )
{
    double theta = atan2(pointB(1)-circleCenter(1), pointB(0)-circleCenter(0))
            - atan2(pointA(1)-circleCenter(1), pointA(0)-circleCenter(0));
    if( theta < 0 ) {
        theta += 2*PI;
    }
    return theta*r;
}

#endif // DRRT_DISTANCE_FUNCTIONS_H
