/* drrt_distance_functions.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/drrt_distance_functions.h>

double DubinsDistAlongTimePath(Eigen::VectorXd x, Eigen::VectorXd y)
{
    Eigen::ArrayXd temp = x.head(3) - y.head(3);
    temp = temp*temp;
    return sqrt(temp.sum());
}

double DubinsDistAlongPath(Eigen::VectorXd x, Eigen::VectorXd y)
{
    Eigen::ArrayXd temp = x.head(2) - y.head(2);
    temp = temp*temp;
    return sqrt(temp.sum());
}

double RightTurnDist(Eigen::VectorXd pointA, Eigen::VectorXd pointB,
                     Eigen::VectorXd circleCenter, double r)
{
    double theta = atan2(pointA(1)-circleCenter(1), pointA(0)-circleCenter(0))
            - atan2(pointB(1)-circleCenter(1), pointB(0)-circleCenter(0));
    if( theta < 0 ) {
        theta += 2*3.1415926536;
    }
    return theta*r;
}

double LeftTurnDist(Eigen::VectorXd pointA, Eigen::VectorXd pointB,
                    Eigen::VectorXd circleCenter, double r)
{
    double theta = atan2(pointB(1)-circleCenter(1), pointB(0)-circleCenter(0))
            - atan2(pointA(1)-circleCenter(1), pointA(0)-circleCenter(0));
    if( theta < 0 ) {
        theta += 2*3.1415926536;
    }
    return theta*r;
}
