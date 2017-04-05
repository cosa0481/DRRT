/* distancefunctions.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/distancefunctions.h>

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

double RightTurnDist(Eigen::VectorXd point_a, Eigen::VectorXd point_b,
                     Eigen::VectorXd circle_center, double r)
{
    double theta = atan2(point_a(1)-circle_center(1),
                         point_a(0)-circle_center(0))
            - atan2(point_b(1)-circle_center(1), point_b(0)-circle_center(0));
    if( theta < 0 ) {
        theta += 2*3.1415926536;
    }
    return theta*r;
}

double LeftTurnDist(Eigen::VectorXd point_a, Eigen::VectorXd point_b,
                    Eigen::VectorXd circle_center, double r)
{
    double theta = atan2(point_b(1)-circle_center(1),
                         point_b(0)-circle_center(0))
            - atan2(point_a(1)-circle_center(1), point_a(0)-circle_center(0));
    if( theta < 0 ) {
        theta += 2*3.1415926536;
    }
    return theta*r;
}
