/* drrt_distance_functions.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/drrt_distance_functions.h>

double EuclideanDist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    Eigen::ArrayXd temp = x - y;
    temp = temp*temp;
    return sqrt(temp.sum());
}

double R3SDist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    Eigen::ArrayXd temp = x.head(3) - y.head(3);
    temp = temp*temp;
    return sqrt( temp.sum() + pow( std::min( std::abs(x(3)-y(3)), std::min(x(3),y(3)) + 2.0*PI - std::max(x(3),y(3)) ), 2 ) );
}

double dubinsDistAlongTimePath( Eigen::VectorXd x, Eigen::VectorXd y )
{
    Eigen::ArrayXd temp = x.head(3) - y.head(3);
    temp = temp*temp;
    return sqrt(temp.sum());
}

double dubinsDistAlongPath( Eigen::VectorXd x, Eigen::VectorXd y )
{
    Eigen::ArrayXd temp = x.head(2) - y.head(2);
    temp = temp*temp;
    return sqrt(temp.sum());
}

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
