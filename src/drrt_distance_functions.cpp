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

double R3Dist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    return sqrt( (x(0)-y(0))*(x(0)-y(0)) + (x(1)-y(1))*(x(1)-y(1))
                 + (x(2)-y(2))*(x(2)-y(2)) + (x(3)-y(3))*(x(3)-y(3))
                 + (x(4)-y(4))*(x(4)-y(4)) + (x(5)-y(5))*(x(5)-y(5)) );
}

double R3SDist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    Eigen::ArrayXd temp = x.head(3) - y.head(3);
    temp = temp*temp;
    return sqrt( temp.sum() + pow( std::min( std::abs(x(3)-y(3)), std::min(x(3),y(3)) + 2.0*PI - std::max(x(3),y(3)) ), 2 ) );
}

double S3Dist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    double t1, t2, t3;
    t1 = std::min( std::abs(x(0)-y(0)), std::min(x(0),y(0)) + 1.0 - std::max(x(0),y(0)) );
    t2 = std::min( std::abs(x(1)-y(1)), std::min(x(1),y(1)) + 1.0 - std::max(x(1),y(1)) );
    t3 = std::min( std::abs(x(3)-y(3)), std::min(x(3),y(3)) + 2.0*PI - std::max(x(3),y(3)) );
    return sqrt( t1*t1 + t2*t2 + t3*t3 );
}

double S3KDSearchDist( Eigen::VectorXd x, Eigen::VectorXd y )
{
    return sqrt( (x(0)-y(0))*(x(0)-y(0)) + (x(1)-y(1))*(x(1)-y(1)) + (x(3)-y(3))*(x(3)-y(3)) );
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
