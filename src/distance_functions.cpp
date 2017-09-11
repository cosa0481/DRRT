#include <DRRT/distance_functions.h>

double DubinsDistance(Eigen::Vector3d a, Eigen::Vector3d b)
{
    Eigen::Array2d temp = a.head(2) - b.head(2);
    temp = temp*temp;
    return sqrt(temp.sum()
                + pow(std::min(std::abs(a(2) - b(2)),
                               std::min(a(2), b(2) + 2.0*3.1415926536 - std::max(a(2), b(2)))),
                      2));
}

Eigen::Vector3d SaturateDubins(Eigen::Vector3d closest_point, double delta, double dist)
{
    Eigen::Vector3d saturated_point;
    saturated_point.head(2) = closest_point.head(2)
            + (position_.head(2) - closest_point.head(2))*delta/dist;
    while(saturated_point(2) < 0.0) saturated_point(2) += 2.0*PI;
    while(saturated_point(2) > 2.0*3.1415926536) saturated_point(2) -= 2.0*3.1415926536;
    return saturated_point;
}
