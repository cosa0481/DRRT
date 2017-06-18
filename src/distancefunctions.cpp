/* distancefunctions.cpp
 * Corin Sandford
 * Fall 2016
 */

#include <DRRT/distancefunctions.h>

using namespace std;

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

/////////////////////// Geometric Functions ///////////////////////

double DistanceSqrdPointToSegment(Eigen::VectorXd point,
                                  Eigen::Vector2d start_point,
                                  Eigen::Vector2d end_point)
{
    Eigen::Vector2d point_position = point.head(2);
    double vx = point_position(0) - start_point(0);
    double vy = point_position(1) - start_point(1);
    double ux = end_point(0) - start_point(0);
    double uy = end_point(1) - start_point(1);
    double determinate = vx*ux + vy*uy;

    if( determinate <= 0 ) {
        return vx*vx + vy*vy;
    } else {
        double len = ux*ux + uy*uy;
        if( determinate >= len ) {
            return (end_point(0)-point_position(0))
                    *(end_point(0)-point_position(0))
                    + (end_point(1)-point_position(1))
                    *(end_point(1)-point_position(1));
        } else {
            return (ux*vy - uy*vx)*(ux*vy - uy*vx) / len;
        }
    }
}


double DistToPolygonSqrd(Eigen::VectorXd point, Eigen::MatrixX2d polygon)
{
    double min_dist_sqrd = INF;

    // Start with the last vs first point
    Eigen::Vector2d start_point = polygon.row(polygon.rows()-1);
    Eigen::Vector2d end_point;
    double this_dist_sqrd;
    for(int i = 0; i < polygon.rows(); i++) {
        end_point = polygon.row(i);
        this_dist_sqrd = DistanceSqrdPointToSegment(point, start_point, end_point);
        if(this_dist_sqrd < min_dist_sqrd) min_dist_sqrd = this_dist_sqrd;
        start_point = end_point;
    }
    return min_dist_sqrd;
}

double SegmentDistSqrd(Eigen::VectorXd PA, Eigen::VectorXd PB,
                       Eigen::VectorXd QA, Eigen::VectorXd QB)
{
    // Check if the points are definately not in collision by seeing
    // if both points of Q are on the same side of line containing P and vice versa

    bool possibleIntersect = true;
    double m, diffA, diffB;

    // First check if P is close to vertical
    if( abs(PB(0) - PA(0)) < 0.000001 ) {
        // P is close to vertical
        if( (QA(0) >= PA(0) && QB(0) >= PA(0))
                || (QA(0) <= PA(0) && QB(0) <= PA(0)) ) {
            // Q is on one side of P
            possibleIntersect = false;
        }
    } else {
        // P is not close to vertical
        m = (PB(1) - PA(1)) / (PB(0) - PA(0));

        // Equation for points on P: y = m(x - PA[1]) + PA[2]
        diffA = (m*(QA(0)-PA(0)) + PA(1)) - QA(1);
        diffB = (m*(QB(0)-PA(0)) + PA(1)) - QB(1);
        if( (diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0) ) {
            // Q is either fully above or below the line containing P
            possibleIntersect = false;
        }
    }

    if( possibleIntersect ) {
        // first check if Q is close to vertical
        if( abs(QB(0) - QA(0)) < 0.000001 ) {
            if( (PA(0) >= QA(0) && PB(0) >= QA(0)) || (PA(0) <= QA(0) && PB(0) <= QA(0)) ) {
                // P is on one side of Q
                possibleIntersect = false;
            }
        } else {
            // Q is not close to vertical
            m = (QB(1) - QA(1)) / (QB(0) - QA(0));

            // Equation for points on Q: y = m(x-QA[1]) + QA[2]
            diffA = (m*(PA(0)-QA(0)) + QA(1)) - PA(1);
            diffB = (m*(PB(0)-QA(0)) + QA(1)) - PB(1);
            if( (diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0) ) {
                // P is either fully above or below the line containing Q
                possibleIntersect = false;
            }
        }
    }

    if( possibleIntersect ) {
        // Then there is an intersection for sure
        return 0.0;
    }

    // When the lines do not intersect in 2D, the min distance must
    // be between one segment's end point and the other segment
    // (assuming lines are not parallel)
    Eigen::Vector4d distances;
    Eigen::Vector2d QA_pos = QA.head(2);
    Eigen::Vector2d QB_pos = QB.head(2);
    Eigen::Vector2d PA_pos = PA.head(2);
    Eigen::Vector2d PB_pos = PB.head(2);
    distances(0) = DistanceSqrdPointToSegment(PA,QA_pos,QB_pos);
    distances(1) = DistanceSqrdPointToSegment(PB,QA_pos,QB_pos);
    distances(2) = DistanceSqrdPointToSegment(QA,PA_pos,PB_pos);
    distances(3) = DistanceSqrdPointToSegment(QB,PA_pos,PB_pos);

    return distances.minCoeff();
}

bool PointInPolygon(Eigen::VectorXd this_point, Eigen::MatrixX2d polygon)
{
    Eigen::Vector2d point = this_point.head(2);
    // MacMartin crossings test
    if(polygon.rows() < 2) return false;

    int num_crossings = 0;
    Eigen::Vector2d start_point = polygon.row(polygon.rows()-1);
    Eigen::Vector2d end_point;
    double x;
    for(int i = 0; i < polygon.rows(); i++) {
        end_point = polygon.row(i);

        // Check if edge crosses the y-value of the point
        if((start_point(1) > point(1) && end_point(1) < point(1))
           || (start_point(1) < point(1) && end_point(1) > point(1))) {
            // It does, no check if ray from point -> (INF,0) intersects
            if(start_point(0) > point(0) && end_point(0) > point(0)) {
                // Definitely yes if both x coordinates are right of the point
                num_crossings++;
            } else if( start_point(0) < point(0) && end_point(0) < point(0)) {
                // Definitely not if both x coordinates are left of the point
            } else { // Have to do "expensive" calculation
                double t = 2*max(start_point(0),end_point(0));
                x = (-((start_point(0)*end_point(1)
                        - start_point(1)*end_point(0))*(point(0)-t))
                     + ((start_point(0)-end_point(0)) * (point(0)*point(1)
                                                         -point(1)*t)))
                     / ((start_point(1)-end_point(1)) * (point(0)-t));

                if(x>point(0)) num_crossings++;
            }
        }
        start_point = end_point;
    }

    // Check crossings (odd means point inside polygon)
    if(num_crossings % 2 == 1) return true;
    return false;
}

double EuclideanDistance2D(Eigen::Vector2d a, Eigen::Vector2d b)
{
    return sqrt( pow(a(0) - b(0), 2) + pow(a(1) - b(1), 2) );
}
