#include <DRRT/distance_functions.h>
#include <DRRT/libraries.h>
#include <DRRT/region.h>
#include <DRRT/kdnode.h>
#include <DRRT/edge.h>

double GetTimeNs(std::chrono::time_point<std::chrono::high_resolution_clock> start)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now() - start).count();
}

double DubinsDistance(Eigen::Vector3d a, Eigen::Vector3d b)
{
    Eigen::Array2d temp = a.head(2) - b.head(2);
    temp = temp*temp;
    double dist = sqrt(temp.sum()
                + pow(std::min(std::abs(a(2) - b(2)),
                               std::min(a(2), b(2)) + 2.0*3.1415926536 - std::max(a(2), b(2))),
                      2));
    return dist;
}

double RightTurnDist(Eigen::Vector2d a, Eigen::Vector2d b,
                     Eigen::Vector2d circle_center, double r)
{
    double theta = std::atan2(a(1) - circle_center(1),
                              a(0) - circle_center(0))
                 - std::atan2(b(1) - circle_center(1),
                              b(0) - circle_center(0));

    while(theta < 0) theta += 2*PI;

    return theta*r;
}

double LeftTurnDist(Eigen::Vector2d a, Eigen::Vector2d b,
                    Eigen::Vector2d circle_center, double r)
{
    double theta = std::atan2(b(1) - circle_center(1),
                              b(0) - circle_center(0))
                 - std::atan2(a(1) - circle_center(1),
                              a(0) - circle_center(0));

    while(theta < 0) theta += 2*PI;

    return theta*r;
}

Eigen::Vector3d SaturateDubins(Eigen::Vector3d position,
                               Eigen::Vector3d closest_point,
                               double delta, double dist)
{
    Eigen::Vector3d saturated_point;
    saturated_point.head(2) = closest_point.head(2)
            + (position.head(2) - closest_point.head(2))*delta/dist;

    saturated_point(2) = position(2);
    while(saturated_point(2) < 0.0) saturated_point(2) += 2.0*PI;
    while(saturated_point(2) > 2.0*PI) saturated_point(2) -= 2.0*PI;
    return saturated_point;
}

double PathLength(std::shared_ptr<Kdnode> node, std::shared_ptr<Kdnode> root)
{
    double length = 0.0;
    Kdnode_ptr current = node;
    Edge_ptr edge_to_parent;
    while(node != root) {
        if(!node->ParentExist()) return INF;
        current->GetRrtParentEdge(edge_to_parent);
        length += edge_to_parent->GetDist();
        current = edge_to_parent->GetEnd();
    }

    return length;
}

double DistToPolygonSqrd(Eigen::VectorXd point, Region polygon)
{
    double min_dist_sqrd = 1000000000000;

    // Start with last vs first point
    Eigen::MatrixXd shape = polygon.GetRegion();
    shape.resize(polygon.GetRegion().rows(), polygon.GetRegion().cols());
    Eigen::VectorXd start = shape.row(shape.rows() - 1);
    start.resize(shape.cols());
    Eigen::VectorXd end;
    end.resize(shape.cols());
    double dist_sqrd;
    for(int i = 0; i < shape.rows(); i++) {
        end = shape.row(i);
        dist_sqrd = DistPointToSegmentSqrd(point, start, end);
        if(dist_sqrd < min_dist_sqrd) min_dist_sqrd = dist_sqrd;
        start = end;
    }
    return min_dist_sqrd;

}

double DistPointToSegmentSqrd(Eigen::VectorXd point, Eigen::VectorXd start,
                              Eigen::VectorXd end)
{
    Eigen::Vector2d pos = point.head(2);
    double vx = pos(0) - start(0);
    double vy = pos(1) - start(1);
    double ux = end(0) - start(0);
    double uy = end(1) - start(1);
    double determinate = vx*ux + vy*uy;

    if(determinate <= 0) return vx*vx + vy*vy;
    else {
        double length = ux*ux + uy*uy;
        if(determinate >= length)
            return (end(0) - pos(0)) * (end(0)-pos(0))
                    + (end(1) - pos(1)) * (end(1) - pos(1));
        else
            return (ux*vy - uy*vx) * (ux*vy - uy*vx) / length;
    }
}

double DistSegmentSqrd(Eigen::VectorXd PA, Eigen::VectorXd PB,
                       Eigen::VectorXd QA, Eigen::VectorXd QB)
{
    bool possible_intersect = true;
    double m, diffA, diffB;
    double tolerance = 0.000001;

    // Check if P is close to vertical
    if(std::abs(PB(0) - PA(0)) < tolerance) {
        // Check if Q is on one sde of P
        if((QA(0) >= PA(0)) && (QB(0) >= PA(0))) possible_intersect = false;
    } else {
        m = (PB(1) - PA(1)) / (PB(0) - PA(0));

        // Equation for points on P: y = m(x - PA(0)) + PA(1)
        diffA = (m*(QA(0) - PA(0)) + PA(1)) - QA(1);
        diffB = (m*(QB(0) - PA(0)) + PA(1)) - QB(1);
        // Check if Q is either fully above or below the line containing P
        if((diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0))
            possible_intersect = false;
    }

    if(possible_intersect) {
        // Check if Q is close to vertical
        if(std::abs(QB(0) - QA(0)) < tolerance) {
            if((PA(0) >= QA(0) && PB(0) >= QA(0))
                    || (PA(0) <= QA(0) && PB(0) <= QA(0)))
                possible_intersect = false;
        } else {
            m = (QB(1) - QA(1)) / (QB(0) - QA(0));

            // Equation for points on Q: y = m(x - QA(0)) + QA(1)
            diffA = (m*(PA(0) - QA(0)) + QA(1)) - PA(1);
            diffB = (m*(PB(0) - QA(0)) + QA(1)) - PB(1);
            // Check if P is either fully above or below the line containing Q
            if((diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0))
                possible_intersect = false;
        }
    }

    if(possible_intersect) return 0.0;

    // If the lines do not intersect in 2D the minimum distance
    // is between one segments endpoint and the orthe segment
    // ASSUMPTION: lines are not parallel
    Eigen::Vector4d distances;
    distances(0) = DistPointToSegmentSqrd(PA, QA, QB);
    distances(1) = DistPointToSegmentSqrd(PB, QA, QB);
    distances(2) = DistPointToSegmentSqrd(QA, PA, PB);
    distances(3) = DistPointToSegmentSqrd(QB, PA, PB);

    return distances.minCoeff();
}

double EuclideanDistance2D(Eigen::Vector2d a, Eigen::Vector2d b)
{
    return sqrt(pow(a(0) - b(0), 2) + pow(a(1) - b(1), 2));
}
