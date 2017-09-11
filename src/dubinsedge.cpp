#include <DRRT/dubinsedge.h>
#include <DRRT/kdnode.h>

Edge_ptr Edge::NewEdge(Kdnode_ptr start, Kdnode_ptr end)
{
    Edge_ptr new_edge = std::make_shared<DubinsEdge>(start, end);
    new_edge->SetDist(DistanceFunction(start->GetPosition(), end->GetPosition()));
    return new_edge;
}

bool DubinsEdge::ValidMove()
{
    return true;
}

Eigen::VectorXd DubinsEdge::PoseAtDistAlongEdge(double dist_along_edge)
{
    Eigen::VectorXd vec;
    vec.resize(NUM_DIM);
    vec(0) = 0;  // x-coordinate
    vec(1) = 0;  // y-coordinate
    vec(2) = PI;
    return vec;
}

void DubinsEdge::CalculateTrajectory()
{

}

void DubinsEdge::CalculateHoverTrajectory()
{

}

bool DubinsEdge::ExplicitEdgeCheck(std::shared_ptr<Obstacle> obstacle)
{
    return false;
}
