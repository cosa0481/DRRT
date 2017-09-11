#ifndef DUBINSEDGE_H
#define DUBINSEDGE_H

#include <DRRT/edge.h>

class DubinsEdge : public Edge
{
public:
    DubinsEdge(std::shared_ptr<Kdnode> start, std::shared_ptr<Kdnode> end)
        : Edge(start, end) {}

    DubinsEdge() : Edge() {}

    bool ValidMove();
    Eigen::VectorXd PoseAtDistAlongEdge(double dist_along_edge);
    void CalculateTrajectory();
    void CalculateHoverTrajectory();
    bool ExplicitEdgeCheck(std::shared_ptr<Obstacle> obstacle);
};

#endif // DUBINSEDGE_H
