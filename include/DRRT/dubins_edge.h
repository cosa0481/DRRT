#ifndef DUBINSEDGE_H
#define DUBINSEDGE_H

#include <DRRT/edge.h>

class ConfigSpace;

class DubinsEdge : public Edge
{
public:
    DubinsEdge(std::shared_ptr<Kdnode> start, std::shared_ptr<Kdnode> end)
        : Edge(start, end) {}

    DubinsEdge() : Edge() {}

    bool ValidMove();
    Eigen::VectorXd PoseAtDistAlongEdge(double dist_along_edge);
    void CalculateTrajectory(std::shared_ptr<ConfigSpace> cspace);
    void CalculateHoverTrajectory();
};

#endif // DUBINSEDGE_H
