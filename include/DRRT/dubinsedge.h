#ifndef DUBINSEDGE_H
#define DUBINSEDGE_H

// #include <DRRT/edge.h> to make a new type of edge
#include <DRRT/edge.h>


// #include this file in datastructures.h
// Remember to implement Edge::NewEdge(Eigen::VectorXd,Eigen::VectorXd)

class DubinsEdge : public Edge
{
public:
    // Constructors
    DubinsEdge()
        : Edge() {}

    DubinsEdge(std::shared_ptr<ConfigSpace> C,
               std::shared_ptr<KDTree> Tree,
               std::shared_ptr<KDTreeNode> start,
               std::shared_ptr<KDTreeNode> end)
        : Edge(C,Tree,start,end) {}

    bool ValidMove();
    Eigen::VectorXd PoseAtDistAlongEdge(double dist_along_edge);
    Eigen::VectorXd PoseAtTimeAlongEdge(double time_along_edge);
    void CalculateTrajectory();
    void CalculateHoverTrajectory();
    bool ExplicitEdgeCheck(std::shared_ptr<Obstacle> obstacle);
};

#endif // DUBINSEDGE_H
