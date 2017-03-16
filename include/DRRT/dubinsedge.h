#ifndef DUBINSEDGE_H
#define DUBINSEDGE_H

// #include <DRRT/edge.h> to make a new type of edge
#include <DRRT/edge.h>

// #include this file in drrt_data_structures.h
// Remember to implement Edge::newEdge(Eigen::VectorXd,Eigen::VectorXd)

class DubinsEdge : public Edge
{
public:
    // Constructors
    DubinsEdge()
        : Edge() {}

    DubinsEdge(std::shared_ptr<CSpace> cspace,
               std::shared_ptr<KDTree> tree,
               std::shared_ptr<KDTreeNode> start,
               std::shared_ptr<KDTreeNode> end)
        : Edge(cspace,tree,start,end) {}

    bool ValidMove();
    Eigen::VectorXd poseAtDistAlongEdge(double distAlongEdge);
    Eigen::VectorXd poseAtTimeAlongEdge(double timeAlongEdge);
    void calculateTrajectory();
    void calculateHoverTrajectory();
    bool ExplicitEdgeCheck(std::shared_ptr<Obstacle> obstacle);
};

#endif // DUBINSEDGE_H
