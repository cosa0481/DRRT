#ifndef DUBINSEDGE_H
#define DUBINSEDGE_H

// #include <DRRT/edge.h> to make a new type of edge
#include <DRRT/edge.h>

// #include this file in drrt_data_structures.h
// Remember to implement Edge::newEdge(Eigen::VectorXd,Eigen::VectorXd)

class DubinsEdge : public Edge
{
public:
    DubinsEdge()
        : Edge() {}

    DubinsEdge(std::shared_ptr<KDTreeNode> start,
               std::shared_ptr<KDTreeNode> end)
        : Edge(start,end) {}

    bool validMove(std::shared_ptr<CSpace> S);
    Eigen::VectorXd poseAtDistAlongEdge(double distAlongEdge);
    Eigen::VectorXd poseAtTimeAlongEdge(double timeAlongEdge);
    void calculateTrajectory(std::shared_ptr<CSpace> S,
                             std::shared_ptr<KDTree> Tree);
    void calculateHoverTrajectory(std::shared_ptr<CSpace> S);
};

#endif // DUBINSEDGE_H
