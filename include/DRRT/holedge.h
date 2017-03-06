/* holedge.h
 * Corin Sandford
 * Holonomic edge model
 * Can test optimality of distance function
 */

#ifndef HOLEDGE_H
#define HOLEDGE_H

#include <DRRT/edge.h>

class HolEdge : public Edge
{
public:
    // Constructors
    HolEdge()
        : Edge() {}

    HolEdge(std::shared_ptr<CSpace> cspace,
            std::shared_ptr<KDTree> tree,
            std::shared_ptr<KDTreeNode> start,
            std::shared_ptr<KDTreeNode> end)
        : Edge(cspace,tree,start,end) {}

    bool ValidMove();
    Eigen::VectorXd poseAtDistAlongEdge(double distAlongEdge);
    Eigen::VectorXd poseAtTimeAlongEdge(double timeAlongEdge);
    void calculateTrajectory();
    void calculateHoverTrajectory();
    bool ExplicitEdgeCheck(std::shared_ptr<CSpace> S,
                           std::shared_ptr<Obstacle> obstacle);
};

#endif // HOLEDGE_H
