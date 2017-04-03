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

    HolEdge(std::shared_ptr<ConfigSpace> ConfigSpace,
            std::shared_ptr<KDTree> tree_,
            std::shared_ptr<KDTreeNode> start,
            std::shared_ptr<KDTreeNode> end)
        : Edge(ConfigSpace,tree_,start,end) {}

    bool ValidMove();
    Eigen::VectorXd PoseAtDistAlongEdge(double distAlongEdge);
    Eigen::VectorXd PoseAtTimeAlongEdge(double timeAlongEdge);
    void CalculateTrajectory();
    void CalculateHoverTrajectory();
    bool ExplicitEdgeCheck(std::shared_ptr<Obstacle> obstacle);
};

#endif // HOLEDGE_H
