#ifndef THETA_STAR_H
#define THETA_STAR_H

#include <DRRT/drrt.h>

double DistFunc(Eigen::VectorXd a, Eigen::VectorXd b);

std::vector<Eigen::VectorXd> ThetaStar(std::shared_ptr<Queue> Q);

bool UpdateVertex(std::shared_ptr<Queue> Q,
                  std::shared_ptr<KDTree> Tree,
                  std::shared_ptr<KDTreeNode> &node,
                  std::shared_ptr<KDTreeNode> &neighbor,
                  std::shared_ptr<KDTreeNode> &min_neighbor);

std::vector<Eigen::VectorXd> GetPath(std::shared_ptr<KDTreeNode> &node);

#endif // THETA_STAR_H
