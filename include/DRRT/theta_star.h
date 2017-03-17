#ifndef THETA_STAR_H
#define THETA_STAR_H

#include <DRRT/drrt.h>

using namespace std;

double dist_func(Eigen::VectorXd a, Eigen::VectorXd b);

std::vector<Eigen::VectorXd> theta_star(std::shared_ptr<CSpace> C);

bool update_vertex(std::shared_ptr<CSpace> C,
                   std::shared_ptr<KDTree> Tree,
                   std::shared_ptr<KDTreeNode> &node,
                   std::shared_ptr<KDTreeNode> &neighbor);

std::vector<Eigen::VectorXd> get_path(std::shared_ptr<KDTreeNode> &node);

#endif // THETA_STAR_H
