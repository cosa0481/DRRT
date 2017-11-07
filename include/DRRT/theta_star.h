#ifndef THETA_STAR_H
#define THETA_STAR_H

#include <DRRT/libraries.h>

class ConfigSpace;
class Kdnode;

double ThetaStarDistance(Eigen::VectorXd a, Eigen::VectorXd b);
std::vector<double> PathToThetas(std::vector<Eigen::VectorXd> path);
std::vector<Eigen::VectorXd> GetPath(std::shared_ptr<Kdnode> node);
bool UpdateVertex(std::shared_ptr<ConfigSpace> cspace,
                  std::shared_ptr<Kdnode> &node, std::shared_ptr<Kdnode> &neighbor,
                  std::shared_ptr<Kdnode> &min_neighbor);
std::vector<Eigen::VectorXd> ThetaStar(std::shared_ptr<ConfigSpace> cspace);

#endif // THETA_STAR_H
