#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <DRRT/obstacle.h>
#include <DRRT/edge.h>
#include <DRRT/kdnode_listnode.h>
#include <DRRT/kdtree.h>

class ConfigSpace;

// These collision detection functions are for Dubin's Space

KdnodeList_ptr FindPointsInConflictWithObstacle(KdTree_ptr tree, Obstacle_ptr obs);
bool DetectCollision(Obstacle_ptr &obs, Eigen::VectorXd start, Eigen::VectorXd end);
bool LineCheck(Kdnode_ptr node1, Kdnode_ptr node2, std::shared_ptr<ConfigSpace> cspace);
bool NodeCheck(std::shared_ptr<ConfigSpace> cspace, Kdnode_ptr node);
bool PointCheck(std::shared_ptr<ConfigSpace> cspace, Eigen::VectorXd point);
bool PointCheck2D(std::shared_ptr<ConfigSpace> cspace, Eigen::Vector2d point, Obstacle_ptr obs);
bool EdgeCheck(Obstacle_ptr obstacle, Edge_ptr edge);
bool EdgeCheck(std::shared_ptr<ConfigSpace> cspace, Edge_ptr edge);
bool EdgeCheck2D(std::shared_ptr<ConfigSpace> cspace, Edge_ptr edge);
bool QuickCheck(std::shared_ptr<ConfigSpace> cspace, Eigen::VectorXd point);
bool QuickCheck2D(std::shared_ptr<ConfigSpace> cspace, Eigen::Vector2d point);

#endif // COLLISION_DETECTION_H
