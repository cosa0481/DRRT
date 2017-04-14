/* sampling.h
 * Corin Sandford
 * Spring 2017
 * C-Space sampling functions
 */

#ifndef SAMPLING_H
#define SAMPLING_H

#include <DRRT/kdtree.h>

// Returns a random double between min and max
double RandDouble( double min, double max );

/////////////////////// C-Space Functions ///////////////////////
// Functions that interact in C-Space, including sampling functions

// Returns a random point in S
Eigen::VectorXd RandPointDefault(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S
std::shared_ptr<KDTreeNode> RandNodeDefault(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, or the goal with probability prob_goal_
std::shared_ptr<KDTreeNode> RandNodeOrGoal(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, but when iteration_sample_point_ == 0
// it returns iteration_sample_point_ instead
std::shared_ptr<KDTreeNode> RandNodeIts(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, but when wait_time_ has passed it returns
// time_sample_point_ instead
std::shared_ptr<KDTreeNode> RandNodeTime(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, but when wait_time_ has passed it returns
// time_sample_point_ instead, also sets the first obstacle to unused
//std::shared_ptr<KDTreeNode> randNodeTimeWithObstacleRemove(
//        std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, but when wait_time_ has passed it returns
// time_sample_point_ instead, also sets the first obstacle to unused
//std::shared_ptr<KDTreeNode> randNodeItsWithObstacleRemove(
//        std::shared_ptr<ConfigSpace> C);

// Returns a random node unless there are points in the sample stack,
// in which case it returns the first one of those
std::shared_ptr<KDTreeNode> RandNodeOrFromStack(std::shared_ptr<ConfigSpace> &C);

/* This returns a random node where the time dimension is drawn uniformly
 * at random from (the min time the robot could reach the point in an
 * obstacle-less environment traveling at max speed) and (current move time)
 * unless there are points in the sample stack, in which case it returns
 * the first one of those
 */
std::shared_ptr<KDTreeNode> RandNodeInTimeOrFromStack(
        std::shared_ptr<ConfigSpace> C);

#endif // SAMPLING_H
