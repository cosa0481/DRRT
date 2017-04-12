#ifndef MOVEROBOT_H
#define MOVEROBOT_H

#include <DRRT/drrt.h>

/* Move robot the distance that it would move in slice_time time
 * if time is not a dimension of the C-Space, then a contant velocity
 * is assumed. This also updates the moveGoal in the event that the robot
 * has lost connectivity with the graph due to dynamic obstacles breaking
 * the first edge of its path
 */
void MoveRobot(std::shared_ptr<Queue> &Q,
               std::shared_ptr<KDTree> &Tree,
               std::shared_ptr<KDTreeNode> &root,
               double slice_time,
               double hyperBallRad,
               std::shared_ptr<RobotData> &R);

// Thread function for moving the robot
void RobotMovement(std::shared_ptr<Queue> Q, std::shared_ptr<KDTree> Tree,
                   std::shared_ptr<RobotData> Robot,
                   double planning_only_time, double slice_time,
                   double goal_threshold, double ball_constant);

#endif // MOVEROBOT_H
