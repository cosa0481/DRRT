#ifndef MOVEROBOT_H
#define MOVEROBOT_H

#include <DRRT/drrt.h>

void RobotMovement(std::shared_ptr<Queue> Q, std::shared_ptr<KDTree> Tree,
                   std::shared_ptr<RobotData> Robot,
                   double planning_only_time, double slice_time,
                   double goal_threshold, double ball_constant);

#endif // MOVEROBOT_H
