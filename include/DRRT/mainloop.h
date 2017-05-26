#ifndef MAINLOOP_H
#define MAINLOOP_H

#include <DRRT/drrt.h>

void RrtMainLoop(std::shared_ptr<Queue> Q, std::shared_ptr<KDTree> Tree,
                 std::shared_ptr<RobotData> Robot,
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time,
                 double ball_constant,
                 double slice_time);

#endif // MAINLOOP_H
