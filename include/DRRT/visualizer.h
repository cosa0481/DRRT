#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <SceneGraph/SceneGraph.h>

// Visualizer function for DRRT
void visualizer(std::shared_ptr<KDTree> Tree,
                std::shared_ptr<RobotData> Robot,
                std::shared_ptr<Queue> Q);

#endif // VISUALIZER_H
