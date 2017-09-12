#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <DRRT/libraries.h>

#include <DRRT/list.h>
#include <DRRT/kdnode_listnode.h>
#include <DRRT/obstacle_listnode.h>
#include <DRRT/edge_listnode.h>

#include <DRRT/heap.h>
#include <DRRT/kd_heapnode.h>

#include <DRRT/neighbor_iterator.h>
#include <DRRT/collision_detection.h>

using namespace std;

// Triangle container
typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixX6d;

typedef struct Queue {
    shared_ptr<ConfigSpace> cspace;
    double change_thresh;
} Queue;

// Information about the robot
typedef struct RobotData {
    bool goal_reached;          // True if robot has reached goal region

} RobotData;

// Configuration space of the planning problem
class ConfigSpace {
public:
    mutex cspace_mutex_;  // Mutex for accessing cspace variables
    int num_dimensions_;  // Number of dimensions in cspace
    ObstacleList_ptr obstacles_;  // List of obstacles in cspace
    double obstacle_thresh_;  // Amount by which an obstacle must move to detect a change

    // System time for start of the program
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    bool in_warmup_time_;

    double robot_radius_;
    double collision_distance_;

    Kdnode_ptr goal_;  // Planner goal node
    Kdnode_ptr start_;  // Planner start node
    Kdnode_ptr move_goal_;  // current goal node for moving robot

    // Bullet Collision Detection
    btCollisionConfiguration* bt_collision_configuration_;
    btCollisionDispatcher* bt_dispatcher_;
    btBroadphaseInterface* bt_broadphase_;
    btCollisionWorld* bt_collision_world_;

    ConfigSpace() {}
};

#endif //DATA_STRUCTURES_H
