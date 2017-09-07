#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <DRRT/libraries.h>

#include <DRRT/list.h>
#include <DRRT/kd_listnode.h>
#include <DRRT/obstacle_listnode.h>
#include <DRRT/edge_listnode.h>

#include <DRRT/heap.h>
#include <DRRT/kd_heapnode.h>

#include <DRRT/kdnode.h>
#include <DRRT/obstacle.h>
#include <DRRT/edge.h>

using namespace std;

// Triangle container
typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixX6d;

typedef shared_ptr<Kdnode> Kdnode_ptr;
typedef shared_ptr<Obstacle> Obstacle_ptr;
typedef shared_ptr<List> List_ptr;
typedef shared_ptr<Edge> Edge_ptr;
//typedef shared_ptr<Queue> Queue_ptr;

typedef struct Queue {
    shared_ptr<ConfigSpace> cspace;
    double change_thresh;
} Queue;

typedef struct RobotData {
    bool goal_reached;          // True if robot has reached goal region

} RobotData;

typedef struct RrtNodeNeighborIterator {

} RrtNodeNeighborIterator;

class ConfigSpace : public std::enable_shared_from_this<ConfigSpace> {
public:
    mutex cspace_mutex_;        // Mutex for accessing cspace variables
    int num_dimensions_;        // Number of dimensions in cspace
    List_ptr obstacles_;        // List of obstacles in cspace
    double obstacle_thresh_;    // Amount by which an obstacle must move to detect a change

    ConfigSpace() {}
};

#endif //DATA_STRUCTURES_H
