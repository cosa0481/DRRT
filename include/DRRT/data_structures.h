#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <DRRT/list.h>
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

class ConfigSpace : public std::enable_shared_from_this<ConfigSpace> {
public:
    mutex cspace_mutex_;        // Mutex for accessing cspace variables
    int num_dimensions_;        // Number of dimensions in cspace
    List_ptr obstacles_;
    double obstacle_delta_;
}
