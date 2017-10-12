#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <DRRT/libraries.h>

#include <DRRT/list.h>
#include <DRRT/kdnode_listnode.h>  // includes kdnode.h
#include <DRRT/obstacle_listnode.h>  // includes obstacle.h
#include <DRRT/edge_listnode.h>  // includes edge.h

#include <DRRT/heap.h>
#include <DRRT/heapnode.h>

#include <DRRT/neighbor_iterator.h>
#include <DRRT/collision_detection.h>

using namespace std;

// Information about the robot
typedef struct RobotData {

    string name;

    mutex mutex;  // Mutex for accessing the robot's state

    bool goal_reached;  // True if robot has reached goal region

    Eigen::VectorXd pose;
    Eigen::VectorXd next_pose;
    Kdnode_ptr next_move_target;

    double dist_next_pose_to_next_move_target;

    bool moving;

    bool current_move_invalid;

    Eigen::MatrixXd move_path;  // Path of the robot from start
    double num_move_points;

    Eigen::MatrixXd local_move_path;  // Path of robot from previous node to next node
    double num_local_move_points;

    Edge_ptr current_edge;  // Current edge robot is traversing
    bool current_edge_used;
    double dist_along_current_edge;  // Distance along the current edge

    double robot_sensor_range;  // Distance from edge of obstacle at which robot will sense it
    double radius;  // Radius from pose that defines the robot's collision shape

    // Theta*
    vector<Eigen::VectorXd> theta_star_path;
    vector<double> thetas;

    // Constructor
    RobotData(string nombre, Eigen::VectorXd start_pose, Kdnode_ptr movetarget)
        : name(nombre), pose(start_pose), next_pose(start_pose),
          next_move_target(movetarget), dist_next_pose_to_next_move_target(0.0),
          moving(false), current_move_invalid(false),
          num_move_points(1), current_edge_used(false)
    {
        pose.resize(NUM_DIM);
        next_pose.resize(NUM_DIM);
        move_path.resize(MAXPATHNODES, NUM_DIM);
        local_move_path.resize(MAXPATHNODES, NUM_DIM);
    }


} RobotData;

typedef std::shared_ptr<RobotData> Robot_ptr;

// Configuration space of the planning problem
class ConfigSpace {
public:
    mutex mutex_;  // Mutex for accessing cspace variables
    int num_dimensions_;  // Number of dimensions in cspace

    ObstacleList_ptr obstacles_;  // List of obstacles in cspace
    double obstacle_thresh_;  // Amount by which an obstacle must move to detect a change
    bool obstacle_update_;  // True if obstacles have been updated and graph needs to be checked

    // System time for start of the program
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    bool in_warmup_time_;
    bool warmup_time_ended_;
    double warmup_time_;
    double elapsed_time_;
    double slice_time_;

    // Tunable parameters
    double goal_sample_prob_;
    double goal_thresh_;
    double collision_thresh_;
    double change_thresh_;
    double min_velocity_;
    double max_velocity_;
    double saturation_delta_;
    double min_turn_radius_;

    // Planning information
    Eigen::VectorXd start_;  // Planner start location
    Eigen::VectorXd end_;  // Planner end location

    Region drivable_region_;

    Kdnode_ptr root_node_;  // Planner end location node
    Kdnode_ptr goal_node_;  // Planner start location node
    Kdnode_ptr move_goal_;  // current goal node for moving robot

    KdnodeList_ptr sample_stack_;  // points to sample in future

    Heap_ptr priority_queue_;  // priority queue for rewiring k-d tree nodes
    KdnodeList_ptr obstacle_successors_;  // obstacle successor list

    // Bullet Collision Detection
    btCollisionConfiguration* bt_collision_config_;
    btCollisionDispatcher* bt_dispatcher_;
    btBroadphaseInterface* bt_broadphase_;
    btCollisionWorld* bt_collision_world_;

    // Robot object
    Robot_ptr robot_;

    // K-D Tree
    KdTree_ptr kdtree_;

    ConfigSpace(Eigen::VectorXd start_pos, Eigen::VectorXd end_pos,
                Eigen::MatrixXd drive_region, double warmuptime)
        : num_dimensions_(NUM_DIM), start_(start_pos), end_(end_pos)
    {
        start_.resize(NUM_DIM);
        end_.resize(NUM_DIM);

        // Drivable region
        drivable_region_ = Region(drive_region);

        // Bullet
        bt_collision_config_ = new btDefaultCollisionConfiguration();
        bt_dispatcher_ = new btCollisionDispatcher(bt_collision_config_);

        btVector3 world_min((btScalar) 0, (btScalar) 0, (btScalar) -0.1);
        btVector3 world_max((btScalar) 50, (btScalar) 50, (btScalar) 0.1);

        // true for disabling raycast accelerator
        bt_broadphase_ = new bt32BitAxisSweep3(world_min, world_max, 20000, 0, true);

        bt_collision_world_ = new btCollisionWorld(bt_dispatcher_,
                                                   bt_broadphase_,
                                                   bt_collision_config_);

        // Obstacle list
        obstacle_update_ = true;
        obstacles_ = std::make_shared<List<ObstacleListNode>>();

        // Warmup time
        warmup_time_ = warmuptime;
        if(warmup_time_ > 0.0) in_warmup_time_ = true;
        else in_warmup_time_ = false;

        // Initialize sample stack
        sample_stack_ = std::make_shared<KdnodeList>();

        // Initialize priority queue
        priority_queue_ = std::make_shared<Heap>(0);  // 0 -> min heap

        // Initialize obstacle successors list
        obstacle_successors_ = std::make_shared<KdnodeList>();
    }
};

typedef std::shared_ptr<ConfigSpace> CSpace_ptr;

#endif //DATA_STRUCTURES_H
