#include <DRRT/drrt.h>
#include "rrtx.h"
#include "../src/list.cpp"

using namespace std;

int main(int argc, char* argv[])
{
    cout << "Dubin's Example" << endl;
    cout << "===============" << endl;

    string obstacle_file = argv[1];
    bool static_obstacles = (strcmp(argv[2], "true") == 0 ? true : false);

    // Configuration Space Parameters
    double plan_only_time = 5;  // plan only for this long
    Eigen::MatrixX2d physical_cspace;  // specify counter-clockwise from origin
// Normal
//    physical_cspace.resize(4, Eigen::NoChange_t());
//    physical_cspace(0,0) = -25;  // lower left
//    physical_cspace(0,1) = -25;
//    physical_cspace(1,0) = 25;  // lower right
//    physical_cspace(1,1) = -25;
//    physical_cspace(2,0) = 25;  // upper right
//    physical_cspace(2,1) = 25;
//    physical_cspace(3,0) = -25;  // upper left
//    physical_cspace(3,1) = 25;

// Smallgrid
    physical_cspace.resize(4, Eigen::NoChange_t());
    physical_cspace(0,0) = -10;
    physical_cspace(0,1) = -10;
    physical_cspace(1,0) = 10;
    physical_cspace(1,1) = -10;
    physical_cspace(2,0) = 10;
    physical_cspace(2,1) = 10;
    physical_cspace(3,0) = -10;
    physical_cspace(3,1) = 10;


    Eigen::Vector3d start, goal;
// Normal
//    start << -25.0, -25.0, -3*PI/4;  // destination
//    goal << 25.0, 25.0, -3*PI/4;  // robot start

// Smallgrid
    start << -10.0, -10.0, -3*PI/4;  // destination
    goal << 10.0, 10.0, -3*PI/4;  // robot start

    CSpace_ptr cspace = make_shared<ConfigSpace>(start, goal, physical_cspace, plan_only_time);

    cspace->warmup_time_ = 0.0;
    cspace->in_warmup_time_ = false;

    cspace->slice_time_ = 1.0/100;

    cspace->min_turn_radius_ = 1.0;
    cspace->max_velocity_ = 30.0;
    cspace->min_velocity_ = 30.0;
    cspace->goal_sample_prob_ = 0.01;
    cspace->goal_thresh_ = 0.1;

    cspace->change_thresh_ = 0.5;
    cspace->collision_thresh_ = 0.1;
    cspace->saturation_delta_ = 5.0;
    cspace->ball_constant_ = 100.0;
    // Set max search ball radius in drrt.cpp::FindNewTarget()

    cspace->static_obstacles_ = static_obstacles;
    if(cspace->static_obstacles_)
        ReadStaticObstaclesFromFile(obstacle_file, cspace);
    else
        ReadObstaclesFromFile(obstacle_file, cspace);

    // Problem Parameters
    Eigen::VectorXi wrap_vec(1);
    wrap_vec(0) = 2;
    Eigen::VectorXd wrap_points_vec(1);
    wrap_points_vec(0) = 2.0*PI;
    double drive_robot = true;
    int num_threads = 1;
    Problem problem = Problem(cspace, drive_robot, wrap_vec, wrap_points_vec, num_threads);

    cout << "Starting Algorithm" << endl;
    Robot_ptr robot_data = Rrtx(problem);

    double algorithm_time = cspace->elapsed_time_;
    cout << "\nTotal Algorithm Time: " << algorithm_time << " s" << endl;

    // Delete bullet pointers
    delete cspace->bt_collision_config_;
    delete cspace->bt_dispatcher_;
    delete cspace->bt_broadphase_;
    delete cspace->bt_collision_world_;

    cout << "Done" << endl;

    return 0;
}
