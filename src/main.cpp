#include <DRRT/drrt.h>
#include "rrtx.h"
#include "../src/list.cpp"

using namespace std;

int main(int argc, char* argv[])
{
    cout << "Dubin's Example" << endl;
    cout << "===============" << endl;

    string obstacle_file = argv[1];

    // Configuration Space Parameters
    double plan_only_time = 30;  // plan only for this long
    Eigen::MatrixX2d physical_cspace;  // specify counter-clockwise from origin
    physical_cspace.resize(4, Eigen::NoChange_t());
    physical_cspace(0,0) = 0;  // lower left
    physical_cspace(0,1) = 0;
    physical_cspace(1,0) = 50;  // lower right
    physical_cspace(1,1) = 0;
    physical_cspace(2,0) = 50;  // upper right
    physical_cspace(2,1) = 50;
    physical_cspace(3,0) = 0;  // upper left
    physical_cspace(3,1) = 50;

    Eigen::Vector3d start, goal;
    start << 0.0, 0.0, -3*PI/4;  // destination
    goal << 49.0, 49.0, -3*PI/4;  // robot start

    CSpace_ptr cspace = make_shared<ConfigSpace>(start, goal, physical_cspace, plan_only_time);

    cspace->warmup_time_ = 0.0;
    cspace->in_warmup_time_ = false;

    cspace->slice_time_ = 1.0/100;

    cspace->min_turn_radius_ = 1.0;
    cspace->max_velocity_ = 20.0;
    cspace->min_velocity_ = 20.0;
    cspace->goal_sample_prob_ = 0.01;
    cspace->goal_thresh_ = 1.0;

    cspace->change_thresh_ = 0.5;
    cspace->collision_thresh_ = 0.1;
    cspace->saturation_delta_ = 5.0;
    cspace->ball_constant_ = 100.0;

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

    double algorithm_time = GetTimeNs(cspace->start_time_);
    cout << "\nTotal Algorithm Time: " << algorithm_time/1000000000 << " s" << endl;

    // Delete bullet pointers
    delete cspace->bt_collision_config_;
    delete cspace->bt_dispatcher_;
    delete cspace->bt_broadphase_;
    delete cspace->bt_collision_world_;

    cout << "Done" << endl;

    return 0;
}
