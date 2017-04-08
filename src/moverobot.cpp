/* moverobot.cpp
 * Corin Sandford
 * Spring 2017
 * Function for thread to move robot
 */

#include <DRRT/moverobot.h>

using namespace std;

void RobotMovement(shared_ptr<Queue> Q, shared_ptr<KDTree> Tree,
                   shared_ptr<RobotData> Robot,
                   double planning_only_time, double slice_time,
                   double goal_threshold, double ball_constant)
{
    double elapsed_time;
    Eigen::Vector3d prev_pose;
    double hyper_ball_rad, current_distance, move_distance;
    while(true) { // will break out when goal is reached
        hyper_ball_rad = min(Q->cspace->saturation_delta_, ball_constant*(
                                        pow(log(1+Tree->tree_size_)
                                            /(Tree->tree_size_),
                                            1/Q->cspace->num_dimensions_) ));
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            elapsed_time = Q->cspace->time_elapsed_;
        }
        if(elapsed_time > planning_only_time + slice_time) {
//            cout << "Moving" << endl;
            MoveRobot(Q,Tree,Tree->root,slice_time,hyper_ball_rad,Robot);

            current_distance = Tree->distanceFunction(Robot->robot_pose,
                                                             Tree->root->position_);
            move_distance = Tree->distanceFunction(Robot->robot_pose,
                                                          prev_pose);

            cout << "Distance to goal: " << current_distance << endl;
            if(current_distance < goal_threshold) {
                cout << "Reached goal" << endl;
                {
                    lock_guard<mutex> lock(Robot->robot_mutex);
                    Robot->goal_reached = true;
                    break;
                }
            } else if(move_distance > Q->cspace->saturation_delta_) {
                cout << "Impossible move ... quitting" << endl;
                exit(-1);
            }
            prev_pose = Robot->robot_pose;
        }
    }
}
