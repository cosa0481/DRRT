/* moverobot.cpp
 * Corin Sandford
 * Spring 2017
 * Function for thread to move robot
 */

#include <DRRT/moverobot.h>

using namespace std;

void MoveRobot(shared_ptr<Queue> &Q,
               shared_ptr<KDTree> &Tree,
               shared_ptr<KDTreeNode> &root,
               double slice_time,
               double hyperBallRad,
               shared_ptr<RobotData> &R )
{
    // Start by updating the location of the robot based on how
    // it moved since the last update (as well as the total path that
    // it has followed)
    if( R->moving ) {
        cout << "Moving "
                  << Tree->distanceFunction(R->robot_pose,R->next_robot_pose)
                  << " units" << endl;
        R->robot_pose = R->next_robot_pose;

        //R.robot_move_path[R.num_robot_move_points+1:R.num_robot_move_points+R.num_local_move_points,:] = R.robot_local_path[1:R.num_local_move_points,:];
        for( int i = 0; i < R->num_local_move_points-1; i++ ) {
            R->robot_move_path.row(R->num_robot_move_points+i) = R->robot_local_path.row(i);
        }
        R->num_robot_move_points += R->num_local_move_points;

        if( !Q->cspace->space_has_time_ ) {
            cout << "new robot pose(w/o time):\n"
                      << R->robot_pose << endl;
        } else {
            cout << "new robot pose(w/ time):\n"
                      << R->robot_pose << endl;
        }

        Q->cspace->warmup_time_just_ended_ = false;
    } else {
        // Movement has just started, so remember that the robot is now moving
        R->moving = true;

        cout << "First pose:" << endl;
        cout << R->robot_pose << endl;

        if( !Q->cspace->move_goal_->rrt_parent_used_ ) {
            // no parent has been found for the node at the robots position_
            R->current_move_invalid = true;
        } else {
            R->robot_edge = Q->cspace->move_goal_->rrt_parent_edge_;
            R->robot_edge_used = true;

            if( Q->cspace->space_has_time_ ) {
                R->time_along_robot_edge = 0.0;
            } else {
                R->dist_along_robot_edge = 0.0;
            }
        }
        Q->cspace->warmup_time_just_ended_ = true;
    }


    // If the robot's current move target has been invalidated due to
    // dynamic obstacles then we need to attempt to find a new
    // (safe) move target. NOTE we handle newly invalid moveTarget
    // after moving the robot (since the robot has already moved this
    // time slice)
    if( R->current_move_invalid ) {
        findNewTarget( Q->cspace, Tree, R, hyperBallRad );
    } else {
        /* Recall that moveGoal is the node whose key is used to determine
         * the level set of cost propogation (this should theoretically
         * be further than the robot from the root of the tree, which
         * will happen here assuming that robot moves at least one edge
         * each slice time. Even if that does not happen, things will
         * still be okay in practice as long as robot is "close" to moveGoal
         */
        Q->cspace->move_goal_->is_move_goal_ = false;
        cout << "R->next_move_target:\n" << R->next_move_target->position_ << endl;
        Q->cspace->move_goal_ = R->next_move_target;
        Q->cspace->move_goal_->is_move_goal_ = true;
    }


    /* Finally, we calculate the point to which the robot will move in
    * slice_time and remember it for the next time this function is called.
    * Also remember all the nodes that it will visit along the way in the
    * local path and the part of the edge trajectory that takes the robot
    * to the first local point (the latter two things are used for
    * visualizition)
    */
    if( !Q->cspace->space_has_time_ ) {
        // Not using the time dimension, so assume speed is equal to robot_velocity_
        shared_ptr<KDTreeNode> nextNode = R->next_move_target;

        // Calculate distance from robot to the end of
        // the current edge it is following
        double nextDist = R->robot_edge->dist_ - R->dist_along_robot_edge;

        double distRemaining = Q->cspace->robot_velocity_*slice_time;

        // Save first local path point
        R->num_local_move_points = 1;
        R->robot_local_path.row(R->num_local_move_points-1) = R->robot_pose;

        // Starting at current location (and looking ahead to nextNode), follow
        // parent pointers back for appropriate distance (or root or dead end)
        while( nextDist <= distRemaining && nextNode != root
               && nextNode->rrt_parent_used_
               && nextNode != nextNode->rrt_parent_edge_->end_node_ ) {
            // Can go all the way to nextNode and still have
            // some distance left to spare

            // Remember robot will move through this point
            R->num_local_move_points += 1;
            R->robot_local_path.row(R->num_local_move_points) = nextNode->position_;

            // Recalculate remaining distance
            distRemaining -= nextDist;

            // Reset distance along edge
            R->dist_along_robot_edge = 0.0;

            // Update trajectory that the robot will be in the middle of
            R->robot_edge = nextNode->rrt_parent_edge_;
            R->robot_edge_used = true;

            // Calculate the dist_ of that trajectory
            nextDist = R->robot_edge->dist_;

            // Update the next node (at the end of that trajectory)
            nextNode = R->robot_edge->end_node_;
        }


        // either 1) nextDist > distRemaining
        // or     2) the path we were following now ends at nextNode

        // Calculate the next pose of the robot
        if( nextDist > distRemaining ) {
            R->dist_along_robot_edge += distRemaining;
            R->next_robot_pose
                    = R->robot_edge->PoseAtDistAlongEdge(R->dist_along_robot_edge);
        } else {
            R->next_robot_pose = nextNode->position_;
            R->dist_along_robot_edge = R->robot_edge->dist_;
        }

        R->next_move_target = R->robot_edge->end_node_;

        // Remember last point in local path
        R->num_local_move_points += 1;
        R->robot_local_path.row(R->num_local_move_points) = R->next_robot_pose;
    } else { // S->space_has_time_
        // Space has time, so path is parameterized by time as well
        shared_ptr<KDTreeNode> nextNode = R->next_move_target;

        // Save first local path point
        double targetTime;
        R->num_local_move_points = 1;
        R->robot_local_path.row(R->num_local_move_points) = R->robot_pose;
        targetTime = R->robot_pose(2) - slice_time;
        while( targetTime < R->robot_edge->end_node_->position_(2)
               && nextNode != root && nextNode->rrt_parent_used_
               && nextNode != nextNode->rrt_parent_edge_->end_node_ ) {
            // Can go all the way to nextNode and still have some
            // time left to spare

            // Remember the robot will move through this point
            R->num_local_move_points += 1;
            R->robot_local_path.row(R->num_local_move_points) = nextNode->position_;

            // Update trajectory that the robot will be in the middle of
            R->robot_edge = nextNode->rrt_parent_edge_;
            R->robot_edge_used = true;

            // Update the next node (at the end of that trajectory)
            nextNode = nextNode->rrt_parent_edge_->end_node_;
        }

        // either: 1) targetTime >= nextNode.position_(2)
        // or      2) the path we were following now ends at nextNode

        // Calculate the next pose of the robot
        if( targetTime >= nextNode->position_(2) ) {
            R->time_along_robot_edge = R->robot_edge->start_node_->position_(2)
                    - targetTime;
            R->next_robot_pose
                    = R->robot_edge->PoseAtTimeAlongEdge(R->time_along_robot_edge);
        } else {
            // The next node is the end of this tree and we reach it
            R->next_robot_pose = nextNode->position_;
            R->time_along_robot_edge = R->robot_edge->start_node_->position_(2)
                    - R->robot_edge->end_node_->position_(2);
        }

        R->next_move_target = R->robot_edge->end_node_;

        // Remember the last point in the local path
        R->num_local_move_points += 1;
        R->robot_local_path.row(R->num_local_move_points) = R->next_robot_pose;
    }
}

void RobotMovement(shared_ptr<Queue> Q, shared_ptr<KDTree> Tree,
                   shared_ptr<RobotData> Robot,
                   double planning_only_time, double slice_time,
                   double goal_threshold, double ball_constant)
{
    double elapsed_time;
    Eigen::Vector3d prev_pose;
    double hyper_ball_rad, current_distance, move_distance;
    while(true) { // will break out when goal is reached
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            elapsed_time = Q->cspace->time_elapsed_;

            if(elapsed_time > planning_only_time + slice_time) {
                // Is the Tree intact?
                {
                    lock_guard<mutex> lock(Tree->tree_mutex_);
                    PrintRrtxPath(Q->cspace->goal_node_);
                    // Tests show yes (i.e. feasible path from goal to start)
                }
                hyper_ball_rad = min(Q->cspace->saturation_delta_,
                                     ball_constant*(
                                     pow(log(1+Tree->tree_size_)
                                         /(Tree->tree_size_),
                                         1/Q->cspace->num_dimensions_) ));

                {
                    lock_guard<mutex> lock(Robot->robot_mutex);
                    MoveRobot(Q,Tree,Tree->root,slice_time,hyper_ball_rad,Robot);

                    current_distance = Tree->distanceFunction(Robot->robot_pose,
                                                        Tree->root->position_);
                    move_distance = Tree->distanceFunction(Robot->robot_pose,
                                                           prev_pose);
                }

                cout << "Distance to goal: " << current_distance << endl;
                cout << "Move Distance: " << move_distance << endl;
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
//        this_thread::sleep_for(chrono::nanoseconds(1000000));
    }
}
