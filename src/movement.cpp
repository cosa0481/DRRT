#include <DRRT/movement.h>

using namespace std;

void Move(CSpace_ptr &cspace, double slice_time, double hyper_ball_rad)
{
    lockguard lock(cspace->robot_->mutex);
    Robot_ptr robot = cspace->robot_;

    if(robot->moving)
    {
        robot->pose = robot->next_pose;

        for(int i = 0; i < robot->num_local_move_points - 1; i++) {
        robot->move_path.row(robot->num_move_points + i)
                = robot->local_move_path.row(i);
        }
        robot->num_move_points += robot->num_local_move_points;

        cspace->warmup_time_ended_ = false;
        } else {
            robot->moving = true;

            if(!cspace->move_goal_->RrtParentExist())
                robot->current_move_invalid = true;
            else {
                cspace->move_goal_->GetRrtParentEdge(robot->current_edge);
                robot->current_edge_used = true;
                robot->dist_along_current_edge = 0.0;
            }

            cspace->warmup_time_ended_ = true;
        }

        if(robot->current_move_invalid)
            FindNewTarget(cspace, hyper_ball_rad);
        else {
            lockguard lock(cspace->mutex_);
            cspace->move_goal_->SetIsGoal(false);
            cspace->move_goal_ = robot->next_move_target;
            cspace->move_goal_->SetIsGoal(true);
    }

    Kdnode_ptr next_node = robot->next_move_target;

    // Calculate distance from robot to end of its current edge
    double next_dist = robot->current_edge->GetDist() - robot->dist_along_current_edge;
    double remaining = cspace->max_velocity_ * slice_time;

    // Save the first local path point
    robot->num_local_move_points = 1;

    /// EIGEN BLOCK ASSERTION FAILED
    /// This may be where it's happening
    /// Assertion failed: ((i>=0) && ( ((BlockRows==1) && (BlockCols==XprType::ColsAtCompileTime) && i<xpr.rows()) ||((BlockRows==XprType::RowsAtCompileTime) && (BlockCols==1) && i<xpr.cols()))), function Block, file /usr/local/include/eigen3/Eigen/src/Core/Block.h, line 119.
    robot->local_move_path.row(robot->num_local_move_points - 1) = robot->pose;

    // Starting at the current location and looking ahead to next_node,
    // follow parents back for the approirate distance (or root or dead end)
    Edge_ptr rrt_parent_edge;
    next_node->GetRrtParentEdge(rrt_parent_edge);
    while((next_dist <= remaining) && (next_node != cspace->kdtree_->root_)
          && next_node->RrtParentExist()
          && (next_node != rrt_parent_edge->GetEnd())) {
        // Go all the way to next_node and still have some distance to spare

        // Robot will move through this point
        robot->num_local_move_points += 1;
        robot->local_move_path.row(robot->num_local_move_points) = next_node->GetPosition();

        // Calculate remaining distance
        remaining -= next_dist;

        // Reset distance along edge
        robot->dist_along_current_edge = 0.0;

        // Update trajectory the robot will be in the middle of
        next_node->GetRrtParentEdge(robot->current_edge);
        robot->current_edge_used = true;

        // Get the distance of that trajectory
        next_dist = robot->current_edge->GetDist();

        // Update the next node at the end of the new trajectory
        next_node = robot->current_edge->GetEnd();
    }

    // Now either next_dist > remaining or next_node is a dead end
    if(next_dist > remaining) {
        robot->dist_along_current_edge += remaining;
        robot->next_pose = robot->current_edge->PoseAtDistAlongEdge(robot->dist_along_current_edge);
    } else {
        robot->next_pose = next_node->GetPosition();
        robot->dist_along_current_edge = robot->current_edge->GetDist();
    }

    robot->next_move_target = robot->current_edge->GetEnd();

    // Last point in local path
    robot->num_local_move_points += 1;
    robot->local_move_path.row(robot->num_local_move_points) = robot->next_pose;
}


void MovementThread(CSpace_ptr &cspace, double plan_time, double ball_constant)
{
    Eigen::VectorXd last_pose;
    {
        lockguard lock(cspace->robot_->mutex);
        last_pose = cspace->robot_->pose;
    }
    double elapsed_time;
    double hyper_ball_rad, current_dist, move_dist;
    bool started = false;
    bool ended = false;
    while(true)
    {
        elapsed_time = cspace->elapsed_time_;

        if(elapsed_time > plan_time + cspace->slice_time_)
        {
            if(!started) {
                started = true;
            }

            hyper_ball_rad = min(cspace->saturation_delta_,
                                 ball_constant*(pow( log(1 + cspace->kdtree_->size_)/cspace->kdtree_->size_,
                                                     1/NUM_DIM)));

            Move(cspace, hyper_ball_rad);
            current_dist = DistanceFunction(cspace->robot_->pose, cspace->kdtree_->root_->GetPosition());
            move_dist = DistanceFunction(cspace->robot_->pose, last_pose);

            if(current_dist < cspace->goal_thresh_) {
                lockguard lock(cspace->robot_->mutex);
                cspace->robot_->goal_reached = true;
                ended = true;
                break;
            } else if(move_dist > 1.5) {
                if(DEBUG)
                    cout << "Moved more than 1.5m in " << cspace->slice_time_ << ". Impossibru!" << endl;
            }

            {
                lockguard lock(cspace->robot_->mutex);
                last_pose = cspace->robot_->pose;
            }
        }
        this_thread::sleep_for(chrono::milliseconds(100));  // 1/10th of a second
    }

    if(ended)
    {
        lockguard lock(cspace->robot_->mutex);
        // Calculate and display distance traveled
        Eigen::ArrayXXd first, last, diff;
        first = cspace->robot_->move_path.block(0,0,
                                                cspace->robot_->num_move_points - 1, 3);
        last = cspace->robot_->move_path.block(1,0,
                                               cspace->robot_->num_move_points - 1, 3);

        diff = first - last;
        diff = diff*diff;
        for(int i = 0; i < diff.rows(); i++)
            diff.col(0)(i) = diff.row(i).sum();

        double move_length = diff.col(0).sqrt().sum();
        cout << "\nRobot travel distance: " << move_length << " m" << endl;
    }
}
