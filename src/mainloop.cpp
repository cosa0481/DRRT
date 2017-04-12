/* mainloop.cpp
 * Corin Sandford
 * Spring 2017
 * RRTX main loop function for running in threads
 */

#include <DRRT/mainloop.h>

using namespace std;

void RrtMainLoop(shared_ptr<Queue> Q, shared_ptr<KDTree> Tree,
                 shared_ptr<RobotData> Robot,
                 chrono::time_point<chrono::high_resolution_clock> start_time,
                 double ball_constant,
                 double slice_time,
                 vector<double> avg_thetas,
                 vector<Eigen::VectorXd> path)
{
    double slice_start = chrono::duration_cast<chrono::nanoseconds>(
                start_time-start_time).count();
    double slice_end;
    double slice_counter = 0;

    double now = GetTimeNs(start_time);
    double elapsed_time;
    double iter_start, iter_end;

    double old_rrt_LMC, current_distance, initial_distance;
    Eigen::Vector3d prev_pose, robot_pose;
    bool removed, added;
    shared_ptr<ListNode> obs_node;
    shared_ptr<Obstacle> obstacle;
    shared_ptr<KDTreeNode> move_goal;

    {
        lock_guard<mutex> lock(Robot->robot_mutex);
        prev_pose = Robot->robot_pose;
    }

    // Importance sampling
    double p_uniform = 0.9;
    double position_bias;
    double theta_bias = PI/10;

    current_distance = Tree->distanceFunction(prev_pose,
                                              Tree->root->position_);
    bool reached_goal = false;

    int i = 0;
    while(!reached_goal) {
        // Calculate initial hyper ball radius
        double hyper_ball_rad = min(Q->cspace->saturation_delta_,
                             ball_constant*(
                             pow(log(1+Tree->tree_size_)/(Tree->tree_size_),
                                 1/Q->cspace->num_dimensions_)));
        now = GetTimeNs(start_time);
        slice_end = (1+slice_counter)*slice_time; // time in the next slice

        // Check for warm up time
        if(Q->cspace->in_warmup_time_
                && Q->cspace->warmup_time_ < Q->cspace->time_elapsed_) {
            Q->cspace->in_warmup_time_ = false;
        }

        // Add / Remove Obstacles
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            obs_node = Q->cspace->obstacles_->front_;
            elapsed_time = Q->cspace->time_elapsed_;
            move_goal = Q->cspace->move_goal_;
        }
        {
            lock_guard<mutex> lock(Robot->robot_mutex);
            robot_pose = Robot->robot_pose;
        }
        removed = false;
        added = false;
        while(obs_node != obs_node->child_) {
            obstacle = obs_node->obstacle_;

            // Remove
            if(!obstacle->sensible_obstacle_ && obstacle->obstacle_used_
                    && (obstacle->start_time_ + obstacle->life_span_
                        <= elapsed_time)) {
                // Time to remove obstacle
                RemoveObstacle(Tree,Q,obstacle,Tree->root,hyper_ball_rad,
                               elapsed_time,move_goal);
                removed = true;
            }
            // Add
            if(!obstacle->sensible_obstacle_ && !obstacle->obstacle_used_
                    && obstacle->start_time_ <= elapsed_time
                    && elapsed_time <= obstacle->start_time_
                                     + obstacle->life_span_) {
                // Time to add obstacle
                obstacle->obstacle_used_ = true; // This line significantly slows down program
                AddObstacle(Tree,Q,obstacle,Tree->root);
                if(Robot->robot_edge_used
                        && Robot->robot_edge->ExplicitEdgeCheck(obstacle)) {
                    {
                        lock_guard<mutex> lock(Robot->robot_mutex);
                        Robot->current_move_invalid = true;
                    }
                }
                added = true;
            }
            obs_node = obs_node->child_;
        }
        if(removed) {
            reduceInconsistency(Q,Q->cspace->move_goal_,
                                Q->cspace->robot_radius_,
                                Tree->root, hyper_ball_rad);
        }
        if(added) {
            propogateDescendants(Q,Tree,Robot);
            if(!markedOS(Q->cspace->move_goal_)) {
                verifyInQueue(Q,Q->cspace->move_goal_);
            }
            reduceInconsistency(Q,Q->cspace->move_goal_,
                                Q->cspace->robot_radius_,
                                Tree->root,hyper_ball_rad);
        }

        // Run iterations based on timing
        elapsed_time = (GetTimeNs(start_time)
                        - Q->cspace->start_time_ns_) / 1000000000.0;
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            Q->cspace->time_elapsed_ = elapsed_time;
        }
//        cout << "elapsed_time: " << elapsed_time << endl;
//        cout << "slice_end: " << slice_end << endl;
        if(elapsed_time >= slice_end) {
            iter_start = GetTimeNs(start_time);
            slice_start = now;
            slice_end = (++slice_counter)*slice_time;

            cout << this_thread::get_id() << " Iteration " << i++
                 << "\n--------------------------------" << endl;


            reduceInconsistency(Q,Q->cspace->move_goal_,
                                Q->cspace->robot_radius_,
                                Tree->root, hyper_ball_rad);
            if(Q->cspace->move_goal_->rrt_LMC_ != old_rrt_LMC) {
                old_rrt_LMC = Q->cspace->move_goal_->rrt_LMC_;
            }

            // Sample the free cspace
            shared_ptr<KDTreeNode> new_node = make_shared<KDTreeNode>();
            shared_ptr<KDTreeNode> closest_node = make_shared<KDTreeNode>();
            shared_ptr<double> closest_dist = make_shared<double>(INF);

            // Importance sample
//            cout << "Main Loop " << this_thread::get_id() << endl;
            bool importance_sampling = true;
            while(importance_sampling) {
                new_node = randNodeOrFromStack(Q->cspace);
                if(new_node->kd_in_tree_) continue;
                Tree->KDFindNearest(closest_node,closest_dist,
                                    new_node->position_);

                // Saturate this node
                initial_distance = Tree->distanceFunction(new_node->position_,
                                                      closest_node->position_);
                if(initial_distance > Q->cspace->saturation_delta_
                        && new_node != Q->cspace->goal_node_) {
                    Edge::Saturate(new_node->position_,
                                   closest_node->position_,
                                   Q->cspace->saturation_delta_,
                                   initial_distance);
                }
                if(ExplicitNodeCheck(Q,new_node)) continue;
                if(RandDouble(0,1) > p_uniform) break;
                if(Tree->distanceFunction(new_node->position_,
                                          Tree->root->position_)
                    > Tree->distanceFunction(Q->cspace->goal_node_->position_,
                                             Tree->root->position_))
                    continue;
                if(Tree->distanceFunction(new_node->position_,
                                          Q->cspace->goal_node_->position_)
                    > Tree->distanceFunction(Q->cspace->goal_node_->position_,
                                             Tree->root->position_))
                    continue;

                // Find the closest line on the any-angle path to the new node
                double dist_node_to_path;
                double min_dist_node_to_path = INF;
                int min_dist_node_to_path_index = 0;
                for(int i = 1; i < Robot->best_any_angle_path.size(); i++) {
                    dist_node_to_path = DistanceSqrdPointToSegment(
                                new_node->position_,
                                Robot->best_any_angle_path.at(i-1).head(2),
                                Robot->best_any_angle_path.at(i).head(2));
                    if(dist_node_to_path < min_dist_node_to_path) {
                        min_dist_node_to_path = dist_node_to_path;
                        min_dist_node_to_path_index = i-1;
                    }
                }

                // Calculate theta bias
                new_node->position_(2)
                        = RandDouble(avg_thetas[min_dist_node_to_path_index]
                                     - theta_bias,
                                     avg_thetas[min_dist_node_to_path_index]
                                     + theta_bias);

                // Calculate position bias
                position_bias = 2*hyper_ball_rad;
                double distance = DistanceSqrdPointToSegment(
                            new_node->position_,
                            path.at(min_dist_node_to_path_index).head(2),
                            path.at(min_dist_node_to_path_index+1).head(2));
                if(distance > position_bias) continue;
                importance_sampling = false;
            }


            // Extend the graph        
            if(Extend(Tree,Q,new_node,closest_node,
                      Q->cspace->saturation_delta_, hyper_ball_rad,
                      Q->cspace->move_goal_)) {
                // want a cookie?
                //cout << "cookie" << endl;
            } else {
                //cout << "extend returned false" << endl;
            }

//            {
//                lock_guard<mutex> lock(Tree->tree_mutex_);
//                cout << "Tree Size: " << Tree->tree_size_ << endl;
//            }


            // Make graph consistent
            reduceInconsistency(Q,Q->cspace->move_goal_,
                                Q->cspace->robot_radius_,
                                Tree->root, hyper_ball_rad);
            if(Q->cspace->move_goal_-> rrt_LMC_ != old_rrt_LMC) {
                old_rrt_LMC = Q->cspace->move_goal_->rrt_LMC_;
            }

            iter_end = GetTimeNs(start_time);
            cout << "Duration: " << (iter_end - iter_start)/MICROSECOND
                 << " ms\n" << endl;

            {
                lock_guard<mutex> lock(Robot->robot_mutex);
                reached_goal = Robot->goal_reached;
            }
        }
    }
}
