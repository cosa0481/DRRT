/* mainloop.cpp
 * Corin Sandford
 * Spring 2017
 * RRTX main loop function for running in threads
 */

#include <DRRT/mainloop.h>

using namespace std;

bool timingml = false;

void RrtMainLoop(shared_ptr<Queue> Q, shared_ptr<KDTree> Tree,
                 shared_ptr<RobotData> Robot,
                 chrono::time_point<chrono::high_resolution_clock> start_time,
                 double ball_constant,
                 double slice_time,
                 vector<double> thetas,
                 vector<Eigen::VectorXd> path)
{
    chrono::steady_clock::time_point t1,t2,i1,i2;
    double delta;

    double slice_start = chrono::duration_cast<chrono::nanoseconds>(
                start_time-start_time).count();
    double slice_end;
    double slice_counter = 0;

    double now = GetTimeNs(start_time);
    double elapsed_time;
    double iter_start/*, iter_end*/;

    double old_rrt_LMC, current_distance, initial_distance;
    Eigen::Vector3d prev_pose;

    {
        lock_guard<mutex> lock(Robot->robot_mutex);
        prev_pose = Robot->robot_pose;
    }

    // Importance sampling
//    double p_uniform = 0.9;
//    double position_bias;
//    double theta_bias = PI/10;

    current_distance = Tree->distanceFunction(prev_pose,
                                              Tree->root->position_);
    bool reached_goal = false;

    int i = 0;
    while(!reached_goal) {
        // Calculate initial hyper ball radius
        double hyper_ball_rad;
        {
            lock_guard<mutex> lock(Tree->tree_mutex_);
            hyper_ball_rad = min(Q->cspace->saturation_delta_,
                             ball_constant*(
                             pow(log(1+Tree->tree_size_)/(Tree->tree_size_),
                                 1/Q->cspace->num_dimensions_)));
        }
        now = GetTimeNs(start_time);
        slice_end = (1+slice_counter)*slice_time; // time in the next slice

        // Check for warm up time
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            if(Q->cspace->in_warmup_time_
                    && Q->cspace->warmup_time_ < Q->cspace->time_elapsed_) {
                Q->cspace->in_warmup_time_ = false;
            }
        }

        // Run iterations based on timing
        elapsed_time = (GetTimeNs(start_time)
                        - Q->cspace->start_time_ns_) / 1000000000.0;
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            Q->cspace->time_elapsed_ = elapsed_time;
        }

        if(elapsed_time >= slice_end) {
            iter_start = GetTimeNs(start_time);
            slice_start = now;
            slice_end = (++slice_counter)*slice_time;

            if(timingml)
                cout << this_thread::get_id() << " Iteration " << i++
                 << "\n--------------------------------" << endl;
            i1 = chrono::steady_clock::now();

            {
                lock_guard<mutex> lock(Q->queuetex);
                {
                    lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
                    {
                        lock_guard<mutex> lock(Tree->tree_mutex_);
                        ReduceInconsistency(Q,Q->cspace->move_goal_,
                                            Q->cspace->robot_radius_,
                                            Tree->root, hyper_ball_rad);
                        if(Q->cspace->move_goal_->rrt_LMC_ != old_rrt_LMC) {
                            old_rrt_LMC = Q->cspace->move_goal_->rrt_LMC_;
                        }
                    } // unlock tree mutex
                } // unlock cspace_mutex_
            } // unlock queuetex

            // Sample the free cspace
            shared_ptr<KDTreeNode> new_node = make_shared<KDTreeNode>();
            shared_ptr<KDTreeNode> closest_node = make_shared<KDTreeNode>();
            shared_ptr<double> closest_dist = make_shared<double>(INF);

            // Importance sample
//            cout << "Main Loop " << this_thread::get_id() << endl;
            bool importance_sampling = true;
            while(importance_sampling) {
                new_node = RandNodeOrFromStack(Q->cspace);
                if(new_node->kd_in_tree_) continue;
                {
                    lock_guard<mutex> lock(Tree->tree_mutex_);
                    Tree->KDFindNearest(closest_node,closest_dist,
                                        new_node->position_);
                }

                // Saturate this node
                initial_distance = Tree->distanceFunction(new_node->position_,
                                                      closest_node->position_);
                {
                    lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
                    if(initial_distance > Q->cspace->saturation_delta_
                            && new_node != Q->cspace->goal_node_) {
                        Edge::Saturate(new_node->position_,
                                       closest_node->position_,
                                       Q->cspace->saturation_delta_,
                                       initial_distance);
                    }
                }
                if(ExplicitNodeCheck(Q,new_node)) continue;
//                if(RandDouble(0,1) > p_uniform) break;
//                if(Tree->distanceFunction(new_node->position_,
//                                          Tree->root->position_)
//                    > Tree->distanceFunction(Q->cspace->goal_node_->position_,
//                                             Tree->root->position_))
//                    continue;
//                if(Tree->distanceFunction(new_node->position_,
//                                          Q->cspace->goal_node_->position_)
//                    > Tree->distanceFunction(Q->cspace->goal_node_->position_,
//                                             Tree->root->position_))
//                    continue;

//                // Find the closest line on the any-angle path to the new node
//                double dist_node_to_path;
//                double min_dist_node_to_path = INF;
//                int min_dist_node_to_path_index = 0;
//                for(int i = 1; i < Robot->best_any_angle_path.size(); i++) {
//                    dist_node_to_path = DistanceSqrdPointToSegment(
//                                new_node->position_,
//                                Robot->best_any_angle_path.at(i-1).head(2),
//                                Robot->best_any_angle_path.at(i).head(2));
//                    if(dist_node_to_path < min_dist_node_to_path) {
//                        min_dist_node_to_path = dist_node_to_path;
//                        min_dist_node_to_path_index = i-1;
//                    }
//                }

//                // Calculate theta bias
//                new_node->position_(2)
//                        = RandDouble(thetas[min_dist_node_to_path_index]
//                                     - theta_bias,
//                                     thetas[min_dist_node_to_path_index]
//                                     + theta_bias);

//                // Calculate position bias
//                position_bias = 2*hyper_ball_rad;
//                double distance = DistanceSqrdPointToSegment(
//                            new_node->position_,
//                            path.at(min_dist_node_to_path_index).head(2),
//                            path.at(min_dist_node_to_path_index+1).head(2));
//                if(distance > position_bias) continue;
                importance_sampling = false;
            }

            // Extend the graph
            // mutex locks inside this function
            t1 = chrono::steady_clock::now();
            Extend(Tree,Q,new_node,closest_node,
                   Q->cspace->saturation_delta_, hyper_ball_rad,
                   Q->cspace->move_goal_);
            t2 = chrono::steady_clock::now();
            delta = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
            if(timingml) cout << "Extend: " << delta << " s" << endl;


            // Make graph consistent
            {
                lock_guard<mutex> lock(Q->queuetex);
                {
                    lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
                    {
                        lock_guard<mutex> lock(Tree->tree_mutex_);
                        ReduceInconsistency(Q,Q->cspace->move_goal_,
                                            Q->cspace->robot_radius_,
                                            Tree->root, hyper_ball_rad);
                        if(Q->cspace->move_goal_->rrt_LMC_ != old_rrt_LMC) {
                            old_rrt_LMC = Q->cspace->move_goal_->rrt_LMC_;
                        }
                    } // unlock tree mutex
                } // unlock cspace_mutex_
            } // unlock queuetex

            i2 = chrono::steady_clock::now();
            delta = chrono::duration_cast<chrono::duration<double> >(i2 - i1).count();
            if(timingml) cout << "Duration: " << delta << " s\n" << endl;

//            this_thread::sleep_for(chrono::nanoseconds(1000000000));

            {
                lock_guard<mutex> lock(Robot->robot_mutex);
                reached_goal = Robot->goal_reached;
            }
        }
    }
}
