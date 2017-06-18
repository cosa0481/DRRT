/* mainloop.cpp
 * Corin Sandford
 * Spring 2017
 * RRTX main loop function for running in threads
 */

#include <DRRT/mainloop.h>

using namespace std;

bool timingml = false; // Use to check on frame rate
bool collect_timing_data = false;

void RrtMainLoop(shared_ptr<Queue> Q, shared_ptr<KDTree> Tree,
                 shared_ptr<RobotData> Robot,
                 chrono::time_point<chrono::high_resolution_clock> start_time,
                 double ball_constant,
                 double slice_time)
{
    chrono::steady_clock::time_point t1,t2,i1,i2;
    double delta;
    ofstream time_file;
    if(collect_timing_data) {
        string file_name = "/Users/corinsandford/ARPG/DRRT/build/itertimes";
        stringstream file_stream;
        file_stream << file_name << this_thread::get_id() << ".txt";
        file_name = file_stream.str();
        time_file.open(file_name);
    }

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
    double p_uniform = 0.9;
    double position_bias = 6;
    double theta_bias = PI/10;
    bool rand = true;

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
            Eigen::MatrixX2d sampling_area;

            if(RandDouble(0,1) > p_uniform) { // Sampling randomly
                rand = true;
                // Create a square defined as a polygon CCW
                sampling_area.resize(4,Eigen::NoChange_t());
                sampling_area(0,0) = 0;
                sampling_area(0,1) = 0;
                sampling_area(1,0) = Q->cspace->upper_bounds_(0);
                sampling_area(1,1) = 0;
                sampling_area(2,0) = Q->cspace->upper_bounds_(0);
                sampling_area(2,1) = Q->cspace->upper_bounds_(1);
                sampling_area(3,0) = 0;
                sampling_area(3,1) = Q->cspace->upper_bounds_(1);
                Q->cspace->drivable_region_ = Region(sampling_area);
            } else { // Sampling inside Theta* bounds
                rand = false;
                sampling_area.resize(2*Robot->best_any_angle_path.size(),
                                     Eigen::NoChange_t());

                // Create line parallel to the line segment and use it's endpoints
                // Need to go counterclockwise for sampling_area because of TriangulatePolygon library
                vector<Eigen::VectorXd> points;
                Eigen::VectorXd start_point, end_point, new_start, new_end;
                double dx, dy, perp_x, perp_y, norm_len;
                for(int i = 1; i < Robot->best_any_angle_path.size(); i++) {
                    start_point = Robot->best_any_angle_path[i-1];
                    end_point = Robot->best_any_angle_path[i];

                    dx = end_point(0) - start_point(0);
                    dy = end_point(1) - start_point(1);
                    perp_x = dy;
                    perp_y = -dx;
                    norm_len = sqrt(perp_x * perp_x + perp_y * perp_y);
                    perp_x = perp_x / norm_len;
                    perp_y = perp_y / norm_len;
                    perp_x *= position_bias/2;
                    perp_y *= position_bias/2;

                    new_start.resize(start_point.rows()); // VectorXd stores 1 column
                    new_end.resize(end_point.rows());
                    new_start(0) = start_point(0) + perp_x;
                    new_start(1) = start_point(1) + perp_y;
                    new_end(0) = end_point(0) + perp_x;
                    new_end(1) = end_point(1) + perp_y;

                    points.push_back(new_start);
                    if(i == Robot->best_any_angle_path.size() - 1) // upper right point
                        points.push_back(new_end);
                }
                for(int i = Robot->best_any_angle_path.size()-1; i > 0; i--) {
                    start_point = Robot->best_any_angle_path[i];
                    end_point = Robot->best_any_angle_path[i-1];

                    dx = end_point(0) - start_point(0);
                    dy = end_point(1) - start_point(1);
                    perp_x = dy;
                    perp_y = -dx;
                    norm_len = sqrt(perp_x * perp_x + perp_y * perp_y);
                    perp_x = perp_x / norm_len;
                    perp_y = perp_y / norm_len;
                    perp_x *= position_bias/2;
                    perp_y *= position_bias/2;

                    new_start.resize(start_point.rows()); // VectorXd stores 1 column
                    new_end.resize(end_point.rows());
                    new_start(0) = start_point(0) + perp_x;
                    new_start(1) = start_point(1) + perp_y;
                    new_end(0) = end_point(0) + perp_x;
                    new_end(1) = end_point(1) + perp_y;

                    points.push_back(new_start);
                    if(i == 1) // bottom left point
                        points.push_back(new_end);
                }

                for(int i = 0; i < points.size(); i++) {
                    sampling_area(i,0) = points[i](0);
                    sampling_area(i,1) = points[i](1);
                }

                points.clear();

                Q->cspace->drivable_region_.SetPolygon(sampling_area);
            }
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

            // Calculate new node angle if importance sampling this iteration
            if(!rand) {
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
                        = RandDouble(Robot->thetas[min_dist_node_to_path_index]
                                     - theta_bias,
                                     Robot->thetas[min_dist_node_to_path_index]
                                     + theta_bias);
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
            if(collect_timing_data)
                time_file << i++ << " " << delta << "\n";

            {
                lock_guard<mutex> lock(Robot->robot_mutex);
                reached_goal = Robot->goal_reached;
            }
        }
    }
    if(collect_timing_data)
        time_file.close();
}
