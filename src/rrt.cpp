#include <DRRT/rrt.h>

using namespace std;

void Rrt(CSpace_ptr cspace)
{
    // Importance sampling with Theta*
    double p_uniform = 0.1;  // Probability of sampling uniformly
    bool random = true;
    double position_bias = 6;  // Distance from Theta* line to sample
    double theta_bias = PI/10;  // Angle about Theta* line to sample

    double start_time = GetTimeNs(cspace->start_time_);
    double now = start_time;
    double elapsed_time = 0.0;
    double slice_start;
    double slice_end;
    int slice_counter = 0;

    double initial_dist, hyper_ball_rad;

    bool reached_goal = false;
    double old_lmc = INF;
    int iteration = 1;
    while(!reached_goal)
    {
        // Calculate ball radius
        hyper_ball_rad = min(cspace->saturation_delta_,
                             cspace->ball_constant_*(pow( log(1 + cspace->kdtree_->size_)/cspace->kdtree_->size_,
                                                 1/NUM_DIM)));

        if(cspace->in_warmup_time_ && (cspace->warmup_time_ < cspace->elapsed_time_)) {
            lockguard lock(cspace->mutex_);
            cspace->in_warmup_time_ = false;
        }

        // Run iterations based on timing
        now = GetTimeNs(cspace->start_time_);
        elapsed_time = now - start_time;
        {
            lockguard lock(cspace->mutex_);
            cspace->elapsed_time_ = elapsed_time/1000000000;
        }

        if(elapsed_time >= slice_end)
        {
            if(DEBUG) cout << "Iteration: " << iteration++ << endl;
            if(DEBUG) cout << "Time: " << cspace->elapsed_time_ << "s" << endl;
            slice_start = now;
            slice_end = (++slice_counter)*cspace->slice_time_;

            ReduceInconsistency(cspace, cspace->move_goal_, cspace->kdtree_->root_, hyper_ball_rad);
            if(cspace->move_goal_->GetLmc() != old_lmc)
                old_lmc = cspace->move_goal_->GetLmc();

            // Sample the CSpace
            Kdnode_ptr new_node;
            Kdnode_ptr closest_node;
            double closest_dist;
            Eigen::MatrixX2d sample_region;
            if(true/*RandomDouble(0,1) < p_uniform*/) {
                random = true;
                sample_region.resize(cspace->initial_region_.GetRegion().rows(),
                                     Eigen::NoChange_t());
                sample_region = cspace->initial_region_.GetRegion();
            } /*else {
                random = false;
                sample_region.resize(2*cspace->robot_->theta_star_path.size(),
                                     Eigen::NoChange_t());

                // Create line parallel to the line segment and use it's endpoints
                // Need to go counterclockwise for sampling_area b/c of TriangulatePolygon library
                vector<Eigen::VectorXd> points;
                Eigen::VectorXd start, end, new_start, new_end;
                double dx, dy, perp_x, perp_y, norm_len;
                for(int i = 1; i < cspace->robot_->theta_star_path.size(); i++) {
                    start = cspace->robot_->theta_star_path[i - 1];
                    end = cspace->robot_->theta_star_path[i];

                    dx = end(0) - start(0);
                    dy = end(1) - start(1);
                    perp_x = dy;
                    perp_y = -dx;
                    norm_len = sqrt(pow(perp_x, 2) + pow(perp_y, 2));
                    perp_x /= norm_len;
                    perp_y /= norm_len;
                    perp_x *= position_bias/2;
                    perp_y *= position_bias/2;

                    new_start.resize(start.rows());  // VectorXd is a column
                    new_end.resize(end.rows());
                    new_start(0) = start(0) + perp_x;
                    new_start(1) = start(1) + perp_y;
                    new_end(0) = end(0) + perp_x;
                    new_end(1) = end(1) + perp_y;

                    points.push_back(new_start);
                    if(i == cspace->robot_->theta_star_path.size() - 1)  // upper right point
                        points.push_back(new_end);
                }
                for(int i = cspace->robot_->theta_star_path.size() - 1; i > 0; i--) {
                    start = cspace->robot_->theta_star_path[i];
                    end = cspace->robot_->theta_star_path[i - 1];

                    dx = end(0) - start(0);
                    dy = end(1) - start(1);
                    perp_x = dy;
                    perp_y = -dx;
                    norm_len = sqrt(pow(perp_x, 2) + pow(perp_y, 2));
                    perp_x /= norm_len;
                    perp_y /= norm_len;
                    perp_x *= position_bias/2;
                    perp_y *= position_bias/2;

                    new_start.resize(start.rows());  // VectorXd is a column
                    new_end.resize(end.rows());
                    new_start(0) = start(0) + perp_x;
                    new_start(1) = start(1) + perp_y;
                    new_end(0) = end(0) + perp_x;
                    new_end(1) = end(1) + perp_y;

                    points.push_back(new_start);
                    if(i == 1)  // bottom left point
                        points.push_back(new_end);
                }

                for(int i = 0; i < points.size(); i++) {
                    sample_region(i, 0) = points[i](0);
                    sample_region(i, 1) = points[i](1);
                }

                points.clear();
                cspace->drivable_region_.SetRegion(sample_region);
            }*/

            new_node = RandomNodeOrFromStack(cspace);

            if(new_node->InTree()) continue;
            {
                lockguard lock(cspace->kdtree_->mutex_);
                closest_dist
                        = cspace->kdtree_->FindNearest(closest_node, new_node->GetPosition());
            }

            // Saturate this node
            initial_dist = DistanceFunction(new_node->GetPosition(), closest_node->GetPosition());
            if(initial_dist > cspace->saturation_delta_ && new_node != cspace->goal_node_) {
                new_node->SetPosition(
                            new_node->Saturate(new_node->GetPosition(),
                                               closest_node->GetPosition(),
                                               cspace->saturation_delta_,
                                               initial_dist));
            }

            if(NodeCheck(cspace, new_node)) continue;

            // Calculate new_node angle if importance sampling this iteration
            if(!random) {
                // Find the closest line on the theta* path to the new_node
                double dist_node_to_path;
                double min_dist_node_to_path = INF;
                int min_dist_node_to_path_index = 0;
                for(int i = 1; i < cspace->robot_->theta_star_path.size(); i++) {
                    dist_node_to_path = DistPointToSegmentSqrd(
                                new_node->GetPosition(),
                                cspace->robot_->theta_star_path.at(i - 1).head(2),
                                cspace->robot_->theta_star_path.at(i).head(2));
                    if(dist_node_to_path < min_dist_node_to_path) {
                        min_dist_node_to_path = dist_node_to_path;
                        min_dist_node_to_path_index = i - 1;
                    }
                }

                // Calculate theta bias
                new_node->SetPosition(Eigen::Vector3d(new_node->GetPosition()(0),
                                                      new_node->GetPosition()(1),
    RandomDouble(cspace->robot_->thetas[min_dist_node_to_path_index] - theta_bias,
                 cspace->robot_->thetas[min_dist_node_to_path_index] + theta_bias)
                                                      )
                                      );
            }

            // Extend the graph
            Extend(cspace, new_node, closest_node, hyper_ball_rad);

            // Make graph consistent
            ReduceInconsistency(cspace, cspace->move_goal_, cspace->root_node_, hyper_ball_rad);
            if(cspace->move_goal_->GetLmc() != old_lmc)
                old_lmc = cspace->move_goal_->GetLmc();
        }

        {
            lockguard lock(cspace->robot_->mutex);
            reached_goal = cspace->robot_->goal_reached;
        }
    }
}
