#include <DRRT/rrt.h>

using namespace std;

void Rrt(CSpace_ptr &cspace, double ball_constant)
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

    double current_dist, initial_dist, hyper_ball_rad;
    Eigen::VectorXd last_pose;
    {
        lockguard lock(cspace->robot_->mutex);
        last_pose = cspace->robot_->pose;
    }

    current_dist = DistanceFunction(last_pose, cspace->kdtree_->root_->GetPosition());

    bool reached_goal = false;
    double old_lmc = INF;
    while(!reached_goal)
    {
        // Calculate ball radius
        hyper_ball_rad = min(cspace->saturation_delta_,
                             ball_constant*(pow( log(1 + cspace->kdtree_->size_)/cspace->kdtree_->size_,
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
            cspace->elapsed_time_ = elapsed_time;
        }

        if(elapsed_time >= slice_end)
        {
            slice_start = now;
            slice_end = (++slice_counter)*cspace->slice_time_;

            ReduceInconsistency(cspace, cspace->move_goal_, cspace->kdtree_->root_, hyper_ball_rad);
            if(cspace->move_goal_->GetLmc() != old_lmc)
                old_lmc = cspace->move_goal_->GetLmc();

            // Sample the CSpace
            Kdnode_ptr new_node;
            Kdnode_ptr closest_node;
            double closest_dist;
            if(RandomDouble(0,1) < p_uniform) {

            }
        }
    }
}
