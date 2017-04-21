/* sampling.cpp
 * Corin Sandford
 * Spring 2017
 * C-Space sampling functions
 */

#include <DRRT/sampling.h>

using namespace std;

double RandDouble( double min, double max )
{
    uniform_real_distribution<double> unid(min,max);
    mt19937 rng; // Mersenn-Twister random-number engine
    rng.seed(random_device{}());
    double random_double = unid(rng);
    return random_double;
}

/////////////////////// Region2D Functions ///////////////////////

Eigen::MatrixX3d TriangulatePolygon(Eigen::MatrixX2d polygon)
{
    Eigen::MatrixX3d triangles;

    int num_mono_polys, count, num_points, genus;
    int n, i;

    i = 1;

    int first, last;

    num_points = polygon.rows();
    first = i;
    last = first + num_points - 1;

    for(int j = 0; j < num_points; j++) {

    }

    return triangles;
}

/////////////////////// C-Space Functions ///////////////////////

Eigen::VectorXd RandPointDefault(shared_ptr<ConfigSpace> C)
{
    double rand;
    double first;
    Eigen::VectorXd second(C->width_.size());

    for( int i = 0; i < C->width_.size(); i++ ) {
        rand = RandDouble(0,C->num_dimensions_);
        first = rand * C->width_(i);
        second(i) = C->lower_bounds_(i) + first;
    }

    /// TEMPORARY HACK?
    while(second(2) > 2*PI) second(2) -= 2*PI;
    while(second(2) < -2*PI) second(2) += 2*PI;
    ///
    return second;
}

shared_ptr<KDTreeNode> RandNodeDefault(shared_ptr<ConfigSpace> C)
{
    Eigen::VectorXd point = RandPointDefault(C);
    return make_shared<KDTreeNode>(point);
}

shared_ptr<KDTreeNode> RandNodeOrGoal(shared_ptr<ConfigSpace> C)
{
    double r = (double)rand()/(RAND_MAX);
    if( r > C->prob_goal_ ) {
        return RandNodeDefault(C);
    } else {
        return C->goal_node_;
    }
}

shared_ptr<KDTreeNode> RandNodeIts(shared_ptr<ConfigSpace> C)
{
    if( C->iterations_until_sample_ == 0 ) {
        C->iterations_until_sample_ -= 1;
        return make_shared<KDTreeNode>(C->iteration_sample_point_);
    }
    C->iterations_until_sample_ -= 1;
    return RandNodeOrGoal(C);
}

shared_ptr<KDTreeNode> RandNodeTime(shared_ptr<ConfigSpace> C)
{
    if( C->wait_time_ != INF && C->time_elapsed_ >= C->wait_time_ ) {
        C->wait_time_ = INF;
        return make_shared<KDTreeNode>(C->time_sample_point_);
    }
    return RandNodeOrGoal( C );
}

//shared_ptr<KDTreeNode> randNodeTimeWithObstacleRemove( shared_ptr<ConfigSpace> S ){}
//shared_ptr<KDTreeNode> randNodeItsWithObstacleRemove( shared_ptr<ConfigSpace> S ){}

shared_ptr<KDTreeNode> RandNodeOrFromStack(shared_ptr<ConfigSpace> &C)
{
    if( C->sample_stack_->length_ > 0 ) {
        // Using the sample_stack_ so KDTreeNode->position_ is popped
        shared_ptr<KDTreeNode> temp = make_shared<KDTreeNode>();
        C->sample_stack_->JListPop(temp);
        return temp;
    } else {
        return RandNodeOrGoal( C );
    }
}

shared_ptr<KDTreeNode> RandNodeInTimeOrFromStack(shared_ptr<ConfigSpace> C)
{
    if( C->sample_stack_->length_ > 0 ) {
        // Using the sample_stack_ so KDTreeNode->position_ is popped
        shared_ptr<KDTreeNode> temp = make_shared<KDTreeNode>();
        C->sample_stack_->JListPop(temp);
        return make_shared<KDTreeNode>(temp->position_);
    } else {
        shared_ptr<KDTreeNode> newNode = RandNodeOrGoal( C );
        if( newNode == C->goal_node_ ) {
            return newNode;
        }

        double minTimeToReachNode = C->start_(2)
                + sqrt(
                        (newNode->position_(0) - C->root_->position_(0))
                        *(newNode->position_(0) - C->root_->position_(0))
                        + (newNode->position_(1) - C->root_->position_(1))
                        *(newNode->position_(1) - C->root_->position_(1))
                      ) / C->robot_velocity_;

        // If point is too soon vs robot's available speed
        // or if it is in the "past" and the robot is moving
        if( newNode->position_(2) < minTimeToReachNode ||
                (newNode->position_(2) > C->move_goal_->position_(2) &&
                 C->move_goal_ != C->goal_node_) ) {
            // Resample time in ok range
            double r = (double) rand() / (RAND_MAX);
            newNode->position_(2) = minTimeToReachNode
                    + r * (C->move_goal_->position_(2) - minTimeToReachNode);
        }
        return newNode;
    }
}
