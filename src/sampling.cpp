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

MatrixX6d TriangulatePolygon(Eigen::MatrixX2d polygon)
{
    MatrixX6d triangles;

    double vertices[polygon.rows()][2];
//    cout << "Polygon" << endl;
    for(int i = 0; i < polygon.rows(); i++) {
//        cout << i+1 << ": " << polygon(i,0) << "," << polygon(i,1) << endl;
        vertices[i][0] = polygon(i,0);
        vertices[i][1] = polygon(i,1);
    }


    // Populates the tris matrix with the index of the vertex
    // used to create the triangle in that row
    int tris[50][3];
//    cout << "triangulate_polygon(npoints,vertices,triangles)" << endl;
    int num_triangles = triangulate_polygon(polygon.rows(),vertices,tris);

    triangles.resize(num_triangles,Eigen::NoChange_t());
    for(int i = 0; i < num_triangles; i++) {
        triangles(i,0) = polygon(tris[i][0]-1,0); // x1
        triangles(i,1) = polygon(tris[i][0]-1,1); // y1
        triangles(i,2) = polygon(tris[i][1]-1,0); // x2
        triangles(i,3) = polygon(tris[i][1]-1,1); // y2
        triangles(i,4) = polygon(tris[i][2]-1,0); // x3
        triangles(i,5) = polygon(tris[i][2]-1,1); // y3
//        cout << "Triangle #" << i+1 << ": "
//             << triangles(i,0) << "," << triangles(i,1) << " : "
//             << triangles(i,2) << "," << triangles(i,3) << " : "
//             << triangles(i,4) << "," << triangles(i,5) << endl;
    }

    return triangles;
}

/////////////////////// C-Space Functions ///////////////////////

Eigen::VectorXd RandPointDefault(shared_ptr<ConfigSpace> C)
{
    double randvar;
    int num_triangles;
    Eigen::VectorXd point(C->width_.size());

    // This comes back in rows of x1 y1 x2 y2 x3 y3
    MatrixX6d triangles;
    {
        lock_guard<mutex> lock(C->cspace_mutex_);
        triangles = TriangulatePolygon(C->drivable_region_.region_);
    }
    num_triangles = triangles.rows();
    Eigen::Vector2d rand;
    Eigen::MatrixX2d points;
    points.resize(num_triangles,Eigen::NoChange_t());
    for(int i = 0; i < num_triangles; i++) {
        rand(0) = RandDouble(0,1);
        rand(1) = RandDouble(0,1);

        points(i,0) = (1-sqrt(rand(0))) * triangles(i,0)
                    + (sqrt(rand(0))*(1-rand(1))) * triangles(i,2)
                    + (rand(1)*sqrt(rand(0))) * triangles(i,4);
        points(i,1) = (1-sqrt(rand(0))) * triangles(i,1)
                    + (sqrt(rand(0))*(1-rand(1))) * triangles(i,3)
                    + (rand(1)*sqrt(rand(0))) * triangles(i,5);
    }

    randvar = RandDouble(0,num_triangles);
    point(0) = points(floor(randvar),0);
    point(1) = points(floor(randvar),1);
    point(2) = C->lower_bounds_(2)
              + RandDouble(0,C->num_dimensions_) * C->width_(2);

//    for( int i = 0; i < C->width_.size(); i++ ) {
//        rand = RandDouble(0,C->num_dimensions_);
//        first = rand * C->width_(i);
//        second(i) = C->lower_bounds_(i) + first;
//    }

    /// TEMPORARY HACK for Dubin's model
    while(point(2) > 2*PI) point(2) -= 2*PI;
    while(point(2) < -2*PI) point(2) += 2*PI;
    ///
//    cout << "Sampled point:\n" << point << endl;
    return point;
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
