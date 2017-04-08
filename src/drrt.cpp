/* drrt.cpp
 * Corin Sandford
 * Fall 2016
 * Contains RRTX "main" function at bottom.
 */

#include <DRRT/drrt.h>

using namespace std;

bool timing = false;
int seg_dist_sqrd = 0;
int dist_sqrd_point_seg = 0;

///////////////////// Print Helpers ///////////////////////
void error(string s) { cout << s << endl; }
void error(int i) { cout << i << endl; }

double GetTimeNs( chrono::time_point
                  <chrono::high_resolution_clock> start )
{
    return chrono::duration_cast<chrono::nanoseconds>(
                chrono::high_resolution_clock::now()-start).count();
}

double RandDouble( double min, double max )
{    
    uniform_real_distribution<double> unid(min,max);
    mt19937 rng; // Mersenn-Twister random-number engine
    rng.seed(random_device{}());
    double random_double = unid(rng);
    return random_double;
}


/////////////////////// Node Functions ///////////////////////

double extractPathLength( shared_ptr<KDTreeNode> node,
                          shared_ptr<KDTreeNode> root )
{
    double pathLength = 0.0;
    shared_ptr<KDTreeNode> thisNode = node;
    while( node != root ) {
        if( !node->rrt_parent_used_ ) {
            pathLength = INF;
            break;
        }
        pathLength += node->rrt_parent_edge_->dist_;
        thisNode = thisNode->rrt_parent_edge_->end_node_;
    }

    return pathLength;
}


/////////////////////// C-Space Functions ///////////////////////

Eigen::VectorXd randPointDefault( shared_ptr<ConfigSpace> S )
{
    double rand;
    double first;
    Eigen::VectorXd second(S->width_.size());

    for( int i = 0; i < S->width_.size(); i++ ) {
        rand = RandDouble(0,S->num_dimensions_);
        first = rand * S->width_(i);
        second(i) = S->lower_bounds_(i) + first;
    }

    /// TEMPORARY HACK
    while(second(2) > 2*PI) second(2) -= 2*PI;
    while(second(2) < -2*PI) second(2) += 2*PI;
    ///
    return second;
}

shared_ptr<KDTreeNode> randNodeDefault( shared_ptr<ConfigSpace> S )
{
    Eigen::VectorXd point = randPointDefault( S );
    return make_shared<KDTreeNode>(point);
}

shared_ptr<KDTreeNode> randNodeOrGoal( shared_ptr<ConfigSpace> S )
{
    double r = (double)rand()/(RAND_MAX);
    if( r > S->prob_goal_ ) {
        return randNodeDefault( S );
    } else {
        return S->goal_node_;
    }
}

shared_ptr<KDTreeNode> randNodeIts(shared_ptr<ConfigSpace> S)
{
    if( S->iterations_until_sample_ == 0 ) {
        S->iterations_until_sample_ -= 1;
        return make_shared<KDTreeNode>(S->iteration_sample_point_);
    }
    S->iterations_until_sample_ -= 1;
    return randNodeOrGoal( S );
}

shared_ptr<KDTreeNode> randNodeTime(shared_ptr<ConfigSpace> S)
{
    if( S->wait_time_ != INF && S->time_elapsed_ >= S->wait_time_ ) {
        S->wait_time_ = INF;
        return make_shared<KDTreeNode>(S->time_sample_point_);
    }
    return randNodeOrGoal( S );
}

//shared_ptr<KDTreeNode> randNodeTimeWithObstacleRemove( shared_ptr<ConfigSpace> S ){}
//shared_ptr<KDTreeNode> randNodeItsWithObstacleRemove( shared_ptr<ConfigSpace> S ){}

shared_ptr<KDTreeNode> randNodeOrFromStack(shared_ptr<ConfigSpace> &S)
{
    if( S->sample_stack_->length_ > 0 ) {
        // Using the sample_stack_ so KDTreeNode->position_ is popped
        shared_ptr<KDTreeNode> temp = make_shared<KDTreeNode>();
        S->sample_stack_->JListPop(temp);
        return temp;
    } else {
        return randNodeOrGoal( S );
    }
}

shared_ptr<KDTreeNode> randNodeInTimeOrFromStack(shared_ptr<ConfigSpace> S)
{
    if( S->sample_stack_->length_ > 0 ) {
        // Using the sample_stack_ so KDTreeNode->position_ is popped
        shared_ptr<KDTreeNode> temp = make_shared<KDTreeNode>();
        S->sample_stack_->JListPop(temp);
        return make_shared<KDTreeNode>(temp->position_);
    } else {
        shared_ptr<KDTreeNode> newNode = randNodeOrGoal( S );
        if( newNode == S->goal_node_ ) {
            return newNode;
        }

        double minTimeToReachNode = S->start_(2)
                + sqrt(
                        (newNode->position_(0) - S->root_->position_(0))
                        *(newNode->position_(0) - S->root_->position_(0))
                        + (newNode->position_(1) - S->root_->position_(1))
                        *(newNode->position_(1) - S->root_->position_(1))
                      ) / S->robot_velocity_;

        // If point is too soon vs robot's available speed
        // or if it is in the "past" and the robot is moving
        if( newNode->position_(2) < minTimeToReachNode ||
                (newNode->position_(2) > S->move_goal_->position_(2) &&
                 S->move_goal_ != S->goal_node_) ) {
            // Resample time in ok range
            double r = (double) rand() / (RAND_MAX);
            newNode->position_(2) = minTimeToReachNode
                    + r * (S->move_goal_->position_(2) - minTimeToReachNode);
        }
        return newNode;
    }
}

/////////////////////// Geometric Functions ///////////////////////

double DistanceSqrdPointToSegment(Eigen::VectorXd point,
                                  Eigen::Vector2d startPoint,
                                  Eigen::Vector2d endPoint)
{
    /// For timing
    chrono::time_point<chrono::high_resolution_clock> startTime
            = chrono::high_resolution_clock::now();
    double time_start = GetTimeNs(startTime);
    dist_sqrd_point_seg++;

    Eigen::Vector2d point_position = point.head(2);
    double vx = point_position(0) - startPoint(0);
    double vy = point_position(1) - startPoint(1);
    double ux = endPoint(0) - startPoint(0);
    double uy = endPoint(1) - startPoint(1);
    double determinate = vx*ux + vy*uy;

    double time_end;
    if( determinate <= 0 ) {
        time_end = GetTimeNs(startTime);
        if(timing) cout << "DistanceSqrdPointToSegment: "
                        << (time_end - time_start)/MICROSECOND << " ms" << endl;
        return vx*vx + vy*vy;
    } else {
        double len = ux*ux + uy*uy;
        if( determinate >= len ) {
            time_end = GetTimeNs(startTime);
            if(timing) cout << "DistanceSqrdPointToSegment: "
                            << (time_end - time_start)/MICROSECOND << " ms" << endl;
            return (endPoint(0)-point_position(0))
                    *(endPoint(0)-point_position(0))
                    + (endPoint(1)-point_position(1))
                    *(endPoint(1)-point_position(1));
        } else {
            time_end = GetTimeNs(startTime);
            if(timing) cout << "DistanceSqrdPointToSegment: "
                            << (time_end - time_start)/MICROSECOND << " ms" << endl;
            return (ux*vy - uy*vx)*(ux*vy - uy*vx) / len;
        }
    }
}


double DistToPolygonSqrd(Eigen::VectorXd point, Eigen::MatrixX2d polygon)
{
    double min_dist_sqrd = INF;

    // Start with the last vs first point
    Eigen::Vector2d start_point = polygon.row(polygon.rows()-1);
    Eigen::Vector2d end_point;
    double this_dist_sqrd;
    for(int i = 0; i < polygon.rows(); i++) {
        end_point = polygon.row(i);
        this_dist_sqrd = DistanceSqrdPointToSegment(point, start_point, end_point);
        if(this_dist_sqrd < min_dist_sqrd) min_dist_sqrd = this_dist_sqrd;
        start_point = end_point;
    }
    return min_dist_sqrd;
}

double SegmentDistSqrd(Eigen::VectorXd PA, Eigen::VectorXd PB,
                       Eigen::VectorXd QA, Eigen::VectorXd QB)
{
    /// Timing
    chrono::time_point<chrono::high_resolution_clock> startTime
            = chrono::high_resolution_clock::now();
    double time_start = GetTimeNs(startTime);
    // Check if the points are definately not in collision by seeing
    // if both points of Q are on the same side of line containing P and vice versa

    bool possibleIntersect = true;
    double m, diffA, diffB;

    // First check if P is close to vertical
    if( abs(PB(0) - PA(0)) < 0.000001 ) {
        // P is close to vertical
        if( (QA(0) >= PA(0) && QB(0) >= PA(0))
                || (QA(0) <= PA(0) && QB(0) <= PA(0)) ) {
            // Q is on one side of P
            possibleIntersect = false;
        }
    } else {
        // P is not close to vertical
        m = (PB(1) - PA(1)) / (PB(0) - PA(0));

        // Equation for points on P: y = m(x - PA[1]) + PA[2]
        diffA = (m*(QA(0)-PA(0)) + PA(1)) - QA(1);
        diffB = (m*(QB(0)-PA(0)) + PA(1)) - QB(1);
        if( (diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0) ) {
            // Q is either fully above or below the line containing P
            possibleIntersect = false;
        }
    }

    if( possibleIntersect ) {
        // first check if Q is close to vertical
        if( abs(QB(0) - QA(0)) < 0.000001 ) {
            if( (PA(0) >= QA(0) && PB(0) >= QA(0)) || (PA(0) <= QA(0) && PB(0) <= QA(0)) ) {
                // P is on one side of Q
                possibleIntersect = false;
            }
        } else {
            // Q is not close to vertical
            m = (QB(1) - QA(1)) / (QB(0) - QA(0));

            // Equation for points on Q: y = m(x-QA[1]) + QA[2]
            diffA = (m*(PA(0)-QA(0)) + QA(1)) - PA(1);
            diffB = (m*(PB(0)-QA(0)) + QA(1)) - PB(1);
            if( (diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0) ) {
                // P is either fully above or below the line containing Q
                possibleIntersect = false;
            }
        }
    }

    if( possibleIntersect ) {
        // Then there is an intersection for sure
        return 0.0;
    }

    // When the lines do not intersect in 2D, the min distance must
    // be between one segment's end point and the other segment
    // (assuming lines are not parallel)
    Eigen::Vector4d distances;
    Eigen::Vector2d QA_pos = QA.head(2);
    Eigen::Vector2d QB_pos = QB.head(2);
    Eigen::Vector2d PA_pos = PA.head(2);
    Eigen::Vector2d PB_pos = PB.head(2);
    distances(0) = DistanceSqrdPointToSegment(PA,QA_pos,QB_pos);
    distances(1) = DistanceSqrdPointToSegment(PB,QA_pos,QB_pos);
    distances(2) = DistanceSqrdPointToSegment(QA,PA_pos,PB_pos);
    distances(3) = DistanceSqrdPointToSegment(QB,PA_pos,PB_pos);

    double time_end = GetTimeNs(startTime);
    if(timing) cout << "SegmentDistSqrd: " << (time_end - time_start)/MICROSECOND << " ms" << endl;
    seg_dist_sqrd++;
    return distances.minCoeff();
}

Eigen::Vector2d FindTransformObjToTimeOfPoint(shared_ptr<Obstacle> O,
                                              Eigen::Vector3d point)
{
    // Start by finding the indicies of the path edge containing time
    int index_before = FindIndexBeforeTime(O->path_,point(2));

    Eigen::Vector2d offset;
    if(index_before < 1) {
        offset(0) = O->path_.row(0)(0);
        offset(1) = O->path_.row(0)(1);
        return offset;
    } else if(index_before == O->path_.rows()) {
        offset(0) = O->path_.row(O->path_.rows())(0);
        offset(1) = O->path_.row(O->path_.rows())(1);
        return offset;
    }

    int index_after = index_before + 1;
    double proportion_along_edge = (point(2) - O->path_.row(index_before)(2))
                                    / (O->path_.row(index_after)(2)
                                       - O->path_.row(index_before)(2));

    offset(0) = O->path_.row(index_before)(0)
            + proportion_along_edge * (O->path_.row(index_after)(0)
                                       - O->path_.row(index_before)(0));
    offset(1) = O->path_.row(index_before)(1)
            + proportion_along_edge * (O->path_.row(index_after)(1)
                                       - O->path_.row(index_before)(1));
    return offset;
}

bool PointInPolygon(Eigen::VectorXd this_point, Eigen::MatrixX2d polygon)
{
    Eigen::Vector2d point = this_point.head(2);
    // MacMartin crossings test
    if(polygon.rows() < 2) return false;

    int num_crossings = 0;
    Eigen::Vector2d start_point = polygon.row(polygon.rows()-1);
    Eigen::Vector2d end_point;
    double x;
    for(int i = 0; i < polygon.rows(); i++) {
        end_point = polygon.row(i);

        // Check if edge crosses the y-value of the point
        if((start_point(1) > point(1) && end_point(1) < point(1))
           || (start_point(1) < point(1) && end_point(1) > point(1))) {
            // It does, no check if ray from point -> (INF,0) intersects
            if(start_point(0) > point(0) && end_point(0) > point(0)) {
                // Definitely yes if both x coordinates are right of the point
                num_crossings++;
            } else if( start_point(0) < point(0) && end_point(0) < point(0)) {
                // Definitely not if both x coordinates are left of the point
            } else { // Have to do "expensive" calculation
                double t = 2*max(start_point(0),end_point(0));
                x = (-((start_point(0)*end_point(1)
                        - start_point(1)*end_point(0))*(point(0)-t))
                     + ((start_point(0)-end_point(0)) * (point(0)*point(1)
                                                         -point(1)*t)))
                     / ((start_point(1)-end_point(1)) * (point(0)-t));

                if(x>point(0)) num_crossings++;
            }
        }
        start_point = end_point;
    }

    // Check crossings (odd means point inside polygon)
    if(num_crossings % 2 == 1) return true;
    return false;
}

int FindIndexBeforeTime(Eigen::MatrixXd path, double timeToFind)
{
    if( path.rows() < 1) {
        return -1;
    }

    int i = -1;
    while( i+1 < path.rows() && path.row(i+1)(2) < timeToFind ) {
        i += 1;
    }
    return i;
}


/////////////////////// Collision Checking Functions ///////////////////////

shared_ptr<JList> FindPointsInConflictWithObstacle(shared_ptr<ConfigSpace> &S,
                                                   shared_ptr<KDTree> Tree,
                                                   shared_ptr<Obstacle> &O,
                                                   shared_ptr<KDTreeNode> &root)
{
    shared_ptr<JList> node_list = make_shared<JList>(true);
    double search_range = 0;

    if(1 <= O->kind_ && O->kind_ <= 5) {
        // 2D obstacle
        if(!S->space_has_time_ && !S->space_has_theta_) {
            // Euclidean space without time
            search_range = S->robot_radius_ + S->saturation_delta_ + O->radius_;
            Tree->KDFindWithinRange(node_list,search_range,O->position_);
        } else if(!S->space_has_time_ && S->space_has_theta_) {
            // Dubin's robot without time [x,y,theta]
            search_range = S->robot_radius_ + O->radius_ + PI; // + S->saturation_delta_
//            cout << "search_range: " << search_range << endl;
            Eigen::Vector3d obs_center_dubins;
            obs_center_dubins << O->position_(0), O->position_(1), PI;
//            cout << "about point:\n" << obs_center_dubins << endl;
            Tree->KDFindWithinRange(node_list,search_range,obs_center_dubins);
//            cout << "number of points in conflict: " << node_list->length_ << endl;
        } else {
            cout << "Error: This type of obstacle not coded for this space"
                 << endl;
        }
    } else if(6 <= O->kind_ && O->kind_ <= 7) {
        // 2D obstacle with time, find points within range of each point along
        // the time path, accumulating all points that are in any of the
        // bounding hyperspheres
        // [x,y,theta,time]

        double base_search_range = S->robot_radius_ + S->saturation_delta_ + O->radius_;
        double search_range;
        int j;
        Eigen::Vector4d query_pose;
        Eigen::Array4d temp, temp1;
        for(int i = 0; i < O->path_.rows(); i++) {
            // Makey query pose the middle of the edge, and add 1/2 edge length
            // through the C-Space to the base_search_range (overestimate)

            if(O->path_.rows() == 1) j = 1;
            else j = i+1;

            temp = O->path_.row(i);
            temp1 = O->path_.row(j);

            temp = temp + temp1;
            temp = temp/2.0;

            temp1 = O->position_;

            query_pose << temp1 + temp;
            cout << "query_pose:\n" << query_pose << endl;
            cout << "O->position_:\n" << O->position_ << endl;
            cout << "temp:\n" << temp << endl;

            if(S->space_has_theta_) query_pose << query_pose, PI;
            cout << "query_pose:\n" << query_pose << endl;

            search_range = base_search_range
                    + Tree->distanceFunction(O->path_.row(i),
                                             O->path_.row(j))/2.0;

            if(S->space_has_theta_) search_range += PI;

            if(i == 1)
                Tree->KDFindWithinRange(node_list,search_range,query_pose);
            else
                Tree->KDFindMoreWithinRange(node_list,search_range,query_pose);

            if(j == O->path_.rows()) break;
        }
    } else cout << "This case has not been coded yet." << endl;

    return node_list;
}

void AddNewObstacle(shared_ptr<KDTree> Tree,
                    shared_ptr<Queue> &queue,
                    shared_ptr<Obstacle> &O,
                    shared_ptr<KDTreeNode> root)
{
    cout << "AddNewObstacle" << endl;
    // Find all points in conflict with the obstacle
    shared_ptr<JList> node_list
            = FindPointsInConflictWithObstacle(queue->cspace,Tree,O,root);

    // For all nodes that might be in conflict
    shared_ptr<KDTreeNode> this_node;
    shared_ptr<double> key = make_shared<double>(0);
//    cout << "points in conflict: " << node_list->length_ << endl;
    while(node_list->length_ > 0) {
//        cout << "point in conflict: " << endl;
        Tree->PopFromRangeList(node_list,this_node,key);
//        cout << this_node->position_ << endl;
        // Check all their edges

        // See if this node's neighbors can be reached

        // Get an iterator for this node's out neighbors
        shared_ptr<RrtNodeNeighborIterator> this_node_out_neighbors
                = make_shared<RrtNodeNeighborIterator>(this_node);

        // Iterate through list
        shared_ptr<JListNode> list_item
                = nextOutNeighbor(this_node_out_neighbors);
        shared_ptr<JListNode> next_item;
        shared_ptr<Edge> neighbor_edge;
        while(list_item->key_ != -1.0) {
            neighbor_edge = list_item->edge_;
            next_item = nextOutNeighbor(this_node_out_neighbors);
            if(neighbor_edge->ExplicitEdgeCheck(O))
                // Mark edge to neighbor at INF cost
                list_item->edge_->dist_ = INF;
            list_item = next_item;
        }

        // See if this node's parent can be reached
        if(this_node->rrt_parent_used_
                && this_node->rrt_parent_edge_->ExplicitEdgeCheck(O)) {
            // Remove this_node from it's parent's successor list
            this_node->rrt_parent_edge_->end_node_->successor_list_->JListRemove(
                        this_node->successor_list_item_in_parent_);

            // This node now has no parent
            this_node->rrt_parent_edge_->end_node_ = this_node;
            this_node->rrt_parent_edge_->dist_ = INF;
            this_node->rrt_parent_used_ = false;

            verifyInOSQueue(queue,this_node);
        }
    }

    // Clean up
    Tree->EmptyRangeList(node_list);
}

void RemoveObstacle(std::shared_ptr<KDTree> Tree,
                    std::shared_ptr<Queue> &Q,
                    std::shared_ptr<Obstacle> &O,
                    std::shared_ptr<KDTreeNode> root,
                    double hyper_ball_rad, double time_elapsed_,
                    std::shared_ptr<KDTreeNode> &move_goal)
{
    cout << "RemoveObstacle" << endl;
    bool neighbors_were_blocked, conflicts_with_other_obs;

    // Find all points in conflict with obstacle
    shared_ptr<JList> node_list
            = FindPointsInConflictWithObstacle(Q->cspace,Tree,O,root);

    // For all nodes that might be in conflict
    shared_ptr<KDTreeNode> this_node = make_shared<KDTreeNode>();
    shared_ptr<double> key = make_shared<double>(0);
//    cout << "points in conflict: " << node_list->length_ << endl;
    while(node_list->length_ > 0) {
//        cout << "point in conflict: " << endl;
        Tree->PopFromRangeList(node_list,this_node,key);
//        cout << this_node->position_ << endl;
        // Check all of their edges

        // See if this node's out neighbors were blocked by the obstacle

        // Get an iterator for this node's out neighbors
        shared_ptr<RrtNodeNeighborIterator> this_node_out_neighbors
                = make_shared<RrtNodeNeighborIterator>(this_node);
        neighbors_were_blocked = false;

        // Iterate through list
        shared_ptr<JListNode> list_item
                = nextOutNeighbor(this_node_out_neighbors);
        shared_ptr<ListNode> o_list_item;
        shared_ptr<JListNode> next_item;
        shared_ptr<Edge> neighbor_edge;
        shared_ptr<KDTreeNode> neighbor_node;
        while(list_item->key_ != -1.0) {
            neighbor_edge = list_item->edge_;
            neighbor_node = list_item->edge_->end_node_;
            next_item = nextOutNeighbor(this_node_out_neighbors);
            if(neighbor_edge->dist_ == INF
                    && neighbor_edge->ExplicitEdgeCheck(O)) {
                // This edge used to be in collision with at least one
                // obstacle (at least the obstacle in question)
                // Need to check if could be in conflict with other obstacles

                {
                    lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
                    o_list_item = Q->cspace->obstacles_->front_;
                }
                conflicts_with_other_obs = false;
                shared_ptr<Obstacle> other_obstacle;
                while(o_list_item != o_list_item->child_) {
                    other_obstacle = o_list_item->obstacle_;
                    if(other_obstacle != O
                            && other_obstacle->obstacle_used_
                            && other_obstacle->start_time_ <= time_elapsed_
                            && time_elapsed_ <= (other_obstacle->start_time_
                                               + other_obstacle->life_span_)) {
                        if(neighbor_edge->ExplicitEdgeCheck(other_obstacle)) {
                            conflicts_with_other_obs = true;
                            break;
                        }
                    }
                    o_list_item = o_list_item->child_;
                }

                if(!conflicts_with_other_obs) {
                    // Reset edge length_ to actual cost
                    list_item->edge_->dist_ = list_item->edge_->dist_original_;
                    neighbors_were_blocked = true;
                }
            }
            list_item = next_item;
        }

        if(neighbors_were_blocked) {
            recalculateLMC(Q,this_node,root,hyper_ball_rad);
            if(this_node->rrt_tree_cost_ != this_node->rrt_LMC_
                    && Q->priority_queue->lessThan(this_node,move_goal))
                verifyInQueue(Q,this_node);
        }
    }
    Tree->EmptyRangeList(node_list);
    O->obstacle_used_ = false;
}

bool checkHeapForEdgeProblems( shared_ptr<Queue> &Q,
                               shared_ptr<KDTree> Tree )
{
    shared_ptr<KDTreeNode> node;
    for( int i = 0; i < Q->priority_queue->index_of_last_; i++ ) {
        node = Q->priority_queue->heap_[i];
        if( checkNeighborsForEdgeProblems( Q->cspace, node, Tree ) ) return true;
    }
    return false;
}

bool checkNeighborsForEdgeProblems(shared_ptr<ConfigSpace>& S,
                                   shared_ptr<KDTreeNode> thisNode,
                                   shared_ptr<KDTree> Tree)
{
    shared_ptr<Edge> this_edge_1, this_edge_2;
    if( thisNode->rrt_parent_used_ ) {
        this_edge_1 = Edge::NewEdge(S, Tree, thisNode,
                                                   thisNode->rrt_parent_edge_->end_node_);
        if( ExplicitEdgeCheck(S, this_edge_1)) {
            return true;
        }
    }

    shared_ptr<JListNode> listItem = thisNode->rrt_neighbors_out_->front_;
    shared_ptr<KDTreeNode> neighborNode;
    while( listItem != listItem->child_ ) {
        neighborNode = listItem->node_;

        this_edge_2 = Edge::NewEdge(S, Tree, neighborNode,
                                  neighborNode->rrt_parent_edge_->end_node_);

        if( neighborNode->rrt_parent_used_
                && ExplicitEdgeCheck(S,this_edge_2)) {
            return true;
        }

        listItem = listItem->child_; // iterate
    }
    return false;
}

bool LineCheck(std::shared_ptr<ConfigSpace> C,
               std::shared_ptr<KDTree> Tree,
               std::shared_ptr<KDTreeNode> node1,
               std::shared_ptr<KDTreeNode> node2) {
    // Save the actual angles of these nodes
    double saved_theta1 = node1->position_(2);
    double saved_theta2 = node2->position_(2);
    // Calculate the angle between the two nodes
    double theta = std::atan2(node2->position_(1)-node1->position_(1),
                              node2->position_(0)-node1->position_(0));
    // Make these nodes point in the same direction
    node1->position_(2) = theta;
    node2->position_(2) = theta;
    // Find the trajectory between them (should be a straight line)
    std::shared_ptr<Edge> edge = Edge::NewEdge(C,Tree,node1,node2);
    // Create trajectory straight line from node1 --> node2
    double traj_length = 50;
    Eigen::VectorXd x_traj = Eigen::VectorXd::Zero(traj_length+1);
    Eigen::VectorXd y_traj = Eigen::VectorXd::Zero(traj_length+1);
    double x_val = node1->position_(0);
    double y_val = node1->position_(1);
    double x_dist = abs(node2->position_(0) - x_val);
    double y_dist = abs(node2->position_(1) - y_val);
    int i = 0;
    while(i < traj_length) {
        x_traj(i) = x_val;
        if(node2->position_(0) > node1->position_(0))
            x_val += x_dist/traj_length;
        else x_val -= x_dist/traj_length;

        y_traj(i) = y_val;
        if(node2->position_(1) > node1->position_(1))
            y_val += y_dist/traj_length;
        else y_val -= y_dist/traj_length;

        i++;
    }
    x_traj(i) = x_val;
    y_traj(i) = y_val;
    edge->trajectory_.resize(traj_length+1,2);
    edge->trajectory_.col(0) = x_traj;
    edge->trajectory_.col(1) = y_traj;
    // Check if this straight line trajectory is valid
    bool unsafe = ExplicitEdgeCheck(C,edge);
    // Restore the nodes' original angles (for some reason if I get rid of
    // this it dies but this doesn't mean anything since theta* doesn't use
    // the theta dimension)
    node1->position_(2) = saved_theta1;
    node2->position_(2) = saved_theta2;
    // Return true if there this edge is unsafe and there is no line of sight
    return unsafe;
}

bool ExplicitEdgeCheck2D(shared_ptr<Obstacle> &O,
                         Eigen::VectorXd start_point,
                         Eigen::VectorXd end_point,
                         double radius)
{
    /// For timing
    double time_start, time_end;
    chrono::time_point<chrono::high_resolution_clock> startTime
            = chrono::high_resolution_clock::now();

    if(!O->obstacle_used_ || O->life_span_ <= 0) return false;

    // Do a quick check to see if any points on the obstacle might be closer
    // to the edge than robot radius
    if(1 <= O->kind_ && O->kind_ <= 5) {

        // Calculate distance squared from center of the obstacle to the edge
        // projected into the first two dimensions
        double dist_sqrd = DistanceSqrdPointToSegment(O->position_,
                                                      start_point.head(2),
                                                      end_point.head(2));

//        cout << "obs:\n" << O->position_.head(2) << endl;
//        cout << "line:\n" << start_point.head(2) << endl << "--" << endl
//             << end_point.head(2) << endl;
//        cout << "dist: " << sqrt(dist_sqrd) << endl;
//        cout << "min: " << radius+O->radius_ << endl;
        if(dist_sqrd > pow((radius + O->radius_),2)) return false;
    }

    if(O->kind_ == 1) return true; // ball is the obstacle, so in collision
    else if( O->kind_ == 2) return false;
    else if(O->kind_ == 3 || O->kind_ == 5) {
        // Need to check vs all edges in the polygon
        if(O->polygon_.rows() < 2) return false;

        // Start with the last point vs the first point
        time_start = GetTimeNs(startTime);
        Eigen::Vector2d A = O->polygon_.row(O->polygon_.rows()-1);
        Eigen::Vector2d B;
        double seg_dist_sqrd;
        for(int i = 0; i < O->polygon_.rows(); i++) {
            B = O->polygon_.row(i);
            seg_dist_sqrd = SegmentDistSqrd(start_point,end_point,A,B);
//            cout << "dist between edge and polygon edge: " << sqrt(seg_dist_sqrd) << endl;
            if(seg_dist_sqrd < pow(radius,2)) {
                // There is a collision with the 2d projection of the obstacle
//                cout << "p_edge -- traj_edge: " << sqrt(seg_dist_sqrd)
//                     << "\n<\nradius:" << radius << endl;
                if(O->kind_ == 5);
                else return true;
            }
            A = B;
        }
        time_end = GetTimeNs(startTime);
        if(timing) cout << "\tCheckingEdges: "
                        << (time_end - time_start)/MICROSECOND << " ms" << endl;
    } else if(O->kind_ == 6 || O->kind_ == 7) {
        // Must check all edges of obstacle path that
        // overlap with robot edge in time

        Eigen::VectorXd early_point, late_point;

        // Make life easier by always checking past to future
        if(start_point(2) < end_point(2)) {
            early_point = start_point;
            late_point = end_point;
        } else {
            early_point = end_point;
            late_point = start_point;
        }

        int first_obs_ind = max(FindIndexBeforeTime(O->path_,early_point(2)),
                                0);
        int last_obs_ind = min(FindIndexBeforeTime(O->path_,late_point(2)),
                               (int)O->path_.rows()-1);

        if(last_obs_ind <= first_obs_ind) return false; // object does not
                                                        // overlap in time

        int i_start, i_end;
        double x_1, y_1, t_1, x_2, y_2, t_2, m_x1, m_y1,
                m_x2, m_y2, t_c, r_x, r_y, o_x, o_y;
        for(i_start = first_obs_ind; i_start < last_obs_ind; i_start++) {
            i_end = i_start + 1;

            x_1 = early_point(0);   // robot start x
            y_1 = early_point(1);   // robot start y
            t_1 = early_point(2);   // robot start time

            x_2 = O->path_.row(i_start)(0) + O->position_(0); // obs start x
            y_2 = O->path_.row(i_start)(1) + O->position_(1); // obs start y
            t_2 = O->path_.row(i_start)(2);                 // obs start time

            // Calculate intermediate quantities (parametric slopes)
            m_x1 = (late_point(0) - x_1)/(late_point(2)-t_1);
            m_y1 = (late_point(1) - y_1)/(late_point(2)-t_1);
            m_x2 = (O->path_.row(i_end)(0) + O->position_(0) - x_2)
                    / (O->path_.row(i_end)(2)-t_2);
            m_y2 = (O->path_.row(i_end)(1) + O->position_(1) - y_2)
                    / (O->path_.row(i_end)(2)-t_2);

            // Solve for time of closest pass of centers
            t_c = (pow(m_x1,2)*t_1 + m_x2*(m_x2*t_2 + x_1 - x_2)
                   - m_x1*(m_x2*(t_1 + t_2) + x_1 - x_2)
                   + (m_y1 - m_y2) * (m_y1*t_1 - m_y2*t_2 - y_1 + y_2))
                  / (pow(m_x1-m_x2,2) + pow(m_y1-m_y2,2));

            // Now bound t_c by the allowable times of robot and obstacle
            if(t_c < max(t_1,t_2)) t_c = max(t_1,t_2);
            else if( t_c > min(late_point(2),O->path_.row(i_end)(2))) {
                t_c = min(late_point(2),O->path_.row(i_end)(2));
            }

            // Finally see if the distance between the robot and the
            // obstacle at t_c is close enough to cause a conflict
            r_x = m_x1*(t_c - t_1) + x_1;   // robot x at t_c
            r_y = m_y1*(t_c - t_1) + y_1;   // robot y at t_c
            o_x = m_x2*(t_c - t_2) + x_2;   // obstacle x at t_c
            o_y = m_y2*(t_c - t_2) + y_2;   // obstacle y at t_c

            if(pow(r_x - o_x,2) + pow(r_y - o_y,2)
                    < pow(O->radius_ + radius,2)) {
                    // Then there is a collision
                    return true;
            }
        }
    }
    return false;
}

bool ExplicitEdgeCheck(shared_ptr<ConfigSpace> &S,
                       shared_ptr<Edge> &edge)
{
    // If ignoring obstacles
    if( S->in_warmup_time_ ) return false;

    shared_ptr<ListNode> obstacle_list_node;
    int length;
    {
        lock_guard<mutex> lock(S->cspace_mutex_);
        obstacle_list_node = S->obstacles_->front_;
        length = S->obstacles_->length_;
    }
    for( int i = 0; i < length; i++ ) {
        if( edge->ExplicitEdgeCheck(obstacle_list_node->obstacle_) ) {
            return true;
        }
        obstacle_list_node = obstacle_list_node->child_; // iterate
    }
    return false;
}

bool QuickCheck2D(shared_ptr<Obstacle> &O, Eigen::VectorXd point,
                  shared_ptr<ConfigSpace> &C)
{
    if(!O->obstacle_used_ || O->life_span_ <= 0) return false;
    if((1 <= O->kind_ && O->kind_ <= 5)
            && C->distanceFunction(O->position_,point) > O->radius_)
        return false;

    if(O->kind_ == 1) return true;
    else if(O->kind_ == 2 || O->kind_ == 4) return false;
    else if(O->kind_ == 3) {
        if(PointInPolygon(point,O->polygon_)) return true;
    } else if(O->kind_ == 5) return false;
    else if(O->kind_ == 6 || O->kind_ == 7) {
        // First transform position_ and polygon to appropriate position_
        // at the time of the point based on the obstacles path through time

        if(point.size() < 3)
            cout << "error: point does not contain time" << endl;

        Eigen::Vector2d offset = FindTransformObjToTimeOfPoint(O,point);

        // Do a quick check based on lower bound of distance to obstacle
        if(C->distanceFunction(O->position_ + offset,point) > O->radius_)
            return false;

        // Transform polygon and do a normal check vs it
        Eigen::ArrayXd polygon;
        polygon = O->original_polygon_.col(0);
        O->polygon_.col(0) = polygon + offset(0);
        polygon = O->original_polygon_.col(1);
        O->polygon_.col(1) = polygon + offset(1);
        if(PointInPolygon(point,O->polygon_)) return true;
    }
    return false;
}

bool QuickCheck(shared_ptr<ConfigSpace> &C, Eigen::VectorXd point)
{
    shared_ptr<ListNode> obstacle_list_node;
    int length;
    {
        lock_guard<mutex> lock(C->cspace_mutex_);
        obstacle_list_node = C->obstacles_->front_;
        length = C->obstacles_->length_;
    }
    for(int i = 0; i < length; i++) {
        if(QuickCheck2D(obstacle_list_node->obstacle_,point,C)) return true;
        obstacle_list_node = obstacle_list_node->child_;
    }
    return false;
}

bool ExplicitPointCheck2D(shared_ptr<ConfigSpace> &C,
                          shared_ptr<Obstacle> &O,
                          Eigen::VectorXd point,
                          double radius)
{
//    cout << "ExplicitPointCheck2D" << endl;
    double this_distance = INF;
    double min_distance = C->collision_distance_;

    if(!O->obstacle_used_ || O->life_span_ <= 0) return false;

    if( 1 <= O->kind_ && O->kind_ <= 5 ) {
        // Do a quick check to see if any points on the obstacle
        // might be closer to point than minDist based on the ball
        // around the obstacle

        O->position_(2) = point(2);
//        cout << "Point:\n" << point << endl;
//        if(point(0) > 3
//           && point(0) < 10
//           && point(1) > 3
//           && point(1) < 10) {
//            cout << "Object:\n" << O->position_ << endl;
//            cout << "dist(O,point): " << C->distanceFunction(O->position_,point) << endl;
//            cout << "collision dist: " << C->distanceFunction(O->position_,point) - radius - O->radius_ << endl;
//            cout << "minimum: " << min_distance << endl;
//            cout << "dist > min?: " << ((C->distanceFunction(O->position_,point) - radius - O->radius_) > min_distance) << endl;
//        }

        // Calculate distance from robot boundary to obstacle center
        this_distance = C->distanceFunction(O->position_,point) - radius;
        if(this_distance - O->radius_ > min_distance) return false;
    }

    if(O->kind_ == 1) {
        // Ball is actual obstacle, os we have a new minimum
        this_distance = this_distance - O->radius_;
        if(this_distance < 0.0) return true;
    } else if(O->kind_ == 2) return false;
    else if(O->kind_ == 3) {
//        cout << "PointInPolygon" << endl;
        if(PointInPolygon(point.head(2),O->polygon_)) return true;
//        cout << "the above should be true but it's false" << endl;
        this_distance = sqrt(DistToPolygonSqrd(point,O->polygon_)) - radius;
        if(this_distance < 0.0) return true;
    } else if(O->kind_ == 5) return false;
    else if(O->kind_ == 6 || O->kind_ == 7) {
        // First transform position_ and polygon to appropriate ponsition
        // at the time of the point, based on the obstacle's path through time
        Eigen::Vector2d offset = FindTransformObjToTimeOfPoint(O,point);

        // Do a quick check to see if any points on the obstacle might
        // be closer to the point than min_dist based on the ball around
        // the obstacle

        // Calculate distance from robot boundary to obstacle center
        Eigen::VectorXd now_pos = O->position_;
        now_pos(0) = now_pos(0) + offset(0);
        now_pos(1) = now_pos(1) + offset(1);
        double this_distance = C->distanceFunction(now_pos, point) - radius;
        if(this_distance - O->radius_ > min_distance) return false;

        // Transform polygon and then do the rest of a normal check
        Eigen::ArrayXd polygon = O->original_polygon_.col(0);
        O->polygon_.col(0) = polygon + offset(0);
        polygon = O->original_polygon_.col(1);
        O->polygon_.col(1) = polygon + offset(1);

        if(PointInPolygon(point,O->polygon_)) return true;
        this_distance = sqrt(DistToPolygonSqrd(point,O->polygon_)) - radius;
        if(this_distance < 0.0) return true;
    }

    return false;
}

bool ExplicitPointCheck(shared_ptr<Queue>& Q, Eigen::VectorXd point)
{
    // If ignoring obstacles
    if(Q->cspace->in_warmup_time_) return false;

    // First do quick check to see if the point can be determined in collision
    // with minimal work (quick check is not implicit check)
    if(QuickCheck(Q->cspace,point)) return true;

    // Point is not inside any obstacles but still may be in collision
    // because of the robot radius
    shared_ptr<ListNode> obstacle_list_node;
    int length;
    {
        lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
        obstacle_list_node = Q->cspace->obstacles_->front_;
        length = Q->cspace->obstacles_->length_;
    }
    for(int i = 0; i < length; i++) {
        if(ExplicitPointCheck2D(Q->cspace,obstacle_list_node->obstacle_,
                                point, Q->cspace->robot_radius_)) return true;
        obstacle_list_node = obstacle_list_node->child_;
    }
    return false;
}

bool ExplicitNodeCheck(shared_ptr<Queue>& Q, shared_ptr<KDTreeNode> node)
{
    return ExplicitPointCheck(Q,node->position_);
}

void RandomSampleObs(shared_ptr<ConfigSpace> &S,
                     shared_ptr<KDTree> Tree,
                     shared_ptr<Obstacle> &O)
{
    // Calculation of the number of samples to use could be
    // more accurate by also multiplying by the ratio of free vs
    // total random samples we have observed up to this point over
    // the total space

    // Sample from a hypercube that contains the entire obstacle,
    // and reject if the point is not within the obstacle

    // Calculate the volume of the bounding hypercube
    double o_hyper_volume_bound = 0;
    if(!S->space_has_time_ && !S->space_has_theta_) {
        // Euclidean space, no time
        o_hyper_volume_bound = pow(2.0*O->radius_,S->num_dimensions_);
        if(S->hyper_volume_ == 0.0) S->hyper_volume_ = S->width_.prod();
    } else if(!S->space_has_time_ && S->space_has_theta_) {
        // Dubin's car, no time
        o_hyper_volume_bound = pow(2.0*O->radius_,2); // * S->width_(2)
        if(S->hyper_volume_ == 0.0) S->hyper_volume_ = S->width_.head(2).prod();
    } else cout << "Not coded yet" << endl;

    double num_obs_samples = Tree->tree_size_
            * o_hyper_volume_bound/S->hyper_volume_ + 1.0;
    Eigen::VectorXd new_point;
    Eigen::Array2d temp, temp1;
    for(int i = 0; i < num_obs_samples; i++) {
        new_point = randPointDefault(S);
        temp = O->position_;
        temp = temp - O->radius_;
        temp1 = new_point.head(2);
        temp1 = temp1*2.0*O->radius_;
        new_point.head(2) = temp + temp1;
        if(QuickCheck2D(O,new_point,S)) {
            shared_ptr<KDTreeNode> point = make_shared<KDTreeNode>(new_point);
            S->sample_stack_->JListPush(point);
        }
    }
}

/////////////////////// RRT Functions ///////////////////////

bool Extend(shared_ptr<KDTree> &Tree,
            shared_ptr<Queue> &Q,
            shared_ptr<KDTreeNode> &new_node,
            shared_ptr<KDTreeNode> &closest_node,
            double delta,
            double hyper_ball_rad,
            shared_ptr<KDTreeNode> &move_goal)
{
    //cout << "Extend" << endl;
    /// For timing
    chrono::time_point<chrono::high_resolution_clock> startTime
            = chrono::high_resolution_clock::now();
    double time_start, time_end;

    // Find all nodes within the (shrinking) hyper ball of
    // (saturated) new_node
    // true argument for using KDTreeNodes as elements
    shared_ptr<JList> node_list = make_shared<JList>(true);
    time_start = GetTimeNs(startTime);
    Tree->KDFindWithinRange(node_list, hyper_ball_rad, new_node->position_);
    time_end = GetTimeNs(startTime);
    if(timing) cout << "\tkdFindWithinRange: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

    // Try to find and link to best parent. This also saves
    // the edges from new_node to the neighbors in the field
    // "temp_edge_" of the neighbors. This saves time in the
    // case that trajectory calculation is complicated.
    time_start = GetTimeNs(startTime);
    findBestParent( Q->cspace, Tree, new_node, node_list, closest_node, true );
    time_end = GetTimeNs(startTime);
    if(timing) cout << "\tfindBestParent: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

    // If no parent was fonud then ignore this node
    if( !new_node->rrt_parent_used_ ) {
        Tree->EmptyRangeList(node_list); // clean up
        return false;
    }

    // Insert the new node into the KDTree
    time_start = GetTimeNs(startTime);
    Tree->KDInsert(new_node);
    time_end = GetTimeNs(startTime);
    if(timing) cout << "\tkdInsert: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

    // Second pass, if there was a parent, then link with neighbors
    // and rewire neighbors that would do better to use new_node as
    // their parent. Note that the edges -from- new_node -to- its
    // neighbors have been stored in "temp_edge_" field of the neighbors
    shared_ptr<JListNode> list_item = node_list->front_;
    shared_ptr<KDTreeNode> near_node;
    shared_ptr<Edge> this_edge;
    double old_LMC;

    for( int i = 0; i < node_list->length_; i++ ) {
        near_node = list_item->node_;

        // If edge from new_node to nearNode was valid
        if(list_item->key_ != -1.0) {
            // Add to initial out neighbor list of new_node
            // (allows info propogation from new_node to nearNode always)
            makeInitialOutNeighborOf( near_node,new_node,near_node->temp_edge_ );

            // Add to current neighbor list of new_node
            // (allows info propogation from new_node to nearNode and
            // vice versa, but only while they are in the D-ball)
            time_start = GetTimeNs(startTime);
            makeNeighborOf( near_node, new_node, near_node->temp_edge_ );
            time_end = GetTimeNs(startTime);
            if(timing) cout << "\tmakeNeighborOf: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

        }

        // In the general case, the trajectories along edges are not simply
        // the reverse of each other, therefore we need to calculate
        // and check the trajectory along the edge from nearNode to new_node
        this_edge = Edge::NewEdge( Q->cspace, Tree, near_node, new_node );
        time_start = GetTimeNs(startTime);
        this_edge->CalculateTrajectory();
        time_end = GetTimeNs(startTime);
        if(timing) cout << "\tcalculateTrajectory: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

        time_start = GetTimeNs(startTime);
        bool edge_is_safe = !ExplicitEdgeCheck(Q->cspace,this_edge);
        time_end = GetTimeNs(startTime);
        if(timing) cout << "\tExplicitEdgeCheck: " << (time_end - time_start)/MICROSECOND << " ms" << endl;
        if( this_edge->ValidMove()
                && edge_is_safe ) {
            // Add to initial in neighbor list of newnode
            // (allows information propogation from new_node to
            // nearNode always)
            time_start = GetTimeNs(startTime);
            makeInitialInNeighborOf( new_node, near_node, this_edge );
            time_end = GetTimeNs(startTime);
            if(timing) cout << "\tmakeInitialNeighborOf: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

            // Add to current neighbor list of new_node
            // (allows info propogation from new_node to nearNode and
            // vice versa, but only while they are in D-ball)
            makeNeighborOf( new_node, near_node, this_edge );
        } else {
            // Edge cannot be created
            list_item = list_item->child_; // iterate through list
            continue;
        }

        // Rewire neighbors that would do better to use this node
        // as their parent unless they are not in the relevant
        // portion of the space vs. moveGoal
        time_start = GetTimeNs(startTime);
        if( near_node->rrt_LMC_ > new_node->rrt_LMC_ + this_edge->dist_
                && new_node->rrt_parent_edge_->end_node_ != near_node
                && new_node->rrt_LMC_ + this_edge->dist_ < move_goal->rrt_LMC_ ) {
            // Make this node the parent of the neighbor node
            makeParentOf( new_node, near_node, this_edge, Tree->root );

            // Recalculate tree cost of neighbor
            old_LMC = near_node->rrt_LMC_;
            near_node->rrt_LMC_ = new_node->rrt_LMC_ + this_edge->dist_;

            // Insert neighbor into priority queue if cost
            // reduction is great enough
            if( old_LMC - near_node->rrt_LMC_ > Q->change_thresh
                    && near_node != Tree->root ) {
                verifyInQueue( Q, near_node );
            }
        }
        time_end = GetTimeNs(startTime);
        if(timing) cout << "\tRewiringNeighbors: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

        list_item = list_item->child_; // iterate through list
    }

    Tree->EmptyRangeList(node_list); // clean up

    // Insert the node into the priority queue
    Q->priority_queue->AddToHeap(new_node);

    return true;
}


/////////////////////// RRT* Functions ///////////////////////

void findBestParent(shared_ptr<ConfigSpace> &S,
                    shared_ptr<KDTree> &Tree,
                    shared_ptr<KDTreeNode>& newNode,
                    shared_ptr<JList> &nodeList,
                    shared_ptr<KDTreeNode>& closestNode,
                    bool saveAllEdges)
{
   // cout << "findBestParent" << endl;
    /// For function duration testing
    double time_start, time_end; // in nanoseconds
    chrono::time_point<chrono::high_resolution_clock> startTime
            = chrono::high_resolution_clock::now();

    // If the list is empty
    if(nodeList->length_ == 0) {
        if(S->goal_node_ != newNode) nodeList->JListPush(closestNode);
    }

    // Update LMC value based on nodes in the list
    newNode->rrt_LMC_ = INF;
    newNode->rrt_tree_cost_ = INF;
    newNode->rrt_parent_used_ = false;

    // Find best parent (or if one even exists)
    shared_ptr<JListNode> listItem = nodeList->front_;
    shared_ptr<KDTreeNode> nearNode;
    shared_ptr<Edge> thisEdge;
    while(listItem->child_ != listItem) {
        nearNode = listItem->node_;

        // First calculate the shortest trajectory (and its distance)
        // that gets from newNode to nearNode while obeying the
        // constraints of the state space and the dynamics
        // of the robot
        thisEdge = Edge::NewEdge(S, Tree, newNode, nearNode);
        time_start = GetTimeNs(startTime);
        thisEdge->CalculateTrajectory();
        time_end = GetTimeNs(startTime);
        if(timing) cout << "\t\tcalculateTrajectory: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

        if( saveAllEdges ) nearNode->temp_edge_ = thisEdge;

        // Check for validity vs edge collisions vs obstacles and
        // vs the time-dynamics of the robot and space
        time_start = GetTimeNs(startTime);
        bool edge_is_safe = !ExplicitEdgeCheck(S,thisEdge);
        time_end = GetTimeNs(startTime);
        if(timing) cout << "\t\tExplicitEdgeCheck: " << (time_end - time_start)/MICROSECOND << " ms" << endl;
        if(!edge_is_safe || !thisEdge->ValidMove()) {
            if(saveAllEdges) nearNode->temp_edge_->dist_ = INF;
            listItem = listItem->child_; // iterate through list
            continue;
        }

        // Check if need to update rrtParent and rrt_parent_edge_
        if(newNode->rrt_LMC_ > nearNode->rrt_LMC_ + thisEdge->dist_) {
            // Found a potential better parent
            newNode->rrt_LMC_ = nearNode->rrt_LMC_ + thisEdge->dist_;
            /// This also takes care of some code in Extend I believe
            time_start = GetTimeNs(startTime);
            makeParentOf(nearNode,newNode,thisEdge,Tree->root);
            time_end = GetTimeNs(startTime);
            if(timing) cout << "\t\tmakeParentOf: " << (time_end - time_start)/MICROSECOND << " ms" << endl;
        }
        listItem = listItem->child_; // iterate thorugh list
    }
}


/////////////////////// RRT# Functions ///////////////////////

void resetNeighborIterator( shared_ptr<RrtNodeNeighborIterator> &It )
{It->list_flag = 0;}

void makeNeighborOf(shared_ptr<KDTreeNode> &newNeighbor,
                    shared_ptr<KDTreeNode> &node,
                    shared_ptr<Edge> &edge)
{
    node->rrt_neighbors_out_->JListPush( edge );
    edge->list_item_in_start_node_ = node->rrt_neighbors_out_->front_;

    newNeighbor->rrt_neighbors_in_->JListPush( edge );
    edge->list_item_in_end_node_ = newNeighbor->rrt_neighbors_in_->front_;
}

void makeInitialOutNeighborOf(shared_ptr<KDTreeNode> &newNeighbor,
                              shared_ptr<KDTreeNode> &node,
                              shared_ptr<Edge> &edge)
{ node->initial_neighbor_list_out_->JListPush( edge ); }

void makeInitialInNeighborOf(shared_ptr<KDTreeNode> &newNeighbor,
                             shared_ptr<KDTreeNode> &node,
                             shared_ptr<Edge> &edge)
{ node->initial_neighbor_list_in_->JListPush( edge ); }

void updateQueue( shared_ptr<Queue> &Q,
                  shared_ptr<KDTreeNode> &newNode,
                  shared_ptr<KDTreeNode> &root,
                  double hyperBallRad )
{
    recalculateLMC( Q, newNode, root, hyperBallRad ); // internally ignores root
     if( Q->priority_queue->markedQ( newNode ) ) {
         Q->priority_queue->UpdateHeap( newNode );
         Q->priority_queue->RemoveFromHeap( newNode );
     }
     if( newNode->rrt_tree_cost_ != newNode->rrt_LMC_ ) {
         Q->priority_queue->AddToHeap( newNode );
     }
}

void reduceInconsistency(shared_ptr<Queue> &Q,
                         shared_ptr<KDTreeNode> &goalNode,
                         double robotRad,
                         shared_ptr<KDTreeNode> &root,
                         double hyperBallRad)
{
    /*if( Q->type == "RRT#" ) {

        shared_ptr<KDTreeNode> thisNode, neighborNode;
        shared_ptr<JListNode> listItem;
        Q->priority_queue->TopHeap(thisNode);
        while( Q->priority_queue->index_of_last_>0 && (Q->priority_queue->lessQ(thisNode,goalNode)
                                         || goalNode->rrt_LMC_ == INF
                                         || goalNode->rrt_tree_cost_ == INF
                                         || Q->priority_queue->markedQ(goalNode)) ) {
            Q->priority_queue->PopHeap(thisNode);
            thisNode->rrt_tree_cost_ = thisNode->rrt_LMC_;

            listItem = thisNode->rrt_neighbors_in_->front_;
            for( int i = 0; i < thisNode->rrt_neighbors_in_->length_; i++ ) {
                neighborNode = listItem->edge->start_node_; // dereference?
                updateQueue( Q, neighborNode, root, hyperBallRad );
                listItem = listItem->child_; // iterate through list
            }
            Q->priority_queue->TopHeap(thisNode);
        }
    } else {*/ // Q->type == "RRTx";
        shared_ptr<KDTreeNode> thisNode;
        Q->priority_queue->TopHeap(thisNode);
        while( Q->priority_queue->index_of_last_ > 0
               && (Q->priority_queue->lessThan(thisNode, goalNode)
                   || goalNode->rrt_LMC_ == INF
                   || goalNode->rrt_tree_cost_ == INF
                   || Q->priority_queue->marked(goalNode) ) ) {
            Q->priority_queue->PopHeap(thisNode);

            // Update neighbors of thisNode if it has
            // changed more than change thresh
            if(thisNode->rrt_tree_cost_ - thisNode->rrt_LMC_ > Q->change_thresh) {
                recalculateLMC( Q, thisNode, root, hyperBallRad );
                rewire( Q, thisNode, root, hyperBallRad, Q->change_thresh );
            }
            thisNode->rrt_tree_cost_ = thisNode->rrt_LMC_;
            Q->priority_queue->TopHeap(thisNode);
        }
//    }
}


/////////////////////// RRTx Functions ///////////////////////

void markOS( shared_ptr<KDTreeNode> &node )
{
    node->in_OS_queue_ = true;
}

void unmarkOS( shared_ptr<KDTreeNode> &node )
{
    node->in_OS_queue_ = false;
}

bool markedOS( shared_ptr<KDTreeNode> node )
{
    return node->in_OS_queue_;
}

bool verifyInQueue(shared_ptr<Queue> &Q,
                   shared_ptr<KDTreeNode> &node)
{
    if( Q->priority_queue->markedQ(node) ) {
       return Q->priority_queue->UpdateHeap(node);
    } else {
       return Q->priority_queue->AddToHeap(node);
    }
}

bool verifyInOSQueue(shared_ptr<Queue> &Q,
                     shared_ptr<KDTreeNode> &node)
{
    if( Q->priority_queue->markedQ(node) ) {
        Q->priority_queue->UpdateHeap(node);
        Q->priority_queue->RemoveFromHeap(node);
    }
    if( !markedOS(node) ) {
        markOS(node);
        Q->obs_successors->JListPush(node);
    }
    return true;
}

void cullCurrentNeighbors( shared_ptr<KDTreeNode> &node,
                           double hyperBallRad )
{
    // Remove outgoing edges from node that are now too long
    shared_ptr<JListNode> listItem = node->rrt_neighbors_out_->front_;
    shared_ptr<JListNode> nextItem;
    shared_ptr<Edge> neighborEdge;
    shared_ptr<KDTreeNode> neighborNode;
    while( listItem != listItem->child_ ) {
        nextItem = listItem->child_; // since we may remove listItem from list
        if( listItem->edge_->dist_ > hyperBallRad ) {
            neighborEdge = listItem->edge_;
            neighborNode = neighborEdge->end_node_;
            node->rrt_neighbors_out_->JListRemove(
                        neighborEdge->list_item_in_start_node_);
            neighborNode->rrt_neighbors_in_->JListRemove(
                        neighborEdge->list_item_in_end_node_);
        }
        listItem = nextItem;
    }
}

shared_ptr<JListNode> nextOutNeighbor(
        shared_ptr<RrtNodeNeighborIterator> &It)
{
    if( It->list_flag == 0 ) {
        It->current_item = It->this_node->initial_neighbor_list_out_->front_;
        It->list_flag = 1;
    } else {
        It->current_item = It->current_item->child_;
    }
    while( It->current_item == It->current_item->child_ ) {
        // Go to the next place tha neighbors are stored
        if( It->list_flag == 1 ) {
            It->current_item = It->this_node->rrt_neighbors_out_->front_;
        } else {
            // Done with all neighbors
            // Returns empty JListNode with empty KDTreeNode
            // so can check for this by checking return_value->key == -1
            return make_shared<JListNode>();
        }
        It->list_flag += 1;
    }
    return It->current_item;
}

shared_ptr<JListNode> nextInNeighbor(
        shared_ptr<RrtNodeNeighborIterator> &It)
{
    if( It->list_flag == 0 ) {
        It->current_item = It->this_node->initial_neighbor_list_in_->front_;
        It->list_flag = 1;
    } else {
        It->current_item = It->current_item->child_;
    }
    while( It->current_item == It->current_item->child_ ) {
        // Go to the next place that neighbors are stored
        if( It->list_flag == 1 ) {
            It->current_item = It->this_node->rrt_neighbors_in_->front_;
        } else {
            // Done with all neighbors
            // Returns empty JListNode with empty KDTreeNode
            // so can check for this by checking return_value->key == -1
            return make_shared<JListNode>();
        }
        It->list_flag += 1;
    }
    return It->current_item;
}

void makeParentOf( shared_ptr<KDTreeNode> &newParent,
                   shared_ptr<KDTreeNode> &node,
                   shared_ptr<Edge> &edge,
                   shared_ptr<KDTreeNode> &root )
{
    // Remove the node from its old parent's successor list
    if( node->rrt_parent_used_ ) {
        node->rrt_parent_edge_->end_node_->successor_list_->JListRemove(
                    node->successor_list_item_in_parent_ );
    }

    // Make newParent the parent of node
    node->rrt_parent_edge_ = edge;
    node->rrt_parent_used_ = true;

    // Place a (non-trajectory) reverse edge into newParent's
    // successor list and save a pointer to its position_ in
    // that list. This edge is used to help keep track of
    // successors and not for movement.
    shared_ptr<Edge> backEdge
            = Edge::Edge::NewEdge(edge->cspace_, edge->tree_, newParent, node );
    backEdge->dist_ = INF;
    newParent->successor_list_->JListPush( backEdge, INF );
    node->successor_list_item_in_parent_ = newParent->successor_list_->front_;
}

bool recalculateLMC(shared_ptr<Queue> &Q,
                    shared_ptr<KDTreeNode> &node,
                    shared_ptr<KDTreeNode> &root,
                    double hyperBallRad)
{
    if( node == root ) {
        return false;
    }

    bool newParentFound = false;
    double neighborDist;
    shared_ptr<KDTreeNode> rrtParent, neighborNode;
    shared_ptr<Edge> parentEdge, neighborEdge;
    shared_ptr<JListNode> listItem, nextItem;

    // Remove outdated nodes from current neighbors list
    cullCurrentNeighbors( node, hyperBallRad );

    // Get an iterator for this node's neighbors
    shared_ptr<RrtNodeNeighborIterator> thisNodeOutNeighbors
            = make_shared<RrtNodeNeighborIterator>(node);

    // Set the iterator to the first neighbor
    listItem = nextOutNeighbor( thisNodeOutNeighbors );

    while( listItem->key_ != -1.0 ) {
        neighborEdge = listItem->edge_;
        neighborNode = neighborEdge->end_node_;
        neighborDist = neighborEdge->dist_;
        nextItem = listItem->child_;

        if( markedOS(neighborNode) ) {
            // neighborNode is already in OS queue (orphaned) or unwired
            listItem = nextOutNeighbor( thisNodeOutNeighbors );
            continue;
        }

        if( node->rrt_LMC_ > neighborNode->rrt_LMC_ + neighborDist
                && (!neighborNode->rrt_parent_used_
                    || neighborNode->rrt_parent_edge_->end_node_ != node)
                && neighborEdge->ValidMove() ) {
            // Found a better parent
            node->rrt_LMC_ = neighborNode->rrt_LMC_ + neighborDist;
            rrtParent = neighborNode;
            parentEdge = listItem->edge_;
            newParentFound = true;
        }

        listItem = nextOutNeighbor( thisNodeOutNeighbors );
    }

    if( newParentFound ) { // this node found a viable parent
        makeParentOf( rrtParent, node, parentEdge, root );
    }
    return true;
}

bool rewire( shared_ptr<Queue> &Q,
             shared_ptr<KDTreeNode> &node,
             shared_ptr<KDTreeNode> &root,
             double hyperBallRad, double changeThresh )
{
    // Only explicitly propogate changes if they are large enough
    double deltaCost = node->rrt_tree_cost_ - node->rrt_LMC_;
    if( deltaCost <= changeThresh ) {
        // Note that using simply "<" causes problems
        // Above note may be outdated
        // node.rrt_tree_cost_ = node.rrt_LMC_!!! Now happens after return
        cout << "not rewiring" << endl;
        return false;
    }

    // Remove outdated nodes from the current neighbors list
    cullCurrentNeighbors( node, hyperBallRad );

    // Get an iterator for this node's neighbors and iterate through list
    shared_ptr<RrtNodeNeighborIterator> thisNodeInNeighbors
            = make_shared<RrtNodeNeighborIterator>(node);
    shared_ptr<JListNode> listItem
            = nextInNeighbor( thisNodeInNeighbors );
    shared_ptr<KDTreeNode> neighborNode;
    shared_ptr<Edge> neighborEdge;

    while( listItem->key_ != -1.0 ) {
        neighborEdge = listItem->edge_;
        neighborNode = neighborEdge->start_node_;

        // Ignore this node's parent and also nodes that cannot
        // reach node due to dynamics of robot or space
        // Not sure about second parent since neighbors are not
        // initially created that cannot reach this node
        if( (node->rrt_parent_used_
             && node->rrt_parent_edge_->end_node_ == neighborNode)
                || !neighborEdge->ValidMove() ) {
            listItem = nextInNeighbor( thisNodeInNeighbors );
            continue;
        }

        neighborEdge = listItem->edge_;

        if( neighborNode->rrt_LMC_  > node->rrt_LMC_ + neighborEdge->dist_
                && (!neighborNode->rrt_parent_used_
                    || neighborNode->rrt_parent_edge_->end_node_ != node )
                && neighborEdge->ValidMove() ) {
            // neighborNode should use node as its parent (it might already)
            neighborNode->rrt_LMC_ = node->rrt_LMC_ + neighborEdge->dist_;
            makeParentOf( node, neighborNode, neighborEdge, root );

            // If the reduction is great enough, then propogate
            // through the neighbor
            if( neighborNode->rrt_tree_cost_ - neighborNode->rrt_LMC_
                    > changeThresh ) {
                verifyInQueue( Q, neighborNode );
            }
        }

        listItem = nextInNeighbor( thisNodeInNeighbors );
    }
    return true;
}

bool propogateDescendants(shared_ptr<Queue> &Q,
                          shared_ptr<KDTree> Tree,
                          shared_ptr<RobotData> &R)
{
    if( Q->obs_successors->length_ <= 0 ) {
        return false;
    }

    // First pass, accumulate all such nodes in a single list, and mark
    // them as belonging in that list, we'll just use the OS stack we've
    // been using adding nodes to the front while moving from back to front
    shared_ptr<JListNode> OS_list_item = Q->obs_successors->back_;
    shared_ptr<KDTreeNode> thisNode, successorNode;
    shared_ptr<JListNode> SuccessorList_item;
    while( OS_list_item != OS_list_item->parent_ ) {
        thisNode = OS_list_item->node_;

        // Add all of this node's successors to OS stack
        SuccessorList_item = thisNode->successor_list_->front_;
        while( SuccessorList_item != SuccessorList_item->child_ ) {
            successorNode = SuccessorList_item->edge_->end_node_;
            verifyInOSQueue( Q, successorNode ); // pushes to front_ of OS
            SuccessorList_item = SuccessorList_item->child_;
        }

        OS_list_item = OS_list_item->parent_;
    }

    // Second pass, put all -out neighbors- of the nodes in OS
    // (not including nodes in OS) into Q and tell them to force rewire.
    // Not going back to front makes Q adjustments slightly faster,
    // since nodes near the front tend to have higher costs
    OS_list_item = Q->obs_successors->back_;
    shared_ptr<JListNode> listItem;
    shared_ptr<KDTreeNode> neighborNode;
    while( OS_list_item != OS_list_item->parent_ ) {
        thisNode = OS_list_item->node_;

        // Get an iterator for this node's neighbors
        shared_ptr<RrtNodeNeighborIterator> thisNodeOutNeighbors
                = make_shared<RrtNodeNeighborIterator>(thisNode);

        // Now iterate through list (add all neighbors to the Q,
        // except those in OS
        listItem = nextOutNeighbor( thisNodeOutNeighbors );
        while( listItem->key_ != -1.0 ) {
            neighborNode = listItem->edge_->end_node_;

            if( markedOS(neighborNode) ) {
                // neighborNode already in OS queue (orphaned) or unwired
                listItem = nextOutNeighbor( thisNodeOutNeighbors );
                continue;
            }

            // Otherwise, make sure that neighborNode is in normal queue
            neighborNode->rrt_tree_cost_ = INF; // node will be inserted with LMC
                                             // key and then guarenteed to
                                             // propogate cost forward since
                                     // useful nodes have rrt_LMC_ < rrt_tree_cost_
            verifyInQueue( Q, neighborNode );

            listItem = nextOutNeighbor( thisNodeOutNeighbors );
        }

        // Add parent to the Q, unless it is in OS
        if( thisNode->rrt_parent_used_
                && !markedOS((thisNode->rrt_parent_edge_->end_node_)) ) {
            thisNode->kd_parent_->rrt_tree_cost_ = INF; // rrtParent = kd_parent_???
            verifyInQueue( Q, thisNode->rrt_parent_edge_->end_node_ );
        }

        OS_list_item = OS_list_item->parent_;
    }

    // Third pass, remove all nodes from OS, unmark them, and
    // remove their connections to their parents. If one was the
    // robot's target then take appropriate measures
    while( Q->obs_successors->length_ > 0 ) {
        Q->obs_successors->JListPop(thisNode);
        unmarkOS(thisNode);

        if( thisNode == R->next_move_target ) {
            R->current_move_invalid = true;
        }

        if( thisNode->rrt_parent_used_ ) {
            // Remove thisNode from its parent's successor list
            thisNode->rrt_parent_edge_->end_node_->successor_list_->JListRemove(
                        thisNode->successor_list_item_in_parent_ );

            // thisNode now has no parent
            thisNode->rrt_parent_edge_
                    = Edge::NewEdge(Q->cspace,Tree,thisNode,thisNode);
            thisNode->rrt_parent_edge_->dist_ = INF;
            thisNode->rrt_parent_used_ = false;
        }

        thisNode->rrt_tree_cost_ = INF;
        thisNode->rrt_LMC_ = INF;
    }
    return true;
}

void addOtherTimesToRoot( shared_ptr<ConfigSpace> &S,
                          shared_ptr<KDTree> &Tree,
                          shared_ptr<KDTreeNode> &goal,
                          shared_ptr<KDTreeNode> &root,
                          string searchType )
{
    double insertStep = 2.0;

    double lastTimeToInsert = goal->position_(2)
            - Tree->distanceFunction(root->position_,goal->position_)
            /S->robot_velocity_;
    double firstTimeToInsert = S->start_(2) + insertStep;
    shared_ptr<KDTreeNode> previousNode = root;
    bool safeToGoal = true;
    Eigen::VectorXd newPose;
    shared_ptr<KDTreeNode> newNode;
    shared_ptr<Edge> thisEdge;
    for( double timeToInsert = firstTimeToInsert;
         timeToInsert < lastTimeToInsert;
         timeToInsert += insertStep ) {
        newPose = root->position_;
        newPose(2) = timeToInsert;

        newNode = make_shared<KDTreeNode>(newPose);

        // Edge from newNode to previousNode
        thisEdge = Edge::NewEdge( S, Tree, newNode, previousNode );
        thisEdge->CalculateHoverTrajectory();

        if( searchType == "RRT*" ) {
            // Make this node the parent of the neighbor node
            newNode->rrt_parent_edge_ = thisEdge;
            newNode->rrt_parent_used_ = true;
        } else if( searchType == "RRT#" ) {
            // Make this node the parent of the neighbor node
            newNode->rrt_parent_edge_ = thisEdge;
            newNode->rrt_parent_used_ = true;
            makeNeighborOf( newNode, previousNode, thisEdge );
        } else if( searchType == "RRTx" ) {
            makeParentOf( previousNode, newNode, thisEdge, root );
            makeInitialOutNeighborOf( previousNode, newNode, thisEdge );
            // Initial neighbor list edge
            makeInitialInNeighborOf( newNode, previousNode, thisEdge );
        }

        // Make sure this edge is safe
        if( ExplicitEdgeCheck( S, thisEdge ) ) {
            // Not safe
            thisEdge->dist_ = INF;
            safeToGoal = false;
            newNode->rrt_LMC_ = INF;
            newNode->rrt_tree_cost_ = INF;
        } else if( safeToGoal ) {
            // If the edge has safe path all the way to the "real" goal
            // then make the cost of reaching "real" goal 0 from newNode
            thisEdge->dist_ = 0.0;
            newNode->rrt_LMC_ = 0.0;
            newNode->rrt_tree_cost_ = 0.0;
        } else {
            thisEdge->dist_ = INF;
            newNode->rrt_LMC_ = INF;
            newNode->rrt_tree_cost_ = INF;
        }

        Tree->KDInsert(newNode);

        previousNode = newNode;
    }
}


// debug: gets goal node which should be too far away on first movement
void findNewTarget(shared_ptr<ConfigSpace> &S,
                   shared_ptr<KDTree> &Tree,
                   shared_ptr<RobotData> &R,
                   double hyperBallRad )
{
    Eigen::VectorXd robPose, nextPose;
    R->robot_edge_used = false;
    R->dist_along_robot_edge = 0.0;
    R->time_along_robot_edge = 0.0;
    nextPose = R->next_move_target->position_;
    {
        lock_guard<mutex> lock(R->robot_mutex);
        robPose = R->robot_pose;
    }

    // Move target has become invalid
    double searchBallRad
            = max(hyperBallRad, Tree->distanceFunction(robPose, nextPose));

    double maxSearchBallRad
            = Tree->distanceFunction(S->lower_bounds_, S->upper_bounds_);
    searchBallRad = min( searchBallRad, maxSearchBallRad );
    shared_ptr<JList> L = make_shared<JList>(true);
    Tree->KDFindWithinRange( L, searchBallRad, robPose );

    shared_ptr<KDTreeNode> dummyRobotNode
            = make_shared<KDTreeNode>(robPose);
    shared_ptr<Edge> edgeToBestNeighbor
            = Edge::NewEdge(S,Tree,dummyRobotNode,dummyRobotNode);

    double bestDistToNeighbor, bestDistToGoal;
    shared_ptr<KDTreeNode> bestNeighbor, neighborNode;

    while( true ) { // will break out when done
        // Searching for new target within radius searchBallRad
        bestDistToNeighbor = INF;
        bestDistToGoal = INF;
        bestNeighbor = make_shared<KDTreeNode>();

        shared_ptr<JListNode> ptr = L->front_;
        shared_ptr<Edge> thisEdge;
        double distToGoal;
        while( ptr != ptr->child_ ) {
            neighborNode = ptr->node_;

            thisEdge = Edge::NewEdge( S,Tree,dummyRobotNode, neighborNode );
            thisEdge->CalculateTrajectory();

            if( thisEdge->ValidMove()
                    && !ExplicitEdgeCheck(S,thisEdge) ) {
                // A safe point was found, see if it is the best so far
                distToGoal = neighborNode->rrt_LMC_ + thisEdge->dist_;
                if( distToGoal < bestDistToGoal
                        && thisEdge->ValidMove() ) {
                    // Found a new and better neighbor
                    bestDistToGoal = distToGoal;
                    bestDistToNeighbor = thisEdge->dist_;
                    bestNeighbor = neighborNode;
                    edgeToBestNeighbor = thisEdge;
                } else { /*error("ptooie");*/ }
            }

            ptr = ptr->child_;
        }
        // Done trying to find a target within ball radius of searchBallRad

        // If a valid neighbor was found, then use it
        if( bestDistToGoal != INF ) {
            R->next_move_target = bestNeighbor;
            R->distance_from_next_robot_pose_to_next_move_target = bestDistToNeighbor;
            R->current_move_invalid = false;
            // Found a valid move target

            R->robot_edge = edgeToBestNeighbor; /** this edge is empty **/
            R->robot_edge_used = true;

            if( S->space_has_time_ ) {
                R->time_along_robot_edge = 0.0;
                // note this is updated before robot moves
            } else {
                R->dist_along_robot_edge = 0.0;
                // note this is updated before robot moves
            }

            // Set moveGoal to be next_move_target
            // NOTE may want to actually insert a new node at the robot's
            // position_ and use that instead, since these "edges" created
            // between robot pose and R.next_move_target may be lengthy
            S->move_goal_->is_move_goal_ = false;
            S->move_goal_ = R->next_move_target;
            S->move_goal_->is_move_goal_ = true;
            break;
        }

        searchBallRad *= 2;
        if( searchBallRad > maxSearchBallRad ) {
            // Unable to find a valid move target so sample randomly
            shared_ptr<KDTreeNode> newNode = randNodeDefault(S);
            double thisDist = Tree->distanceFunction(newNode->position_,
                                                     robPose);
            Edge::Saturate(
                        newNode->position_,
                        robPose, S->saturation_delta_, thisDist);
            Tree->KDInsert(newNode);
        }
        Tree->KDFindMoreWithinRange( L, searchBallRad, robPose );

    }
    Tree->EmptyRangeList(L); // cleanup
}

void MoveRobot(shared_ptr<Queue> &Q,
               shared_ptr<KDTree> &Tree,
               shared_ptr<KDTreeNode> &root,
               double slice_time,
               double hyperBallRad,
               shared_ptr<RobotData> &R )
{
    cout << "DistanceSqrdPointToSegment calls: " << dist_sqrd_point_seg << endl;
    cout << "SegmentDistanceSqrd calls: " << seg_dist_sqrd << endl;
    // Start by updating the location of the robot based on how
    // it moved since the last update (as well as the total path that
    // it has followed)
    if( R->moving ) {
        {
            lock_guard<mutex> lock(R->robot_mutex);
            cout << "Moving "
                      << Tree->distanceFunction(R->robot_pose,R->next_robot_pose)
                      << " units" << endl;
            R->robot_pose = R->next_robot_pose;
        }

        //R.robot_move_path[R.num_robot_move_points+1:R.num_robot_move_points+R.num_local_move_points,:] = R.robot_local_path[1:R.num_local_move_points,:];
        for( int i = 0; i < R->num_local_move_points-1; i++ ) {
            R->robot_move_path.row(R->num_robot_move_points+i) = R->robot_local_path.row(i);
        }
        R->num_robot_move_points += R->num_local_move_points;

        {
            lock_guard<mutex> lock(R->robot_mutex);
            if( !Q->cspace->space_has_time_ ) {
                cout << "new robot pose(w/o time):\n"
                          << R->robot_pose << endl;
            } else {
                cout << "new robot pose(w/ time):\n"
                          << R->robot_pose << endl;
            }
        }
        Q->cspace->warmup_time_just_ended_ = false;
    } else {
        // Movement has just started, so remember that the robot is now moving
        R->moving = true;

        error("First pose:");
        {
            lock_guard<mutex> lock(R->robot_mutex);
            cout << R->robot_pose << endl;
        }

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


    // If the robot's current move target has been invalidate due to
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
        {
            lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
            Q->cspace->move_goal_->is_move_goal_ = false;
            Q->cspace->move_goal_ = R->next_move_target;
            Q->cspace->move_goal_->is_move_goal_ = true;
        }
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
        {
            lock_guard<mutex> lock(R->robot_mutex);
            R->robot_local_path.row(R->num_local_move_points-1) = R->robot_pose;
        }
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
        {
            lock_guard<mutex> lock(R->robot_mutex);
            R->robot_local_path.row(R->num_local_move_points) = R->robot_pose;
            targetTime = R->robot_pose(2) - slice_time;
        }
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
