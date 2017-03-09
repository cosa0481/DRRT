/* drrt.cpp
 * Corin Sandford
 * Fall 2016
 * Contains RRTX "main" function at bottom.
 */

#include <DRRT/drrt.h>

using namespace std;

bool timing = false;

///////////////////// Print Helpers ///////////////////////
void error(string s) { cout << s << endl; }
void error(int i) { cout << i << endl; }

double getTimeNs( chrono::time_point
                  <chrono::high_resolution_clock> start )
{
    return chrono::duration_cast<chrono::nanoseconds>(
                chrono::high_resolution_clock::now()-start).count();
}

double randDouble( double min, double max )
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
        if( !node->rrtParentUsed ) {
            pathLength = INF;
            break;
        }
        pathLength += node->rrtParentEdge->dist;
        thisNode = thisNode->rrtParentEdge->endNode;
    }

    return pathLength;
}


/////////////////////// C-Space Functions ///////////////////////

Eigen::VectorXd randPointDefault( shared_ptr<CSpace> S )
{
    double rand;
    double first;
    Eigen::VectorXd second(S->width.size());

    for( int i = 0; i < S->width.size(); i++ ) {
        rand = randDouble(0,S->d);
        first = rand * S->width(i);
        second(i) = S->lowerBounds(i) + first;
    }
    return second;
}

shared_ptr<KDTreeNode> randNodeDefault( shared_ptr<CSpace> S )
{
    Eigen::VectorXd point = randPointDefault( S );
    return make_shared<KDTreeNode>(point);
}

shared_ptr<KDTreeNode> randNodeOrGoal( shared_ptr<CSpace> S )
{
    double r = (double)rand()/(RAND_MAX);
    if( r > S->pGoal ) {
        return randNodeDefault( S );
    } else {
        return S->goalNode;
    }
}

shared_ptr<KDTreeNode> randNodeIts(shared_ptr<CSpace> S)
{
    if( S->itsUntilSample == 0 ) {
        S->itsUntilSample -= 1;
        return make_shared<KDTreeNode>(S->itsSamplePoint);
    }
    S->itsUntilSample -= 1;
    return randNodeOrGoal( S );
}

shared_ptr<KDTreeNode> randNodeTime(shared_ptr<CSpace> S)
{
    if( S->waitTime != INF && S->timeElapsed >= S->waitTime ) {
        S->waitTime = INF;
        return make_shared<KDTreeNode>(S->timeSamplePoint);
    }
    return randNodeOrGoal( S );
}

//shared_ptr<KDTreeNode> randNodeTimeWithObstacleRemove( shared_ptr<CSpace> S ){}
//shared_ptr<KDTreeNode> randNodeItsWithObstacleRemove( shared_ptr<CSpace> S ){}

shared_ptr<KDTreeNode> randNodeOrFromStack(shared_ptr<CSpace> &S)
{
    if( S->sampleStack->length > 0 ) {
        // Using the sampleStack so KDTreeNode->position is popped
        shared_ptr<KDTreeNode> temp = make_shared<KDTreeNode>();
        S->sampleStack->JlistPop(temp);
        return temp;
    } else {
        return randNodeOrGoal( S );
    }
}

shared_ptr<KDTreeNode> randNodeInTimeOrFromStack(shared_ptr<CSpace> S)
{
    if( S->sampleStack->length > 0 ) {
        // Using the sampleStack so KDTreeNode->position is popped
        shared_ptr<KDTreeNode> temp = make_shared<KDTreeNode>();
        S->sampleStack->JlistPop(temp);
        return make_shared<KDTreeNode>(temp->position);
    } else {
        shared_ptr<KDTreeNode> newNode = randNodeOrGoal( S );
        if( newNode == S->goalNode ) {
            return newNode;
        }

        double minTimeToReachNode = S->start(2)
                + sqrt(
                        (newNode->position(0) - S->root->position(0))
                        *(newNode->position(0) - S->root->position(0))
                        + (newNode->position(1) - S->root->position(1))
                        *(newNode->position(1) - S->root->position(1))
                      ) / S->robotVelocity;

        // If point is too soon vs robot's available speed
        // or if it is in the "past" and the robot is moving
        if( newNode->position(2) < minTimeToReachNode ||
                (newNode->position(2) > S->moveGoal->position(2) &&
                 S->moveGoal != S->goalNode) ) {
            // Resample time in ok range
            double r = (double) rand() / (RAND_MAX);
            newNode->position(2) = minTimeToReachNode
                    + r * (S->moveGoal->position(2) - minTimeToReachNode);
        }
        return newNode;
    }
}

/////////////////////// Geometric Functions ///////////////////////

double DistanceSqrdPointToSegment(Eigen::VectorXd point,
                                  Eigen::Vector2d startPoint,
                                  Eigen::Vector2d endPoint)
{
    Eigen::Vector2d point_position = point.head(2);
    double vx = point_position(0) - startPoint(0);
    double vy = point_position(1) - startPoint(1);
    double ux = endPoint(0) - startPoint(0);
    double uy = endPoint(1) - startPoint(1);
    double determinate = vx*ux + vy*uy;

    if( determinate <= 0 ) {
        return vx*vx + vy*vy;
    } else {
        double len = ux*ux + uy*uy;
        if( determinate >= len ) {
            return (endPoint(0)-point_position(0))
                    *(endPoint(0)-point_position(0))
                    + (endPoint(1)-point_position(1))
                    *(endPoint(1)-point_position(1));
        } else {
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
    double time_start = getTimeNs(startTime);
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

    double time_end = getTimeNs(startTime);
    if(timing) cout << "SegmentDistSqrd: " << (time_end - time_start)/1000000000 << endl;
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

shared_ptr<JList> FindPointsInConflictWithObstacle(shared_ptr<CSpace> &S,
                                                   shared_ptr<KDTree> Tree,
                                                   shared_ptr<Obstacle> &O,
                                                   shared_ptr<KDTreeNode> &root)
{
    shared_ptr<JList> node_list = make_shared<JList>(true);
    double search_range = 0;

    if(1 <= O->kind_ && O->kind_ <= 5) {
        // 2D obstacle
        if(!S->spaceHasTime && !S->spaceHasTheta) {
            // Euclidean space without time
            search_range = S->robotRadius + S->delta + O->radius_;
            Tree->kdFindWithinRange(node_list,search_range,O->position_);
        } else if(!S->spaceHasTime && S->spaceHasTheta) {
            // Dubin's robot without time [x,y,theta]
            search_range = S->robotRadius + O->radius_ + PI; // + S->delta
//            cout << "search_range: " << search_range << endl;
            Eigen::Vector3d obs_center_dubins;
            obs_center_dubins << O->position_(0), O->position_(1), PI;
//            cout << "about point:\n" << obs_center_dubins << endl;
            Tree->kdFindWithinRange(node_list,search_range,obs_center_dubins);
//            cout << "number of points in conflict: " << node_list->length << endl;
        } else {
            cout << "Error: This type of obstacle not coded for this space"
                 << endl;
        }
    } else if(6 <= O->kind_ && O->kind_ <= 7) {
        // 2D obstacle with time, find points within range of each point along
        // the time path, accumulating all points that are in any of the
        // bounding hyperspheres
        // [x,y,theta,time]

        double base_search_range = S->robotRadius + S->delta + O->radius_;
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

            if(S->spaceHasTheta) query_pose << query_pose, PI;
            cout << "query_pose:\n" << query_pose << endl;

            search_range = base_search_range
                    + Tree->distanceFunction(O->path_.row(i),
                                             O->path_.row(j))/2.0;

            if(S->spaceHasTheta) search_range += PI;

            if(i == 1)
                Tree->kdFindWithinRange(node_list,search_range,query_pose);
            else
                Tree->kdFindMoreWithinRange(node_list,search_range,query_pose);

            if(j == O->path_.rows()) break;
        }
    } else cout << "This case has not been coded yet." << endl;

    return node_list;
}

void AddNewObstacle(shared_ptr<KDTree> Tree,
                    shared_ptr<Queue> &Q,
                    shared_ptr<Obstacle> &O,
                    shared_ptr<KDTreeNode> root,
                    shared_ptr<RobotData> &R)
{
    cout << "AddNewObstacle" << endl;
    // Find all points in conflict with the obstacle
    shared_ptr<JList> node_list
            = FindPointsInConflictWithObstacle(Q->S,Tree,O,root);

    // For all nodes that might be in conflict
    shared_ptr<KDTreeNode> this_node;
    shared_ptr<double> key = make_shared<double>(0);
//    cout << "points in conflict: " << node_list->length << endl;
    while(node_list->length > 0) {
//        cout << "point in conflict: " << endl;
        Tree->popFromRangeList(node_list,this_node,key);
//        cout << this_node->position << endl;
        // Check all their edges

        // See if this node's neighbors can be reached

        // Get an iterator for this node's out neighbors
        shared_ptr<RRTNodeNeighborIterator> this_node_out_neighbors
                = make_shared<RRTNodeNeighborIterator>(this_node);

        // Iterate through list
        shared_ptr<JListNode> list_item
                = nextOutNeighbor(this_node_out_neighbors);
        shared_ptr<JListNode> next_item;
        shared_ptr<Edge> neighbor_edge;
        while(list_item->key != -1.0) {
            neighbor_edge = list_item->edge;
            next_item = nextOutNeighbor(this_node_out_neighbors);
            if(neighbor_edge->ExplicitEdgeCheck(O))
                // Mark edge to neighbor at INF cost
                list_item->edge->dist = INF;
            list_item = next_item;
        }

        // See if this node's parent can be reached
        if(this_node->rrtParentUsed
                && this_node->rrtParentEdge->ExplicitEdgeCheck(O)) {
            // Remove this_node from it's parent's successor list
            this_node->rrtParentEdge->endNode->SuccessorList->JlistRemove(
                        this_node->successorListItemInParent);

            // This node now has no parent
            this_node->rrtParentEdge->endNode = this_node;
            this_node->rrtParentEdge->dist = INF;
            this_node->rrtParentUsed = false;

            verifyInOSQueue(Q,this_node);
        }
    }

    // Clean up
    Tree->emptyRangeList(node_list);

    // Now check the robot's current move to its target
    if(R->robotEdgeUsed && R->robotEdge->ExplicitEdgeCheck(O))
        R->currentMoveInvalid = true;
}

void RemoveObstacle(std::shared_ptr<KDTree> Tree,
                    std::shared_ptr<Queue> &Q,
                    std::shared_ptr<Obstacle> &O,
                    std::shared_ptr<KDTreeNode> root,
                    double hyper_ball_rad, double time_elapsed,
                    std::shared_ptr<KDTreeNode> &move_goal)
{
    cout << "RemoveObstacle" << endl;
    bool neighbors_were_blocked, conflicts_with_other_obs;

    // Find all points in conflict with obstacle
    shared_ptr<JList> node_list
            = FindPointsInConflictWithObstacle(Q->S,Tree,O,root);

    // For all nodes that might be in conflict
    shared_ptr<KDTreeNode> this_node = make_shared<KDTreeNode>();
    shared_ptr<double> key = make_shared<double>(0);
//    cout << "points in conflict: " << node_list->length << endl;
    while(node_list->length > 0) {
//        cout << "point in conflict: " << endl;
        Tree->popFromRangeList(node_list,this_node,key);
//        cout << this_node->position << endl;
        // Check all of their edges

        // See if this node's out neighbors were blocked by the obstacle

        // Get an iterator for this node's out neighbors
        shared_ptr<RRTNodeNeighborIterator> this_node_out_neighbors
                = make_shared<RRTNodeNeighborIterator>(this_node);
        neighbors_were_blocked = false;

        // Iterate through list
        shared_ptr<JListNode> list_item
                = nextOutNeighbor(this_node_out_neighbors);
        shared_ptr<ListNode> o_list_item;
        shared_ptr<JListNode> next_item;
        shared_ptr<Edge> neighbor_edge;
        shared_ptr<KDTreeNode> neighbor_node;
        while(list_item->key != -1.0) {
            neighbor_edge = list_item->edge;
            neighbor_node = list_item->edge->endNode;
            next_item = nextOutNeighbor(this_node_out_neighbors);
            if(neighbor_edge->dist == INF
                    && neighbor_edge->ExplicitEdgeCheck(O)) {
                // This edge used to be in collision with at least one
                // obstacle (at least the obstacle in question)
                // Need to check if could be in conflict with other obstacles

                {
                    lock_guard<mutex> lock(Q->S->cspace_mutex_);
                    o_list_item = Q->S->obstacles->front_;
                }
                conflicts_with_other_obs = false;
                shared_ptr<Obstacle> other_obstacle;
                while(o_list_item != o_list_item->child_) {
                    other_obstacle = o_list_item->obstacle_;
                    if(other_obstacle != O
                            && other_obstacle->obstacle_used_
                            && other_obstacle->start_time_ <= time_elapsed
                            && time_elapsed <= (other_obstacle->start_time_
                                               + other_obstacle->life_span_)) {
                        if(neighbor_edge->ExplicitEdgeCheck(other_obstacle)) {
                            conflicts_with_other_obs = true;
                            break;
                        }
                    }
                    o_list_item = o_list_item->child_;
                }

                if(!conflicts_with_other_obs) {
                    // Reset edge length to actual cost
                    list_item->edge->dist = list_item->edge->distOriginal;
                    neighbors_were_blocked = true;
                }
            }
            list_item = next_item;
        }

        if(neighbors_were_blocked) {
            recalculateLMC(Q,this_node,root,hyper_ball_rad);
            if(this_node->rrtTreeCost != this_node->rrtLMC
                    && Q->Q->lessThan(this_node,move_goal))
                verifyInQueue(Q,this_node);
        }
    }
    Tree->emptyRangeList(node_list);
    O->obstacle_used_ = false;
}

bool checkHeapForEdgeProblems( shared_ptr<Queue> &Q,
                               shared_ptr<KDTree> Tree )
{
    shared_ptr<KDTreeNode> node;
    for( int i = 0; i < Q->Q->indexOfLast; i++ ) {
        node = Q->Q->H[i];
        if( checkNeighborsForEdgeProblems( Q->S, node, Tree ) ) return true;
    }
    return false;
}

bool checkNeighborsForEdgeProblems(shared_ptr<CSpace>& S,
                                   shared_ptr<KDTreeNode> thisNode,
                                   shared_ptr<KDTree> Tree)
{
    shared_ptr<Edge> this_edge_1, this_edge_2;
    if( thisNode->rrtParentUsed ) {
        this_edge_1 = Edge::newEdge(S, Tree, thisNode,
                                                   thisNode->rrtParentEdge->endNode);
        if( ExplicitEdgeCheck(S, this_edge_1)) {
            return true;
        }
    }

    shared_ptr<JListNode> listItem = thisNode->rrtNeighborsOut->front;
    shared_ptr<KDTreeNode> neighborNode;
    while( listItem != listItem->child ) {
        neighborNode = listItem->node;

        this_edge_2 = Edge::newEdge(S, Tree, neighborNode,
                                  neighborNode->rrtParentEdge->endNode);

        if( neighborNode->rrtParentUsed
                && ExplicitEdgeCheck(S,this_edge_2)) {
            return true;
        }

        listItem = listItem->child; // iterate
    }
    return false;
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
        time_start = getTimeNs(startTime);
        double dist_sqrd = DistanceSqrdPointToSegment(O->position_,
                                                      start_point.head(2),
                                                      end_point.head(2));
        time_end = getTimeNs(startTime);
        if(timing) cout << "\tDistanceSqrdPointToSegment: "
                        << (time_end - time_start)/1000000000 << endl;
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
        time_start = getTimeNs(startTime);
        Eigen::Vector2d A = O->polygon_.row(O->polygon_.rows()-1);
        Eigen::Vector2d B;
        double seg_dist_sqrd;
        for(int i = 0; i < O->polygon_.rows(); i++) {
            B = O->polygon_.row(i);
            seg_dist_sqrd = SegmentDistSqrd(start_point,end_point,A,B);
            if(seg_dist_sqrd < pow(radius,2)) {
                // There is a collision with the 2d projection of the obstacle
//                cout << "p_edge -- traj_edge: " << sqrt(seg_dist_sqrd)
//                     << "\n<\nradius:" << radius << endl;
                if(O->kind_ == 5);
                else return true;
            }
            A = B;
        }
        time_end = getTimeNs(startTime);
        if(timing) cout << "\tCheckingEdges: "
                        << (time_end - time_start)/1000000000 << endl;
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

bool ExplicitEdgeCheck(shared_ptr<CSpace> &S,
                       shared_ptr<Edge> &edge)
{
    // If ignoring obstacles
    if( S->inWarmupTime ) return false;

    shared_ptr<ListNode> obstacle_list_node;
    int length;
    {
        lock_guard<mutex> lock(S->cspace_mutex_);
        obstacle_list_node = S->obstacles->front_;
        length = S->obstacles->length_;
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
                  shared_ptr<CSpace> &C)
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
        // First transform position and polygon to appropriate position
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

bool QuickCheck(shared_ptr<CSpace> &C, Eigen::VectorXd point)
{
    shared_ptr<ListNode> obstacle_list_node;
    int length;
    {
        lock_guard<mutex> lock(C->cspace_mutex_);
        obstacle_list_node = C->obstacles->front_;
        length = C->obstacles->length_;
    }
    for(int i = 0; i < length; i++) {
        if(QuickCheck2D(obstacle_list_node->obstacle_,point,C)) return true;
        obstacle_list_node = obstacle_list_node->child_;
    }
    return false;
}

bool ExplicitPointCheck2D(shared_ptr<CSpace> &C,
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
        // First transform position and polygon to appropriate ponsition
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
    if(Q->S->inWarmupTime) return false;

    // First do quick check to see if the point can be determined in collision
    // with minimal work (quick check is not implicit check)
    if(QuickCheck(Q->S,point)) return true;

    // Point is not inside any obstacles but still may be in collision
    // because of the robot radius
    shared_ptr<ListNode> obstacle_list_node;
    int length;
    {
        lock_guard<mutex> lock(Q->S->cspace_mutex_);
        obstacle_list_node = Q->S->obstacles->front_;
        length = Q->S->obstacles->length_;
    }
    for(int i = 0; i < length; i++) {
        if(ExplicitPointCheck2D(Q->S,obstacle_list_node->obstacle_,
                                point, Q->S->robotRadius)) return true;
        obstacle_list_node = obstacle_list_node->child_;
    }
    return false;
}

bool ExplicitNodeCheck(shared_ptr<Queue>& Q, shared_ptr<KDTreeNode> node)
{
    return ExplicitPointCheck(Q,node->position);
}

void RandomSampleObs(shared_ptr<CSpace> &S,
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
    if(!S->spaceHasTime && !S->spaceHasTheta) {
        // Euclidean space, no time
        o_hyper_volume_bound = pow(2.0*O->radius_,S->d);
        if(S->hypervolume == 0.0) S->hypervolume = S->width.prod();
    } else if(!S->spaceHasTime && S->spaceHasTheta) {
        // Dubin's car, no time
        o_hyper_volume_bound = pow(2.0*O->radius_,2); // * S->width(2)
        if(S->hypervolume == 0.0) S->hypervolume = S->width.head(2).prod();
    } else cout << "Not coded yet" << endl;

    double num_obs_samples = Tree->treeSize
            * o_hyper_volume_bound/S->hypervolume + 1.0;
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
            S->sampleStack->JlistPush(point);
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
    time_start = getTimeNs(startTime);
    Tree->kdFindWithinRange(node_list, hyper_ball_rad, new_node->position);
    time_end = getTimeNs(startTime);
    if(timing) cout << "\tkdFindWithinRange: " << (time_end - time_start)/1000000000 << endl;

    // Try to find and link to best parent. This also saves
    // the edges from new_node to the neighbors in the field
    // "tempEdge" of the neighbors. This saves time in the
    // case that trajectory calculation is complicated.
    time_start = getTimeNs(startTime);
    findBestParent( Q->S, Tree, new_node, node_list, closest_node, true );
    time_end = getTimeNs(startTime);
    if(timing) cout << "\tfindBestParent: " << (time_end - time_start)/1000000000 << endl;

    // If no parent was fonud then ignore this node
    if( !new_node->rrtParentUsed ) {
        Tree->emptyRangeList(node_list); // clean up
        return false;
    }

    // Insert the new node into the KDTree
    time_start = getTimeNs(startTime);
    Tree->kdInsert(new_node);
    time_end = getTimeNs(startTime);
    if(timing) cout << "\tkdInsert: " << (time_end - time_start)/1000000000 << endl;

    // Second pass, if there was a parent, then link with neighbors
    // and rewire neighbors that would do better to use new_node as
    // their parent. Note that the edges -from- new_node -to- its
    // neighbors have been stored in "tempEdge" field of the neighbors
    shared_ptr<JListNode> list_item = node_list->front;
    shared_ptr<KDTreeNode> near_node;
    shared_ptr<Edge> this_edge;
    double old_LMC;

    for( int i = 0; i < node_list->length; i++ ) {
        near_node = list_item->node;

        // If edge from new_node to nearNode was valid
        if(list_item->key != -1.0) {
            // Add to initial out neighbor list of new_node
            // (allows info propogation from new_node to nearNode always)
            makeInitialOutNeighborOf( near_node,new_node,near_node->tempEdge );

            // Add to current neighbor list of new_node
            // (allows info propogation from new_node to nearNode and
            // vice versa, but only while they are in the D-ball)
            time_start = getTimeNs(startTime);
            makeNeighborOf( near_node, new_node, near_node->tempEdge );
            time_end = getTimeNs(startTime);
            if(timing) cout << "\tmakeNeighborOf: " << (time_end - time_start)/1000000000 << endl;

        }

        // In the general case, the trajectories along edges are not simply
        // the reverse of each other, therefore we need to calculate
        // and check the trajectory along the edge from nearNode to new_node
        this_edge = Edge::newEdge( Q->S, Tree, near_node, new_node );
        time_start = getTimeNs(startTime);
        this_edge->calculateTrajectory();
        time_end = getTimeNs(startTime);
        if(timing) cout << "\tcalculateTrajectory: " << (time_end - time_start)/1000000000 << endl;

        time_start = getTimeNs(startTime);
        bool edge_is_safe = !ExplicitEdgeCheck(Q->S,this_edge);
        time_end = getTimeNs(startTime);
        if(timing) cout << "\tExplicitEdgeCheck: " << (time_end - time_start)/1000000000 << endl;
        if( this_edge->ValidMove()
                && edge_is_safe ) {
            // Add to initial in neighbor list of newnode
            // (allows information propogation from new_node to
            // nearNode always)
            time_start = getTimeNs(startTime);
            makeInitialInNeighborOf( new_node, near_node, this_edge );
            time_end = getTimeNs(startTime);
            if(timing) cout << "\tmakeInitialNeighborOf: " << (time_end - time_start)/1000000000 << endl;

            // Add to current neighbor list of new_node
            // (allows info propogation from new_node to nearNode and
            // vice versa, but only while they are in D-ball)
            makeNeighborOf( new_node, near_node, this_edge );
        } else {
            // Edge cannot be created
            list_item = list_item->child; // iterate through list
            continue;
        }

        // Rewire neighbors that would do better to use this node
        // as their parent unless they are not in the relevant
        // portion of the space vs. moveGoal
        time_start = getTimeNs(startTime);
        if( near_node->rrtLMC > new_node->rrtLMC + this_edge->dist
                && new_node->rrtParentEdge->endNode != near_node
                && new_node->rrtLMC + this_edge->dist < move_goal->rrtLMC ) {
            // Make this node the parent of the neighbor node
            makeParentOf( new_node, near_node, this_edge, Tree->root );

            // Recalculate tree cost of neighbor
            old_LMC = near_node->rrtLMC;
            near_node->rrtLMC = new_node->rrtLMC + this_edge->dist;

            // Insert neighbor into priority queue if cost
            // reduction is great enough
            if( old_LMC - near_node->rrtLMC > Q->changeThresh
                    && near_node != Tree->root ) {
                verifyInQueue( Q, near_node );
            }
        }
        time_end = getTimeNs(startTime);
        if(timing) cout << "\tRewiringNeighbors: " << (time_end - time_start)/1000000000 << endl;

        list_item = list_item->child; // iterate through list
    }

    Tree->emptyRangeList(node_list); // clean up

    // Insert the node into the priority queue
    Q->Q->addToHeap(new_node);

    return true;
}


/////////////////////// RRT* Functions ///////////////////////

void findBestParent(shared_ptr<CSpace> &S,
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
    if(nodeList->length == 0) {
        if(S->goalNode != newNode) nodeList->JlistPush(closestNode);
    }

    // Update LMC value based on nodes in the list
    newNode->rrtLMC = INF;
    newNode->rrtTreeCost = INF;
    newNode->rrtParentUsed = false;

    // Find best parent (or if one even exists)
    shared_ptr<JListNode> listItem = nodeList->front;
    shared_ptr<KDTreeNode> nearNode;
    shared_ptr<Edge> thisEdge;
    while(listItem->child != listItem) {
        nearNode = listItem->node;

        // First calculate the shortest trajectory (and its distance)
        // that gets from newNode to nearNode while obeying the
        // constraints of the state space and the dynamics
        // of the robot
        thisEdge = Edge::newEdge(S, Tree, newNode, nearNode);
        time_start = getTimeNs(startTime);
        thisEdge->calculateTrajectory();
        time_end = getTimeNs(startTime);
        if(timing) cout << "\t\tcalculateTrajectory: " << (time_end - time_start)/1000000000 << endl;

        if( saveAllEdges ) nearNode->tempEdge = thisEdge;

        // Check for validity vs edge collisions vs obstacles and
        // vs the time-dynamics of the robot and space
        time_start = getTimeNs(startTime);
        bool edge_is_safe = !ExplicitEdgeCheck(S,thisEdge);
        time_end = getTimeNs(startTime);
        if(timing) cout << "\t\tExplicitEdgeCheck: " << (time_end - time_start)/1000000000 << endl;
        if(!edge_is_safe || !thisEdge->ValidMove()) {
            if(saveAllEdges) nearNode->tempEdge->dist = INF;
            listItem = listItem->child; // iterate through list
            continue;
        }

        // Check if need to update rrtParent and rrtParentEdge
        if(newNode->rrtLMC > nearNode->rrtLMC + thisEdge->dist) {
            // Found a potential better parent
            newNode->rrtLMC = nearNode->rrtLMC + thisEdge->dist;
            /// This also takes care of some code in Extend I believe
            time_start = getTimeNs(startTime);
            makeParentOf(nearNode,newNode,thisEdge,Tree->root);
            time_end = getTimeNs(startTime);
            if(timing) cout << "\t\tmakeParentOf: " << (time_end - time_start)/1000000000 << endl;
        }
        listItem = listItem->child; // iterate thorugh list
    }
}


/////////////////////// RRT# Functions ///////////////////////

void resetNeighborIterator( shared_ptr<RRTNodeNeighborIterator> &It )
{It->listFlag = 0;}

void makeNeighborOf(shared_ptr<KDTreeNode> &newNeighbor,
                    shared_ptr<KDTreeNode> &node,
                    shared_ptr<Edge> &edge)
{
    node->rrtNeighborsOut->JlistPush( edge );
    edge->listItemInStartNode = node->rrtNeighborsOut->front;

    newNeighbor->rrtNeighborsIn->JlistPush( edge );
    edge->listItemInEndNode = newNeighbor->rrtNeighborsIn->front;
}

void makeInitialOutNeighborOf(shared_ptr<KDTreeNode> &newNeighbor,
                              shared_ptr<KDTreeNode> &node,
                              shared_ptr<Edge> &edge)
{ node->InitialNeighborListOut->JlistPush( edge ); }

void makeInitialInNeighborOf(shared_ptr<KDTreeNode> &newNeighbor,
                             shared_ptr<KDTreeNode> &node,
                             shared_ptr<Edge> &edge)
{ node->InitialNeighborListIn->JlistPush( edge ); }

void updateQueue( shared_ptr<Queue> &Q,
                  shared_ptr<KDTreeNode> &newNode,
                  shared_ptr<KDTreeNode> &root,
                  double hyperBallRad )
{
    recalculateLMC( Q, newNode, root, hyperBallRad ); // internally ignores root
     if( Q->Q->markedQ( newNode ) ) {
         Q->Q->updateHeap( newNode );
         Q->Q->removeFromHeap( newNode );
     }
     if( newNode->rrtTreeCost != newNode->rrtLMC ) {
         Q->Q->addToHeap( newNode );
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
        Q->Q->topHeap(thisNode);
        while( Q->Q->indexOfLast>0 && (Q->Q->lessQ(thisNode,goalNode)
                                         || goalNode->rrtLMC == INF
                                         || goalNode->rrtTreeCost == INF
                                         || Q->Q->markedQ(goalNode)) ) {
            Q->Q->popHeap(thisNode);
            thisNode->rrtTreeCost = thisNode->rrtLMC;

            listItem = thisNode->rrtNeighborsIn->front;
            for( int i = 0; i < thisNode->rrtNeighborsIn->length; i++ ) {
                neighborNode = listItem->edge->startNode; // dereference?
                updateQueue( Q, neighborNode, root, hyperBallRad );
                listItem = listItem->child; // iterate through list
            }
            Q->Q->topHeap(thisNode);
        }
    } else {*/ // Q->type == "RRTx";
        shared_ptr<KDTreeNode> thisNode;
        Q->Q->topHeap(thisNode);
        while( Q->Q->indexOfLast > 0
               && (Q->Q->lessThan(thisNode, goalNode)
                   || goalNode->rrtLMC == INF
                   || goalNode->rrtTreeCost == INF
                   || Q->Q->marked(goalNode) ) ) {
            Q->Q->popHeap(thisNode);

            // Update neighbors of thisNode if it has
            // changed more than change thresh
            if(thisNode->rrtTreeCost - thisNode->rrtLMC > Q->changeThresh) {
                recalculateLMC( Q, thisNode, root, hyperBallRad );
                rewire( Q, thisNode, root, hyperBallRad, Q->changeThresh );
            }
            thisNode->rrtTreeCost = thisNode->rrtLMC;
            Q->Q->topHeap(thisNode);
        }
//    }
}


/////////////////////// RRTx Functions ///////////////////////

void markOS( shared_ptr<KDTreeNode> &node )
{
    node->inOSQueue = true;
}

void unmarkOS( shared_ptr<KDTreeNode> &node )
{
    node->inOSQueue = false;
}

bool markedOS( shared_ptr<KDTreeNode> node )
{
    return node->inOSQueue;
}

bool verifyInQueue(shared_ptr<Queue> &Q,
                   shared_ptr<KDTreeNode> &node)
{
    if( Q->Q->markedQ(node) ) {
       return Q->Q->updateHeap(node);
    } else {
       return Q->Q->addToHeap(node);
    }
}

bool verifyInOSQueue(shared_ptr<Queue> &Q,
                     shared_ptr<KDTreeNode> &node)
{
    if( Q->Q->markedQ(node) ) {
        Q->Q->updateHeap(node);
        Q->Q->removeFromHeap(node);
    }
    if( !markedOS(node) ) {
        markOS(node);
        Q->OS->JlistPush(node);
    }
    return true;
}

void cullCurrentNeighbors( shared_ptr<KDTreeNode> &node,
                           double hyperBallRad )
{
    // Remove outgoing edges from node that are now too long
    shared_ptr<JListNode> listItem = node->rrtNeighborsOut->front;
    shared_ptr<JListNode> nextItem;
    shared_ptr<Edge> neighborEdge;
    shared_ptr<KDTreeNode> neighborNode;
    while( listItem != listItem->child ) {
        nextItem = listItem->child; // since we may remove listItem from list
        if( listItem->edge->dist > hyperBallRad ) {
            neighborEdge = listItem->edge;
            neighborNode = neighborEdge->endNode;
            node->rrtNeighborsOut->JlistRemove(
                        neighborEdge->listItemInStartNode);
            neighborNode->rrtNeighborsIn->JlistRemove(
                        neighborEdge->listItemInEndNode);
        }
        listItem = nextItem;
    }
}

shared_ptr<JListNode> nextOutNeighbor(
        shared_ptr<RRTNodeNeighborIterator> &It)
{
    if( It->listFlag == 0 ) {
        It->listItem = It->thisNode->InitialNeighborListOut->front;
        It->listFlag = 1;
    } else {
        It->listItem = It->listItem->child;
    }
    while( It->listItem == It->listItem->child ) {
        // Go to the next place tha neighbors are stored
        if( It->listFlag == 1 ) {
            It->listItem = It->thisNode->rrtNeighborsOut->front;
        } else {
            // Done with all neighbors
            // Returns empty JListNode with empty KDTreeNode
            // so can check for this by checking return_value->key == -1
            return make_shared<JListNode>();
        }
        It->listFlag += 1;
    }
    return It->listItem;
}

shared_ptr<JListNode> nextInNeighbor(
        shared_ptr<RRTNodeNeighborIterator> &It)
{
    if( It->listFlag == 0 ) {
        It->listItem = It->thisNode->InitialNeighborListIn->front;
        It->listFlag = 1;
    } else {
        It->listItem = It->listItem->child;
    }
    while( It->listItem == It->listItem->child ) {
        // Go to the next place that neighbors are stored
        if( It->listFlag == 1 ) {
            It->listItem = It->thisNode->rrtNeighborsIn->front;
        } else {
            // Done with all neighbors
            // Returns empty JListNode with empty KDTreeNode
            // so can check for this by checking return_value->key == -1
            return make_shared<JListNode>();
        }
        It->listFlag += 1;
    }
    return It->listItem;
}

void makeParentOf( shared_ptr<KDTreeNode> &newParent,
                   shared_ptr<KDTreeNode> &node,
                   shared_ptr<Edge> &edge,
                   shared_ptr<KDTreeNode> &root )
{
    // Remove the node from its old parent's successor list
    if( node->rrtParentUsed ) {
        node->rrtParentEdge->endNode->SuccessorList->JlistRemove(
                    node->successorListItemInParent );
    }

    // Make newParent the parent of node
    node->rrtParentEdge = edge;
    node->rrtParentUsed = true;

    // Place a (non-trajectory) reverse edge into newParent's
    // successor list and save a pointer to its position in
    // that list. This edge is used to help keep track of
    // successors and not for movement.
    shared_ptr<Edge> backEdge
            = Edge::Edge::newEdge(edge->cspace, edge->tree, newParent, node );
    backEdge->dist = INF;
    newParent->SuccessorList->JlistPush( backEdge, INF );
    node->successorListItemInParent = newParent->SuccessorList->front;
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
    shared_ptr<RRTNodeNeighborIterator> thisNodeOutNeighbors
            = make_shared<RRTNodeNeighborIterator>(node);

    // Set the iterator to the first neighbor
    listItem = nextOutNeighbor( thisNodeOutNeighbors );

    while( listItem->key != -1.0 ) {
        neighborEdge = listItem->edge;
        neighborNode = neighborEdge->endNode;
        neighborDist = neighborEdge->dist;
        nextItem = listItem->child;

        if( markedOS(neighborNode) ) {
            // neighborNode is already in OS queue (orphaned) or unwired
            listItem = nextOutNeighbor( thisNodeOutNeighbors );
            continue;
        }

        if( node->rrtLMC > neighborNode->rrtLMC + neighborDist
                && (!neighborNode->rrtParentUsed
                    || neighborNode->rrtParentEdge->endNode != node)
                && neighborEdge->ValidMove() ) {
            // Found a better parent
            node->rrtLMC = neighborNode->rrtLMC + neighborDist;
            rrtParent = neighborNode;
            parentEdge = listItem->edge;
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
    double deltaCost = node->rrtTreeCost - node->rrtLMC;
    if( deltaCost <= changeThresh ) {
        // Note that using simply "<" causes problems
        // Above note may be outdated
        // node.rrtTreeCost = node.rrtLMC!!! Now happens after return
        cout << "not rewiring" << endl;
        return false;
    }

    // Remove outdated nodes from the current neighbors list
    cullCurrentNeighbors( node, hyperBallRad );

    // Get an iterator for this node's neighbors and iterate through list
    shared_ptr<RRTNodeNeighborIterator> thisNodeInNeighbors
            = make_shared<RRTNodeNeighborIterator>(node);
    shared_ptr<JListNode> listItem
            = nextInNeighbor( thisNodeInNeighbors );
    shared_ptr<KDTreeNode> neighborNode;
    shared_ptr<Edge> neighborEdge;

    while( listItem->key != -1.0 ) {
        neighborEdge = listItem->edge;
        neighborNode = neighborEdge->startNode;

        // Ignore this node's parent and also nodes that cannot
        // reach node due to dynamics of robot or space
        // Not sure about second parent since neighbors are not
        // initially created that cannot reach this node
        if( (node->rrtParentUsed
             && node->rrtParentEdge->endNode == neighborNode)
                || !neighborEdge->ValidMove() ) {
            listItem = nextInNeighbor( thisNodeInNeighbors );
            continue;
        }

        neighborEdge = listItem->edge;

        if( neighborNode->rrtLMC  > node->rrtLMC + neighborEdge->dist
                && (!neighborNode->rrtParentUsed
                    || neighborNode->rrtParentEdge->endNode != node )
                && neighborEdge->ValidMove() ) {
            // neighborNode should use node as its parent (it might already)
            neighborNode->rrtLMC = node->rrtLMC + neighborEdge->dist;
            makeParentOf( node, neighborNode, neighborEdge, root );

            // If the reduction is great enough, then propogate
            // through the neighbor
            if( neighborNode->rrtTreeCost - neighborNode->rrtLMC
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
    if( Q->OS->length <= 0 ) {
        return false;
    }

    // First pass, accumulate all such nodes in a single list, and mark
    // them as belonging in that list, we'll just use the OS stack we've
    // been using adding nodes to the front while moving from back to front
    shared_ptr<JListNode> OS_list_item = Q->OS->back;
    shared_ptr<KDTreeNode> thisNode, successorNode;
    shared_ptr<JListNode> SuccessorList_item;
    while( OS_list_item != OS_list_item->parent ) {
        thisNode = OS_list_item->node;

        // Add all of this node's successors to OS stack
        SuccessorList_item = thisNode->SuccessorList->front;
        while( SuccessorList_item != SuccessorList_item->child ) {
            successorNode = SuccessorList_item->edge->endNode;
            verifyInOSQueue( Q, successorNode ); // pushes to front of OS
            SuccessorList_item = SuccessorList_item->child;
        }

        OS_list_item = OS_list_item->parent;
    }

    // Second pass, put all -out neighbors- of the nodes in OS
    // (not including nodes in OS) into Q and tell them to force rewire.
    // Not going back to front makes Q adjustments slightly faster,
    // since nodes near the front tend to have higher costs
    OS_list_item = Q->OS->back;
    shared_ptr<JListNode> listItem;
    shared_ptr<KDTreeNode> neighborNode;
    while( OS_list_item != OS_list_item->parent ) {
        thisNode = OS_list_item->node;

        // Get an iterator for this node's neighbors
        shared_ptr<RRTNodeNeighborIterator> thisNodeOutNeighbors
                = make_shared<RRTNodeNeighborIterator>(thisNode);

        // Now iterate through list (add all neighbors to the Q,
        // except those in OS
        listItem = nextOutNeighbor( thisNodeOutNeighbors );
        while( listItem->key != -1.0 ) {
            neighborNode = listItem->edge->endNode;

            if( markedOS(neighborNode) ) {
                // neighborNode already in OS queue (orphaned) or unwired
                listItem = nextOutNeighbor( thisNodeOutNeighbors );
                continue;
            }

            // Otherwise, make sure that neighborNode is in normal queue
            neighborNode->rrtTreeCost = INF; // node will be inserted with LMC
                                             // key and then guarenteed to
                                             // propogate cost forward since
                                     // useful nodes have rrtLMC < rrtTreeCost
            verifyInQueue( Q, neighborNode );

            listItem = nextOutNeighbor( thisNodeOutNeighbors );
        }

        // Add parent to the Q, unless it is in OS
        if( thisNode->rrtParentUsed
                && !markedOS((thisNode->rrtParentEdge->endNode)) ) {
            thisNode->kdParent->rrtTreeCost = INF; // rrtParent = kdParent???
            verifyInQueue( Q, thisNode->rrtParentEdge->endNode );
        }

        OS_list_item = OS_list_item->parent;
    }

    // Third pass, remove all nodes from OS, unmark them, and
    // remove their connections to their parents. If one was the
    // robot's target then take appropriate measures
    while( Q->OS->length > 0 ) {
        Q->OS->JlistPop(thisNode);
        unmarkOS(thisNode);

        if( thisNode == R->nextMoveTarget ) {
            R->currentMoveInvalid = true;
        }

        if( thisNode->rrtParentUsed ) {
            // Remove thisNode from its parent's successor list
            thisNode->rrtParentEdge->endNode->SuccessorList->JlistRemove(
                        thisNode->successorListItemInParent );

            // thisNode now has no parent
            thisNode->rrtParentEdge
                    = Edge::newEdge(Q->S,Tree,thisNode,thisNode);
            thisNode->rrtParentEdge->dist = INF;
            thisNode->rrtParentUsed = false;
        }

        thisNode->rrtTreeCost = INF;
        thisNode->rrtLMC = INF;
    }
    return true;
}

void addOtherTimesToRoot( shared_ptr<CSpace> &S,
                          shared_ptr<KDTree> &Tree,
                          shared_ptr<KDTreeNode> &goal,
                          shared_ptr<KDTreeNode> &root,
                          string searchType )
{
    double insertStep = 2.0;

    double lastTimeToInsert = goal->position(2)
            - Tree->distanceFunction(root->position,goal->position)
            /S->robotVelocity;
    double firstTimeToInsert = S->start(2) + insertStep;
    shared_ptr<KDTreeNode> previousNode = root;
    bool safeToGoal = true;
    Eigen::VectorXd newPose;
    shared_ptr<KDTreeNode> newNode;
    shared_ptr<Edge> thisEdge;
    for( double timeToInsert = firstTimeToInsert;
         timeToInsert < lastTimeToInsert;
         timeToInsert += insertStep ) {
        newPose = root->position;
        newPose(2) = timeToInsert;

        newNode = make_shared<KDTreeNode>(newPose);

        // Edge from newNode to previousNode
        thisEdge = Edge::newEdge( S, Tree, newNode, previousNode );
        thisEdge->calculateHoverTrajectory();

        if( searchType == "RRT*" ) {
            // Make this node the parent of the neighbor node
            newNode->rrtParentEdge = thisEdge;
            newNode->rrtParentUsed = true;
        } else if( searchType == "RRT#" ) {
            // Make this node the parent of the neighbor node
            newNode->rrtParentEdge = thisEdge;
            newNode->rrtParentUsed = true;
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
            thisEdge->dist = INF;
            safeToGoal = false;
            newNode->rrtLMC = INF;
            newNode->rrtTreeCost = INF;
        } else if( safeToGoal ) {
            // If the edge has safe path all the way to the "real" goal
            // then make the cost of reaching "real" goal 0 from newNode
            thisEdge->dist = 0.0;
            newNode->rrtLMC = 0.0;
            newNode->rrtTreeCost = 0.0;
        } else {
            thisEdge->dist = INF;
            newNode->rrtLMC = INF;
            newNode->rrtTreeCost = INF;
        }

        Tree->kdInsert(newNode);

        previousNode = newNode;
    }
}


// debug: gets goal node which should be too far away on first movement
void findNewTarget(shared_ptr<CSpace> &S,
                   shared_ptr<KDTree> &Tree,
                   shared_ptr<RobotData> &R,
                   double hyperBallRad )
{
    Eigen::VectorXd robPose, nextPose;
    R->robotEdgeUsed = false;
    R->distAlongRobotEdge = 0.0;
    R->timeAlongRobotEdge = 0.0;
    nextPose = R->nextMoveTarget->position;
    {
        lock_guard<mutex> lock(R->robotMutex_);
        robPose = R->robotPose;
    }

    // Move target has become invalid
    double searchBallRad
            = max(hyperBallRad, Tree->distanceFunction(robPose, nextPose));

    double maxSearchBallRad
            = Tree->distanceFunction(S->lowerBounds, S->upperBounds);
    searchBallRad = min( searchBallRad, maxSearchBallRad );
    shared_ptr<JList> L = make_shared<JList>(true);
    Tree->kdFindWithinRange( L, searchBallRad, robPose );

    shared_ptr<KDTreeNode> dummyRobotNode
            = make_shared<KDTreeNode>(robPose);
    shared_ptr<Edge> edgeToBestNeighbor
            = Edge::newEdge(S,Tree,dummyRobotNode,dummyRobotNode);

    double bestDistToNeighbor, bestDistToGoal;
    shared_ptr<KDTreeNode> bestNeighbor, neighborNode;

    while( true ) { // will break out when done
        // Searching for new target within radius searchBallRad
        bestDistToNeighbor = INF;
        bestDistToGoal = INF;
        bestNeighbor = make_shared<KDTreeNode>();

        shared_ptr<JListNode> ptr = L->front;
        shared_ptr<Edge> thisEdge;
        double distToGoal;
        while( ptr != ptr->child ) {
            neighborNode = ptr->node;

            thisEdge = Edge::newEdge( S,Tree,dummyRobotNode, neighborNode );
            thisEdge->calculateTrajectory();

            if( thisEdge->ValidMove()
                    && !ExplicitEdgeCheck(S,thisEdge) ) {
                // A safe point was found, see if it is the best so far
                distToGoal = neighborNode->rrtLMC + thisEdge->dist;
                if( distToGoal < bestDistToGoal
                        && thisEdge->ValidMove() ) {
                    // Found a new and better neighbor
                    bestDistToGoal = distToGoal;
                    bestDistToNeighbor = thisEdge->dist;
                    bestNeighbor = neighborNode;
                    edgeToBestNeighbor = thisEdge;
                } else { /*error("ptooie");*/ }
            }

            ptr = ptr->child;
        }
        // Done trying to find a target within ball radius of searchBallRad

        // If a valid neighbor was found, then use it
        if( bestDistToGoal != INF ) {
            R->nextMoveTarget = bestNeighbor;
            R->distanceFromNextRobotPoseToNextMoveTarget = bestDistToNeighbor;
            R->currentMoveInvalid = false;
            // Found a valid move target

            R->robotEdge = edgeToBestNeighbor; /** this edge is empty **/
            R->robotEdgeUsed = true;

            if( S->spaceHasTime ) {
                R->timeAlongRobotEdge = 0.0;
                // note this is updated before robot moves
            } else {
                R->distAlongRobotEdge = 0.0;
                // note this is updated before robot moves
            }

            // Set moveGoal to be nextMoveTarget
            // NOTE may want to actually insert a new node at the robot's
            // position and use that instead, since these "edges" created
            // between robot pose and R.nextMoveTarget may be lengthy
            S->moveGoal->isMoveGoal = false;
            S->moveGoal = R->nextMoveTarget;
            S->moveGoal->isMoveGoal = true;
            break;
        }

        searchBallRad *= 2;
        if( searchBallRad > maxSearchBallRad ) {
            // Unable to find a valid move target so sample randomly
            shared_ptr<KDTreeNode> newNode = randNodeDefault(S);
            double thisDist = Tree->distanceFunction(newNode->position,
                                                     robPose);
            Edge::saturate(
                        newNode->position,
                        robPose, S->delta, thisDist);
            Tree->kdInsert(newNode);
        }
        Tree->kdFindMoreWithinRange( L, searchBallRad, robPose );

    }
    Tree->emptyRangeList(L); // cleanup
}

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
        {
            lock_guard<mutex> lock(R->robotMutex_);
            cout << "Moving "
                      << Tree->distanceFunction(R->robotPose,R->nextRobotPose)
                      << " units" << endl;
            R->robotPose = R->nextRobotPose;
        }

        //R.robotMovePath[R.numRobotMovePoints+1:R.numRobotMovePoints+R.numLocalMovePoints,:] = R.robotLocalPath[1:R.numLocalMovePoints,:];
        for( int i = 0; i < R->numLocalMovePoints-1; i++ ) {
            R->robotMovePath.row(R->numRobotMovePoints+i) = R->robotLocalPath.row(i);
        }
        R->numRobotMovePoints += R->numLocalMovePoints;

        {
            lock_guard<mutex> lock(R->robotMutex_);
            if( !Q->S->spaceHasTime ) {
                cout << "new robot pose(w/o time):\n"
                          << R->robotPose << endl;
            } else {
                cout << "new robot pose(w/ time):\n"
                          << R->robotPose << endl;
            }
        }
        Q->S->warmup_time_just_ended = false;
    } else {
        // Movement has just started, so remember that the robot is now moving
        R->moving = true;

        error("First pose:");
        {
            lock_guard<mutex> lock(R->robotMutex_);
            cout << R->robotPose << endl;
        }

        if( !Q->S->moveGoal->rrtParentUsed ) {
            // no parent has been found for the node at the robots position
            R->currentMoveInvalid = true;
        } else {
            R->robotEdge = Q->S->moveGoal->rrtParentEdge;
            R->robotEdgeUsed = true;

            if( Q->S->spaceHasTime ) {
                R->timeAlongRobotEdge = 0.0;
            } else {
                R->distAlongRobotEdge = 0.0;
            }
        }
        Q->S->warmup_time_just_ended = true;
    }


    // If the robot's current move target has been invalidate due to
    // dynamic obstacles then we need to attempt to find a new
    // (safe) move target. NOTE we handle newly invalid moveTarget
    // after moving the robot (since the robot has already moved this
    // time slice)
    if( R->currentMoveInvalid ) {
        findNewTarget( Q->S, Tree, R, hyperBallRad );
    } else {
        /* Recall that moveGoal is the node whose key is used to determine
         * the level set of cost propogation (this should theoretically
         * be further than the robot from the root of the tree, which
         * will happen here assuming that robot moves at least one edge
         * each slice time. Even if that does not happen, things will
         * still be okay in practice as long as robot is "close" to moveGoal
         */
        Q->S->moveGoal->isMoveGoal = false;
        Q->S->moveGoal = R->nextMoveTarget;
        Q->S->moveGoal->isMoveGoal = true;
    }


    /* Finally, we calculate the point to which the robot will move in
    * slice_time and remember it for the next time this function is called.
    * Also remember all the nodes that it will visit along the way in the
    * local path and the part of the edge trajectory that takes the robot
    * to the first local point (the latter two things are used for
    * visualizition)
    */
    if( !Q->S->spaceHasTime ) {
        // Not using the time dimension, so assume speed is equal to robotVelocity
        shared_ptr<KDTreeNode> nextNode = R->nextMoveTarget;

        // Calculate distance from robot to the end of
        // the current edge it is following
        double nextDist = R->robotEdge->dist - R->distAlongRobotEdge;

        double distRemaining = Q->S->robotVelocity*slice_time;

        // Save first local path point
        R->numLocalMovePoints = 1;
        {
            lock_guard<mutex> lock(R->robotMutex_);
            R->robotLocalPath.row(R->numLocalMovePoints-1) = R->robotPose;
        }
        // Starting at current location (and looking ahead to nextNode), follow
        // parent pointers back for appropriate distance (or root or dead end)
        while( nextDist <= distRemaining && nextNode != root
               && nextNode->rrtParentUsed
               && nextNode != nextNode->rrtParentEdge->endNode ) {
            // Can go all the way to nextNode and still have
            // some distance left to spare

            // Remember robot will move through this point
            R->numLocalMovePoints += 1;
            R->robotLocalPath.row(R->numLocalMovePoints) = nextNode->position;

            // Recalculate remaining distance
            distRemaining -= nextDist;

            // Reset distance along edge
            R->distAlongRobotEdge = 0.0;

            // Update trajectory that the robot will be in the middle of
            R->robotEdge = nextNode->rrtParentEdge;
            R->robotEdgeUsed = true;

            // Calculate the dist of that trajectory
            nextDist = R->robotEdge->dist;

            // Update the next node (at the end of that trajectory)
            nextNode = R->robotEdge->endNode;
        }


        // either 1) nextDist > distRemaining
        // or     2) the path we were following now ends at nextNode

        // Calculate the next pose of the robot
        if( nextDist > distRemaining ) {
            R->distAlongRobotEdge += distRemaining;
            R->nextRobotPose
                    = R->robotEdge->poseAtDistAlongEdge(R->distAlongRobotEdge);
        } else {
            R->nextRobotPose = nextNode->position;
            R->distAlongRobotEdge = R->robotEdge->dist;
        }

        R->nextMoveTarget = R->robotEdge->endNode;

        // Remember last point in local path
        R->numLocalMovePoints += 1;
        R->robotLocalPath.row(R->numLocalMovePoints) = R->nextRobotPose;
    } else { // S->spaceHasTime
        // Space has time, so path is parameterized by time as well
        shared_ptr<KDTreeNode> nextNode = R->nextMoveTarget;

        // Save first local path point
        double targetTime;
        R->numLocalMovePoints = 1;
        {
            lock_guard<mutex> lock(R->robotMutex_);
            R->robotLocalPath.row(R->numLocalMovePoints) = R->robotPose;
            targetTime = R->robotPose(2) - slice_time;
        }
        while( targetTime < R->robotEdge->endNode->position(2)
               && nextNode != root && nextNode->rrtParentUsed
               && nextNode != nextNode->rrtParentEdge->endNode ) {
            // Can go all the way to nextNode and still have some
            // time left to spare

            // Remember the robot will move through this point
            R->numLocalMovePoints += 1;
            R->robotLocalPath.row(R->numLocalMovePoints) = nextNode->position;

            // Update trajectory that the robot will be in the middle of
            R->robotEdge = nextNode->rrtParentEdge;
            R->robotEdgeUsed = true;

            // Update the next node (at the end of that trajectory)
            nextNode = nextNode->rrtParentEdge->endNode;
        }

        // either: 1) targetTime >= nextNode.position(2)
        // or      2) the path we were following now ends at nextNode

        // Calculate the next pose of the robot
        if( targetTime >= nextNode->position(2) ) {
            R->timeAlongRobotEdge = R->robotEdge->startNode->position(2)
                    - targetTime;
            R->nextRobotPose
                    = R->robotEdge->poseAtTimeAlongEdge(R->timeAlongRobotEdge);
        } else {
            // The next node is the end of this tree and we reach it
            R->nextRobotPose = nextNode->position;
            R->timeAlongRobotEdge = R->robotEdge->startNode->position(2)
                    - R->robotEdge->endNode->position(2);
        }

        R->nextMoveTarget = R->robotEdge->endNode;

        // Remember the last point in the local path
        R->numLocalMovePoints += 1;
        R->robotLocalPath.row(R->numLocalMovePoints) = R->nextRobotPose;
    }
}
