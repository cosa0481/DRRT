/* drrt.cpp
 * Corin Sandford
 * Fall 2016
 * Contains RRTX "main" function at bottom.
 */

#include <DRRT/drrt.h>
#include <DRRT/obstacle.h>

using namespace std;

bool timing = false;
int seg_dist_sqrd = 0;
int dist_sqrd_point_seg = 0;

///////////////////// Print Helpers ///////////////////////
void error(string s) { cout << s << endl; }
void error(int i) { cout << i << endl; }

///////////////////// Helper Functions ///////////////////////
void PrintRrtxPath(shared_ptr<KDTreeNode> &leaf)
{
    std::cout << "\nRRTx Path" << std::endl;
    while(leaf->rrt_parent_used_) {
        cout << "pose: " << leaf->rrt_LMC_ << "\n" << leaf->position_ << endl;
        cout << "VVVVVVVV" << endl;
        leaf = leaf->rrt_parent_edge_->end_node_;
    }
    cout << leaf->position_ << endl;
}

double GetTimeNs( chrono::time_point
                  <chrono::high_resolution_clock> start )
{
    return chrono::duration_cast<chrono::nanoseconds>(
                chrono::high_resolution_clock::now()-start).count();
}


/////////////////////// Node Functions ///////////////////////

double ExtractPathLength(shared_ptr<KDTreeNode> node,
                         shared_ptr<KDTreeNode> root)
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

/////////////////////// Collision Checking Functions ///////////////////////

bool CheckHeapForEdgeProblems( shared_ptr<Queue> &Q,
                               shared_ptr<KDTree> Tree )
{
    shared_ptr<KDTreeNode> node;
    for( int i = 0; i < Q->priority_queue->index_of_last_; i++ ) {
        node = Q->priority_queue->heap_[i];
        if( CheckNeighborsForEdgeProblems( Q->cspace, node, Tree ) ) return true;
    }
    return false;
}

bool CheckNeighborsForEdgeProblems(shared_ptr<ConfigSpace>& C,
                                   shared_ptr<KDTreeNode> thisNode,
                                   shared_ptr<KDTree> Tree)
{
    shared_ptr<Edge> this_edge_1, this_edge_2;
    if( thisNode->rrt_parent_used_ ) {
        this_edge_1 = Edge::NewEdge(C, Tree, thisNode,
                                                   thisNode->rrt_parent_edge_->end_node_);
        if( ExplicitEdgeCheck(C, this_edge_1)) {
            return true;
        }
    }

    shared_ptr<JListNode> listItem = thisNode->rrt_neighbors_out_->front_;
    shared_ptr<KDTreeNode> neighborNode;
    while( listItem != listItem->child_ ) {
        neighborNode = listItem->node_;

        this_edge_2 = Edge::NewEdge(C, Tree, neighborNode,
                                  neighborNode->rrt_parent_edge_->end_node_);

        if( neighborNode->rrt_parent_used_
                && ExplicitEdgeCheck(C,this_edge_2)) {
            return true;
        }

        listItem = listItem->child_; // iterate
    }
    return false;
}

void RandomSampleObs(shared_ptr<ConfigSpace> &C,
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
    if(!C->space_has_time_ && !C->space_has_theta_) {
        // Euclidean space, no time
        o_hyper_volume_bound = pow(2.0*O->radius_,C->num_dimensions_);
        if(C->hyper_volume_ == 0.0) C->hyper_volume_ = C->width_.prod();
    } else if(!C->space_has_time_ && C->space_has_theta_) {
        // Dubin's car, no time
        o_hyper_volume_bound = pow(2.0*O->radius_,2); // * S->width_(2)
        if(C->hyper_volume_ == 0.0) C->hyper_volume_ = C->width_.head(2).prod();
    } else cout << "Not coded yet" << endl;

    double num_obs_samples = Tree->tree_size_
            * o_hyper_volume_bound/C->hyper_volume_ + 1.0;
    Eigen::VectorXd new_point;
    Eigen::Array2d temp, temp1;
    for(int i = 0; i < num_obs_samples; i++) {
        new_point = RandPointDefault(C);
        temp = O->position_;
        temp = temp - O->radius_;
        temp1 = new_point.head(2);
        temp1 = temp1*2.0*O->radius_;
        new_point.head(2) = temp + temp1;
        if(true/*UNDEFINED SYMBOLQuickCheck2D(C,new_point.head(2),O)*/) {
            shared_ptr<KDTreeNode> point = make_shared<KDTreeNode>(new_point);
            C->sample_stack_->JListPush(point);
        }
    }
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
    FindBestParent( Q->cspace, Tree, new_node, node_list, closest_node, true );
    time_end = GetTimeNs(startTime);
    if(timing) cout << "\tfindBestParent: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

    // If no parent was found then ignore this node
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
            MakeInitialOutNeighborOf( near_node,new_node,near_node->temp_edge_ );

            // Add to current neighbor list of new_node
            // (allows info propogation from new_node to nearNode and
            // vice versa, but only while they are in the D-ball)
            time_start = GetTimeNs(startTime);
            MakeNeighborOf( near_node, new_node, near_node->temp_edge_ );
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
            MakeInitialInNeighborOf( new_node, near_node, this_edge );
            time_end = GetTimeNs(startTime);
            if(timing) cout << "\tmakeInitialNeighborOf: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

            // Add to current neighbor list of new_node
            // (allows info propogation from new_node to nearNode and
            // vice versa, but only while they are in D-ball)
            MakeNeighborOf( new_node, near_node, this_edge );
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
            MakeParentOf( new_node, near_node, this_edge);

            // Recalculate tree cost of neighbor
            old_LMC = near_node->rrt_LMC_;
            near_node->rrt_LMC_ = new_node->rrt_LMC_ + this_edge->dist_;

            // Insert neighbor into priority queue if cost
            // reduction is great enough
            if( old_LMC - near_node->rrt_LMC_ > Q->change_thresh
                    && near_node != Tree->root ) {
                VerifyInQueue( Q, near_node );
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

void FindBestParent(shared_ptr<ConfigSpace> &C,
                    shared_ptr<KDTree> &Tree,
                    shared_ptr<KDTreeNode>& new_node,
                    shared_ptr<JList> &node_list,
                    shared_ptr<KDTreeNode>& closest_node,
                    bool save_all_edges)
{
   // cout << "FindBestParent" << endl;
    /// For function duration testing
    double time_start, time_end; // in nanoseconds
    chrono::time_point<chrono::high_resolution_clock> startTime
            = chrono::high_resolution_clock::now();

    // If the list is empty
    if(node_list->length_ == 0) {
        if(C->goal_node_ != new_node) node_list->JListPush(closest_node);
    }

    // Update LMC value based on nodes in the list
    new_node->rrt_LMC_ = INF;
    new_node->rrt_tree_cost_ = INF;
    new_node->rrt_parent_used_ = false;

    // Find best parent (or if one even exists)
    shared_ptr<JListNode> listItem = node_list->front_;
    shared_ptr<KDTreeNode> nearNode;
    shared_ptr<Edge> thisEdge;
    while(listItem->child_ != listItem) {
        nearNode = listItem->node_;

        // First calculate the shortest trajectory (and its distance)
        // that gets from newNode to nearNode while obeying the
        // constraints of the state space and the dynamics
        // of the robot
        thisEdge = Edge::NewEdge(C, Tree, new_node, nearNode);
        time_start = GetTimeNs(startTime);
        thisEdge->CalculateTrajectory();
        time_end = GetTimeNs(startTime);
        if(timing) cout << "\t\tcalculateTrajectory: " << (time_end - time_start)/MICROSECOND << " ms" << endl;

        if( save_all_edges ) nearNode->temp_edge_ = thisEdge;

        // Check for validity vs edge collisions vs obstacles and
        // vs the time-dynamics of the robot and space
        time_start = GetTimeNs(startTime);
        bool edge_is_safe = !ExplicitEdgeCheck(C,thisEdge);
        time_end = GetTimeNs(startTime);
        if(timing) cout << "\t\tExplicitEdgeCheck: " << (time_end - time_start)/MICROSECOND << " ms" << endl;
        if(!edge_is_safe || !thisEdge->ValidMove()) {
            if(save_all_edges) nearNode->temp_edge_->dist_ = INF;
            listItem = listItem->child_; // iterate through list
            continue;
        }

        // Check if need to update rrtParent and rrt_parent_edge_
        if(new_node->rrt_LMC_ > nearNode->rrt_LMC_ + thisEdge->dist_) {
            // Found a potential better parent
            new_node->rrt_LMC_ = nearNode->rrt_LMC_ + thisEdge->dist_;
            /// This also takes care of some code in Extend I believe
            time_start = GetTimeNs(startTime);
            MakeParentOf(nearNode,new_node,thisEdge);
            time_end = GetTimeNs(startTime);
            if(timing) cout << "\t\tmakeParentOf: " << (time_end - time_start)/MICROSECOND << " ms" << endl;
        }

        listItem = listItem->child_; // iterate thorugh list
    }
}


/////////////////////// RRT# Functions ///////////////////////

void ResetNeighborIterator( shared_ptr<RrtNodeNeighborIterator> &It )
{ It->list_flag = 0; }

void MakeNeighborOf(shared_ptr<KDTreeNode> &new_neighbor,
                    shared_ptr<KDTreeNode> &node,
                    shared_ptr<Edge> &edge)
{
    node->rrt_neighbors_out_->JListPush( edge );
    edge->list_item_in_start_node_ = node->rrt_neighbors_out_->front_;

    new_neighbor->rrt_neighbors_in_->JListPush( edge );
    edge->list_item_in_end_node_ = new_neighbor->rrt_neighbors_in_->front_;
}

void MakeInitialOutNeighborOf(shared_ptr<KDTreeNode> &new_neighbor,
                              shared_ptr<KDTreeNode> &node,
                              shared_ptr<Edge> &edge)
{ node->initial_neighbor_list_out_->JListPush(edge); }

void MakeInitialInNeighborOf(shared_ptr<KDTreeNode> &new_neighbor,
                             shared_ptr<KDTreeNode> &node,
                             shared_ptr<Edge> &edge)
{ node->initial_neighbor_list_in_->JListPush(edge); }

void UpdateQueue(shared_ptr<Queue> &Q,
                  shared_ptr<KDTreeNode> &new_node,
                  shared_ptr<KDTreeNode> &root,
                  double hyper_ball_rad )
{
    RecalculateLMC( Q, new_node, root, hyper_ball_rad ); // internally ignores root
    if( Q->priority_queue->markedQ( new_node ) ) {
        Q->priority_queue->UpdateHeap( new_node );
        Q->priority_queue->RemoveFromHeap( new_node );
    }
    if( new_node->rrt_tree_cost_ != new_node->rrt_LMC_ ) {
        Q->priority_queue->AddToHeap( new_node );
    }
}

void ReduceInconsistency(shared_ptr<Queue> &Q,
                         shared_ptr<KDTreeNode> &goal_node,
                         double robot_rad,
                         shared_ptr<KDTreeNode> &root,
                         double hyper_ball_rad)
{
    shared_ptr<KDTreeNode> this_node;
    Q->priority_queue->TopHeap(this_node);
    while( Q->priority_queue->index_of_last_ > 0
           && (Q->priority_queue->lessThan(this_node, goal_node)
               || goal_node->rrt_LMC_ == INF
               || goal_node->rrt_tree_cost_ == INF
               || Q->priority_queue->marked(goal_node) ) ) {
        Q->priority_queue->PopHeap(this_node);

        // Update neighbors of thisNode if it has
        // changed more than change thresh
        if(this_node->rrt_tree_cost_ - this_node->rrt_LMC_ > Q->change_thresh) {
            RecalculateLMC( Q, this_node, root, hyper_ball_rad );
            Rewire( Q, this_node, root, hyper_ball_rad, Q->change_thresh );
        }
        this_node->rrt_tree_cost_ = this_node->rrt_LMC_;
        Q->priority_queue->TopHeap(this_node);
    }
}


/////////////////////// RRTx Functions ///////////////////////

void MarkOS( shared_ptr<KDTreeNode> &node )
{
    node->in_OS_queue_ = true;
}

void UnmarkOS( shared_ptr<KDTreeNode> &node )
{
    node->in_OS_queue_ = false;
}

bool MarkedOS( shared_ptr<KDTreeNode> node )
{
    return node->in_OS_queue_;
}

bool VerifyInQueue(shared_ptr<Queue> &Q, shared_ptr<KDTreeNode> &node)
{
    if( Q->priority_queue->markedQ(node) ) {
       return Q->priority_queue->UpdateHeap(node);
    } else {
       return Q->priority_queue->AddToHeap(node);
    }
}

bool VerifyInOSQueue(shared_ptr<Queue> &Q, shared_ptr<KDTreeNode> &node)
{
    if( Q->priority_queue->markedQ(node) ) {
        Q->priority_queue->UpdateHeap(node);
        Q->priority_queue->RemoveFromHeap(node);
    }
    if( !MarkedOS(node) ) {
        MarkOS(node);
        Q->obs_successors->JListPush(node);
    }
    return true;
}

void CullCurrentNeighbors(shared_ptr<KDTreeNode> &node, double hyper_ball_rad )
{
    // Remove outgoing edges from node that are now too long
    shared_ptr<JListNode> listItem = node->rrt_neighbors_out_->front_;
    shared_ptr<JListNode> nextItem;
    shared_ptr<Edge> neighborEdge;
    shared_ptr<KDTreeNode> neighborNode;
    while( listItem != listItem->child_ ) {
        nextItem = listItem->child_; // since we may remove listItem from list
        if( listItem->edge_->dist_ > hyper_ball_rad ) {
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

shared_ptr<JListNode> NextOutNeighbor(shared_ptr<RrtNodeNeighborIterator> &It)
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

shared_ptr<JListNode> NextInNeighbor(shared_ptr<RrtNodeNeighborIterator> &It)
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

void MakeParentOf(shared_ptr<KDTreeNode> &new_parent,
                  shared_ptr<KDTreeNode> &node,
                  shared_ptr<Edge> &edge)
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
            = Edge::Edge::NewEdge(edge->cspace_, edge->tree_, new_parent, node );
    backEdge->dist_ = INF;
    new_parent->successor_list_->JListPush( backEdge, INF );
    node->successor_list_item_in_parent_ = new_parent->successor_list_->front_;
}

bool RecalculateLMC(shared_ptr<Queue> &Q,
                    shared_ptr<KDTreeNode> &node,
                    shared_ptr<KDTreeNode> &root,
                    double hyper_ball_rad)
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
    CullCurrentNeighbors( node, hyper_ball_rad );

    // Get an iterator for this node's neighbors
    shared_ptr<RrtNodeNeighborIterator> thisNodeOutNeighbors
            = make_shared<RrtNodeNeighborIterator>(node);

    // Set the iterator to the first neighbor
    listItem = NextOutNeighbor( thisNodeOutNeighbors );

    while( listItem->key_ != -1.0 ) {
        neighborEdge = listItem->edge_;
        neighborNode = neighborEdge->end_node_;
        neighborDist = neighborEdge->dist_;
        nextItem = listItem->child_;

        if( MarkedOS(neighborNode) ) {
            // neighborNode is already in OS queue (orphaned) or unwired
            listItem = NextOutNeighbor( thisNodeOutNeighbors );
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

        listItem = NextOutNeighbor( thisNodeOutNeighbors );
    }

    if( newParentFound ) { // this node found a viable parent
        MakeParentOf(rrtParent, node, parentEdge);
    }
    return true;
}

bool Rewire(shared_ptr<Queue> &Q,
            shared_ptr<KDTreeNode> &node,
            shared_ptr<KDTreeNode> &root,
            double hyper_ball_rad, double change_thresh )
{
    // Only explicitly propogate changes if they are large enough
    double deltaCost = node->rrt_tree_cost_ - node->rrt_LMC_;
    if( deltaCost <= change_thresh ) {
        // Note that using simply "<" causes problems
        // Above note may be outdated
        // node.rrt_tree_cost_ = node.rrt_LMC_!!! Now happens after return
        return false;
    }

    // Remove outdated nodes from the current neighbors list
    CullCurrentNeighbors( node, hyper_ball_rad );

    // Get an iterator for this node's neighbors and iterate through list
    shared_ptr<RrtNodeNeighborIterator> thisNodeInNeighbors
            = make_shared<RrtNodeNeighborIterator>(node);
    shared_ptr<JListNode> listItem
            = NextInNeighbor( thisNodeInNeighbors );
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
            listItem = NextInNeighbor( thisNodeInNeighbors );
            continue;
        }

        neighborEdge = listItem->edge_;

        if( neighborNode->rrt_LMC_  > node->rrt_LMC_ + neighborEdge->dist_
                && (!neighborNode->rrt_parent_used_
                    || neighborNode->rrt_parent_edge_->end_node_ != node )
                && neighborEdge->ValidMove() ) {
            // neighborNode should use node as its parent (it might already)
            neighborNode->rrt_LMC_ = node->rrt_LMC_ + neighborEdge->dist_;
            MakeParentOf( node, neighborNode, neighborEdge);

            // If the reduction is great enough, then propogate
            // through the neighbor
            if( neighborNode->rrt_tree_cost_ - neighborNode->rrt_LMC_
                    > change_thresh ) {
                VerifyInQueue( Q, neighborNode );
            }
        }

        listItem = NextInNeighbor( thisNodeInNeighbors );
    }
    return true;
}

bool PropogateDescendants(shared_ptr<Queue> &Q,
                          shared_ptr<KDTree> Tree,
                          shared_ptr<RobotData> &Robot)
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
            VerifyInOSQueue( Q, successorNode ); // pushes to front_ of OS
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
        listItem = NextOutNeighbor( thisNodeOutNeighbors );
        while( listItem->key_ != -1.0 ) {
            neighborNode = listItem->edge_->end_node_;

            if( MarkedOS(neighborNode) ) {
                // neighborNode already in OS queue (orphaned) or unwired
                listItem = NextOutNeighbor( thisNodeOutNeighbors );
                continue;
            }

            // Otherwise, make sure that neighborNode is in normal queue
            neighborNode->rrt_tree_cost_ = INF; // node will be inserted with LMC
                                             // key and then guarenteed to
                                             // propogate cost forward since
                                     // useful nodes have rrt_LMC_ < rrt_tree_cost_
            VerifyInQueue( Q, neighborNode );

            listItem = NextOutNeighbor( thisNodeOutNeighbors );
        }

        // Add parent to the Q, unless it is in OS
        if( thisNode->rrt_parent_used_
                && !MarkedOS((thisNode->rrt_parent_edge_->end_node_)) ) {
            thisNode->kd_parent_->rrt_tree_cost_ = INF; // rrtParent = kd_parent_???
            VerifyInQueue( Q, thisNode->rrt_parent_edge_->end_node_ );
        }

        OS_list_item = OS_list_item->parent_;
    }

    // Third pass, remove all nodes from OS, unmark them, and
    // remove their connections to their parents. If one was the
    // robot's target then take appropriate measures
    while( Q->obs_successors->length_ > 0 ) {
        Q->obs_successors->JListPop(thisNode);
        UnmarkOS(thisNode);

        if( thisNode == Robot->next_move_target ) {
            Robot->current_move_invalid = true;
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

void AddOtherTimesToRoot(shared_ptr<ConfigSpace> &C,
                         shared_ptr<KDTree> &Tree,
                         shared_ptr<KDTreeNode> &goal,
                         shared_ptr<KDTreeNode> &root,
                         string search_type)
{
    double insertStep = 2.0;

    double lastTimeToInsert = goal->position_(2)
            - Tree->distanceFunction(root->position_,goal->position_)
            /C->robot_velocity_;
    double firstTimeToInsert = C->start_(2) + insertStep;
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
        thisEdge = Edge::NewEdge( C, Tree, newNode, previousNode );
        thisEdge->CalculateHoverTrajectory();

        if( search_type == "RRT*" ) {
            // Make this node the parent of the neighbor node
            newNode->rrt_parent_edge_ = thisEdge;
            newNode->rrt_parent_used_ = true;
        } else if( search_type == "RRT#" ) {
            // Make this node the parent of the neighbor node
            newNode->rrt_parent_edge_ = thisEdge;
            newNode->rrt_parent_used_ = true;
            MakeNeighborOf( newNode, previousNode, thisEdge );
        } else if( search_type == "RRTx" ) {
            MakeParentOf( previousNode, newNode, thisEdge);
            MakeInitialOutNeighborOf( previousNode, newNode, thisEdge );
            // Initial neighbor list edge
            MakeInitialInNeighborOf( newNode, previousNode, thisEdge );
        }

        // Make sure this edge is safe
        if( ExplicitEdgeCheck( C, thisEdge ) ) {
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
void FindNewTarget(shared_ptr<ConfigSpace> &C,
                   shared_ptr<KDTree> &Tree,
                   shared_ptr<RobotData> &Robot,
                   double hyper_ball_rad)
{
    Eigen::VectorXd robPose, nextPose;
    robPose = Robot->robot_pose;
    Robot->robot_edge_used = false;
    Robot->dist_along_robot_edge = 0.0;
    Robot->time_along_robot_edge = 0.0;
    nextPose = Robot->next_move_target->position_;

    // Move target has become invalid
    double searchBallRad
            = max(hyper_ball_rad, Tree->distanceFunction(robPose, nextPose));

    double maxSearchBallRad
            = Tree->distanceFunction(C->lower_bounds_, C->upper_bounds_);
    searchBallRad = min( searchBallRad, maxSearchBallRad );
    shared_ptr<JList> L = make_shared<JList>(true);
    Tree->KDFindWithinRange( L, searchBallRad, robPose );

    shared_ptr<KDTreeNode> dummyRobotNode
            = make_shared<KDTreeNode>(robPose);
    shared_ptr<Edge> edgeToBestNeighbor
            = Edge::NewEdge(C,Tree,dummyRobotNode,dummyRobotNode);

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

            thisEdge = Edge::NewEdge( C,Tree,dummyRobotNode, neighborNode );
            thisEdge->CalculateTrajectory();

            if( thisEdge->ValidMove()
                    && !ExplicitEdgeCheck(C,thisEdge) ) {
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
            Robot->next_move_target = bestNeighbor;
            Robot->distance_from_next_robot_pose_to_next_move_target = bestDistToNeighbor;
            Robot->current_move_invalid = false;
            // Found a valid move target

            Robot->robot_edge = edgeToBestNeighbor; /** this edge is empty **/
            Robot->robot_edge_used = true;

            if( C->space_has_time_ ) {
                Robot->time_along_robot_edge = 0.0;
                // note this is updated before robot moves
            } else {
                Robot->dist_along_robot_edge = 0.0;
                // note this is updated before robot moves
            }

            // Set moveGoal to be next_move_target
            // NOTE may want to actually insert a new node at the robot's
            // position_ and use that instead, since these "edges" created
            // between robot pose and R.next_move_target may be lengthy
            C->move_goal_->is_move_goal_ = false;
            C->move_goal_ = Robot->next_move_target;
            C->move_goal_->is_move_goal_ = true;
            break;
        }

        searchBallRad *= 2;
        if( searchBallRad > maxSearchBallRad ) {
            // Unable to find a valid move target so sample randomly
            shared_ptr<KDTreeNode> newNode = RandNodeDefault(C);
            double thisDist = Tree->distanceFunction(newNode->position_,
                                                     robPose);
            Edge::Saturate(
                        newNode->position_,
                        robPose, C->saturation_delta_, thisDist);
            Tree->KDInsert(newNode);
        }
        Tree->KDFindMoreWithinRange( L, searchBallRad, robPose );

    }
    Tree->EmptyRangeList(L); // cleanup
}
