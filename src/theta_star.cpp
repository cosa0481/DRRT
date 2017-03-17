#include <DRRT/theta_star.h>

shared_ptr<BinaryHeap> open_set;
shared_ptr<JList> closed_set;

using namespace std;

// This is the same as the main distance function
double dist_func(Eigen::VectorXd a, Eigen::VectorXd b)
{
    Eigen::ArrayXd temp = a - b;
    temp = temp*temp;
    return sqrt(temp.sum());
}

vector<Eigen::VectorXd> theta_star(shared_ptr<Queue> Q)
{
    Eigen::VectorXi wrap_vec(1);
    Eigen::VectorXd wrap_points_vec(1);
    wrap_vec(0) = 2;
    wrap_points_vec(0) = 2.0*PI;
    shared_ptr<KDTree> Tree =
            make_shared<KDTree>(3,wrap_vec,wrap_points_vec);
    Tree->setDistanceFunction(dist_func);

    shared_ptr<KDTreeNode> start = make_shared<KDTreeNode>(Q->S->start);
    start->rrtLMC = 0;
    shared_ptr<Edge> start_edge = Edge::newEdge(Q->S,Tree,start,start);
    start->rrtParentEdge = start_edge;
    start->rrtParentUsed = true;
    Tree->kdInsert(start);

    /// NEED TO BE ABLE TO APPLY THIS DYNAMICALY
    shared_ptr<ListNode> osnode;
    {
        lock_guard<mutex> lock(Q->S->cspace_mutex_);
        osnode = Q->S->obstacles->front_;
        osnode->obstacle_->obstacle_used_ = true;
        AddNewObstacle(Tree,Q,osnode->obstacle_,Tree->root);
    }


    /// INSERT ALL CSPACE POINTS SO THIS ALGORITHM CAN TRAVERSE TO FIND
    /// QUICK OPTIMAL PATH
    /// CURRENTLY WRITTEN FOR DUBINS MODEL
    /// SHOULD BE GENERALIZED
    cout << "Building K-D Tree of CSpace" << endl;
    Eigen::MatrixX3d points;
    Eigen::Vector3d this_point;
    shared_ptr<KDTreeNode> new_node;
    double xWidth, yWidth, angle;
    xWidth = Q->S->d*Q->S->width(0);
    yWidth = Q->S->d*Q->S->width(1);
    for( int i = 0; i < xWidth; i++ ) {
        for( int j = 0; j < yWidth; j++ ) {
            if( i == 0 && j == 0 ) {
                angle = -3*PI/4;
            } else if( i == 0 && j > 0 ) {
                // y axis, go down
                angle = -PI/2;
            } else if( j == 0 && i > 0 ) {
                // x axis, go left
                angle = PI;
            } else if( i == 0 && j < 0 ) {
                // -y axis, go up
                angle = PI/2;
            } else if( j == 0 && i < 0 ) {
                // -x axis, go right
                angle = 0;
            } else {
                angle = -PI + i/sqrt(i*i+j*j);
            }
            this_point(0) = i;
            this_point(1) = j;
            this_point(2) = angle;
            new_node = make_shared<KDTreeNode>(this_point);
            // Use this KDTreeNode variable to hold the cost
            // For now this is just EUCLIDEAN DISTANCE defined in dist_func
            new_node->rrtLMC
                = Tree->distanceFunction(new_node->position.head(2),
                                         start->position.head(2));
            Tree->kdInsert(new_node);      // this calls addVizNode
            Tree->removeVizNode(new_node); // not visualizing
        }
    }

//    cout << "Linking Neighbors for Search" << endl;

    /// Need to make each node have eight neighbors surrounding it
    /// each node is at a grid point
    shared_ptr<KDTreeNode> this_node = make_shared<KDTreeNode>();
    shared_ptr<JList> node_list = make_shared<JList>(true); // uses KDTreeNodes
    shared_ptr<JListNode> this_item = make_shared<JListNode>();
    shared_ptr<KDTreeNode> near_node = make_shared<KDTreeNode>();
    shared_ptr<Edge> near_edge;
//    for( int i = 0; i < xWidth; i++ ) {
//        for( int j = 0; j < yWidth; j++ ) {
//            if( i == 0 && j == 0 ) {
//                angle = -3*PI/4;
//            } else if( i == 0 && j > 0 ) {
//                // y axis, go down
//                angle = -PI/2;
//            } else if( j == 0 && i > 0 ) {
//                // x axis, go left
//                angle = PI;
//            } else if( i == 0 && j < 0 ) {
//                // -y axis, go up
//                angle = PI/2;
//            } else if( j == 0 && i < 0 ) {
//                // -x axis, go right
//                angle = 0;
//            } else {
//                angle = -PI + i/sqrt(i*i+j*j);
//            }
//            Tree->getNodeAt(Eigen::Vector3d(i,j,angle), this_node);
//            // This sholud get all eight neighbors of the node
//            // since the tree is built at grid points with spacing 1
//            Tree->kdFindWithinRange(node_list,2,this_node->position);

//            this_item = node_list->front;
//            for( int k = 0; k < node_list->length; k++ ) {
//                near_node = this_item->node;
//                near_edge = Edge::newEdge(Q->S,Tree,this_node,near_node);
//                makeNeighborOf(near_node,this_node,near_edge);
//                this_item = this_item->child;
//            }
//            Tree->emptyRangeList(node_list);
//        }
//    }

    // This is where the robot starts
    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>();
    Tree->getNodeAt(Eigen::Vector3d(20,20,-PI+20/sqrt(20*20+20*20)),goal);

    open_set = make_shared<BinaryHeap>(false); // Priority Queue
    open_set->addToHeap(goal); // add starting position
    closed_set = make_shared<JList>(true);

    cout << "Searching for Best Any-Angle Path" << endl;

    shared_ptr<KDTreeNode> node, end_node, min_neighbor;
    shared_ptr<JListNode> list_item, item;
    shared_ptr<Edge> neighbor_edge, min_edge;
    end_node = make_shared<KDTreeNode>(Eigen::Vector3d(-1,-1,-1));
    end_node->rrtParentEdge = Edge::newEdge(Q->S,Tree,end_node,end_node);
    end_node->rrtLMC = INF;
    while(open_set->indexOfLast > 0) {
        open_set->popHeap(node);
//        cout << "open_set->popHeap:\n" << node->position << endl;

        if(node == start) {
            cout << "Reached goal" << endl;
            vector<Eigen::VectorXd> path = get_path(end_node->rrtParentEdge->startNode);
            path.insert(path.begin(), start->position);
            return path;
        }

        closed_set->JlistPush(node);

        // Find eight neighbors around this node
        Tree->kdFindWithinRange(node_list,2,node->position);
        min_neighbor = make_shared<KDTreeNode>();
        min_neighbor->rrtLMC = INF;

        // Iterate through neighbors
        this_item = node_list->front;
        for(int i = 0; i < node_list->length; i++) {
            near_node = this_item->node;
//            cout << "near_node:\n" << near_node->position << endl;
            if(!closed_set->JlistContains(near_node,item)) {
//                if(!open_set->marked(near_node)) {
//                    cout << "near_node not marked" << endl;
//                    near_node->rrtLMC = INF;
//                    near_node->rrtParentUsed = false;
//                }
                // should enforce that rrtParentEdge is longest line of sight
                // to last best node
                update_vertex(Q,Tree,node,near_node,min_neighbor);
            }
            this_item = this_item->child;
        }
        end_node = min_neighbor;
        end_node->rrtParentUsed = true;
//        cout << "new_end:\n" << end_node->rrtParentEdge->startNode->position << "\n--\n"
//                             << end_node->rrtParentEdge->endNode->position << endl;

        if(open_set->marked(end_node)) {
            open_set->removeFromHeap(end_node);
        }
        open_set->addToHeap(end_node);

        Tree->emptyRangeList(node_list); // clean up

//        shared_ptr<RRTNodeNeighborIterator> NeighborIterator
//                = make_shared<RRTNodeNeighborIterator>(node);
//        list_item = nextOutNeighbor(NeighborIterator);
//        while(list_item->key != -1.0) {
//            neighbor_edge = list_item->edge;
//            neighbor_node = neighbor_edge->endNode;

//            if(!closed_set->JlistContains(neighbor_node,item)) {
//                if(!open_set->marked(neighbor_node)) {
//                    neighbor_node->rrtLMC = INF;
//                    neighbor_node->rrtParentUsed = false;
//                }
//                update_vertex(Q->S,Tree,node,neighbor_node);
//            }
//            list_item = nextOutNeighbor(NeighborIterator);
//        }
    }
    Eigen::VectorXd null;
    null.setZero(0);
    vector<Eigen::VectorXd> null_vec;
    null_vec.push_back(null);
    return null_vec;
}

bool update_vertex(shared_ptr<Queue> Q,
                   shared_ptr<KDTree> Tree,
                   shared_ptr<KDTreeNode> &node,
                   shared_ptr<KDTreeNode> &neighbor,
                   shared_ptr<KDTreeNode> &min_neighbor)
{
//    cout << "UpdateVertex" << endl;
    shared_ptr<Edge> this_edge;
//    cout << "neighborLMC: " << neighbor->rrtLMC << endl;
//    cout << "minNeighborLMC: " << min_neighbor->rrtLMC << endl;
    if(node->rrtParentUsed) {
        this_edge = Edge::newEdge(Q->S,Tree,node->rrtParentEdge->startNode,neighbor);
        if(!LineCheck(Q->S,Tree,this_edge->startNode,this_edge->endNode)
                && neighbor->rrtLMC < min_neighbor->rrtLMC) {
            min_neighbor = neighbor;
            min_neighbor->rrtParentEdge = this_edge;
            return true;
        }
    }
    this_edge = Edge::newEdge(Q->S,Tree,node,neighbor);
    if(!LineCheck(Q->S,Tree,this_edge->startNode,this_edge->endNode)
            && neighbor->rrtLMC < min_neighbor->rrtLMC) {
        min_neighbor = neighbor;
        min_neighbor->rrtParentEdge = this_edge;
        return true;
    }
    return false;


//    if( node->rrtParentUsed ) {
//        this_edge = Edge::newEdge(Q->S,Tree,node->rrtParentEdge->endNode,neighbor);
//        this_edge->calculateTrajectory();
//        if(this_edge->ValidMove() && !ExplicitEdgeCheck(Q->S,this_edge)) {
//            if(neighbor->rrtLMC
//                    > node->rrtParentEdge->endNode->rrtLMC + this_edge->dist) {
//                neighbor->rrtLMC
//                        = node->rrtParentEdge->endNode->rrtLMC + this_edge->dist;
//                neighbor->rrtParentEdge = this_edge;
//                neighbor->rrtParentUsed = true;
//                if(open_set->marked(neighbor)) {
//                    open_set->removeFromHeap(neighbor);
//                }
//                open_set->addToHeap(neighbor);
//                return true;
//            }
//            return false;
//        }
//    }
//    this_edge = Edge::newEdge(Q->S,Tree,node,neighbor);
//    this_edge->calculateTrajectory();
//    if(node->rrtLMC + this_edge->dist < neighbor->rrtLMC) {
//        neighbor->rrtLMC = node->rrtLMC + this_edge->dist;
//        neighbor->rrtParentEdge = this_edge;
//        neighbor->rrtParentUsed = true;
//        if(open_set->marked(neighbor)) {
//            open_set->removeFromHeap(neighbor);
//        }
//        open_set->addToHeap(neighbor);
//        return true;
//    }
//    return false;
}

vector<Eigen::VectorXd> get_path(shared_ptr<KDTreeNode> &node)
{
    cout << "get_path(\n" << node->position << "\n)" <<endl;
    vector<Eigen::VectorXd> path;
    path.push_back(node->position);
//    cout << "parent: "<< node->rrtParentEdge->endNode->position << endl;
    if( node->rrtParentEdge->endNode != node) {
        vector<Eigen::VectorXd> rec_path
                = get_path(node->rrtParentEdge->endNode);
        for( int i = 0; i < rec_path.size(); i++ ) {
            path.push_back(rec_path.at(i));
        }
    }
    return path;
}
