#include <DRRT/theta_star.h>

shared_ptr<BinaryHeap> open_set;
shared_ptr<JList> closed_set;

double dist_func(Eigen::VectorXd a, Eigen::VectorXd b)
{
    Eigen::ArrayXd temp = a.head(2) - b.head(2);
    temp = temp*temp;
    return sqrt( temp.sum()
                 + pow( min( abs(a(2)-b(2)),
                                  min(a(2),b(2)) + 2.0*PI
                                    - max(a(2),b(2)) ), 2 ) );
}

vector<Eigen::VectorXd> theta_star(shared_ptr<CSpace> C)
{
    Eigen::VectorXi wrap_vec(1);
    Eigen::VectorXd wrap_points_vec(1);
    wrap_vec(0) = 2;
    wrap_points_vec(0) = 2.0*PI;
    shared_ptr<KDTree> Tree =
            make_shared<KDTree>(3,wrap_vec,wrap_points_vec);
    Tree->setDistanceFunction(dist_func);

    shared_ptr<KDTreeNode> start = make_shared<KDTreeNode>(C->start);
    start->rrtLMC = 0;
    shared_ptr<Edge> start_edge = Edge::newEdge(C,Tree,start,start);
    start->rrtParentEdge = start_edge;
    start->rrtParentUsed = true;
    Tree->kdInsert(start);

    /// INSERT ALL CSPACE POINTS SO THIS ALGORITHM CAN TRAVERSE TO FIND
    /// QUICK OPTIMAL PATH
    /// CURRENTLY WRITTEN FOR DUBINS MODEL
    /// SHOULD BE GENERALIZED
    cout << "Building K-D Tree of CSpace" << endl;
    Eigen::MatrixX3d points;
    Eigen::Vector3d this_point;
    shared_ptr<KDTreeNode> new_node;
    double xWidth, yWidth, angle;
    xWidth = C->d*C->width(0);
    yWidth = C->d*C->width(1);
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
            new_node->rrtLMC
                = Tree->distanceFunction(new_node->position,start->position);
            Tree->kdInsert(new_node);
            Tree->removeVizNode(new_node); // Not visualizing
        }
    }

    cout << "Linking Neighbors for Search" << endl;

    /// Need to make each node have eight neighbors surrounding it
    /// each node is at a grid point
    shared_ptr<KDTreeNode> this_node = make_shared<KDTreeNode>();
    shared_ptr<JList> node_list = make_shared<JList>(true); // uses KDTreeNodes
    shared_ptr<JListNode> this_item = make_shared<JListNode>();
    shared_ptr<KDTreeNode> near_node = make_shared<KDTreeNode>();
    shared_ptr<Edge> near_edge;
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
            Tree->getNodeAt(Eigen::Vector3d(i,j,angle), this_node);
            Tree->kdFindWithinRange(node_list,2,this_node->position);

            this_item = node_list->front;
            for( int k = 0; k < node_list->length; k++ ) {
                near_node = this_item->node;
                near_edge = Edge::newEdge(C,Tree,this_node,near_node);
                near_edge->calculateTrajectory();
                if( near_edge->validMove() && !explicitEdgeCheck(C,near_edge)) {
                    makeNeighborOf(near_node,this_node,near_edge);
                }
                this_item = this_item->child;
            }
            Tree->emptyRangeList(node_list);
//            if(this_node->position == Eigen::Vector3d(20,20,angle)) {
//                cout << "20,20 Out Neighbors: " << this_node << endl;
//                cout << this_node->position << endl;
//                this_node->rrtNeighborsOut->JlistPrint();
//            }
        }
    }

    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>();
    Tree->getNodeAt(Eigen::Vector3d(20,20,-PI+20/sqrt(20*20+20*20)),goal);

    open_set = make_shared<BinaryHeap>(false); // Priority Queue
    open_set->addToHeap(goal);
    closed_set = make_shared<JList>(true);

    cout << "Searching for Best Any-Angle Path" << endl;

    shared_ptr<KDTreeNode> node, neighbor_node;
    shared_ptr<JListNode> list_item, item;
    shared_ptr<Edge> neighbor_edge;
    while(open_set->indexOfLast > 0) {
        open_set->popHeap(node);
//        cout << "node: " << node << "\n" << node->position << endl;
        if(node == start) {
            cout << "Reached goal" << endl;
            return get_path(node);
        }

        closed_set->JlistPush(node);
        shared_ptr<RRTNodeNeighborIterator> NeighborIterator
                = make_shared<RRTNodeNeighborIterator>(node);
        list_item = nextOutNeighbor(NeighborIterator);
        while(list_item->key != -1.0) {
            neighbor_edge = list_item->edge;
            neighbor_node = neighbor_edge->endNode;

            if(!closed_set->JlistContains(neighbor_node,item)) {
                if(!open_set->marked(neighbor_node)) {
                    neighbor_node->rrtLMC = INF;
                    neighbor_node->rrtParentUsed = false;
                }
                update_vertex(C,Tree,node,neighbor_node);
            }
            list_item = nextOutNeighbor(NeighborIterator);
        }
    }
    Eigen::VectorXd null;
    null.setZero(0);
    vector<Eigen::VectorXd> null_vec;
    null_vec.push_back(null);
    return null_vec;
}

bool update_vertex(shared_ptr<CSpace> C,
                   shared_ptr<KDTree> Tree,
                   shared_ptr<KDTreeNode> &node,
                   shared_ptr<KDTreeNode> &neighbor)
{
//    cout << "update_vertex(\n" << node->position << "\n,\n" << neighbor->position << "\n)" << endl;
    shared_ptr<Edge> this_edge;
    if( node->rrtParentUsed ) {
        this_edge = Edge::newEdge(C,Tree,node->rrtParentEdge->endNode,neighbor);
        this_edge->calculateTrajectory();
        if(this_edge->validMove() && !explicitEdgeCheck(C,this_edge)) {
            if(neighbor->rrtLMC
                    > node->rrtParentEdge->endNode->rrtLMC + this_edge->dist) {
                neighbor->rrtLMC
                        = node->rrtParentEdge->endNode->rrtLMC + this_edge->dist;
                neighbor->rrtParentEdge = this_edge;
                neighbor->rrtParentUsed = true;
                if(open_set->marked(neighbor)) {
                    open_set->removeFromHeap(neighbor);
                }
                open_set->addToHeap(neighbor);
                return true;
            }
            return false;
        }
    }
    this_edge = Edge::newEdge(C,Tree,node,neighbor);
    this_edge->calculateTrajectory();
    if(node->rrtLMC + this_edge->dist < neighbor->rrtLMC) {
        neighbor->rrtLMC = node->rrtLMC + this_edge->dist;
        neighbor->rrtParentEdge = this_edge;
        neighbor->rrtParentUsed = true;
        if(open_set->marked(neighbor)) {
            open_set->removeFromHeap(neighbor);
        }
        open_set->addToHeap(neighbor);
        return true;
    }
    return false;
}

vector<Eigen::VectorXd> get_path(shared_ptr<KDTreeNode> &node)
{
//    cout << "get_path(\n" << node->position << "\n)" <<endl;
    vector<Eigen::VectorXd> path;
    path.push_back(node->position);
    if( node->rrtParentEdge->endNode != node) {
        vector<Eigen::VectorXd> rec_path
                = get_path(node->rrtParentEdge->endNode);
        for( int i = 0; i < rec_path.size(); i++ ) {
            path.push_back(rec_path.at(i));
        }
    }
    return path;
}
