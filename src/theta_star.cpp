#include <DRRT/theta_star.h>

using namespace std;

// Open and closed sets for Theta*
shared_ptr<BinaryHeap> open_set;
shared_ptr<JList> closed_set;

// Euclidean distance
double DistFunc(Eigen::VectorXd a, Eigen::VectorXd b)
{
    Eigen::ArrayXd temp = a - b;
    temp = temp*temp;
    return sqrt(temp.sum());
}

vector<Eigen::VectorXd> ThetaStar(shared_ptr<Queue> queue)
{

    Eigen::VectorXi wrap_vec(1);
    Eigen::VectorXd wrap_points_vec(1);
    wrap_vec(0) = 2;
    wrap_points_vec(0) = 2.0*PI;
    shared_ptr<KDTree> tree =
            make_shared<KDTree>(3,wrap_vec,wrap_points_vec);
    tree->setDistanceFunction(DistFunc);

    shared_ptr<KDTreeNode> start = make_shared<KDTreeNode>(queue->S->start);
    start->rrtLMC = 0;
    shared_ptr<Edge> start_edge = Edge::newEdge(queue->S,tree,start,start);
    start->rrtParentEdge = start_edge;
    start->rrtParentUsed = true;
    tree->kdInsert(start);

    /// NEED TO BE ABLE TO APPLY THIS DYNAMICALY
    // Here I'm just adding the obstacles manually since I know how many exist
    shared_ptr<ListNode> osnode;
    {
        lock_guard<mutex> lock(queue->S->cspace_mutex_);
        osnode = queue->S->obstacles->front_;
        // below takes care of add loop in main
        osnode->obstacle_->obstacle_used_ = true;
        AddNewObstacle(tree,queue,osnode->obstacle_,tree->root);
    }

    cout << "Building K-D Tree of CSpace" << endl;
    Eigen::MatrixX3d points;
    Eigen::Vector3d this_point;
    shared_ptr<KDTreeNode> new_node;
    double x_width, y_width, angle;
    x_width = queue->S->d*queue->S->width(0);
    y_width = queue->S->d*queue->S->width(1);
    for( int i = 0; i < x_width; i++ ) {
        for( int j = 0; j < y_width; j++ ) {
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
            new_node->rrtLMC
                = tree->distanceFunction(new_node->position.head(2),
                                         start->position.head(2));
            tree->kdInsert(new_node);      // this calls addVizNode
            tree->removeVizNode(new_node); // not visualizing
        }
    }

    /// Need to make each node have eight neighbors surrounding it
    /// each node is at a grid point
    shared_ptr<KDTreeNode> this_node = make_shared<KDTreeNode>();
    shared_ptr<JList> node_list = make_shared<JList>(true); // uses KDTreeNodes
    shared_ptr<JListNode> this_item = make_shared<JListNode>();
    shared_ptr<KDTreeNode> near_node = make_shared<KDTreeNode>();

    // This is where the robot starts
    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>();
    tree->getNodeAt(Eigen::Vector3d(20,20,-PI+20/sqrt(20*20+20*20)),goal);

    open_set = make_shared<BinaryHeap>(false); // Priority Queue
    open_set->addToHeap(goal); // add starting position
    closed_set = make_shared<JList>(true); // Uses KDTreeNodes

    cout << "Searching for Best Any-Angle Path" << endl;

    shared_ptr<KDTreeNode> node, end_node, min_neighbor;
    shared_ptr<JListNode> item;
    end_node = make_shared<KDTreeNode>(Eigen::Vector3d(-1,-1,-1));
    end_node->rrtParentEdge = Edge::newEdge(queue->S,tree,end_node,end_node);
    end_node->rrtLMC = INF;
    while(open_set->indexOfLast > 0) {
        open_set->popHeap(node);

        if(node == start) {
            cout << "Found Any-Angle Path" << endl;
            vector<Eigen::VectorXd> path = GetPath(end_node->rrtParentEdge->startNode);
            path.insert(path.begin(), start->position);
            return path;
        }

        closed_set->JlistPush(node);

        // Find eight neighbors around this node
        tree->kdFindWithinRange(node_list,2,node->position);
        min_neighbor = make_shared<KDTreeNode>();
        min_neighbor->rrtLMC = INF;

        // Iterate through neighbors
        this_item = node_list->front;
        for(int i = 0; i < node_list->length; i++) {
            near_node = this_item->node;
            // If the neighbor is not in the closed set
            if(!closed_set->JlistContains(near_node,item)) {
//                if(!open_set->marked(near_node)) {
//                    cout << "near_node not marked" << endl;
//                    near_node->rrtLMC = INF;
//                    near_node->rrtParentUsed = false;
//                }
                UpdateVertex(queue,tree,node,near_node,min_neighbor);
            }
            this_item = this_item->child;
        }
        end_node = min_neighbor;
        end_node->rrtParentUsed = true;

        if(open_set->marked(end_node)) {
            open_set->removeFromHeap(end_node);
        }
        open_set->addToHeap(end_node);

        tree->emptyRangeList(node_list); // clean up
    }
    Eigen::VectorXd null;
    null.setZero(0);
    vector<Eigen::VectorXd> null_vec;
    null_vec.push_back(null);
    return null_vec;
}

bool UpdateVertex(shared_ptr<Queue> queue,
                   shared_ptr<KDTree> tree,
                   shared_ptr<KDTreeNode> &node,
                   shared_ptr<KDTreeNode> &neighbor,
                   shared_ptr<KDTreeNode> &min_neighbor)
{
    shared_ptr<Edge> this_edge;
    if(node->rrtParentUsed) {
        this_edge = Edge::newEdge(queue->S,tree,
                                  node->rrtParentEdge->startNode,neighbor);
        if(!LineCheck(queue->S,tree,this_edge->startNode,this_edge->endNode)
                && neighbor->rrtLMC < min_neighbor->rrtLMC) {
            min_neighbor = neighbor;
            min_neighbor->rrtParentEdge = this_edge;
            return true;
        }
    }
    this_edge = Edge::newEdge(queue->S,tree,node,neighbor);
    if(!LineCheck(queue->S,tree,this_edge->startNode,this_edge->endNode)
            && neighbor->rrtLMC < min_neighbor->rrtLMC) {
        min_neighbor = neighbor;
        min_neighbor->rrtParentEdge = this_edge;
        return true;
    }
    return false;
}

vector<Eigen::VectorXd> GetPath(shared_ptr<KDTreeNode> &node)
{
    vector<Eigen::VectorXd> path;
    path.push_back(node->position);
    if(node->rrtParentEdge->endNode != node) {
        vector<Eigen::VectorXd> rec_path
                = GetPath(node->rrtParentEdge->endNode);
        for(int i = 0; i < rec_path.size(); i++) {
            path.push_back(rec_path.at(i));
        }
    }
    return path;
}
