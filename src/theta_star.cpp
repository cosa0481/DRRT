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

vector<Eigen::VectorXd> ThetaStar(shared_ptr<Queue> Q)
{
    cout << "Theta*\n------" << endl;
    Eigen::VectorXi wrap_vec(1);
    Eigen::VectorXd wrap_points_vec(1);
    wrap_vec(0) = 2;
    wrap_points_vec(0) = 2.0*PI;
    shared_ptr<KDTree> tree =
            make_shared<KDTree>(3,wrap_vec,wrap_points_vec);
    tree->SetDistanceFunction(DistFunc);

    shared_ptr<KDTreeNode> start = make_shared<KDTreeNode>(Q->cspace->start_);
    start->rrt_LMC_ = 0;
    shared_ptr<Edge> start_edge = Edge::NewEdge(Q->cspace,tree,start,start);
    start->rrt_parent_edge_ = start_edge;
    start->rrt_parent_used_ = true;
    tree->KDInsert(start);

    /// NEED TO BE ABLE TO APPLY THIS DYNAMICALLY
    // Here I'm just adding the obstacles manually since I know how many exist
    shared_ptr<ListNode> osnode;
    {
        lock_guard<mutex> lock(Q->cspace->cspace_mutex_);
        osnode = Q->cspace->obstacles_->front_;
        while(osnode->child_ != osnode) {
//            cout << "obstacle:\n" << osnode->obstacle_->position_ << endl;
            // below takes care of add loop in main
            osnode->obstacle_->obstacle_used_ = true;
            AddNewObstacle(tree,Q,osnode->obstacle_,tree->root);
            osnode = osnode->child_;
        }
    }

    cout << "Building K-D Tree of ConfigSpace" << endl;
    Eigen::MatrixX3d points;
    Eigen::Vector3d this_point;
    shared_ptr<KDTreeNode> new_node;
    double x_width, y_width, angle;
    x_width = Q->cspace->num_dimensions_*Q->cspace->width_(0);
    y_width = Q->cspace->num_dimensions_*Q->cspace->width_(1);
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
            new_node->rrt_LMC_
                = tree->distanceFunction(new_node->position_.head(2),
                                         start->position_.head(2));
            tree->KDInsert(new_node);      // this calls AddVizNode
            tree->RemoveVizNode(new_node); // not visualizing
        }
    }

    shared_ptr<KDTreeNode> this_node = make_shared<KDTreeNode>();
    shared_ptr<JList> node_list = make_shared<JList>(true); // uses KDTreeNodes
    shared_ptr<JListNode> this_item = make_shared<JListNode>();
    shared_ptr<KDTreeNode> near_node = make_shared<KDTreeNode>();

    // This is where the robot starts
    shared_ptr<KDTreeNode> goal = make_shared<KDTreeNode>();
    double x_start = 49;
    double y_start = 49;
    tree->GetNodeAt(Eigen::Vector3d(x_start,y_start,
                                    -PI+x_start/sqrt(x_start*x_start
                                                     +y_start*y_start)),
                    goal);
    goal->rrt_parent_edge_ = Edge::NewEdge(Q->cspace,tree,goal,goal);

    open_set = make_shared<BinaryHeap>(false); // Priority Queue
    open_set->AddToHeap(goal); // add starting position_
    closed_set = make_shared<JList>(true); // Uses KDTreeNodes

    cout << "Searching for Best Any-Angle Path" << endl;

    shared_ptr<KDTreeNode> node, end_node, min_neighbor;
    end_node = make_shared<KDTreeNode>(Eigen::Vector3d(-1,-1,-1));
    end_node->rrt_parent_edge_ = Edge::NewEdge(Q->cspace,tree,end_node,end_node);
    end_node->rrt_LMC_ = INF;
    while(open_set->index_of_last_ > 0) {
        open_set->PopHeap(node);

        if(node == start) {
            cout << "Found Any-Angle Path" << endl;
            vector<Eigen::VectorXd> path = GetPath(start);
            return path;
        }

        closed_set->JListPush(node);

        // Find eight neighbors around this node
        tree->KDFindWithinRange(node_list,2,node->position_);
        min_neighbor = make_shared<KDTreeNode>();
        min_neighbor->rrt_LMC_ = INF;

        // Iterate through neighbors
        this_item = node_list->front_;
        for(int i = 0; i < node_list->length_; i++) {
            near_node = this_item->node_;
            // If the neighbor is not in the closed set
            if(!closed_set->JListContains(near_node)) {
//                if(!open_set->marked(near_node)) {
//                    cout << "near_node not marked" << endl;
//                    near_node->rrt_LMC_ = INF;
//                    near_node->rrt_parent_used_ = false;
//                }
                UpdateVertex(Q,tree,node,near_node,min_neighbor);
            }
            this_item = this_item->child_;
        }

        if(min_neighbor->dist_ != -1) {
            end_node = min_neighbor;
            end_node->rrt_parent_used_ = true;

            if(open_set->marked(end_node)) {
                open_set->RemoveFromHeap(end_node);
            }
            open_set->AddToHeap(end_node);
        }

        tree->EmptyRangeList(node_list); // clean up
    }
    cout << "I'm here :(" << endl;
    exit(-1);
    Eigen::VectorXd null;
    null.setZero(0);
    vector<Eigen::VectorXd> null_vec;
    null_vec.push_back(null);
    return null_vec;
}

bool UpdateVertex(shared_ptr<Queue> Q,
                  shared_ptr<KDTree> Tree,
                  shared_ptr<KDTreeNode> &node,
                  shared_ptr<KDTreeNode> &neighbor,
                  shared_ptr<KDTreeNode> &min_neighbor)
{
    shared_ptr<Edge> this_edge;
    if(node->rrt_parent_used_) {
        this_edge = Edge::NewEdge(Q->cspace,Tree,
                                  node->rrt_parent_edge_->start_node_,neighbor);
        if(!LineCheck(Q->cspace,Tree,this_edge->start_node_,this_edge->end_node_)
                && neighbor->rrt_LMC_ < min_neighbor->rrt_LMC_) {
            min_neighbor = neighbor;
            min_neighbor->rrt_parent_edge_ = this_edge;
//            cout << "node:\n" << node->rrt_parent_edge_->start_node_->position_
//                 << "\nparent of:\n" << neighbor->position_ << endl;
            return true;
        }
    }
    this_edge = Edge::NewEdge(Q->cspace,Tree,node,neighbor);
    if(!LineCheck(Q->cspace,Tree,this_edge->start_node_,this_edge->end_node_)
            && neighbor->rrt_LMC_ < min_neighbor->rrt_LMC_) {
        min_neighbor = neighbor;
        min_neighbor->rrt_parent_edge_ = this_edge;
//        cout << "node:\n" << node->position_
//             << "\nparent of:\n" << neighbor->position_ << endl;
        return true;
    }
//    cout << "no update" << endl;
    return false;
}

vector<Eigen::VectorXd> GetPath(shared_ptr<KDTreeNode> &node)
{
    vector<Eigen::VectorXd> path;
    path.push_back(node->position_);
    if(node->rrt_parent_edge_->start_node_ != node->rrt_parent_edge_->end_node_) {
        vector<Eigen::VectorXd> rec_path
                = GetPath(node->rrt_parent_edge_->start_node_);
        for(int i = 0; i < rec_path.size(); i++) {
            path.push_back(rec_path.at(i));
        }
    }
    return path;
}
