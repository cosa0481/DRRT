#include <DRRT/theta_star.h>
#include <DRRT/drrt.h>
#include "../src/list.cpp"

using namespace std;

Heap_ptr open_set;
KdnodeList_ptr closed_set;

double ThetaStarDistance(Eigen::VectorXd a, Eigen::VectorXd b)
{
    Eigen::ArrayXd temp = a - b;
    temp = temp*temp;
    return std::sqrt(temp.sum());
}

vector<double> PathToThetas(std::vector<Eigen::VectorXd> path)
{
    vector<double> thetas;
    double angle;
    Eigen::VectorXd curr_point, prev_point;
    for(int i = 0; i < path.size(); i++) {
        if(i == 0) continue;
        curr_point = path.at(i);
        prev_point = path.at(i - 1);
        angle = std::atan2(prev_point(1) - curr_point(1),
                           prev_point(0) - curr_point(0));
        thetas.push_back(angle);
    }
    return thetas;
}

vector<Eigen::VectorXd> GetPath(Kdnode_ptr node)
{
    vector<Eigen::VectorXd> path;
    path.push_back(node->GetPosition());
    Edge_ptr rrt_parent_edge = Edge::NewEdge();
    node->GetRrtParentEdge(rrt_parent_edge);
    if(rrt_parent_edge->GetStart() != rrt_parent_edge->GetEnd()) {
        vector<Eigen::VectorXd> rec_path = GetPath(rrt_parent_edge->GetStart());
        for(int i = 0; i < rec_path.size(); i++)
            path.push_back(rec_path.at(i));
    }
    return path;
}

bool UpdateVertex(CSpace_ptr cspace, Kdnode_ptr &node, Kdnode_ptr &neighbor, Kdnode_ptr &min_neighbor)
{
    Edge_ptr edge;
    if(node->RrtParentExist())
    {
        Kdnode_ptr current = node;
        while(current->RrtParentExist()) {
            Edge_ptr rrt_parent_edge = Edge::NewEdge();
            current->GetRrtParentEdge(rrt_parent_edge);
            current = rrt_parent_edge->GetStart();
            edge = Edge::NewEdge(current, neighbor);
            if(!LineCheck(edge->GetStart(), edge->GetEnd(), cspace)
                    && neighbor->GetCost() < min_neighbor->GetCost()) {
                min_neighbor = neighbor;
                min_neighbor->SetRrtParentEdge(edge);
                min_neighbor->SetRrtParentExist(true);
                return true;
            }
        }
    }

    edge = Edge::NewEdge(node, neighbor);
    if(!LineCheck(edge->GetStart(), edge->GetEnd(), cspace)
            && neighbor->GetCost() < min_neighbor->GetCost()) {
        min_neighbor = neighbor;
        min_neighbor->SetRrtParentEdge(edge);
        return true;
    }

    return false;
}

vector<Eigen::VectorXd> ThetaStar(CSpace_ptr cspace)
{
    // Create a K-D Tree
    Eigen::VectorXi wrap_vec(1);
    Eigen::VectorXd wrap_points_vec(1);
    wrap_vec(0) = 2;
    wrap_points_vec(0) = 2.0*PI;
    KdTree_ptr kdtree = make_shared<KdTree>(3, wrap_vec, wrap_points_vec);

    // Add ending point of Theta* as root
    Kdnode_ptr start = make_shared<Kdnode>(cspace->start_);
    start->SetLmc(0.0);
    start->SetCost(0.0);
    Edge_ptr start_edge = Edge::NewEdge(start, start);
    start->SetRrtParentEdge(start_edge);
    start->SetRrtParentExist(true);
    kdtree->Insert(start);

    // Define Theta* starting point
    Kdnode_ptr goal = make_shared<Kdnode>();
    double x_start = cspace->goal_node_->GetPosition()(0);
    double y_start = cspace->goal_node_->GetPosition()(1);

    // Build K-D Tree of nodes in a uniform grid
    Eigen::MatrixX3d points;
    Eigen::Vector3d point;
    Kdnode_ptr new_node;
    double x_width, y_width, angle;
    x_width = 20;
    y_width = 20;
    for(int i = 0; i < x_width; i++) {
        for(int j = 0; j < y_width; j++) {

            // Aim each point at the goal
            if(i == 0 && j == 0)
                angle = -3*PI/4;
            else if(i == 0 && j > 0)
                angle = -PI/2;
            else if(i > 0 && j == 0)
                angle = PI;
            else if(i == 0 && j < 0)
                angle = PI/2;
            else if(i < 0 && j == 0)
                angle = 0;
            else
                angle = -PI + i/std::sqrt(i*i + j*j);

            point(0) = i;
            point(1) = j;
            point(2) = angle;
            new_node = make_shared<Kdnode>(point);
            new_node->SetCost(ThetaStarDistance(new_node->GetPosition().head(2),
                                                start->GetPosition().head(2)));

            kdtree->Insert(new_node);
            // Do not visualize these points
            {
                lockguard lock(cspace->mutex_);
                KdnodeListNode_ptr temp_listnode = make_shared<KdnodeListNode>();
                cspace->new_nodes_->Pop(temp_listnode);
            }
        }
    }

    // Deactivate nodes within obstacles with AddObstacle
    ObstacleListNode_ptr obstacle_listnode;
    {
        lockguard lock(cspace->mutex_);
        obstacle_listnode = cspace->obstacles_->GetFront();
        while(!obstacle_listnode->IsEmpty()) {
            Obstacle_ptr obstacle = make_shared<Obstacle>();
            obstacle_listnode->GetData(obstacle);
            obstacle->AddObstacle();
            obstacle_listnode = obstacle_listnode->GetChild();
        }
    }

    RangeList_ptr node_list = make_shared<RangeList>();
    RangeListNode_ptr node_item = make_shared<RangeListNode>();
    Kdnode_ptr near_node = make_shared<Kdnode>();

    kdtree->GetNodeAt(goal, Eigen::Vector3d(x_start, y_start,
                                            -PI + x_start/std::sqrt(x_start*x_start
                                                                    + y_start*y_start)));
    goal->SetRrtParentEdge(Edge::NewEdge(goal, goal));

    // Initialize open set with Theta* starting position
    open_set = make_shared<Heap>(0); // 0 -> min heap
    HeapNode_ptr goal_heapnode = make_shared<HeapNode>(goal);
    open_set->Add(goal_heapnode);

    // Initialize closed set
    closed_set = make_shared<KdnodeList>();

    Kdnode_ptr node, end_node, min_neighbor;
    end_node = make_shared<Kdnode>(Eigen::Vector3d(-1,-1,-1));
    end_node->SetRrtParentEdge(Edge::NewEdge(end_node, end_node));
    end_node->SetCost(INF);

    while(open_set->GetIndexOfLast() > 0)
    {
        if(node == start)
            return GetPath(node);

        KdnodeListNode_ptr node_listnode = make_shared<KdnodeListNode>(node);
        closed_set->Push(node_listnode);

        // Find eight neighbors around node
        node_list = kdtree->FindWithinRange(2, node->GetPosition());  // range = 2
        min_neighbor = make_shared<Kdnode>();
        min_neighbor->SetCost(INF);

        // Iterate through the eight neighbors
        node_item = node_list->GetFront();
        for(int i = 0; i < node_list->GetLength(); i++ ) {
            node_item->GetData(near_node);
            KdnodeListNode_ptr this_item = make_shared<KdnodeListNode>(near_node);
            if(!closed_set->Contains(this_item))
                UpdateVertex(cspace, node, near_node, min_neighbor);
            node_item = node_item->GetChild();
        }

        if(min_neighbor->GetCost() != INF) {
            end_node = min_neighbor;
            end_node->SetRrtParentExist(true);

            HeapNode_ptr end_node_heapnode = make_shared<HeapNode>(end_node);

            if(end_node->InPriorityQueue()) {
                open_set->Remove(end_node_heapnode);
            }
            open_set->Add(end_node_heapnode);
        }

        kdtree->EmptyRangeList(node_list);
    }

    cout << "Theta* didn't find a path :(" << endl;
    exit(-2);
}
