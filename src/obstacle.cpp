#include <DRRT/data_structures.h>
#include <DRRT/obstacle_listnode.h>
#include <DRRT/collision_detection.h>

void Obstacle::UpdateObstacles()
{

}

bool Obstacle::MoveObstacles(std::shared_ptr<ConfigSpace> &cspace)
{
    lockguard lock(cspace->cspace_mutex_);
    ObstacleListNode_ptr obstacle_node = cspace->obstacles_->GetFront();
    if(obstacle_node->IsEmpty() || !obstacle_node->InList()) return false;
    Obstacle_ptr obstacle;
    bool moved = false;
    for(int i = 0; i < cspace->obstacles_->GetLength(); i++) {
        bool obs_moved = false;
        obstacle_node->GetData(obstacle);
        obs_moved = obstacle->MoveObstacle();
        obstacle_node = obstacle_node->GetChild();
        if(obs_moved) moved = true;
    }
    return moved;
}

bool Obstacle::MoveObstacle()
{
    double now = GetTimeNs(cspace->start_time_)/NSPS;  // Current time in seconds
    if(current_path_idx_ < path_times_.size()
            && path_times_(current_path_idx_) < now) {
        origin_ = path_.row(current_path_idx_);
        current_path_idx_++;
        return true;
    }
    return false;
}

// TODO: Understand this function
void Obstacle::AddObstacle(KdTree_ptr tree)
{
    KdnodeList_ptr node_list = FindPointsInConflictWithObstacle(tree, GetSharedPointer());

    KdnodeListNode_ptr node_listnode;
    Kdnode_ptr node;
    while(node_list->GetLength() > 0) {
        node_list->Pop(node_listnode);
        node_listnode->GetData(node);

        RrtNodeNeighborIterator_ptr out_neighbors = std::make_shared<RrtNodeNeighborIterator>(node);
        EdgeListNode_ptr list_item = NextOutNeighbor(out_neighbors);
        EdgeListNode_ptr next_item;
        Edge_ptr neighbor_edge;
        while(!list_item->IsEmpty()) {
            list_item->GetData(neighbor_edge);
            next_item = NextOutNeighbor(out_neighbors);
            if(EdgeCheck(cspace, neighbor_edge)) {
                neighbor_edge->SetDist(INF);
            }
            list_item = next_item;
        }

        Edge_ptr parent_edge;
        if(node->RrtParentExist()) {
            node->GetRrtParentEdge(parent_edge);
            if(EdgeCheck(GetSharedPointer(), parent_edge)) {
                EdgeListNode_ptr successor_in_parent = parent_edge->GetEnd()->GetSuccessorInParent();
                parent_edge->GetEnd()->GetSuccessorList()->Remove(successor_in_parent);
                if(parent_edge->GetEnd()->GetSuccessorInParent()->InList()) {
                    if(DEBUG) std::cout << "ERROR: successor in parent not removed from successor list" << std::endl;
                }

                parent_edge->SetEnd(node);
                parent_edge->SetDist(INF);
                node->SetRrtParentExist(false);

                // TODO: Add node to the priority queue for rewiring
                std::cout << "TODO" << std::endl;
                exit(-1);
            }
        }
    }
    node_list->Empty();
    is_used_ = true;
}

// TODO: Understand this function
void Obstacle::RemoveObstacle(KdTree_ptr tree)
{
    // Find nodes in conflict with this obstacle and add them to the priority queue for rewiring
    KdnodeList_ptr node_list = FindPointsInConflictWithObstacle(tree, GetSharedPointer());

    KdnodeListNode_ptr node_listnode;
    Kdnode_ptr node;
    while(node_list->GetLength() > 0) {
        node_list->Pop(node_listnode);
        node_listnode->GetData(node);

        // Check this node's out neighbors for blocks by the obstacle
        RrtNodeNeighborIterator_ptr out_neighbors = std::make_shared<RrtNodeNeighborIterator>(node);
        EdgeListNode_ptr list_item = NextOutNeighbor(out_neighbors);
        EdgeListNode_ptr next_item;
        Edge_ptr neighbor_edge;
        Kdnode_ptr neighbor_node;
        bool conflicts_with_other_obs = false;
        bool neighbors_were_blocked = false;
        while(!list_item->IsEmpty()) {
            list_item->GetData(neighbor_edge);
            neighbor_node = neighbor_edge->GetEnd();
            next_item = NextOutNeighbor(out_neighbors);
            // Check if edge in collision with this obstacle
            if(EdgeCheck(GetSharedPointer(), neighbor_edge)) {
                // If so, check for collision with other obstacles
                lockguard lock(cspace->cspace_mutex_);
                ObstacleListNode_ptr obs_listnode = cspace->obstacles_->GetFront();
                Obstacle_ptr other_obs;
                while(obs_listnode != obs_listnode->GetChild()) {
                    obs_listnode->GetData(other_obs);
                    if(other_obs != GetSharedPointer()
                            && other_obs->IsUsed()
                            && other_obs->GetStartTime() <= GetTimeNs(cspace->start_time_)
                            && GetTimeNs(cspace->start_time_) <= (other_obs->GetStartTime()
                                                                  + other_obs->GetLifeTime())) {
                        if(EdgeCheck(other_obs, neighbor_edge)) {
                            conflicts_with_other_obs = true;
                            break;
                        }
                    }
                    obs_listnode = obs_listnode->GetChild();
                }
                if(!conflicts_with_other_obs) {
                    // Reset edge distance to actual cost
                    neighbor_edge->SetDist(DistanceFunction(neighbor_edge->GetStart()->GetPosition(),
                                                            neighbor_edge->GetEnd()->GetPosition()));
                    neighbors_were_blocked = true;
                }
            }
            list_item = next_item;
        }
        if(neighbors_were_blocked) {
            // NOTE: Below also checked LMC != tree_cost
           if(node->GetCost() < cspace->move_goal_->GetCost())
               // TODO: Add node to the priority queue for rewiring
               std::cout << "TODO" << std::endl;
           exit(-1);
        }
    }
    node_list->Empty();
    is_used_ = false;
}

void Obstacle::AddToCSpace()
{
    lockguard lock(cspace->cspace_mutex_);
    Obstacle_ptr obstacle = GetSharedPointer();
    ObstacleListNode_ptr obstacle_listnode = std::make_shared<ObstacleListNode>(obstacle);
    cspace->obstacles_->Push(obstacle_listnode);
}

// For reading polygons from a file
void Obstacle::ReadObstaclesFromFile(std::string obs_file, std::shared_ptr<ConfigSpace> cspace)
{
    ifstream read_stream;
    string line, substring;
    stringstream line_stream;
    int num_polygons = 0;
    int num_points;
    int num_moves;
    read_stream.open(obs_file);
    if(read_stream.is_open()) {
        std::cout << "Obstacle File: " << obs_file << std::endl;
        // Get number of obstacles in file
        getline(read_stream, line);
        num_polygons = stoi(line);
        if(DEBUG) std::cout << "Number of polygons: " << num_polygons << std::endl;
        line = "";
        for(int i = 0; i < num_polygons; i++) {
            // Read through first delimiter line
            getline(read_stream, line);
            line = "";

            Eigen::Vector3d origin;
            getline(read_stream, line);
            line_stream = stringstream(line);
            getline(line_stream, substring, ',');
            origin(0) = stod(substring);
            getline(line_stream, substring);
            origin(1) = stod(substring);
            origin(2) = 0.0;
            substring = "";
            line = "";

            // Get number of moves
            getline(read_stream, line);
            num_moves = stoi(line);
            Eigen::MatrixXd path;
            Eigen::VectorXd path_times;
            path.resize(num_moves, NUM_DIM);
            path_times.resize(num_moves);
            for(int j = 0; j < num_moves; j++) {
                getline(read_stream, line);
                path_times(j) = stod(line);
                line = "";
            }
            Eigen::Vector3d new_origin;
            for(int k = 0; k < num_moves; k++) {
                getline(read_stream, line);
                line_stream = stringstream(line);
                getline(line_stream, substring, ',');
                new_origin(0) = stod(substring);
                getline(line_stream, substring);
                new_origin(1) = stod(substring);
                substring = "";
                new_origin(2) = 0.0;
                path.row(k)(0) = new_origin(0);
                path.row(k)(1) = new_origin(1);
                path.row(k)(2) = new_origin(2);
            }

            // Get number of points in obstacle polygon
            getline(read_stream, line);
            num_points = stoi(line);
            line = "";
            Eigen::MatrixXd polygon;
            if(num_points > 0) polygon.resize(num_points, 2);
            for(int p = 0; p < num_points; p++) {
                getline(read_stream, line);
                line_stream = stringstream(line);
                getline(line_stream, substring, ',');
                polygon.row(p)(0) = stod(substring);
                getline(line_stream, substring);
                polygon.row(p)(1) = stod(substring);
                substring = "";
                line = "";
            }

            Obstacle_ptr new_obstacle = std::make_shared<Obstacle>(3, origin, polygon, path, path_times, cspace);

            // Add new obstacle to Bullet for collision detection
            std::shared_ptr<btCollisionObject> coll_obj = std::make_shared<btCollisionObject>();
            std::shared_ptr<btConvexHullShape> coll_shape = std::make_shared<btConvexHullShape>();
            Eigen::MatrixX2d shape = new_obstacle->GetShape().GetRegion();
            for(int q = 0; q < shape.rows(); q++) {
                coll_shape->addPoint(
                            btVector3((btScalar) shape(q,0),
                                      (btScalar) shape(q,1),
                                      (btScalar) -0.1));
                coll_shape->addPoint(
                            btVector3((btScalar) shape(q,0),
                                      (btScalar) shape(q,1),
                                      (btScalar) 0.1));
            }
            coll_obj->setCollisionShape(coll_shape.get());
            cspace->bt_collision_world_->addCollisionObject(coll_obj.get());
            new_obstacle->SetCollisionObject(coll_obj);
            new_obstacle->SetCollisionShape(coll_shape);
            new_obstacle->AddToCSpace();
        }
    } else { if(DEBUG) std::cout << "Error opening obstacle file: " << obs_file << std::endl; }
    read_stream.close();
    std::cout << "Read in " << num_polygons << " obstacles" << std::endl;
}
