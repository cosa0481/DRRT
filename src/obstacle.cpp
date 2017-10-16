#include <DRRT/data_structures.h>
#include <DRRT/obstacle_listnode.h>
#include <DRRT/collision_detection.h>
#include <DRRT/drrt.h>
#include "../src/list.cpp"

void Obstacle::UpdateObstacles(CSpace_ptr cspace)
{
    bool goal_reached = false;
    bool moved;

    while(!goal_reached)
    {
        moved = Obstacle::MoveObstacles(cspace);
        {
            lockguard lock(cspace->mutex_);
            cspace->obstacle_update_ = moved;
        }

        if(moved) {
            // TODO: ThetaStar
            std::cout << "TODO: ThetaStar" << std::endl;
//            cspace->robot->theta_star_path = ThetaStar();
//            cspace->robot->thetas = PathToThetas(cspace->robot->theta_star_path);
            exit(-1);
        }

        {
            lockguard lock(cspace->robot_->mutex);
            goal_reached = cspace->robot_->goal_reached;
        }
    }
}

bool Obstacle::MoveObstacles(std::shared_ptr<ConfigSpace> &cspace)
{
    lockguard lock(cspace->mutex_);
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
void Obstacle::AddObstacle()
{
    RangeList_ptr range_list = FindPointsInConflictWithObstacle(cspace->kdtree_, GetSharedPointer());

    Kdnode_ptr node;
    while(range_list->GetLength() > 0) {
        cspace->kdtree_->PopFromRangeList(range_list, node);

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

                // Add node to the priority queue for rewiring
                AddToPriorityQueue(cspace, node);
            }
        }
    }
    cspace->kdtree_->EmptyRangeList(range_list);
    is_used_ = true;
}

// TODO: Understand this function
void Obstacle::RemoveObstacle()
{
    // Find nodes in conflict with this obstacle and add them to the priority queue for rewiring
    RangeList_ptr range_list = FindPointsInConflictWithObstacle(cspace->kdtree_, GetSharedPointer());

    Kdnode_ptr node;
    while(range_list->GetLength() > 0) {
        cspace->kdtree_->PopFromRangeList(range_list, node);

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
                lockguard lock(cspace->mutex_);
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
            double min_node = std::min(node->GetCost(), node->GetLmc());
            double min_goal = std::min(cspace->move_goal_->GetCost(), cspace->move_goal_->GetLmc());
            if((node->GetCost() != node->GetLmc()) && (min_node < min_goal)) {
                // Add node to the priority queue for rewiring
                AddToPriorityQueue(cspace, node);
            }
        }
    }
    cspace->kdtree_->EmptyRangeList(range_list);
    is_used_ = false;
}

void Obstacle::AddToCSpace()
{
    lockguard lock(cspace->mutex_);
    Obstacle_ptr obstacle = GetSharedPointer();
    ObstacleListNode_ptr obstacle_listnode = std::make_shared<ObstacleListNode>(obstacle);
    cspace->obstacles_->Push(obstacle_listnode);
}
