#include <DRRT/drrt.h>

using namespace std;

// Priority queue //

void GetHeapNodeFor(Heap_ptr &heap, Kdnode_ptr node,
                    HeapNode_ptr &heap_node_for_kdnode)
{
    vector<HeapNode_ptr> heap_vec;
    heap->GetHeap(heap_vec);

    HeapNode_ptr hnode;
    Kdnode_ptr kdnode;
    for(int i = 0; i < heap_vec.size(); i++) {
        hnode = heap_vec[i];
        hnode->GetData(kdnode);
        if(node == kdnode) heap_node_for_kdnode = hnode;
    }
}

void AddToPriorityQueue(CSpace_ptr &cspace, Kdnode_ptr &node)
{
    lockguard lock(cspace->mutex_);
    HeapNode_ptr heapnode = make_shared<HeapNode>();
    if(node->InPriorityQueue()) {
        GetHeapNodeFor(cspace->priority_queue_, node, heapnode);
        cspace->priority_queue_->Update(heapnode);
    } else {
        node->SetInPriorityQueue(true);
        heapnode = make_shared<HeapNode>(node);
        cspace->priority_queue_->Add(heapnode);
    }
}

void AddToOSQueue(CSpace_ptr &cspace, Kdnode_ptr &node)
{
    lockguard lock(cspace->mutex_);
    HeapNode_ptr heapnode = make_shared<HeapNode>();
    if(node->InPriorityQueue()) {
        GetHeapNodeFor(cspace->priority_queue_, node, heapnode);
        cspace->priority_queue_->Update(heapnode);
        cspace->priority_queue_->Remove(heapnode);
    }

    KdnodeListNode_ptr listnode = make_shared<KdnodeListNode>(node);
    if(!node->InOSQueue()) {
        node->SetInOSQueue(true);
        cspace->obstacle_successors_->Push(listnode);
    }
}

void UpdateQueue(CSpace_ptr &cspace, Kdnode_ptr &node,
                 Kdnode_ptr &root, double hyper_ball_rad)
{
    RecalculateLmc(cspace, node, root, hyper_ball_rad);
    HeapNode_ptr heapnode = make_shared<HeapNode>();
    if(node->InPriorityQueue()) {
        GetHeapNodeFor(cspace->priority_queue_, node, heapnode);
        cspace->priority_queue_->Update(heapnode);
        cspace->priority_queue_->Remove(heapnode);
    } else {
        heapnode = make_shared<HeapNode>(node);
    }
    if(node->GetCost() != node->GetLmc())
        node->SetInPriorityQueue(true);
        cspace->priority_queue_->Add(heapnode);
}


// RRTx //

void MakeOutNeighbor(Kdnode_ptr &new_neighbor, Kdnode_ptr &node, Edge_ptr &edge)
{
    EdgeListNode_ptr edge_listnode = make_shared<EdgeListNode>(edge);

    node->GetOutNeighbors()->Push(edge_listnode);
    edge->list_item_in_start = node->GetOutNeighbors()->GetFront();

    new_neighbor->GetInNeighbors()->Push(edge_listnode);
    edge->list_item_in_end = node->GetInNeighbors()->GetFront();
}

void MakeInNeighbor(Kdnode_ptr &new_neighbor, Kdnode_ptr &node, Edge_ptr &edge)
{
    EdgeListNode_ptr edge_listnode = make_shared<EdgeListNode>(edge);

    node->GetInNeighbors()->Push(edge_listnode);
    edge->list_item_in_end = node->GetInNeighbors()->GetFront();

    new_neighbor->GetOutNeighbors()->Push(edge_listnode);
    edge->list_item_in_start = node->GetOutNeighbors()->GetFront();
}

void MakeInitialOutNeighbor(Kdnode_ptr &node, Edge_ptr &edge)
{
    EdgeListNode_ptr edge_listnode = make_shared<EdgeListNode>(edge);
    node->GetInitialOutNeighbors()->Push(edge_listnode);
}

void MakeInitialInNeighbor(Kdnode_ptr &node, Edge_ptr &edge)
{
    EdgeListNode_ptr edge_listnode = make_shared<EdgeListNode>(edge);
    node->GetInitialInNeighbors()->Push(edge_listnode);
}

void MakeParent(Kdnode_ptr &new_parent, Kdnode_ptr &node, Edge_ptr &edge)
{
    // Remove node from its old parent's successor list
    if(node->RrtParentExist()) {
        Edge_ptr parent_edge;
        EdgeListNode_ptr successor_listnode = node->GetSuccessorInParent();
        node->GetRrtParentEdge(parent_edge);
        parent_edge->GetEnd()->GetSuccessorList()->Remove(successor_listnode);
    }

    // Make new_parent the parent of the node
    node->SetRrtParentEdge(edge);
    node->SetRrtParentExist(true);

    // Place a non-trajectory reverse edge into the new_parent's successor
    // list and save a pointer to its position in that list
    // This edge is used to help keep track of successors (not movement)
    Edge_ptr back_edge = Edge::NewEdge(new_parent, node);
    back_edge->SetDist(INF);
    EdgeListNode_ptr backedge_listnode = make_shared<EdgeListNode>(back_edge);
    new_parent->GetSuccessorList()->Push(backedge_listnode);
    node->SetSuccessorInParent(new_parent->GetSuccessorList()->GetFront());
}

void FindBestParent(CSpace_ptr &cspace, Kdnode_ptr &new_node,
                    RangeList_ptr &near_nodes, Kdnode_ptr &closest_node)
{
    // Handle no near nodes case
    if(near_nodes->GetLength() == 0)
        if(new_node != cspace->goal_node_) {
            double closest_dist = DistanceFunction(new_node->GetPosition(), closest_node->GetPosition());
            RangeListNode_ptr closest_rangelistnode = make_shared<RangeListNode>(closest_node, closest_dist);
            near_nodes->Push(closest_rangelistnode);
        }

    new_node->SetLmc(INF);
    new_node->SetCost(INF);
    new_node->SetRrtParentExist(false);

    // Find best parent (or if one even exists)
    RangeListNode_ptr near_node_item = near_nodes->GetFront();
    Kdnode_ptr near_node;
    double near_range;
    Edge_ptr near_edge;
    while(near_node_item->GetChild() != near_node_item)
    {
        near_range = near_node_item->GetData(near_node);

        // Calculate shortest trajectory and distance that gets from new_node
        // to near_node
        near_edge = Edge::NewEdge(new_node, near_node);
        near_edge->CalculateTrajectory(cspace);

        // Check for validity of this edge
        bool edge_is_safe = !EdgeCheck(cspace, near_edge);
        {
            lockguard lock(cspace->kdtree_->mutex_);
            near_node->temp_edge_ = near_edge;
            if(!edge_is_safe || !near_edge->ValidMove()) {
                near_node->temp_edge_->SetDist(INF);
                near_node_item = near_node_item->GetChild();
                continue;
            }
        }

        // Check for potential better parent
        if(new_node->GetLmc() > near_node->GetLmc() + near_edge->GetDist()) {
            lockguard lock(cspace->kdtree_->mutex_);
            new_node->SetLmc(near_node->GetLmc() + near_edge->GetDist());
            MakeParent(near_node, new_node, near_edge);
        }

        near_node_item = near_node_item->GetChild();
    }
}

void FindNewTarget(CSpace_ptr &cspace, double radius)
{
    Eigen::VectorXd robot_pose, next_pose;
    {
        lockguard lock(cspace->robot_->mutex);
        robot_pose = cspace->robot_->pose;
        cspace->robot_->current_edge_used = false;
        cspace->robot_->dist_along_current_edge = 0.0;
        next_pose = cspace->robot_->next_move_target->GetPosition();
    }

    // Move target has become invalid
    double search_ball_rad = max(radius, DistanceFunction(robot_pose, next_pose));
    Eigen::MatrixX2d shape = cspace->drivable_region_.GetRegion();
    Eigen::Vector2d origin;
    origin(0) = shape.row(shape.rows()-1)(0) - shape.row(0)(0);
    origin(1) = shape.row(shape.rows()-1)(1) - shape.row(0)(1);
    double dist, max = 0;
    for(int i = 0; i < shape.rows(); i++) {
        dist = sqrt(pow(shape.row(i)(0) - origin(0), 2)
                    + pow(shape.row(i)(1) - origin(1), 2));
        if(dist > max) max = dist;
    }
    double max_search_rad = max;
    search_ball_rad = min(search_ball_rad, max_search_rad);

    RangeList_ptr range_list = cspace->kdtree_->FindWithinRange(search_ball_rad, robot_pose);

    Kdnode_ptr dummy_robot_node = make_shared<Kdnode>(robot_pose);
    Edge_ptr best_neighbor_edge = Edge::NewEdge(dummy_robot_node, dummy_robot_node);

    double best_dist_to_neighbor, best_dist_to_goal;
    Kdnode_ptr best_neighbor, neighbor_node;
    while(true)
    {
        // Search for new target within radius of search_ball_rad
        best_dist_to_neighbor = INF;
        best_dist_to_goal = INF;
        best_neighbor = make_shared<Kdnode>();

        RangeListNode_ptr range_item = range_list->GetFront();
        Edge_ptr range_edge;
        double dist_to_goal, dist_to_neighbor;
        while(range_item != range_item->GetChild()) {
            dist_to_neighbor = range_item->GetData(neighbor_node);

            range_edge = Edge::NewEdge(dummy_robot_node, neighbor_node);
            range_edge->CalculateTrajectory(cspace);
            if(range_edge->ValidMove() && EdgeCheck(cspace, range_edge)) {
                // Safe point found, see if it is the best so far
                dist_to_goal = neighbor_node->GetLmc() + range_edge->GetDist();
                if(dist_to_goal < best_dist_to_goal) {
                    // Found a new and better neighbor
                    best_dist_to_goal = dist_to_goal;
                    best_dist_to_neighbor = range_edge->GetDist();
                    best_neighbor = neighbor_node;
                    best_neighbor_edge = range_edge;
                }
            }

            range_item = range_item->GetChild();
        }
        // Done trying to find a target within radius

        // If valid neighbor found, use it
        if(best_dist_to_goal != INF) {
            lockguard lock(cspace->mutex_);
            cspace->robot_->next_move_target = best_neighbor;
            cspace->robot_->dist_next_pose_to_next_move_target = best_dist_to_neighbor;
            cspace->robot_->current_move_invalid = false;

            cspace->robot_->current_edge = best_neighbor_edge;
            cspace->robot_->current_edge_used = true;

            cspace->robot_->dist_along_current_edge = 0.0;

            // Set move_goal to be next_move_target
            // TODO: may want to actually insert a new nod at the robot's
            // position and use that since the edges created between robot
            // pose and next_move_target may be lengthy
            cspace->move_goal_->SetIsGoal(false);
            cspace->move_goal_ = cspace->robot_->next_move_target;
            cspace->move_goal_->SetIsGoal(true);
            break;
        }

        search_ball_rad *= 2;
        if(search_ball_rad > max_search_rad) {
            // Unable to find a valid move target so sample randomly
            Kdnode_ptr new_node = RandomNode(cspace);
            double new_dist = DistanceFunction(new_node->GetPosition(), robot_pose);
            new_node->Saturate(new_node->GetPosition(), robot_pose,
                               cspace->saturation_delta_, new_dist);
            cspace->kdtree_->Insert(new_node);
        }
        range_list = cspace->kdtree_->FindMoreWithinRange(search_ball_rad, robot_pose, range_list);
    }
    cspace->kdtree_->EmptyRangeList(range_list);
}

bool Extend(CSpace_ptr &cspace, Kdnode_ptr &new_node,
            Kdnode_ptr &closest_node, double delta, double hyper_ball_rad)
{
    // Find all nodes within hyper ball of new_node
    RangeList_ptr near_nodes = std::make_shared<RangeList>();
    {
        lockguard lock(cspace->kdtree_->mutex_);
        near_nodes = cspace->kdtree_->FindWithinRange(hyper_ball_rad, new_node->GetPosition());
    }

    // Find and link to best parent
    // (Saves edges from new_node to neighbors in the field temp_edge of neighbors)
    FindBestParent(cspace, new_node, near_nodes, closest_node);

    // If no parent was found then ignore this node
    if(!new_node->ParentExist()) {
        cspace->kdtree_->EmptyRangeList(near_nodes);
        return false;
    }

    // Otherwise insert the new node into the k-d tree
    {
        lockguard lock(cspace->kdtree_->mutex_);
        cspace->kdtree_->Insert(new_node);
    }

    // Link the neighbors and rewire neighbors that would do better to use
    // new_node as their parent
    // (Edges from new_node to its neighbors have been stored in temp_edge of neighbors)
    RangeListNode_ptr near_node_item = near_nodes->GetFront();
    Kdnode_ptr near_node;
    Edge_ptr near_edge;
    double old_lmc, near_node_range;
    for(int i = 0; i < near_nodes->GetLength(); i++) {
        near_node_range = near_node_item->GetData(near_node);

        if(near_node_range != INF) {
            // Add to initial out-neighbor list of new_node
            lockguard lock(cspace->kdtree_->mutex_);
            MakeInitialOutNeighbor(new_node, near_node->temp_edge_);

            // Add to current out-neighbor list of new_node
            // and current in-neighbor list of near_node
            MakeOutNeighbor(near_node, new_node, near_node->temp_edge_);
        }

        // Calculate the trajectory along the edge from near_node to new_node
        near_edge = Edge::NewEdge(near_node, new_node);
        near_edge->CalculateTrajectory(cspace);

        bool edge_is_safe = !EdgeCheck(cspace, near_edge);
        if(near_edge->ValidMove() && edge_is_safe) {
            // Add to initial in-neighbor list of new_node
            lockguard lock(cspace->kdtree_->mutex_);
            MakeInitialInNeighbor(near_node, near_edge);

            // Add to current in-neighbor list of new_node
            // and current out-neighbor list of near_node
            MakeInNeighbor(near_node, new_node, near_edge);
        } else {
            // Edge cannot be created
            near_node_item = near_node_item->GetChild();
            continue;
        }

        // Rewire neighbors that would do better to use this node
        // as their parent unless they are not in the relevant portion of space
        Edge_ptr parent_edge;
        new_node->GetRrtParentEdge(parent_edge);
        if(near_node->GetLmc() > new_node->GetLmc() + near_edge->GetDist()
                && parent_edge->GetEnd() != near_node
                && new_node->GetLmc() + near_edge->GetDist() < cspace->move_goal_->GetLmc()) {
            lockguard lock(cspace->kdtree_->mutex_);

            // Make new_node the parent of the near_node
            MakeParent(new_node, near_node, near_edge);

            // Recalculate cost of the near node
            old_lmc = near_node->GetLmc();
            near_node->SetLmc(new_node->GetLmc() + near_edge->GetDist());

            // Insert neighbor into priority queue if cost reduction
            // is great enough
            if((old_lmc - near_node->GetLmc() > cspace->change_thresh_)
                    && near_node != cspace->kdtree_->root_) {
                AddToPriorityQueue(cspace, near_node);
            }
        }

        near_node_item = near_node_item->GetChild();
    }

    cspace->kdtree_->EmptyRangeList(near_nodes);

    // Insert new_node into the priority queue for initial rewiring
    // Add node to priority queue for rewiring
    AddToPriorityQueue(cspace, new_node);
    return true;
}

void CullCurrentOutNeighbors(Kdnode_ptr &node, double radius)
{
    // Remove outgoing edges from node that are longer than radius
    EdgeListNode_ptr out_item = node->GetOutNeighbors()->GetFront();
    EdgeListNode_ptr next_out_item;
    Edge_ptr neighbor_edge;
    Kdnode_ptr neighbor_node;
    while(out_item != out_item->GetChild()) {
        next_out_item = out_item->GetChild();
        out_item->GetData(neighbor_edge);
        if(neighbor_edge->GetDist() > radius) {
            neighbor_node = neighbor_edge->GetEnd();
            node->GetOutNeighbors()->Remove(neighbor_edge->list_item_in_start);
            neighbor_node->GetInNeighbors()->Remove(neighbor_edge->list_item_in_end);
        }
        out_item = next_out_item;
    }
}

void RecalculateLmc(CSpace_ptr &cspace, Kdnode_ptr &node, Kdnode_ptr &root, double radius)
{
    if(node != root)
    {
        bool new_parent_found = false;
        double neighbor_dist;
        Kdnode_ptr rrt_parent, neighbor_node;
        Edge_ptr parent_edge, neighbor_edge, rrt_parent_edge;
        EdgeListNode_ptr edge_item;

        // Remove outdated nodes from the current out neighbor list
        CullCurrentOutNeighbors(node, radius);

        // Get an iterator for this node
        RrtNodeNeighborIterator_ptr node_out_neighbors
                = make_shared<RrtNodeNeighborIterator>(node);

        // Set the iterator to the first neighbor
        edge_item = NextOutNeighbor(node_out_neighbors);
        while(!edge_item->IsEmpty()) {
            edge_item->GetData(neighbor_edge);
            neighbor_node = neighbor_edge->GetEnd();
            neighbor_dist = neighbor_edge->GetDist();

            if(neighbor_node->InOSQueue()) {
                // neighbor_node is already orphaned or unwired
                edge_item = NextOutNeighbor(node_out_neighbors);
                continue;
            }

            neighbor_node->GetRrtParentEdge(rrt_parent_edge);
            if((node->GetLmc() > neighbor_node->GetLmc() + neighbor_dist)
                    && (!neighbor_node->RrtParentExist() || rrt_parent_edge->GetEnd() != node)
                    && neighbor_edge->ValidMove()) {
                // Found a better parent
                node->SetLmc(neighbor_node->GetLmc() + neighbor_dist);
                rrt_parent = neighbor_node;
                parent_edge = neighbor_edge;
                new_parent_found = true;
            }

            edge_item = NextOutNeighbor(node_out_neighbors);
        }

        if(new_parent_found) MakeParent(rrt_parent, node, parent_edge);
    }
}

void Rewire(CSpace_ptr &cspace, Kdnode_ptr &node, Kdnode_ptr &root, double radius)
{
    // Only explicitly propagate changes if they are large enough
    double delta_cost = node->GetCost() - node->GetLmc();
    if(delta_cost > cspace->change_thresh_)
    {
        // Remove outdated nodes from the current out neighbor list
        CullCurrentOutNeighbors(node, radius);

        // Get an iterator for this node's neighbors and iterate through
        // the in neighbors
        RrtNodeNeighborIterator_ptr node_in_neighbors
                = make_shared<RrtNodeNeighborIterator>(node);
        EdgeListNode_ptr edge_item = NextInNeighbor(node_in_neighbors);
        Kdnode_ptr neighbor_node;
        Edge_ptr neighbor_edge, rrt_parent_edge;
        while(!edge_item->IsEmpty()) {
            edge_item->GetData(neighbor_edge);
            neighbor_node = neighbor_edge->GetStart();

            // Ignore this node's parent and also nodes that cannot reach
            // the node due to dynamics
            node->GetRrtParentEdge(rrt_parent_edge);
            if((node->RrtParentExist() && rrt_parent_edge->GetEnd() == neighbor_node)
                    || !neighbor_edge->ValidMove()) {
                edge_item = NextInNeighbor(node_in_neighbors);
                continue;
            }

            node->GetRrtParentEdge(rrt_parent_edge);
            if((neighbor_node->GetLmc() > node->GetLmc() + neighbor_edge->GetDist())
                    && (!neighbor_node->RrtParentExist() || rrt_parent_edge->GetEnd() != node)
                    && neighbor_edge->ValidMove()) {
                // neighbor_node should use node as its parent
                neighbor_node->SetLmc(node->GetLmc() + neighbor_edge->GetDist());
                MakeParent(node, neighbor_node, neighbor_edge);

                // If the reduction was great enough then propagate to neighbor
                if(neighbor_node->GetCost() - neighbor_node->GetLmc() > cspace->change_thresh_)
                    AddToPriorityQueue(cspace, neighbor_node);
            }

            edge_item = NextInNeighbor(node_in_neighbors);
        }
    }
}

void ReduceInconsistency(CSpace_ptr &cspace, Kdnode_ptr goal, Kdnode_ptr &root, double radius)
{
    lockguard lock(cspace->mutex_);
    HeapNode_ptr node_heap_item;
    Kdnode_ptr node;
    cspace->priority_queue_->Top(node_heap_item);
    node_heap_item->GetData(node);
    double min_node_cost = min(node->GetCost(), node->GetLmc());
    double min_goal_cost = min(goal->GetCost(), goal->GetLmc());
    bool less_than = (min_node_cost < min_goal_cost) || (min_node_cost == min_goal_cost && node->IsGoal());
    while(cspace->priority_queue_->GetIndexOfLast() > 0
          && (less_than || goal->GetLmc() == INF || goal->GetCost() == INF || goal->InPriorityQueue())) {
        cspace->priority_queue_->Pop(node_heap_item);
        node_heap_item->GetData(node);

        // Update neighbors of node if it has changed enough
        if(node->GetCost() - node->GetLmc() > cspace->change_thresh_) {
            RecalculateLmc(cspace, node, root, radius);
            Rewire(cspace, node, root, radius);
        }
        node->SetCost(node->GetLmc());
    }
}

void PropogateDescendents(CSpace_ptr &cspace)
{
    if(cspace->obstacle_successors_->GetLength() > 0)
    {
        lockguard lock(cspace->mutex_);
        KdnodeListNode_ptr os_list_item = cspace->obstacle_successors_->GetBack();
        Kdnode_ptr node, successor_node;
        EdgeListNode_ptr successor_list_item;
        Edge_ptr successor_edge;
        while(os_list_item != os_list_item->GetParent()) {
            os_list_item->GetData(node);

            // Add all of this node's sucessors to the OS stack
            successor_list_item = node->GetSuccessorList()->GetFront();
            while(successor_list_item != successor_list_item->GetChild()) {
                successor_list_item->GetData(successor_edge);
                successor_node = successor_edge->GetEnd();
                AddToOSQueue(cspace, successor_node);
                successor_list_item = successor_list_item->GetChild();
            }

            os_list_item = os_list_item->GetParent();
        }

        os_list_item = cspace->obstacle_successors_->GetBack();
        EdgeListNode_ptr out_neighbor_item;
        Kdnode_ptr neighbor_node;
        Edge_ptr neighbor_edge;
        while(os_list_item != os_list_item->GetParent()) {
            os_list_item->GetData(node);

            RrtNodeNeighborIterator_ptr node_out_neighbors
                    = make_shared<RrtNodeNeighborIterator>(node);

            // Add all neighbors to the priority queue except those in OS stack
            out_neighbor_item = NextOutNeighbor(node_out_neighbors);
            while(!out_neighbor_item->IsEmpty()) {
                out_neighbor_item->GetData(neighbor_edge);
                neighbor_node = neighbor_edge->GetEnd();

                if(neighbor_node->InOSQueue()) {
                    out_neighbor_item = NextOutNeighbor(node_out_neighbors);
                    continue;
                }

                neighbor_node->SetCost(INF);
                AddToPriorityQueue(cspace, neighbor_node);

                out_neighbor_item = NextOutNeighbor(node_out_neighbors);
            }

            // Add parent to the priority queue unless it is in the OS queue
            Edge_ptr rrt_parent_edge;
            Kdnode_ptr parent_node;
            node->GetRrtParentEdge(rrt_parent_edge);
            if(node->RrtParentExist() && !rrt_parent_edge->GetEnd()->InOSQueue()) {
                node->parent_->SetCost(INF);
                parent_node = rrt_parent_edge->GetEnd();
                AddToPriorityQueue(cspace, parent_node);
            }

            os_list_item = os_list_item->GetParent();
        }

        KdnodeListNode_ptr node_list_item;
        while(cspace->obstacle_successors_->GetLength() > 0) {
            cspace->obstacle_successors_->Pop(node_list_item);
            node_list_item->GetData(node);
            node->SetInOSQueue(false);

            if(node == cspace->robot_->next_move_target)
                cspace->robot_->current_move_invalid = true;

            if(node->RrtParentExist()) {
                // Remove node from it's parent's successor list
                Edge_ptr rrt_parent_edge;
                node->GetRrtParentEdge(rrt_parent_edge);
                EdgeListNode_ptr node_successor_in_parent = node->GetSuccessorInParent();
                rrt_parent_edge->GetEnd()->GetSuccessorList()->Remove(node_successor_in_parent);

                // node now has no parent
                rrt_parent_edge = Edge::NewEdge(node, node);
                rrt_parent_edge->SetDist(INF);
                node->SetRrtParentEdge(rrt_parent_edge);
                node->SetRrtParentExist(false);
            }

            node->SetCost(INF);
            node->SetLmc(INF);
        }
    }
}
