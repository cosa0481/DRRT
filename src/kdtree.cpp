#include <DRRT/kdtree.h>
#include <DRRT/data_structures.h>
#include "../src/list.cpp"

void KdTree::Print(Kdnode_ptr node, int indent, char type)
{
    if(indent) std::cout << std::string(indent - 1, ' ') << type;
    std::cout << node->GetPosition()(0) << ","
              << node->GetPosition()(1) << " ("
              << node->GetCost() << ")";

    if(node->ParentExist()) {
        Edge_ptr parent_edge;
        node->GetRrtParentEdge(parent_edge);
        std::cout << " : " << parent_edge->GetEnd()->GetPosition()(0)
                  << "," << parent_edge->GetEnd()->GetPosition()(1);
        std::cout << " (" << parent_edge->GetDist() << ")";
    }

    if(!node->LChildExist() && !node->RChildExist()) {
        std::cout << " | leaf" << std::endl;
    } else {
        std::cout << std::endl;
        if(node->LChildExist()) Print(node->lchild_, indent + 4, '<');
        if(node->RChildExist()) Print(node->rchild_, indent + 4, '>');
    }
}

void KdTree::GetNodeAt(Kdnode_ptr &node, Eigen::VectorXd pos)
{
    Kdnode_ptr parent = root_;
    node = parent;
    while(true) {
        if(pos(parent->split_) < parent->GetPosition()(parent->split_)) {
            // Go left
            if(parent->GetPosition() == pos) {
                node = parent;
                break;
            }
            if(!parent->LChildExist()) std::cout << "no left child" << std::endl;
            parent = parent->lchild_;
            continue;
        } else {
            // Go right
            if(parent->GetPosition() == pos) {
                node = parent;
                break;
            }
            if(!parent->RChildExist()) std::cout << "no right child" << std::endl;
            parent = parent->rchild_;
            continue;
        }
    }
}

bool KdTree::Insert(Kdnode_ptr &node)
{
    lockguard lock(mutex_);
    if(node->InTree()) return false;
    node->SetInTree(true);

    if(size_ == 0) {
        root_ = node;
        root_->split_ = 0;
        size_ = 1;
        return true;
    }

    Kdnode_ptr parent = root_;
    while(true) {
        if(node->GetPosition()(parent->split_)
                < parent->GetPosition()(parent->split_)) {
            // Go left
            if(!parent->LChildExist()) {
                parent->lchild_ = node;
                parent->SetLChildExist(true);
                break;
            }
            parent = parent->lchild_;
            continue;
        } else {
            // Go right
            if(!parent->RChildExist()) {
                parent->rchild_ = node;
                parent->SetRChildExist(true);
                break;
            }
            parent = parent->rchild_;
            continue;
        }
    }
    node->parent_ = parent;
    node->SetParentExist(true);
    if(parent->split_ == NUM_DIM - 1) node->split_ = 0;
    else node->split_ = parent->split_ + 1;

    size_ += 1;
    return true;
}

double KdTree::FindNearest(Kdnode_ptr &nearest, Eigen::VectorXd query)
{
    double dist_to_root = DistanceFunction(query, root_->GetPosition());

    Kdnode_ptr l_node = std::make_shared<Kdnode>();
    double l_dist;
    l_dist = FindNearestInSubtree(l_node, root_, query, root_, dist_to_root);

    // If any dimensions wrap around, need to search identities (ghosts)
    if(num_wraps_ > 0) {
        GhostPointIterator_ptr point_iterator
                = std::make_shared<GhostPointIterator>(GetSharedPointer(), query);
        Eigen::VectorXd ghost_point;

        // Iterate through ghost points
        while(true) {
            ghost_point = point_iterator->GetNextGhostPoint(l_dist);
            if(ghost_point.isZero(0)) break;

            // See if any points in space are closer to this ghost
            double dist_ghost_to_root = DistanceFunction(ghost_point, root_->GetPosition());
            Kdnode_ptr ghost_l_node = std::make_shared<Kdnode>();
            double ghost_l_dist;
            ghost_l_dist = FindNearestInSubtree(ghost_l_node, root_,
                                                ghost_point, root_,
                                                dist_ghost_to_root);

            if(ghost_l_dist < l_dist) {
                // Found a point that is closer to the query point
                l_node = ghost_l_node;
                l_dist = ghost_l_dist;
            }
        }
    }
    nearest = l_node;
    return l_dist;
}

double KdTree::FindNearestInSubtree(Kdnode_ptr &nearest, Kdnode_ptr sub_root,
                                    Eigen::VectorXd query, Kdnode_ptr &suggested,
                                    double suggested_dist)
{
    Kdnode_ptr parent = sub_root;
    Kdnode_ptr current_closest_node = suggested;
    double current_closest_dist = suggested_dist;

    while(true) {
        if(query(parent->split_) < parent->GetPosition()(parent->split_)) {
            // Traverse tree to the left
            if(!parent->LChildExist()) break;
            parent = parent->lchild_;
            continue;
        } else {
            // Traverse tree to the right
            if(!parent->RChildExist()) break;
            parent = parent->rchild_;
            continue;
        }
    }

    double new_dist = DistanceFunction(query, parent->GetPosition());
    if(new_dist < current_closest_dist) {
        current_closest_node = parent;
        current_closest_dist = new_dist;
    }

    while(true)
    {
        // Check if there could possibly be any claser nodes on the other
        // side of the parent. If not, then check grandparent, etc...
        double parent_hyperplane_dist = query(parent->split_)
                - parent->GetPosition()(parent->split_);

        if(parent_hyperplane_dist > current_closest_dist) {
            // There are not any closer nodes on the other side of the parent
            // The parent itself is also too far away

            if(parent == sub_root) {
                // Parent is the root
                nearest = current_closest_node;
                return current_closest_dist;
            }

            parent = parent->parent_;
            continue;
        }

        // Now there could be a closer node on the other side of the parent
        // including the parent itself

        // Check if the parent itself is closer
        if(current_closest_node != parent) {
            new_dist = DistanceFunction(query, parent->GetPosition());
            if(new_dist < current_closest_dist) {
                current_closest_node = parent;
                current_closest_dist = new_dist;
            }
        }

        // Check the other side of the parent
        if((query(parent->split_) < parent->GetPosition()(parent->split_))
                && parent->RChildExist()) {
            // The query point is on the left side of the parent, so look
            // at the right side of the parent

            // Find the right subtree distance
            Kdnode_ptr r_node = std::make_shared<Kdnode>();
            double r_dist;
            r_dist = FindNearestInSubtree(r_node, parent->rchild_, query,
                                          current_closest_node, current_closest_dist);
            if(r_dist < current_closest_dist) {
                current_closest_node = r_node;
                current_closest_dist = r_dist;
            }
        } else if((parent->GetPosition()(parent->split_) <= query(parent->split_))
                  && parent->LChildExist()) {
            // The query point is on the right side of the parent, so look
            // at the left side of the parent

            // Find the left subtree distance
            Kdnode_ptr l_node = std::make_shared<Kdnode>();
            double l_dist;
            l_dist = FindNearestInSubtree(l_node, parent->lchild_, query,
                                          current_closest_node, current_closest_dist);
            if(l_dist < current_closest_dist) {
                current_closest_node = l_node;
                current_closest_dist = l_dist;
            }
        }

        if(parent == sub_root) {
            // Parent is the root
            nearest = current_closest_node;
            return current_closest_dist;
        }
        parent = parent->parent_;
    }
}

double KdTree::FindNearestWithGuess(Kdnode_ptr &nearest, Eigen::VectorXd query,
                                    Kdnode_ptr guess)
{
    double dist_to_guess = DistanceFunction(query, guess->GetPosition());
    if(guess == root_)
        return FindNearestInSubtree(nearest, root_, query, root_, dist_to_guess);

    Kdnode_ptr l_node = std::make_shared<Kdnode>();
    double l_dist;
    l_dist = FindNearestInSubtreeWithGuess(l_node, root_, query, guess, dist_to_guess);

    // If any dimensions wrap around, need to search identities (ghosts)
    if(num_wraps_ > 0) {
        GhostPointIterator_ptr point_iterator
                = std::make_shared<GhostPointIterator>(GetSharedPointer(), query);
        Eigen::VectorXd ghost_point;

        // Iterate through ghost points
        while(true) {
            ghost_point = point_iterator->GetNextGhostPoint(l_dist);
            if(ghost_point.isZero(0)) break;

            // See if any points in space are closer to this ghost
            double dist_ghost_to_guess = DistanceFunction(ghost_point, guess->GetPosition());

            Kdnode_ptr ghost_l_node;
            double ghost_l_dist;
            ghost_l_dist = FindNearestInSubtreeWithGuess(ghost_l_node, root_,
                                                         ghost_point, guess,
                                                         dist_ghost_to_guess);

            if(ghost_l_dist < l_dist) {
                // Found a point that is closer to the query point
                l_node = ghost_l_node;
                l_dist = ghost_l_dist;
            }
        }
    }

    nearest = l_node;
    return l_dist;
}

double KdTree::FindNearestInSubtreeWithGuess(Kdnode_ptr &nearest,
                                             Kdnode_ptr sub_root,
                                             Eigen::VectorXd query,
                                             Kdnode_ptr &suggested,
                                             double suggested_dist)
{
    Kdnode_ptr parent = sub_root;
    Kdnode_ptr current_closest_node = suggested;
    double current_closest_dist = suggested_dist;

    while(true) {
        if(query(parent->split_) < parent->GetPosition()(parent->split_)) {
            // Traverse tree to the left
            if(!parent->LChildExist()) break;
            parent = parent->lchild_;
            continue;
        } else {
            // Traverse tree to the right
            if(!parent->RChildExist()) break;
            parent = parent->rchild_;
            continue;
        }
    }

    double new_dist = DistanceFunction(query, parent->GetPosition());
    if(new_dist < current_closest_dist) {
        current_closest_node = parent;
        current_closest_dist = new_dist;
    }

    // Now walk back up the tree
    while(true)
    {
        // Check if there could possibly be any claser nodes on the
        // other side of the parent. If not, then check grandparents, etc...
        double parent_hyperplane_dist = query(parent->split_)
                - parent->GetPosition()(parent->split_);

        if(parent_hyperplane_dist > current_closest_dist) {
            // There are no closer nodes on the other side of the parent
            // and the parent itself is too far away
            if(parent == sub_root) {
                nearest = current_closest_node;
                return current_closest_dist;
            }

            parent = parent->parent_;
            continue;
        }

        // There could be a closer node on the other side of the parent
        // including the parent itself

        // Check the parent
        if(current_closest_node != parent) {
            new_dist = DistanceFunction(query, parent->GetPosition());
            if(new_dist < current_closest_dist) {
                current_closest_node = parent;
                current_closest_dist = new_dist;
            }
        }

        // Check the other side of the parent
        if((query(parent->split_) < parent->GetPosition()(parent->split_))
                && parent->RChildExist()) {
            // query point is on the left side of the parent so look
            // at the right side of the parent

            // Find right subtree distance
            Kdnode_ptr r_node = std::make_shared<Kdnode>();
            double r_dist;
            r_dist = FindNearestInSubtree(r_node, parent->rchild_, query,
                                          current_closest_node, current_closest_dist);

            if(r_dist < current_closest_dist) {
                current_closest_node = r_node;
                current_closest_dist = r_dist;
            }
        } else if((parent->GetPosition()(parent->split_) <= query(parent->split_))
                  && parent->LChildExist()) {
            // query point is on the right side of the parent, so look
            // at the left side of the parent

            // Find left subtree distance
            Kdnode_ptr l_node = std::make_shared<Kdnode>();
            double l_dist;
            l_dist = FindNearestInSubtree(l_node, parent->lchild_, query,
                                          current_closest_node,
                                          current_closest_dist);

            if(l_dist < current_closest_dist) {
                current_closest_node = l_node;
                current_closest_dist = l_dist;
            }
        }

        if(parent == sub_root) {
            // Parent is root and we are done
            // But check the root
            double root_dist = DistanceFunction(query, parent->GetPosition());
            if(root_dist < current_closest_dist) {
                nearest = parent;
                return root_dist;
            }
            nearest = current_closest_node;
            return current_closest_dist;
        }

        parent = parent->parent_;
    }
}

void KdTree::AddToRangeList(RangeList_ptr list, Kdnode_ptr &node, double value)
{
    node->SetInRangeList(true);
    RangeListNode_ptr range_node = std::make_shared<RangeListNode>(node, value);
    list->Push(range_node);
}

double KdTree::PopFromRangeList(RangeList_ptr list, Kdnode_ptr &node)
{
    RangeListNode_ptr range_node = std::make_shared<RangeListNode>();
    list->Pop(range_node);
    double dist = range_node->GetData(node);
    node->SetInRangeList(false);
    return dist;
}

void KdTree::EmptyRangeList(RangeList_ptr list)
{
    RangeListNode_ptr range_node = std::make_shared<RangeListNode>();
    Kdnode_ptr node = std::make_shared<Kdnode>();
    double dist;
    while(list->GetLength() > 0) {
        list->Pop(range_node);
        dist = range_node->GetData(node);
        node->SetInRangeList(false);
    }
}

RangeList_ptr KdTree::FindWithinRange(double range, Eigen::VectorXd query)
{
    RangeList_ptr range_list = std::make_shared<RangeList>();

    // Insert root node in list if it is in range
    double root_dist = DistanceFunction(query, root_->GetPosition());
    if(root_dist <= range) AddToRangeList(range_list, root_, root_dist);

    // Find nodes within range
    range_list = FindWithinRangeInSubtree(root_, range, query, range_list);

    if(num_wraps_ > 0) {
        // If dimensions wrap, need to search vs. identities (ghosts)
        GhostPointIterator_ptr point_iterator
                = std::make_shared<GhostPointIterator>(GetSharedPointer(), query);
        while(true) {
            Eigen::VectorXd ghost_point = point_iterator->GetNextGhostPoint(range);
            if(ghost_point.isZero(0)) break;

            // See if any points in the space are closer to this ghost
            range_list = FindWithinRangeInSubtree(root_, range, ghost_point, range_list);
        }
    }

    return range_list;
}

RangeList_ptr KdTree::FindWithinRangeInSubtree(Kdnode_ptr sub_root,
                                               double range,
                                               Eigen::VectorXd query,
                                               RangeList_ptr range_list)
{
    // Walk down tree as if node would be inserted
    Kdnode_ptr parent = sub_root;
    while(true) {
        if(query(parent->split_) < parent->GetPosition()(parent->split_)) {
            // Traverse tree to the left
            if(!parent->LChildExist()) break;
            parent = parent->lchild_;
            continue;
        } else {
            // Traverse tree to the left
            if(!parent->RChildExist()) break;
            parent = parent->rchild_;
            continue;
        }
    }

    double new_dist = DistanceFunction(query, parent->GetPosition());
    if(!parent->InRangeList() && (new_dist < range)) AddToRangeList(range_list, parent, new_dist);

    // Now walk back up tree
    while(true)
    {
        // Check if there could be any nodes on the other side of the parent
        // in range; if not check grandparent, etc...

        double parent_hyperplane_dist = query(parent->split_)
                - parent->GetPosition()(parent->split_);
        if(parent_hyperplane_dist > range) {
            // There are no closer nodes in range on the other side of the
            // parent and the parent itself is too far away
            if(parent == sub_root) return range_list;
            parent = parent->parent_;
            continue;
        }

        // There could be a closer node on the other side of the parent
        // including the parent itself within range

        // Check the parent
        if(!parent->InRangeList()) {
            new_dist = DistanceFunction(query, parent->GetPosition());
            if(new_dist < range) AddToRangeList(range_list, parent, new_dist);
        }

        // Check the other side of the parent
        if((query(parent->split_) < parent->GetPosition()(parent->split_))
                && parent->RChildExist()) {
            // query point is on the left side of the parent, so look at the
            // right side of the parent
            range_list = FindWithinRangeInSubtree(parent->rchild_, range,
                                                  query, range_list);
        } else if((parent->GetPosition()(parent->split_) <= query(parent->split_))
                  && parent->LChildExist()) {
            // query point is on the right side of the parent, so look at the
            // left side of the parent
            range_list = FindWithinRangeInSubtree(parent->lchild_, range,
                                                  query, range_list);
        }

        if(parent == sub_root) return range_list;

        parent = parent->parent_;
    }
}

RangeList_ptr KdTree::FindMoreWithinRange(double range, Eigen::VectorXd query, RangeList_ptr range_list)
{
    // Insert root node in list if it is in range
    double root_dist = DistanceFunction(query, root_->GetPosition());
    if(root_dist <= range) AddToRangeList(range_list, root_, root_dist);

    // Find nodes within range
    range_list = FindWithinRangeInSubtree(root_, range, query, range_list);

    if(num_wraps_ > 0) {
        // If dimensions wrap, need to search vs. identities (ghosts)
        GhostPointIterator_ptr point_iterator
                = std::make_shared<GhostPointIterator>(GetSharedPointer(), query);
        while(true) {
            Eigen::VectorXd ghost_point = point_iterator->GetNextGhostPoint(range);
            if(ghost_point.isZero(0)) break;

            // See if any points in the space are closer to this ghost
            range_list = FindWithinRangeInSubtree(root_, range, ghost_point, range_list);
        }
    }

    return range_list;
}
