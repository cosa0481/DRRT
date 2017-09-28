#include <DRRT/kdtree.h>


void KdTree::Print(Kdnode_ptr node, int indent, char type)
{
    if(indent) std::cout << std::string(indent - 1, ' ') << type;
    std::cout << node->GetPosition()(0) << ","
              << node->GetPosition()(1) << ": "
              << node->GetCost();

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
        if(pos(parent->GetKdSplit()) < parent->GetPosition()(parent->GetKdSplit())) {
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

bool KdTree::KdInsert(Kdnode_ptr &node)
{
    if(node->InTree()) return false;
    node->SetInTree(true);

    if(size_ == 0) {
        root_ = node;
        root_->SetKdSplit(0);
        size_ = 1;
        return true;
    }

    Kdnode_ptr parent = root_;
    while(true) {
        if(node->GetPosition()(parent->GetKdSplit())
                < parent->GetPosition()(parent->GetKdSplit())) {
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
    if(parent->GetKdSplit() == NUM_DIM - 1) node->SetKdSplit(0);
    else node->SetKdSplit(parent->GetKdSplit() + 1);

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
    Kdnode_ptr parent = root_;
    Kdnode_ptr current_closest_node = suggested;
    double current_closest_dist = suggested_dist;

    while(true) {
        if(query(parent->GetKdSplit()) < parent->GetPosition()(parent->GetKdSplit())) {
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
        double parent_hyperplane_dist = query(parent->GetKdSplit())
                - parent->GetPosition()(parent->GetKdSplit());

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
        if((query(parent->GetKdSplit()) < parent->GetPosition()(parent->GetKdSplit()))
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
        } else if((parent->GetPosition()(parent->GetKdSplit()) <= query(parent->GetKdSplit()))
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

}

KdnodeList_ptr KdTree::FindWithinRange(double range, Eigen::VectorXd query)
{

}

KdnodeList_ptr KdTree::FindWithinRangeInSubtree(Kdnode_ptr sub_root,
                                                double range,
                                                Eigen::VectorXd query)
{

}

KdnodeList_ptr KdTree::FindMoreWithinRange(double, Eigen::VectorXd query)
{

}
