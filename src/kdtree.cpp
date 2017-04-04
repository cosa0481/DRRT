/* kdtree.cpp
 * Implemented for C++ by: Corin Sandford
 * Original Julia code by: Michael Otte
 * Fall 2016
 */

#ifndef KDTREE_CPP
#define KDTREE_CPP

#include <DRRT/kdtree.h>
#include <iostream>

void KDTree::AddVizNode(std::shared_ptr<KDTreeNode> node)
{
    std::lock_guard<std::mutex> lock(this->tree_mutex_);
    this->nodes_.push_back(node);
}

void KDTree::RemoveVizNode(std::shared_ptr<KDTreeNode> &node)
{
    std::lock_guard<std::mutex> lock(this->tree_mutex_);
    this->nodes_.erase(
                std::remove(this->nodes_.begin(),this->nodes_.end(),node),
                this->nodes_.end());
}

void KDTree::PrintTree(std::shared_ptr<KDTreeNode> node,
                       int indent, char type)
{
    if(indent) std::cout << std::string(indent-1,' ') << type;
    std::cout << node->position_(0) << ","
              << node->position_(1) << ": "
              << node->rrt_LMC_;
    if( node->rrt_parent_used_ ) {
        std::cout << " : " << node->rrt_parent_edge_->end_node_->position_(0) << ","
                  << node->rrt_parent_edge_->end_node_->position_(1);
        std::cout << " (" << node->rrt_parent_edge_->dist_ << ")";
    }
    if( !node->kd_child_L_exist_ && !node->kd_child_R_exist_ ) {
        std::cout << " | leaf" << std::endl;
    } else {
        std::cout << std::endl;
        if(node->kd_child_L_exist_) {
            PrintTree(node->kd_child_L_, indent+4, '<');
        }
        if(node->kd_child_R_exist_) {
            PrintTree(node->kd_child_R_, indent+4, '>');
        }
    }
}

void KDTree::GetNodeAt(Eigen::VectorXd pos,
                       std::shared_ptr<KDTreeNode>& node)
{
    std::shared_ptr<KDTreeNode> parent = this->root;
    node = parent;
    while( true ) {
        if(pos(parent->kd_split_) < parent->position_(parent->kd_split_)) {
            // Traverse tree to the left
            if( parent->position_ == pos ) {
                // The node gets inserted as the left child of the parent
                node = parent;
                break;
            }
            if(!parent->kd_child_L_exist_) std::cout << "no left child" << std::endl;
            parent = parent->kd_child_L_;
            continue;
        }
        else {
            // Traverse tree to the right
            if( parent->position_ == pos ) {
                // The node gets inserted as the right child of the parent
                node = parent;
                break;
            }
            if(!parent->kd_child_R_exist_) std::cout << "no right child" << std::endl;
            parent = parent->kd_child_R_;
            continue;
        }
    }
}

bool KDTree::KDInsert(std::shared_ptr<KDTreeNode>& node)
{
    if( node->kd_in_tree_ ) return false;
    node->kd_in_tree_ = true;
    // Add node to visualizer
    AddVizNode(node);

    if( this->tree_size_ == 0 ) {
        this->root = node;
        this->root->kd_split_ = 0;
        this->tree_size_ = 1;
        return true;
    }

    // Figure out where to put this node
    std::shared_ptr<KDTreeNode> parent = this->root;
    while( true ) {
        if(node->position_(parent->kd_split_)
                < parent->position_(parent->kd_split_)) {
            // Traverse tree to the left
            if( !parent->kd_child_L_exist_ ) {
                // The node gets inserted as the left child of the parent
                parent->kd_child_L_ = node;
                parent->kd_child_L_exist_ = true;
                break;
            }
            parent = parent->kd_child_L_;
            continue;
        }
        else {
            // Traverse tree to the right
            if( !parent->kd_child_R_exist_ ) {
                // The node gets inserted as the right child of the parent
                parent->kd_child_R_ = node;
                parent->kd_child_R_exist_ = true;
                break;
            }
            parent = parent->kd_child_R_;
            continue;
        }
    }

    node->kd_parent_ = parent;
    node->kd_parent_exist_ = true;
    if( parent->kd_split_ == this->dimensions_-1 ) { node->kd_split_ = 0; }
    else { node->kd_split_ = parent->kd_split_ + 1; }

    this->tree_size_ += 1;
    return true;
}

/////////////////////// Nearest ///////////////////////

bool KDTree::KDFindNearestInSubtree(std::shared_ptr<KDTreeNode>& nearestNode,
                                    std::shared_ptr<double> nearestNodeDist,
                                    std::shared_ptr<KDTreeNode>& root,
                                    Eigen::VectorXd queryPoint,
                           std::shared_ptr<KDTreeNode>& suggestedClosestNode,
                                    double suggestedClosestDist)
{
    std::shared_ptr<KDTreeNode> parent = root;
    std::shared_ptr<KDTreeNode> currentClosestNode = suggestedClosestNode;
    std::shared_ptr<double> currentClosestDist
            = std::make_shared<double>(suggestedClosestDist);

    while( true ) {
        if( queryPoint(parent->kd_split_) < parent->position_(parent->kd_split_) ) {
            // Traverse tree to the left
            if( !parent->kd_child_L_exist_ ) {
                // The queryPoint would be inserted as the left child of the parent
                break;
            }
            parent = parent->kd_child_L_;
            continue;
        }
        else {
            // Traverse tree to the right
            if( !parent->kd_child_R_exist_ ) {
                // The queryPoint would be inserted as the right child of the parent
                break;
            }
            parent = parent->kd_child_R_;
            continue;
        }
    }

    double newDist = this->distanceFunction(queryPoint, parent->position_ );
    if( newDist < *currentClosestDist ) {
//        std::cout << "nearest0 (no return)\n" << parent.get()->position_ << std::endl;
        currentClosestNode = parent;
        currentClosestDist = std::make_shared<double>(newDist);
    }


    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could plossibly be any closer nodes on the other
        // side of the parent. If not then check grandparent etc.
        double parentHyperPlaneDist = queryPoint(parent->kd_split_)
                - parent->position_(parent->kd_split_);

        if( parentHyperPlaneDist > *currentClosestDist ) {
            // Then there could not be any closer nodes on the other side of
            // the parent and the parent itself is also too far away

            if( parent == root ) {
                // The parent is the root and we are done
                nearestNode = currentClosestNode;
                *nearestNodeDist = *currentClosestDist;
                return true;
            }

            parent = parent->kd_parent_;
            continue;
        }

        // If we are here, then there could be a closer node on the other
        // side of the parent including the parent itself

        // First check the parent itself (if it is not already the
        // closest node)
        if( currentClosestNode != parent ) {
            newDist = this->distanceFunction( queryPoint, parent->position_);
            if( newDist < *currentClosestDist ) {
                currentClosestNode = parent;
                currentClosestDist = std::make_shared<double>(newDist);
            }
        }

        // Now check on the other side of the parent
        if( (queryPoint(parent->kd_split_) < parent->position_(parent->kd_split_))
                && parent->kd_child_R_exist_ ) {
            // queryPoint is on the left side of the parent, so we need to
            // look at the right side of it (if it exists)

            // Find right subtree distance
            std::shared_ptr<KDTreeNode> Rnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Rdist = std::make_shared<double>(0);
            KDFindNearestInSubtree( Rnode, Rdist,
                                    parent->kd_child_R_, queryPoint,
                                    currentClosestNode, *currentClosestDist );
            if( *Rdist < *currentClosestDist ) {
                currentClosestNode = Rnode;
                currentClosestDist = Rdist;
            }
        }
        else if( (parent->position_(parent->kd_split_)
                  <= queryPoint(parent->kd_split_))
                 && parent->kd_child_L_exist_ ) {
            // queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)

            // Find left subtree distance
            std::shared_ptr<KDTreeNode> Lnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Ldist = std::make_shared<double>(0);
            KDFindNearestInSubtree( Lnode, Ldist,
                                    parent->kd_child_L_, queryPoint,
                                    currentClosestNode, *currentClosestDist );
            if( *Ldist < *currentClosestDist ) {
                currentClosestNode = Lnode;
                currentClosestDist = Ldist;
            }
        }


        if( parent == root ) {
            // The parent is the root and we are done
            nearestNode = currentClosestNode;
            *nearestNodeDist = *currentClosestDist;
            return true;
        }
        parent = parent->kd_parent_;
    }
}

bool KDTree::KDFindNearest(std::shared_ptr<KDTreeNode>& nearestNode,
                           std::shared_ptr<double> nearestNodeDist,
                           Eigen::VectorXd queryPoint)
{
    // Initial search (only search if the space does not wrap around)
    double distToRoot = this->distanceFunction(queryPoint,
                                               this->root->position_);
    std::shared_ptr<KDTreeNode> Lnode
            = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> Ldist = std::make_shared<double>(0);
    KDFindNearestInSubtree( Lnode, Ldist, this->root,
                            queryPoint, this->root, distToRoot );

    if( this->num_wraps_ > 0 ) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        std::shared_ptr<GhostPointIterator> pointIterator
                = std::make_shared<GhostPointIterator>
                (GhostPointIterator( this, queryPoint ));
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = GetNextGhostPoint( pointIterator, *Ldist );
            if( thisGhostPoint.isZero(0) ) break;

            // Now see if any points in the space are closer to this ghost
            double distGhostToRoot = this->distanceFunction(thisGhostPoint,
                                                        this->root->position_);
            std::shared_ptr<KDTreeNode> thisLnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> thisLdist = std::make_shared<double>(0);
            KDFindNearestInSubtree( thisLnode, thisLdist, this->root,
                                    thisGhostPoint, this->root,
                                    distGhostToRoot );

            if( *thisLdist < *Ldist ) {
                // Found a closer point
                Lnode = thisLnode;
                Ldist = thisLdist;
            }
        }
    }
    nearestNode = Lnode;
    *nearestNodeDist = *Ldist;
    return true;
}

bool KDTree::KDFindNearestinSubtreeWithGuess(std::shared_ptr<KDTreeNode>
                                                            nearestNode,
                                             std::shared_ptr<double>
                                                            nearestNodeDist,
                                             std::shared_ptr<KDTreeNode> root,
                                             Eigen::VectorXd queryPoint,
                                             std::shared_ptr<KDTreeNode>
                                                        suggestedClosestNode,
                                             double suggestedClosestDist)
{
    std::shared_ptr<KDTreeNode> parent = root;
    std::shared_ptr<KDTreeNode> currentClosestNode = suggestedClosestNode;
    std::shared_ptr<double> currentClosestDist
            = std::make_shared<double>(suggestedClosestDist);
    while( true ) {
        if( queryPoint(parent->kd_split_) < parent->position_(parent->kd_split_) ) {
            // Traverse tree to the left
            if( !parent->kd_child_L_exist_ ) {
                // The queryPoint would be inserted as the left child of the parent
                break;
            }
            parent = parent->kd_child_L_;
            continue;
        }
        else {
            // Traverse tree to the right
            if( !parent->kd_child_R_exist_ ) {
                // The queryPoint would be inserted as the right child of the parent
                break;
            }
            parent = parent->kd_child_R_;
            continue;
        }
    }

    double newDist = this->distanceFunction( queryPoint, parent->position_);
    if( newDist < *currentClosestDist ) {
        currentClosestNode = parent;
        currentClosestDist = std::make_shared<double>(newDist);
    }

    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could plossibly be any closer nodes on the other
        // side of the parent. If not then check grandparent etc.

        double parentHyperPlaneDist = queryPoint(parent->kd_split_)
                - parent->position_(parent->kd_split_);

        if( parentHyperPlaneDist > *currentClosestDist ) {
            // Then there could not be any closer nodes on the other side of
            // the parent and the parent itself is also too far away

            if( parent == root ) {
                // The parent is the root and we are done
                nearestNode = currentClosestNode;
                nearestNodeDist = currentClosestDist;
                return true;
            }

            parent = parent->kd_parent_;
            continue;
        }

        // If we are here, then there could be a closer node on the other
        // side of the parent including the parent itself

        // First check the parent itself (if it is not already the
        // closest node)
        if( currentClosestNode != parent ) {
            newDist = this->distanceFunction(queryPoint, parent->position_);
            if( newDist < *currentClosestDist ) {
                currentClosestNode = parent;
                currentClosestDist = std::make_shared<double>(newDist);
            }
        }

        // Now check on the other side of the parent
        if( (queryPoint(parent->kd_split_) < parent->position_(parent->kd_split_))
                && parent->kd_child_R_exist_ ) {
            // queryPoint is on the left side of the parent, so we need to
            // look at the right side of it (if it exists)

            // Find right subtree distance
            std::shared_ptr<KDTreeNode> Rnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Rdist = 0;
            KDFindNearestInSubtree( Rnode, Rdist,
                                    parent->kd_child_R_, queryPoint,
                                    currentClosestNode, *currentClosestDist );

            if( *Rdist < *currentClosestDist ) {
                currentClosestNode = Rnode;
                currentClosestDist = Rdist;
            }
        }
        else if( (parent->position_(parent->kd_split_)
                  <= queryPoint(parent->kd_split_)) && parent->kd_child_L_exist_ ) {
            // queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)

            // Find left subtree distance
            std::shared_ptr<KDTreeNode> Lnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Ldist = 0;
            KDFindNearestInSubtree( Lnode, Ldist,
                                    parent->kd_child_L_, queryPoint,
                                    currentClosestNode, *currentClosestDist );
            if( *Ldist < *currentClosestDist ) {
                currentClosestNode = Lnode;
                currentClosestDist = Ldist;
            }
        }

        if( parent == root ) {
            // The parent is the root and we are done
            // Need to do one last check vs the root
            double thisDist = this->distanceFunction(queryPoint,
                                                     parent->position_);
            if( thisDist < *currentClosestDist ) {
                nearestNode = parent;
                nearestNodeDist = std::make_shared<double>(thisDist);
                return true;
            }
            nearestNode = currentClosestNode;
            nearestNodeDist = currentClosestDist;
            return true;
        }

        parent = parent->kd_parent_;
    }
}

bool KDTree::KDFindNearestWithGuess(std::shared_ptr<KDTreeNode> nearestNode,
                                    std::shared_ptr<double> nearestNodeDist,
                                    Eigen::VectorXd queryPoint,
                                    std::shared_ptr<KDTreeNode> guess )
{
    double distToGuess = this->distanceFunction(queryPoint, guess->position_);
    if( guess == this->root ) {
        KDFindNearestInSubtree( nearestNode, nearestNodeDist, this->root,
                                queryPoint, this->root, distToGuess );
        return true;
    }

    std::shared_ptr<KDTreeNode> Lnode = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> Ldist = 0;
    KDFindNearestinSubtreeWithGuess(Lnode, Ldist,
                                    this->root, queryPoint, guess,
                                    distToGuess);

    if( this->num_wraps_ > 0 ) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        std::shared_ptr<GhostPointIterator> pointIterator
                = std::make_shared<GhostPointIterator>(this, queryPoint);
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = GetNextGhostPoint( pointIterator, *Ldist );
            Eigen::VectorXd zeros( this->num_wraps_, 0 );
            if( thisGhostPoint == zeros) break;

            // Now see if any points in the space are closer to this ghost
            double distGhostToGuess = this->distanceFunction(thisGhostPoint,
                                                             guess->position_);
            std::shared_ptr<KDTreeNode> thisLnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> thisLdist = 0;
            KDFindNearestinSubtreeWithGuess( thisLnode, thisLdist,
                                             this->root, thisGhostPoint,
                                             guess, distGhostToGuess );

            if( *thisLdist < *Ldist ) {
                // Found a closer point
                Lnode = thisLnode;
                Ldist = thisLdist;
            }
        }
    }
    nearestNode = Lnode;
    nearestNodeDist = Ldist;
    return true;
}

/////////////////////// K Nearest ///////////////////////
std::shared_ptr<KDTreeNode> KDTree::AddToKNNHeap(std::shared_ptr<BinaryHeap> H,
                                              std::shared_ptr<KDTreeNode> node,
                                                 double key,
                                                 int k)
{
    std::vector<std::shared_ptr<KDTreeNode>> heap;
    H->GetHeap(heap);
    std::shared_ptr<KDTreeNode> top = std::make_shared<KDTreeNode>();
    if( node->in_heap_ ) {
        H->TopHeapB(top);
        if( top->dist_ == -1 ) {
            std::cout << "Heap node has no reference tree node!" << std::endl;
        }
        return top;
    } else if( *H->GetIndexOfLast() < k ) {
        // Just insert
        node->dist_ = key;
        H->AddToHeapB(node);
    } else if( heap[1]->dist_ > key ) {
        H->PopHeapB(top);
        node->dist_ = key;
        H->AddToHeapB(node);
    }
    H->TopHeapB(top);
    if( top->dist_ == -1 ) {
        std::cout << "Heap node has no reference tree node!" << std::endl;
    }
    return top;
}

bool KDTree::KDFindKNearestInSubtree(std::shared_ptr<KDTreeNode> farthestNode,
                                     std::shared_ptr<double> farthestNodeDist,
                                     std::shared_ptr<KDTreeNode> root,
                                     int k,
                                     Eigen::VectorXd queryPoint,
                                     std::shared_ptr<BinaryHeap> nearestHeap)
{
    // Walk down the tree as if the node would be inserted
    std::shared_ptr<KDTreeNode> parent = root;
    std::shared_ptr<KDTreeNode> currentWorstClosestNode;
    nearestHeap->TopHeapB(currentWorstClosestNode);
    std::shared_ptr<double> currentWorstClosestDist
            = std::make_shared<double>(currentWorstClosestNode->dist_);
    while( true ) {
        if( queryPoint(parent->kd_split_) < parent->position_(parent->kd_split_) ) {
            // Traverse the tree to the left
            if( !parent->kd_child_L_exist_ ) {
                // The queryPoint would be inserted as the left child of the parent
                break;
            }
            parent = parent->kd_child_L_;
            continue;
        } else {
            // Traverse the tree to the right
            if( !parent->kd_child_R_exist_ ) {
                // The queryPoint would be inserted as the right child of the parent
                break;
            }
            parent = parent->kd_child_R_;
            continue;
        }
    }

    double newDist = this->distanceFunction( queryPoint, parent->position_);
    if( newDist < *currentWorstClosestDist ) {
        currentWorstClosestNode = AddToKNNHeap(nearestHeap, parent,
                                               newDist, k);
        *currentWorstClosestDist = currentWorstClosestNode->dist_;
    }

    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could possibly be any closer nodes on the other
        // side of the parent (and the parent itself is also too far away)

        double parentHyperPlaneDist
                = queryPoint(parent->kd_split_)
                - parent->position_(parent->kd_split_);

        if( parentHyperPlaneDist > *currentWorstClosestDist ) {
            // Then there could not be any closer nodes on the other side
            // of the parent (and the parent itself is also too far away)
            if( parent == root ) {
                // The parent is te root and we are done
                *farthestNode = *currentWorstClosestNode;
                *farthestNodeDist = *currentWorstClosestDist;
                return true;
            }
            parent = parent->kd_parent_;
            continue;
        }

        // If we are here, then there could be a closer node on the other side
        // of the parent (including the parent itself)

        // First check the parent itself (if it is not already one of
        // the closest nodes)
        if( !parent->in_heap_ ) {
            newDist = this->distanceFunction(queryPoint, parent->position_);
            if( newDist < *currentWorstClosestDist ) {
                currentWorstClosestNode = AddToKNNHeap( nearestHeap, parent,
                                                        newDist, k );
                *currentWorstClosestDist = currentWorstClosestNode->dist_;
            }
        }

        // Now check the other side of the parent
        if( queryPoint(parent->kd_split_)
                < parent->position_(parent->kd_split_) && parent->kd_child_R_exist_ ) {
            // queryPoint is on the left side of the parent, so we need to look
            // at the right side of it (if it exists)
            KDFindKNearestInSubtree( currentWorstClosestNode,
                                     currentWorstClosestDist,
                                     parent->kd_child_R_, k, queryPoint,
                                     nearestHeap );
            *currentWorstClosestDist = currentWorstClosestNode->dist_;
        } else if( parent->position_(parent->kd_split_)
                   <= queryPoint(parent->kd_split_) && parent->kd_child_L_exist_ ) {
            // queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)
            KDFindKNearestInSubtree( currentWorstClosestNode,
                                     currentWorstClosestDist,
                                     parent->kd_child_L_, k, queryPoint,
                                     nearestHeap );
            *currentWorstClosestDist = currentWorstClosestNode->dist_;
        }

        if( parent == root ) {
            // The parent is the root and we are done
            *farthestNode = *currentWorstClosestNode;
            *farthestNodeDist = *currentWorstClosestDist;
        }

        parent = parent->kd_parent_;
    }
}

std::vector<std::shared_ptr<KDTreeNode>> KDTree::KDFindKNearest(int k,
                                                Eigen::VectorXd queryPoint)
{
    std::shared_ptr<BinaryHeap> Heap = std::make_shared<BinaryHeap>(true);
    // true >> use heap functions (key not keyQ) not priority queue functions

    // Insert root node in heap
    this->root->dist_ = this->distanceFunction(queryPoint, this->root->position_);
    Heap->AddToHeapB( this->root );

    // Insert a dummy node in the heap with INF key
    std::shared_ptr<KDTreeNode> dummyNode
            = std::make_shared<KDTreeNode>((double)INF);
    Heap->AddToHeapB( dummyNode );

    // Find k nearest neighbors
    std::shared_ptr<KDTreeNode> farthestNearestNode
            = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> farthestNearestNodeDist
            = std::make_shared<double>(0);
    KDFindKNearestInSubtree( farthestNearestNode, farthestNearestNodeDist,
                             this->root, k,
                             queryPoint, Heap );

    if( this->num_wraps_ > 0 ) {
        std::cout << "ERROR: knn search not implemented for wrapped space"
                  << std::endl;
    }

    // Remove the dummy node if still there (guarenteed to be on top, due to INF key)
    std::shared_ptr<KDTreeNode> topNode = std::make_shared<KDTreeNode>();
    Heap->TopHeapB(topNode);
    if( topNode == dummyNode ) Heap->PopHeapB(topNode);
    std::vector<std::shared_ptr<KDTreeNode>> dHeap;
    Heap->CleanHeapB(dHeap);
    return dHeap;
}

/////////////////////// Within Range ///////////////////////

bool KDTree::AddToRangeList(std::shared_ptr<JList> &S,
                            std::shared_ptr<KDTreeNode> &node,
                            double key)
{
    // "in_heap_" is a misnomer because this is a list
    if( node->in_heap_ ) {
        // Node already in list
        return false;
    }
    node->in_heap_ = true;
    S->JListPush( node, key );
    return true;
}

void KDTree::PopFromRangeList(std::shared_ptr<JList> &S,
                              std::shared_ptr<KDTreeNode> &t,
                              std::shared_ptr<double> k)
{
    S->JListPopKey(t,k);
    t->in_heap_ = false;
}

void KDTree::EmptyRangeList(std::shared_ptr<JList>& S)
{
    std::shared_ptr<KDTreeNode> n = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> k = std::make_shared<double>(0);
    while( S->length_ > 0 ) {
        S->JListPopKey( n, k );
        n->in_heap_ = false;
    }

}

bool KDTree::KDFindWithinRangeInSubtree(std::shared_ptr<KDTreeNode> &root,
                                        double range,
                                        Eigen::VectorXd queryPoint,
                                        std::shared_ptr<JList> &nodeList)
{
    // Walk down the tree as if the node would be inserted
    std::shared_ptr<KDTreeNode> parent = root;
    while(true) {
        if(queryPoint(parent->kd_split_) < parent->position_(parent->kd_split_)) {
            // Traverse tree to the left
            if(!parent->kd_child_L_exist_) {
                // The queryPoint would be inserted as
                // the left child of the parent
                break;
            }
            parent = parent->kd_child_L_;
            continue;
        } else {
            // Traverse tree to the right
            if(!parent->kd_child_R_exist_) {
                // The queryPoint would be inserted as
                // the right child of the parent
                break;
            }
            parent = parent->kd_child_R_;
            continue;
        }
    }

    double newDist = this->distanceFunction(queryPoint, parent->position_);
    if(newDist < range) {
        AddToRangeList(nodeList, parent, newDist);
    }

    // Now walk back up the tree (will break out when done)
    while(true) {
        // Now check if there could possibly be any nodes on the other
        // side of the parent within range, if not then check grandparent etc.


        /// NOT SURE WHAT THIS MEASURES
        double parentHyperPlaneDist = queryPoint(parent->kd_split_)
                                        - parent->position_(parent->kd_split_);

        if(parentHyperPlaneDist > range) {
            // Then there could not be any closer nodes within range on the other
            // side of the parent (and the parent itself is also too far away
            if(parent == root) {
                // The parent is the root and we are done
                return true;
            }
            parent = parent->kd_parent_;
            continue;
        }

        // If we are here, then there could be a closer node on the other side
        // of the parent (including the parent itself) that is within range

        // First check the parent itself (if it is not already one
        // of the closest nodes)
        if(!parent->in_heap_) { // in_heap_ is a misnomer since this is a list
            newDist = this->distanceFunction(queryPoint, parent->position_);
            if(newDist < range) {
                AddToRangeList(nodeList, parent, newDist);
            }
        }

        // Now check on the other side of the parent
        if(queryPoint(parent->kd_split_) < parent->position_(parent->kd_split_)
                && parent->kd_child_R_exist_) {
            // The queryPoint is on the left side of the porent, so we need to
            // look at the right side of it (if it exists)
            KDFindWithinRangeInSubtree(parent->kd_child_R_, range,
                                       queryPoint, nodeList);
        } else if(parent->position_(parent->kd_split_)
                  <= queryPoint(parent->kd_split_) && parent->kd_child_L_exist_) {
            // The queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)
            KDFindWithinRangeInSubtree(parent->kd_child_L_, range,
                                       queryPoint, nodeList);
        }

        if(parent == root) {
            // The parent is the root and we are done
            return true;
        }

        parent = parent->kd_parent_;
    }
}

void KDTree::KDFindWithinRange(std::shared_ptr<JList> &S,
                               double range,
                               Eigen::VectorXd queryPoint)
{
//    std::cout << "KDFindWithinRange" << std::endl;
    // Insert root node in list if it is within range
    double distToRoot
            = this->distanceFunction(queryPoint, this->root->position_);
    if(distToRoot <= range) AddToRangeList(S, this->root, distToRoot);

    // Find nodes within range
    KDFindWithinRangeInSubtree(this->root, range, queryPoint, S);

//    std::cout << "Found " << S->length_ << " points in Range " << range << std::endl;

    if(this->num_wraps_ > 0) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        std::shared_ptr<GhostPointIterator> pointIterator
                = std::make_shared<GhostPointIterator>(this, queryPoint);
        while(true) {
            Eigen::VectorXd thisGhostPoint
                    = GetNextGhostPoint(pointIterator, range);
            if(thisGhostPoint.isZero(0)) break;

            // Now see if any points in the space are closer to this ghost
            KDFindWithinRangeInSubtree(this->root, range, thisGhostPoint, S);
        }
    }

//    std::cout << "Found " << S->length_ << " points" << std::endl;
}

void KDTree::KDFindMoreWithinRange(std::shared_ptr<JList> &L,
                                   double range, Eigen::VectorXd queryPoint)
{
    // Insert root node in list if it is within range
    double distToRoot
            = this->distanceFunction(queryPoint, this->root->position_);
    if( distToRoot <= range ) {
        AddToRangeList( L, this->root, distToRoot );
    }

    // Find nodes within range
    KDFindWithinRangeInSubtree( this->root, range, queryPoint, L );

    if( this->num_wraps_ > 0 ) {
        // If dimensions wrap around, need to search vs. identities (ghosts)
        std::shared_ptr<GhostPointIterator> pointIterator
                = std::make_shared<GhostPointIterator>(this, queryPoint);
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = GetNextGhostPoint( pointIterator, range );
            if( thisGhostPoint.isZero(0) ) {
                break;
            }

            // Now see if any points in the space are closer to this ghost
            KDFindWithinRangeInSubtree( this->root, range, thisGhostPoint, L );
        }
    }

}

void KDTree::KDInsert(Eigen::VectorXd a)
{
    std::shared_ptr<KDTreeNode> N = std::make_shared<KDTreeNode>();
    N->position_ = a;
    this->KDInsert(N);
}

#endif // KDTREE_CPP
