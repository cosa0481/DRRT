/* kdtree.cpp
 * Implemented for C++ by: Corin Sandford
 * Original Julia code by: Michael Otte
 * Fall 2016
 */

#ifndef KDTREE_CPP
#define KDTREE_CPP

#include <DRRT/kdtree.h>
#include <iostream>

void KDTree::addTreetoPQ(std::shared_ptr<Queue> &Q,
                         std::shared_ptr<KDTreeNode> &node)
{
    Q->Q->addToHeap(node);
    if(node->kdChildLExist || node->kdChildRExist) {
        if(node->kdChildLExist) addTreetoPQ(Q, node->kdChildL);
        if(node->kdChildRExist) addTreetoPQ(Q, node->kdChildR);
    }
}

void KDTree::printTree(std::shared_ptr<KDTreeNode> node,
                       int indent, char type)
{
    if(indent) std::cout << std::string(indent-1,' ') << type;
    std::cout << node->position(0) << ","
              << node->position(1) << ": "
              << node->rrtLMC;
    if( node->rrtParentUsed ) {
        std::cout << " : " << node->rrtParentEdge->endNode->position(0) << ","
                  << node->rrtParentEdge->endNode->position(1);
        std::cout << " (" << node->rrtParentEdge->dist << ")";
    }
    if( !node->kdChildLExist && !node->kdChildRExist ) {
        std::cout << " | leaf" << std::endl;
    } else {
        std::cout << std::endl;
        if(node->kdChildLExist) {
            printTree(node->kdChildL, indent+4, '<');
        }
        if(node->kdChildRExist) {
            printTree(node->kdChildR, indent+4, '>');
        }
    }
}

bool KDTree::kdInsert(std::shared_ptr<KDTreeNode>& node)
{
    if( node->kdInTree ) return false;
    node->kdInTree = true;

    if( this->treeSize == 0 ) {
        this->root = node;
        this->root->kdSplit = 0;
        this->treeSize = 1;
        return true;
    }

    // Figure out where to put this node
    std::shared_ptr<KDTreeNode> parent = this->root;
    while( true ) {
        if(node->position(parent->kdSplit)
                < parent->position(parent->kdSplit)) {
            // Traverse tree to the left
            if( !parent->kdChildLExist ) {
                // The node gets inserted as the left child of the parent
                parent->kdChildL = node;
                parent->kdChildLExist = true;
                break;
            }
            parent = parent->kdChildL;
            continue;
        }
        else {
            // Traverse tree to the right
            if( !parent->kdChildRExist ) {
                // The node gets inserted as the right child of the parent
                parent->kdChildR = node;
                parent->kdChildRExist = true;
                break;
            }
            parent = parent->kdChildR;
            continue;
        }
    }

    node->kdParent = parent;
    node->kdParentExist = true;
    if( parent->kdSplit == this->d-1 ) { node->kdSplit = 0; }
    else { node->kdSplit = parent->kdSplit + 1; }

    this->treeSize += 1;
    return true;
}

/////////////////////// Nearest ///////////////////////

bool KDTree::kdFindNearestInSubtree(std::shared_ptr<KDTreeNode>& nearestNode,
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
        if( queryPoint(parent->kdSplit) < parent->position(parent->kdSplit) ) {
            // Traverse tree to the left
            if( !parent->kdChildLExist ) {
                // The queryPoint would be inserted as the left child of the parent
                break;
            }
            parent = parent->kdChildL;
            continue;
        }
        else {
            // Traverse tree to the right
            if( !parent->kdChildRExist ) {
                // The queryPoint would be inserted as the right child of the parent
                break;
            }
            parent = parent->kdChildR;
            continue;
        }
    }

    double newDist = this->distanceFunction(queryPoint, parent->position );
    if( newDist < *currentClosestDist ) {
//        std::cout << "nearest0 (no return)\n" << parent.get()->position << std::endl;
        currentClosestNode = parent;
        currentClosestDist = std::make_shared<double>(newDist);
    }


    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could plossibly be any closer nodes on the other
        // side of the parent. If not then check grandparent etc.
        double parentHyperPlaneDist = queryPoint(parent->kdSplit)
                - parent->position(parent->kdSplit);

        if( parentHyperPlaneDist > *currentClosestDist ) {
            // Then there could not be any closer nodes on the other side of
            // the parent and the parent itself is also too far away

            if( parent == root ) {
                // The parent is the root and we are done
                nearestNode = currentClosestNode;
                *nearestNodeDist = *currentClosestDist;
                return true;
            }

            parent = parent->kdParent;
            continue;
        }

        // If we are here, then there could be a closer node on the other
        // side of the parent including the parent itself

        // First check the parent itself (if it is not already the
        // closest node)
        if( currentClosestNode != parent ) {
            newDist = this->distanceFunction( queryPoint, parent->position);
            if( newDist < *currentClosestDist ) {
                currentClosestNode = parent;
                currentClosestDist = std::make_shared<double>(newDist);
            }
        }

        // Now check on the other side of the parent
        if( (queryPoint(parent->kdSplit) < parent->position(parent->kdSplit))
                && parent->kdChildRExist ) {
            // queryPoint is on the left side of the parent, so we need to
            // look at the right side of it (if it exists)

            // Find right subtree distance
            std::shared_ptr<KDTreeNode> Rnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Rdist = std::make_shared<double>(0);
            kdFindNearestInSubtree( Rnode, Rdist,
                                    parent->kdChildR, queryPoint,
                                    currentClosestNode, *currentClosestDist );
            if( *Rdist < *currentClosestDist ) {
                currentClosestNode = Rnode;
                currentClosestDist = Rdist;
            }
        }
        else if( (parent->position(parent->kdSplit)
                  <= queryPoint(parent->kdSplit))
                 && parent->kdChildLExist ) {
            // queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)

            // Find left subtree distance
            std::shared_ptr<KDTreeNode> Lnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Ldist = std::make_shared<double>(0);
            kdFindNearestInSubtree( Lnode, Ldist,
                                    parent->kdChildL, queryPoint,
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
        parent = parent->kdParent;
    }
}

bool KDTree::kdFindNearest(std::shared_ptr<KDTreeNode>& nearestNode,
                           std::shared_ptr<double> nearestNodeDist,
                           Eigen::VectorXd queryPoint)
{
    // Initial search (only search if the space does not wrap around)
    double distToRoot = this->distanceFunction(queryPoint,
                                               this->root->position);
    std::shared_ptr<KDTreeNode> Lnode
            = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> Ldist = std::make_shared<double>(0);
    kdFindNearestInSubtree( Lnode, Ldist, this->root,
                            queryPoint, this->root, distToRoot );

    if( this->numWraps > 0 ) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        std::shared_ptr<ghostPointIterator> pointIterator
                = std::make_shared<ghostPointIterator>
                (ghostPointIterator( this, queryPoint ));
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = getNextGhostPoint( pointIterator, *Ldist );
            if( thisGhostPoint.isZero(0) ) break;

            // Now see if any points in the space are closer to this ghost
            double distGhostToRoot = this->distanceFunction(thisGhostPoint,
                                                        this->root->position);
            std::shared_ptr<KDTreeNode> thisLnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> thisLdist = std::make_shared<double>(0);
            kdFindNearestInSubtree( thisLnode, thisLdist, this->root,
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

bool KDTree::kdFindNearestinSubtreeWithGuess(std::shared_ptr<KDTreeNode>
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
        if( queryPoint(parent->kdSplit) < parent->position(parent->kdSplit) ) {
            // Traverse tree to the left
            if( !parent->kdChildLExist ) {
                // The queryPoint would be inserted as the left child of the parent
                break;
            }
            parent = parent->kdChildL;
            continue;
        }
        else {
            // Traverse tree to the right
            if( !parent->kdChildRExist ) {
                // The queryPoint would be inserted as the right child of the parent
                break;
            }
            parent = parent->kdChildR;
            continue;
        }
    }

    double newDist = this->distanceFunction( queryPoint, parent->position);
    if( newDist < *currentClosestDist ) {
        currentClosestNode = parent;
        currentClosestDist = std::make_shared<double>(newDist);
    }

    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could plossibly be any closer nodes on the other
        // side of the parent. If not then check grandparent etc.

        double parentHyperPlaneDist = queryPoint(parent->kdSplit)
                - parent->position(parent->kdSplit);

        if( parentHyperPlaneDist > *currentClosestDist ) {
            // Then there could not be any closer nodes on the other side of
            // the parent and the parent itself is also too far away

            if( parent == root ) {
                // The parent is the root and we are done
                nearestNode = currentClosestNode;
                nearestNodeDist = currentClosestDist;
                return true;
            }

            parent = parent->kdParent;
            continue;
        }

        // If we are here, then there could be a closer node on the other
        // side of the parent including the parent itself

        // First check the parent itself (if it is not already the
        // closest node)
        if( currentClosestNode != parent ) {
            newDist = this->distanceFunction(queryPoint, parent->position);
            if( newDist < *currentClosestDist ) {
                currentClosestNode = parent;
                currentClosestDist = std::make_shared<double>(newDist);
            }
        }

        // Now check on the other side of the parent
        if( (queryPoint(parent->kdSplit) < parent->position(parent->kdSplit))
                && parent->kdChildRExist ) {
            // queryPoint is on the left side of the parent, so we need to
            // look at the right side of it (if it exists)

            // Find right subtree distance
            std::shared_ptr<KDTreeNode> Rnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Rdist = 0;
            kdFindNearestInSubtree( Rnode, Rdist,
                                    parent->kdChildR, queryPoint,
                                    currentClosestNode, *currentClosestDist );

            if( *Rdist < *currentClosestDist ) {
                currentClosestNode = Rnode;
                currentClosestDist = Rdist;
            }
        }
        else if( (parent->position(parent->kdSplit)
                  <= queryPoint(parent->kdSplit)) && parent->kdChildLExist ) {
            // queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)

            // Find left subtree distance
            std::shared_ptr<KDTreeNode> Lnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Ldist = 0;
            kdFindNearestInSubtree( Lnode, Ldist,
                                    parent->kdChildL, queryPoint,
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
                                                     parent->position);
            if( thisDist < *currentClosestDist ) {
                nearestNode = parent;
                nearestNodeDist = std::make_shared<double>(thisDist);
                return true;
            }
            nearestNode = currentClosestNode;
            nearestNodeDist = currentClosestDist;
            return true;
        }

        parent = parent->kdParent;
    }
}

bool KDTree::kdFindNearestWithGuess(std::shared_ptr<KDTreeNode> nearestNode,
                                    std::shared_ptr<double> nearestNodeDist,
                                    Eigen::VectorXd queryPoint,
                                    std::shared_ptr<KDTreeNode> guess )
{
    double distToGuess = this->distanceFunction(queryPoint, guess->position);
    if( guess == this->root ) {
        kdFindNearestInSubtree( nearestNode, nearestNodeDist, this->root,
                                queryPoint, this->root, distToGuess );
        return true;
    }

    std::shared_ptr<KDTreeNode> Lnode = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> Ldist = 0;
    kdFindNearestinSubtreeWithGuess(Lnode, Ldist,
                                    this->root, queryPoint, guess,
                                    distToGuess);

    if( this->numWraps > 0 ) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        std::shared_ptr<ghostPointIterator> pointIterator
                = std::make_shared<ghostPointIterator>(this, queryPoint);
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = getNextGhostPoint( pointIterator, *Ldist );
            Eigen::VectorXd zeros( this->numWraps, 0 );
            if( thisGhostPoint == zeros) break;

            // Now see if any points in the space are closer to this ghost
            double distGhostToGuess = this->distanceFunction(thisGhostPoint,
                                                             guess->position);
            std::shared_ptr<KDTreeNode> thisLnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> thisLdist = 0;
            kdFindNearestinSubtreeWithGuess( thisLnode, thisLdist,
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
std::shared_ptr<KDTreeNode> KDTree::addToKNNHeap(std::shared_ptr<BinaryHeap> H,
                                              std::shared_ptr<KDTreeNode> node,
                                                 double key,
                                                 int k)
{
    std::vector<std::shared_ptr<KDTreeNode>> heap;
    H->getHeap(heap);
    std::shared_ptr<KDTreeNode> top = std::make_shared<KDTreeNode>();
    if( node->inHeap ) {
        H->topHeapB(top);
        if( top->dist == -1 ) {
            std::cout << "Heap node has no reference tree node!" << std::endl;
        }
        return top;
    } else if( *H->getIndexOfLast() < k ) {
        // Just insert
        node->dist = key;
        H->addToHeapB(node);
    } else if( heap[1]->dist > key ) {
        H->popHeapB(top);
        node->dist = key;
        H->addToHeapB(node);
    }
    H->topHeapB(top);
    if( top->dist == -1 ) {
        std::cout << "Heap node has no reference tree node!" << std::endl;
    }
    return top;
}

bool KDTree::kdFindKNearestInSubtree(std::shared_ptr<KDTreeNode> farthestNode,
                                     std::shared_ptr<double> farthestNodeDist,
                                     std::shared_ptr<KDTreeNode> root,
                                     int k,
                                     Eigen::VectorXd queryPoint,
                                     std::shared_ptr<BinaryHeap> nearestHeap)
{
    // Walk down the tree as if the node would be inserted
    std::shared_ptr<KDTreeNode> parent = root;
    std::shared_ptr<KDTreeNode> currentWorstClosestNode;
    nearestHeap->topHeapB(currentWorstClosestNode);
    std::shared_ptr<double> currentWorstClosestDist
            = std::make_shared<double>(currentWorstClosestNode->dist);
    while( true ) {
        if( queryPoint(parent->kdSplit) < parent->position(parent->kdSplit) ) {
            // Traverse the tree to the left
            if( !parent->kdChildLExist ) {
                // The queryPoint would be inserted as the left child of the parent
                break;
            }
            parent = parent->kdChildL;
            continue;
        } else {
            // Traverse the tree to the right
            if( !parent->kdChildRExist ) {
                // The queryPoint would be inserted as the right child of the parent
                break;
            }
            parent = parent->kdChildR;
            continue;
        }
    }

    double newDist = this->distanceFunction( queryPoint, parent->position);
    if( newDist < *currentWorstClosestDist ) {
        currentWorstClosestNode = addToKNNHeap(nearestHeap, parent,
                                               newDist, k);
        *currentWorstClosestDist = currentWorstClosestNode->dist;
    }

    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could possibly be any closer nodes on the other
        // side of the parent (and the parent itself is also too far away)

        double parentHyperPlaneDist
                = queryPoint(parent->kdSplit)
                - parent->position(parent->kdSplit);

        if( parentHyperPlaneDist > *currentWorstClosestDist ) {
            // Then there could not be any closer nodes on the other side
            // of the parent (and the parent itself is also too far away)
            if( parent == root ) {
                // The parent is te root and we are done
                *farthestNode = *currentWorstClosestNode;
                *farthestNodeDist = *currentWorstClosestDist;
                return true;
            }
            parent = parent->kdParent;
            continue;
        }

        // If we are here, then there could be a closer node on the other side
        // of the parent (including the parent itself)

        // First check the parent itself (if it is not already one of
        // the closest nodes)
        if( !parent->inHeap ) {
            newDist = this->distanceFunction(queryPoint, parent->position);
            if( newDist < *currentWorstClosestDist ) {
                currentWorstClosestNode = addToKNNHeap( nearestHeap, parent,
                                                        newDist, k );
                *currentWorstClosestDist = currentWorstClosestNode->dist;
            }
        }

        // Now check the other side of the parent
        if( queryPoint(parent->kdSplit)
                < parent->position(parent->kdSplit) && parent->kdChildRExist ) {
            // queryPoint is on the left side of the parent, so we need to look
            // at the right side of it (if it exists)
            kdFindKNearestInSubtree( currentWorstClosestNode,
                                     currentWorstClosestDist,
                                     parent->kdChildR, k, queryPoint,
                                     nearestHeap );
            *currentWorstClosestDist = currentWorstClosestNode->dist;
        } else if( parent->position(parent->kdSplit)
                   <= queryPoint(parent->kdSplit) && parent->kdChildLExist ) {
            // queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)
            kdFindKNearestInSubtree( currentWorstClosestNode,
                                     currentWorstClosestDist,
                                     parent->kdChildL, k, queryPoint,
                                     nearestHeap );
            *currentWorstClosestDist = currentWorstClosestNode->dist;
        }

        if( parent == root ) {
            // The parent is the root and we are done
            *farthestNode = *currentWorstClosestNode;
            *farthestNodeDist = *currentWorstClosestDist;
        }

        parent = parent->kdParent;
    }
}

std::vector<std::shared_ptr<KDTreeNode>> KDTree::kdFindKNearest(int k,
                                                Eigen::VectorXd queryPoint)
{
    std::shared_ptr<BinaryHeap> Heap = std::make_shared<BinaryHeap>(true);
    // true >> use heap functions (key not keyQ) not priority queue functions

    // Insert root node in heap
    this->root->dist = this->distanceFunction(queryPoint, this->root->position);
    Heap->addToHeapB( this->root );

    // Insert a dummy node in the heap with INF key
    std::shared_ptr<KDTreeNode> dummyNode
            = std::make_shared<KDTreeNode>((double)INF);
    Heap->addToHeapB( dummyNode );

    // Find k nearest neighbors
    std::shared_ptr<KDTreeNode> farthestNearestNode
            = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> farthestNearestNodeDist
            = std::make_shared<double>(0);
    kdFindKNearestInSubtree( farthestNearestNode, farthestNearestNodeDist,
                             this->root, k,
                             queryPoint, Heap );

    if( this->numWraps > 0 ) {
        std::cout << "ERROR: knn search not implemented for wrapped space"
                  << std::endl;
    }

    // Remove the dummy node if still there (guarenteed to be on top, due to INF key)
    std::shared_ptr<KDTreeNode> topNode = std::make_shared<KDTreeNode>();
    Heap->topHeapB(topNode);
    if( topNode == dummyNode ) Heap->popHeapB(topNode);
    std::vector<std::shared_ptr<KDTreeNode>> dHeap;
    Heap->cleanHeapB(dHeap);
    return dHeap;
}

/////////////////////// Within Range ///////////////////////

bool KDTree::addToRangeList(std::shared_ptr<JList> &S,
                            std::shared_ptr<KDTreeNode> &node,
                            double key)
{
    // "inHeap" is a misnomer because this is a list
    if( node->inHeap ) {
        // Node already in list
        return false;
    }
    node->inHeap = true;
    S->JlistPush( node, key );
    return true;
}

void KDTree::popFromRangeList(std::shared_ptr<JList> &S,
                              std::shared_ptr<KDTreeNode> &t,
                              std::shared_ptr<double> k)
{
    S->JlistPopKey(t,k);
    t->inHeap = false;
}

void KDTree::emptyRangeList(std::shared_ptr<JList>& S)
{
    std::shared_ptr<KDTreeNode> n = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> k = std::make_shared<double>(0);
    while( S->length > 0 ) {
        S->JlistPopKey( n, k );
        n->inHeap = false;
    }

}

bool KDTree::kdFindWithinRangeInSubtree(std::shared_ptr<KDTreeNode> &root,
                                        double range,
                                        Eigen::VectorXd queryPoint,
                                        std::shared_ptr<JList> &nodeList)
{
    // Walk down the tree as if the node would be inserted
    std::shared_ptr<KDTreeNode> parent = root;
    while(true) {
        if(queryPoint(parent->kdSplit) < parent->position(parent->kdSplit)) {
            // Traverse tree to the left
            if(!parent->kdChildLExist) {
                // The queryPoint would be inserted as
                // the left child of the parent
                break;
            }
            parent = parent->kdChildL;
            continue;
        } else {
            // Traverse tree to the right
            if(!parent->kdChildRExist) {
                // The queryPoint would be inserted as
                // the right child of the parent
                break;
            }
            parent = parent->kdChildR;
            continue;
        }
    }

    double newDist = this->distanceFunction(queryPoint, parent->position);
    if(newDist < range) {
        addToRangeList(nodeList, parent, newDist);
    }

    // Now walk back up the tree (will break out when done)
    while(true) {
        // Now check if there could possibly be any nodes on the other
        // side of the parent within range, if not then check grandparent etc.


        /// NOT SURE WHAT THIS MEASURES
        double parentHyperPlaneDist = queryPoint(parent->kdSplit)
                                        - parent->position(parent->kdSplit);

        if(parentHyperPlaneDist > range) {
            // Then there could not be any closer nodes within range on the other
            // side of the parent (and the parent itself is also too far away
            if(parent == root) {
                // The parent is the root and we are done
                return true;
            }
            parent = parent->kdParent;
            continue;
        }

        // If we are here, then there could be a closer node on the other side
        // of the parent (including the parent itself) that is within range

        // First check the parent itself (if it is not already one
        // of the closest nodes)
        if(!parent->inHeap) { // inHeap is a misnomer since this is a list
            newDist = this->distanceFunction(queryPoint, parent->position);
            if(newDist < range) {
                addToRangeList(nodeList, parent, newDist);
            }
        }

        // Now check on the other side of the parent
        if(queryPoint(parent->kdSplit) < parent->position(parent->kdSplit)
                && parent->kdChildRExist) {
            // The queryPoint is on the left side of the porent, so we need to
            // look at the right side of it (if it exists)
            kdFindWithinRangeInSubtree(parent->kdChildR, range,
                                       queryPoint, nodeList);
        } else if(parent->position(parent->kdSplit)
                  <= queryPoint(parent->kdSplit) && parent->kdChildLExist) {
            // The queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)
            kdFindWithinRangeInSubtree(parent->kdChildL, range,
                                       queryPoint, nodeList);
        }

        if(parent == root) {
            // The parent is the root and we are done
            return true;
        }

        parent = parent->kdParent;
    }
}

void KDTree::kdFindWithinRange(std::shared_ptr<JList> &S,
                               double range,
                               Eigen::VectorXd queryPoint)
{
    // Insert root node in list if it is within range
    double distToRoot
            = this->distanceFunction(queryPoint, this->root->position);
    if(distToRoot <= range) addToRangeList(S, this->root, distToRoot);

    // Find nodes within range
    kdFindWithinRangeInSubtree(this->root, range, queryPoint, S);

    if(this->numWraps > 0) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        std::shared_ptr<ghostPointIterator> pointIterator
                = std::make_shared<ghostPointIterator>(this, queryPoint);
        while(true) {
            Eigen::VectorXd thisGhostPoint
                    = getNextGhostPoint(pointIterator, range);
            if(thisGhostPoint.isZero(0)) break;

            // Now see if any points in the space are closer to this ghost
            kdFindWithinRangeInSubtree(this->root, range, thisGhostPoint, S);
        }
    }
}

void KDTree::kdFindMoreWithinRange(std::shared_ptr<JList> &L,
                                   double range, Eigen::VectorXd queryPoint)
{
    // Insert root node in list if it is within range
    double distToRoot
            = this->distanceFunction(queryPoint, this->root->position);
    if( distToRoot <= range ) {
        addToRangeList( L, this->root, distToRoot );
    }

    // Find nodes within range
    kdFindWithinRangeInSubtree( this->root, range, queryPoint, L );

    if( this->numWraps > 0 ) {
        // If dimensions wrap around, need to search vs. identities (ghosts)
        std::shared_ptr<ghostPointIterator> pointIterator
                = std::make_shared<ghostPointIterator>(this, queryPoint);
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = getNextGhostPoint( pointIterator, range );
            if( thisGhostPoint.isZero(0) ) {
                break;
            }

            // Now see if any points in the space are closer to this ghost
            kdFindWithinRangeInSubtree( this->root, range, thisGhostPoint, L );
        }
    }

}

void KDTree::kdInsert(Eigen::VectorXd a)
{
    std::shared_ptr<KDTreeNode> N = std::make_shared<KDTreeNode>();
    N->position = a;
    this->kdInsert(N);
}

#endif // KDTREE_CPP
