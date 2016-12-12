/* kdtree.cpp
 * Implemented for C++ by: Corin Sandford
 * Original Julia code by: Michael Otte
 * Fall 2016
 */

#ifndef KDTREE_CPP
#define KDTREE_CPP

#include <DRRT/kdtree.h>

double distFunc( std::string distanceFunction,
                 Eigen::VectorXd pointA, Eigen::VectorXd pointB)
{
    if( distanceFunction == "R3Dist" ) {
        return sqrt((pointA[1]-pointB[1])*(pointA[1]-pointB[1])
                + (pointA[2]-pointB[2])*(pointA[2]-pointB[2]) ) +
                sqrt((pointA[3]-pointB[3])*(pointA[3]-pointB[3])
                + (pointA[4]-pointB[4])*(pointA[4]-pointB[4]) ) +
                sqrt((pointA[5]-pointB[5])*(pointA[5]-pointB[5])
                + (pointA[6]-pointB[6])*(pointA[6]-pointB[6]) );
    } else if( distanceFunction == "R3SDist" ) {
        return R3SDist(pointA,pointB);
    } else if( distanceFunction == "EuclideanDist" ) {
        return EuclideanDist(pointA,pointB);
    } else if( distanceFunction == "threeTwoDRobotDist" ) {
        // drrt.h >> distance(), Wdist()
        return EuclideanDist(pointA,pointB);
    } else if( distanceFunction == "KDdist" ) {
        return R3SDist(pointA,pointB);
    }

    return 0.0;
}

void KDTreeInit( KDTree* K, int d, std::string distanceFunction )
{
    K->d = d;
    K->distanceFunction = distanceFunction;
    K->treeSize = 0;
}

bool kdInsert( KDTree* tree, KDTreeNode* node )
{
    if( node->kdInTree ) return false;
    node->kdInTree = true;

    if( tree->treeSize == 0 ) {
        tree->root = node;
        tree->root->kdSplit = 1;
        tree->treeSize = 1;
        return true;
    }

    // Figure out where to put this node
    KDTreeNode* parent = tree->root;
    while( true ) {
        if( node->position(parent->kdSplit) < parent->position(parent->kdSplit) ) {
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
    if( parent->kdSplit == tree->d-1 ) {
        node->kdSplit = 0;
    }
    else {
        node->kdSplit = parent->kdSplit + 1;
    }
    tree->treeSize += 1;
    return true;
}

/////////////////////// Nearest ///////////////////////

bool kdFindNearestInSubtree( KDTreeNode& nearestNode,
                             double &nearestNodeDist,
                             std::string distanceFunction,
                             KDTreeNode* root,
                             Eigen::VectorXd queryPoint,
                             KDTreeNode* suggestedClosestNode,
                             double suggestedClosestDist )
{
    KDTreeNode* parent = root;
    KDTreeNode* currentClosestNode = suggestedClosestNode;
    double currentClosestDist = suggestedClosestDist;
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

    double newDist = distFunc( distanceFunction, queryPoint, parent->position );
    if( newDist < currentClosestDist ) {
        currentClosestNode = parent;
        currentClosestDist = newDist;
    }


    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could plossibly be any closer nodes on the other
        // side of the parent. If not then check grandparent etc.
        double parentHyperPlaneDist = queryPoint(parent->kdSplit) - parent->position(parent->kdSplit);

        if( parentHyperPlaneDist > currentClosestDist ) {
            // Then there could not be any closer nodes on the other side of
            // the parent and the parent itself is also too far away

            if( parent == root ) {
                // The parent is the root and we are done
                nearestNode = *currentClosestNode;
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
            newDist = distFunc( distanceFunction, queryPoint, parent->position );
            if( newDist < currentClosestDist ) {
                currentClosestNode = parent;
                currentClosestDist = newDist;
            }
        }

        // Now check on the other side of the parent
        if( (queryPoint(parent->kdSplit) < parent->position(parent->kdSplit)) && parent->kdChildRExist ) {
            // queryPoint is on the left side of the parent, so we need to
            // look at the right side of it (if it exists)

            // Find right subtree distance
            KDTreeNode Rnode = KDTreeNode();
            double Rdist = 0;
            kdFindNearestInSubtree( Rnode, Rdist, distanceFunction, parent->kdChildR, queryPoint, currentClosestNode, currentClosestDist );

            if( Rdist < currentClosestDist ) {
                currentClosestNode = &Rnode;
                currentClosestDist = Rdist;
            }
        }
        else if( (parent->position(parent->kdSplit) <= queryPoint(parent->kdSplit)) && parent->kdChildLExist ) {
            // queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)

            // Find left subtree distance
            KDTreeNode Lnode = KDTreeNode();
            double Ldist = 0;
            kdFindNearestInSubtree( Lnode, Ldist, distanceFunction, parent->kdChildL, queryPoint, currentClosestNode, currentClosestDist );
            if( Ldist < currentClosestDist ) {
                currentClosestNode = &Lnode;
                currentClosestDist = Ldist;
            }
        }

        if( parent == root ) {
            // The parent is the root and we are done
            nearestNode = *currentClosestNode;
            nearestNodeDist = currentClosestDist;
            return true;
        }

        parent = parent->kdParent;
    }
}

bool kdFindNearest( KDTreeNode &nearestNode, double &nearestNodeDist,
                    KDTree* tree, Eigen::VectorXd queryPoint )
{
    // Initial search (only search if the space does not wrap around)
    double distToRoot = distFunc( tree->distanceFunction, queryPoint,
                                  tree->root->position );
    KDTreeNode Lnode = KDTreeNode();
    double Ldist = 0;
    kdFindNearestInSubtree( Lnode, Ldist, tree->distanceFunction, tree->root,
                            queryPoint, tree->root, distToRoot );


    if( tree->numWraps > 0 ) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        ghostPointIterator* pointIterator = new ghostPointIterator( tree, queryPoint );
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = getNextGhostPoint( pointIterator, Ldist );
            if( thisGhostPoint.isZero(0) ) break;

            // Now see if any points in the space are closer to this ghost
            double distGhostToRoot = distFunc( tree->distanceFunction, thisGhostPoint,
                                               tree->root->position );
            KDTreeNode thisLnode = KDTreeNode();
            double thisLdist = 0;
            kdFindNearestInSubtree( thisLnode, thisLdist, tree->distanceFunction,
                                    tree->root, thisGhostPoint, tree->root,
                                    distGhostToRoot );

            if( thisLdist < Ldist ) {
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

bool kdFindNearestinSubtreeWithGuess( KDTreeNode* nearestNode,
                                      double &nearestNodeDist,
                                      std::string distanceFunction,
                                      KDTreeNode* root,
                                      Eigen::VectorXd queryPoint,
                                      KDTreeNode* suggestedClosestNode,
                                      double suggestedClosestDist )
{
    KDTreeNode* parent = root;
    KDTreeNode* currentClosestNode = suggestedClosestNode;
    double currentClosestDist = suggestedClosestDist;
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

    double newDist = distFunc( distanceFunction, queryPoint, parent->position );
    if( newDist < currentClosestDist ) {
        currentClosestNode = parent;
        currentClosestDist = newDist;
    }

    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could plossibly be any closer nodes on the other
        // side of the parent. If not then check grandparent etc.

        double parentHyperPlaneDist = queryPoint(parent->kdSplit) - parent->position(parent->kdSplit);

        if( parentHyperPlaneDist > currentClosestDist ) {
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
            newDist = distFunc( distanceFunction, queryPoint, parent->position );
            if( newDist < currentClosestDist ) {
                currentClosestNode = parent;
                currentClosestDist = newDist;
            }
        }

        // Now check on the other side of the parent
        if( (queryPoint(parent->kdSplit) < parent->position(parent->kdSplit)) && parent->kdChildRExist ) {
            // queryPoint is on the left side of the parent, so we need to
            // look at the right side of it (if it exists)

            // Find right subtree distance
            KDTreeNode Rnode = KDTreeNode();
            double Rdist = 0;
            kdFindNearestInSubtree( Rnode, Rdist, distanceFunction, parent->kdChildR, queryPoint,currentClosestNode, currentClosestDist );

            if( Rdist < currentClosestDist ) {
                currentClosestNode = &Rnode;
                currentClosestDist = Rdist;
            }
        }
        else if( (parent->position(parent->kdSplit) <= queryPoint(parent->kdSplit)) && parent->kdChildLExist ) {
            // queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)

            // Find left subtree distance
            KDTreeNode Lnode = KDTreeNode();
            double Ldist = 0;
            kdFindNearestInSubtree( Lnode, Ldist, distanceFunction, parent->kdChildL, queryPoint, currentClosestNode, currentClosestDist );
            if( Ldist < currentClosestDist ) {
                currentClosestNode = &Lnode;
                currentClosestDist = Ldist;
            }
        }

        if( parent == root ) {
            // The parent is the root and we are done
            // Need to do one last check vs the root
            double thisDist = distFunc( distanceFunction, queryPoint, parent->position );
            if( thisDist < currentClosestDist ) {
                nearestNode = parent;
                nearestNodeDist = thisDist;
                return true;
            }

            nearestNode = currentClosestNode;
            nearestNodeDist = currentClosestDist;
            return true;
        }

        parent = parent->kdParent;
    }
}

bool kdFindNearestWithGuess( KDTreeNode* nearestNode, double &nearestNodeDist,
                             KDTree* tree, Eigen::VectorXd queryPoint, KDTreeNode* guess )
{
    double distToGuess = distFunc( tree->distanceFunction, queryPoint, guess->position );
    if( guess == tree->root ) {
        kdFindNearestInSubtree( *nearestNode, nearestNodeDist, tree->distanceFunction, tree->root, queryPoint, tree->root, distToGuess );
        return true;
    }

    KDTreeNode* Lnode = new KDTreeNode();
    double Ldist = 0;
    kdFindNearestinSubtreeWithGuess( Lnode, Ldist, tree->distanceFunction, tree->root, queryPoint, guess, distToGuess );

    if( tree->numWraps > 0 ) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        ghostPointIterator* pointIterator =  new ghostPointIterator( tree, queryPoint );
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = getNextGhostPoint( pointIterator, Ldist );
            Eigen::VectorXd zeros( pointIterator->kdTree->numWraps, 0 );
            if( thisGhostPoint == zeros) break;

            // Now see if any points in the space are closer to this ghost
            double distGhostToGuess = distFunc( tree->distanceFunction, thisGhostPoint, guess->position );
            KDTreeNode* thisLnode = new KDTreeNode();
            double thisLdist = 0;
            kdFindNearestinSubtreeWithGuess( thisLnode, thisLdist, tree->distanceFunction, tree->root, thisGhostPoint, guess, distGhostToGuess );

            if( thisLdist < Ldist ) {
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
KDTreeNode* addToKNNHeap( BinaryHeap* H, KDTreeNode *node, double key, int k )
{
    std::vector<KDTreeNode> heap = *H->getHeap();
    if( node->inHeap ) {
        if( H->topHeapB()->dist == -1 ) {
            std::cout << "Heap node has no reference tree node!" << std::endl;
        }
        return H->topHeapB();
    } else if( *H->getIndexOfLast() < k ) {
        // Just insert
        node->dist = key;
        H->addToHeapB( node );
    } else if( heap[1].dist > key ) {
        H->popHeapB();
        node->dist = key;
        H->addToHeapB( node );
    }
    if( H->topHeapB()->dist == -1 ) {
        std::cout << "Heap node has no reference tree node!" << std::endl;
    }
    return H->topHeapB();
}

bool kdFindKNearestInSubtree( KDTreeNode* farthestNode, double* farthestNodeDist,
                              std::string distanceFunction, KDTreeNode* root, int k,
                              Eigen::VectorXd queryPoint, BinaryHeap* nearestHeap )
{
    // Walk down the tree as if the node would be inserted
    KDTreeNode* parent = root;
    KDTreeNode* currentWorstClosestNode = nearestHeap->topHeapB();
    double currentWorstClosestDist = currentWorstClosestNode->dist;
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

    double newDist = distFunc( distanceFunction, queryPoint, parent->position );
    if( newDist < currentWorstClosestDist ) {
        currentWorstClosestNode = addToKNNHeap( nearestHeap, parent, newDist, k );
        currentWorstClosestDist = currentWorstClosestNode->dist;
    }

    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could possibly be any closer nodes on the other
        // side of the parent (and the parent itself is also too far away)

        double parentHyperPlaneDist = queryPoint(parent->kdSplit) - parent->position(parent->kdSplit);

        if( parentHyperPlaneDist > currentWorstClosestDist ) {
            // Then there could not be any closer nodes on the other side of the parent
            // (and the parent itself is also too far away)
            if( parent == root ) {
                // The parent is te root and we are done
                farthestNode = currentWorstClosestNode;
                *farthestNodeDist = currentWorstClosestDist;
                return true;
            }
            parent = parent->kdParent;
            continue;
        }

        // If we are here, then there could be a closer node on the other side
        // of the parent (including the parent itself)

        // First check the parent itself (if it is not already one of
        // the closest nodes)
        /* ***** Point of possible bug where parent is KDTreeNode* but has inHeap been set? ***** */
        if( !parent->inHeap ) {
            newDist = distFunc( distanceFunction, queryPoint, parent->position );
            if( newDist < currentWorstClosestDist ) {
                currentWorstClosestNode = addToKNNHeap( nearestHeap, parent, newDist, k );
                currentWorstClosestDist = currentWorstClosestNode->dist;
            }
        }

        // Now check the other side of the parent
        if( queryPoint(parent->kdSplit) < parent->position(parent->kdSplit) && parent->kdChildRExist ) {
            // queryPoint is on the left side of the parent, so we need to look
            // at the right side of it (if it exists)
            kdFindKNearestInSubtree( currentWorstClosestNode, &currentWorstClosestDist, distanceFunction, parent->kdChildR, k, queryPoint, nearestHeap );
            currentWorstClosestDist = currentWorstClosestNode->dist;
        } else if( parent->position(parent->kdSplit) <= queryPoint(parent->kdSplit) && parent->kdChildLExist ) {
            // queryPoint is on the right side of the parent, so we need to look
            // at the left side of it (if it exists)
            kdFindKNearestInSubtree( currentWorstClosestNode, &currentWorstClosestDist, distanceFunction, parent->kdChildL, k, queryPoint, nearestHeap );
            currentWorstClosestDist = currentWorstClosestNode->dist;
        }

        if( parent == root ) {
            // The parent is the root and we are done
            farthestNode = currentWorstClosestNode;
            *farthestNodeDist = currentWorstClosestDist;
        }

        parent = parent->kdParent;
    }
}

std::vector<KDTreeNode> kdFindKNearest( KDTree *tree, int k, Eigen::VectorXd queryPoint )
{
    BinaryHeap H = BinaryHeap(true); // true >> use heap functions (key not keyQ)

    // Insert root node in heap
    tree->root->dist = distFunc( tree->distanceFunction, queryPoint, tree->root->position );
    H.addToHeapB( tree->root );

    // Insert a dummy node in the heap with INF key
    KDTreeNode* dummyNode = new KDTreeNode((double)INF);
    H.addToHeapB( dummyNode );

    // Find k nearest neighbors
    KDTreeNode* farthestNearestNode = new KDTreeNode();
    double* farthestNearestNodeDist = 0;
    kdFindKNearestInSubtree( farthestNearestNode, farthestNearestNodeDist, tree->distanceFunction, tree->root, k, queryPoint, &H );

    if( tree->numWraps > 0 ) {
        std::cout << "ERROR: knn search not implemented for wrapped space" << std::endl;
    }

    // Remove the dummy node if still there (guarenteed to be on top, due to INF key)
    KDTreeNode* topNode = H.topHeapB();
    if( topNode == dummyNode ) H.popHeapB();

    return H.cleanHeapB();
}

/////////////////////// Within Range ///////////////////////

bool addToRangeList( JList &S, KDTreeNode* node, double key )
{
    // "inHeap" is a misnomer because this is a list
    if( node->inHeap ) {
        // Node already in list
        return false;
    }
    node->inHeap = true;
    S.JlistPush( node, key );
    return true;
}

void popFromRangeList( JList &S, KDTreeNode* t, double &k )
{
    S.JlistPopKey(t,k);
    t->inHeap = false;
}

void emptyRangeList( JList &S )
{
    KDTreeNode* n = new KDTreeNode();
    double k = 0;
    while( S.length > 0 ) {
        S.JlistPopKey( n, k );
        n->inHeap = false;
    }

}

bool kdFindWithinRangeInSubtree( std::string distanceFunction, KDTreeNode* root,
                                 double range, Eigen::VectorXd queryPoint,
                                 JList &nodeList )
{
    // Walk down the tree as if the node would be inserted
    KDTreeNode* parent = root;
    while( true ) {
        //std::cout << "parent:\n" << parent->position << std::endl;
        if( queryPoint(parent->kdSplit) < parent->position(parent->kdSplit) ) {
            // Traverse tree to the left
            if( !parent->kdChildLExist ) {
                // The queryPoint would be inserted as the left child of the parent
                break;
            }
            parent = parent->kdChildL;
            continue;
        } else {
            // Traverse tree to the right
            if( !parent->kdChildRExist ) {
                // The queryPoint would be inserted as the right child of the parent
                break;
            }
            parent = parent->kdChildR;
            continue;
        }
    }

    double newDist = distFunc( distanceFunction, queryPoint, parent->position );
    if( newDist < range ) {
        addToRangeList( nodeList, parent, newDist );
    }

    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could possibly be any nodes on the other
        // side of the parent within range, if not then check grandparent etc.

        double parentHyperPlaneDist = queryPoint(parent->kdSplit) - parent->position(parent->kdSplit);

        if( parentHyperPlaneDist > range ) {
            // Then there could not be any closer nodes within range on the other
            // side of the parent (and the parent itself is also too far away
            if( parent == root ) {
                // The parent is the root and we are done
                return true;
            }

            parent = parent->kdParent;
            continue;
        }

        // If we are here, then there could be a closer node on the other side
        // of the parent (including the parent itself) that is within range

        // First check the parent itself (if it is not already one of the closest
        // nodes)
        if( !parent->inHeap ) { // inHeap is a misnomer since this is a list
            newDist = distFunc( distanceFunction, queryPoint, parent->position );
            if( newDist < range ) {
                addToRangeList( nodeList, parent, newDist );
            }
        }

        // Now check on the other side of the parent
        if( queryPoint(parent->kdSplit) < parent->position(parent->kdSplit) && parent->kdChildRExist ) {
            // The queryPoint is on the left side of the porent, so we need to
            // look at the right side of it (if it exists)
            kdFindWithinRangeInSubtree(distanceFunction, parent->kdChildR, range, queryPoint, nodeList );
        } else if( parent->position(parent->kdSplit) <= queryPoint(parent->kdSplit) && parent->kdChildLExist ) {
            // The queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)
            kdFindWithinRangeInSubtree( distanceFunction, parent->kdChildL, range, queryPoint, nodeList );
        }

        if( parent == root ) {
            // The parent is the root and we are done
            return true;
        }

        parent = parent->kdParent;
    }
}

void kdFindWithinRange( JList &S, KDTree* tree, double range, Eigen::VectorXd queryPoint )
{
    // Insert root node in list if it is within range
    double distToRoot = distFunc( tree->distanceFunction, queryPoint, tree->root->position );
    if( distToRoot <= range ) {
        addToRangeList( S, tree->root, distToRoot );
    }

    // Find nodes within range
    kdFindWithinRangeInSubtree( tree->distanceFunction, tree->root, range, queryPoint, S );

    if( tree->numWraps > 0 ) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)

        ghostPointIterator* pointIterator = new ghostPointIterator( tree, queryPoint );
        while( true ) {
            Eigen::VectorXd thisGhostPoint = getNextGhostPoint( pointIterator, range );
            if( thisGhostPoint.isZero(0) ) break;

            // Now see if any points in the space are closer to this ghost
            kdFindWithinRangeInSubtree( tree->distanceFunction, tree->root, range, thisGhostPoint, S );
        }
    }
}

void kdFindMoreWithinRange( JList &L, KDTree* tree, double range, Eigen::VectorXd queryPoint )
{
    // Insert root node in list if it is within range
    double distToRoot = distFunc( tree->distanceFunction, queryPoint, tree->root->position );
    if( distToRoot <= range ) {
        addToRangeList( L, tree->root, distToRoot );
    }

    // Find nodes within range
    kdFindWithinRangeInSubtree( tree->distanceFunction, tree->root, range, queryPoint, L );

    if( tree->numWraps > 0 ) {
        // If dimensions wrap around, need to search vs. identities (ghosts)
        ghostPointIterator* pointIterator = new ghostPointIterator( tree, queryPoint );
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = getNextGhostPoint( pointIterator, range );
            if( thisGhostPoint.isZero(0) ) {
                break;
            }

            // Now see if any points in the space are closer to this ghost
            kdFindWithinRangeInSubtree( tree->distanceFunction, tree->root, range, thisGhostPoint, L );
        }
    }

}

void kdInsert( KDTree* tree, Eigen::VectorXd a )
{
    KDTreeNode* N = new KDTreeNode();
    N->position = a;
    kdInsert(tree, N);
}

#endif // KDTREE_CPP
