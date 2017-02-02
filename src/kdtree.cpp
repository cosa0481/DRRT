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
        return R3Dist(pointA,pointB);
    } else if( distanceFunction == "R3SDist" ) {
        return R3SDist(pointA,pointB);
    } else if( distanceFunction == "EuclideanDist" ) {
        return EuclideanDist(pointA,pointB);
    } else if( distanceFunction == "threeTwoDRobotDist" ) {
        // drrt.h >> distance(), Wdist()
        return EuclideanDist(pointA,pointB);
    } else if( distanceFunction == "KDdist" ) {
        return R3SDist(pointA,pointB);
    } else if( distanceFunction == "S3Dist" ) {
        return S3KDSearchDist(pointA,pointB);
    }

    return 0.0;
}

void KDTreeInit( KDTree* K, int d, std::string distanceFunction )
{
    K->d = d;
    K->distanceFunction = distanceFunction;
    K->treeSize = 0;
}

bool kdInsert( KDTree* tree, std::shared_ptr<KDTreeNode> node )
{
    if( node.get()->kdInTree ) return false;
    node.get()->kdInTree = true;

    if( tree->treeSize == 0 ) {
        tree->root = node;
        tree->root->kdSplit = 1;
        tree->treeSize = 1;
        return true;
    }

    // Figure out where to put this node
    std::shared_ptr<KDTreeNode> parent = tree->root;
    while( true ) {
        if( node.get()->position(parent->kdSplit)
                < parent->position(parent->kdSplit) ) {
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

    //std::cout << "kdInsert : Just added\n" << node->position << "\n" << Edist(node->position,parent->position) << " units from\n" << parent->position << std::endl;
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

bool kdFindNearestInSubtree(std::shared_ptr<KDTreeNode> nearestNode,
                            std::shared_ptr<double> nearestNodeDist,
                            std::string distanceFunction,
                            std::shared_ptr<KDTreeNode> root,
                            Eigen::VectorXd queryPoint,
                            std::shared_ptr<KDTreeNode> suggestedClosestNode,
                            double suggestedClosestDist)
{
    std::shared_ptr<KDTreeNode> parent = root;
    std::shared_ptr<KDTreeNode> currentClosestNode = suggestedClosestNode;
    std::shared_ptr<double> currentClosestDist
            = std::make_shared<double>(suggestedClosestDist);

//    std::cout << "starting at\n" << parent->position << std::endl;
//    std::cout << "suggested\n" << suggestedClosestNode->position << std::endl;
//    std::cout << "dist " << suggestedClosestDist << std::endl;
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
    if( newDist < *currentClosestDist ) {
//        std::cout << "nearest0 (no return)\n" << parent.get()->position << std::endl;
        currentClosestNode = parent;
        currentClosestDist = std::make_shared<double>(newDist);
    }


    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could plossibly be any closer nodes on the other
        // side of the parent. If not then check grandparent etc.
        double parentHyperPlaneDist = queryPoint(parent->kdSplit) - parent->position(parent->kdSplit);

        if( parentHyperPlaneDist > *currentClosestDist ) {
            // Then there could not be any closer nodes on the other side of
            // the parent and the parent itself is also too far away

            if( parent == root ) {
                // The parent is the root and we are done
                *nearestNode = *currentClosestNode;
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
            newDist = distFunc(distanceFunction, queryPoint, parent->position);
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
            kdFindNearestInSubtree( Rnode, Rdist, distanceFunction,
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
            kdFindNearestInSubtree( Lnode, Ldist, distanceFunction,
                                    parent->kdChildL, queryPoint,
                                    currentClosestNode, *currentClosestDist );
            if( *Ldist < *currentClosestDist ) {
                currentClosestNode = Lnode;
                currentClosestDist = Ldist;
            }
        }


        if( parent == root ) {
            // The parent is the root and we are done
            *nearestNode = *currentClosestNode;
            *nearestNodeDist = *currentClosestDist;
            return true;
        }
        parent = parent->kdParent;
    }
}

bool kdFindNearest(std::shared_ptr<KDTreeNode> nearestNode,
                   std::shared_ptr<double> nearestNodeDist,
                   KDTree* tree,
                   Eigen::VectorXd queryPoint)
{
    // Initial search (only search if the space does not wrap around)
    double distToRoot = distFunc(tree->distanceFunction, queryPoint,
                                 tree->root->position);
    std::shared_ptr<KDTreeNode> Lnode
            = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> Ldist = std::make_shared<double>(0);
    kdFindNearestInSubtree( Lnode, Ldist, tree->distanceFunction, tree->root,
                            queryPoint, tree->root, distToRoot );

//    std::cout << "Lnode\n" << Lnode->position << std::endl;

    if( tree->numWraps > 0 ) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        std::shared_ptr<ghostPointIterator> pointIterator
                = std::make_shared<ghostPointIterator>
                (ghostPointIterator( tree, queryPoint ));
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = getNextGhostPoint( pointIterator, *Ldist );
            if( thisGhostPoint.isZero(0) ) break;

            // Now see if any points in the space are closer to this ghost
            double distGhostToRoot = distFunc( tree->distanceFunction,
                                               thisGhostPoint,
                                               tree->root->position );
            std::shared_ptr<KDTreeNode> thisLnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> thisLdist = std::make_shared<double>(0);
            kdFindNearestInSubtree( thisLnode, thisLdist,
                                    tree->distanceFunction, tree->root,
                                    thisGhostPoint, tree->root,
                                    distGhostToRoot );

            if( *thisLdist < *Ldist ) {
                // Found a closer point
                Lnode = thisLnode;
                Ldist = thisLdist;
            }
        }
    }
    *nearestNode = *Lnode;
    *nearestNodeDist = *Ldist;
    return true;
}

bool kdFindNearestinSubtreeWithGuess( std::shared_ptr<KDTreeNode> nearestNode,
                                      std::shared_ptr<double> nearestNodeDist,
                                      std::string distanceFunction,
                                      std::shared_ptr<KDTreeNode> root,
                                      Eigen::VectorXd queryPoint,
                           std::shared_ptr<KDTreeNode> suggestedClosestNode,
                                      double suggestedClosestDist )
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

    double newDist = distFunc( distanceFunction, queryPoint, parent->position );
    if( newDist < *currentClosestDist ) {
        currentClosestNode = parent;
        currentClosestDist = std::make_shared<double>(newDist);
    }

    // Now walk back up the tree (will break out when done)
    while( true ) {
        // Now check if there could plossibly be any closer nodes on the other
        // side of the parent. If not then check grandparent etc.

        double parentHyperPlaneDist = queryPoint(parent->kdSplit) - parent->position(parent->kdSplit);

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
            newDist = distFunc( distanceFunction, queryPoint, parent->position );
            if( newDist < *currentClosestDist ) {
                currentClosestNode = parent;
                currentClosestDist = std::make_shared<double>(newDist);
            }
        }

        // Now check on the other side of the parent
        if( (queryPoint(parent->kdSplit) < parent->position(parent->kdSplit)) && parent->kdChildRExist ) {
            // queryPoint is on the left side of the parent, so we need to
            // look at the right side of it (if it exists)

            // Find right subtree distance
            std::shared_ptr<KDTreeNode> Rnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Rdist = 0;
            kdFindNearestInSubtree( Rnode, Rdist, distanceFunction,
                                    parent->kdChildR, queryPoint,
                                    currentClosestNode, *currentClosestDist );

            if( *Rdist < *currentClosestDist ) {
                currentClosestNode = Rnode;
                currentClosestDist = Rdist;
            }
        }
        else if( (parent->position(parent->kdSplit) <= queryPoint(parent->kdSplit)) && parent->kdChildLExist ) {
            // queryPoint is on the right side of the parent, so we need to
            // look at the left side of it (if it exists)

            // Find left subtree distance
            std::shared_ptr<KDTreeNode> Lnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> Ldist = 0;
            kdFindNearestInSubtree( Lnode, Ldist, distanceFunction, parent->kdChildL, queryPoint, currentClosestNode, *currentClosestDist );
            if( *Ldist < *currentClosestDist ) {
                currentClosestNode = Lnode;
                currentClosestDist = Ldist;
            }
        }

        if( parent == root ) {
            // The parent is the root and we are done
            // Need to do one last check vs the root
            double thisDist = distFunc( distanceFunction, queryPoint, parent->position );
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

bool kdFindNearestWithGuess( std::shared_ptr<KDTreeNode> nearestNode, std::shared_ptr<double> nearestNodeDist,
                             KDTree* tree, Eigen::VectorXd queryPoint, std::shared_ptr<KDTreeNode> guess )
{
    double distToGuess = distFunc( tree->distanceFunction, queryPoint, guess->position );
    if( guess == tree->root ) {
        kdFindNearestInSubtree( nearestNode, nearestNodeDist, tree->distanceFunction, tree->root, queryPoint, tree->root, distToGuess );
        return true;
    }

    std::shared_ptr<KDTreeNode> Lnode = std::make_shared<KDTreeNode>();
    std::shared_ptr<double> Ldist = 0;
    kdFindNearestinSubtreeWithGuess( Lnode, Ldist, tree->distanceFunction,
                                     tree->root, queryPoint, guess,
                                     distToGuess );

    if( tree->numWraps > 0 ) {
        // If dimensions wrap around, we need to search vs. identities (ghosts)
        std::shared_ptr<ghostPointIterator> pointIterator
                = std::make_shared<ghostPointIterator>
                (ghostPointIterator( tree, queryPoint ));
        Eigen::VectorXd thisGhostPoint;
        while( true ) {
            thisGhostPoint = getNextGhostPoint( pointIterator, *Ldist );
            Eigen::VectorXd zeros( pointIterator->kdTree->numWraps, 0 );
            if( thisGhostPoint == zeros) break;

            // Now see if any points in the space are closer to this ghost
            double distGhostToGuess = distFunc(tree->distanceFunction,
                                               thisGhostPoint, guess->position);
            std::shared_ptr<KDTreeNode> thisLnode
                    = std::make_shared<KDTreeNode>();
            std::shared_ptr<double> thisLdist = 0;
            kdFindNearestinSubtreeWithGuess( thisLnode, thisLdist,
                                             tree->distanceFunction,
                                             tree->root, thisGhostPoint, guess,
                                             distGhostToGuess );

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
std::shared_ptr<KDTreeNode> addToKNNHeap(BinaryHeap* H,
                                         std::shared_ptr<KDTreeNode> node,
                                         double key, int k)
{
    std::vector<std::shared_ptr<KDTreeNode>> heap = H->getHeap();
    if( node.get()->inHeap ) {
        if( H->topHeapB()->dist == -1 ) {
            std::cout << "Heap node has no reference tree node!" << std::endl;
        }
        return H->topHeapB();
    } else if( *H->getIndexOfLast() < k ) {
        // Just insert
        node.get()->dist = key;
        H->addToHeapB( node );
    } else if( heap[1].get()->dist > key ) {
        H->popHeapB();
        node.get()->dist = key;
        H->addToHeapB( node );
    }
    if( H->topHeapB()->dist == -1 ) {
        std::cout << "Heap node has no reference tree node!" << std::endl;
    }
    return H->topHeapB();
}

bool kdFindKNearestInSubtree( std::shared_ptr<KDTreeNode> farthestNode, double* farthestNodeDist,
                              std::string distanceFunction, std::shared_ptr<KDTreeNode> root, int k,
                              Eigen::VectorXd queryPoint, BinaryHeap* nearestHeap )
{
    // Walk down the tree as if the node would be inserted
    std::shared_ptr<KDTreeNode> parent = root;
    std::shared_ptr<KDTreeNode> currentWorstClosestNode = nearestHeap->topHeapB();
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
        /* ***** Point of possible bug where parent is std::shared_ptr<KDTreeNode> but has inHeap been set? ***** */
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

std::vector<std::shared_ptr<KDTreeNode>> kdFindKNearest( KDTree *tree, int k, Eigen::VectorXd queryPoint )
{
    BinaryHeap H = BinaryHeap(true); // true >> use heap functions (key not keyQ)

    // Insert root node in heap
    tree->root->dist = distFunc( tree->distanceFunction, queryPoint, tree->root->position );
    H.addToHeapB( tree->root );

    // Insert a dummy node in the heap with INF key
    std::shared_ptr<KDTreeNode> dummyNode = std::make_shared<KDTreeNode>((double)INF);
    H.addToHeapB( dummyNode );

    // Find k nearest neighbors
    std::shared_ptr<KDTreeNode> farthestNearestNode = std::make_shared<KDTreeNode>();
    double* farthestNearestNodeDist = 0;
    kdFindKNearestInSubtree( farthestNearestNode, farthestNearestNodeDist, tree->distanceFunction, tree->root, k, queryPoint, &H );

    if( tree->numWraps > 0 ) {
        std::cout << "ERROR: knn search not implemented for wrapped space" << std::endl;
    }

    // Remove the dummy node if still there (guarenteed to be on top, due to INF key)
    std::shared_ptr<KDTreeNode> topNode = H.topHeapB();
    if( topNode == dummyNode ) H.popHeapB();

    return H.cleanHeapB();
}

/////////////////////// Within Range ///////////////////////

bool addToRangeList( JList &S, std::shared_ptr<KDTreeNode> node, double key )
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

void popFromRangeList( JList &S, std::shared_ptr<KDTreeNode> t, double &k )
{
    S.JlistPopKey(t,k);
    t->inHeap = false;
}

void emptyRangeList( JList &S )
{
    std::shared_ptr<KDTreeNode> n = std::make_shared<KDTreeNode>();
    double k = 0;
    while( S.length > 0 ) {
        S.JlistPopKey( n, k );
        n->inHeap = false;
    }

}

bool kdFindWithinRangeInSubtree( std::string distanceFunction, std::shared_ptr<KDTreeNode> root,
                                 double range, Eigen::VectorXd queryPoint,
                                 JList &nodeList )
{
    // Walk down the tree as if the node would be inserted
    std::shared_ptr<KDTreeNode> parent = root;
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

        std::shared_ptr<ghostPointIterator> pointIterator
                = std::make_shared<ghostPointIterator>
                (tree, queryPoint);
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
        std::shared_ptr<ghostPointIterator> pointIterator
                = std::make_shared<ghostPointIterator>
                (tree, queryPoint);
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
    std::shared_ptr<KDTreeNode> N = std::make_shared<KDTreeNode>();
    N->position = a;
    kdInsert(tree, N);
}

#endif // KDTREE_CPP
