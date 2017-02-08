/* drrt.cpp
 * Corin Sandford
 * Fall 2016
 * Contains RRTX "main" function at bottom.
 */

#include <DRRT/drrt.h>

///////////////////// Print Helpers ///////////////////////
void error(std::string s) { std::cout << s << std::endl; }
void error(int i) { std::cout << i << std::endl; }

double getTimeNs( std::chrono::time_point
                  <std::chrono::high_resolution_clock> start )
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now()-start).count();
}

double randDouble( double min, double max )
{    
    std::uniform_real_distribution<double> unid(min,max);
    std::mt19937 rng; // Mersenn-Twister random-number engine
    rng.seed(std::random_device{}());
    double random_double = unid(rng);
    return random_double;
}


/////////////////////// Node Functions ///////////////////////

double extractPathLength( std::shared_ptr<KDTreeNode> node, std::shared_ptr<KDTreeNode> root )
{
    double pathLength = 0.0;
    std::shared_ptr<KDTreeNode> thisNode = node;
    while( node != root ) {
        if( !node->rrtParentUsed ) {
            pathLength = INF;
            break;
        }
        pathLength += node->rrtParentEdge->dist;
        thisNode = thisNode->rrtParentEdge->endNode;
    }

    return pathLength;
}


/////////////////////// C-Space Functions ///////////////////////

Eigen::VectorXd randPointDefault( std::shared_ptr<CSpace> S )
{
    double rand;
    double first;
    Eigen::VectorXd second(S->width.size());

    for( int i = 0; i < S->width.size(); i++ ) {
        rand = randDouble(0,S->d);
        first = rand * S->width(i);
        second(i) = S->lowerBounds(i) + first;
    }
    return second;
}

std::shared_ptr<KDTreeNode> randNodeDefault( std::shared_ptr<CSpace> S )
{
    Eigen::VectorXd point = randPointDefault( S );
    return std::make_shared<KDTreeNode>(point);
}

std::shared_ptr<KDTreeNode> randNodeOrGoal( std::shared_ptr<CSpace> S )
{
    double r = (double)rand()/(RAND_MAX);
    if( r > S->pGoal ) {
        return randNodeDefault( S );
    } else {
        return S->goalNode;
    }
}

std::shared_ptr<KDTreeNode> randNodeIts( std::shared_ptr<CSpace> S )
{
    if( S->itsUntilSample == 0 ) {
        S->itsUntilSample -= 1;
        return std::make_shared<KDTreeNode>(S->itsSamplePoint);
    }
    S->itsUntilSample -= 1;
    return randNodeOrGoal( S );
}

std::shared_ptr<KDTreeNode> randNodeTime( std::shared_ptr<CSpace> S )
{
    if( S->waitTime != INF && S->timeElapsed >= S->waitTime ) {
        S->waitTime = INF;
        return std::make_shared<KDTreeNode>(S->timeSamplePoint);
    }
    return randNodeOrGoal( S );
}

//std::shared_ptr<KDTreeNode> randNodeTimeWithObstacleRemove( std::shared_ptr<CSpace> S ){}
//std::shared_ptr<KDTreeNode> randNodeItsWithObstacleRemove( std::shared_ptr<CSpace> S ){}

std::shared_ptr<KDTreeNode> randNodeOrFromStack( std::shared_ptr<CSpace> S )
{
    if( S->sampleStack->length > 0 ) {
        // Using the sampleStack so KDTreeNode->position is popped
        std::shared_ptr<KDTreeNode> temp = std::make_shared<KDTreeNode>();
        S->sampleStack->JlistPop(temp);
        return std::make_shared<KDTreeNode>(temp->position);
    } else {
        return randNodeOrGoal( S );
    }
}

std::shared_ptr<KDTreeNode> randNodeInTimeOrFromStack( std::shared_ptr<CSpace> S )
{
    if( S->sampleStack->length > 0 ) {
        // Using the sampleStack so KDTreeNode->position is popped
        std::shared_ptr<KDTreeNode> temp = std::make_shared<KDTreeNode>();
        S->sampleStack->JlistPop(temp);
        return std::make_shared<KDTreeNode>(temp->position);
    } else {
        std::shared_ptr<KDTreeNode> newNode = randNodeOrGoal( S );
        if( newNode == S->goalNode ) {
            return newNode;
        }

        double minTimeToReachNode = S->start(2) + sqrt( (newNode->position(0) - S->root->position(0))*(newNode->position(0) - S->root->position(0)) + (newNode->position(1) - S->root->position(1))*(newNode->position(1) - S->root->position(1)) ) / S->robotVelocity;

        // If point is too soon vs robot's available speed
        // or if it is in the "past" and the robot is moving
        if( newNode->position(2) < minTimeToReachNode ||
                (newNode->position(2) > S->moveGoal->position(2) &&
                 S->moveGoal != S->goalNode) ) {
            // Resample time in ok range
            double r = (double) rand() / (RAND_MAX);
            newNode->position(2) = minTimeToReachNode + r * (S->moveGoal->position(2) - minTimeToReachNode);
        }
        return newNode;
    }
}

bool checkNeighborsForEdgeProblems( std::shared_ptr<CSpace> S, std::shared_ptr<KDTreeNode> thisNode )
{
    if( thisNode->rrtParentUsed ) {
//        if( explicitEdgeCheck( S, thisNode, thisNode->rrtParentEdge->endNode ) ) {
//            return true;
//        }
    }

    std::shared_ptr<JListNode> listItem = thisNode->rrtNeighborsOut->front;
    std::shared_ptr<KDTreeNode> neighborNode;
    while( listItem != listItem->child ) {
        neighborNode = listItem->node;

//        if( neighborNode->rrtParentUsed && explicitEdgeCheck( S, neighborNode, neighborNode->rrtParentEdge->endNode ) ) {
//            return true;
//        }

        listItem = listItem->child; // iterate
    }
    return false;
}

/////////////////////// Geometric Functions ///////////////////////

double distanceSqrdPointToSegment( Eigen::VectorXd point,
                                   Eigen::VectorXd startPoint,
                                   Eigen::VectorXd endPoint )
{
    double vx = point(0) - startPoint(0);
    double vy = point(1) - startPoint(1);
    double ux = endPoint(0) - startPoint(0);
    double uy = endPoint(1) - startPoint(1);
    double determinate = vx*ux + vy*uy;

    if( determinate <= 0 ) {
        return vx*vx + vy*vy;
    } else {
        double len = ux*ux + uy*uy;
        if( determinate >= len ) {
            return (endPoint(0)-point(0))*(endPoint(0)-point(0)) +
                    (endPoint(1)-point(1))*(endPoint(1)-point(1));
        } else {
            return (ux*vy - uy*vx)*(ux*vy - uy*vx) / len;
        }
    }
}

double segmentDistSqrd( Eigen::VectorXd PA, Eigen::VectorXd PB,
                        Eigen::VectorXd QA, Eigen::VectorXd QB )
{
    // Check if the points are definately not in collision by seeing
    // if both points of Q are on the same side of line containing P and vice versa

    bool possibleIntersect = true;
    double m, diffA, diffB;

    // First check if P is close to vertical
    if( std::abs(PB(0) - PA(0)) < 0.000001 ) {
        // P is close to vertical
        if( (QA(0) >= PA(0) && QB(0) >= PA(0))
                || (QA(0) <= PA(0) && QB(0) <= PA(0)) ) {
            // Q is on one side of P
            possibleIntersect = false;
        }
    } else {
        // P is not close to vertical
        m = (PB(1) - PA(1)) / (PB(0) - PA(0));

        // Equation for points on P: y = m(x - PA[1]) + PA[2]
        diffA = (m*(QA(0)-PA(0)) + PA(1)) - QA(1);
        diffB = (m*(QB(0)-PA(0)) + PA(1)) - QB(1);
        if( (diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0) ) {
            // Q is either fully above or below the line containing P
            possibleIntersect = false;
        }
    }

    if( possibleIntersect ) {
        // first check if Q is close to vertical
        if( std::abs(QB(0) - QA(0)) < 0.000001 ) {
            if( (PA(0) >= QA(0) && PB(0) >= QA(0)) || (PA(0) <= QA(0) && PB(0) <= QA(0)) ) {
                // P is on one side of Q
                possibleIntersect = false;
            }
        } else {
            // Q is not close to vertical
            m = (QB(1) - QA(1)) / (QB(0) - QA(0));

            // Equation for points on Q: y = m(x-QA[1]) + QA[2]
            diffA = (m*(PA(0)-QA(0)) + QA(1)) - PA(1);
            diffB = (m*(PB(0)-QA(0)) + QA(1)) - PB(1);
            if( (diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0) ) {
                // P is either fully above or below the line containing Q
                possibleIntersect = false;
            }
        }
    }

    if( possibleIntersect ) {
        // Then there is an intersection for sure
        return 0.0;
    }

    // When the lines do not intersect in 2D, the min distance must
    // be between one segment's end point and the other segment
    // (assuming lines are not parallel)
    Eigen::VectorXd distances;
    distances(0) = distanceSqrdPointToSegment(PA,QA,QB);
    distances(1) = distanceSqrdPointToSegment(PB,QA,QB);
    distances(2) = distanceSqrdPointToSegment(QA,PA,PB);
    distances(3) = distanceSqrdPointToSegment(QB,PA,PB);

    return distances.minCoeff();  //*std::min_element( distances.begin(), distances.end() );
}

int findIndexBeforeTime( Eigen::MatrixXd path, double timeToFind )
{
    if( path.rows() < 1) {
        return -1;
    }

    int i = -1;
    while( i+1 < path.rows() && path(i+1,2) < timeToFind ) {
        i += 1;
    }
    return i;
}


/////////////////////// Collision Checking Functions ///////////////////////

bool explicitEdgeCheck( std::shared_ptr<CSpace> S, std::shared_ptr<Edge> edge )
{
    // If ignoring obstacles
    if( S->inWarmupTime ) return false;

//    std::shared_ptr<JListNode> obstacleListNode = S->obstacles.front;
//    for( int i = 0; i < obstacles.length; i++ ) {
//        if( explicitEdgeCheck( S, edge, obstacleListNode->node) ) { // obstacleListNode.data ( also maybe should be explicitNodeCheck? )
//            return true;
//        }
//        obstacleListNode = obstacleListNode->child; // iterate
//    }
    return false;
}

/////////////////////// RRT Functions ///////////////////////

bool extend(std::shared_ptr<CSpace> S,
            std::shared_ptr<KDTree> Tree,
            std::shared_ptr<Queue> Q,
            std::shared_ptr<KDTreeNode> newNode,
            std::shared_ptr<KDTreeNode> closestNode,
            double delta, double hyperBallRad,
            std::shared_ptr<KDTreeNode> moveGoal )
{
    if( Q->type == "RRT" ) {
        // First calculate the shortest trajectory (and its distance) that
        // gets from newNode to closestNode while obeying the constraints
        // of the state space and the dynamics of the robot
        std::shared_ptr<Edge> thisEdge
                = Edge::newEdge( newNode, closestNode );
        thisEdge->calculateTrajectory( S, Tree );

        // Figure out if we can link to the nearest node
        if ( !thisEdge->validMove( S )
             || explicitEdgeCheck( S, thisEdge) ) {
            // We cannot link to nearest neighbor
            return false;
        }

        // Otherwise we can link to the nearest neighbor
        newNode->rrtParentEdge = thisEdge;
        newNode->rrtTreeCost = closestNode->rrtLMC + newNode->rrtParentEdge->dist;
        newNode->rrtLMC = newNode->rrtTreeCost; // only for compatability with visualization
        newNode->rrtParentUsed = true;

        // insert the new node into the KDTree
        return Tree->kdInsert( newNode );
    } else if( Q->type == "RRT*" ) {
        // Find all nodes within the (shrinking hyperball of (Edge::saturated) newNode
        std::shared_ptr<JList> nodeList = std::make_shared<JList>(true);
        Tree->kdFindWithinRange( nodeList, hyperBallRad, newNode->position );

        // Try to find and link to best parent
        findBestParent( S, Tree, newNode, nodeList, closestNode, true);
        if( !newNode->rrtParentUsed ) {
            Tree->emptyRangeList( nodeList ); // clean up
            return false;
        }

        newNode->rrtTreeCost = newNode->rrtLMC;

        // Insert new node into the KDTre
        Tree->kdInsert( newNode );

        // If this is inserted in a an unhelpful part of the C-space then
        // don't waste time rewiring (assumes triange inequality, added
        // by MO, not technically part of RRT* but can only improve it)
        if( newNode->rrtLMC > moveGoal->rrtLMC ) {
            Tree->emptyRangeList( nodeList ); // clean up
            return false;
        }

        // Now rewire neighbors that should use newNode as their parent
        std::shared_ptr<JListNode> listItem = nodeList->front;
        std::shared_ptr<KDTreeNode> nearNode;
        std::shared_ptr<Edge> thisEdge;
        for( int i = 0; i < nodeList->length; i++ ) {
            nearNode = listItem->node;

            // Watch out for cycles
            if( newNode->rrtParentEdge->endNode == nearNode ) {
                listItem = listItem->child; // iterate through list
                continue;
            }

            // Calculate the shortest trajectory (and its distance) that
            // gets from nearNode to newNode while obeying the constraints
            // of the state space and the dynamics of the robot
            thisEdge = Edge::newEdge( nearNode, newNode );
            thisEdge->calculateTrajectory( S, Tree );

            // Rewire neighbors that would do betten to use this node
            // as their parent unless they are in collision or
            // impossible due to dynamics of robot/space
            if( nearNode->rrtLMC > newNode->rrtLMC + thisEdge->dist
                    && thisEdge->validMove( S )
                    && !explicitEdgeCheck( S, thisEdge ) ) {
                // Make this node the parent of the neighbor node
                nearNode->rrtParentEdge = thisEdge;
                nearNode->rrtParentUsed = true;

                // Recalculate tree cost of neighbor
                nearNode->rrtTreeCost = nearNode->rrtLMC + thisEdge->dist;
                nearNode->rrtLMC = newNode->rrtLMC + thisEdge->dist;
            }
            listItem = listItem->child; // iterate through list
        }
        Tree->emptyRangeList( nodeList ); // clean up
        return true;
    } else if( Q->type == "RRT#" ) {
        // Find all nodes within the (shrinking) hyperball of
        // (Edge::saturated) newNode
        std::shared_ptr<JList> nodeList = std::make_shared<JList>(true);
        Tree->kdFindWithinRange(nodeList, hyperBallRad, newNode->position );

        // Try to find and link to best parent, this also saves the
        // edges from newNode to the neighbors in the field "tempEdge"
        // of the neighbors. This saves time in the case that
        // trajectory calculation is complicated.
        findBestParent( S, Tree, newNode, nodeList, closestNode, true );

        // If no parent was found then ignore this node
        if( !newNode->rrtParentUsed ) {
            Tree->emptyRangeList( nodeList ); // clean up
            return false;
        }

        // Insert the new node into the KDTree
        Tree->kdInsert( newNode );

        // First pass, make edges between newNode and all of its valid
        // neighbors. Note that the edges have been stored in "tempEdge"
        // field of the neighbors
        std::shared_ptr<JListNode> listItem = nodeList->front;
        std::shared_ptr<KDTreeNode> nearNode;
        while( listItem != listItem->child ) {
            nearNode = listItem->node;

            if( nearNode->tempEdge->dist == INF ) { // obstacle, edge invalid
                listItem = listItem->child; // iterate through list
                continue;
            }

            // Make nearNode a neighbor (can be reached from) newNode
            makeNeighborOf( nearNode, newNode, nearNode->tempEdge );

            listItem = listItem->child; // iterate through list
        }

        // Second pass, make edges (if possible) between all valid nodes in
        // D-ball and newNode, also rewire neighbors that should use
        // newNode as their parent
        listItem = nodeList->front;
        std::shared_ptr<Edge> thisEdge;
        while( listItem != listItem->child ) {
            nearNode = listItem->node;

            // In the general case the trajectories along edges are not
            // simply the reverse of each other, therefore we need to
            // calculate and check the trajectory along the edge from
            // nearNode to newNode.
            thisEdge = Edge::newEdge( nearNode, newNode );
            thisEdge->calculateTrajectory( S, Tree );

            if( thisEdge->validMove( S )
                    && !explicitEdgeCheck( S, thisEdge ) ) {
                makeNeighborOf( newNode, nearNode, thisEdge );
            } else {
                // Edge cannot be created
                listItem = listItem->child; // iterate through list
                continue;
            }

            // Rewire neighbors that would do better to use this
            // node as their parent unless they are not in the
            // relevant portion of the space vs. moveGoal
            if( nearNode->rrtLMC > newNode->rrtLMC + thisEdge->dist &&
                    newNode->rrtParentEdge->endNode != nearNode &&
                    newNode->rrtLMC + thisEdge->dist < moveGoal->rrtLMC ) {
                // Make this node the parent of the neighbor node
                nearNode->rrtParentEdge = thisEdge;
                nearNode->rrtParentUsed = true;

                // Recalculate tree cost of neighbor
                nearNode->rrtLMC = newNode->rrtLMC + thisEdge->dist;

                // Insert the neighbor into priority queue if it is not consistant
                if( nearNode->rrtLMC != nearNode->rrtTreeCost && Q->Q->markedQ(nearNode) ) {
                    Q->Q->updateHeap( nearNode );
                } else if( nearNode->rrtTreeCost != nearNode->rrtLMC ) {
                    Q->Q->addToHeap( nearNode );
                } else if( newNode->rrtTreeCost == newNode->rrtLMC && Q->Q->markedQ(nearNode) ) {
                    Q->Q->updateHeap( nearNode );
                    Q->Q->removeFromHeap( nearNode );
                }
            }

            listItem = listItem->child; // iterate through list
        }
        Tree->emptyRangeList( nodeList ); // clean up

        // Insert the node into the priority queue
        Q->Q->addToHeap( newNode );
        return true;
    } else { // Q->type == "RRTx"
        // Find all nodes within the (shrinking) hyper ball of
        // (Edge::saturated) newNode
        std::shared_ptr<JList> nodeList = std::make_shared<JList>(true); // true argument for using KDTreeNodes as elements
        Tree->kdFindWithinRange( nodeList, hyperBallRad, newNode->position );

        // Try to find and link to best parent. This also saves
        // the edges from newNode to the neighbors in the field
        // "tempEdge" of the neighbors. This saves time in the
        // case that trajectory calculation is complicated.
        findBestParent( S, Tree, newNode, nodeList, closestNode, true );

        // If no parent was fonud then ignore this node
        if( !newNode->rrtParentUsed ) {
            Tree->emptyRangeList(nodeList); // clean up
            return false;
        }

        /* For RRTx, need to add the new node to its parent's successor list.
         * Place a (non-trajectory) reverse edge into newParent's successor
         * list and save a pointer to its position in that list. This edge
         * is used to help keep track of successors and not for movement.
         */
        std::shared_ptr<KDTreeNode> parentNode
                = newNode->rrtParentEdge->endNode;
        std::shared_ptr<Edge> backEdge = Edge::newEdge( parentNode, newNode );
        backEdge->dist = INF;
        parentNode->SuccessorList->JlistPush( backEdge, INF );
        newNode->successorListItemInParent
                = parentNode.get()->SuccessorList->front;

        // Insert the new node into the KDTree
        Tree->kdInsert(newNode);


        // Second pass, if there was a parent, then link with neighbors
        // and rewire neighbors that would do better to use newNode as
        // their parent. Note that the edges -from- newNode -to- its
        // neighbors have been stored in "tempEdge" field of the neighbors
        std::shared_ptr<JListNode> listItem = nodeList->front;
        std::shared_ptr<KDTreeNode> nearNode;
        std::shared_ptr<Edge> thisEdge;
        double oldLMC;
        for( int i = 0; i < nodeList->length; i++ ) {
            nearNode = listItem->node;

            // If edge from newNode to nearNode was valid
            if( listItem->key != INF /*&& listItem->key != -1.0*/ ) {
                // Add to initial out neighbor list of newNode
                // (allows info propogation from newNode to nearNode always)
                makeInitialOutNeighborOf( nearNode,newNode,nearNode->tempEdge );

                // Add to current neighbor list of newNode
                // (allows onfo propogation from newNode to nearNode and
                // vice versa, but only while they are in the D-ball)
                makeNeighborOf( nearNode, newNode, nearNode->tempEdge );
            }

            // In the general case, the trajectories along edges are not simply
            // the reverse of each other, therefore we need to calculate
            // and check the trajectory along the edge from nearNode to newNode
            thisEdge = Edge::newEdge( nearNode, newNode );
            thisEdge->calculateTrajectory( S, Tree );


            if( thisEdge->validMove(S)
                    && !explicitEdgeCheck(S,thisEdge) ) {
                // Add to initial in neighbor list of newnode
                // (allows information propogation from newNode to
                // nearNode always)
                makeInitialInNeighborOf( newNode, nearNode, thisEdge );

                // Add to current neighbor list of newNode
                // (allows info propogation from newNode to nearNode and
                // vice versa, but only while they are in D-ball)
                makeNeighborOf( newNode, nearNode, thisEdge );
            } else {
                // Edge cannot be created
                listItem = listItem->child; // iterate through list
                continue;
            }

            // Rewire neighbors that would do better to use this node
            // as their parent unless they are not in the relevant
            // portion of the space vs. moveGoal
            if( nearNode.get()->rrtLMC > newNode.get()->rrtLMC + thisEdge->dist
                    && newNode.get()->rrtParentEdge->endNode != nearNode
                    && newNode.get()->rrtLMC + thisEdge->dist < moveGoal->rrtLMC ) {
                // Make this node the parent of the neighbor node
                makeParentOf( newNode, nearNode, thisEdge, Tree->root );

                // Recalculate tree cost of neighbor
                oldLMC = nearNode.get()->rrtLMC;
                nearNode.get()->rrtLMC = newNode.get()->rrtLMC + thisEdge->dist;

                // Insert neighbor into priority queue if cost
                // reduction is great enough
                if( oldLMC - nearNode.get()->rrtLMC > Q->changeThresh
                        && nearNode != Tree->root ) {
                    verifyInQueue( Q, nearNode );
                }
            }

            listItem = listItem->child; // iterate through list
        }


        Tree->emptyRangeList(nodeList); // clean up

        // Insert the node into the priority queue
        Q->Q->addToHeap(newNode);

        return true;
    }
}


/////////////////////// RRT* Functions ///////////////////////

void findBestParent(std::shared_ptr<CSpace> S,
                    std::shared_ptr<KDTree> Tree,
                    std::shared_ptr<KDTreeNode> newNode,
                    std::shared_ptr<JList> nodeList,
                    std::shared_ptr<KDTreeNode> closestNode,
                    bool saveAllEdges)
{
    // If the list is empty
    if( nodeList->length == 0 ) {
        if( S->goalNode != newNode ) {
            nodeList->JlistPush( closestNode );
        }
    }

    // Update LMC value based on nodes in the list
    newNode->rrtLMC = INF;
    newNode->rrtTreeCost = INF;
    newNode->rrtParentUsed = false;

    // Find best parent (or if one even exists)
    std::shared_ptr<JListNode> listItem = nodeList->front;
    std::shared_ptr<KDTreeNode> nearNode;
    std::shared_ptr<Edge> thisEdge;
    while( listItem->child != listItem ) {
        nearNode = listItem->node;

        // First calculate the shortest trajectory (and its distance)
        // that gets from newNode to nearNode while obeying the
        // constraints of the state space and the dynamics
        // of the robot
        thisEdge = Edge::newEdge( newNode, nearNode );
        thisEdge->calculateTrajectory( S, Tree );

        if( saveAllEdges ) {
            nearNode->tempEdge = thisEdge;
        }

        // Check for validity vs edge collisions vs obstacles and
        // vs the time-dynamics of the robot and space
        if( explicitEdgeCheck(S,thisEdge) || !thisEdge->validMove(S) ) {
            if( saveAllEdges ) {
                nearNode->tempEdge->dist = INF;
            }

            listItem = listItem->child; // iterate through list
            continue;
        }

        if( newNode->rrtLMC > nearNode->rrtLMC + thisEdge->dist ) {
            // Found a potential better parent
            newNode->rrtLMC = nearNode->rrtLMC + thisEdge->dist;
            newNode->rrtParentEdge = thisEdge;
            newNode->rrtParentUsed = true;
        }

        listItem = listItem->child; // iterate thorugh list
    }
}


/////////////////////// RRT# Functions ///////////////////////
bool checkHeapForEdgeProblems( std::shared_ptr<Queue> Q )
{
    std::shared_ptr<KDTreeNode> node;
    for( int i = 0; i < Q->Q->indexOfLast; i++ ) {
        node = Q->Q->H[i];
        if( checkNeighborsForEdgeProblems( Q->S, node ) ) return true;
    }
    return false;
}

void resetNeighborIterator( std::shared_ptr<RRTNodeNeighborIterator> It )
{It->listFlag = 0;}

std::shared_ptr<JListNode> nextOutNeighbor( std::shared_ptr<RRTNodeNeighborIterator> It, std::shared_ptr<Queue> Q )
{
    if( typeid(*Q) == typeid(rrtSharpQueue) ) {
        if( It->listFlag == 0 ) {
            It->listItem = It->thisNode->rrtNeighborsOut->front;
            It->listFlag = 1;
        } else {
            It->listItem = It->listItem->child;
        }
        if( It->listItem == It->listItem->child ) {
            // Done with all neighbors
            return std::make_shared<JListNode>();
        }
        return It->listItem;
    }
    return std::make_shared<JListNode>();
}

std::shared_ptr<JListNode> nextInNeighbor( std::shared_ptr<RRTNodeNeighborIterator> It, std::shared_ptr<Queue> Q )
{
    if( typeid(*Q) == typeid(rrtSharpQueue) ) {
        if( It->listFlag == 0 ) {
            It->listItem = It->thisNode->rrtNeighborsIn->front;
            It->listFlag = 1;
        } else {
            It->listItem = It->listItem->child;
        }
        if( It->listItem == It->listItem->child ) {
            // Done with all neighbors
            return std::make_shared<JListNode>();
        }
        return It->listItem;
    }
    return std::make_shared<JListNode>();
}

void makeNeighborOf(std::shared_ptr<KDTreeNode> newNeighbor,
                    std::shared_ptr<KDTreeNode> node, std::shared_ptr<Edge> edge)
{
    node->rrtNeighborsOut->JlistPush( edge );
    edge->listItemInStartNode = newNeighbor->rrtNeighborsOut->front;

    newNeighbor->rrtNeighborsIn->JlistPush( edge );
    edge->listItemInEndNode = newNeighbor->rrtNeighborsIn->front;
}

void makeInitialOutNeighborOf(std::shared_ptr<KDTreeNode> newNeighbor,
                              std::shared_ptr<KDTreeNode> node, std::shared_ptr<Edge> edge)
{node->InitialNeighborListOut->JlistPush( edge );}

void makeInitialInNeighborOf(std::shared_ptr<KDTreeNode> newNeighbor,
                             std::shared_ptr<KDTreeNode> node, std::shared_ptr<Edge> edge)
{node->InitialNeighborListIn->JlistPush( edge );}

bool recalculateLMC(std::shared_ptr<Queue> Q, std::shared_ptr<KDTreeNode> node,
                    std::shared_ptr<KDTreeNode> root)
{
    if( node == root ) {
        return false;
    }

    //double oldrrtLMC = node->rrtLMC;

    std::shared_ptr<JListNode> listItem = node->rrtNeighborsOut->front;
    std::shared_ptr<Edge> neighborEdge;
    std::shared_ptr<KDTreeNode> neighborNode;
    double neighborDist;
    for( int i = 0; i < node->rrtNeighborsOut->length; i++ ) {
        neighborEdge = listItem->edge;
        neighborNode = neighborEdge->endNode;
        neighborDist = neighborEdge->dist;

        if( node->rrtLMC > neighborNode->rrtLMC + neighborDist
                && neighborEdge->validMove( Q->S ) ) {
            // Found a potentially better parent
            node->rrtLMC = neighborNode->rrtLMC + neighborDist;
            node->rrtParentEdge = listItem->edge;
            node->rrtParentUsed = true;
        }

        listItem = listItem->child; // iterate through list
    }
    return true;
}

void updateQueue( std::shared_ptr<Queue> Q, std::shared_ptr<KDTreeNode> newNode,
                  std::shared_ptr<KDTreeNode> root, double hyperBallRad )
{
    recalculateLMC( Q, newNode, root ); // internally ignores root
     if( Q->Q->markedQ( newNode ) ) {
         Q->Q->updateHeap( newNode );
         Q->Q->removeFromHeap( newNode );
     }
     if( newNode->rrtTreeCost != newNode->rrtLMC ) {
         Q->Q->addToHeap( newNode );
     }
}

void reduceInconsistency(std::shared_ptr<Queue> Q,
                         std::shared_ptr<KDTreeNode> goalNode,
                         double robotRad, std::shared_ptr<KDTreeNode> root,
                         double hyperBallRad)
{
    if( Q->type == "RRT#" ) {

        std::shared_ptr<KDTreeNode> thisNode, neighborNode;
        std::shared_ptr<JListNode> listItem;
        while( Q->Q->indexOfLast>0 && (Q->Q->lessQ((Q->Q->topHeap()),goalNode)
                                         || goalNode->rrtLMC == INF
                                         || goalNode->rrtTreeCost == INF
                                         || Q->Q->markedQ(goalNode)) ) {
            thisNode = Q->Q->popHeap();
            thisNode->rrtTreeCost = thisNode->rrtLMC;

            listItem = thisNode->rrtNeighborsIn->front;
            for( int i = 0; i < thisNode->rrtNeighborsIn->length; i++ ) {
                neighborNode = listItem->edge->startNode; // dereference?

                updateQueue( Q, neighborNode, root, hyperBallRad );

                listItem = listItem->child; // iterate through list
            }
        }
    } else { // Q->type == "RRTx";
        std::shared_ptr<KDTreeNode> thisNode;
        while( Q->Q->indexOfLast > 0
               && (Q->Q->lessQ((Q->Q->topHeap()), goalNode)
                   || goalNode->rrtLMC == INF
                   || goalNode->rrtTreeCost == INF
                   || Q->Q->markedQ(goalNode) ) ) {
            thisNode = Q->Q->popHeap();

            // Update neighbors of thisNode if it has
            // changed more than change thresh
            if( thisNode->rrtTreeCost - thisNode->rrtLMC > Q->changeThresh ) {
                recalculateLMCMineVTwo( Q, thisNode, root, hyperBallRad );
                rewire( Q, thisNode, root, hyperBallRad, Q->changeThresh );
            }
            thisNode->rrtTreeCost = thisNode->rrtLMC;
        }
    }
}


/////////////////////// RRTx Functions ///////////////////////

void markOS( std::shared_ptr<KDTreeNode> node )
{
    node.get()->inOSQueue = true;
}

void unmarkOS( std::shared_ptr<KDTreeNode> node )
{
    node.get()->inOSQueue = false;
}

bool markedOS( std::shared_ptr<KDTreeNode> node )
{
    return node.get()->inOSQueue;
}

bool verifyInQueue(std::shared_ptr<Queue> Q, std::shared_ptr<KDTreeNode> node)
{
    if( Q->Q->markedQ(node) ) {
       return Q->Q->updateHeap(node);  // should these return this?
    } else {
       return Q->Q->addToHeap(node);
    }
}

bool verifyInOSQueue(std::shared_ptr<Queue> Q,
                     std::shared_ptr<KDTreeNode> node)
{
    if( Q->Q->markedQ(node) ) {
        Q->Q->updateHeap(node);
        Q->Q->removeFromHeap(node);
    }
    if( !markedOS(node) ) {
        markOS(node);
        Q->OS->JlistPush(node);
    }
    return true;
}

void cullCurrentNeighbors( std::shared_ptr<KDTreeNode> node,
                           double hyperBallRad )
{
    // Remove outgoing edges from node that are now too long
    std::shared_ptr<JListNode> listItem = node->rrtNeighborsOut->front;
    std::shared_ptr<JListNode> nextItem;
    std::shared_ptr<KDTreeNode> neighborNode;
    std::shared_ptr<Edge> neighborEdge;
    while( listItem != listItem->child ) {
        nextItem = listItem->child; // since we may remove listItem from list
        if( listItem->edge->dist > hyperBallRad ) {
            neighborEdge = listItem->edge;
            neighborNode = neighborEdge->endNode;
            node->rrtNeighborsOut->JlistRemove(
                        neighborEdge->listItemInStartNode);
            node->rrtNeighborsIn->JlistRemove(
                        neighborEdge->listItemInEndNode);
        }
        listItem = nextItem;
    }
}

std::shared_ptr<JListNode> nextOutNeighbor(
        std::shared_ptr<RRTNodeNeighborIterator> It, Queue Q )
{
    if( It->listFlag == 0 ) {
        It->listItem = It->thisNode->InitialNeighborListOut->front;
        It->listFlag = 1;
    } else {
        It->listItem = It->listItem->child;
    }
    while( It->listItem == It->listItem->child ) {
        // Go to the next place tha neighbors are stored
        if( It->listFlag == 1 ) {
            It->listItem = It->thisNode->rrtNeighborsOut->front;
        } else {
            // Done with all neighbors
            return std::make_shared<JListNode>();
        }
        It->listFlag += 1;
    }
    return It->listItem;
}

std::shared_ptr<JListNode> nextInNeighbor(
        std::shared_ptr<RRTNodeNeighborIterator> It, Queue Q )
{
    if( It->listFlag == 0 ) {
        It->listItem = It->thisNode->InitialNeighborListIn->front;
        It->listFlag = 1;
    } else {
        It->listItem = It->listItem->child;
    }
    while( It->listItem == It->listItem->child ) {
        // Go to the next place that neighbors are stored
        if( It->listFlag == 1 ) {
            It->listItem = It->thisNode->rrtNeighborsIn->front;
        } else {
            // Done with all neighbors
            return std::make_shared<JListNode>();
        }
        It->listFlag += 1;
    }
    return It->listItem;
}

void makeParentOf( std::shared_ptr<KDTreeNode> newParent,
                   std::shared_ptr<KDTreeNode> node,
                   std::shared_ptr<Edge> edge,
                   std::shared_ptr<KDTreeNode> root )
{
    // Remove the node from its old parent's successor list
    if( node->rrtParentUsed ) {
        node->rrtParentEdge->endNode->SuccessorList->JlistRemove( node->successorListItemInParent );
    }

    // Make newParent the parent of node
    node->rrtParentEdge = edge;
    node->rrtParentUsed = true;

    // Place a (non-trajectory) reverse edge into newParent's
    // successor list and save a pointer to its position in
    // that list. This edge is used to help keep track of
    // successors and not for movement.
    std::shared_ptr<Edge> backEdge = Edge::Edge::newEdge( newParent, node );
    backEdge->dist = INF;
    newParent->SuccessorList->JlistPush( backEdge, INF );
    node->successorListItemInParent = newParent->SuccessorList->front;
}

bool recalculateLMCMineVTwo(std::shared_ptr<Queue> Q,
                            std::shared_ptr<KDTreeNode> node,
                            std::shared_ptr<KDTreeNode> root,
                            double hyperBallRad)
{
    if( node == root ) {
        return false;
    }

    bool newParentFound = false;
    double neighborDist;
    std::shared_ptr<KDTreeNode> rrtParent, neighborNode;
    std::shared_ptr<Edge> parentEdge, neighborEdge;
    std::shared_ptr<JListNode> listItem, nextItem;

    // Remove outdated nodes from current neighbors list
    cullCurrentNeighbors( node, hyperBallRad );

    // Get an iterator for this node's neighbors
    std::shared_ptr<RRTNodeNeighborIterator> thisNodeOutNeighbors
            = std::make_shared<RRTNodeNeighborIterator>(node);

    // Set the iterator to the first neighbor
    listItem = nextOutNeighbor( thisNodeOutNeighbors, Q );

    while( listItem->key != -1.0 ) {
        neighborEdge = listItem->edge;
        neighborNode = neighborEdge->endNode;
        neighborDist = neighborEdge->dist;
        nextItem = listItem->child;

        if( markedOS(neighborNode) ) {
            // neighborNode is already in OS queue (orphaned) or unwired
            listItem = nextOutNeighbor( thisNodeOutNeighbors, Q );
            continue;
        }

        if( node.get()->rrtLMC > neighborNode->rrtLMC + neighborDist &&
                (!neighborNode->rrtParentUsed ||
                neighborNode->rrtParentEdge->endNode != node) &&
                neighborEdge->validMove(Q->S) /*Not sure why this last part of this check is necessary*/ ) {
            // Found a better parent
            node->rrtLMC = neighborNode->rrtLMC + neighborDist;
            rrtParent = neighborNode;
            parentEdge = listItem->edge;
            newParentFound = true;
        }

        listItem = nextOutNeighbor( thisNodeOutNeighbors, Q );
    }

    if( newParentFound ) { // this node found a viable parent
        makeParentOf( rrtParent, node, parentEdge, root );
    }
    return true;
}

bool rewire( std::shared_ptr<Queue> Q, std::shared_ptr<KDTreeNode> node,
             std::shared_ptr<KDTreeNode> root,
             double hyperBallRad, double changeThresh )
{
    // Only explicitly propogate changes if they are large enough
    double deltaCost = node->rrtTreeCost - node->rrtLMC;
    if( deltaCost <= changeThresh ) {
        // Note that using simply "<" causes problems
        // May be outdated
        // node.rrtTreeCost = node.rrtLMC!!! Now happens after return
        return false;
    }

    // Remove outdated nodes from the current neighbors list
    cullCurrentNeighbors( node, hyperBallRad );

    // Get an iterator for this node's neighbors and iterate through list
    std::shared_ptr<RRTNodeNeighborIterator> thisNodeInNeighbors
            = std::make_shared<RRTNodeNeighborIterator>(node);
    std::shared_ptr<JListNode> listItem
            = nextInNeighbor( thisNodeInNeighbors, Q );
    std::shared_ptr<KDTreeNode> neighborNode;
    std::shared_ptr<Edge> neighborEdge;
    double deltaCostNeighbor;
    while( listItem->key != -1.0 ) {
        neighborEdge = listItem->edge;
        neighborNode = neighborEdge->startNode;

        // Ignore this node's parent and also nodes that cannot
        // reach node due to dynamics of robot or space
        // Not shure about second parent since neighbors are not
        // initially created that cannot reach this node
        if( (node->rrtParentUsed
             && node->rrtParentEdge->endNode == neighborNode)
                || !neighborEdge->validMove(Q->S) ) {
            listItem = nextInNeighbor( thisNodeInNeighbors, Q );
            continue;
        }

        neighborEdge = listItem->edge;

        deltaCostNeighbor = neighborNode->rrtLMC
                - (node->rrtLMC + neighborEdge->dist);
        if( deltaCostNeighbor > 0 ) {
            // neighborNode should use node as its parent (it might already)
            neighborNode->rrtLMC = node->rrtLMC + neighborEdge->dist;
            if( !neighborNode->rrtParentUsed
                    || neighborNode->rrtParentEdge->endNode != node ) {
                makeParentOf( node, neighborNode ,neighborEdge, root );

            }

            // If the reduction is great enough, then propogate
            // through the neighbor
            if( neighborNode->rrtTreeCost - neighborNode->rrtLMC
                    > changeThresh ) {
                verifyInQueue( Q, neighborNode );
            }
        }

        listItem = nextInNeighbor( thisNodeInNeighbors, Q );
    }
    return true;
}

bool propogateDescendants(std::shared_ptr<Queue> Q,
                          std::shared_ptr<RobotData> R)
{
    if( Q->OS->length <= 0 ) {
        return false;
    }

    // First pass, accumulate all such nodes in a single list, and mark
    // them as belonging in that list, we'll just use the OS stack we've
    // been using adding nodes to the front while moving from back to front
    std::shared_ptr<JListNode> OS_list_item = Q->OS->back;
    std::shared_ptr<KDTreeNode> thisNode, successorNode;
    std::shared_ptr<JListNode> SuccessorList_item;
    while( OS_list_item != OS_list_item->parent ) {
        thisNode = OS_list_item->node;

        // Add all of this node's successors to OS stack
        SuccessorList_item = thisNode->SuccessorList->front;
        while( SuccessorList_item != SuccessorList_item->child ) {
            successorNode = SuccessorList_item->edge->endNode;
            verifyInOSQueue( Q, successorNode ); // pushes to front of OS
            SuccessorList_item = SuccessorList_item->child;
        }

        OS_list_item = OS_list_item->parent;
    }

    // Second pass, put all -out neighbors- of the nodes in OS
    // (not including nodes in OS) into Q and tell them to force rewire.
    // Not going back to front makes Q adjustments slightly faster,
    // since nodes near the front tend to have higher costs
    OS_list_item = Q->OS->back;
    std::shared_ptr<JListNode> listItem;
    std::shared_ptr<KDTreeNode> neighborNode;
    while( OS_list_item != OS_list_item->parent ) {
        thisNode = OS_list_item->node;

        // Get an iterator for this node's neighbors
        std::shared_ptr<RRTNodeNeighborIterator> thisNodeOutNeighbors
                = std::make_shared<RRTNodeNeighborIterator>(thisNode);

        // Now iterate through list (add all neighbors to the Q,
        // except those in OS
        listItem = nextOutNeighbor( thisNodeOutNeighbors, Q );
        while( listItem->key != -1.0 ) {
            neighborNode = listItem->edge->endNode;

            if( markedOS(neighborNode) ) {
                // neighborNode already in OS queue (orphaned) or unwired
                listItem = nextOutNeighbor( thisNodeOutNeighbors, Q );
                continue;
            }

            // Otherwise, make sure that neighborNode is in normal queue
            neighborNode->rrtTreeCost = INF; // node will be inserted with LMC
                                             // key and then guarenteed to
                                             // propogate cost forward since
                                     // useful nodes have rrtLMC < rrtTreeCost
            verifyInQueue( Q, neighborNode );

            listItem = nextOutNeighbor( thisNodeOutNeighbors, Q );
        }

        // Add parent to the Q, unless it is in OS
        if( thisNode->rrtParentUsed
                && !markedOS((thisNode->rrtParentEdge->endNode)) ) {
            thisNode->kdParent->rrtTreeCost = INF; // rrtParent = kdParent???
            verifyInQueue( Q, thisNode->rrtParentEdge->endNode );
        }

        OS_list_item = OS_list_item->parent;
    }

    // Third pass, remove all nodes from OS, unmark them, and
    // remove their connections to their parents. If one was the
    // robot's target then take appropriate measures
    while( Q->OS->length > 0 ) {
        Q->OS->JlistPop(thisNode);
        unmarkOS(thisNode);

        if( thisNode == R->nextMoveTarget ) {
            R->currentMoveInvalid = true;
        }

        if( thisNode->rrtParentUsed ) {
            // Remove thisNode from its parent's successor list
            thisNode->rrtParentEdge->endNode->SuccessorList->JlistRemove( thisNode->successorListItemInParent );

            // thisNode now has no parent
            thisNode->rrtParentEdge = Edge::newEdge(thisNode, thisNode);
            thisNode->rrtParentEdge->dist = INF;
            thisNode->rrtParentUsed = false;
        }

        thisNode->rrtTreeCost = INF;
        thisNode->rrtLMC = INF;
    }
    return true;
}

void addOtherTimesToRoot( std::shared_ptr<CSpace> S,
                          std::shared_ptr<KDTree> Tree,
                          std::shared_ptr<KDTreeNode> goal,
                          std::shared_ptr<KDTreeNode> root,
                          std::string searchType )
{
    double insertStep = 2.0;

    double lastTimeToInsert = goal->position(2)
            - Tree->distanceFunction(root->position,goal->position)
            /S->robotVelocity;
    double firstTimeToInsert = S->start(2) + insertStep;
    std::shared_ptr<KDTreeNode> previousNode = root;
    bool safeToGoal = true;
    Eigen::VectorXd newPose;
    std::shared_ptr<KDTreeNode> newNode;
    std::shared_ptr<Edge> thisEdge;
    for( double timeToInsert = firstTimeToInsert;
         timeToInsert < lastTimeToInsert;
         timeToInsert += insertStep ) {
        newPose = root->position;
        newPose(2) = timeToInsert;

        newNode = std::make_shared<KDTreeNode>(newPose);

        // Edge from newNode to previousNode
        thisEdge = Edge::newEdge( newNode, previousNode );
        thisEdge->calculateHoverTrajectory( S );

        if( searchType == "RRT*" ) {
            // Make this node the parent of the neighbor node
            newNode->rrtParentEdge = thisEdge;
            newNode->rrtParentUsed = true;
        } else if( searchType == "RRT#" ) {
            // Make this node the parent of the neighbor node
            newNode->rrtParentEdge = thisEdge;
            newNode->rrtParentUsed = true;
            makeNeighborOf( newNode, previousNode, thisEdge );
        } else if( searchType == "RRTx" ) {
            makeParentOf( previousNode, newNode, thisEdge, root );
            makeInitialOutNeighborOf( previousNode, newNode, thisEdge );
            // Initial neighbor list edge
            makeInitialInNeighborOf( newNode, previousNode, thisEdge );
        }

        // Make sure this edge is safe
        if( explicitEdgeCheck( S, thisEdge ) ) {
            // Not safe
            thisEdge->dist = INF;
            safeToGoal = false;
            newNode->rrtLMC = INF;
            newNode->rrtTreeCost = INF;
        } else if( safeToGoal ) {
            // If the edge has safe path all the way to the "real" goal
            // then make the cost of reaching "real" goal 0 from newNode
            thisEdge->dist = 0.0;
            newNode->rrtLMC = 0.0;
            newNode->rrtTreeCost = 0.0;
        } else {
            thisEdge->dist = INF;
            newNode->rrtLMC = INF;
            newNode->rrtTreeCost = INF;
        }

        Tree->kdInsert(newNode);

        previousNode = newNode;
    }
}


// debug: gets goal node which should be too far away on first movement
void findNewTarget(std::shared_ptr<CSpace> S,
                    std::shared_ptr<KDTree> Tree,
                    std::shared_ptr<RobotData> R,
                    double hyperBallRad )
{
    R->robotEdgeUsed = false;
    R->distAlongRobotEdge = 0.0;
    R->timeAlongRobotEdge = 0.0;
    //R->robotEdgeForPlottingUsed = false;
    //R->distAlongRobotEdgeForPlotting = 0.0;
    //R->timeAlongRobotEdgeForPlotting = 0.0;

    // Move target has become invalid
    double searchBallRad = std::max(hyperBallRad,
                                    Tree->distanceFunction(R->robotPose,
                                                R->nextMoveTarget->position));
    // dist = KDTree::distanceFunction ?

    double maxSearchBallRad
            = Tree->distanceFunction(S->lowerBounds, S->upperBounds);
    searchBallRad = std::min( searchBallRad, maxSearchBallRad );
    std::shared_ptr<JList> L = std::make_shared<JList>(true);
    Tree->kdFindWithinRange( L, searchBallRad, R->robotPose );

    std::shared_ptr<KDTreeNode> dummyRobotNode
            = std::make_shared<KDTreeNode>(R->robotPose);
    std::shared_ptr<Edge> edgeToBestNeighbor
            = Edge::newEdge(dummyRobotNode,dummyRobotNode);

    double bestDistToNeighbor, bestDistToGoal;
    std::shared_ptr<KDTreeNode> bestNeighbor, neighborNode;

    while( true ) { // will break out when done
        // Searching for new target within radius searchBallRad
        bestDistToNeighbor = INF;
        bestDistToGoal = INF;
        bestNeighbor = std::make_shared<KDTreeNode>();

        std::shared_ptr<JListNode> ptr = L->front;
        std::shared_ptr<Edge> thisEdge;
        double distToGoal;
        while( ptr != ptr->child ) {
            neighborNode = ptr->node;

            thisEdge = Edge::newEdge( dummyRobotNode, neighborNode );
            thisEdge->calculateTrajectory( S, Tree );

            if( thisEdge->validMove(S)
                    && !explicitEdgeCheck(S,thisEdge) ) {
                // A safe point was found, see if it is the best so far
                distToGoal = neighborNode->rrtLMC + thisEdge->dist;
                if( distToGoal < bestDistToGoal
                        && thisEdge->validMove(S) ) {
                    // Found a new and better neighbor
                    bestDistToGoal = distToGoal;
                    bestDistToNeighbor = thisEdge->dist;
                    bestNeighbor = neighborNode;
                    edgeToBestNeighbor = thisEdge;
                } else { /*error("ptooie");*/ }
            }

            ptr = ptr->child;
        }
        // Done trying to find a target within ball radius of searchBallRad

        // If a valid neighbor was found, then use it
        if( bestDistToGoal != INF ) {
            R->nextMoveTarget = bestNeighbor;
            R->distanceFromNextRobotPoseToNextMoveTarget = bestDistToNeighbor;
            R->currentMoveInvalid = false;
            // Found a valid move target

            R->robotEdge = edgeToBestNeighbor; /** this edge is empty **/
            //R->robotEdgeForPlotting = edgeToBestNeighbor;
            R->robotEdgeUsed = true;
            //R->robotEdgeForPlottingUsed = true;

            if( S->spaceHasTime ) {
                R->timeAlongRobotEdge = 0.0;
                // note this is updated before robot moves
                //R->timeAlongRobotEdgeForPlotting = 0.0;
            } else {
                R->distAlongRobotEdge = 0.0;
                // note this is updated before robot moves
                //R->distAlongRobotEdgeForPlotting = 0.0;
            }

            // Set moveGoal to be nextMoveTarget
            // NOTE may want to actually insert a new node at the robot's
            // position and use that instead, since these "edges" created
            // between robot pose and R.nextMoveTarget may be lengthy
            S->moveGoal->isMoveGoal = false;
            S->moveGoal = R->nextMoveTarget;
            S->moveGoal->isMoveGoal = true;

            break;
        }

        searchBallRad *= 2;
        if( searchBallRad > maxSearchBallRad ) {
            // Unable to find a valid move target
            std::shared_ptr<KDTreeNode> newNode = randNodeDefault(S);
            double thisDist = Tree->distanceFunction(newNode->position,
                                                     R->robotPose);
            KDTree::saturate(
                        std::make_shared<Eigen::Vector4d>(newNode->position),
                        R->robotPose, DELTA, thisDist);
            Tree->kdInsert(newNode);
        }
        Tree->kdFindMoreWithinRange( L, searchBallRad, R->robotPose );

    }
    Tree->emptyRangeList(L); // cleanup
}

void moveRobot(std::shared_ptr<CSpace> S, std::shared_ptr<Queue> Q,
                std::shared_ptr<KDTree> Tree, double slice_time,
                std::shared_ptr<KDTreeNode> root, double hyperBallRad,
                std::shared_ptr<RobotData> R )
{
    // Start by updating the location of the robot based on how
    // it moved since the last update (as well as the total path that
    // it has followed)
    if( R->moving ) {

        std::cout << "Moving "
                  << Tree->distanceFunction(R->robotPose,R->nextRobotPose)
                  << " units" << std::endl;
        R->robotPose = R->nextRobotPose;
        //R.robotMovePath[R.numRobotMovePoints+1:R.numRobotMovePoints+R.numLocalMovePoints,:] = R.robotLocalPath[1:R.numLocalMovePoints,:];
        for( int i = 0; i < R->numLocalMovePoints-1; i++ ) {
            R->robotMovePath.row(R->numRobotMovePoints+i) = R->robotLocalPath.row(i);
        }
        R->numRobotMovePoints += R->numLocalMovePoints;

        if( !S->spaceHasTime ) {
            // new robot pose (R.robotPose(0) R.robotPose(1))
            std::cout << "new robot pose(w/o t):\n" << R->robotPose << std::endl;
        } else {
            // new robot pose (R.robotPose(0) R.robotPose(1) R.robotPose(2))
            std::cout << "new robot pose(w/ t):\n" << R->robotPose << std::endl;
        }

        // Save stuff for plotting
//        if( S->spaceHasTime ) {
//            R->timeAlongRobotEdgeForPlotting = R->timeAlongRobotEdge;
//        } else {
//            R->distAlongRobotEdgeForPlotting = R->distAlongRobotEdge;
//        }
//        R->robotEdgeForPlotting = R->robotEdge;
//        R->robotEdgeForPlottingUsed = true;
    } else {
        // Movement has just started, so remember that the robot is now moving
        R->moving = true;

        error("First pose:");
        std::cout << R->robotPose << std::endl;

        if( !S->moveGoal->rrtParentUsed ) {
            // no parent has been found for the node at the robots position
            R->currentMoveInvalid = true;
        } else {
            R->robotEdge = S->moveGoal->rrtParentEdge;
//            R->robotEdgeForPlotting = R->robotEdge;
            R->robotEdgeUsed = true;
//            R->robotEdgeForPlottingUsed = true;

            if( S->spaceHasTime ) {
                R->timeAlongRobotEdge = 0.0;
//                R->timeAlongRobotEdgeForPlotting = 0.0;
            } else {
                R->distAlongRobotEdge = 0.0;
//                R->distAlongRobotEdgeForPlotting = 0.0;
            }
        }
    }

    // If the robot's current move target has been invalidate due to
    // dynamic obstacles then we need to attempt to find a new
    // (safe) move target. NOTE we handle newly invalid moveTarget
    // after moving the robot (since the robot has already moved this
    // time slice)
    if( R->currentMoveInvalid ) {
        findNewTarget( S, Tree, R, hyperBallRad );
    } else {
        /* Recall that moveGoal is the node whose key is used to determine
         * the level set of cost propogation (this should theoretically
         * be further than the robot from the root of the tree, which
         * will happen here assuming that robot moves at least one edge
         * each slice time. Even if that does not happen, things will
         * still be okay in practice as long as robot is "close" to moveGoal
         */
        S->moveGoal->isMoveGoal = false;
        S->moveGoal = R->nextMoveTarget;
        S->moveGoal->isMoveGoal = true;
    }


    /* Finally, we calculate the point to which the robot will move in
    * slice_time and remember it for the next time this function is called.
    * Also remember all the nodes that it will visit along the way in the
    * local path and the part of the edge trajectory that takes the robot
    * to the first local point (the latter two things are used for
    * visualizition)
    */
    if( !S->spaceHasTime ) {
        // Not using the time dimension, so assume speed is equal to robotVelocity
        std::shared_ptr<KDTreeNode> nextNode = R->nextMoveTarget;

        // Calculate distance from robot to the end of
        // the current edge it is following
        double nextDist = R->robotEdge->dist - R->distAlongRobotEdge;

        double distRemaining = S->robotVelocity*slice_time;

        // Save first local path point
        R->numLocalMovePoints = 1;
        R->robotLocalPath.row(R->numLocalMovePoints-1) = R->robotPose;

        // Starting at current location (and looking ahead to nextNode), follow
        // parent pointers back for appropriate distance (or root or dead end)
        while( nextDist <= distRemaining && nextNode != root
               && nextNode->rrtParentUsed
               && nextNode != nextNode->rrtParentEdge->endNode ) {
            // Can go all the way to nextNode and still have
            // some distance left to spare

            // Remember robot will move through this point
            R->numLocalMovePoints += 1;
            R->robotLocalPath.row(R->numLocalMovePoints) = nextNode->position;

            // Recalculate remaining distance
            distRemaining -= nextDist;

            // Reset distance along edge
            R->distAlongRobotEdge = 0.0;

            // Update trajectory that the robot will be in the middle of
            R->robotEdge = nextNode->rrtParentEdge;
            R->robotEdgeUsed = true;

            // Calculate the dist of that trajectory
            nextDist = R->robotEdge->dist;

            // Update the next node (at the end of that trajectory)
            nextNode = R->robotEdge->endNode;
        }

        // either 1) nextDist > distRemaining
        // or     2) the path we were following now ends at nextNode

        // Calculate the next pose of the robot
        if( nextDist > distRemaining ) {
            R->distAlongRobotEdge += distRemaining;
            R->nextRobotPose
                    = R->robotEdge->poseAtDistAlongEdge(R->distAlongRobotEdge);
        } else {
            R->nextRobotPose = nextNode->position;
            R->distAlongRobotEdge = R->robotEdge->dist;
        }

        R->nextMoveTarget = R->robotEdge->endNode;

        // Remember last point in local path
        R->numLocalMovePoints += 1;
        R->robotLocalPath.row(R->numLocalMovePoints) = R->nextRobotPose;
    } else { // S->spaceHasTime
        // Space has time, so path is parameterized by time as well
        std::shared_ptr<KDTreeNode> nextNode = R->nextMoveTarget;

        // Save first local path point
        R->numLocalMovePoints = 1;
        R->robotLocalPath.row(R->numLocalMovePoints) = R->robotPose;

        double targetTime = R->robotPose(2) - slice_time;
        while( targetTime < R->robotEdge->endNode->position(2)
               && nextNode != root && nextNode->rrtParentUsed
               && nextNode != nextNode->rrtParentEdge->endNode ) {
            // Can go all the way to nextNode and still have some
            // time left to spare

            // Remember the robot will move through this point
            R->numLocalMovePoints += 1;
            R->robotLocalPath.row(R->numLocalMovePoints) = nextNode->position;

            // Update trajectory that the robot will be in the middle of
            R->robotEdge = nextNode->rrtParentEdge;
            R->robotEdgeUsed = true;

            // Update the next node (at the end of that trajectory)
            nextNode = nextNode->rrtParentEdge->endNode;
        }

        // either: 1) targetTime >= nextNode.position(2)
        // or      2) the path we were following now ends at nextNode

        // Calculate the next pose of the robot
        if( targetTime >= nextNode->position(2) ) {
            R->timeAlongRobotEdge = R->robotEdge->startNode->position(2)
                    - targetTime;
            R->nextRobotPose
                    = R->robotEdge->poseAtTimeAlongEdge(R->timeAlongRobotEdge);
        } else {
            // The next node is the end of this tree and we reach it
            R->nextRobotPose = nextNode->position;
            R->timeAlongRobotEdge = R->robotEdge->startNode->position(2)
                    - R->robotEdge->endNode->position(2);
        }

        R->nextMoveTarget = R->robotEdge->endNode;

        // Remember the last point in the local path
        R->numLocalMovePoints += 1;
        R->robotLocalPath.row(R->numLocalMovePoints) = R->nextRobotPose;
    }
}
