/* drrt.cpp
 * Corin Sandford
 * Fall 2016
 * Contains RRTX "main" function at bottom.
 */

#include <DRRT/drrt.h>

int randInt( int min, int max )
{
    std::random_device rd;
    std::mt19937 rng(rd());     // Mersenn-Twister random-number engine
    std::uniform_int_distribution<int> uni(min,max);
    int random_integer = uni(rng);
    return random_integer;
}


/////////////////////// Node Functions ///////////////////////

double distance( KDTreeNode* x, KDTreeNode* y )
{
    return distFunc( "threeTwoDRobotDist", x->position, y->position );
}

double Wdist( KDTreeNode* x, KDTreeNode* y )
{
    return distFunc( "threeTwoDRobotDist", x->position, y->position );
}

double extractPathLength( KDTreeNode* node, KDTreeNode* root )
{
    double pathLength = 0.0;
    KDTreeNode* thisNode = node;
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

Eigen::VectorXd randPointDefault( CSpace* S )
{
    int rand = randInt(1,S->d);
    Eigen::VectorXd first;
    Eigen::VectorXd second;

    for( int i = 0; i < S->lowerBounds.size(); i++ ) {
        first(i) = rand * S->width(i);
        second(i) = S->lowerBounds(i) + first(i);
    }
    return second;
}

KDTreeNode* randNodeDefault( CSpace* S )
{
    Eigen::VectorXd point = randPointDefault( S );
    return new KDTreeNode( point );
}

KDTreeNode* randNodeOrGoal( CSpace* S )
{
    int rand = randInt(0,1);
    if( rand > S->pGoal ) {
        return randNodeDefault( S );
    } else {
        return S->goalNode;
    }
}

KDTreeNode* randNodeIts( CSpace* S )
{
    if( S->itsUntilSample == 0 ) {
        S->itsUntilSample -= 1;
        return new KDTreeNode( S->itsSamplePoint );
    }
    S->itsUntilSample -= 1;
    return randNodeOrGoal( S );
}

KDTreeNode* randNodeTime( CSpace* S )
{
    if( S->waitTime != INF && S->timeElapsed >= S->waitTime ) {
        S->waitTime = INF;
        return new KDTreeNode( S->timeSamplePoint );
    }
    return randNodeOrGoal( S );
}

//KDTreeNode* randNodeTimeWithObstacleRemove( CSpace* S ){}
//KDTreeNode* randNodeItsWithObstacleRemove( CSpace* S ){}

KDTreeNode* randNodeOrFromStack( CSpace* S )
{
    if( S->sampleStack->length > 0 ) {
        // Using the sampleStack so KDTreeNode->position is popped
        KDTreeNode* temp = new KDTreeNode();
        S->sampleStack->JlistPop(temp);
        return new KDTreeNode( temp->position );
    } else {
        return randNodeOrGoal( S );
    }
}

KDTreeNode* randNodeInTimeOrFromStack( CSpace* S )
{
    if( S->sampleStack->length > 0 ) {
        // Using the sampleStack so KDTreeNode->position is popped
        KDTreeNode* temp = new KDTreeNode();
        S->sampleStack->JlistPop(temp);
        return new KDTreeNode( temp->position );
    } else {
        KDTreeNode* newNode = randNodeOrGoal( S );
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
            newNode->position(2) = minTimeToReachNode + randInt(0,1) * (S->moveGoal->position(2) - minTimeToReachNode);
        }
        return newNode;
    }
}

bool checkNeighborsForEdgeProblems( CSpace* S, KDTreeNode* thisNode )
{
    if( thisNode->rrtParentUsed ) {
//        if( explicitEdgeCheck( S, thisNode, thisNode->rrtParentEdge->endNode ) ) {
//            return true;
//        }
    }

    JListNode* listItem = thisNode->rrtNeighborsOut->front;
    KDTreeNode* neighborNode;
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

bool explicitEdgeCheck( CSpace* S, Edge* edge )
{
    // If ignoring obstacles
    if( S->inWarmupTime ) return false;

//    JListNode* obstacleListNode = S->obstacles.front;
//    for( int i = 0; i < obstacles.length; i++ ) {
//        if( explicitEdgeCheck( S, edge, obstacleListNode->node) ) { // obstacleListNode.data ( also maybe should be explicitNodeCheck? )
//            return true;
//        }
//        obstacleListNode = obstacleListNode->child; // iterate
//    }
    return false;
}

/////////////////////// RRT Functions ///////////////////////

bool extend( CSpace* S, KDTree* Tree, Queue* Q, KDTreeNode* newNode,
                KDTreeNode* closestNode, double delta,
                double hyperBallRad, KDTreeNode* moveGoal )
{
    if( typeid(*Q) == typeid(rrtQueue) ) {
        // First calculate the shortest trajectory (and its distance) that
        // gets from newNode to closestNode while obeying the constraints
        // of the state space and the dynamics of the robot
        Edge* thisEdge = new Edge( newNode, closestNode );
        calculateTrajectory( S, thisEdge );

        // Figure out if we can link to the nearest node
        if ( !validMove( S, thisEdge ) || explicitEdgeCheck( S, thisEdge) ) {
            // We cannot link to nearest neighbor
            return false;
        }

        // Otherwise we can link to the nearest neighbor
        newNode->rrtParentEdge = thisEdge;
        newNode->rrtTreeCost = closestNode->rrtLMC + newNode->rrtParentEdge->dist;
        newNode->rrtLMC = newNode->rrtTreeCost; // only for compatability with visualization
        newNode->rrtParentUsed = true;

        // insert the new node into the KDTree
        return kdInsert( Tree, newNode );
    } else if( typeid(*Q) == typeid(rrtStarQueue) ) {
        // Find all nodes within the (shrinking hyperball of (saturated) newNode
        JList* nodeList = new JList(true);
        kdFindWithinRange( nodeList, Tree, hyperBallRad, newNode->position );

        // Try to find and link to best parent
        findBestParent( S, newNode, nodeList, closestNode, false); // saveAllEdges: false b/c no implementation for saving
        if( !newNode->rrtParentUsed ) {
            emptyRangeList( nodeList ); // clean up
            return false;
        }

        newNode->rrtTreeCost = newNode->rrtLMC;

        // Insert new node into the KDTre
        kdInsert( Tree, newNode );

        // If this is inserted in a an unhelpful part of the C-space then
        // don't waste time rewiring (assumes triange inequality, added
        // by MO, not technically part of RRT* but can only improve it)
        if( newNode->rrtLMC > moveGoal->rrtLMC ) {
            emptyRangeList( nodeList ); // clean up
            return false;
        }

        // Now rewire neighbors that should use newNode as their parent
        JListNode* listItem = nodeList->front;
        KDTreeNode* nearNode;
        Edge* thisEdge;
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
            thisEdge = newEdge( nearNode, newNode );
            calculateTrajectory( S, thisEdge );

            // Rewire neighbors that would do betten to use this node
            // as their parent unless they are in collision or
            // impossible due to dynamics of robot/space
            if( nearNode->rrtLMC > newNode->rrtLMC + thisEdge->dist && validMove( S, thisEdge ) && !explicitEdgeCheck( S, thisEdge ) ) {
                // Make this node the parent of the neighbor node
                nearNode->rrtParentEdge = thisEdge;
                nearNode->rrtParentUsed = true;

                // Recalculate tree cost of neighbor
                nearNode->rrtTreeCost = nearNode->rrtLMC + thisEdge->dist;
                nearNode->rrtLMC = newNode->rrtLMC + thisEdge->dist;
            }
            listItem = listItem->child; // iterate through list
        }
        emptyRangeList( nodeList ); // clean up
        return true;
    } else if( typeid(*Q) == typeid(rrtSharpQueue) ) {
        return kdInsert( Tree, newNode );
    } else { // typeid(*Q) == typeid(rrtXQueue)
        return kdInsert( Tree, newNode );
    }
}


/////////////////////// RRT* Functions ///////////////////////

void findBestParent( CSpace* S, KDTreeNode* newNode, JList* nodeList,
                     KDTreeNode* closestNode, bool saveAllEdges )
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
    JListNode* listItem = nodeList->front;
    KDTreeNode* nearNode;
    Edge* thisEdge;
    while( listItem->child != listItem ) {
        nearNode = listItem->node;

        // First calculate the shortest trajectory (and its distance)
        // that gets from newNode to nearNode while obeying the
        // constraints of the state space and the dynamics
        // of the robot
        thisEdge = newEdge( newNode, nearNode );
        calculateTrajectory( S, thisEdge );

        if( saveAllEdges ) {
            nearNode->tempEdge = thisEdge;
        }

        // Check for validity vs edge collisions vs obstacles and
        // vs the time-dynamics of the robot and space
        if( explicitEdgeCheck(S,thisEdge) || !validMove(S,thisEdge) ) {
            if( saveAllEdges ) {
                nearNode->tempEdge->dist = INF;
            }

            listItem = listItem->child; // iterate through list
            continue;
        }

        if( newNode->rrtLMC > nearNode->rrtLMC + thisEdge->dist ) {
            // Found a potentiall better parent
            newNode->rrtLMC = nearNode->rrtLMC + thisEdge->dist;
            newNode->rrtParentEdge = thisEdge;
            newNode->rrtParentUsed = true;
        }

        listItem = listItem->child; // iterate thorugh list
    }
}


/////////////////////// RRT# Functions ///////////////////////
bool checkHeapForEdgeProblems( Queue* Q )
{
    KDTreeNode* node;
    for( int i = 0; i < Q->Q->indexOfLast; i++ ) {
        node = &Q->Q->H[i];
        if( checkNeighborsForEdgeProblems( Q->S, node ) ) return true;
    }
    return false;
}

void resetNeighborIterator( RRTNodeNeighborIterator* It )
{
    It->listFlag = 0;
}

JListNode* nextOutNeighbor( RRTNodeNeighborIterator* It, Queue* Q )
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
            return new JListNode();
        }
        return It->listItem;
    }
    return new JListNode();
}

JListNode* nextInNeighbor( RRTNodeNeighborIterator* It, Queue* Q )
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
            return new JListNode();
        }
        return It->listItem;
    }
    return new JListNode();
}

void makeNeighborOf( KDTreeNode* newNeighbor, KDTreeNode* node, Edge* edge )
{
    node->rrtNeighborsOut->JlistPush( edge );
    edge->listItemInStartNode = newNeighbor->rrtNeighborsOut->front;

    newNeighbor->rrtNeighborsIn->JlistPush( edge );
    edge->listItemInEndNode = newNeighbor->rrtNeighborsIn->front;
}

void makeInitialOutNeighborOf( KDTreeNode* newNeighbor, KDTreeNode* node, Edge* edge )
{
    node->InitialNeighborListOut->JlistPush( edge );
}

void makeInitialInNeighborOf( KDTreeNode* newNeighbor, KDTreeNode* node, Edge* edge )
{
    node->InitialNeighborListIn->JlistPush( edge );
}



void reduceInconsistency( Queue* Q, KDTreeNode* goalNode, double robotRad,
                          KDTreeNode* root, double hyperBallRad )
{
    if( typeid(*Q) == typeid(rrtSharpQueue) ) {

    } else { // typeid(*Q) == typeid(rrtXQueue)

    }
}


/////////////////////// RRTx Functions ///////////////////////

void markOS( KDTreeNode* node )
{

}

void unmarkOS( KDTreeNode* node )
{

}

bool markedOS( KDTreeNode* node )
{
    return true;
}

void moveRobot( CSpace* S, BinaryHeap* Q, KDTree* Tree, double slice_time,
                KDTreeNode* root, double hyperBallRad, RobotData* R )
{

}

/////////////////////// main (despite the name) ///////////////////////
void RRTX( CSpace *S, double total_planning_time, double slice_time,
           double delta, double ballConstant, double changeThresh,
           std::string searchType, bool MoveRobotFlag,
           bool saveVideoData, bool saveTree, std::string dataFile )
{
    //double robotSensorRange = 20.0; // used for "sensing" obstacles (should make input)

    double startTime = time(0)*1000;    // time in milliseconds
    double saveElapsedTime = 0.0;       // will hold save time to correct for time spent
                                        // not in the algorithm itself

    // Initialization stuff

    Eigen::VectorXi wraps;
    wraps(0) = 3;
    Eigen::VectorXd wrapPoints;
    wrapPoints(0) = 2.0*PI;

    // KDtree
    // Dubin's car so 4th dimension wraps at 0 = 2pi
    KDTree* KD = new KDTree( S->d,"KDdist", wraps, wrapPoints );

    Queue* Q = new Queue();
    if( searchType == "RRT" ) {
        rrtQueue* Q = new rrtQueue();
        Q->S = S;
    } else if( searchType == "RRT*" ) {
        rrtStarQueue* Q = new rrtStarQueue();
        Q->S = S;
    } else if( searchType == "RRT#" ) {
        rrtSharpQueue* Q = new rrtSharpQueue();
        Q->Q = new BinaryHeap(false); // false >> use priority queue functions (keyQ)
                                      // sorted based on cost from goal
        Q->S = S;
    } else if( searchType == "RRTx" ) {
        rrtXQueue* Q = new rrtXQueue();
        Q->Q = new BinaryHeap(false); // false >> use priority queue functions (keyQ)
                                      // sorted based on cost from goal
        Q->OS = new JList(true); // Obstacle successor stack
                                 // true >> uses KDTreeNode's (Needs Obstacles??)
        Q->S = S;
        Q->changeThresh = changeThresh;
    } else {
        std::cout << "Unknown search type: " << searchType << std::endl;
        exit(1);
    }

    S->sampleStack = new JList(true);   // stores a stack of points that we desire
                                        // to insert in the future (used when an
                                        // obstacle is removed) true >> uses KDTreeNode's

    S->delta = delta;

    double robotRads = S->robotRadius;

    // Define root node in the search tree graph
    KDTreeNode* root = new KDTreeNode(S->start);

    // Explicit check root
    //explicitNodeCheck(S,root);

    root->rrtTreeCost = 0.0;
    root->rrtLMC = 0.0;

    // Insert the root into the KDTree
    kdInsert(KD,root);

    // Define a goal node
    KDTreeNode* goal = new KDTreeNode(S->goal);
    goal->rrtTreeCost = INF;
    goal->rrtLMC = INF;
    S->goalNode = goal;
    S->root = root;

    S->moveGoal = goal;     // this will store a node at least as far from
                            // the root as the robot. During movement its key
                            // is used to limit propogation beyond the region
                            // we care about

    S->moveGoal->isMoveGoal = true;

    // Parameters that have to do with the robot path following simulation
    RobotData R = RobotData( S->goal, goal, 20000 );

    double sliceCounter = 0; // helps with saving accurate time data

    if( S->spaceHasTime ) {
        // Add other "times" to root of tree
        // addOtherTimesToRoot( S, KD, goal, root, searchType );
    }

    // End of initialization

    // While planning time left, plan (will break out when done)
    double robot_slice_start = time(0)*1000.0; // time in milliseconds
    S->startTimeNs = robot_slice_start/1000000.0; // time in nanoseconds
    S->timeElapsed = 0.0;
    double slice_end_time;

    double oldrrtLMC = INF;
    double now_time = time(0)*1000.0;
    bool warmUpTimeJustEnded;
    double truncElapsedTime;

    while( true ) {
        double hyperBallRad = std::min( delta, ballConstant*(pow(std::log(1+KD->treeSize)/(KD->treeSize),1/S->d)));
        now_time = time(0)*1000.0;

        // Calculate the end time of the first slice
        slice_end_time = (1+sliceCounter)*slice_time;

        // See if warmup time has ended
        warmUpTimeJustEnded = false;
        if( S->inWarmupTime && S->warmupTime < S->timeElapsed ) {
            warmUpTimeJustEnded = true;
            S->inWarmupTime = false;
        }

        // If this robot has used all of its allotted planning time of this slice
        S->timeElapsed = (time(0)/1000 - S->startTimeNs)/1000000000 - saveElapsedTime;
        if( S->timeElapsed >= slice_end_time ) {
            // Calculate the end time of the next slice
            slice_end_time = (1+sliceCounter)*slice_time;

            robot_slice_start = now_time;
            sliceCounter += 1;
            truncElapsedTime = floor(S->timeElapsed*1000)/1000;

            // Move robot if the robot is allowed to move,
            // otherwise planning is finished so break
            /* Changed elapsedTime[checkPtr] to S->timeElapsed since I believe they are the same at this point */
            if( S->timeElapsed > total_planning_time + slice_time ) {
                if( MoveRobotFlag ) {
                    moveRobot( S,Q->Q,KD,slice_time,root,hyperBallRad,&R ); // Assumes Q is rrtxQueue and Q.Q is its BinaryHeap
                } else {
                    std::cout << "done (robot not moved)" << std::endl;
                    break;
                }
            }

            // Make graph consistent (RRT# and RRTx)
            if( searchType == "RRTx" || searchType == "RRT#" ) {
                reduceInconsistency(Q, S->moveGoal, robotRads, root, hyperBallRad );
                if( S->moveGoal->rrtLMC != oldrrtLMC ) {
                    oldrrtLMC = S->moveGoal->rrtLMC;
                }
            }

            // Check if robot has reached its movement goal
            if( R.robotPose == root->position ) {
                break;
            }

            // START normal graph search stuff
            // Pick a random node
            KDTreeNode* newNode = randNodeOrFromStack( S );

            // Happens when we explicitly sample the goal every so often
            if( newNode->kdInTree ) {
                // Nodes will be updated automatically as
                // information gets propogated to it
                continue;
            }

            // Find closest old node to the new node
            KDTreeNode* closestNode = new KDTreeNode();
            double* closestDist = NULL;
            kdFindNearest( closestNode, closestDist, KD, newNode->position );

            // Saturate
            if( *closestDist > delta && newNode != S->goalNode ) {
                saturate( newNode->position, closestNode->position, delta );
            }

            // Check for collisions vs obstacles
            //bool explicitlyUnSafe;
            //explicityNodeCheck( explicitlyUnSafe, S, newNode );
            //if( explicitlyUnSafe ) {
            //    continue;
            //}

            // Extend
            extend( S, KD, Q, newNode, closestNode, delta, hyperBallRad, S->moveGoal );

            // Make graph consistent (RRT# and RRTx)
            if( searchType == "RRTx" || searchType == "RRT#" ) {
                reduceInconsistency(Q, S->moveGoal, robotRads, root, hyperBallRad );
                if( S->moveGoal->rrtLMC != oldrrtLMC ) {
                    oldrrtLMC = S->moveGoal->rrtLMC;
                }
            }
        }
    }

    Eigen::ArrayXd firstpoints, lastpoints;
    firstpoints = R.robotMovePath.block( 0,0, R.numRobotMovePoints-1,6 );
    lastpoints = R.robotMovePath.block( 1,0, R.numRobotMovePoints-1,6 );

    double moveLength = 0.0;
    std::cout << "Robot traveled: " << moveLength << std::endl;

    double totalTime = time(0)*1000.0 - startTime;
    std::cout << "Total time: " << totalTime << std::endl;
}
