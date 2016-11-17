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
        return new KDTreeNode( S->sampleStack->JlistPop()->position );
    } else {
        return randNodeOrGoal( S );
    }
}

KDTreeNode* randNodeInTimeOrFromStack( CSpace* S )
{
    if( S->sampleStack->length > 0 ) {
        // Using the sampleStack so KDTreeNode->position is popped
        return new KDTreeNode( S->sampleStack->JlistPop()->position );
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


/////////////////////// RRT Functions ///////////////////////
bool extendRRT( CSpace* S, KDTree* Tree, KDTreeNode* newNode,
                KDTreeNode* closestNode, double delta,
                double hyperBallRad, KDTreeNode* moveGoal )
{
    // First calculate the shortest trajectory (and its distance) that
    // gets from newNode to closestNode while obeying the constraints
    // of the state space and the dynamics of the robot
    Edge* thisEdge = new Edge( newNode, closestNode );
    calculateTrajectory( S, thisEdge );

    // Figure out if we can link to the nearest node
    if ( !validMove( S, thisEdge ) /*|| explicitEdgeCheck( S, thisEdge)*/ ) {
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
}


/////////////////////// RRT* Functions ///////////////////////
KDTreeNode* findBestParent( CSpace* S, KDTreeNode* newNode, JList nodeList,
                            KDTreeNode* closestNode, bool saveAllEdges )
{
    return new KDTreeNode();
}

bool extendRRTStar( CSpace* S, KDTree* Tree, KDTreeNode* newNode,
                    KDTreeNode* closestNode, double delta,
                    double hyperBallRad, KDTreeNode* moveGoal )
{
    return false;
}


/////////////////////// RRT# Functions ///////////////////////


/////////////////////// main (despite the name) ///////////////////////
void RRTX( CSpace *S, double total_planning_time, double slice_time,
           double delta, double ballConstant, double changeThresh,
           std::string searchType, bool MoveRobotFlag,
           bool saveVideoData, bool saveTree, std::string dataFile )
{
    std::cout << "Hey" << std::endl;
}
