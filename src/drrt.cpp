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

float distance( KDTreeNode* x, KDTreeNode* y )
{
    return distFunc( "threeTwoDRobotDist", x->position, y->position );
}

float Wdist( KDTreeNode* x, KDTreeNode* y )
{
    return distFunc( "threeTwoDRobotDist", x->position, y->position );
}

float extractPathLength( KDTreeNode* node, KDTreeNode* root )
{
    float pathLength = 0.0;
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

std::vector<float> randPointDefault( CSpace* S )
{
    int rand = randInt(1,S->d);
    std::vector<float> first;
    std::vector<float> second;

    for( int i = 0; i < S->lowerBounds.size(); i++ ) {
        first[i] = rand * S->width[i];
        second[i] = S->lowerBounds[i] + first[i];
    }
    return second;
}

KDTreeNode* randNodeDefault( CSpace* S )
{
    std::vector<float> point = randPointDefault( S );
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

        float minTimeToReachNode = S->start[3] + sqrt( (newNode->position[1] - S->root->position[1])*(newNode->position[1] - S->root->position[1]) + (newNode->position[2] - S->root->position[2])*(newNode->position[2] - S->root->position[2]) ) / S->robotVelocity;

        // If point is too soon vs robot's available speed
        // or if it is in the "past" and the robot is moving
        if( newNode->position[3] < minTimeToReachNode ||
                (newNode->position[3] > S->moveGoal->position[3] &&
                 S->moveGoal != S->goalNode) ) {
            // Resample time in ok range
            newNode->position[3] = minTimeToReachNode + randInt(0,1) * (S->moveGoal->position[3] - minTimeToReachNode);
        }
        return newNode;
    }
}


/////////////////////// Geometric Functions ///////////////////////

float distanceSqrdPointToSegment( std::vector<float> point, std::vector<float> startPoint,
                                  std::vector<float> endPoint )
{
    float vx = point[1] - startPoint[1];
    float vy = point[2] - startPoint[2];
    float ux = endPoint[1] - startPoint[1];
    float uy = endPoint[2] - startPoint[2];
    float determinate = vx*ux + vy*uy;

    if( determinate <= 0 ) {
        return vx*vx + vy*vy;
    } else {
        float len = ux*ux + uy*uy;
        if( determinate >= len ) {
            return (endPoint[1]-point[1])*(endPoint[1]-point[1]) +
                    (endPoint[2]-point[2])*(endPoint[2]-point[2]);
        } else {
            return (ux*vy - uy*vx)*(ux*vy - uy*vx) / len;
        }
    }
}

float segmentDistSqrd(std::vector<float> PA, std::vector<float> PB,
                      std::vector<float> QA, std::vector<float> QB)
{
    // Check if the points are definately not in collision by seeing
    // if both points of Q are on the same side of line containing P and vice versa

    bool possibleIntersect = true;
    float m, diffA, diffB;

    // First check if P is close to vertical
    if( std::abs(PB[1] - PA[1]) < 0.000001 ) {
        // P is close to vertical
        if( (QA[1] >= PA[1] && QB[1] >= PA[1])
                || (QA[1] <= PA[1] && QB[1] <= PA[1]) ) {
            // Q is on one side of P
            possibleIntersect = false;
        }
    } else {
        // P is not close to vertical
        m = (PB[2] - PA[2]) / (PB[1] - PA[1]);

        // Equation for points on P: y = m(x - PA[1]) + PA[2]
        diffA = (m*(QA[1]-PA[1]) + PA[2]) - QA[2];
        diffB = (m*(QB[1]-PA[1]) + PA[2]) - QB[2];
        if( (diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0) ) {
            // Q is either fully above or below the line containing P
            possibleIntersect = false;
        }
    }

    if( possibleIntersect ) {
        // first check if Q is close to vertical
        if( std::abs(QB[1] - QA[1]) < 0.000001 ) {
            if( (PA[1] >= QA[1] && PB[1] >= QA[1]) || (PA[1] <= QA[1] && PB[1] <= QA[1]) ) {
                // P is on one side of Q
                possibleIntersect = false;
            }
        } else {
            // Q is not close to vertical
            m = (QB[2] - QA[2]) / (QB[1] - QA[1]);

            // Equation for points on Q: y = m(x-QA[1]) + QA[2]
            diffA = (m*(PA[1]-QA[1]) + QA[2]) - PA[2];
            diffB = (m*(PB[1]-QA[1]) + QA[2]) - PB[2];
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
    std::vector<float> distances = { distanceSqrdPointToSegment(PA,QA,QB),
                                     distanceSqrdPointToSegment(PB,QA,QB),
                                     distanceSqrdPointToSegment(QA,PA,PB),
                                     distanceSqrdPointToSegment(QB,PA,PB) };

    return *std::min_element( distances.begin(), distances.end() );
}

int findIndexBeforeTime( std::vector<std::vector<float>> path, float timeToFind )
{
    if( path.size() < 1) {
        return -1;
    }

    int i = 0;
    while( i+1 < path.size() && path[i+1][3] < timeToFind ) {
        i += 1;
    }
    return i;
}


/////////////////////// Collision Checking Functions ///////////////////////


/////////////////////// RRT Functions ///////////////////////
bool extendRRT( CSpace* S, KDTree* Tree, KDTreeNode* newNode,
                KDTreeNode* closestNode, float delta,
                float hyperBallRad, KDTreeNode* moveGoal )
{
    // First calculate the shortest trajectory (and its distance) that
    // gets from newNode to closestNode while obeying the constraints
    // of the state space and the dynamics of the robot
    Edge* thisEdge = new Edge( newNode, closestNode );
}
