#ifndef DRRT_H
#define DRRT_H

#include <DRRT/kdtree.h>

/////////////////////// Node Functions ///////////////////////
// Functions that interact with nodes and not much else.
// This includes a handfull of functions that traverse the search
// tree in one way or another

// Returns Distance between two nodes (e.g., in the C-space)
// Should obey the triangle inequality
float distance( KDTreeNode* x, KDTreeNode* y );
float distance( Edge* e1, Edge* e2 );

/* Returns distance between two nodes (e.g., in the workspace)
 * Should obey the triangle inequality.
 * Note that it is assumed that the first two dimensions of
 * the C-space are position in the workspace
 */
float Wdist( KDTreeNode* x, KDTreeNode* y );
float Wdist( Edge* e1, Edge* e2 );

// Returns true if nodeA certifies nodeB (only used in Alg. A)
bool certifierOf( KDTreeNode* nodeA, KDTreeNode* nodeB );
bool certifierOf( Edge* eA, Edge* eB );

// Extracts the cost of the graph path from th node to the root
// by adding up the edge lengths between node and root
float extractPathLength( KDTreeNode* node, KDTreeNode* root );
float extractPathLength( Edge* e, Edge* root );

/////////////////////// Obstacle Functions ///////////////////////
// This does NOT include collision checking, which appears lower
// in its own section. Functions involving obstacles that require
// C-space access also appear lower down in the C-Space or RRTx section

// Decreases the life of the obstacle
//void decreaseLife( Obstacle* O );

/////////////////////// C-Space Functions ///////////////////////
// Functions that interact in C-Space, including sampling functions

// Returns a random point in S
float randPointDefault( CSpace<float> S );

// Returns a random node from S
KDTreeNode* randNodeDefault( CSpace<float> S );

// Returns a random node from S, or the goal with probability pGoal
// *** Not sure what CSpace type this should be
KDTreeNode* randNodeOrGoal( CSpace<float> S );

// Returns a random node from S, but when itsSamplePoint == 0
// it returns itsSamplePoint instead
KDTreeNode* randNodeIts( CSpace<KDTreeNode> S );

// Returns a random node from S, but when waitTime has passed it returns
// timeSamplePoint instead
KDTreeNode* randNodeTime( CSpace<KDTreeNode> S );

// Returns a random node from S, but when waitTime has passed it returns
// timeSamplePoint instead, also sets the first obstacle to unused
KDTreeNode* randNodeTimeWithObstacleRemove( CSpace<KDTreeNode> S );

// Returns a random node from S, but when waitTime has passed it returns
// timeSamplePoint instead, also sets the first obstacle to unused
KDTreeNode* randNodeItsWithObstacleRemove( CSpace<KDTreeNode> S );

// Returns a random node unless there are points in the sample stack,
// in which case it returns the first one of those
KDTreeNode* randNodeOrFromStack( CSpace<KDTreeNode> S );

/* This returns a random node where the time dimension is drawn uniformly
 * at random from (the min time the robot could reach the point in an
 * obstacle-less environment traveling at max speed) and (current move time)
 * unless there are points in the sample stack, in which case it returns
 * the first one of those
 */
KDTreeNode* randNodeInTimeOrFromStack( CSpace<KDTreeNode> S );

// Returns random point from within the obstacle
//KDTreeNode* randomSampleObs( CSpace S, KDTree* KD, Obstacle* O );

// Adds obstacle to the C-Space
//void addObsToCSpace( CSpace C, Obstacle* O );

// Makes sure that node can, in fact, reach all neighbors
// USED FOR ERROR CHECKING
//void checkNeighborsForEdgeProblems( )

/////////////////////// Geometric Functions ///////////////////////
// Slightly more complex geometric functions

// Returns the min distance squared between the point and the segment
// [startPoint, endPoint] assumes a 2D space
float distanceSqrdPointToSegment( std::vector<float> point, std::vector<float> startPoint, std::vector<float> endPoint );

// All intput args represent points, this returns the minimum distance
// between line segments [PA PB] and [QA QB] and assumes 2D space
float segmentDistSqrd( std::vector<float> PA, std::vector<float> PB,
                       std::vector<float> QA, std::vector<float> QB );

// Returns the index of the first time coordinate (3rd dimension) smaller
// than the time
// TODO: replace with binary search
int findIndexBeforeTime( std::vector<float> path, float timeToFind );

/////////////////////// Collision Checking Functions ///////////////////////
// Collision checking, etc. This includus certificate stuff that is currently
// unused, but could be added later with little difficulty

/////////////////////// RRT Functions ///////////////////////
// Functions used for RRT. Some of these are also used in RRT*
// RRT#, and RRTx

// Takes care of inserting a new node in RRT
bool extendRRT( CSpace<KDTreeNode> S, KDTree* Tree, KDTreeNode* newNode, KDTreeNode* closestNode, float delta, float hyperBallRad, KDTreeNode* moveGoal );

/////////////////////// RRT* Functions ///////////////////////
// Functions used for RRT*. Some of these are also used in RRT# and RRTx

/* This looks through the list to find the best parent for newNode
 * If the list is empty then it uses closestNode instead. If a parent
 * is found then newNode is linked to its parent. If saveAllEdges is true
 * then it saves a copy of each edge in each node (to avoid work duplication
 * in RRT# and RRTx)
 */
KDTreeNode* findBestParent( CSpace<KDTreeNode> S, KDTreeNode* newNode, JList nodeList, KDTreeNode* closestNode, bool saveAllEdges );

// Takes care of inserting a new node in RRT*
bool extendRRTStar( CSpace<KDTreeNode> S, KDTree* Tree, KDTreeNode* newNode, KDTreeNode* closestNode, float delta, float hyperBallRad, KDTreeNode* moveGoal );

/////////////////////// RRT# Functions ///////////////////////
// Functions used for RRT#. Some of these are also used in RRTx
// THis includes priority heap related key functions, etc.

// Returns the key value of a node
float keyQ( KDTreeNode* node );

// Less than function for key values
bool lessQ( KDTreeNode* a, KDTreeNode* b );

// Greater than function for key values
bool greaterQ( KDTreeNode* a, KDTreeNode* b );

// Priority queue marker function (marks when a node is in the queue)
void markQ( KDTreeNode* node );



#endif // DRRT_H
