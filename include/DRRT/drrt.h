/* drrt.h
 * Corin Sandford
 * Fall 2016
 * main function located here
 */

#ifndef DRRT_H
#define DRRT_H

#include <DRRT/kdtree.h> // Pretty much everything is included here
#include <random>
#include <stdlib.h>
#include <algorithm>
#include <time.h>
#include <chrono>

// Outputs error message
void error( std::string e );
void error( double d );
void error( int i );

// Returns current time in nanoseconds
double getTimeNs( std::chrono::time_point<std::chrono::high_resolution_clock> start );

// Returns a random integer between min and max
int randInt( int min, int max );

/////////////////////// Node Functions ///////////////////////
// Functions that interact with nodes and not much else.
// This includes a handfull of functions that traverse the search
// tree in one way or another

// Returns Distance between two nodes (e.g., in the C-space)
// Should obey the triangle inequality
double distance( KDTreeNode* x, KDTreeNode* y );

/* Returns distance between two nodes (e.g., in the workspace)
 * Should obey the triangle inequality.
 * Note that it is assumed that the first two dimensions of
 * the C-space are position in the workspace
 */
double Wdist( KDTreeNode* x, KDTreeNode* y );

// Returns true if nodeA certifies nodeB (only used in Alg. A)
//bool certifierOf( KDTreeNode* nodeA, KDTreeNode* nodeB );

// Extracts the cost of the graph path from th node to the root
// by adding up the edge lengths between node and root
double extractPathLength( KDTreeNode* node, KDTreeNode* root );


/////////////////////// Obstacle Functions ///////////////////////
// This does NOT include collision checking, which appears lower
// in its own section. Functions involving obstacles that require
// C-space access also appear lower down in the C-Space or RRTx section

// Decreases the life of the obstacle
//void decreaseLife( Obstacle* O );


/////////////////////// C-Space Functions ///////////////////////
// Functions that interact in C-Space, including sampling functions

// Returns a random point in S
Eigen::VectorXd randPointDefault( CSpace* S );

// Returns a random node from S
KDTreeNode* randNodeDefault( CSpace* S );

// Returns a random node from S, or the goal with probability pGoal
KDTreeNode* randNodeOrGoal( CSpace* S );

// Returns a random node from S, but when itsSamplePoint == 0
// it returns itsSamplePoint instead
KDTreeNode* randNodeIts( CSpace* S );

// Returns a random node from S, but when waitTime has passed it returns
// timeSamplePoint instead
KDTreeNode* randNodeTime( CSpace* S );

// Returns a random node from S, but when waitTime has passed it returns
// timeSamplePoint instead, also sets the first obstacle to unused
KDTreeNode* randNodeTimeWithObstacleRemove( CSpace* S );

// Returns a random node from S, but when waitTime has passed it returns
// timeSamplePoint instead, also sets the first obstacle to unused
KDTreeNode* randNodeItsWithObstacleRemove( CSpace* S );

// Returns a random node unless there are points in the sample stack,
// in which case it returns the first one of those
KDTreeNode* randNodeOrFromStack( CSpace* S );

/* This returns a random node where the time dimension is drawn uniformly
 * at random from (the min time the robot could reach the point in an
 * obstacle-less environment traveling at max speed) and (current move time)
 * unless there are points in the sample stack, in which case it returns
 * the first one of those
 */
KDTreeNode* randNodeInTimeOrFromStack( CSpace* S );

// Returns random point from within the obstacle -collision-
//KDTreeNode* randomSampleObs( CSpace S, KDTree* KD, Obstacle* O );

// Adds obstacle to the C-Space -collision-
//void addObsToCSpace( CSpace C, Obstacle* O );

// Makes sure that node can, in fact, reach all neighbors
// Returns true if there is an edge problem
// (used for error checking) -collision-
// MISSING IMPLEMENTATION OF EXPLICITEDGECHECK(CSpace,KDTreeNode,KDTreeNode)
bool checkNeighborsForEdgeProblems( CSpace* S, KDTreeNode* thisNode );


/////////////////////// Geometric Functions ///////////////////////
// Slightly more complex geometric functions

// Returns the min distance squared between the point and the segment
// [startPoint, endPoint] assumes a 2D space
double distanceSqrdPointToSegment( Eigen::VectorXd point,
                                   Eigen::VectorXd startPoint,
                                   Eigen::VectorXd endPoint );

// All intput args represent points, this returns the minimum distance
// between line segments [PA PB] and [QA QB] and assumes 2D space
double segmentDistSqrd( Eigen::VectorXd PA, Eigen::VectorXd PB,
                        Eigen::VectorXd QA, Eigen::VectorXd QB );

// Returns the index of the first time coordinate (3rd dimension) smaller
// than the time
// TODO: replace with binary search
int findIndexBeforeTime( Eigen::MatrixXd path, double timeToFind );


/////////////////////// Collision Checking Functions ///////////////////////
// Collision checking, etc. This includus certificate stuff that is currently
// unused, but could be added later with little difficulty

// Checks if the edge is in collision with any obstacles in the C-space
// Returns true if there the edge is in collision
// MISSING IMPLEMENTATION OF EXPLICITEDGECHECK(CSpace,KDTreeNode,KDTreeNode)
bool explicitEdgeCheck( CSpace* S, Edge* edge );


/////////////////////// RRT Functions ///////////////////////
// Functions used for RRT. Some of these are also used in RRT*
// RRT#, and RRTx

// Takes care of inserting a new node in RRT
// Returns true if successful
bool extend( CSpace* S, KDTree* Tree, Queue* Q, KDTreeNode* newNode,
             KDTreeNode* closestNode, double delta,
             double hyperBallRad, KDTreeNode* moveGoal );


/////////////////////// RRT* Functions ///////////////////////
// Functions used for RRT*. Some of these are also used in RRT# and RRTx

/* This looks through the list to find the best parent for newNode
 * If the list is empty then it uses closestNode instead. If a parent
 * is found then newNode is linked to its parent. If saveAllEdges is true
 * then it saves a copy of each edge in each node (to avoid work duplication
 * in RRT# and RRTx)
 */
void findBestParent( CSpace* S, KDTreeNode* newNode, JList* nodeList,
                     KDTreeNode* closestNode, bool saveAllEdges );

// Takes care of inserting a new node in RRT*
// Uses above implementation of extend with Q = rrtStarQueue


/////////////////////// RRT# Functions ///////////////////////
// Functions used for RRT#. Some of these are also used in RRTx
// This includes priority heap related key functions **(these are in heap.h)** etc.

// Checks all nodes in the heap to see if there are edge problems -collision-
// Returns true if there are edge problems
bool checkHeapForEdgeProblems( Queue* Q );

// Resets the neighbor iterator
void resetNeighborIterator( RRTNodeNeighborIterator* It );

// Returns the JListNode containing the next neighbor of the node
// for which this iterator was created
JListNode* nextOutNeighbor( RRTNodeNeighborIterator* It, Queue* Q );

// Returns the JListNode containing the next neighbor of the node
// for which this iterator was created
JListNode* nextInNeighbor( RRTNodeNeighborIterator* It, Queue* Q );

// Links an edge -from- node -to- newNeighbor
// Edge should already be populated correctly.
void makeNeighborOf( KDTreeNode* newNeighbor, KDTreeNode* node, Edge* edge );

// Links an "initial" "out" edge -from- node -to newNeighbor
// Edge should already be populated correctly.
// This is actually only used for -RRTx- but is included here because
// of its similarity to the function above.
void makeInitialOutNeighborOf( KDTreeNode* newNeighbor, KDTreeNode* node, Edge* edge );

// Links an "initial" "in" edge -from- node -to- newNeighbor
// (i.e. the edge is only stored on the recieving node and not on the sending node)
// Edge should already be populated correctly.
// This is actually only used for -RRTx- but is included here because
// of its similarity to the functions above.
void makeInitialInNeighborOf( KDTreeNode* newNeighbor, KDTreeNode* node, Edge* edge );

// Recalculates LMC based on neighbors that this node can reach
// Note the first argument in unused but necessary for the multiple
// dispatch that is used to differentiate between RRT* and RRT#
// Returns true if successful
bool recalculateLMC( Queue* Q, KDTreeNode* node, KDTreeNode* root );

// Updates the priority queue (adds node if necessary, does not if not)
// Returns true if node is added
void updateQueue( Queue* Q, KDTreeNode* newNode,
                  KDTreeNode* root, double hyperBallRad );

// Takes care of inserting a new node in RRT#
// Uses above implementation of extend with Q = rrtSharpQueue

// Propogates cost information through the graph
void reduceInconsistency( Queue* Q, KDTreeNode* goalNode, double robotRad,
                          KDTreeNode* root, double hyperBallRad );


/////////////////////// RRTx Functions ///////////////////////
// Functions used for RRTx

// Successor stack marker function (marks when a node is in the successor stack OS)
void markOS( KDTreeNode* node );

// Successor stack unmarker function (unmarks when a node is removed from OS)
void unmarkOS( KDTreeNode* node );

// Successor stack queue check marker function (checks if the node is marked OS)
bool markedOS( KDTreeNode node );

// Makes sure the node is in the priority queue
bool verifyInQueue( Queue* Q, KDTreeNode* node );

// Makes sure the node is in the OS queue
// Removes it from the normal queue if necessary
bool verifyInOSQueue( Queue* Q, KDTreeNode* node );

// Removes members of the current neighbor list of node that are too far away
void cullCurrentNeighbors( KDTreeNode* node, double hyperBallRad );

// RRTx based version
// Returns the JListNode containing the next outgoing neighbor edge of the
// node for which this iterator was created
JListNode* nextOutNeighbor( RRTNodeNeighborIterator* It, Queue Q );

// RRTx based version
// Returns the JListNode containing the next outgoing neighbor edge of the
// node for which this iterator was created
JListNode* nextInNeighbor( RRTNodeNeighborIterator* It, Queue Q );

// Makes newParent the parent of node via the edge
void makeParentOf( KDTreeNode* newParent, KDTreeNode* node,
                   Edge* edge, KDTreeNode* root );

// Recalculates LMC based on neighbors
// Returns true if successful
bool recalculateLMCMineVTwo( Queue* Q, KDTreeNode* node,
                             KDTreeNode* root, double hyperBallRad );

// Takes care of inserting a new node
// Uses above implementation of extend with Q = rrtXQueue

// This is the (non-initial) rewire function used by RRTx that is
// responsible for propogating changes through the graph
// Returns true if successful
bool rewire( Queue* Q, KDTreeNode* node, KDTreeNode* root,
             double hyperBallRad, double changeThresh );

// Propogates cost information through the graph
// Uses above implementation with Q = rrtXQueue

// Propogates orphan status to all nodes in the basin(s) of attraction
// of the nodes in Q.OS stack (that have higher cost). This also takes
// the robot to remember if node the robot was moving at is one of the
// nodes that has become an orphan. Returns true if successful.
bool propogateDescendants( Queue* Q, RobotData* R );

/* If C-Space has a time dimension, add a sequence of descendents
 * to the root, where each great^n-grandchild is at the same
 * position as a root, but at a sequence of times from 0 to the
 * last ("earliest") time that the robot could arrive at that
 * position (assuming no obstacles) but at a tree distance defined
 * to be 0 from the root.
 * This helps the robot to reach the goal location as quickly as
 * possible instead of burning time
 */
void addOtherTimesToRoot( CSpace* S, KDTree* Tree,
                          KDTreeNode* goal, KDTreeNode* root,
                          std::string searchType );

// Attempts to find a new move target for the robot, places
// it into RobotData (used when the old target has become invalid)
void findNewTarget( CSpace* S, KDTree* Tree,
                    RobotData* R, double hyperBallRad );

/* Move robot the distance that it would move in slice_time time
 * if time is not a dimension of the C-Space, then a contant velocity
 * is assumed. This also updates the moveGoal in the event that the robot
 * has lost connectivity with the graph due to dynamic obstacles breaking
 * the first edge of its path
 */
void moveRobot( CSpace* S, Queue* Q, KDTree* Tree, double slice_time,
                KDTreeNode* root, double hyperBallRad, RobotData* R );

// This returns a -rangeList- (see KDTree code) containing all points
// that are in conflict with the obstacle. Note that rangeList must
// be DESTROYED PROPERLY using L.emptyRangeList to avoid problems -collision-
/*JList* findPointsInConflictWithObstacle( CSpace* S, KDTree* Tree,
                                         Obstacle* O, KDTreeNode* root );*/

// This adds the obstacle (checks for edge conflicts with the obstactle
// and then puts the affected nodes into the appropriate heaps -collision-
/*void addNewObstacle( CSpace* S, KDTree* Tree, Queue* Q, Obstacle* O,
                     KDTreeNode* root, int fileCounter, RobotData* R );*/

// This removes the obstacle (checks for edge conflicts with the obstacle
// and then puts the affected nodes into the appropriate heaps)
/*void removeObstacle( CSpace* S, KDTree* Tree, Queue* Q, Obstacle* O,
                     KDTreeNode* root, double hyperBallRad,
                     double timeElapsed, KDTreeNode* moveGoal );*/


/////////////////////// main (despite the name) ///////////////////////
// The following now calls RRT, RRT*, RRT#, and RRTx.
// Behaviour is determined by parameters passed in, note that RRT# vs
// (RRT* and RRT) helper functions above are called correctly using
// Julia's multiple dispatch, where the ype of the queue being used is
// different for each algorithm

// S is the CSpace, the algorithm runs until either N nodes have been
// sampled or TimeOut seconds pass, delta is the saturation distance,
// ballConstant is the ball constant
void RRTX( CSpace* S, double total_planning_time, double slice_time,
           double delta, double ballConstant, double changeThresh,
           std::string searchType, bool MoveRobotFlag,
           bool saveVideoData, bool saveTree, std::string dataFile
           /* "statsArgs..." ???*/ );

#endif // DRRT_H
