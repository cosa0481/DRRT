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
#include <math.h>
#include <fstream>

///////////////////// RRT Problem ///////////////////////
// This holds all the parameters needed to run RRTx with the DRRT library
class Problem{
public:
    std::string search_type;            // algorithm search type ("RRTx")
    std::shared_ptr<CSpace> c_space;    // problem configuration space
    double planning_only_time;          // plan before driving for this long
    double slice_time;                  // plan time per iteration
    double delta;                       // min distance of kd-tree nodes
    double ball_constant;               // nearest-neighbor search radius
    double change_threshold;            // obstacle change detection
    double goal_threshold;              // goal detection
    bool move_robot_flag;               // true -> drive robot
    Eigen::VectorXi wraps;              // wrapping dimensions (0=1st)
    Eigen::VectorXd wrap_points;        // points at which they wrap

    // Constructor
    Problem(std::string se_t,
            std::shared_ptr<CSpace> c_s,
            double p_o_t,
            double sl_t,
            double d,
            double b_c,
            double c_t,
            double g_t,
            bool m_r_f,
            Eigen::VectorXi w,
            Eigen::VectorXd w_p)
        :   search_type(se_t),
            c_space(c_s),
            planning_only_time(p_o_t),
            slice_time(sl_t),
            delta(d),
            ball_constant(b_c),
            change_threshold(c_t),
            goal_threshold(g_t),
            move_robot_flag(m_r_f),
            wraps(w),
            wrap_points(w_p)
    {}
};


///////////////////// Helper Functions ///////////////////////
// Returns current time in nanoseconds
double getTimeNs( std::chrono::time_point<std::chrono::high_resolution_clock> start );

// Returns a random double between min and max
double randDouble( double min, double max );

/////////////////////// Node Functions ///////////////////////
// Functions that interact with nodes and not much else.
// This includes a handfull of functions that traverse the search
// tree in one way or another

// Returns Distance between two nodes (e.g., in the C-space)
// Should obey the triangle inequality
//double distance(std::shared_ptr<KDTreeNode> x, std::shared_ptr<KDTreeNode> y);

/* Returns distance between two nodes (e.g., in the workspace)
 * Should obey the triangle inequality.
 * Note that it is assumed that the first two dimensions of
 * the C-space are position in the workspace
 */
//double Wdist( std::shared_ptr<KDTreeNode> x, std::shared_ptr<KDTreeNode> y );

// Returns true if nodeA certifies nodeB (only used in Alg. A)
//bool certifierOf( std::shared_ptr<KDTreeNode> nodeA, std::shared_ptr<KDTreeNode> nodeB );

// Extracts the cost of the graph path from th node to the root
// by adding up the edge lengths between node and root
double extractPathLength(std::shared_ptr<KDTreeNode> node,
                         std::shared_ptr<KDTreeNode> root);


/////////////////////// Obstacle Functions ///////////////////////
// This does NOT include collision checking, which appears lower
// in its own section. Functions involving obstacles that require
// C-space access also appear lower down in the C-Space or RRTx section

// Decreases the life of the obstacle
//void decreaseLife( Obstacle* O );


/////////////////////// C-Space Functions ///////////////////////
// Functions that interact in C-Space, including sampling functions

// Returns a random point in S
Eigen::VectorXd randPointDefault(std::shared_ptr<CSpace> S);

// Returns a random node from S
std::shared_ptr<KDTreeNode> randNodeDefault(std::shared_ptr<CSpace> S);

// Returns a random node from S, or the goal with probability pGoal
std::shared_ptr<KDTreeNode> randNodeOrGoal(std::shared_ptr<CSpace> S);

// Returns a random node from S, but when itsSamplePoint == 0
// it returns itsSamplePoint instead
std::shared_ptr<KDTreeNode> randNodeIts(std::shared_ptr<CSpace> S);

// Returns a random node from S, but when waitTime has passed it returns
// timeSamplePoint instead
std::shared_ptr<KDTreeNode> randNodeTime(std::shared_ptr<CSpace> S);

// Returns a random node from S, but when waitTime has passed it returns
// timeSamplePoint instead, also sets the first obstacle to unused
std::shared_ptr<KDTreeNode> randNodeTimeWithObstacleRemove(
        std::shared_ptr<CSpace> S);

// Returns a random node from S, but when waitTime has passed it returns
// timeSamplePoint instead, also sets the first obstacle to unused
std::shared_ptr<KDTreeNode> randNodeItsWithObstacleRemove(
        std::shared_ptr<CSpace> S);

// Returns a random node unless there are points in the sample stack,
// in which case it returns the first one of those
std::shared_ptr<KDTreeNode> randNodeOrFromStack(std::shared_ptr<CSpace> &S);

/* This returns a random node where the time dimension is drawn uniformly
 * at random from (the min time the robot could reach the point in an
 * obstacle-less environment traveling at max speed) and (current move time)
 * unless there are points in the sample stack, in which case it returns
 * the first one of those
 */
std::shared_ptr<KDTreeNode> randNodeInTimeOrFromStack(
        std::shared_ptr<CSpace> S);

// Returns random point from within the obstacle to the CSpace
// sampleStack (resides after QuickCheck2D in drrt.cpp so this can use it)
void RandomSampleObs(std::shared_ptr<CSpace> &S,
                     std::shared_ptr<KDTree> Tree,
                     std::shared_ptr<Obstacle> &O);


/////////////////////// Geometric Functions ///////////////////////
// Slightly more complex geometric functions

// Returns the min distance squared between the point and the segment
// [startPoint, endPoint] assumes a 2D space
double DistanceSqrdPointToSegment(Eigen::VectorXd point,
                                  Eigen::Vector2d startPoint,
                                  Eigen::Vector2d endPoint);

// This returns the distance of the closest point on the boundary
// of the polygon to the point (assumes 2D space)
double DistToPolygonSqrd(Eigen::VectorXd point, Eigen::MatrixX2d polygon);

// All intput args represent points, this returns the minimum distance
// between line segments [PA PB] and [QA QB] and assumes 2D space
double SegmentDistSqrd(Eigen::VectorXd PA, Eigen::VectorXd PB,
                       Eigen::VectorXd QA, Eigen::VectorXd QB);

// Finds the transform of polygon to the approperiate position at
// the time of the point, based on the obstacle's path through time
// (Assumes obstacle type 6)
Eigen::Vector2d FindTransformObjToTimeOfPoint(std::shared_ptr<Obstacle> O,
                                              Eigen::Vector3d point);

// Returns true if the point is in the polygon (open set of it anyway)
// each row in polygon is a vertex and subsequent vertices define edges
// Top and bottom rows of polygon also form an edge
// Polygon does not have to be convex but should be simple
bool PointInPolygon(Eigen::VectorXd this_point, Eigen::MatrixX2d polygon);

// Returns the index of the first time coordinate (3rd dimension) smaller
// than the time
// TODO: replace with binary search
int FindIndexBeforeTime(Eigen::MatrixXd path, double timeToFind);


/////////////////////// Collision Checking Functions ///////////////////////
// Collision checking, etc. This includus certificate stuff that is currently
// unused, but could be added later with little difficulty

// This returns a -rangeList- (see KDTree code) containing all points
// that are in conflict with the obstacle. Note that rangeList must
// be DESTROYED PROPERLY using L.emptyRangeList to avoid problems -collision-
std::shared_ptr<JList> FindPointsInConflictWithObstacle(
        std::shared_ptr<CSpace> &S,
        std::shared_ptr<KDTree> Tree,
        std::shared_ptr<Obstacle> &O,
        std::shared_ptr<KDTreeNode> &root);

// This adds the obstacle (checks for edge conflicts with the obstactle
// and then puts the affected nodes into the appropriate heaps -collision-
void AddNewObstacle(std::shared_ptr<KDTree> Tree,
                    std::shared_ptr<Queue>& Q,
                    std::shared_ptr<Obstacle>& O,
                    std::shared_ptr<KDTreeNode> root,
                    std::shared_ptr<RobotData>& R);

// This removes the obstacle (checks for edge conflicts with the obstacle
// and then puts the affected nodes into the appropriate heaps)
void RemoveObstacle(std::shared_ptr<KDTree> Tree,
                    std::shared_ptr<Queue>& Q,
                    std::shared_ptr<Obstacle>& O,
                    std::shared_ptr<KDTreeNode> root,
                    double hyper_ball_rad, double time_elapsed,
                    std::shared_ptr<KDTreeNode>& move_goal );

// Checks all nodes in the heap to see if there are edge problems -collision-
// Returns true if there are edge problems
bool checkHeapForEdgeProblems(std::shared_ptr<Queue> &Q,
                              std::shared_ptr<KDTree> Tree);

// Makes sure that node can, in fact, reach all neighbors
// Returns true if there is an edge problem
// (used for error checking) -collision-
bool checkNeighborsForEdgeProblems(std::shared_ptr<CSpace>& S,
                                   std::shared_ptr<KDTreeNode> thisNode,
                                   std::shared_ptr<KDTree> Tree);

// Checks if the edge between the points is in collision with the obstacle
// (the point is closer than robot radius to the edge)
bool ExplicitEdgeCheck2D(std::shared_ptr<Obstacle> &O,
                         Eigen::VectorXd start_point,
                         Eigen::VectorXd end_point,
                         double radius);

// Checks if the edge is in collision with any obstacles in the C-space
// Returns true if there the edge is in collision
bool ExplicitEdgeCheck(std::shared_ptr<CSpace> &S,
                       std::shared_ptr<Edge> &edge);

bool QuickCheck2D(std::shared_ptr<Obstacle> &O, Eigen::Vector2d point,
                  std::shared_ptr<CSpace> &C);

bool QuickCheck(std::shared_ptr<CSpace> &C, Eigen::Vector2d point);


bool ExplicitPointCheck2D(std::shared_ptr<CSpace> &C,
                          std::shared_ptr<Obstacle> &O,
                          Eigen::VectorXd point,
                          double radius);

bool ExplicitPointCheck(std::shared_ptr<Queue>& Q, Eigen::VectorXd point);

bool ExplicitNodeCheck(std::shared_ptr<Queue>& Q,
                       std::shared_ptr<KDTreeNode> node);

/////////////////////// RRT Functions ///////////////////////
// Functions used for RRT. Some of these are also used in RRT*
// RRT#, and RRTx

// Takes care of inserting a new node in RRT
// Returns true if successful
bool Extend(std::shared_ptr<KDTree> &Tree,
            std::shared_ptr<Queue> &Q,
            std::shared_ptr<KDTreeNode> &newNode,
            std::shared_ptr<KDTreeNode> &closest_node,
            double delta,
            double hyper_ball_rad,
            std::shared_ptr<KDTreeNode> &move_goal);


/////////////////////// RRT* Functions ///////////////////////
// Functions used for RRT*. Some of these are also used in RRT# and RRTx

/* This looks through the list to find the best parent for newNode
 * If the list is empty then it uses closestNode instead. If a parent
 * is found then newNode is linked to its parent. If saveAllEdges is true
 * then it saves a copy of each edge in each node (to avoid work duplication
 * in RRT# and RRTx)
 */
/// THIS FUNCTION INITIALLY SETS rrtLMC COST ///
void findBestParent(std::shared_ptr<CSpace> &S,
                    std::shared_ptr<KDTree> &Tree,
                    std::shared_ptr<KDTreeNode> &newNode,
                    std::shared_ptr<JList> &nodeList,
                    std::shared_ptr<KDTreeNode> &closestNode,
                    bool saveAllEdges );


/////////////////////// RRT# Functions ///////////////////////
// Functions used for RRT#. Some of these are also used in RRTx

// Resets the neighbor iterator
void resetNeighborIterator(std::shared_ptr<RRTNodeNeighborIterator> &It);

// Links an edge -from- node -to- newNeighbor
// Edge should already be populated correctly.
void makeNeighborOf(std::shared_ptr<KDTreeNode> &newNeighbor,
                    std::shared_ptr<KDTreeNode> &node,
                    std::shared_ptr<Edge> &edge);

// Links an "initial" "out" edge -from- node -to newNeighbor
// Edge should already be populated correctly.
// This is actually only used for -RRTx- but is included here because
// of its similarity to the function above.
void makeInitialOutNeighborOf(std::shared_ptr<KDTreeNode> &newNeighbor,
                              std::shared_ptr<KDTreeNode> &node,
                              std::shared_ptr<Edge> &edge);

// Links an "initial" "in" edge -from- node -to- newNeighbor
// (i.e. the edge is only stored on the recieving node and not on the sending node)
// Edge should already be populated correctly.
// This is actually only used for -RRTx- but is included here because
// of its similarity to the functions above.
void makeInitialInNeighborOf(std::shared_ptr<KDTreeNode> &newNeighbor,
                             std::shared_ptr<KDTreeNode> &node,
                             std::shared_ptr<Edge> &edge);

// Updates the priority queue (adds node if necessary, does not if not)
// Returns true if node is added
void updateQueue(std::shared_ptr<Queue> &Q,
                 std::shared_ptr<KDTreeNode> &newNode,
                 std::shared_ptr<KDTreeNode> &root,
                 double hyperBallRad);

// Propogates cost information through the graph
void reduceInconsistency(std::shared_ptr<Queue> &Q,
                         std::shared_ptr<KDTreeNode> &goalNode,
                         double robotRad,
                         std::shared_ptr<KDTreeNode> &root,
                         double hyperBallRad);


/////////////////////// RRTx Functions ///////////////////////
// Functions used for RRTx

// Successor stack marker function (marks when a node is in the successor stack OS)
void markOS(std::shared_ptr<KDTreeNode> &node);

// Successor stack unmarker function (unmarks when a node is removed from OS)
void unmarkOS(std::shared_ptr<KDTreeNode> &node);

// Successor stack queue check marker function (checks if the node is marked OS)
bool markedOS(std::shared_ptr<KDTreeNode> node);

// Makes sure the node is in the priority queue
bool verifyInQueue(std::shared_ptr<Queue> &Q,
                   std::shared_ptr<KDTreeNode> &node);

// Makes sure the node is in the OS queue
// Removes it from the normal queue if necessary
bool verifyInOSQueue(std::shared_ptr<Queue> &Q,
                     std::shared_ptr<KDTreeNode> &node);

// Removes members of the current neighbor list of node that are too far away
void cullCurrentNeighbors(std::shared_ptr<KDTreeNode> &node,
                          double hyperBallRad);

// RRTx based version
// Returns the JListNode containing the next outgoing neighbor edge of the
// node for which this iterator was created
std::shared_ptr<JListNode> nextOutNeighbor(
        std::shared_ptr<RRTNodeNeighborIterator> &It);

// RRTx based version
// Returns the JListNode containing the next outgoing neighbor edge of the
// node for which this iterator was created
std::shared_ptr<JListNode> nextInNeighbor(
        std::shared_ptr<RRTNodeNeighborIterator> &It);

// Makes newParent the parent of node via the edge
void makeParentOf(std::shared_ptr<KDTreeNode> &newParent,
                  std::shared_ptr<KDTreeNode> &node,
                  std::shared_ptr<Edge> &edge,
                  std::shared_ptr<KDTreeNode> &root);

// Recalculates LMC based on neighbors
// Returns true if successful
bool recalculateLMC(std::shared_ptr<Queue> &Q,
                            std::shared_ptr<KDTreeNode> &node,
                            std::shared_ptr<KDTreeNode> &root,
                            double hyperBallRad);

// This is the (non-initial) rewire function used by RRTx that is
// responsible for propogating changes through the graph
// Returns true if successful
bool rewire( std::shared_ptr<Queue> &Q,
             std::shared_ptr<KDTreeNode> &node,
             std::shared_ptr<KDTreeNode> &root,
             double hyperBallRad,
             double changeThresh);

// Propogates orphan status to all nodes in the basin(s) of attraction
// of the nodes in Q.OS stack (that have higher cost). This also takes
// the robot to remember if node the robot was moving at is one of the
// nodes that has become an orphan. Returns true if successful.
bool propogateDescendants(std::shared_ptr<Queue> &Q,
                          std::shared_ptr<KDTree> Tree,
                          std::shared_ptr<RobotData> &R);

/* If C-Space has a time dimension, add a sequence of descendents
 * to the root, where each great^n-grandchild is at the same
 * position as a root, but at a sequence of times from 0 to the
 * last ("earliest") time that the robot could arrive at that
 * position (assuming no obstacles) but at a tree distance defined
 * to be 0 from the root.
 * This helps the robot to reach the goal location as quickly as
 * possible instead of burning time
 */
void addOtherTimesToRoot( std::shared_ptr<CSpace> &S,
                          std::shared_ptr<KDTree> &Tree,
                          std::shared_ptr<KDTreeNode> &goal,
                          std::shared_ptr<KDTreeNode> &root,
                          std::string searchType);

// Attempts to find a new move target for the robot, places
// it into RobotData (used when the old target has become invalid)
void findNewTarget(std::shared_ptr<CSpace> &S,
                    std::shared_ptr<KDTree> &Tree,
                    std::shared_ptr<RobotData> &R,
                    double hyperBallRad);

/* Move robot the distance that it would move in slice_time time
 * if time is not a dimension of the C-Space, then a contant velocity
 * is assumed. This also updates the moveGoal in the event that the robot
 * has lost connectivity with the graph due to dynamic obstacles breaking
 * the first edge of its path
 */
void MoveRobot(std::shared_ptr<Queue> &Q,
               std::shared_ptr<KDTree> &Tree,
               std::shared_ptr<KDTreeNode> &root,
               double slice_time,
               double hyperBallRad,
               std::shared_ptr<RobotData> &R);

#endif // DRRT_H
