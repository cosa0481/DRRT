/* drrt.h
 * Corin Sandford
 * Fall 2016
 * main function located here
 */

#ifndef DRRT_H
#define DRRT_H

#include <DRRT/kdtree.h> // Pretty much everything is included at this point

///////////////////// RRT Problem ///////////////////////
// This holds all the parameters needed to run RRTx with the DRRT library
typedef struct Problem{
public:
    std::string search_type;            // algorithm search type ("RRTx")
    std::shared_ptr<ConfigSpace> c_space;    // problem configuration space
    double planning_only_time;          // plan before driving for this long
    double slice_time;                  // plan time per iteration
    double delta;                       // min distance of kd-tree nodes
    double ball_constant;               // nearest-neighbor search radius
    double change_threshold;            // obstacle change detection
    double goal_threshold;              // goal detection
    bool move_robot_flag;               // true -> drive robot
    Eigen::VectorXi wraps_;              // wrapping dimensions (0=1st)
    Eigen::VectorXd wrap_points;        // points at which they wrap

    // Constructor
    Problem(std::string se_t,
            std::shared_ptr<ConfigSpace> c_s,
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
            wraps_(w),
            wrap_points(w_p)
    {}
} Problem;


///////////////////// Helper Functions ///////////////////////
// Prints the RRTX optimal path from the given leaf node
void PrintRrtxPath(std::shared_ptr<KDTreeNode> &leaf);

// Returns current time in nanoseconds
double GetTimeNs( std::chrono::time_point<std::chrono::high_resolution_clock> start );

// Returns a random double between min and max
double RandDouble( double min, double max );

/////////////////////// Node Functions ///////////////////////
// Functions that interact with nodes and not much else.
// This includes a handfull of functions that traverse the search
// tree in one way or another

// Extracts the cost of the graph path from th node to the root
// by adding up the edge lengths between node and root
double ExtractPathLength(std::shared_ptr<KDTreeNode> node,
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
Eigen::VectorXd RandPointDefault(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S
std::shared_ptr<KDTreeNode> RandNodeDefault(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, or the goal with probability prob_goal_
std::shared_ptr<KDTreeNode> RandNodeOrGoal(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, but when iteration_sample_point_ == 0
// it returns iteration_sample_point_ instead
std::shared_ptr<KDTreeNode> RandNodeIts(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, but when wait_time_ has passed it returns
// time_sample_point_ instead
std::shared_ptr<KDTreeNode> RandNodeTime(std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, but when wait_time_ has passed it returns
// time_sample_point_ instead, also sets the first obstacle to unused
//std::shared_ptr<KDTreeNode> randNodeTimeWithObstacleRemove(
//        std::shared_ptr<ConfigSpace> C);

// Returns a random node from S, but when wait_time_ has passed it returns
// time_sample_point_ instead, also sets the first obstacle to unused
//std::shared_ptr<KDTreeNode> randNodeItsWithObstacleRemove(
//        std::shared_ptr<ConfigSpace> C);

// Returns a random node unless there are points in the sample stack,
// in which case it returns the first one of those
std::shared_ptr<KDTreeNode> RandNodeOrFromStack(std::shared_ptr<ConfigSpace> &C);

/* This returns a random node where the time dimension is drawn uniformly
 * at random from (the min time the robot could reach the point in an
 * obstacle-less environment traveling at max speed) and (current move time)
 * unless there are points in the sample stack, in which case it returns
 * the first one of those
 */
std::shared_ptr<KDTreeNode> RandNodeInTimeOrFromStack(
        std::shared_ptr<ConfigSpace> C);

/////////////////////// Collision Checking Functions ///////////////////////
// Collision checking, etc. This includus certificate stuff that is currently
// unused, but could be added later with little difficulty

// Checks all nodes in the heap to see if there are edge problems -collision-
// Returns true if there are edge problems
bool CheckHeapForEdgeProblems(std::shared_ptr<Queue> &Q,
                              std::shared_ptr<KDTree> Tree);

// Makes sure that node can, in fact, reach all neighbors
// Returns true if there is an edge problem
// (used for error checking) -collision-
bool CheckNeighborsForEdgeProblems(std::shared_ptr<ConfigSpace> &C,
                                   std::shared_ptr<KDTreeNode> thisNode,
                                   std::shared_ptr<KDTree> Tree);

// Returns random point from within the obstacle to the ConfigSpace
// sample_stack_
void RandomSampleObs(std::shared_ptr<ConfigSpace> &C,
                     std::shared_ptr<KDTree> Tree,
                     std::shared_ptr<Obstacle> &O);

// Finds the transform of polygon to the approperiate position at
// the time of the point, based on the obstacle's path through time
// (Assumes obstacle type 6)
Eigen::Vector2d FindTransformObjToTimeOfPoint(std::shared_ptr<Obstacle> O,
                                              Eigen::Vector3d point);

// Returns the index of the first time coordinate (3rd dimension) smaller
// than the time
// TODO: replace with binary search
int FindIndexBeforeTime(Eigen::MatrixXd path, double timeToFind);


/////////////////////// RRT Functions ///////////////////////
// Functions used for RRT. Some of these are also used in RRT*
// RRT#, and RRTx

// Takes care of inserting a new node in RRT
// Returns true if successful
bool Extend(std::shared_ptr<KDTree> &Tree,
            std::shared_ptr<Queue> &Q,
            std::shared_ptr<KDTreeNode> &new_node,
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
/// THIS FUNCTION INITIALLY SETS rrt_LMC_ COST ///
void FindBestParent(std::shared_ptr<ConfigSpace> &C,
                    std::shared_ptr<KDTree> &Tree,
                    std::shared_ptr<KDTreeNode> &new_node,
                    std::shared_ptr<JList> &node_list,
                    std::shared_ptr<KDTreeNode> &closest_node,
                    bool save_all_edges );


/////////////////////// RRT# Functions ///////////////////////
// Functions used for RRT#. Some of these are also used in RRTx

// Resets the neighbor iterator
void ResetNeighborIterator(std::shared_ptr<RrtNodeNeighborIterator> &It);

// Links an edge -from- node -to- newNeighbor
// Edge should already be populated correctly.
void MakeNeighborOf(std::shared_ptr<KDTreeNode> &new_neighbor,
                    std::shared_ptr<KDTreeNode> &node,
                    std::shared_ptr<Edge> &edge);

// Links an "initial" "out" edge -from- node -to newNeighbor
// Edge should already be populated correctly.
// This is actually only used for -RRTx- but is included here because
// of its similarity to the function above.
void MakeInitialOutNeighborOf(std::shared_ptr<KDTreeNode> &new_neighbor,
                              std::shared_ptr<KDTreeNode> &node,
                              std::shared_ptr<Edge> &edge);

// Links an "initial" "in" edge -from- node -to- newNeighbor
// (i.e. the edge is only stored on the recieving node and not on the sending node)
// Edge should already be populated correctly.
// This is actually only used for -RRTx- but is included here because
// of its similarity to the functions above.
void MakeInitialInNeighborOf(std::shared_ptr<KDTreeNode> &new_neighbor,
                             std::shared_ptr<KDTreeNode> &node,
                             std::shared_ptr<Edge> &edge);

// Updates the priority queue (adds node if necessary, does not if not)
// Returns true if node is added
void UpdateQueue(std::shared_ptr<Queue> &Q,
                 std::shared_ptr<KDTreeNode> &new_node,
                 std::shared_ptr<KDTreeNode> &root,
                 double hyper_ball_rad);

// Propogates cost information through the graph
void ReduceInconsistency(std::shared_ptr<Queue> &Q,
                         std::shared_ptr<KDTreeNode> &goal_node,
                         double robot_rad,
                         std::shared_ptr<KDTreeNode> &root,
                         double hyper_ball_rad);


/////////////////////// RRTx Functions ///////////////////////
// Functions used for RRTx

// Successor stack marker function (marks when a node is in the successor stack OS)
void MarkOS(std::shared_ptr<KDTreeNode> &node);

// Successor stack unmarker function (unmarks when a node is removed from OS)
void UnmarkOS(std::shared_ptr<KDTreeNode> &node);

// Successor stack queue check marker function (checks if the node is marked OS)
bool MarkedOS(std::shared_ptr<KDTreeNode> node);

// Makes sure the node is in the priority queue
bool VerifyInQueue(std::shared_ptr<Queue> &Q,
                   std::shared_ptr<KDTreeNode> &node);

// Makes sure the node is in the OS queue
// Removes it from the normal queue if necessary
bool VerifyInOSQueue(std::shared_ptr<Queue> &Q,
                     std::shared_ptr<KDTreeNode> &node);

// Removes members of the current neighbor list of node that are too far away
void CullCurrentNeighbors(std::shared_ptr<KDTreeNode> &node,
                          double hyper_ball_rad);

// RRTx based version
// Returns the JListNode containing the next outgoing neighbor edge of the
// node for which this iterator was created
std::shared_ptr<JListNode> NextOutNeighbor(
        std::shared_ptr<RrtNodeNeighborIterator> &It);

// RRTx based version
// Returns the JListNode containing the next outgoing neighbor edge of the
// node for which this iterator was created
std::shared_ptr<JListNode> NextInNeighbor(
        std::shared_ptr<RrtNodeNeighborIterator> &It);

// Makes newParent the parent of node via the edge
void MakeParentOf(std::shared_ptr<KDTreeNode> &new_parent,
                  std::shared_ptr<KDTreeNode> &node,
                  std::shared_ptr<Edge> &edge);

// Recalculates LMC based on neighbors
// Returns true if successful
bool RecalculateLMC(std::shared_ptr<Queue> &Q,
                    std::shared_ptr<KDTreeNode> &node,
                    std::shared_ptr<KDTreeNode> &root,
                    double hyper_ball_rad);

// This is the (non-initial) rewire function used by RRTx that is
// responsible for propogating changes through the graph
// Returns true if successful
bool Rewire(std::shared_ptr<Queue> &Q,
            std::shared_ptr<KDTreeNode> &node,
            std::shared_ptr<KDTreeNode> &root,
            double hyper_ball_rad,
            double change_thresh);

// Propogates orphan status to all nodes in the basin(s) of attraction
// of the nodes in Q.OS stack (that have higher cost). This also takes
// the robot to remember if node the robot was moving at is one of the
// nodes that has become an orphan. Returns true if successful.
bool PropogateDescendants(std::shared_ptr<Queue> &Q,
                          std::shared_ptr<KDTree> Tree,
                          std::shared_ptr<RobotData> &Robot);

/* If C-Space has a time dimension, add a sequence of descendents
 * to the root, where each great^n-grandchild is at the same
 * position as a root, but at a sequence of times from 0 to the
 * last ("earliest") time that the robot could arrive at that
 * position (assuming no obstacles) but at a tree distance defined
 * to be 0 from the root.
 * This helps the robot to reach the goal location as quickly as
 * possible instead of burning time
 */
void AddOtherTimesToRoot( std::shared_ptr<ConfigSpace> &C,
                          std::shared_ptr<KDTree> &Tree,
                          std::shared_ptr<KDTreeNode> &goal,
                          std::shared_ptr<KDTreeNode> &root,
                          std::string search_type);

// Attempts to find a new move target for the robot, places
// it into RobotData (used when the old target has become invalid)
void FindNewTarget(std::shared_ptr<ConfigSpace> &C,
                    std::shared_ptr<KDTree> &Tree,
                    std::shared_ptr<RobotData> &Robot,
                    double hyper_ball_rad);


#endif // DRRT_H
