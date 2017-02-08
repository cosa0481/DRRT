#ifndef DRRT_DATA_STRUCTURES_H
#define DRRT_DATA_STRUCTURES_H

#include <DRRT/list.h>
#include <DRRT/heap.h>
#include <DRRT/edge.h>
//#include <DRRT/dubinsedge.h>

class CSpace{
public:
    int d;                   // dimensions
    List obstacles;          // a list of obstacles
    float obsDelta;          // the granularity of obstacle checks on edges
    Eigen::VectorXd lowerBounds;   // 1xD vector containing the lower bounds
    Eigen::VectorXd upperBounds;   // 1xD vector containing the upper bounds
    Eigen::VectorXd width;   // 1xD vector containing upperBounds-lowerBounds
    Eigen::VectorXd start;   // 1xD vector containing start location
    Eigen::VectorXd goal;    // 1xD vector containing goal location

    /* Flags that indicate what type of search space we are using
     * (these are mostly here to reduce the amount of duplicate code
     * for similar spaces, although they should probably one day be
     * replaced with a different approach that takes advantage of
     * Julia's multiple dispatch and polymorphism
     */
    bool spaceHasTime;  // if true then the 3rd dimension of the space is time
    bool spaceHasTheta; // if true then the 4th dimension of the space
                        // is theta in particular a Dubin's system is used

    // Stuff for sampling functions
    float pGoal;  // the probabality that the goal is sampled
    std::string randNode; // the sampling function to use (takes a CSpace)

    std::shared_ptr<KDTreeNode> goalNode;  // the goal node
    std::shared_ptr<KDTreeNode> root;      // the root node
    std::shared_ptr<KDTreeNode> moveGoal;  // the current node goal for robot

    int itsUntilSample;    // a count down to sample a particular point
    Eigen::VectorXd itsSamplePoint;   // sample this when itsUntilSample == 0
    Eigen::VectorXd timeSamplePoint;  // sample this when waitTime has passed
    float waitTime;                   // time to wait in seconds
    u_int64_t startTimeNs;            // time this started
    float timeElapsed; // elapsed time since started ( where time spent saving
                       // experimental data has been removed )

    //Obstacle* obstacleToRemove;    // an obstacle to remove

    float robotRadius;       // robot radius
    float robotVelocity;     // robot velocity (used for Dubins w/o time)

    float dubinsMinVelocity; // min velocity of Dubin's car (for dubins + time)
    float dubinsMaxVelocity; // max velocity of Dubin's car (for dubins + time)

    // This jlist must "hold" MatrixXd's ?
    // So use the JListNode->node->position when using this stack
    std::shared_ptr<JList> sampleStack; // points to sample in the future

    float hypervolume;          // hypervolume of the space
    float delta;                // RRT parameter delta
    float minTurningRadius;     // min turning radius e.g. for Dubin's car

    float warmupTime;   // the amount of warm up time allowed (obstacles are
                        // ignored for warm up time)
    bool inWarmupTime;  // true if we are in the warm up time

    // Constructor
    CSpace(int D, /*float ObsDelta,*/
           Eigen::VectorXd lower, Eigen::VectorXd upper,
           Eigen::VectorXd startpoint, Eigen::VectorXd endpoint)
        : d(D), lowerBounds(lower), upperBounds(upper),
          start(startpoint), goal(endpoint)
    {
        hypervolume = 0.0; // flag indicating that this needs to be calculated
        inWarmupTime = false;
        warmupTime = 0.0; // default value for time for build
                          // graph with no obstacles
        Eigen::ArrayXd upper_array = upper;
        Eigen::ArrayXd lower_array = lower;
        width = upper_array - lower_array;
    }
};

typedef struct Queue{
    std::string type;
    std::shared_ptr<CSpace> S;
    BinaryHeap* Q;      // normal queue (sorted based on cost from goal)
    std::shared_ptr<JList> OS;          // obstacle successor stack
    float changeThresh; // threshold of local changes that we care about

} Queue;

// Queue data structure used for RRT, basically empty, used
// to bake coding easier
typedef struct rrtQueue : Queue{} rrtQueue;

// Queue data structure used for RRT*, basically empty, used
// to make coding easier
typedef struct rrtStarQueue : Queue{} rrtStarQueue;

// Queue data structure used for RRT#
typedef struct rrtSharpQueue : Queue{} rrtSharpQueue;

// Queue data structure used for RRTx
typedef struct rrtXQueue : Queue{} rrtXQueue;

// This is used to make iteration through a particular node's
// neighbor edges easier given that each node stores all of its
// neighbor edges in three different places
typedef struct RRTNodeNeighborIterator{
    std::shared_ptr<KDTreeNode> thisNode;   // the node who's neighbors
                            // we are iterating through

    int listFlag;           // flag with the following values:
                            //  0: uninitialized
                            //  1: successors
                            //  2: original neighbors
                            //  3: current neighbors

    std::shared_ptr<JListNode> listItem; // a pointer to the position in the
                            // current neighbor list we are
                            // iterating through

    // Constructor
    RRTNodeNeighborIterator( std::shared_ptr<KDTreeNode> node ):
        thisNode(node), listFlag(0)
    {}

} RRTNodeNeighborIterator;

/* This holds the stuff associated with the robot that is
 * necessary for movement. Although some of the fields are
 * primarily used for simulation of robot movement,
 * currentMoveInvalid is important for the algorithm in general
 */
typedef struct RobotData{
    Eigen::VectorXd robotPose;  // this is where the robot is
                                // (i.e. where it was at the end of
                                // the last control loop

    Eigen::VectorXd nextRobotPose;  // this is where the robot will
                                    // be at the end of the current
                                    // control loop

    std::shared_ptr<KDTreeNode> nextMoveTarget; // this is the node at the
                            // root-end of the edge containing nextRobotPose

    double distanceFromNextRobotPoseToNextMoveTarget; // this holds the
                        // distance from nextRobotPose to nextMoveTarget
                        // along the trajectory the robot
                        // will be following at that time

    bool moving;  // set to true when the robot starts moving

    bool currentMoveInvalid; // this gets set to true if nextMoveTarget
                             // has become invalid due to dynamic obstacles

    Eigen::MatrixXd robotMovePath; // this holds the path the robot has
                 // followed from the start of movement up through robotPose
    double numRobotMovePoints;  // the number of points in robotMovePath

    Eigen::MatrixXd robotLocalPath; // this holds the path between robotPose
                             // and nextRobotPose (not including the former)
    float numLocalMovePoints;   // the number of points in robotLocalPath

    std::shared_ptr<Edge> robotEdge; // this is the edge that contains the
                                     // trajectory that the
                                     // robot is currently following

    bool robotEdgeUsed; // true if robotEdge is populated;

    // Note that currently only one of the two following parameters
    // is used at a time.
    // Which one is used depends on if time is explicitely part of
    // the state space.
    float distAlongRobotEdge; // the current distance that the robot
                              // "will be" along robotEdge (next time slice)

    float timeAlongRobotEdge; // the current time that the robot "will be"
                              // along robotEdge (i.e. next time slice)

    // Constructor
    RobotData(Eigen::VectorXd rP, std::shared_ptr<KDTreeNode> nMT,
              int maxPathNodes) :
        robotPose(rP), nextRobotPose(rP), nextMoveTarget(nMT),
        distanceFromNextRobotPoseToNextMoveTarget(0.0), moving(false),
        currentMoveInvalid(false), numRobotMovePoints(1),
        /*Original Julia code does not have an
         * initial value for numLocalMovePoints*/
        numLocalMovePoints(1), robotEdgeUsed(false),
        distAlongRobotEdge(0.0), timeAlongRobotEdge(0.0)
    {
        robotLocalPath.resize(maxPathNodes,4);
        robotMovePath.resize(maxPathNodes,4);
    }

} RobotData;

#endif // DRRT_DATA_STRUCTURES_H
