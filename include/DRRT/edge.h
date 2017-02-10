#ifndef EDGE_H
#define EDGE_H

#include <DRRT/kdtreenode.h> // PI defined here

// Infinity value for distance
#define INF 1000000000000
#define MAXPATHNODES 100000
#define DELTA 10

class CSpace;
class KDTree;

// Edge for a Dubin's state space
class Edge{
public:
    std::shared_ptr<KDTreeNode> startNode;
    std::shared_ptr<KDTreeNode> endNode;
    // This field should be populated by Edge::calculateTrajectory()
    double dist;    // the distance between startNode and endNode
                    // i.e. how far the robot must travel through
                    // the *configuration* space to get from startNode
                    // endNode. This is the distance used to calc
                    // RRTTreeCost and RRTLMC

    double distOriginal; // saves the original value of dist, so we don't
                         // need to recalculate if this edge is removed and
                         // then added again

    // pointer to this edge's location in startNode
    std::shared_ptr<JListNode> listItemInStartNode;
    // pointer to this edge's location in endNode
    std::shared_ptr<JListNode> listItemInEndNode;

    double Wdist;   // this contains the distance that the robot must travel
                    // through the *workspace* along the edge (so far only
                    // used for time based C-Spaces)

    std::string edgeType;   // one of the following types:
                            // lsl, rsr, lsr, rsl, lrl, rlr

    Eigen::MatrixXd trajectory; // stores a discritized version
                                // of the Dubin's path
                                // [x y] or [x y time] (these are rows)
                                // depending on if time is being used

    double velocity; // the velocity that this robot travels along this edge
                     // only used if time is part of the state space

    // Constructor
    Edge() : dist(-1) { trajectory.resize(MAXPATHNODES,4); }
    Edge(std::shared_ptr<KDTreeNode> s, std::shared_ptr<KDTreeNode> e)
        : startNode(s), endNode(e)
    {trajectory.resize(MAXPATHNODES,4);}


    /////////////////////// Critical Functions ///////////////////////
    // Critical functions that must be modified vs the CSpace and
    // Workspace being used ('D' is for "Dubin's")

    // Returns the distance between two points, assuming they are
    // in the C-Space
    // Should obey the triangle inequality
    // Use the following version of dist for Dubins space [X Y Theta Time]
    //virtual double Edist(Eigen::VectorXd x, Eigen::VectorXd y)=0;

    /* This returns the distance that is used internally in the
     * kd-tree (see notes there). The kd-implementation handles
     * wrapping dimensions (e.g. a circle or taurus) by running
     * multiple searches, 1 for each wrapped identity of the query
     * point. So this is the distance function that is the -non-wrapping-
     * version of what the actual distance function between points
     * in the KD space is
     * Use the follwing version of KDist for Dubins space (with
     * and without time, since time == 0 in the latter case, but
     * still exists as a dimension in points
     */
    //static double EKDdist(Eigen::VectorXd x, Eigen::VectorXd y);

    /* Returns the workspace distance between two points, this should
     * obey the triangle inequality. e.g. in the current version of this
     * code, it is assumed that the workspace is the first two dimensionss
     * of the CSpace. It is used for calculating the "real world" distance
     * that the robot covers (e.ge in a particular amount of time when
     * determining if a move is possible given robot max speed
     * Use the following version of Wdist for Euclidian space and Dubin's
     * space
     */
    //virtual double EWdist(Eigen::VectorXd x, Eigen::VectorXd y)=0;

    // Moves newPoint toward closestPoint such that each robot is no further
    // than delta. Points reperesent the cartesian product of R robots
    // Use the following version of Edge::saturate for Dubin's space
//    virtual void saturate(std::shared_ptr<Eigen::Vector4d> newPoint,
//                         Eigen::Vector4d closestPoint,
//                         double delta )=0;


    /////////////////////// Edge Functions ///////////////////////

    // Allocates a new edge
    // This must be implemented by all edge types!!
    static std::shared_ptr<Edge> newEdge(std::shared_ptr<KDTreeNode> startNode,
                                         std::shared_ptr<KDTreeNode> endNode);

    // Saturate moving nP within delta of cP
    // This must be implemented by all edge types!!
    static void saturate(std::shared_ptr<Eigen::Vector4d> nP,
                         Eigen::Vector4d cP,
                         double delta,
                         double dist);

    // Returns true if the dynamics of the robot in the space will
    // allow a robot to follow the edge
    // Dubin's edge version
    virtual bool validMove(std::shared_ptr<CSpace> S)=0;

    /* Returns the pose of a robot that is located dist along the edge
     * Note that 'dist' and 'far' are with respect to whatever type of
     * distance is stored in Edge->dist
     * Dubin's edge version (COULD BE MADE MORE EFFICIENT)
     */
    virtual Eigen::VectorXd poseAtDistAlongEdge(double distAlongEdge)=0;

    // Returns the pose of a robot that is located time along the edge
    // Dubin's edge version (could be made more efficient)
    virtual Eigen::VectorXd poseAtTimeAlongEdge(double timeAlongEdge)=0;

    /* Dubin's version, figures out which one of the 6 possibilities is the
     * shortest (ignoring obstacles) subject to the robot's (constant)
     * velocity and minimum turning radius. At the very least this function
     * should populate the dist field of edge
     */
    virtual void calculateTrajectory(std::shared_ptr<CSpace> S,
                                     std::shared_ptr<KDTree> Tree)=0;

    // This calculates a trajectory of what the robot is supposed to do
    // when it is hovering "in place". Dubin's edge version
    virtual void calculateHoverTrajectory(std::shared_ptr<CSpace> S)=0;


    ///////////////////// Collision Checking Functions /////////////////////
    // These are collision checking functions that depend on edge type
    // More general collision checking functions appear in drrt.h

    // Checks if the edge is in collision with a particular obstacle
    // Returns true if in collision
    // Dubin's edge version
    //virtual bool explicitEdgeCheck( std::shared_ptr<CSpace> S, std::shared_ptr<Edge> edge, Obstacle* obstacle );

};

#endif // EDGE_H
