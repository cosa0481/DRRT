/* edge.h
 * Corin Sandford
 * Edge class is abstract
 * Make sure to handle case where ConfigSpace has time and where ConfigSpace does not
 * have time
 */

#ifndef EDGE_H
#define EDGE_H

#include <DRRT/kdtreenode.h> // PI defined here

class ConfigSpace;
class KDTree;

// Edge for a Dubin's state space
class Edge: public std::enable_shared_from_this<Edge> {
public:
    std::shared_ptr<ConfigSpace> cspace_;
    std::shared_ptr<KDTree> tree_;   // k-d tree to which this edge belongs

    std::shared_ptr<KDTreeNode> start_node_;
    std::shared_ptr<KDTreeNode> end_node_;

    // This field should be populated by Edge::CalculateTrajectory()
    double dist_;    // the distance between startNode and endNode
                    // i.e. how far the robot must travel through
                    // the *configuration* space to get from startNode
                    // endNode. This is the distance used to calc
                    // RRTTreeCost and RRTLMC

    double dist_original_; // saves the original value of dist, so we don't
                           // need to recalculate if this edge is removed and
                           // then added again

    // pointer to this edge's location in startNode
    std::shared_ptr<JListNode> list_item_in_start_node_;
    // pointer to this edge's location in endNode
    std::shared_ptr<JListNode> list_item_in_end_node_;

    double w_dist_; // this contains the distance that the robot must travel
                    // through the *workspace* along the edge (so far only
                    // used for time based C-Spaces)

    std::string edge_type_; // one of the following types:
                            // lsl, rsr, lsr, rsl, lrl, rlr

    Eigen::MatrixXd trajectory_; // stores a discritized version
                                 // of the Dubin's path
                                 // [x y] or [x y time] (these are rows)
                                 // depending on if time is being used

    double velocity_; // the velocity that this robot travels along this edge
                      // only used if time is part of the state space

    // Constructor
    Edge() : dist_(-1) { trajectory_ = Eigen::MatrixXd::Zero(MAXPATHNODES,3); }
    Edge(std::shared_ptr<ConfigSpace> &CS,
         std::shared_ptr<KDTree> &T,
         std::shared_ptr<KDTreeNode> &s,
         std::shared_ptr<KDTreeNode> &e)
        : cspace_(CS), tree_(T), start_node_(s), end_node_(e)
    {
        trajectory_ = Eigen::MatrixXd::Zero(MAXPATHNODES,3);
    }

    /////////////////////// Edge Functions ///////////////////////

    // Allocates a new edge
    // This must be implemented by all edge types!!
    static std::shared_ptr<Edge> NewEdge(std::shared_ptr<ConfigSpace> C,
                                         std::shared_ptr<KDTree> Tree,
                                         std::shared_ptr<KDTreeNode> &start_node,
                                         std::shared_ptr<KDTreeNode> &end_node);

    // Saturate moving nP within delta of cP
    // This must be implemented by all edge types!!
    static void Saturate(Eigen::VectorXd &nP,
                         Eigen::VectorXd cP,
                         double delta,
                         double distance);

    std::shared_ptr<Edge> GetPointer() { return shared_from_this(); }

    // Returns true if the dynamics of the robot in the space will
    // allow a robot to follow the edge
    // Dubin's edge version
    virtual bool ValidMove()=0;

    /* Returns the pose of a robot that is located dist along the edge
     * Note that 'dist' and 'far' are with respect to whatever type of
     * distance is stored in Edge->dist_
     * Dubin's edge version (COULD BE MADE MORE EFFICIENT)
     */
    virtual Eigen::VectorXd PoseAtDistAlongEdge(double dist_along_edge)=0;

    // Returns the pose of a robot that is located time along the edge
    // Dubin's edge version (could be made more efficient)
    virtual Eigen::VectorXd PoseAtTimeAlongEdge(double time_along_edge)=0;

    /* Dubin's version, figures out which one of the 6 possibilities is the
     * shortest (ignoring obstacles) subject to the robot's (constant)
     * velocity and minimum turning radius. At the very least this function
     * should populate the dist field of edge
     */
    virtual void CalculateTrajectory()=0;

    // This calculates a trajectory_ of what the robot is supposed to do
    // when it is hovering "in place". Dubin's edge version
    virtual void CalculateHoverTrajectory()=0;


    ///////////////////// Collision Checking Functions /////////////////////
    // These are collision checking functions that depend on edge type
    // More general collision checking functions appear in drrt.h

    // Checks if the edge is in collision with a particular obstacle
    // Returns true if in collision
    // Dubin's edge version
    virtual bool ExplicitEdgeCheck(std::shared_ptr<Obstacle> obstacle)=0;

};

#endif // EDGE_H
