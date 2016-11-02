#ifndef EDGE_H
#define EDGE_H

#include <vector>

class KDTreeNode;

// Edge for a Dubin's state space
class Edge{
public:
    KDTreeNode* startNode;
    KDTreeNode* endNode;
    // This field should be populated by calculateTrajectory(CSpace *S)
    float dist;     // the distance between startNode and endNode
                    // i.e. how far the robot must travel through
                    // the *configuration* space to get from startNode
                    // endNode. This is the distance used to calc
                    // RRTTreeCost and RRTLMC

    float distOriginal; // saves the original value of dist, so we don't
                        // need to recalculate if this edge is removed and
                        // then added again

    //JListNode<Edge>* listItemInStartNode;   // pointer to this edge's location
                                            // in startNode

    //JListNode<Edge>* listItemInEndNode;     // pointer to this edge's location
                                            // in endNode

    float Wdist;    // this contains the distance that the robot must travel
                    // through the *workspace* along the edge (so far only
                    // used for time based C-Spaces)

    std::string edgeType;   // one of the following types depending on the edge:
                            //      lsl, rsr, lsr, rsl, lrl, rlr

    std::vector<std::vector<float>> trajectory; // stores a discritized version
                                                // of the Dubin's path
                                                // [x y] or [x y time] depending
                                                // on if time is being used

    float velocity; // the velocity that this robot travels along this edge
                    // only used if time is part of the state space

    // Constructor
    Edge() {}
    Edge( KDTreeNode* s, KDTreeNode* e ) : startNode(s), endNode(e)
    {}

    // Functions
    // Critical functions that must be modified vs the CSpace and
    // Workspace being used ('D' is for "Dubin's")

    // Returns the distance between two points, assuming they are in the C-Space
    // Should obey the triangle inequality
    // Use the following version of dist for Dubins space [X Y Theta Time]
    float Ddist( std::vector<float> x, std::vector<float> y );

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
    float DKDdist( std::vector<float> x, std::vector<float> y );

    /* Returns the workspace distance between two points, this should
     * obey the triangle inequality. e.g. in the current version of this
     * code, it is assumed that the workspace is the first two dimensionss
     * of the CSpace. It is used for calculating the "real world" distance
     * that the robot covers (e.ge in a particular amount of time when
     * determining if a move is possible given robot max speed
     * Use the following version of Wdist for Euclidian space and Dubin's
     * space
     */
    float DWdist( std::vector<float> x, std::vector<float> y );
};

#endif // EDGE_H
