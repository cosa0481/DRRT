#ifndef EDGE_H
#define EDGE_H

#include <DRRT/kdtreenode.h>

// Edge for a Dubin's state space
class Edge{
public:
    KDTreeNode* startNode;
    KDTreeNode* endNode;
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
};

#endif // EDGE_H
