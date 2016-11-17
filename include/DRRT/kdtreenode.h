#ifndef KDTREENODE_H
#define KDTREENODE_H

#include <vector>
#include <DRRT/jlist.h>
#include <DRRT/edge.h> // Eigen included here

/* Node that can be used in the KDTree, where T is the type of
 * data used to measure distance along each dimension. Other nodes
 * can also be used as long as they have these fields that are
 * initialized as follows by a default constructor and the parent
 * and children types are the same as the node's type itself.
 */
class KDTreeNode{
public:
    // Data used for KD Tree
    bool kdInTree;                  // set to true if this node is in the kd-tree
    bool kdParentExist;             // set to true if parent in the tree is used
    bool kdChildLExist;             // set to true if left child in the tree is used
    bool kdChildRExist;             // set to true if right child in the tree is used

    // Data used for heap in KNN-search
    int heapIndex;                  // named such to allow the use of default heap functions
    bool inHeap;                    // ditto
    double dist;                    // ditto, this will hold the distance

    // More data used for KD Tree
    Eigen::VectorXd position;       // a d x 1 array where d is the dimensions of the space (5)
    int kdSplit;                    // the dimension used for splitting at this node
    KDTreeNode* kdParent;           // parent in the tree
    KDTreeNode* kdChildL;           // left child in the tree
    KDTreeNode* kdChildR;           // right child in the tree

    // RRT
    bool rrtParentUsed;             // flag for if this node has a parent
    Edge* rrtParentEdge;            // edge to the node that is this node's parent

    // RRT*
    double rrtTreeCost;             // the cost to get to the root through the tree

    // RRT#
    JList* rrtNeighborsOut;         // edges in the graph that can be reached
                                    // from this node
    JList* rrtNeighborsIn;          // edges in the graph that reach this node

    int priorityQueueIndex;         // index in the queue
    bool inPriorityQueue;           // flag for in the queue

    double rrtLMC;                  // locally minimum cost (1-step look ahead)
    double rrtH;                    // the heuristic estimate of the distance to the goal !!!!!
    Edge* tempEdge;                 // this is a temporary storage location to avoid
                                    // calculating the same trajectory multiple times

    // RRTx (his idea)
    JList* SuccessorList;           // edges to nodes that use this node as their parent
    JList* InitialNeighborListOut;  // edges to nodes in the original ball that can be reached
                                    // from this node
    JList* InitialNeighborListIn;   // edges to nodes in the original ball that can reach this node

    bool inOSQueue;                 // flag for in the OS queue
    bool isMoveGoal;                // true if this is move goal (robot pose)

    JListNode* successorListItemInParent;   // pointer to the list node in the parent's successor list
                                            // that holds parent's edge to this node

    // Constructors
    KDTreeNode() : kdInTree(false), kdParentExist(false), kdChildLExist(false),
                   kdChildRExist(false), heapIndex(-1), inHeap(false), dist(-1),
                   rrtParentUsed(false), rrtNeighborsOut(new JList()), rrtNeighborsIn(new JList()),
                   priorityQueueIndex(-1), inPriorityQueue(false), SuccessorList(new JList()),
                   InitialNeighborListOut(new JList()), InitialNeighborListIn(new JList()),
                   inOSQueue(false), isMoveGoal(false)
    {}
    KDTreeNode(float d) :  kdInTree(false), kdParentExist(false), kdChildLExist(false),
        kdChildRExist(false), heapIndex(-1), inHeap(false), dist(d),
        rrtParentUsed(false), rrtNeighborsOut(new JList()), rrtNeighborsIn(new JList()),
        priorityQueueIndex(-1), inPriorityQueue(false), SuccessorList(new JList()),
        InitialNeighborListOut(new JList()), InitialNeighborListIn(new JList()),
        inOSQueue(false), isMoveGoal(false)
    {}
    KDTreeNode(float d, Eigen::VectorXd pos) :  kdInTree(false), kdParentExist(false),
        kdChildLExist(false), kdChildRExist(false), heapIndex(-1), inHeap(false), dist(d),
        position(pos), rrtParentUsed(false), rrtNeighborsOut(new JList()),
        rrtNeighborsIn(new JList()), priorityQueueIndex(-1), inPriorityQueue(false),
        SuccessorList(new JList()), InitialNeighborListOut(new JList()),
        InitialNeighborListIn(new JList()), inOSQueue(false), isMoveGoal(false)
    {}
    KDTreeNode(Eigen::VectorXd pos) : kdInTree(false), kdParentExist(false),
        kdChildLExist(false), kdChildRExist(false), heapIndex(-1), inHeap(false), dist(-1),
        position(pos), rrtParentUsed(false), rrtNeighborsOut(new JList()),
        rrtNeighborsIn(new JList()), priorityQueueIndex(-1), inPriorityQueue(false),
        SuccessorList(new JList()), InitialNeighborListOut(new JList()),
        InitialNeighborListIn(new JList()), inOSQueue(false), isMoveGoal(false)
    {}
};

#endif // KDTREENODE_H
