/* heap.h
 * Corin Sandford
 * Fall 2016
 */

#ifndef HEAP_H
#define HEAP_H

#include <vector>
#include <iostream>
#include <DRRT/kdtreenode.h>

// A binary heap data structure
class BinaryHeap {
public:
    std::vector<KDTreeNode> H;  // stores the things that are in the heap
    int indexOfLast;            // the index of the last node in the heap array
    int parentOfLast;           // stores the index of the parent of the last node
    bool useDefault;            // flag for using key or keyQ e.g. (DRRT.jl>>3289)

    // Functions for interacting with marks and indices
    // Returns the key value of the node
    double keyD( KDTreeNode node ) { return node.dist; }
    double keyQ( KDTreeNode node ) { return std::min( node.rrtTreeCost, node.rrtLMC ); }
    double key( KDTreeNode node ) { return ( useDefault ? keyD(node) : keyQ(node) ); }
    // Default less than function
    bool lessD( KDTreeNode a, KDTreeNode b ) { return (a.dist < b.dist); }
    bool lessQ( KDTreeNode a, KDTreeNode b ) { return ((keyQ(a) < keyQ(b)) || (keyQ(a) == keyQ(b) && a.isMoveGoal)); }
    bool lessThan( KDTreeNode a, KDTreeNode b ) { return ( useDefault ? lessD(a,b) : lessQ(a,b) ); }
    // Default greater than function DATA
    bool greaterD( KDTreeNode a, KDTreeNode b ) { return (a.dist > b.dist); }
    bool greaterQ( KDTreeNode a, KDTreeNode b ) { return ((keyQ(a) > keyQ(b)) || (keyQ(a) == keyQ(b) && b.isMoveGoal)); }
    bool greaterThan( KDTreeNode a, KDTreeNode b ) { return ( useDefault ? greaterD(a,b) : greaterQ(a,b) ); }
    // Default heap marker function (marks when a node is in the heap)
    void markD( KDTreeNode* node ) { node->inHeap = true; }
    void markQ( KDTreeNode* node ) { node->inPriorityQueue = true; }
    void mark( KDTreeNode* node ) { return ( useDefault ? markD(node) : markQ(node) ); }
    // Default heap unmarker function (un marks when a node is removed)
    void unmarkD( KDTreeNode* node ) { node->inHeap = false; }
    void unmarkQ( KDTreeNode* node ) { node->inPriorityQueue = false; }
    void unmark( KDTreeNode* node ) { return ( useDefault ? unmarkD(node) : unmarkQ(node) ); }
    // Default heap check marker function (checks if node is marked)
    bool markedD( KDTreeNode node ) { return node.inHeap; }
    bool markedQ( KDTreeNode node ) { return node.inPriorityQueue; }
    bool marked( KDTreeNode node ) { return ( useDefault ? markedD(node) : markedQ(node) ); }
    // Sets the heap index to a value
    void setIndexD( KDTreeNode* node, int value ) { node->heapIndex = value; }
    void setIndexQ( KDTreeNode* node, int value ) { node->priorityQueueIndex = value; }
    void setIndex( KDTreeNode* node, int value ) { return ( useDefault ? setIndexD(node,value) : setIndexQ(node,value) ); }
    // Set the heap index to the unused value (-1)
    void unsetIndexD( KDTreeNode* node ) { node->heapIndex = -1; }
    void unsetIndexQ( KDTreeNode* node ) { node->priorityQueueIndex = -1; }
    void unsetIndex( KDTreeNode* node ) { return ( useDefault ? unsetIndexD(node) : unsetIndexQ(node) ); }
    // Returns the heap index
    int getIndexD( KDTreeNode node ) { return node.heapIndex; }
    int getIndexQ( KDTreeNode node ) { return node.priorityQueueIndex; }
    int getIndex( KDTreeNode node ) { return ( useDefault ? getIndexD(node) : getIndexQ(node) ); }

    // Constructor
    BinaryHeap( bool useD ) : indexOfLast(0), parentOfLast(-1), useDefault(useD)
    {
        H.push_back( KDTreeNode() );
    }

    // Returns the heap in a vector array
    std::vector<KDTreeNode>* getHeap() { return &H; }
    // Returns the index of the last node in the heap array
    int* getIndexOfLast() { return &indexOfLast; }
    // Returns the index of the parent of the last index in the heap array
    int* getParentOfLast() { return &parentOfLast; }

    /* Heap operation functions for returning the smallest thing */

    // Compares a node with its parent and switches them if the parent's
    // cost is more than the node's cost. Repeats if a switch happens
    bool bubbleUp( int n );

    // Compares a node n with its childeren, and switches them if a child's
    // cost is less than the node's cost. Repeats if a switch happens
    bool bubbleDown( int n );

    // Add a node to the heap
    bool addToHeap( KDTreeNode* node );

    // Returns the node thas is on the top of the heap
    // If heap is empty, return node with data = -1
    KDTreeNode* topHeap();

    // Removes the top valued node from the heap and returns it
    // If heap is empty, return node with data = -1
    KDTreeNode* popHeap();

    // Removes the node from the heap, assuming that it is in the heap
    bool removeFromHeap( KDTreeNode* node );

    // Updates a node that is already in the heap
    bool updateHeap( KDTreeNode node );

    // Returns true if heap is good, false if bad
    bool checkHeap();

    // Removes all items from the heap and returns an array containing
    // the heap items (unsorted)
    std::vector<KDTreeNode> cleanHeap();

    /* Heap operation functions for returning the largest thing */

    // Compares a node n with its parent, and switches them if
    // the parent's cost is less than the node's cost. Repeats
    // if a switch happens
    bool bubbleUpB( int n );

    // Compares an node n with its children, and switches them if
    // a child's cost is more than the node's cost. Repeats if
    // a switch happens
    bool bubbleDownB( int n );

    // Add node to the heap
    bool addToHeapB( KDTreeNode* node );

    // Returns the node that is on top of the heap
    KDTreeNode* topHeapB() { return topHeap(); }

    // Removes the top valued node from the heap and returns it
    KDTreeNode* popHeapB();

    // Removes the node from the heap, assuming it is in the heap
    bool removeFromHeapB( KDTreeNode* node );

    // Updates a node that is already in the heap
    bool updateHeapB( KDTreeNode node );

    // Returns 1 if heap is good, 0 if bad
    bool checkHeapB();

    // Removes all items from the heap and returns an array containing
    // the heap items (unsorted)
    std::vector<KDTreeNode> cleanHeapB() { return cleanHeap(); }
};

#endif // HEAP_H
