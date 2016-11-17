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

    // Functions for interacting with marks and indices
    // Returns the key value of the node
    double key( KDTreeNode t ) { return t.dist; }
    // Default less than function
    bool lessThan( KDTreeNode a, KDTreeNode b ) { return (a.dist < b.dist); }
    // Default greater than function DATA
    bool greaterThan( KDTreeNode a, KDTreeNode b ) { return (a.dist > b.dist); }
    // Default heap marker function (marks when a node is in the heap)
    void mark( KDTreeNode* t ) { t->inHeap = true; }
    // Default heap unmarker function (un marks when a node is removed)
    void unmark( KDTreeNode* t ) { t->inHeap = false; }
    // Default heap check marker function (checks if node is marked)
    bool marked( KDTreeNode t ) { return t.inHeap; }
    // Sets the heap index to a value
    void setIndex( KDTreeNode* t, int value ) { t->heapIndex = value; }
    // Set the heap index to the unused value (-1)
    void unsetIndex( KDTreeNode* t ) { t->heapIndex = -1; }
    // Returns the heap index
    int getIndex( KDTreeNode t ) { return t.heapIndex; }

    // Constructor
    BinaryHeap() : indexOfLast(0), parentOfLast(-1)
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
    KDTreeNode popHeap();

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
    KDTreeNode popHeapB();

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
